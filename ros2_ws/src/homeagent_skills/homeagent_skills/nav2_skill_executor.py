import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from homeagent_interfaces.msg import SafetyDecision, SkillResult

from .navigation_target import trusted_navigation_pose
from .panda_targets import PANDA_JOINT_NAMES
from .panda_stow_guard import validate_panda_stow


class Nav2SkillExecutor(Node):
    """Execute Safety-approved navigation and verify the physical postcondition."""

    def __init__(self) -> None:
        super().__init__("homeagent_nav2_skill_executor")

        self.declare_parameter("action_server_timeout_sec", 3.0)
        self.declare_parameter("tf_preflight_timeout_sec", 5.0)
        self.declare_parameter("success_position_tolerance_m", 0.30)
        self.declare_parameter("max_false_success_retries", 1)
        self.declare_parameter("false_success_retry_delay_sec", 0.5)
        self.declare_parameter("require_panda_stow", False)
        self.declare_parameter("panda_stow_tolerance_rad", 0.07)
        self.declare_parameter("panda_state_max_age_sec", 2.0)

        self._timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self._tf_timeout = float(
            self.get_parameter("tf_preflight_timeout_sec").value
        )
        self._success_tolerance = float(
            self.get_parameter("success_position_tolerance_m").value
        )
        self._max_retries = int(
            self.get_parameter("max_false_success_retries").value
        )
        self._retry_delay = float(
            self.get_parameter("false_success_retry_delay_sec").value
        )
        self._require_panda_stow = bool(
            self.get_parameter("require_panda_stow").value
        )
        self._panda_tolerance = float(
            self.get_parameter("panda_stow_tolerance_rad").value
        )
        self._panda_state_max_age = float(
            self.get_parameter("panda_state_max_age_sec").value
        )
        self._panda_positions = {}
        self._panda_last_update = None
        self._panda_joint_names = set(PANDA_JOINT_NAMES)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 20
        )

        self._client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self._result_pub = self.create_publisher(
            SkillResult, "/homeagent/skill_result", 10
        )
        self.create_subscription(
            SafetyDecision,
            "/homeagent/action_approved",
            self._on_approved,
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )

        self._active = {}
        self._retry_timers = {}
        self._panda_guard_timer = self.create_timer(
            0.3, self._watch_panda_stow_while_navigating
        )

        self.get_logger().info(
            "HomeAgent Nav2 skill executor ready "
            f"postcondition_tolerance={self._success_tolerance:.3f}m "
            f"require_panda_stow={self._require_panda_stow}"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        saw_panda_joint = False
        for name, position in zip(msg.name, msg.position):
            if name in self._panda_joint_names:
                self._panda_positions[name] = float(position)
                saw_panda_joint = True
        if saw_panda_joint:
            self._panda_last_update = time.monotonic()

    def _panda_stow_preflight(self):
        if not self._require_panda_stow:
            return True, "NOT_REQUIRED", {}
        if self._panda_last_update is None:
            return False, "PANDA_ARM_STATE_UNAVAILABLE", {
                "reason": "no Panda joint state received"
            }
        age = time.monotonic() - self._panda_last_update
        if age > self._panda_state_max_age:
            return False, "PANDA_ARM_STATE_STALE", {
                "last_state_age_sec": age,
                "max_age_sec": self._panda_state_max_age,
            }
        allowed, code, evidence = validate_panda_stow(
            self._panda_positions,
            tolerance_rad=self._panda_tolerance,
        )
        return allowed, code, evidence

    def _watch_panda_stow_while_navigating(self) -> None:
        if not self._require_panda_stow:
            return
        for request_id, state in list(self._active.items()):
            handle = state.get("goal_handle")
            if handle is None:
                continue
            allowed, code, evidence = self._panda_stow_preflight()
            if allowed:
                continue
            self.get_logger().error(
                f"PANDA_NAV_CANCEL request_id={request_id} reason={code}"
            )
            # Send Nav2 cancellation to stop its controller when the arm has
            # moved out of the stowed envelope during base navigation.
            handle.cancel_goal_async()
            self._fail(request_id, code, evidence)

    def _on_approved(self, decision: SafetyDecision) -> None:
        if not decision.allowed:
            self.get_logger().error(
                f"Fail-closed: non-approved decision {decision.request_id}"
            )
            return

        if decision.action != "navigate":
            return

        try:
            proposal = json.loads(decision.proposal_json or "{}")
            target, pose, target_source = trusted_navigation_pose(proposal)
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self._publish_result(
                decision,
                success=False,
                code="INVALID_TRUSTED_NAVIGATION_CONTEXT",
                result={"error": str(exc)},
            )
            return

        stowed, stow_code, stow_evidence = self._panda_stow_preflight()
        if not stowed:
            self.get_logger().warning(
                f"PANDA_NAV_BLOCK request_id={decision.request_id} "
                f"reason={stow_code}"
            )
            self._publish_result(
                decision,
                success=False,
                code=stow_code,
                result=stow_evidence,
            )
            return

        if not self._client.wait_for_server(timeout_sec=self._timeout):
            self._publish_result(
                decision,
                success=False,
                code="NAV2_UNAVAILABLE",
                result={"target": target},
            )
            return

        self._active[decision.request_id] = {
            "decision": decision,
            "target": target,
            "pose": pose,
            "target_source": target_source,
            "attempt": 0,
            "preflight_deadline": time.monotonic() + self._tf_timeout,
            "tf_wait_logged": False,
        }
        self._try_send(decision.request_id)

    def _map_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
            )
        except TransformException:
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return [float(t.x), float(t.y), float(yaw)]

    def _try_send(self, request_id: str) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        # Bounded retry may occur after the Panda moved; re-check the
        # physical arm stow posture before every Nav2 goal transmission.
        stowed, stow_code, stow_evidence = self._panda_stow_preflight()
        if not stowed:
            self._fail(request_id, stow_code, stow_evidence)
            return

        current = self._map_pose()
        if current is None:
            if time.monotonic() < state["preflight_deadline"]:
                if not state["tf_wait_logged"]:
                    self.get_logger().warning(
                        f"NAV2_WAIT_TF request_id={request_id} "
                        "waiting for map->base_footprint"
                    )
                    state["tf_wait_logged"] = True
                self._schedule(
                    request_id,
                    0.2,
                    self._try_send,
                )
                return

            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="NAV2_TF_UNAVAILABLE",
                result={
                    "target": state["target"],
                    "goal_xyyaw": list(state["pose"]),
                    "target_source": state["target_source"],
                },
            )
            return

        state["attempt"] += 1
        state["tf_wait_logged"] = False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(*state["pose"])

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, rid=request_id:
            self._on_goal_response(rid, future)
        )

        self.get_logger().info(
            f"NAV2_SEND request_id={request_id} "
            f"attempt={state['attempt']} "
            f"target={state['target']} "
            f"pose={list(state['pose'])} "
            f"source={state['target_source']} "
            f"preflight_map_pose={current}"
        )

    def _on_goal_response(self, request_id, future) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail(
                request_id,
                "NAV2_SEND_FAILED",
                {"error": str(exc)},
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self._fail(
                request_id,
                "NAV2_GOAL_REJECTED",
                {},
            )
            return

        state["goal_handle"] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, rid=request_id:
            self._on_result(rid, result)
        )

    def _on_result(self, request_id, future) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self._fail(
                request_id,
                "NAV2_RESULT_FAILED",
                {"error": str(exc)},
            )
            return

        if status != 4:
            self._fail(
                request_id,
                "NAV2_FAILED",
                {"status": status},
            )
            return

        actual = self._map_pose()
        if actual is None:
            position_error = None
        else:
            position_error = math.hypot(
                actual[0] - state["pose"][0],
                actual[1] - state["pose"][1],
            )

        postcondition_ok = (
            position_error is not None
            and position_error <= self._success_tolerance
        )

        if not postcondition_ok:
            retries_used = state["attempt"] - 1
            if retries_used < self._max_retries:
                self.get_logger().warning(
                    f"NAV2_FALSE_SUCCESS_RETRY request_id={request_id} "
                    f"attempt={state['attempt']} "
                    f"position_error={position_error} "
                    f"actual_map_pose={actual}"
                )
                state["preflight_deadline"] = (
                    time.monotonic() + self._tf_timeout
                )
                self._schedule(
                    request_id,
                    self._retry_delay,
                    self._try_send,
                )
                return

            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="NAV2_POSTCONDITION_FAILED",
                result={
                    "target": state["target"],
                    "goal_xyyaw": list(state["pose"]),
                    "target_source": state["target_source"],
                    "status": status,
                    "actual_map_pose": actual,
                    "goal_position_error_m": position_error,
                    "attempts": state["attempt"],
                },
            )
            return

        self._active.pop(request_id, None)
        self._publish_result(
            state["decision"],
            success=True,
            code="NAV2_SUCCEEDED",
            result={
                "target": state["target"],
                "goal_xyyaw": list(state["pose"]),
                "target_source": state["target_source"],
                "status": status,
                "actual_map_pose": actual,
                "goal_position_error_m": position_error,
                "attempts": state["attempt"],
                "postcondition_verified": True,
            },
        )

    def _schedule(self, request_id: str, delay: float, callback) -> None:
        old = self._retry_timers.pop(request_id, None)
        if old is not None:
            old.cancel()

        def fire():
            timer = self._retry_timers.pop(request_id, None)
            if timer is not None:
                timer.cancel()
            callback(request_id)

        self._retry_timers[request_id] = self.create_timer(
            float(delay),
            fire,
        )

    def _fail(self, request_id: str, code: str, extra: dict) -> None:
        state = self._active.pop(request_id, None)
        timer = self._retry_timers.pop(request_id, None)
        if timer is not None:
            timer.cancel()
        if state is None:
            return

        result = {
            "target": state["target"],
            "goal_xyyaw": list(state["pose"]),
            "target_source": state["target_source"],
            "attempts": state["attempt"],
        }
        result.update(extra)
        self._publish_result(
            state["decision"],
            success=False,
            code=code,
            result=result,
        )

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_result(
        self,
        decision: SafetyDecision,
        *,
        success: bool,
        code: str,
        result: dict,
    ) -> None:
        msg = SkillResult()
        msg.request_id = decision.request_id
        msg.action = decision.action
        msg.success = bool(success)
        msg.code = code
        msg.result_json = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(msg)

        log = self.get_logger().info if success else self.get_logger().warning
        log(
            f"SKILL_RESULT request_id={msg.request_id} "
            f"action={msg.action} success={msg.success} code={msg.code}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2SkillExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
