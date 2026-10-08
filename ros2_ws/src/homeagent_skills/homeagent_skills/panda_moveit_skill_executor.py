"""Panda-specific Safety-approved MoveIt2 skill executor.

Only stow_arm and look_at are supported. The candidate cannot inject Panda
joint targets. These are fixed named skills at the adapter boundary.

Not a pick/grasp adapter; unsupported actions are left to other adapters.
"""
import json
import math
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from homeagent_interfaces.msg import SafetyDecision, SkillResult
from .panda_targets import PANDA_JOINT_NAMES, resolve_panda_approved_action


class PandaMoveItSkillExecutor(Node):
    def __init__(self):
        super().__init__("homeagent_panda_moveit_skill_executor")
        self.declare_parameter("action_server_timeout_sec", 3.0)
        self.declare_parameter("postcondition_tolerance_rad", 0.05)
        self._timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self._tolerance = float(
            self.get_parameter("postcondition_tolerance_rad").value
        )
        self._client = ActionClient(self, MoveGroup, "/move_action")
        self._result_pub = self.create_publisher(
            SkillResult, "/homeagent/skill_result", 10
        )
        self._approved_sub = self.create_subscription(
            SafetyDecision, "/homeagent/action_approved", self._on_approved, 10
        )
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joints, 20
        )
        self._current = {}
        self._active = {}
        self._timer = self.create_timer(0.15, self._check_postconditions)
        self.get_logger().info(
            "Panda MoveIt skill ready: ONLY stow_arm and look_at via Safety"
        )

    def _on_joints(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in PANDA_JOINT_NAMES:
                self._current[name] = float(position)

    def _publish(self, decision, *, success, code, payload):
        result = SkillResult()
        result.request_id = decision.request_id
        result.action = decision.action
        result.success = bool(success)
        result.code = code
        result.result_json = json.dumps(payload, ensure_ascii=False)
        self._result_pub.publish(result)
        self.get_logger().info(
            f"SKILL_RESULT request_id={decision.request_id} "
            f"action={decision.action} success={success} code={code}"
        )

    def _on_approved(self, decision):
        if decision.allowed is not True:
            return
        if decision.action not in {"stow_arm", "look_at"}:
            return

        try:
            proposal = json.loads(decision.proposal_json or "{}")
            resolved = resolve_panda_approved_action(decision.action, proposal)
            assert resolved is not None
            target_name, joint_target = resolved
        except (json.JSONDecodeError, TypeError, ValueError, AssertionError) as exc:
            self._publish(
                decision,
                success=False,
                code="INVALID_PANDA_APPROVED_ACTION",
                payload={"error": str(exc)},
            )
            return

        if self._active:
            self._publish(
                decision, success=False, code="PANDA_MOVEIT_BUSY",
                payload={"active_request_ids": list(self._active)},
            )
            return

        if not self._client.wait_for_server(timeout_sec=self._timeout):
            self._publish(
                decision, success=False, code="PANDA_MOVEIT_UNAVAILABLE",
                payload={"target_name": target_name},
            )
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 6.0
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = f"homeagent_panda_{target_name}"
        for name, value in zip(PANDA_JOINT_NAMES, joint_target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.015
            joint.tolerance_below = 0.015
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._active[decision.request_id] = {
            "decision": decision,
            "target_name": target_name,
            "joint_target": joint_target,
            "stage": "await_goal_response",
            "started_at": time.monotonic(),
        }
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, rid=decision.request_id:
            self._on_goal_response(rid, done)
        )
        self.get_logger().info(
            f"PANDA_MOVEIT_SEND request_id={decision.request_id} "
            f"action={decision.action} target={target_name} "
            "source=trusted_named_profile"
        )

    def _on_goal_response(self, rid, future):
        state = self._active.get(rid)
        if state is None:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self._fail(rid, "PANDA_GOAL_RESPONSE_FAILED", str(exc))
            return
        if handle is None or not handle.accepted:
            self._fail(rid, "PANDA_GOAL_REJECTED", "MoveGroup rejected goal")
            return
        state["stage"] = "executing"
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done, key=rid: self._on_move_result(key, done)
        )

    def _on_move_result(self, rid, future):
        state = self._active.get(rid)
        if state is None:
            return
        try:
            wrapped = future.result()
            response = wrapped.result
            status = int(wrapped.status)
            code = int(response.error_code.val)
            points = len(response.planned_trajectory.joint_trajectory.points)
        except Exception as exc:
            self._fail(rid, "PANDA_MOVE_RESULT_FAILED", str(exc))
            return
        state["status"] = status
        state["error_code"] = code
        state["planned_points"] = points
        if status != 4 or code != MoveItErrorCodes.SUCCESS:
            self._fail(
                rid, "PANDA_MOVEIT_FAILED",
                f"status={status} error_code={code}",
            )
            return
        state["stage"] = "verify"
        state["verify_deadline"] = time.monotonic() + 3.0

    def _check_postconditions(self):
        for rid, state in list(self._active.items()):
            if state.get("stage") != "verify":
                continue
            values = [self._current.get(n) for n in PANDA_JOINT_NAMES]
            if not all(v is not None and math.isfinite(v) for v in values):
                if time.monotonic() < state["verify_deadline"]:
                    continue
                self._fail(rid, "PANDA_JOINT_STATE_UNAVAILABLE", "joint state missing")
                continue

            error = max(
                abs(v - target)
                for v, target in zip(values, state["joint_target"])
            )
            if error > self._tolerance:
                if time.monotonic() < state["verify_deadline"]:
                    continue
                self._fail(
                    rid, "PANDA_POSTCONDITION_FAILED",
                    f"joint error={error:.5f} > tolerance={self._tolerance:.5f}",
                )
                continue

            self._active.pop(rid, None)
            self._publish(
                state["decision"],
                success=True,
                code="MOVEIT_SUCCEEDED",
                payload={
                    "target_name": state["target_name"],
                    "source": "trusted_named_profile",
                    "joint_names": list(PANDA_JOINT_NAMES),
                    "joint_target": list(state["joint_target"]),
                    "observed_joint_positions": values,
                    "max_abs_joint_error_rad": error,
                    "planned_points": state["planned_points"],
                    "status": state["status"],
                    "moveit_error_code": state["error_code"],
                    "postcondition_verified": True,
                },
            )

    def _fail(self, rid, code, reason):
        state = self._active.pop(rid, None)
        if state is not None:
            self._publish(
                state["decision"],
                success=False,
                code=code,
                payload={"target_name": state["target_name"], "reason": reason},
            )


def main(args=None):
    rclpy.init(args=args)
    node = PandaMoveItSkillExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
