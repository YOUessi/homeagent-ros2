import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from homeagent_interfaces.msg import SafetyDecision, SkillResult

from .navigation_target import trusted_navigation_pose



class Nav2SkillExecutor(Node):
    """Execute only Safety-approved, memory-resolved navigation actions."""

    def __init__(self) -> None:
        super().__init__("homeagent_nav2_skill_executor")
        self.declare_parameter("action_server_timeout_sec", 3.0)
        self._timeout = float(self.get_parameter("action_server_timeout_sec").value)

        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._result_pub = self.create_publisher(
            SkillResult, "/homeagent/skill_result", 10
        )
        self._approved_sub = self.create_subscription(
            SafetyDecision,
            "/homeagent/action_approved",
            self._on_approved,
            10,
        )
        self._active = {}
        self.get_logger().info("HomeAgent Nav2 skill executor ready")

    def _on_approved(self, decision: SafetyDecision) -> None:
        if not decision.allowed:
            self.get_logger().error(
                f"Fail-closed: non-approved decision {decision.request_id}"
            )
            return

        if decision.action != "navigate":
            self._publish_result(
                decision,
                success=False,
                code="UNSUPPORTED_BY_NAV2_ADAPTER",
                result={"action": decision.action},
            )
            return

        try:
            proposal = json.loads(decision.proposal_json or "{}")
            target, pose = trusted_navigation_pose(proposal)
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self._publish_result(
                decision,
                success=False,
                code="INVALID_TRUSTED_NAVIGATION_CONTEXT",
                result={"error": str(exc)},
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

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(*pose)

        send_future = self._client.send_goal_async(goal)
        self._active[decision.request_id] = {
            "decision": decision,
            "target": target,
            "pose": pose,
        }
        send_future.add_done_callback(
            lambda future, request_id=decision.request_id:
            self._on_goal_response(request_id, future)
        )
        self.get_logger().info(
            f"NAV2_SEND request_id={decision.request_id} target={target} "
            f"pose={list(pose)} source=trusted_memory"
        )

    def _on_goal_response(self, request_id, future) -> None:
        state = self._active.get(request_id)
        if state is None:
            return
        decision = state["decision"]

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._active.pop(request_id, None)
            self._publish_result(
                decision,
                success=False,
                code="NAV2_SEND_FAILED",
                result={"error": str(exc), "target": state["target"]},
            )
            return

        if not goal_handle.accepted:
            self._active.pop(request_id, None)
            self._publish_result(
                decision,
                success=False,
                code="NAV2_GOAL_REJECTED",
                result={"target": state["target"]},
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, rid=request_id: self._on_result(rid, result)
        )

    def _on_result(self, request_id, future) -> None:
        state = self._active.pop(request_id, None)
        if state is None:
            return
        decision = state["decision"]

        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self._publish_result(
                decision,
                success=False,
                code="NAV2_RESULT_FAILED",
                result={"error": str(exc), "target": state["target"]},
            )
            return

        # action_msgs/GoalStatus: 4 == STATUS_SUCCEEDED
        success = status == 4
        self._publish_result(
            decision,
            success=success,
            code="NAV2_SUCCEEDED" if success else "NAV2_FAILED",
            result={
                "target": state["target"],
                "goal_xyyaw": list(state["pose"]),
                "target_source": "trusted_memory",
                "status": status,
            },
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
        msg.success = success
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
