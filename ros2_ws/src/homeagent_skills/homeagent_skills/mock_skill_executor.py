import json

import rclpy
from rclpy.node import Node

from homeagent_interfaces.msg import SafetyDecision, SkillResult


SUPPORTED_ACTIONS = {
    "navigate",
    "observe",
    "speak",
    "pick",
    "place",
    "handover",
    "look_at",
    "stop",
}


class MockSkillExecutor(Node):
    """Safe-side deterministic executor, replaced later by Nav2/MoveIt adapters."""

    def __init__(self) -> None:
        super().__init__("homeagent_mock_skill_executor")
        self._approved_sub = self.create_subscription(
            SafetyDecision,
            "/homeagent/action_approved",
            self._on_approved,
            10,
        )
        self._result_pub = self.create_publisher(
            SkillResult, "/homeagent/skill_result", 10
        )
        self.get_logger().info("HomeAgent mock skill executor ready")

    def _on_approved(self, decision: SafetyDecision) -> None:
        if not decision.allowed:
            self.get_logger().error(
                f"Fail-closed: received non-approved decision {decision.request_id}"
            )
            return

        try:
            proposal = json.loads(decision.proposal_json or "{}")
        except json.JSONDecodeError as exc:
            self._publish_result(
                decision,
                success=False,
                code="INVALID_PROPOSAL_JSON",
                result={"error": str(exc)},
            )
            return

        action = proposal.get("action", decision.action)
        params = proposal.get("params") or {}
        if action not in SUPPORTED_ACTIONS:
            self._publish_result(
                decision,
                success=False,
                code="UNSUPPORTED_SKILL",
                result={"action": action},
            )
            return

        if action == "navigate":
            result = {
                "adapter": "mock",
                "target": params.get("target"),
                "status": "reached",
            }
            code = "MOCK_NAVIGATION_COMPLETE"
        elif action == "observe":
            result = {
                "adapter": "mock",
                "object": params.get("object"),
                "detected": True,
                "zone": "living_room",
                "pose": {"x": 1.2, "y": -0.4},
                "confidence": 0.82,
            }
            code = "MOCK_OBSERVATION_COMPLETE"
        elif action == "stop":
            result = {"adapter": "mock", "status": "stopped"}
            code = "MOCK_STOP_COMPLETE"
        else:
            result = {"adapter": "mock", "action": action, "params": params}
            code = "MOCK_SKILL_COMPLETE"

        self._publish_result(
            decision,
            success=True,
            code=code,
            result=result,
        )

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
        level = self.get_logger().info if success else self.get_logger().error
        level(
            f"SKILL_RESULT request_id={msg.request_id} action={msg.action} "
            f"success={msg.success} code={msg.code}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockSkillExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
