import json
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import ActionProposal, SafetyDecision, SkillResult


class MockPlannerNode(Node):
    """Deterministic planner used before LLM access for repeatable integration tests."""

    def __init__(self) -> None:
        super().__init__("homeagent_mock_planner")
        self._command_sub = self.create_subscription(
            String, "/homeagent/user_command", self._on_command, 10
        )
        self._proposal_pub = self.create_publisher(
            ActionProposal, "/homeagent/action_proposal", 10
        )
        self._approved_sub = self.create_subscription(
            SafetyDecision, "/homeagent/action_approved", self._on_approved, 10
        )
        self._rejected_sub = self.create_subscription(
            SafetyDecision, "/homeagent/action_rejected", self._on_rejected, 10
        )
        self._skill_result_sub = self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._on_skill_result, 10
        )
        self.get_logger().info("HomeAgent mock planner ready")

    def _on_command(self, msg: String) -> None:
        proposal = self._plan(msg.data.strip())
        self._proposal_pub.publish(proposal)
        self.get_logger().info(
            f"PROPOSE action={proposal.action} request_id={proposal.request_id}"
        )

    def _make_proposal(self, text: str, action: str, params: dict, context: dict) -> ActionProposal:
        proposal = ActionProposal()
        proposal.request_id = str(uuid.uuid4())
        proposal.action = action
        proposal.params_json = json.dumps(params, ensure_ascii=False)
        proposal.context_json = json.dumps(context, ensure_ascii=False)
        proposal.source = json.dumps({"type": "mock", "text": text}, ensure_ascii=False)
        return proposal

    def _plan(self, text: str) -> ActionProposal:
        if "刀" in text and ("小孩" in text or "孩子" in text):
            return self._make_proposal(
                text,
                "handover",
                {
                    "object": "kitchen_knife",
                    "object_tags": ["sharp"],
                    "recipient": "child",
                },
                {"recipient_age": 10, "safety_context_trusted": True},
            )

        if "客厅" in text:
            return self._make_proposal(
                text,
                "navigate",
                {"target": "living_room"},
                {"forbidden_zones": ["utility_room"]},
            )

        if "水杯" in text or "杯子" in text:
            return self._make_proposal(
                text,
                "observe",
                {"object": "cup"},
                {},
            )

        return self._make_proposal(
            text,
            "speak",
            {"text": "当前 mock planner 还不能规划该任务。"},
            {},
        )

    def _on_approved(self, msg: SafetyDecision) -> None:
        self.get_logger().info(
            f"APPROVED request_id={msg.request_id} action={msg.action} code={msg.code}"
        )

    def _on_rejected(self, msg: SafetyDecision) -> None:
        self.get_logger().warning(
            f"REJECTED request_id={msg.request_id} action={msg.action} code={msg.code} reason={msg.reason}"
        )

    def _on_skill_result(self, msg: SkillResult) -> None:
        self.get_logger().info(
            f"RESULT request_id={msg.request_id} action={msg.action} "
            f"success={msg.success} code={msg.code} result={msg.result_json}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
