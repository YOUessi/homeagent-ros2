import json
import os
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import ActionProposal, SafetyDecision, SkillResult

from .deepseek_client import DeepSeekClient, DeepSeekError
from .planner_core import PlannerOutputError


class DeepSeekPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_deepseek_planner")
        self.declare_parameter("api_base", "https://api.deepseek.com")
        self.declare_parameter("model", "deepseek-chat")
        self.declare_parameter("api_key_env", "DEEPSEEK_API_KEY")

        api_base = self.get_parameter("api_base").value
        model = self.get_parameter("model").value
        api_key_env = self.get_parameter("api_key_env").value
        api_key = os.getenv(api_key_env, "")

        self._client = None
        if api_key:
            self._client = DeepSeekClient(
                api_key=api_key,
                base_url=api_base,
                model=model,
            )
            self.get_logger().info(f"DeepSeek planner ready: model={model}")
        else:
            self.get_logger().warning(
                f"{api_key_env} is not set; planner will reject incoming commands locally"
            )

        self._model = model
        self._command_sub = self.create_subscription(
            String, "/homeagent/user_command", self._on_command, 10
        )
        self._proposal_pub = self.create_publisher(
            ActionProposal, "/homeagent/action_candidate", 10
        )
        self._planner_error_pub = self.create_publisher(
            String, "/homeagent/planner_error", 10
        )
        self.create_subscription(
            SafetyDecision,
            "/homeagent/action_rejected",
            self._on_rejected,
            10,
        )
        self.create_subscription(
            SkillResult,
            "/homeagent/skill_result",
            self._on_skill_result,
            10,
        )

    def _on_command(self, msg: String) -> None:
        if self._client is None:
            self._publish_error("NO_API_KEY", "DeepSeek API key is not configured")
            return

        try:
            plan = self._client.plan(msg.data.strip())
        except (DeepSeekError, PlannerOutputError) as exc:
            self._publish_error("PLANNER_FAILED", str(exc))
            return

        proposal = ActionProposal()
        proposal.request_id = str(uuid.uuid4())
        proposal.action = plan["action"]
        proposal.params_json = json.dumps(plan["params"], ensure_ascii=False)
        # Safety-critical context must come from trusted world state, never the LLM.
        proposal.context_json = "{}"
        proposal.source = json.dumps(
            {"type": "deepseek", "model": self._model}, ensure_ascii=False
        )
        self._proposal_pub.publish(proposal)
        self.get_logger().info(
            f"PROPOSE action={proposal.action} request_id={proposal.request_id}"
        )

    def _publish_error(self, code: str, message: str) -> None:
        out = String()
        out.data = json.dumps(
            {"code": code, "message": message},
            ensure_ascii=False,
        )
        self._planner_error_pub.publish(out)
        self.get_logger().error(f"{code}: {message}")

    def _on_rejected(self, msg: SafetyDecision) -> None:
        self.get_logger().warning(
            f"REJECTED request_id={msg.request_id} code={msg.code} reason={msg.reason}"
        )

    def _on_skill_result(self, msg: SkillResult) -> None:
        self.get_logger().info(
            f"RESULT request_id={msg.request_id} action={msg.action} "
            f"success={msg.success} code={msg.code}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeepSeekPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
