import json
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockPlannerNode(Node):
    """Deterministic planner used to validate Agent -> Safety integration before LLM access."""

    def __init__(self) -> None:
        super().__init__("homeagent_mock_planner")
        self._command_sub = self.create_subscription(
            String, "/homeagent/user_command", self._on_command, 10
        )
        self._proposal_pub = self.create_publisher(
            String, "/homeagent/action_proposal", 10
        )
        self._approved_sub = self.create_subscription(
            String, "/homeagent/action_approved", self._on_approved, 10
        )
        self._rejected_sub = self.create_subscription(
            String, "/homeagent/action_rejected", self._on_rejected, 10
        )
        self.get_logger().info("HomeAgent mock planner ready")

    def _on_command(self, msg: String) -> None:
        text = msg.data.strip()
        proposal = self._plan(text)
        out = String()
        out.data = json.dumps(proposal, ensure_ascii=False)
        self._proposal_pub.publish(out)
        self.get_logger().info(
            f"PROPOSE action={proposal['action']} request_id={proposal['request_id']}"
        )

    def _plan(self, text: str):
        request_id = str(uuid.uuid4())

        if "刀" in text and ("小孩" in text or "孩子" in text):
            return {
                "request_id": request_id,
                "action": "handover",
                "params": {
                    "object": "kitchen_knife",
                    "object_tags": ["sharp"],
                    "recipient": "child",
                },
                "context": {"recipient_age": 10},
                "source": {"type": "mock", "text": text},
            }

        if "客厅" in text:
            return {
                "request_id": request_id,
                "action": "navigate",
                "params": {"target": "living_room"},
                "context": {"forbidden_zones": ["utility_room"]},
                "source": {"type": "mock", "text": text},
            }

        if "水杯" in text or "杯子" in text:
            return {
                "request_id": request_id,
                "action": "observe",
                "params": {"object": "cup"},
                "context": {},
                "source": {"type": "mock", "text": text},
            }

        return {
            "request_id": request_id,
            "action": "speak",
            "params": {"text": "当前 mock planner 还不能规划该任务。"},
            "context": {},
            "source": {"type": "mock", "text": text},
        }

    def _on_approved(self, msg: String) -> None:
        self.get_logger().info(f"APPROVED {msg.data}")

    def _on_rejected(self, msg: String) -> None:
        self.get_logger().warning(f"REJECTED {msg.data}")


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
