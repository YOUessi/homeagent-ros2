import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .policy import evaluate_action


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_safety")
        self._proposal_sub = self.create_subscription(
            String,
            "/homeagent/action_proposal",
            self._on_proposal,
            10,
        )
        self._approved_pub = self.create_publisher(
            String, "/homeagent/action_approved", 10
        )
        self._rejected_pub = self.create_publisher(
            String, "/homeagent/action_rejected", 10
        )
        self.get_logger().info("HomeAgent safety engine ready")

    def _on_proposal(self, msg: String) -> None:
        try:
            proposal = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_rejection(None, "INVALID_JSON", str(exc))
            return

        decision = evaluate_action(proposal)
        if decision.allowed:
            envelope = {
                "proposal": proposal,
                "safety": {"allowed": True, "code": decision.code},
            }
            out = String()
            out.data = json.dumps(envelope, ensure_ascii=False)
            self._approved_pub.publish(out)
            self.get_logger().info(
                f"ALLOW action={proposal.get('action')} request_id={proposal.get('request_id')}"
            )
            return

        self._publish_rejection(
            proposal,
            decision.code,
            decision.reason,
        )

    def _publish_rejection(self, proposal, code: str, reason: str) -> None:
        envelope = {
            "proposal": proposal,
            "safety": {"allowed": False, "code": code, "reason": reason},
        }
        out = String()
        out.data = json.dumps(envelope, ensure_ascii=False)
        self._rejected_pub.publish(out)
        self.get_logger().warning(f"REJECT code={code} reason={reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
