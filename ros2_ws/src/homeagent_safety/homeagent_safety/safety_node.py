import json

import rclpy
from rclpy.node import Node

from homeagent_interfaces.msg import ActionProposal, SafetyDecision
from .policy import evaluate_action


class SafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_safety")
        self._proposal_sub = self.create_subscription(
            ActionProposal,
            "/homeagent/action_proposal",
            self._on_proposal,
            10,
        )
        self._approved_pub = self.create_publisher(
            SafetyDecision, "/homeagent/action_approved", 10
        )
        self._rejected_pub = self.create_publisher(
            SafetyDecision, "/homeagent/action_rejected", 10
        )
        self.get_logger().info("HomeAgent safety engine ready")

    def _on_proposal(self, msg: ActionProposal) -> None:
        try:
            params = json.loads(msg.params_json or "{}")
            context = json.loads(msg.context_json or "{}")
        except json.JSONDecodeError as exc:
            self._publish_decision(
                msg,
                allowed=False,
                code="INVALID_JSON",
                reason=str(exc),
            )
            return

        proposal = {
            "request_id": msg.request_id,
            "action": msg.action,
            "params": params,
            "context": context,
            "source": msg.source,
        }
        decision = evaluate_action(proposal)
        self._publish_decision(
            msg,
            allowed=decision.allowed,
            code=decision.code,
            reason=decision.reason,
            proposal=proposal,
        )

    def _publish_decision(
        self,
        msg: ActionProposal,
        *,
        allowed: bool,
        code: str,
        reason: str,
        proposal=None,
    ) -> None:
        out = SafetyDecision()
        out.request_id = msg.request_id
        out.allowed = allowed
        out.code = code
        out.reason = reason
        out.action = msg.action
        out.proposal_json = json.dumps(proposal or {}, ensure_ascii=False)

        if allowed:
            self._approved_pub.publish(out)
            self.get_logger().info(
                f"ALLOW action={msg.action} request_id={msg.request_id}"
            )
        else:
            self._rejected_pub.publish(out)
            self.get_logger().warning(
                f"REJECT action={msg.action} code={code} request_id={msg.request_id} reason={reason}"
            )


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
