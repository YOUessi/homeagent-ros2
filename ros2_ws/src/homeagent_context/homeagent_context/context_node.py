import json
from typing import Optional

import rclpy
from rclpy.node import Node

from homeagent_interfaces.msg import ActionProposal
from homeagent_interfaces.srv import MemoryQuery

from .resolver import (
    MANIPULATION_ACTIONS,
    OBJECT_PREGRASP_PREFIX,
    resolve_trusted_context,
)


class TrustedContextNode(Node):
    """Enrich untrusted Agent candidates with trusted world-state context."""

    def __init__(self) -> None:
        super().__init__("homeagent_context")
        self.declare_parameter("forbidden_zones", ["utility_room"])
        self.declare_parameter("object_observation_max_age_sec", 5.0)
        self._forbidden_zones = list(
            self.get_parameter("forbidden_zones").value
        )
        self._object_observation_max_age = float(
            self.get_parameter("object_observation_max_age_sec").value
        )

        self._memory = self.create_client(
            MemoryQuery, "/homeagent/memory/query"
        )
        self._candidate_sub = self.create_subscription(
            ActionProposal,
            "/homeagent/action_candidate",
            self._on_candidate,
            10,
        )
        self._proposal_pub = self.create_publisher(
            ActionProposal,
            "/homeagent/action_proposal",
            10,
        )
        self.get_logger().info("HomeAgent trusted context resolver ready")

    def _on_candidate(self, msg: ActionProposal) -> None:
        try:
            params = json.loads(msg.params_json or "{}")
        except json.JSONDecodeError as exc:
            self._publish_with_context(
                msg,
                {
                    "safety_context_trusted": False,
                    "context_error": f"INVALID_PARAMS_JSON: {exc}",
                    "context_source": "trusted_world_state",
                },
            )
            return

        if msg.action == "navigate":
            self._resolve_navigation(msg, params)
            return

        if msg.action not in MANIPULATION_ACTIONS:
            context = resolve_trusted_context(
                action=msg.action,
                params=params,
                forbidden_zones=self._forbidden_zones,
            )
            self._publish_with_context(msg, context)
            return

        if not self._ensure_memory_ready():
            self._publish_untrusted(msg, "MEMORY_SERVICE_UNAVAILABLE")
            return

        object_name = params.get("object", "")
        if not object_name:
            self._publish_untrusted(msg, "MISSING_OBJECT")
            return

        request = MemoryQuery.Request()
        request.entity_type = "object"
        request.entity_id = ""
        request.name = str(object_name)
        future = self._memory.call_async(request)
        future.add_done_callback(
            lambda done, candidate=msg, parsed=params:
            self._on_object(candidate, parsed, done)
        )

    def _resolve_navigation(self, msg: ActionProposal, params: dict) -> None:
        target = str(params.get("target", ""))
        if not target:
            self._publish_with_context(
                msg,
                {
                    "safety_context_trusted": False,
                    "context_source": "trusted_world_state",
                    "context_error": "MISSING_TARGET",
                    "forbidden_zones": self._forbidden_zones,
                },
            )
            return

        if not self._ensure_memory_ready():
            self._publish_with_context(
                msg,
                {
                    "safety_context_trusted": False,
                    "context_source": "trusted_world_state",
                    "context_error": "MEMORY_SERVICE_UNAVAILABLE",
                    "forbidden_zones": self._forbidden_zones,
                },
            )
            return

        request = MemoryQuery.Request()
        request.entity_id = ""

        if target.startswith(OBJECT_PREGRASP_PREFIX):
            object_name = target[len(OBJECT_PREGRASP_PREFIX):].strip()
            if not object_name:
                self._publish_with_context(
                    msg,
                    {
                        "safety_context_trusted": False,
                        "context_source": "trusted_world_state",
                        "context_error": "MISSING_OBJECT_PREGRASP_TARGET",
                        "forbidden_zones": self._forbidden_zones,
                    },
                )
                return

            request.entity_type = "object"
            request.name = object_name
            future = self._memory.call_async(request)
            future.add_done_callback(
                lambda done, candidate=msg, parsed=params:
                self._on_navigation_object(candidate, parsed, done)
            )
            return

        request.entity_type = "place"
        request.name = target
        future = self._memory.call_async(request)
        future.add_done_callback(
            lambda done, candidate=msg, parsed=params:
            self._on_place(candidate, parsed, done)
        )

    def _on_navigation_object(
        self, msg: ActionProposal, params: dict, future
    ) -> None:
        try:
            response = future.result()
            object_record = (
                json.loads(response.record_json) if response.found else None
            )
        except Exception as exc:
            self._publish_with_context(
                msg,
                {
                    "safety_context_trusted": False,
                    "context_source": "trusted_world_state",
                    "context_error": f"OBJECT_QUERY_FAILED: {exc}",
                    "forbidden_zones": self._forbidden_zones,
                },
            )
            return

        context = resolve_trusted_context(
            action=msg.action,
            params=params,
            object_record=object_record,
            forbidden_zones=self._forbidden_zones,
            observation_max_age_sec=self._object_observation_max_age,
        )
        self._publish_with_context(msg, context)

    def _on_place(self, msg: ActionProposal, params: dict, future) -> None:
        try:
            response = future.result()
            place_record = (
                json.loads(response.record_json) if response.found else None
            )
        except Exception as exc:
            self._publish_with_context(
                msg,
                {
                    "safety_context_trusted": False,
                    "context_source": "trusted_world_state",
                    "context_error": f"PLACE_QUERY_FAILED: {exc}",
                    "forbidden_zones": self._forbidden_zones,
                },
            )
            return

        context = resolve_trusted_context(
            action=msg.action,
            params=params,
            place_record=place_record,
            forbidden_zones=self._forbidden_zones,
        )
        self._publish_with_context(msg, context)

    def _on_object(self, msg: ActionProposal, params: dict, future) -> None:
        try:
            response = future.result()
            object_record = (
                json.loads(response.record_json) if response.found else None
            )
        except Exception as exc:
            self._publish_untrusted(msg, f"OBJECT_QUERY_FAILED: {exc}")
            return

        if msg.action != "handover":
            context = resolve_trusted_context(
                action=msg.action,
                params=params,
                object_record=object_record,
                forbidden_zones=self._forbidden_zones,
            )
            self._publish_with_context(msg, context)
            return

        recipient = params.get("recipient", "")
        if not recipient:
            self._publish_untrusted(msg, "MISSING_RECIPIENT")
            return

        request = MemoryQuery.Request()
        request.entity_type = "person"
        request.entity_id = ""
        request.name = str(recipient)
        future = self._memory.call_async(request)
        future.add_done_callback(
            lambda done, candidate=msg, parsed=params, obj=object_record:
            self._on_person(candidate, parsed, obj, done)
        )

    def _on_person(
        self,
        msg: ActionProposal,
        params: dict,
        object_record: Optional[dict],
        future,
    ) -> None:
        try:
            response = future.result()
            person_record = (
                json.loads(response.record_json) if response.found else None
            )
        except Exception as exc:
            self._publish_untrusted(msg, f"RECIPIENT_QUERY_FAILED: {exc}")
            return

        context = resolve_trusted_context(
            action=msg.action,
            params=params,
            object_record=object_record,
            person_record=person_record,
            forbidden_zones=self._forbidden_zones,
        )
        self._publish_with_context(msg, context)

    def _ensure_memory_ready(self) -> bool:
        if not self._memory.service_is_ready():
            self._memory.wait_for_service(timeout_sec=0.1)
        return self._memory.service_is_ready()

    def _publish_untrusted(self, msg: ActionProposal, error: str) -> None:
        self._publish_with_context(
            msg,
            {
                "safety_context_trusted": False,
                "context_source": "trusted_world_state",
                "context_error": error,
            },
        )

    def _publish_with_context(
        self, candidate: ActionProposal, context: dict
    ) -> None:
        proposal = ActionProposal()
        proposal.request_id = candidate.request_id
        proposal.action = candidate.action
        proposal.params_json = candidate.params_json
        proposal.context_json = json.dumps(context, ensure_ascii=False)
        proposal.source = candidate.source
        self._proposal_pub.publish(proposal)
        self.get_logger().info(
            f"CONTEXT request_id={proposal.request_id} action={proposal.action} "
            f"trusted={context.get('safety_context_trusted')}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrustedContextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
