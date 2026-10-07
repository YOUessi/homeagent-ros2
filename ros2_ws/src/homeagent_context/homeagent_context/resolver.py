from typing import Any, Dict, Iterable, Optional


MANIPULATION_ACTIONS = {"pick", "place", "handover"}


def resolve_trusted_context(
    *,
    action: str,
    params: Dict[str, Any],
    object_record: Optional[Dict[str, Any]] = None,
    person_record: Optional[Dict[str, Any]] = None,
    forbidden_zones: Iterable[str] = (),
) -> Dict[str, Any]:
    """Build safety context only from trusted policy/memory records."""
    context: Dict[str, Any] = {
        "safety_context_trusted": True,
        "context_source": "trusted_world_state",
    }

    if action == "navigate":
        context["forbidden_zones"] = list(forbidden_zones)
        return context

    if action not in MANIPULATION_ACTIONS:
        return context

    if object_record is None:
        return {
            "safety_context_trusted": False,
            "context_source": "trusted_world_state",
            "context_error": "OBJECT_NOT_IN_MEMORY",
        }

    object_payload = object_record.get("payload") or {}
    context["object_tags"] = list(object_payload.get("tags") or [])
    context["object_confidence"] = float(object_record.get("confidence", 0.0))

    if action == "handover":
        if person_record is None:
            return {
                **context,
                "safety_context_trusted": False,
                "context_error": "RECIPIENT_NOT_IN_MEMORY",
            }
        person_payload = person_record.get("payload") or {}
        age = person_payload.get("age")
        if age is None:
            return {
                **context,
                "safety_context_trusted": False,
                "context_error": "RECIPIENT_AGE_UNKNOWN",
            }
        context["recipient_age"] = int(age)
        context["recipient_confidence"] = float(
            person_record.get("confidence", 0.0)
        )

    return context
