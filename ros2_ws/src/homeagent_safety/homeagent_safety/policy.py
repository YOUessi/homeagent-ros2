from dataclasses import dataclass
from typing import Any, Dict


ALLOWED_ACTIONS = {
    "navigate",
    "observe",
    "speak",
    "pick",
    "place",
    "handover",
    "look_at",
    "stop",
}

DANGEROUS_TAGS = {"sharp", "weapon", "hot", "toxic", "flammable"}


@dataclass(frozen=True)
class Decision:
    allowed: bool
    code: str
    reason: str


def _reject(code: str, reason: str) -> Decision:
    return Decision(False, code, reason)


def evaluate_action(proposal: Dict[str, Any]) -> Decision:
    """Evaluate one high-level Agent action before it reaches a robot skill."""
    if not isinstance(proposal, dict):
        return _reject("INVALID_SCHEMA", "proposal must be a JSON object")

    action = proposal.get("action")
    if not isinstance(action, str) or not action:
        return _reject("INVALID_SCHEMA", "missing action")
    if action not in ALLOWED_ACTIONS:
        return _reject("UNKNOWN_ACTION", f"action '{action}' is not whitelisted")

    params = proposal.get("params") or {}
    context = proposal.get("context") or {}
    if not isinstance(params, dict) or not isinstance(context, dict):
        return _reject("INVALID_SCHEMA", "params/context must be JSON objects")

    if context.get("emergency_stop") is True and action != "stop":
        return _reject("E_STOP_ACTIVE", "emergency stop is active")

    if action == "navigate":
        target = params.get("target")
        if not isinstance(target, str) or not target:
            return _reject("MISSING_TARGET", "navigate requires params.target")
        forbidden = context.get("forbidden_zones") or []
        if target in forbidden:
            return _reject("FORBIDDEN_ZONE", f"target zone '{target}' is forbidden")

    if action in {"pick", "handover"}:
        obj = params.get("object")
        if not isinstance(obj, str) or not obj:
            return _reject("MISSING_OBJECT", f"{action} requires params.object")

    if action == "handover":
        tags = set(params.get("object_tags") or []) | set(context.get("object_tags") or [])
        recipient_age = context.get("recipient_age", params.get("recipient_age"))
        if tags & DANGEROUS_TAGS:
            if recipient_age is None:
                return _reject(
                    "RECIPIENT_UNKNOWN",
                    "dangerous-object handover requires recipient age/context",
                )
            try:
                age = int(recipient_age)
            except (TypeError, ValueError):
                return _reject("INVALID_RECIPIENT", "recipient_age must be an integer")
            if age < 18:
                return _reject(
                    "DANGEROUS_HANDOVER_MINOR",
                    "dangerous objects cannot be handed to a minor",
                )

    return Decision(True, "ALLOW", "action passed safety policy")
