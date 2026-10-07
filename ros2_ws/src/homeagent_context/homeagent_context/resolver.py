import math
from typing import Any, Dict, Iterable, Optional


MANIPULATION_ACTIONS = {"pick", "place", "handover"}


def _normalize_map_pose(place_record: Dict[str, Any]) -> Optional[list]:
    payload = place_record.get("payload") or {}
    raw = payload.get("map_pose")

    if isinstance(raw, dict):
        raw = [raw.get("x"), raw.get("y"), raw.get("yaw", 0.0)]

    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        return None

    try:
        pose = [float(raw[0]), float(raw[1]), float(raw[2])]
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(value) for value in pose):
        return None
    return pose


def _normalize_arm_joint_target(raw: Any) -> Optional[list]:
    if isinstance(raw, dict):
        raw = [
            raw.get("joint1"),
            raw.get("joint2"),
            raw.get("joint3"),
            raw.get("joint4"),
        ]

    if not isinstance(raw, (list, tuple)) or len(raw) != 4:
        return None

    try:
        joints = [float(value) for value in raw]
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(value) for value in joints):
        return None
    return joints


def resolve_trusted_context(
    *,
    action: str,
    params: Dict[str, Any],
    object_record: Optional[Dict[str, Any]] = None,
    person_record: Optional[Dict[str, Any]] = None,
    place_record: Optional[Dict[str, Any]] = None,
    forbidden_zones: Iterable[str] = (),
) -> Dict[str, Any]:
    """Build execution/safety context only from trusted policy and memory records."""
    context: Dict[str, Any] = {
        "safety_context_trusted": True,
        "context_source": "trusted_world_state",
    }

    if action == "navigate":
        context["forbidden_zones"] = list(forbidden_zones)
        if place_record is None:
            return {
                **context,
                "safety_context_trusted": False,
                "context_error": "PLACE_NOT_IN_MEMORY",
            }

        pose = _normalize_map_pose(place_record)
        if pose is None:
            return {
                **context,
                "safety_context_trusted": False,
                "context_error": "PLACE_POSE_INVALID",
            }

        context["resolved_target_pose"] = pose
        context["place_id"] = str(place_record.get("entity_id", ""))
        context["place_name"] = str(place_record.get("name", params.get("target", "")))
        context["place_confidence"] = float(place_record.get("confidence", 0.0))
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

    if action == "pick":
        manipulation = object_payload.get("manipulation") or {}
        joint_target = _normalize_arm_joint_target(
            manipulation.get("pick_approach_joint_target")
        )
        if joint_target is None:
            return {
                **context,
                "safety_context_trusted": False,
                "context_error": "PICK_APPROACH_UNKNOWN",
            }
        context["resolved_arm_joint_target"] = joint_target
        context["manipulation_stage"] = "pick_approach"
        context["manipulation_source"] = "object_memory"
        return context

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
