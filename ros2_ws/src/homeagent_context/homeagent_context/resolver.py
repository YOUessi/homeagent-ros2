import math
import time
from typing import Any, Dict, Iterable, Optional


MANIPULATION_ACTIONS = {"pick", "place", "handover"}
OBJECT_PREGRASP_PREFIX = "object_pregrasp:"


def _finite_floats(raw, expected_len: int) -> Optional[list]:
    if not isinstance(raw, (list, tuple)) or len(raw) != expected_len:
        return None
    try:
        values = [float(value) for value in raw]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _normalize_map_pose(place_record: Dict[str, Any]) -> Optional[list]:
    payload = place_record.get("payload") or {}
    raw = payload.get("map_pose")

    if isinstance(raw, dict):
        raw = [raw.get("x"), raw.get("y"), raw.get("yaw", 0.0)]

    return _finite_floats(raw, 3)


def _normalize_arm_joint_target(raw: Any) -> Optional[list]:
    if isinstance(raw, dict):
        raw = [
            raw.get("joint1"),
            raw.get("joint2"),
            raw.get("joint3"),
            raw.get("joint4"),
        ]
    return _finite_floats(raw, 4)


def _normalize_object_observation(
    object_record: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    payload = object_record.get("payload") or {}
    location = payload.get("location") or {}
    pose = location.get("pose") or {}

    try:
        x = float(pose.get("x"))
        y = float(pose.get("y"))
        z = float(pose.get("z"))
        yaw = float(pose.get("yaw", 0.0))
        observed_at = float(location.get("observed_at"))
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(v) for v in [x, y, z, yaw, observed_at]):
        return None

    frame = str(pose.get("frame", ""))
    zone = str(location.get("zone", ""))
    if frame not in {"map", "world"}:
        return None

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "frame": frame,
        "zone": zone,
        "observed_at": observed_at,
    }


def _normalize_raw_world_pose(
    object_record: Dict[str, Any],
) -> Optional[Dict[str, float]]:
    payload = object_record.get("payload") or {}
    perception = payload.get("perception") or {}
    raw = perception.get("raw_world_pose") or {}

    try:
        x = float(raw.get("x"))
        y = float(raw.get("y"))
        z = float(raw.get("z"))
        yaw = float(raw.get("yaw", 0.0))
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(v) for v in [x, y, z, yaw]):
        return None
    if str(raw.get("frame", "")) != "gazebo_world":
        return None

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "frame": "gazebo_world",
    }


def _normalize_pregrasp_offset(
    object_record: Dict[str, Any],
) -> Optional[Dict[str, float]]:
    payload = object_record.get("payload") or {}
    manipulation = payload.get("manipulation") or {}
    raw = manipulation.get("mobile_pregrasp_offset") or {}

    try:
        x = float(raw.get("x"))
        y = float(raw.get("y"))
        yaw = float(raw.get("yaw", 0.0))
        expected_z = float(raw.get("expected_object_z", 0.0))
    except (TypeError, ValueError):
        return None

    if not all(math.isfinite(v) for v in [x, y, yaw, expected_z]):
        return None

    return {
        "x": x,
        "y": y,
        "yaw": yaw,
        "expected_object_z": expected_z,
    }


def _object_height_error(
    observation: Dict[str, Any],
    offset: Dict[str, float],
    *,
    tolerance_m: float = 0.08,
) -> Optional[Dict[str, float]]:
    """Reject a pregrasp calibrated for a different object support height."""
    expected_z = float(offset["expected_object_z"])
    actual_z = float(observation["z"])
    error = abs(actual_z - expected_z)
    if error <= tolerance_m:
        return None
    return {
        "observed_object_z": actual_z,
        "expected_object_z": expected_z,
        "object_height_error_m": error,
    }


def _resolved_pregrasp_pose(
    observation: Dict[str, Any],
    offset: Dict[str, float],
) -> list:
    base_yaw = observation["yaw"] + offset["yaw"]
    c = math.cos(base_yaw)
    s = math.sin(base_yaw)
    dx_world = c * offset["x"] - s * offset["y"]
    dy_world = s * offset["x"] + c * offset["y"]
    return [
        observation["x"] - dx_world,
        observation["y"] - dy_world,
        base_yaw,
    ]


def _object_pregrasp_context(
    *,
    target: str,
    object_record: Optional[Dict[str, Any]],
    forbidden_zones: Iterable[str],
    observation_max_age_sec: float,
    now_wall_time: Optional[float],
) -> Dict[str, Any]:
    context: Dict[str, Any] = {
        "safety_context_trusted": True,
        "context_source": "trusted_world_state",
        "forbidden_zones": list(forbidden_zones),
        "navigation_source": "object_memory_observation",
    }

    if object_record is None:
        return {
            **context,
            "safety_context_trusted": False,
            "context_error": "OBJECT_NOT_IN_MEMORY",
        }

    observation = _normalize_object_observation(object_record)
    if observation is None:
        return {
            **context,
            "safety_context_trusted": False,
            "context_error": "OBJECT_POSE_NOT_OBSERVED",
        }

    now = time.time() if now_wall_time is None else float(now_wall_time)
    age = max(0.0, now - observation["observed_at"])
    if age > float(observation_max_age_sec):
        return {
            **context,
            "safety_context_trusted": False,
            "context_error": "OBJECT_OBSERVATION_STALE",
            "object_observation_age_sec": age,
        }

    offset = _normalize_pregrasp_offset(object_record)
    if offset is None:
        return {
            **context,
            "safety_context_trusted": False,
            "context_error": "MOBILE_PREGRASP_UNKNOWN",
        }

    height_error = _object_height_error(observation, offset)
    if height_error is not None:
        return {
            **context,
            "safety_context_trusted": False,
            "context_error": "OBJECT_HEIGHT_OUT_OF_RANGE",
            **height_error,
        }

    resolved_pose = _resolved_pregrasp_pose(observation, offset)

    context.update(
        {
            "resolved_target_pose": resolved_pose,
            "target_zone": observation["zone"],
            "object_name": str(object_record.get("name", "")),
            "object_id": str(object_record.get("entity_id", "")),
            "object_observed_pose": observation,
            "object_observation_age_sec": age,
            "pregrasp_offset": offset,
            "requested_target": target,
        }
    )
    return context


def resolve_trusted_context(
    *,
    action: str,
    params: Dict[str, Any],
    object_record: Optional[Dict[str, Any]] = None,
    person_record: Optional[Dict[str, Any]] = None,
    place_record: Optional[Dict[str, Any]] = None,
    forbidden_zones: Iterable[str] = (),
    observation_max_age_sec: float = 5.0,
    now_wall_time: Optional[float] = None,
) -> Dict[str, Any]:
    """Build execution/safety context only from trusted policy and memory records."""
    context: Dict[str, Any] = {
        "safety_context_trusted": True,
        "context_source": "trusted_world_state",
    }

    if action == "navigate":
        target = str(params.get("target", ""))
        if target.startswith(OBJECT_PREGRASP_PREFIX):
            return _object_pregrasp_context(
                target=target,
                object_record=object_record,
                forbidden_zones=forbidden_zones,
                observation_max_age_sec=observation_max_age_sec,
                now_wall_time=now_wall_time,
            )

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
        context["place_name"] = str(
            place_record.get("name", params.get("target", ""))
        )
        context["place_confidence"] = float(
            place_record.get("confidence", 0.0)
        )
        context["navigation_source"] = "place_memory"
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
    context["object_confidence"] = float(
        object_record.get("confidence", 0.0)
    )

    if action == "pick":
        observed = _normalize_object_observation(object_record)
        pregrasp = _normalize_pregrasp_offset(object_record)
        if observed is not None and pregrasp is not None:
            height_error = _object_height_error(observed, pregrasp)
            if height_error is not None:
                return {
                    **context,
                    "safety_context_trusted": False,
                    "context_error": "OBJECT_HEIGHT_OUT_OF_RANGE",
                    **height_error,
                }

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

        pregrasp_target = _normalize_arm_joint_target(
            manipulation.get("pick_pregrasp_joint_target")
        )
        if pregrasp_target is not None:
            context["resolved_arm_pregrasp_joint_target"] = pregrasp_target

        context["manipulation_stage"] = "pick_approach"
        context["manipulation_source"] = "object_memory"

        if observed is not None:
            context["object_observed_pose"] = observed
        if observed is not None and pregrasp is not None:
            context["pregrasp_offset"] = pregrasp
            context["resolved_mobile_pregrasp_pose"] = (
                _resolved_pregrasp_pose(observed, pregrasp)
            )

        raw_world = _normalize_raw_world_pose(object_record)
        if raw_world is not None and pregrasp is not None:
            context["object_raw_world_pose"] = raw_world
            context["resolved_sim_world_pregrasp_pose"] = (
                _resolved_pregrasp_pose(raw_world, pregrasp)
            )
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
