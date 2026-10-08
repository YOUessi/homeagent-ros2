import math
from typing import Tuple


def trusted_navigation_pose(
    proposal: dict,
) -> Tuple[str, Tuple[float, float, float], str]:
    """Extract a Safety-approved navigation target from trusted context."""
    params = proposal.get("params") or {}
    context = proposal.get("context") or {}

    target = params.get("target")
    if not isinstance(target, str) or not target:
        raise ValueError("missing navigation target")

    if context.get("safety_context_trusted") is not True:
        raise ValueError("navigation context is not trusted")

    raw_pose = context.get("resolved_target_pose")
    if not isinstance(raw_pose, list) or len(raw_pose) != 3:
        raise ValueError("resolved_target_pose must be [x, y, yaw]")

    try:
        pose = tuple(float(value) for value in raw_pose)
    except (TypeError, ValueError) as exc:
        raise ValueError("resolved_target_pose contains non-numeric values") from exc

    if not all(math.isfinite(value) for value in pose):
        raise ValueError("resolved_target_pose contains non-finite values")

    source = str(context.get("navigation_source") or "trusted_memory")
    return target, pose, source
