"""Fail-closed Panda stow precondition for mobile-base navigation."""
import math

from .panda_targets import PANDA_JOINT_NAMES, PANDA_READY


def validate_panda_stow(positions, *, tolerance_rad=0.07):
    """Return (allowed, code, evidence).

    Named 7-axis positions must match the robot-owned READY profile.
    Raw planner joint targets cannot override this safety prerequisite.
    """
    if not isinstance(positions, dict):
        return False, "PANDA_ARM_STATE_UNAVAILABLE", {
            "reason": "joint states were not received"
        }
    missing = [name for name in PANDA_JOINT_NAMES if name not in positions]
    if missing:
        return False, "PANDA_ARM_STATE_UNAVAILABLE", {
            "missing_joints": missing
        }
    try:
        values = [float(positions[k]) for k in PANDA_JOINT_NAMES]
        limit = float(tolerance_rad)
    except (TypeError, ValueError):
        return False, "PANDA_ARM_STATE_INVALID", {
            "reason": "non-numeric Panda joint position"
        }
    if not all(math.isfinite(v) for v in values) or (
        not math.isfinite(limit) or limit <= 0
    ):
        return False, "PANDA_ARM_STATE_INVALID", {
            "reason": "non-finite Panda joint state or tolerance"
        }
    largest = max(abs(v - target) for v, target in zip(values, PANDA_READY))
    evidence = {
        "max_abs_joint_error_rad": largest,
        "tolerance_rad": limit,
        "joint_names": list(PANDA_JOINT_NAMES),
    }
    if largest > limit:
        return False, "PANDA_ARM_NOT_STOWED", evidence
    return True, "PANDA_ARM_STOW_VERIFIED", evidence
