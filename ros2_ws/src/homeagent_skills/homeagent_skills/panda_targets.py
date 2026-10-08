"""Trusted named Panda joint targets for approved high-level skills.

The Agent cannot send raw Panda joint arrays or trajectories. These values
belong to the robot-specific adapter and are not derived from candidate params.
"""
import math

PANDA_JOINT_NAMES = tuple(f"panda_joint{i}" for i in range(1, 8))
PANDA_READY = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)
PANDA_INSPECT = (0.40, -0.65, 0.18, -2.20, 0.08, 1.82, 0.53)
PANDA_NAMED_ACTIONS = {
    "stow_arm": ("ready", PANDA_READY),
    "look_at": ("inspect", PANDA_INSPECT),
}


def resolve_panda_approved_action(action, proposal):
    """Return named target for actions owned by Panda (never untrusted joints)."""
    if action not in PANDA_NAMED_ACTIONS:
        return None
    if not isinstance(proposal, dict):
        raise ValueError("proposal must be an object")
    name, target = PANDA_NAMED_ACTIONS[action]
    if not all(math.isfinite(v) for v in target):
        raise ValueError("non-finite trusted Panda target")
    return name, tuple(target)
