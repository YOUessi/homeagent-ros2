import math


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]

NAMED_ARM_TARGETS = {
    "home": [0.0, 0.0, 0.0, 0.0],
    "inspect": [0.35, -0.55, 1.05, -0.50],
    "handover": [0.0, -0.30, 0.65, -0.35],
}

ACTION_TO_ARM_TARGET = {
    "look_at": "inspect",
    "handover": "handover",
}


def _validated_joint_target(raw):
    if not isinstance(raw, (list, tuple)) or len(raw) != len(JOINT_NAMES):
        raise ValueError("arm joint target must contain four joints")
    try:
        joints = [float(value) for value in raw]
    except (TypeError, ValueError) as exc:
        raise ValueError("arm joint target must be numeric") from exc
    if not all(math.isfinite(value) for value in joints):
        raise ValueError("arm joint target must be finite")
    return joints


def target_for_action(action: str):
    name = ACTION_TO_ARM_TARGET.get(action)
    if name is None:
        return None
    return name, list(NAMED_ARM_TARGETS[name])


def resolve_arm_target(action: str, proposal: dict):
    """Resolve a Safety-approved arm target without trusting Agent joint angles."""
    named = target_for_action(action)
    if named is not None:
        name, joints = named
        return name, joints, "trusted_skill_library"

    if action != "pick":
        return None

    context = proposal.get("context") or {}
    params = proposal.get("params") or {}
    if context.get("safety_context_trusted") is not True:
        raise ValueError("pick context is not trusted")

    joints = _validated_joint_target(context.get("resolved_arm_joint_target"))
    obj = params.get("object")
    if not isinstance(obj, str) or not obj:
        raise ValueError("pick requires params.object")

    return f"pick_approach:{obj}", joints, "trusted_object_memory"
