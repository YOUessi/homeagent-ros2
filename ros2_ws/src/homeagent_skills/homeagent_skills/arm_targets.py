NAMED_ARM_TARGETS = {
    "home": [0.0, 0.0, 0.0, 0.0],
    "inspect": [0.35, -0.55, 1.05, -0.50],
    "handover": [0.0, -0.30, 0.65, -0.35],
}

ACTION_TO_ARM_TARGET = {
    "look_at": "inspect",
    "handover": "handover",
}


def target_for_action(action: str):
    name = ACTION_TO_ARM_TARGET.get(action)
    if name is None:
        return None
    return name, list(NAMED_ARM_TARGETS[name])
