from homeagent_skills.panda_targets import (
    PANDA_INSPECT, PANDA_JOINT_NAMES, PANDA_READY, resolve_panda_approved_action
)


def test_panda_profile_contains_seven_distinct_joint_names():
    assert len(PANDA_JOINT_NAMES) == 7
    assert len(set(PANDA_JOINT_NAMES)) == 7
    assert len(PANDA_READY) == 7
    assert len(PANDA_INSPECT) == 7


def test_named_panda_action_ignores_untrusted_joint_overrides():
    name, goal = resolve_panda_approved_action(
        "look_at", {"params": {"target": "inspection_point", "joints": [99] * 7}}
    )
    assert name == "inspect"
    assert goal == PANDA_INSPECT


def test_navigation_and_pick_not_owned_by_panda_demo_adapter():
    assert resolve_panda_approved_action(
        "navigate", {"params": {"target": "living_room"}}
    ) is None
    assert resolve_panda_approved_action(
        "pick", {"params": {"object": "cup"}}
    ) is None


def test_panda_stow_uses_ready_profile_only():
    assert resolve_panda_approved_action(
        "stow_arm", {"params": {}}
    ) == ("ready", PANDA_READY)
