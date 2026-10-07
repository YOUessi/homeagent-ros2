import pytest

from homeagent_skills.arm_targets import resolve_arm_target


def test_named_arm_target_comes_from_trusted_skill_library():
    name, joints, source = resolve_arm_target("look_at", {"context": {}})
    assert name == "inspect"
    assert joints == [0.35, -0.55, 1.05, -0.50]
    assert source == "trusted_skill_library"


def test_pick_target_comes_from_trusted_object_context():
    name, joints, source = resolve_arm_target(
        "pick",
        {
            "params": {"object": "cup"},
            "context": {
                "safety_context_trusted": True,
                "resolved_arm_joint_target": [0.2, -0.75, 1.15, -0.35],
            },
        },
    )
    assert name == "pick_approach:cup"
    assert joints == [0.2, -0.75, 1.15, -0.35]
    assert source == "trusted_object_memory"


def test_pick_rejects_agent_untrusted_joint_target():
    with pytest.raises(ValueError, match="not trusted"):
        resolve_arm_target(
            "pick",
            {
                "params": {"object": "cup"},
                "context": {
                    "safety_context_trusted": False,
                    "resolved_arm_joint_target": [0.2, -0.75, 1.15, -0.35],
                },
            },
        )
