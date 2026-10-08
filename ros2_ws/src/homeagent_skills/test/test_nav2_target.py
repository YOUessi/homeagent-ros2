import pytest

from homeagent_skills.navigation_target import trusted_navigation_pose


def test_trusted_navigation_pose_comes_from_context():
    target, pose, source = trusted_navigation_pose(
        {
            "params": {"target": "living_room"},
            "context": {
                "safety_context_trusted": True,
                "resolved_target_pose": [0.8, 0.0, 0.0],
                "navigation_source": "place_memory",
            },
        }
    )
    assert target == "living_room"
    assert pose == (0.8, 0.0, 0.0)
    assert source == "place_memory"


def test_untrusted_navigation_pose_is_rejected():
    with pytest.raises(ValueError, match="not trusted"):
        trusted_navigation_pose(
            {
                "params": {"target": "living_room"},
                "context": {
                    "safety_context_trusted": False,
                    "resolved_target_pose": [0.8, 0.0, 0.0],
                },
            }
        )


def test_non_finite_navigation_pose_is_rejected():
    with pytest.raises(ValueError, match="non-finite"):
        trusted_navigation_pose(
            {
                "params": {"target": "living_room"},
                "context": {
                    "safety_context_trusted": True,
                    "resolved_target_pose": [float("nan"), 0.0, 0.0],
                },
            }
        )
