import math

from homeagent_skills.panda_targets import (
    PANDA_INSPECT, PANDA_JOINT_NAMES, PANDA_READY,
)
from homeagent_skills.panda_stow_guard import validate_panda_stow


def _state(positions):
    return dict(zip(PANDA_JOINT_NAMES, positions))


def test_panda_stowed_within_tolerance():
    ok, code, data = validate_panda_stow(_state(PANDA_READY))
    assert ok and code == "PANDA_ARM_STOW_VERIFIED"
    assert data["max_abs_joint_error_rad"] == 0.0


def test_panda_inspection_blocks_navigation():
    ok, code, data = validate_panda_stow(_state(PANDA_INSPECT))
    assert not ok and code == "PANDA_ARM_NOT_STOWED"
    assert data["max_abs_joint_error_rad"] > 0.1


def test_panda_missing_joint_fail_closed():
    partial = _state(PANDA_READY)
    del partial["panda_joint4"]
    ok, code, data = validate_panda_stow(partial)
    assert not ok and code == "PANDA_ARM_STATE_UNAVAILABLE"
    assert data["missing_joints"] == ["panda_joint4"]


def test_panda_nonfinite_joint_fail_closed():
    sample = _state(PANDA_READY)
    sample["panda_joint4"] = math.nan
    ok, code, _ = validate_panda_stow(sample)
    assert not ok and code == "PANDA_ARM_STATE_INVALID"


def test_panda_insufficiently_stowed_fail_closed():
    sample = _state(PANDA_READY)
    sample["panda_joint2"] += 0.10
    ok, code, _ = validate_panda_stow(sample, tolerance_rad=0.07)
    assert not ok and code == "PANDA_ARM_NOT_STOWED"
