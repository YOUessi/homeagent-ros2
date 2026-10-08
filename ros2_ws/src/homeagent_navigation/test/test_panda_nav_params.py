from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config" / "nav2_params_panda.yaml"


def _profile():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def test_panda_dwb_goal_tolerance_matches_nav2_goal_checker():
    controller = _profile()["controller_server"]["ros__parameters"]
    goal = controller["general_goal_checker"]
    local = controller["FollowPath"]

    assert goal["plugin"] == "nav2_controller::SimpleGoalChecker"
    assert 0.05 <= float(goal["xy_goal_tolerance"]) <= 0.15
    assert (
        float(local["xy_goal_tolerance"])
        == float(goal["xy_goal_tolerance"])
    ), "DWB must not start rotating before the translation goal is reached"


def test_panda_dwb_stopped_velocity_is_a_small_fraction_of_max_speed():
    controller = _profile()["controller_server"]["ros__parameters"]
    local = controller["FollowPath"]
    max_speed = float(local["max_vel_x"])
    stopped_speed = float(local["trans_stopped_velocity"])

    assert max_speed > 0
    assert 0 < stopped_speed < max_speed * 0.25


def test_panda_navigation_keeps_stall_watchdog_enabled():
    controller = _profile()["controller_server"]["ros__parameters"]
    progress = controller["progress_checker"]
    assert progress["plugin"] == "nav2_controller::SimpleProgressChecker"
    assert 0 < float(progress["required_movement_radius"]) < 0.15
    assert 5.0 <= float(progress["movement_time_allowance"]) <= 30.0


def test_panda_navigation_uses_larger_combined_robot_collision_envelope():
    profile = _profile()
    local = profile["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = profile["global_costmap"]["global_costmap"]["ros__parameters"]
    assert float(local["robot_radius"]) >= 0.42
    assert float(global_costmap["robot_radius"]) >= 0.42
