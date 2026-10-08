import math

from homeagent_perception.observation import (
    build_map_pose,
    find_model_index,
    perception_payload,
    quaternion_to_yaw,
)


def test_quaternion_to_yaw_identity():
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == 0.0


def test_quaternion_to_yaw_quarter_turn():
    z = math.sin(math.pi / 4.0)
    w = math.cos(math.pi / 4.0)
    assert abs(quaternion_to_yaw(0.0, 0.0, z, w) - math.pi / 2.0) < 1e-9


def test_build_map_pose_and_payload():
    pose = build_map_pose(
        position=[1.2, -0.3, 0.45],
        quaternion=[0.0, 0.0, 0.0, 1.0],
    )
    assert pose["frame"] == "map"
    assert pose["x"] == 1.2
    assert pose["z"] == 0.45
    payload = perception_payload("physical_cup", pose)
    assert payload["perception"]["backend"] == "gazebo_ground_truth"
    assert payload["perception"]["model_name"] == "physical_cup"


def test_find_model_index():
    assert find_model_index(["ground", "physical_cup"], "physical_cup") == 1
    assert find_model_index(["ground"], "physical_cup") is None


def test_world_pose_projects_into_map_using_robot_alignment():
    from homeagent_perception.observation import transform_world_pose_to_map

    object_world = {
        "frame": "gazebo_world",
        "x": 1.0,
        "y": 0.0,
        "z": 0.62,
        "yaw": 0.0,
    }
    robot_world = {
        "frame": "gazebo_world",
        "x": 0.2,
        "y": 0.1,
        "z": 0.13,
        "yaw": 0.0,
    }
    robot_map = {
        "frame": "map",
        "x": 0.15,
        "y": 0.08,
        "z": 0.0,
        "yaw": 0.0,
    }

    pose = transform_world_pose_to_map(
        object_world=object_world,
        robot_world=robot_world,
        robot_map=robot_map,
    )
    assert abs(pose["x"] - 0.95) < 1e-9
    assert abs(pose["y"] + 0.02) < 1e-9
    assert abs(pose["z"] - 0.62) < 1e-9
    assert pose["frame"] == "map"


def test_world_pose_projection_handles_robot_yaw():
    from homeagent_perception.observation import transform_world_pose_to_map

    object_world = {
        "frame": "gazebo_world",
        "x": 1.0,
        "y": 0.0,
        "z": 0.62,
        "yaw": 0.0,
    }
    robot_world = {
        "frame": "gazebo_world",
        "x": 0.0,
        "y": 0.0,
        "z": 0.13,
        "yaw": 0.0,
    }
    robot_map = {
        "frame": "map",
        "x": 2.0,
        "y": 3.0,
        "z": 0.0,
        "yaw": math.pi / 2.0,
    }

    pose = transform_world_pose_to_map(
        object_world=object_world,
        robot_world=robot_world,
        robot_map=robot_map,
    )
    assert abs(pose["x"] - 2.0) < 1e-9
    assert abs(pose["y"] - 4.0) < 1e-9
    assert abs(pose["yaw"] - math.pi / 2.0) < 1e-9
