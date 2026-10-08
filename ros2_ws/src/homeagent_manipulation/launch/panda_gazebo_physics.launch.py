"""Independent Gazebo Classic physics + MoveIt2 launch for official Panda CAD.

This is a fixed-base physical-joint baseline. It intentionally does not claim
HomeBot mounting or contact grasp until their own tests pass.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    manipulation = Path(get_package_share_directory("homeagent_manipulation"))
    description = Path(get_package_share_directory("homeagent_description"))
    gazebo = Path(get_package_share_directory("gazebo_ros"))

    robot_file = Path(os.environ.get(
        "HOMEAGENT_PANDA_GAZEBO_URDF",
        "/workspace/artifacts/panda_physics/panda_gazebo.urdf",
    ))
    if not robot_file.exists():
        raise RuntimeError(f"Missing generated Panda physics URDF: {robot_file}")
    robot_description = robot_file.read_text(encoding="utf-8")

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description["robot_description"] = robot_description

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo / "launch" / "gazebo.launch.py")),
        launch_arguments={
            "world": str(description / "worlds" / "home_room.world"),
            "gui": "false",
            "verbose": "true",
        }.items(),
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-entity", "panda_physics",
                    "-topic", "robot_description",
                ],
            ),
        ],
    )
    controllers = []
    for offset, name in (
        (5.0, "joint_state_broadcaster"),
        (6.0, "panda_arm_controller"),
        (7.0, "panda_hand_controller"),
    ):
        controllers.append(
            TimerAction(
                period=offset,
                actions=[Node(
                    package="controller_manager",
                    executable="spawner",
                    output="screen",
                    arguments=[
                        name, "--controller-manager",
                        "/controller_manager",
                        "--controller-manager-timeout", "25",
                    ],
                )],
            ),
        )

    move_group = TimerAction(
        period=5.5,
        actions=[Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": True},
                {"allow_trajectory_execution": True},
            ],
        )],
    )
    return LaunchDescription(
        [gzserver, rsp, spawn, *controllers, move_group]
    )
