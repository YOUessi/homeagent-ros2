import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    manipulation_share = Path(
        get_package_share_directory("homeagent_manipulation")
    )
    description_share = Path(
        get_package_share_directory("homeagent_description")
    )
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    controllers_file = str(
        manipulation_share / "config" / "ros2_controllers.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "homebot_arm",
            package_name="homeagent_manipulation",
        )
        .robot_description(
            file_path="config/homebot_arm_gazebo.urdf.xacro",
            mappings={"controllers_file": controllers_file},
        )
        .robot_description_semantic(
            file_path="config/homebot_arm.srdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "gazebo.launch.py")
        ),
        launch_arguments={
            "world": str(
                description_share / "worlds" / "home_room.world"
            ),
            "gui": "false",
            "verbose": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "homebot_arm",
            "-topic",
            "robot_description",
            "-z",
            "0.12",
        ],
    )

    joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    arm_controller = TimerAction(
        period=3.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "homearm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    gripper_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    move_group = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": True},
                    {
                        "allow_trajectory_execution": True,
                        "publish_robot_description_semantic": True,
                        "publish_planning_scene": True,
                        "publish_geometry_updates": True,
                        "publish_state_updates": True,
                        "publish_transforms_updates": True,
                    },
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn,
            joint_state_broadcaster,
            arm_controller,
            gripper_controller,
            move_group,
        ]
    )
