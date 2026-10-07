import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("homeagent_description")
    gazebo_share = get_package_share_directory("gazebo_ros")

    xacro_file = os.path.join(description_share, "urdf", "homebot.urdf.xacro")
    world_file = os.path.join(description_share, "worlds", "home_room.world")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
            "gui": gui,
        }.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "homebot",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.16",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start Gazebo client GUI.",
            ),
            gazebo,
            state_publisher,
            spawn,
        ]
    )
