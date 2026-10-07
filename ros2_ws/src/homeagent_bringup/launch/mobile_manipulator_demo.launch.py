import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch(package: str, filename: str, arguments=None):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    nav2_stack = _launch(
        "homeagent_navigation",
        "homebot_nav2.launch.py",
    )

    arm_stack = _launch(
        "homeagent_manipulation",
        "homearm_moveit.launch.py",
        {"use_sim_time": "true"},
    )

    arm_mount = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="homearm_mount_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.22",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "arm_base_footprint",
        ],
    )

    homeagent_core = _launch(
        "homeagent_bringup",
        "homeagent_core.launch.py",
        {
            "use_nav2": "true",
            "use_moveit": "true",
            "use_deepseek": "false",
            "use_sim_time": "true",
            "memory_db": "/tmp/homeagent_mobile_manipulator.sqlite3",
        },
    )

    return LaunchDescription(
        [
            nav2_stack,
            arm_stack,
            arm_mount,
            homeagent_core,
        ]
    )
