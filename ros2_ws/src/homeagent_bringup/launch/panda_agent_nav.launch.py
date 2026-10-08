"""Unified Panda mobile manipulator + Nav2 + trusted HomeAgent stack.

Tests:
- one Gazebo model homebot_panda with seven physical Panda joints;
- Nav2 using enlarged stowed-arm collision envelope;
- task interpretation, memory, safety and ROS2 skill execution;
- no automatic Panda pick, which has not passed object-contact validation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    nav_share = get_package_share_directory("homeagent_navigation")
    physics = include(
        "homeagent_manipulation", "panda_gazebo_physics.launch.py"
    )
    nav2 = include(
        "homeagent_navigation", "homebot_nav2_runtime.launch.py",
        {
            "params_file": os.path.join(
                nav_share, "config", "nav2_params_panda.yaml"
            ),
        },
    )
    core = include(
        "homeagent_bringup", "homeagent_core.launch.py",
        {
            "use_nav2": "true",
            "nav_require_panda_stow": "true",
            "nav_postcondition_tolerance_m": "0.15",
            "nav_false_success_max_retries": "2",
            "nav_false_success_retry_delay_sec": "3.0",
            "use_panda_moveit": "true",
            "use_moveit": "false",
            "use_gazebo_contact_pick": "false",
            "use_deepseek": "false",
            "use_sim_time": "true",
            "memory_db": "/tmp/homeagent_panda_agent_nav_memory.sqlite3",
        },
    )

    # One launch context preserves coherent remappings and TF graph, while
    # staggering startup prevents Nav2 lifecycle from racing Gazebo spawning.
    return LaunchDescription([
        physics,
        TimerAction(period=8.0, actions=[nav2]),
        TimerAction(period=14.0, actions=[core]),
    ])
