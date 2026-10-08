import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package: str, filename: str, arguments=None):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    params_file = LaunchConfiguration("nav2_params_file")
    navigation_share = get_package_share_directory("homeagent_navigation")

    physics_robot = _include(
        "homeagent_manipulation",
        "homebot_arm_gazebo_moveit.launch.py",
        {"gui": gui},
    )

    nav2 = _include(
        "homeagent_navigation",
        "homebot_nav2_runtime.launch.py",
        {"params_file": params_file},
    )

    core = _include(
        "homeagent_bringup",
        "homeagent_core.launch.py",
        {
            "use_nav2": "true",
            "use_moveit": "true",
            "moveit_execute_pick": "false",
            "use_gazebo_contact_pick": "true",
            "gazebo_contact_spawn_object": "false",
            "use_deepseek": "false",
            "use_sim_time": "true",
            "memory_db": "/tmp/homeagent_perception_fetch.sqlite3",
        },
    )

    perception = Node(
        package="homeagent_perception",
        executable="gazebo_object_perception",
        name="homeagent_perception",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "model_name": "physical_cup",
                "entity_id": "object-cup",
                "zone_hint": "living_room",
                "source": "gazebo_ground_truth_perception",
                "min_update_interval_sec": 0.25,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Launch Gazebo client for visual demonstration.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    navigation_share, "config", "nav2_params_fetch.yaml"
                ),
                description="Nav2 params; alternate config for isolated diagnostics.",
            ),
            physics_robot,
            nav2,
            core,
            perception,
        ]
    )
