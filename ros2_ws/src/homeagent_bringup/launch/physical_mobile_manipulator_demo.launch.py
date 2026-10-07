import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package, filename, arguments=None):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    physics_robot = _include(
        "homeagent_manipulation",
        "homebot_arm_gazebo_moveit.launch.py",
    )

    nav2 = _include(
        "homeagent_navigation",
        "homebot_nav2_runtime.launch.py",
    )

    core = _include(
        "homeagent_bringup",
        "homeagent_core.launch.py",
        {
            "use_nav2": "true",
            "use_moveit": "false",
            "use_gazebo_contact_pick": "true",
            "use_deepseek": "false",
            "use_sim_time": "true",
            "memory_db": "/tmp/homeagent_physical_mobile.sqlite3",
        },
    )

    return LaunchDescription([physics_robot, nav2, core])
