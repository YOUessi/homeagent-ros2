import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_share = get_package_share_directory("homeagent_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    params_file = LaunchConfiguration("params_file")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "map": os.path.join(
                navigation_share, "maps", "home_room.yaml"
            ),
            "use_sim_time": "true",
            "params_file": params_file,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    navigation_share, "config", "nav2_params.yaml"
                ),
                description="Nav2 parameter file for this runtime.",
            ),
            nav2,
        ]
    )
