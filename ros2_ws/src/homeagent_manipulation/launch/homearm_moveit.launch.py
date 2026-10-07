from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = Path(get_package_share_directory("homeagent_manipulation"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    clock_params = {"use_sim_time": use_sim_time}

    moveit_config = (
        MoveItConfigsBuilder(
            "homearm",
            package_name="homeagent_manipulation",
        )
        .robot_description(file_path="config/homearm.urdf.xacro")
        .robot_description_semantic(file_path="config/homearm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="homearm_robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, clock_params],
        remappings=[
            ("robot_description", "/homearm/robot_description"),
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            str(package_share / "config" / "ros2_controllers.yaml"),
            clock_params,
        ],
    )

    joint_state_broadcaster = TimerAction(
        period=1.0,
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

    homearm_controller = TimerAction(
        period=1.5,
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
        period=2.0,
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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            clock_params,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock for HomeArm / MoveIt2.",
            ),
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster,
            homearm_controller,
            gripper_controller,
            move_group,
        ]
    )
