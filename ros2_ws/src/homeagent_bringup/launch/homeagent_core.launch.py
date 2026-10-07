from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_deepseek = LaunchConfiguration("use_deepseek")
    use_nav2 = LaunchConfiguration("use_nav2")
    memory_db = LaunchConfiguration("memory_db")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_deepseek",
                default_value="false",
                description="Use DeepSeek planner instead of deterministic mock planner.",
            ),
            DeclareLaunchArgument(
                "use_nav2",
                default_value="false",
                description="Use Nav2-backed navigation skill executor.",
            ),
            DeclareLaunchArgument(
                "memory_db",
                default_value="/tmp/homeagent_memory.sqlite3",
                description="SQLite path for household memory.",
            ),
            Node(
                package="homeagent_safety",
                executable="safety_node",
                name="homeagent_safety",
                output="screen",
            ),
            Node(
                package="homeagent_context",
                executable="context_node",
                name="homeagent_context",
                output="screen",
            ),
            Node(
                package="homeagent_memory",
                executable="memory_node",
                name="homeagent_memory",
                output="screen",
                parameters=[{"database_path": memory_db}],
            ),
            Node(
                package="homeagent_skills",
                executable="mock_skill_executor",
                name="homeagent_skill_executor",
                output="screen",
                condition=UnlessCondition(use_nav2),
            ),
            Node(
                package="homeagent_skills",
                executable="nav2_skill_executor",
                name="homeagent_skill_executor",
                output="screen",
                condition=IfCondition(use_nav2),
            ),
            Node(
                package="homeagent_orchestrator",
                executable="mock_planner",
                name="homeagent_planner",
                output="screen",
                condition=UnlessCondition(use_deepseek),
            ),
            Node(
                package="homeagent_orchestrator",
                executable="deepseek_planner",
                name="homeagent_planner",
                output="screen",
                condition=IfCondition(use_deepseek),
            ),
        ]
    )
