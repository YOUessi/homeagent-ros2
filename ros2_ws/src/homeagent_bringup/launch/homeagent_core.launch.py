from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_deepseek = LaunchConfiguration("use_deepseek")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_moveit = LaunchConfiguration("use_moveit")
    moveit_execute_pick = LaunchConfiguration("moveit_execute_pick")
    use_gazebo_contact_pick = LaunchConfiguration("use_gazebo_contact_pick")
    gazebo_contact_spawn_object = LaunchConfiguration(
        "gazebo_contact_spawn_object"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    memory_db = LaunchConfiguration("memory_db")

    common_params = {"use_sim_time": use_sim_time}

    any_real_skill = PythonExpression(
        [
            "'",
            use_nav2,
            "'.lower() in ['true','1','yes'] or '",
            use_moveit,
            "'.lower() in ['true','1','yes'] or '",
            use_gazebo_contact_pick,
            "'.lower() in ['true','1','yes']",
        ]
    )

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
                description="Enable the Nav2-backed navigation skill adapter.",
            ),
            DeclareLaunchArgument(
                "use_moveit",
                default_value="false",
                description="Enable the MoveIt2-backed HomeArm skill adapter.",
            ),
            DeclareLaunchArgument(
                "moveit_execute_pick",
                default_value="true",
                description=(
                    "Let the generic MoveIt adapter own pick. Set false when "
                    "a specialized physical pick adapter owns that action."
                ),
            ),
            DeclareLaunchArgument(
                "use_gazebo_contact_pick",
                default_value="false",
                description="Enable simulation-only Gazebo contact-gated pick adapter.",
            ),
            DeclareLaunchArgument(
                "gazebo_contact_spawn_object",
                default_value="true",
                description=(
                    "Let the Gazebo contact-pick adapter spawn its demo target. "
                    "Set false when perception observes a pre-existing object."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock for HomeAgent nodes.",
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
                parameters=[common_params],
            ),
            Node(
                package="homeagent_context",
                executable="context_node",
                name="homeagent_context",
                output="screen",
                parameters=[common_params],
            ),
            Node(
                package="homeagent_memory",
                executable="memory_node",
                name="homeagent_memory",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "database_path": memory_db,
                    }
                ],
            ),
            Node(
                package="homeagent_skills",
                executable="mock_skill_executor",
                name="homeagent_mock_skill_executor",
                output="screen",
                parameters=[common_params],
                condition=UnlessCondition(any_real_skill),
            ),
            Node(
                package="homeagent_skills",
                executable="nav2_skill_executor",
                name="homeagent_nav2_skill_executor",
                output="screen",
                parameters=[common_params],
                condition=IfCondition(use_nav2),
            ),
            Node(
                package="homeagent_skills",
                executable="moveit_skill_executor",
                name="homeagent_moveit_skill_executor",
                output="screen",
                parameters=[
                    common_params,
                    {"execute_pick": moveit_execute_pick},
                ],
                condition=IfCondition(use_moveit),
            ),
            Node(
                package="homeagent_skills",
                executable="gazebo_contact_pick_executor",
                name="homeagent_gazebo_contact_pick_executor",
                output="screen",
                parameters=[
                    common_params,
                    {
                        "spawn_contact_object":
                            gazebo_contact_spawn_object,
                    },
                ],
                condition=IfCondition(use_gazebo_contact_pick),
            ),
            Node(
                package="homeagent_orchestrator",
                executable="mock_planner",
                name="homeagent_planner",
                output="screen",
                parameters=[common_params],
                condition=UnlessCondition(use_deepseek),
            ),
            Node(
                package="homeagent_orchestrator",
                executable="deepseek_planner",
                name="homeagent_planner",
                output="screen",
                parameters=[common_params],
                condition=IfCondition(use_deepseek),
            ),
        ]
    )
