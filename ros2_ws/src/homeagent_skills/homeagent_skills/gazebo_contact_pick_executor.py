import json
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from geometry_msgs.msg import Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from homeagent_interfaces.msg import SafetyDecision, SkillResult
from .arm_targets import resolve_arm_target


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_JOINT = "gripper_joint"
GRASP_PROBE_TARGET = [0.20, -0.66, 1.06, -0.35]
CARRY_TARGET = [0.0, -0.30, 0.65, -0.35]
TOOL_FRAME = "homebot_arm::tool_link"
SIM_OBJECT = "physical_cup"
BASE_MODEL = "homebot_arm"

CUP_SDF = """<sdf version='1.6'><model name='physical_cup'><link name='link'>
<gravity>true</gravity><self_collide>true</self_collide>
<inertial><mass>0.05</mass><inertia>
<ixx>0.000032</ixx><iyy>0.000032</iyy><izz>0.000011</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='collision'><geometry><cylinder>
<radius>0.021</radius><length>0.08</length></cylinder></geometry>
<surface><friction><ode><mu>40</mu><mu2>40</mu2></ode></friction></surface>
</collision><visual name='visual'><geometry><cylinder>
<radius>0.021</radius><length>0.08</length></cylinder></geometry></visual>
</link></model></sdf>"""


def _xyz(pose):
    return [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]


def _distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _pose_yaw(pose):
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _clamp(value, low, high):
    return max(low, min(high, value))


class GazeboContactPickExecutor(Node):
    """Simulation-only pick adapter gated by Safety and bilateral Gazebo contact."""

    def __init__(self):
        super().__init__("homeagent_gazebo_contact_pick_executor")
        self.declare_parameter("action_server_timeout_sec", 4.0)
        self.declare_parameter("spawn_contact_object", True)
        self.declare_parameter("local_alignment_enabled", True)
        self.declare_parameter("alignment_timeout_sec", 20.0)
        self.declare_parameter("alignment_position_tolerance_m", 0.005)
        self.declare_parameter("alignment_yaw_tolerance_rad", 0.004)
        self._timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self._spawn_contact_object = bool(
            self.get_parameter("spawn_contact_object").value
        )
        self._local_alignment_enabled = bool(
            self.get_parameter("local_alignment_enabled").value
        )
        self._alignment_timeout = float(
            self.get_parameter("alignment_timeout_sec").value
        )
        self._alignment_position_tolerance = float(
            self.get_parameter("alignment_position_tolerance_m").value
        )
        self._alignment_yaw_tolerance = float(
            self.get_parameter("alignment_yaw_tolerance_rad").value
        )
        self._move = ActionClient(self, MoveGroup, "/move_action")
        self._gripper = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self._spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self._state = self.create_client(
            GetEntityState, "/gazebo/get_entity_state"
        )
        self._result_pub = self.create_publisher(
            SkillResult, "/homeagent/skill_result", 10
        )
        self._alignment_cmd_pub = self.create_publisher(
            Twist, "/cmd_vel_nav", 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            SafetyDecision,
            "/homeagent/action_approved",
            self._on_approved,
            10,
        )
        self._active = {}
        self._delay_timers = {}
        self._alignment_timers = {}
        self._alignment_query_pending = set()
        self.get_logger().info(
            "HomeAgent Gazebo contact pick executor ready "
            f"spawn_contact_object={self._spawn_contact_object}"
        )

    def _on_approved(self, decision):
        if not decision.allowed or decision.action != "pick":
            return
        try:
            proposal = json.loads(decision.proposal_json or "{}")
            resolved = resolve_arm_target("pick", proposal)
            if resolved is None:
                raise ValueError("pick target unresolved")
            target_name, joints, source = resolved
            obj = str((proposal.get("params") or {}).get("object", ""))
            if obj != "cup":
                raise ValueError("physical demo currently supports cup only")
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self._publish(
                decision, False, "INVALID_PHYSICAL_PICK_CONTEXT",
                {"error": str(exc)}
            )
            return

        if not self._ready():
            self._publish(
                decision, False, "GAZEBO_PICK_RUNTIME_UNAVAILABLE",
                {"target_name": target_name}
            )
            return

        proposal_context = proposal.get("context") or {}
        self._active[decision.request_id] = {
            "decision": decision,
            "target_name": target_name,
            "approach": list(joints),
            "target_source": source,
            "object": obj,
            "object_observed_pose": proposal_context.get(
                "object_observed_pose"
            ),
            "arm_pregrasp": proposal_context.get(
                "resolved_arm_pregrasp_joint_target"
            ),
            "mobile_pregrasp_pose": proposal_context.get(
                "resolved_mobile_pregrasp_pose"
            ),
            "sim_world_pregrasp_pose": proposal_context.get(
                "resolved_sim_world_pregrasp_pose"
            ),
            "spawn_contact_object": self._spawn_contact_object,
        }

        if self._spawn_contact_object:
            self._send_arm(
                decision.request_id, joints, "trusted_pick_approach",
                self._on_approach_result
            )
            return

        existing = self._query("world")
        existing.add_done_callback(
            lambda done, rid=decision.request_id, target=joints:
            self._on_existing_target(rid, target, done)
        )

    def _ready(self):
        spawn_ready = True
        if self._spawn_contact_object:
            spawn_ready = self._spawn.wait_for_service(
                timeout_sec=self._timeout
            )
        return (
            self._move.wait_for_server(timeout_sec=self._timeout)
            and self._gripper.wait_for_server(timeout_sec=self._timeout)
            and spawn_ready
            and self._state.wait_for_service(timeout_sec=self._timeout)
        )

    def _on_existing_target(self, rid, joints, future):
        state = self._active.get(rid)
        if state is None:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._fail(rid, "GAZEBO_TARGET_QUERY_FAILED", str(exc))
            return

        if response is None or not response.success:
            self._fail(
                rid,
                "GAZEBO_TARGET_NOT_PRESENT",
                f"{SIM_OBJECT} does not exist in Gazebo",
            )
            return

        state["existing_object_before_pick"] = True
        self.get_logger().info(
            f"GAZEBO_EXISTING_TARGET request_id={rid} object={SIM_OBJECT}"
        )

        if self._local_alignment_enabled:
            target = state.get("sim_world_pregrasp_pose")
            arm_pregrasp = state.get("arm_pregrasp")
            if not isinstance(target, list) or len(target) != 3:
                self._fail(
                    rid,
                    "LOCAL_ALIGNMENT_CONTEXT_MISSING",
                    "trusted Gazebo-world pregrasp pose is required",
                )
                return
            if not isinstance(arm_pregrasp, list) or len(arm_pregrasp) != 4:
                self._fail(
                    rid,
                    "ARM_PREGRASP_CONTEXT_MISSING",
                    "trusted arm pregrasp target is required",
                )
                return
            try:
                state["alignment_target"] = [
                    float(target[0]),
                    float(target[1]),
                    float(target[2]),
                ]
                state["arm_pregrasp"] = [
                    float(value) for value in arm_pregrasp
                ]
            except (TypeError, ValueError):
                self._fail(
                    rid,
                    "LOCAL_ALIGNMENT_CONTEXT_INVALID",
                    "mobile pregrasp pose must be numeric",
                )
                return

            state["alignment_arm_target"] = list(
                state["arm_pregrasp"]
            )
            state["alignment_started_at"] = time.monotonic()
            state["alignment_initial_pose"] = None
            timer = self.create_timer(
                0.10,
                lambda request_id=rid: self._alignment_tick(request_id),
            )
            self._alignment_timers[rid] = timer
            self.get_logger().info(
                f"LOCAL_ALIGN_START request_id={rid} "
                f"target={state['alignment_target']}"
            )
            return

        self._send_arm(
            rid,
            joints,
            "trusted_pick_approach",
            self._on_approach_result,
        )

    def _alignment_tick(self, rid):
        state = self._active.get(rid)
        if state is None:
            self._stop_alignment(rid)
            return
        if rid in self._alignment_query_pending:
            return
        if (
            time.monotonic() - state["alignment_started_at"]
            > self._alignment_timeout
        ):
            self._stop_alignment(rid)
            self._fail(
                rid,
                "LOCAL_ALIGNMENT_TIMEOUT",
                "pregrasp base alignment timed out",
            )
            return

        request = GetEntityState.Request()
        request.name = BASE_MODEL
        request.reference_frame = "world"
        self._alignment_query_pending.add(rid)
        future = self._state.call_async(request)
        future.add_done_callback(
            lambda done, request_id=rid:
            self._on_alignment_world_state(request_id, done)
        )

    def _on_alignment_world_state(self, rid, future):
        self._alignment_query_pending.discard(rid)
        state = self._active.get(rid)
        if state is None:
            return

        try:
            response = future.result()
        except Exception as exc:
            self._stop_alignment(rid)
            self._fail(rid, "LOCAL_ALIGNMENT_STATE_FAILED", str(exc))
            return

        if response is None or not response.success:
            self._stop_alignment(rid)
            self._fail(
                rid,
                "LOCAL_ALIGNMENT_STATE_FAILED",
                "Gazebo base world pose unavailable",
            )
            return

        pose = response.state.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = _pose_yaw(pose)
        target_x, target_y, target_yaw = state["alignment_target"]

        if state["alignment_initial_pose"] is None:
            state["alignment_initial_pose"] = [x, y, yaw]

        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        yaw_error = _wrap_angle(target_yaw - yaw)

        command = Twist()

        if distance > self._alignment_position_tolerance:
            heading_error = _wrap_angle(math.atan2(dy, dx) - yaw)
            direction = 1.0
            if abs(heading_error) > math.pi / 2.0:
                direction = -1.0
                heading_error = _wrap_angle(
                    heading_error - math.copysign(math.pi, heading_error)
                )

            if abs(heading_error) > 0.06:
                command.angular.z = _clamp(
                    1.4 * heading_error, -0.55, 0.55
                )
            else:
                speed = _clamp(0.7 * distance, 0.003, 0.04)
                command.linear.x = direction * speed
                command.angular.z = _clamp(
                    1.2 * heading_error, -0.22, 0.22
                )
            self._alignment_cmd_pub.publish(command)
            return

        if abs(yaw_error) > self._alignment_yaw_tolerance:
            command.angular.z = _clamp(
                1.4 * yaw_error, -0.45, 0.45
            )
            self._alignment_cmd_pub.publish(command)
            return

        self._alignment_cmd_pub.publish(Twist())
        self._stop_alignment(rid)
        state["local_alignment"] = {
            "frame": "gazebo_world",
            "measurement_source": "gazebo_get_entity_state",
            "initial_pose": state["alignment_initial_pose"],
            "final_pose": [x, y, yaw],
            "target_pose": state["alignment_target"],
            "position_error_m": distance,
            "yaw_error_rad": abs(yaw_error),
        }
        self.get_logger().info(
            f"LOCAL_ALIGN_DONE request_id={rid} "
            f"frame=gazebo_world position_error={distance:.4f} "
            f"yaw_error={abs(yaw_error):.4f}"
        )
        self._send_arm(
            rid,
            state["alignment_arm_target"],
            "trusted_pick_pregrasp",
            self._on_pregrasp_result,
            velocity_scale=0.20,
        )

    def _stop_alignment(self, rid):
        timer = self._alignment_timers.pop(rid, None)
        if timer is not None:
            timer.cancel()
        self._alignment_query_pending.discard(rid)
        self._alignment_cmd_pub.publish(Twist())

    def _arm_goal(self, joints, name, velocity_scale=0.20):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        scale = float(velocity_scale)
        goal.request.max_velocity_scaling_factor = scale
        goal.request.max_acceleration_scaling_factor = scale
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = name
        for joint_name, position in zip(ARM_JOINTS, joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _send_arm(
        self,
        rid,
        joints,
        label,
        result_cb,
        velocity_scale=0.20,
    ):
        future = self._move.send_goal_async(
            self._arm_goal(joints, label, velocity_scale)
        )
        future.add_done_callback(
            lambda done: self._on_arm_goal(rid, done, result_cb)
        )

    def _on_arm_goal(self, rid, future, result_cb):
        state = self._active.get(rid)
        if state is None:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self._fail(rid, "MOVEIT_SEND_FAILED", str(exc))
            return
        if handle is None or not handle.accepted:
            self._fail(rid, "MOVEIT_GOAL_REJECTED", "goal rejected")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done: result_cb(rid, done)
        )

    def _moveit_success(self, future):
        wrapped = future.result()
        if wrapped is None:
            return False, None
        ok = (
            int(wrapped.status) == 4
            and int(wrapped.result.error_code.val)
            == MoveItErrorCodes.SUCCESS
        )
        return ok, wrapped

    def _on_pregrasp_result(self, rid, future):
        ok, wrapped = self._moveit_success(future)
        if not ok:
            self._fail(rid, "MOVEIT_PREGRASP_FAILED", "pregrasp failed")
            return

        state = self._active[rid]
        state["pregrasp_points"] = len(
            wrapped.result.planned_trajectory.joint_trajectory.points
        )
        self.get_logger().info(
            f"GAZEBO_PICK_PREGRASP request_id={rid} "
            f"source={state['target_source']}"
        )

        self._send_arm(
            rid,
            state["approach"],
            "trusted_pick_final_approach",
            self._on_approach_result,
            velocity_scale=0.05,
        )

    def _on_approach_result(self, rid, future):
        ok, wrapped = self._moveit_success(future)
        if not ok:
            self._fail(rid, "MOVEIT_APPROACH_FAILED", "approach failed")
            return
        state = self._active[rid]
        state["approach_points"] = len(
            wrapped.result.planned_trajectory.joint_trajectory.points
        )
        self.get_logger().info(
            f"GAZEBO_PICK_APPROACH request_id={rid} "
            f"source={state['target_source']}"
        )
        self._close_gripper(rid)

    def _close_gripper(self, rid):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [0.0]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        goal.goal_time_tolerance.sec = 2
        future = self._gripper.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._on_gripper_goal(rid, done)
        )

    def _on_gripper_goal(self, rid, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._fail(rid, "GRIPPER_SEND_FAILED", str(exc))
            return
        if handle is None or not handle.accepted:
            self._fail(rid, "GRIPPER_GOAL_REJECTED", "goal rejected")
            return
        rf = handle.get_result_async()
        rf.add_done_callback(lambda done: self._on_gripper_result(rid, done))

    def _on_gripper_result(self, rid, future):
        wrapped = future.result()
        if wrapped is None or int(wrapped.status) != 4:
            self._fail(rid, "GRIPPER_FAILED", "close failed")
            return

        if not self._spawn_contact_object:
            self.get_logger().info(
                f"GAZEBO_CONTACT_EXISTING_OBJECT request_id={rid} "
                f"object={SIM_OBJECT}"
            )
            self._delay(rid, 0.4, lambda: self._query_before_world(rid))
            return

        request = SpawnEntity.Request()
        request.name = SIM_OBJECT
        request.xml = CUP_SDF
        request.reference_frame = TOOL_FRAME
        request.initial_pose.position.x = 0.12
        request.initial_pose.position.y = 0.0015
        request.initial_pose.orientation.w = 1.0
        sf = self._spawn.call_async(request)
        sf.add_done_callback(lambda done: self._on_spawn(rid, done))

    def _on_spawn(self, rid, future):
        response = future.result()
        if response is None or not response.success:
            self._fail(rid, "GAZEBO_OBJECT_SPAWN_FAILED", "spawn failed")
            return
        self.get_logger().info(
            f"GAZEBO_CONTACT_OBJECT request_id={rid} object={SIM_OBJECT}"
        )
        self._delay(rid, 0.6, lambda: self._query_before_world(rid))

    def _query(self, ref):
        request = GetEntityState.Request()
        request.name = SIM_OBJECT
        request.reference_frame = ref
        return self._state.call_async(request)

    def _query_before_world(self, rid):
        f = self._query("world")
        f.add_done_callback(lambda done: self._on_before_world(rid, done))

    def _on_before_world(self, rid, future):
        response = future.result()
        if response is None or not response.success:
            self._fail(rid, "GAZEBO_STATE_FAILED", "world state unavailable")
            return
        self._active[rid]["before_world"] = _xyz(response.state.pose)
        f = self._query(TOOL_FRAME)
        f.add_done_callback(lambda done: self._on_before_tool(rid, done))

    def _on_before_tool(self, rid, future):
        response = future.result()
        if response is None or not response.success:
            self._fail(rid, "GAZEBO_STATE_FAILED", "tool state unavailable")
            return
        self._active[rid]["before_tool"] = _xyz(response.state.pose)

        # Never execute the large carry motion until a small proof motion
        # confirms that the cup is physically following the gripper.
        self._send_arm(
            rid,
            GRASP_PROBE_TARGET,
            "contact_grasp_probe",
            self._on_probe_result,
            velocity_scale=0.12,
        )

    def _on_probe_result(self, rid, future):
        ok, wrapped = self._moveit_success(future)
        if not ok:
            self._abort_unconfirmed_grasp(
                rid,
                "GRASP_PROBE_MOVE_FAILED",
                {"error": "small grasp proof motion failed"},
            )
            return

        state = self._active.get(rid)
        if state is None:
            return
        state["probe_points"] = len(
            wrapped.result.planned_trajectory.joint_trajectory.points
        )
        self._delay(rid, 0.35, lambda: self._query_probe_world(rid))

    def _query_probe_world(self, rid):
        future = self._query("world")
        future.add_done_callback(
            lambda done: self._on_probe_world(rid, done)
        )

    def _on_probe_world(self, rid, future):
        state = self._active.get(rid)
        if state is None:
            return
        response = future.result()
        if response is None or not response.success:
            self._abort_unconfirmed_grasp(
                rid,
                "GRASP_PROBE_STATE_FAILED",
                {"error": "probe world state unavailable"},
            )
            return
        state["probe_after_world"] = _xyz(response.state.pose)
        future = self._query(TOOL_FRAME)
        future.add_done_callback(
            lambda done: self._on_probe_tool(rid, done)
        )

    def _on_probe_tool(self, rid, future):
        state = self._active.get(rid)
        if state is None:
            return
        response = future.result()
        if response is None or not response.success:
            self._abort_unconfirmed_grasp(
                rid,
                "GRASP_PROBE_STATE_FAILED",
                {"error": "probe tool-relative state unavailable"},
            )
            return

        probe_after_tool = _xyz(response.state.pose)
        probe_world_motion = _distance(
            state["before_world"], state["probe_after_world"]
        )
        probe_relative_drift = _distance(
            state["before_tool"], probe_after_tool
        )
        confirmed = (
            probe_world_motion > 0.008
            and probe_relative_drift < 0.015
        )

        state["grasp_probe"] = {
            "target": GRASP_PROBE_TARGET,
            "planned_points": state.get("probe_points"),
            "cup_world_motion_m": probe_world_motion,
            "cup_relative_tool_drift_m": probe_relative_drift,
            "confirmed": confirmed,
        }

        self.get_logger().info(
            f"GRASP_PROBE request_id={rid} confirmed={confirmed} "
            f"world_motion={probe_world_motion:.4f} "
            f"relative_drift={probe_relative_drift:.4f}"
        )

        if not confirmed:
            self._abort_unconfirmed_grasp(
                rid,
                "GAZEBO_GRASP_NOT_CONFIRMED",
                {
                    "grasp_probe": state["grasp_probe"],
                    "carry_executed": False,
                },
            )
            return

        # Reset carry baselines after the successful proof motion.
        state["before_world"] = state["probe_after_world"]
        state["before_tool"] = probe_after_tool
        state["grasp_confirmed_before_carry"] = True
        self._send_arm(
            rid,
            CARRY_TARGET,
            "contact_gated_carry",
            self._on_carry_result,
            velocity_scale=0.16,
        )

    def _on_carry_result(self, rid, future):
        ok, wrapped = self._moveit_success(future)
        if not ok:
            self._fail(rid, "MOVEIT_CARRY_FAILED", "carry failed")
            return
        self._active[rid]["carry_points"] = len(
            wrapped.result.planned_trajectory.joint_trajectory.points
        )
        self._delay(rid, 0.6, lambda: self._query_after_world(rid))

    def _query_after_world(self, rid):
        f = self._query("world")
        f.add_done_callback(lambda done: self._on_after_world(rid, done))

    def _on_after_world(self, rid, future):
        response = future.result()
        if response is None or not response.success:
            self._fail(rid, "GAZEBO_STATE_FAILED", "final world state unavailable")
            return
        self._active[rid]["after_world"] = _xyz(response.state.pose)
        f = self._query(TOOL_FRAME)
        f.add_done_callback(lambda done: self._finish(rid, done))

    def _finish(self, rid, future):
        state = self._active.pop(rid, None)
        if state is None:
            return
        response = future.result()
        if response is None or not response.success:
            self._publish(
                state["decision"], False, "GAZEBO_STATE_FAILED",
                {"error": "final tool state unavailable"}
            )
            return
        after_tool = _xyz(response.state.pose)
        world_motion = _distance(
            state["before_world"], state["after_world"]
        )
        relative_drift = _distance(state["before_tool"], after_tool)
        held = world_motion > 0.08 and relative_drift < 0.035

        payload = {
            "target_name": state["target_name"],
            "target_source": state["target_source"],
            "pregrasp_joint_target": state.get("arm_pregrasp"),
            "pregrasp_points": state.get("pregrasp_points"),
            "approach_joint_target": state["approach"],
            "approach_points": state["approach_points"],
            "grasp_probe": state.get("grasp_probe"),
            "grasp_confirmed_before_carry": bool(
                state.get("grasp_confirmed_before_carry")
            ),
            "carry_joint_target": CARRY_TARGET,
            "carry_points": state["carry_points"],
            "planning_scene_attach_used": False,
            "physics_constraint_attach_used": True,
            "contact_required": True,
            "friction_only_grasp": False,
            "spawn_contact_object": state["spawn_contact_object"],
            "object_preexisted": bool(
                state.get("existing_object_before_pick")
            ),
            "perception_observed_target": (
                state.get("object_observed_pose") is not None
            ),
            "object_observed_pose": state.get("object_observed_pose"),
            "local_alignment_used": state.get("local_alignment") is not None,
            "local_alignment": state.get("local_alignment"),
            "autonomous_table_pick": False,
            "cup_before_carry_world": state["before_world"],
            "cup_after_carry_world": state["after_world"],
            "cup_before_carry_relative_tool": state["before_tool"],
            "cup_after_carry_relative_tool": after_tool,
            "cup_world_motion_m": world_motion,
            "cup_relative_tool_drift_m": relative_drift,
            "contact_gated_physical_hold": held,
        }
        self._publish(
            state["decision"],
            held,
            "GAZEBO_CONTACT_PICK_SUCCEEDED" if held
            else "GAZEBO_CONTACT_PICK_FAILED",
            payload,
        )

    def _open_gripper_best_effort(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [0.04]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        goal.goal_time_tolerance.sec = 2
        try:
            self._gripper.send_goal_async(goal)
        except Exception:
            pass

    def _abort_unconfirmed_grasp(self, rid, code, evidence):
        self._stop_alignment(rid)
        timer = self._delay_timers.pop(rid, None)
        if timer is not None:
            timer.cancel()

        state = self._active.pop(rid, None)
        self._open_gripper_best_effort()
        if state is None:
            return

        payload = {
            "target_name": state["target_name"],
            "target_source": state["target_source"],
            "planning_scene_attach_used": False,
            "physics_constraint_attach_used": False,
            "contact_required": True,
            "grasp_confirmed_before_carry": False,
            "carry_executed": False,
            "grasp_probe": state.get("grasp_probe"),
            "local_alignment": state.get("local_alignment"),
        }
        payload.update(evidence)
        self.get_logger().warning(
            f"GRASP_ABORT request_id={rid} code={code} "
            "carry_executed=false"
        )
        self._publish(
            state["decision"],
            False,
            code,
            payload,
        )

    def _delay(self, rid, seconds, callback):
        def fire():
            timer = self._delay_timers.pop(rid, None)
            if timer is not None:
                timer.cancel()
            callback()
        self._delay_timers[rid] = self.create_timer(seconds, fire)

    def _fail(self, rid, code, error):
        self._stop_alignment(rid)
        state = self._active.pop(rid, None)
        if state is not None:
            self._publish(
                state["decision"], False, code, {"error": error}
            )

    def _publish(self, decision, success, code, result):
        msg = SkillResult()
        msg.request_id = decision.request_id
        msg.action = decision.action
        msg.success = bool(success)
        msg.code = code
        msg.result_json = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(msg)
        log = self.get_logger().info if success else self.get_logger().warning
        log(
            f"SKILL_RESULT request_id={msg.request_id} "
            f"action={msg.action} success={msg.success} code={msg.code}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GazeboContactPickExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
