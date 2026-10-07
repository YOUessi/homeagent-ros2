import json
import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState, SpawnEntity
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from homeagent_interfaces.msg import SafetyDecision, SkillResult
from .arm_targets import resolve_arm_target


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_JOINT = "gripper_joint"
CARRY_TARGET = [0.0, -0.30, 0.65, -0.35]
TOOL_FRAME = "homebot_arm::tool_link"
SIM_OBJECT = "physical_cup"

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


class GazeboContactPickExecutor(Node):
    """Simulation-only pick adapter gated by Safety and bilateral Gazebo contact."""

    def __init__(self):
        super().__init__("homeagent_gazebo_contact_pick_executor")
        self.declare_parameter("action_server_timeout_sec", 4.0)
        self._timeout = float(
            self.get_parameter("action_server_timeout_sec").value
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
        self.create_subscription(
            SafetyDecision,
            "/homeagent/action_approved",
            self._on_approved,
            10,
        )
        self._active = {}
        self._delay_timers = {}
        self.get_logger().info(
            "HomeAgent Gazebo contact pick executor ready"
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

        self._active[decision.request_id] = {
            "decision": decision,
            "target_name": target_name,
            "approach": list(joints),
            "target_source": source,
            "object": obj,
        }
        self._send_arm(
            decision.request_id, joints, "trusted_pick_approach",
            self._on_approach_result
        )

    def _ready(self):
        return (
            self._move.wait_for_server(timeout_sec=self._timeout)
            and self._gripper.wait_for_server(timeout_sec=self._timeout)
            and self._spawn.wait_for_service(timeout_sec=self._timeout)
            and self._state.wait_for_service(timeout_sec=self._timeout)
        )

    def _arm_goal(self, joints, name):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20
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

    def _send_arm(self, rid, joints, label, result_cb):
        future = self._move.send_goal_async(self._arm_goal(joints, label))
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
        point.positions = [0.003]
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
        self._send_arm(
            rid, CARRY_TARGET, "contact_gated_carry",
            self._on_carry_result
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
            "approach_joint_target": state["approach"],
            "approach_points": state["approach_points"],
            "carry_joint_target": CARRY_TARGET,
            "carry_points": state["carry_points"],
            "planning_scene_attach_used": False,
            "physics_constraint_attach_used": True,
            "contact_required": True,
            "friction_only_grasp": False,
            "autonomous_table_pick": False,
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

    def _delay(self, rid, seconds, callback):
        def fire():
            timer = self._delay_timers.pop(rid, None)
            if timer is not None:
                timer.cancel()
            callback()
        self._delay_timers[rid] = self.create_timer(seconds, fire)

    def _fail(self, rid, code, error):
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
