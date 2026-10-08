import json

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from homeagent_interfaces.msg import SafetyDecision, SkillResult

from .arm_targets import resolve_arm_target


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_JOINT = "gripper_joint"
GRIPPER_CLOSED_POSITION = 0.0


class MoveItSkillExecutor(Node):
    """Execute approved HomeArm actions through MoveIt2 and ros2_control."""

    def __init__(self) -> None:
        super().__init__("homeagent_moveit_skill_executor")
        self.declare_parameter("action_server_timeout_sec", 3.0)
        self.declare_parameter("allowed_planning_time_sec", 5.0)
        self.declare_parameter("execute_pick", True)
        self._server_timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self._planning_time = float(
            self.get_parameter("allowed_planning_time_sec").value
        )
        self._execute_pick = bool(
            self.get_parameter("execute_pick").value
        )

        self._client = ActionClient(self, MoveGroup, "/move_action")
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self._apply_scene = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self._result_pub = self.create_publisher(
            SkillResult, "/homeagent/skill_result", 10
        )
        self._approved_sub = self.create_subscription(
            SafetyDecision,
            "/homeagent/action_approved",
            self._on_approved,
            10,
        )
        self._active = {}
        self.get_logger().info("HomeAgent MoveIt2 skill executor ready")

    def _on_approved(self, decision: SafetyDecision) -> None:
        if not decision.allowed:
            return
        if decision.action == "pick" and not self._execute_pick:
            # A specialized physical pick adapter owns pick in this runtime.
            return

        try:
            proposal = json.loads(decision.proposal_json or "{}")
            resolved = resolve_arm_target(decision.action, proposal)
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self._publish_result(
                decision,
                success=False,
                code="INVALID_TRUSTED_ARM_CONTEXT",
                result={"error": str(exc)},
            )
            return

        if resolved is None:
            # Another skill adapter owns this approved action.
            return

        target_name, joints, target_source = resolved

        if not self._client.wait_for_server(timeout_sec=self._server_timeout):
            self._publish_result(
                decision,
                success=False,
                code="MOVEIT_UNAVAILABLE",
                result={"target_name": target_name},
            )
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = self._planning_time
        goal.request.max_velocity_scaling_factor = 0.35
        goal.request.max_acceleration_scaling_factor = 0.35
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = f"homeagent_{target_name}"
        for name, position in zip(JOINT_NAMES, joints):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        params = proposal.get("params") or {}
        send_future = self._client.send_goal_async(goal)
        self._active[decision.request_id] = {
            "decision": decision,
            "target_name": target_name,
            "joints": joints,
            "target_source": target_source,
            "object_name": str(params.get("object", "")),
        }
        send_future.add_done_callback(
            lambda future, rid=decision.request_id:
            self._on_goal_response(rid, future)
        )
        self.get_logger().info(
            f"MOVEIT_SEND request_id={decision.request_id} "
            f"action={decision.action} target={target_name} "
            f"source={target_source}"
        )

    def _on_goal_response(self, request_id, future) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        try:
            handle = future.result()
        except Exception as exc:
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="MOVEIT_SEND_FAILED",
                result={"error": str(exc)},
            )
            return

        if handle is None or not handle.accepted:
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="MOVEIT_GOAL_REJECTED",
                result={"target_name": state["target_name"]},
            )
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done, rid=request_id: self._on_moveit_result(rid, done)
        )

    def _on_moveit_result(self, request_id, future) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        decision = state["decision"]
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = wrapped.result
            error_code = int(result.error_code.val)
        except Exception as exc:
            self._active.pop(request_id, None)
            self._publish_result(
                decision,
                success=False,
                code="MOVEIT_RESULT_FAILED",
                result={"error": str(exc)},
            )
            return

        success = status == 4 and error_code == MoveItErrorCodes.SUCCESS
        payload = {
            "target_name": state["target_name"],
            "joint_target": state["joints"],
            "target_source": state["target_source"],
            "status": status,
            "moveit_error_code": error_code,
            "planning_time_sec": float(result.planning_time),
            "planned_points": len(
                result.planned_trajectory.joint_trajectory.points
            ),
        }

        if not success:
            self._active.pop(request_id, None)
            self._publish_result(
                decision,
                success=False,
                code="MOVEIT_FAILED",
                result=payload,
            )
            return

        if decision.action != "pick":
            self._active.pop(request_id, None)
            self._publish_result(
                decision,
                success=True,
                code="MOVEIT_SUCCEEDED",
                result=payload,
            )
            return

        state["approach_payload"] = {
            **payload,
            "manipulation_stage": "pick_approach",
        }
        self._start_gripper_close(request_id)

    def _start_gripper_close(self, request_id: str) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        if not self._gripper_client.wait_for_server(
            timeout_sec=self._server_timeout
        ):
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="GRIPPER_UNAVAILABLE",
                result={
                    **state.get("approach_payload", {}),
                    "gripper_closed": False,
                    "physical_grasp": False,
                },
            )
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_CLOSED_POSITION]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        goal.goal_time_tolerance.sec = 1

        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, rid=request_id:
            self._on_gripper_goal_response(rid, done)
        )
        self.get_logger().info(
            f"GRIPPER_CLOSE request_id={request_id} "
            f"position={GRIPPER_CLOSED_POSITION}"
        )

    def _on_gripper_goal_response(self, request_id, future) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        try:
            handle = future.result()
        except Exception as exc:
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="GRIPPER_SEND_FAILED",
                result={"error": str(exc), **state.get("approach_payload", {})},
            )
            return

        if handle is None or not handle.accepted:
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="GRIPPER_GOAL_REJECTED",
                result=state.get("approach_payload", {}),
            )
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done, rid=request_id:
            self._on_gripper_result(rid, done)
        )

    def _on_gripper_result(self, request_id, future) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        try:
            wrapped = future.result()
            status = int(wrapped.status)
            error_code = int(wrapped.result.error_code)
        except Exception as exc:
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="GRIPPER_RESULT_FAILED",
                result={"error": str(exc), **state.get("approach_payload", {})},
            )
            return

        if status != 4 or error_code != 0:
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="GRIPPER_FAILED",
                result={
                    **state.get("approach_payload", {}),
                    "gripper_status": status,
                    "gripper_error_code": error_code,
                    "gripper_closed": False,
                    "physical_grasp": False,
                },
            )
            return

        state["gripper_status"] = status
        state["gripper_error_code"] = error_code
        self._apply_logical_attachment(request_id)

    def _apply_logical_attachment(self, request_id: str) -> None:
        state = self._active.get(request_id)
        if state is None:
            return

        if not self._apply_scene.wait_for_service(
            timeout_sec=self._server_timeout
        ):
            self._active.pop(request_id, None)
            self._publish_result(
                state["decision"],
                success=False,
                code="PLANNING_SCENE_UNAVAILABLE",
                result={
                    **state.get("approach_payload", {}),
                    "gripper_closed": True,
                    "planning_scene_attached": False,
                    "physical_grasp": False,
                },
            )
            return

        object_name = state.get("object_name") or "picked_object"
        attached = self._make_attached_object(object_name)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_scene.call_async(request)
        future.add_done_callback(
            lambda done, rid=request_id:
            self._on_attach_result(rid, done)
        )
        self.get_logger().info(
            f"PLANNING_SCENE_ATTACH request_id={request_id} object={object_name}"
        )

    def _on_attach_result(self, request_id, future) -> None:
        state = self._active.pop(request_id, None)
        if state is None:
            return

        try:
            response = future.result()
            attached = bool(response.success)
        except Exception as exc:
            self._publish_result(
                state["decision"],
                success=False,
                code="PLANNING_SCENE_ATTACH_FAILED",
                result={
                    "error": str(exc),
                    **state.get("approach_payload", {}),
                    "gripper_closed": True,
                    "planning_scene_attached": False,
                    "physical_grasp": False,
                },
            )
            return

        payload = {
            **state.get("approach_payload", {}),
            "manipulation_stage": "logical_attach",
            "approach_completed": True,
            "gripper_status": state.get("gripper_status"),
            "gripper_error_code": state.get("gripper_error_code"),
            "gripper_closed": True,
            "planning_scene_attached": attached,
            "logical_attach_complete": attached,
            "physical_grasp": False,
            "grasp_complete": False,
        }

        self._publish_result(
            state["decision"],
            success=attached,
            code=(
                "MOVEIT_PICK_LOGICAL_ATTACH_SUCCEEDED"
                if attached
                else "PLANNING_SCENE_ATTACH_FAILED"
            ),
            result=payload,
        )

    @staticmethod
    def _make_attached_object(object_name: str) -> AttachedCollisionObject:
        attached = AttachedCollisionObject()
        attached.link_name = "tool_link"
        attached.touch_links = [
            "tool_link",
            "gripper_fixed_finger_link",
            "gripper_finger_link",
            "wrist_link",
        ]
        attached.weight = 0.2

        attached.object.header.frame_id = "tool_link"
        attached.object.id = object_name
        attached.object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.10, 0.035]

        pose = Pose()
        pose.position.x = 0.12
        pose.orientation.w = 1.0

        attached.object.primitives = [primitive]
        attached.object.primitive_poses = [pose]
        return attached

    def _publish_result(
        self,
        decision: SafetyDecision,
        *,
        success: bool,
        code: str,
        result: dict,
    ) -> None:
        msg = SkillResult()
        msg.request_id = decision.request_id
        msg.action = decision.action
        msg.success = success
        msg.code = code
        msg.result_json = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(msg)
        log = self.get_logger().info if success else self.get_logger().warning
        log(
            f"SKILL_RESULT request_id={msg.request_id} "
            f"action={msg.action} success={msg.success} code={msg.code}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItSkillExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
