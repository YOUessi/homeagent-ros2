import json

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node

from homeagent_interfaces.msg import SafetyDecision, SkillResult

from .arm_targets import target_for_action


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


class MoveItSkillExecutor(Node):
    """Execute approved HomeArm actions through MoveIt2."""

    def __init__(self) -> None:
        super().__init__("homeagent_moveit_skill_executor")
        self.declare_parameter("action_server_timeout_sec", 3.0)
        self.declare_parameter("allowed_planning_time_sec", 5.0)
        self._server_timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self._planning_time = float(
            self.get_parameter("allowed_planning_time_sec").value
        )

        self._client = ActionClient(self, MoveGroup, "/move_action")
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

        resolved = target_for_action(decision.action)
        if resolved is None:
            # Another skill adapter owns this approved action.
            return

        target_name, joints = resolved

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

        send_future = self._client.send_goal_async(goal)
        self._active[decision.request_id] = {
            "decision": decision,
            "target_name": target_name,
            "joints": joints,
        }
        send_future.add_done_callback(
            lambda future, rid=decision.request_id:
            self._on_goal_response(rid, future)
        )
        self.get_logger().info(
            f"MOVEIT_SEND request_id={decision.request_id} "
            f"action={decision.action} target={target_name}"
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
            lambda done, rid=request_id: self._on_result(rid, done)
        )

    def _on_result(self, request_id, future) -> None:
        state = self._active.pop(request_id, None)
        if state is None:
            return

        decision = state["decision"]
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = wrapped.result
            error_code = int(result.error_code.val)
        except Exception as exc:
            self._publish_result(
                decision,
                success=False,
                code="MOVEIT_RESULT_FAILED",
                result={"error": str(exc)},
            )
            return

        success = status == 4 and error_code == MoveItErrorCodes.SUCCESS
        self._publish_result(
            decision,
            success=success,
            code="MOVEIT_SUCCEEDED" if success else "MOVEIT_FAILED",
            result={
                "target_name": state["target_name"],
                "joint_target": state["joints"],
                "status": status,
                "moveit_error_code": error_code,
                "planning_time_sec": float(result.planning_time),
                "planned_points": len(
                    result.planned_trajectory.joint_trajectory.points
                ),
            },
        )

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
