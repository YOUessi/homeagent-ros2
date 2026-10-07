#!/usr/bin/env python3
import json
import math
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from homeagent_skills.arm_targets import NAMED_ARM_TARGETS


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


class MoveItProbe(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_moveit_probe")
        self.positions = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def _on_joint_state(self, msg: JointState) -> None:
        for name, value in zip(msg.name, msg.position):
            self.positions[name] = float(value)

    def wait_for_joint_state(self, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(name in self.positions for name in JOINT_NAMES):
                return
        raise RuntimeError("timed out waiting for HomeArm /joint_states")

    def current(self):
        return [self.positions[name] for name in JOINT_NAMES]

    def send_joint_goal(self, target, timeout: float = 30.0):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/move_action unavailable")

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.35
        goal.request.max_acceleration_scaling_factor = 0.35
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = "homearm_joint_goal"
        for name, position in zip(JOINT_NAMES, target):
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

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            raise RuntimeError("MoveGroup goal send timed out")

        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveGroup goal rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            raise RuntimeError("MoveGroup result timed out")

        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("MoveGroup returned no result")
        return int(wrapped.status), wrapped.result


def main() -> int:
    rclpy.init()
    node = MoveItProbe()
    try:
        node.wait_for_joint_state()
        initial = node.current()
        target = NAMED_ARM_TARGETS["inspect"]

        status, result = node.send_joint_goal(target)
        # Give joint-state broadcaster time to publish the settled state.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        final = node.current()

        errors = [abs(a - b) for a, b in zip(final, target)]
        movement = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(final, initial))
        )
        max_error = max(errors)

        report = {
            "group": "arm",
            "target_name": "inspect",
            "joint_names": JOINT_NAMES,
            "initial": initial,
            "target": target,
            "final": final,
            "action_status": status,
            "moveit_error_code": int(result.error_code.val),
            "planning_time_sec": float(result.planning_time),
            "planned_points": len(result.planned_trajectory.joint_trajectory.points),
            "executed_points": len(result.executed_trajectory.joint_trajectory.points),
            "movement_l2_rad": movement,
            "max_joint_error_rad": max_error,
            "passed": (
                status == 4
                and int(result.error_code.val) == MoveItErrorCodes.SUCCESS
                and movement > 0.25
                and max_error < 0.05
            ),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
