#!/usr/bin/env python3
import json
import math
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from homeagent_skills.arm_targets import NAMED_ARM_TARGETS


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


class PhysicsArmProbe(Node):
    def __init__(self):
        super().__init__("homeagent_gazebo_homearm_probe")
        self.positions = {}
        self.odom = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 20
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def _on_joint_state(self, msg):
        for name, value in zip(msg.name, msg.position):
            self.positions[name] = float(value)

    def _on_odom(self, msg):
        self.odom = msg

    def wait_ready(self, timeout=18.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.odom is not None
                and all(name in self.positions for name in JOINT_NAMES)
                and self.client.server_is_ready()
            ):
                return
        raise RuntimeError(
            "timed out waiting for Gazebo base + HomeArm joint states + MoveIt"
        )

    def current(self):
        return [self.positions[name] for name in JOINT_NAMES]

    def send_goal(self, target):
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/move_action unavailable")

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = "gazebo_homearm_inspect"
        for name, position in zip(JOINT_NAMES, target):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("MoveGroup goal response timeout")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("MoveGroup goal rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=35.0)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("MoveGroup execution timeout")
        return result_future.result()


def main():
    rclpy.init()
    node = PhysicsArmProbe()
    try:
        node.wait_ready()
        initial = node.current()
        base0 = node.odom.pose.pose.position
        base_start = [float(base0.x), float(base0.y)]

        target = NAMED_ARM_TARGETS["inspect"]
        wrapped = node.send_goal(target)

        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        final = node.current()
        base1 = node.odom.pose.pose.position
        base_final = [float(base1.x), float(base1.y)]

        errors = [abs(a - b) for a, b in zip(final, target)]
        movement = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(final, initial))
        )
        base_drift = math.hypot(
            base_final[0] - base_start[0],
            base_final[1] - base_start[1],
        )

        report = {
            "hardware": "gazebo_ros2_control/GazeboSystem",
            "group": "arm",
            "target_name": "inspect",
            "initial_arm": initial,
            "target_arm": target,
            "final_arm": final,
            "moveit_status": int(wrapped.status),
            "moveit_error_code": int(wrapped.result.error_code.val),
            "planned_points": len(
                wrapped.result.planned_trajectory.joint_trajectory.points
            ),
            "arm_movement_l2_rad": movement,
            "max_joint_error_rad": max(errors),
            "base_start_xy": base_start,
            "base_final_xy": base_final,
            "base_drift_m": base_drift,
            "passed": (
                int(wrapped.status) == 4
                and int(wrapped.result.error_code.val)
                == MoveItErrorCodes.SUCCESS
                and movement > 0.25
                and max(errors) < 0.08
            ),
        }
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
