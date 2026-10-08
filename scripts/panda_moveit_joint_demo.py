#!/usr/bin/env python3
"""Real MoveIt 2 + ROS2 controller joint motion on the official Panda CAD model.

This currently uses mock_components hardware. It is a joint-space planning /
controller demo, not Gazebo contact dynamics.
"""
import json
import math
import time

import rclpy
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
INSPECT = [0.40, -0.65, 0.18, -2.20, 0.08, 1.82, 0.53]


class PandaProbe(Node):
    def __init__(self):
        super().__init__("homeagent_panda_motion_probe")
        self.latest = {}
        self.create_subscription(JointState, "/joint_states", self.on_state, 20)
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.gripper = ActionClient(
            self, GripperCommand, "/panda_hand_controller/gripper_cmd"
        )

    def on_state(self, msg):
        for name, value in zip(msg.name, msg.position):
            self.latest[name] = float(value)

    def wait_ready(self):
        end = time.monotonic() + 20.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                all(name in self.latest for name in JOINTS)
                and self.move.server_is_ready()
                and self.gripper.server_is_ready()
            ):
                return
        raise RuntimeError("Panda joint states / MoveIt / hand controller not ready")

    def current(self):
        return [self.latest[name] for name in JOINTS]

    def move_to(self, target, name):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = "panda_arm"
        req.pipeline_id = "ompl"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 8.0
        req.max_velocity_scaling_factor = 0.25
        req.max_acceleration_scaling_factor = 0.25
        req.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = name
        for joint, position in zip(JOINTS, target):
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = float(position)
            jc.tolerance_above = 0.015
            jc.tolerance_below = 0.015
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]

        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        sf = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=12.0)
        if not sf.done() or sf.result() is None or not sf.result().accepted:
            raise RuntimeError(name + ": MoveGroup goal not accepted")

        rf = sf.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=55.0)
        if not rf.done() or rf.result() is None:
            raise RuntimeError(name + ": MoveGroup timed out")
        wrapped = rf.result()
        error = int(wrapped.result.error_code.val)
        return {
            "name": name,
            "status": int(wrapped.status),
            "moveit_error_code": error,
            "planned_points": len(
                wrapped.result.planned_trajectory.joint_trajectory.points
            ),
            "success": int(wrapped.status) == 4
            and error == MoveItErrorCodes.SUCCESS,
        }

    def set_gripper(self, opening):
        goal = GripperCommand.Goal()
        goal.command.position = float(opening)
        goal.command.max_effort = 20.0
        sf = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=8.0)
        if not sf.done() or sf.result() is None or not sf.result().accepted:
            raise RuntimeError("gripper goal rejected")
        rf = sf.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
        if not rf.done() or rf.result() is None:
            raise RuntimeError("gripper timeout")
        return {
            "status": int(rf.result().status),
            "position": float(rf.result().result.position),
            "reached_goal": bool(rf.result().result.reached_goal),
        }


def main():
    rclpy.init()
    node = PandaProbe()
    try:
        node.wait_ready()
        initial = node.current()
        print("PANDA_MOTION_START", flush=True)
        inspect = node.move_to(INSPECT, "panda_inspect")
        print("INSPECT", json.dumps(inspect), flush=True)
        intermediate = node.current()
        fingers = node.set_gripper(0.03)
        print("GRIPPER_OPEN", json.dumps(fingers), flush=True)
        home = node.move_to(READY, "panda_ready")
        print("READY", json.dumps(home), flush=True)
        final = node.current()

        movement = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(intermediate, initial))
        )
        max_inspect_error = max(
            abs(a - b) for a, b in zip(intermediate, INSPECT)
        )
        max_home_error = max(abs(a - b) for a, b in zip(final, READY))
        report = {
            "robot": "Franka Panda",
            "joints": JOINTS,
            "hardware": "mock_components (controller-level, no Gazebo dynamics)",
            "initial": initial,
            "inspect_result": inspect,
            "inspect_positions": intermediate,
            "gripper_open_result": fingers,
            "return_result": home,
            "return_positions": final,
            "movement_l2_rad": movement,
            "inspect_max_error_rad": max_inspect_error,
            "return_max_error_rad": max_home_error,
            "passed": (
                inspect["success"]
                and home["success"]
                and fingers["status"] == 4
                and movement > 0.2
                and max_inspect_error < 0.05
                and max_home_error < 0.05
            ),
        }
        print(json.dumps(report, indent=2), flush=True)
        return 0 if report["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
