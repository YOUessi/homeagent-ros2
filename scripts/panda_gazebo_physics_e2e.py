#!/usr/bin/env python3
"""GazeboSystem Panda physical-joint acceptance; NOT mock_components.

An isolated startup trajectory moves Gazebo's zero-joint spawn into a valid
Panda ready posture. Only then does MoveIt plan the inspect/return motions.
Physical end-effector contact grasp is NOT tested by this script.
"""
import json
import math
import os
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetEntityState
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

from panda_moveit_joint_demo import PandaProbe, JOINTS, READY, INSPECT


class PhysicalPandaProbe(PandaProbe):
    def __init__(self):
        super().__init__()
        self.initializer = ActionClient(
            self, FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )
        self.entity_state = self.create_client(
            GetEntityState,
            "/gazebo/get_entity_state",
        )

    def finger_world_gap(self):
        if not self.entity_state.wait_for_service(timeout_sec=4.0):
            raise RuntimeError("Gazebo GetEntityState unavailable")
        points = []
        for name in ("panda_leftfinger", "panda_rightfinger"):
            request = GetEntityState.Request()
            request.name = f"panda_physics::{name}"
            request.reference_frame = "world"
            future = self.entity_state.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=4.0)
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(f"failed to query Gazebo link {name}")
            p = response.state.pose.position
            points.append((float(p.x), float(p.y), float(p.z)))
        return math.sqrt(
            sum((a - b) ** 2 for a, b in zip(*points))
        )

    def initialize_ready(self):
        if not self.initializer.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Gazebo Panda arm trajectory controller unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in READY]
        pt.time_from_start.sec = 8
        goal.trajectory.points = [pt]
        goal.goal_time_tolerance.sec = 3

        send = self.initializer.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=12.0)
        if not send.done() or send.result() is None or not send.result().accepted:
            raise RuntimeError("Gazebo Panda initial joint state command rejected")
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=30.0)
        if not result.done() or result.result() is None:
            raise RuntimeError("Gazebo Panda initial joint state command timed out")
        wrapped = result.result()
        if int(wrapped.status) != 4:
            raise RuntimeError(f"Gazebo Panda init status={wrapped.status}")
        return int(wrapped.result.error_code)


def sample_state(node, seconds=1.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.current()


def main():
    rclpy.init()
    node = PhysicalPandaProbe()
    try:
        node.wait_ready()
        initially = node.current()
        init_code = node.initialize_ready()
        after_init = sample_state(node)
        init_error = max(abs(a - b) for a, b in zip(after_init, READY))
        print("GAZEBO_INITIAL_READY", json.dumps({
            "result_error_code": init_code,
            "start": initially,
            "end": after_init,
            "max_error_rad": init_error,
        }), flush=True)

        if init_code != 0 or init_error > 0.10:
            raise RuntimeError("Initial Panda GazeboSystem movement did not converge")

        inspect = node.move_to(INSPECT, "gazebo_panda_inspect")
        after_inspect = sample_state(node)
        closed_finger_link_distance = node.finger_world_gap()
        fingers = node.set_gripper(0.03)
        after_hand = sample_state(node)
        opened_finger_link_distance = node.finger_world_gap()
        return_result = node.move_to(READY, "gazebo_panda_ready")
        after_return = sample_state(node)

        displacement = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(after_inspect, after_init))
        )
        inspect_error = max(abs(a - b) for a, b in zip(after_inspect, INSPECT))
        return_error = max(abs(a - b) for a, b in zip(after_return, READY))
        finger_positions = {
            k: node.latest.get(k)
            for k in ("panda_finger_joint1", "panda_finger_joint2")
        }
        physical_gripper_gap_delta = (
            opened_finger_link_distance - closed_finger_link_distance
        )
        report = {
            "robot": "Panda 7 DoF + 2 fingers",
            "hardware": "gazebo_ros2_control/GazeboSystem",
            "gazebo_master": os.environ.get("GAZEBO_MASTER_URI"),
            "initial_ready_error_rad": init_error,
            "inspect_result": inspect,
            "inspect_joint_positions": after_inspect,
            "inspection_motion_l2_rad": displacement,
            "inspect_max_joint_error_rad": inspect_error,
            "gripper_result": fingers,
            "gripper_joint_positions": finger_positions,
            "gripper_link_distance_before_m": closed_finger_link_distance,
            "gripper_link_distance_after_m": opened_finger_link_distance,
            "gripper_link_distance_delta_m": physical_gripper_gap_delta,
            "gripper_physics_observation": "Gazebo GetEntityState link positions",
            "return_result": return_result,
            "return_max_joint_error_rad": return_error,
            "physical_contact_grasp_tested": False,
            "passed": (
                inspect["success"]
                and return_result["success"]
                and displacement > 0.20
                and inspect_error < 0.07
                and return_error < 0.07
                and int(fingers["status"]) == 4
                and physical_gripper_gap_delta > 0.035
                and abs(float(fingers["position"]) - 0.03) < 0.006
            ),
        }
        path = os.environ.get(
            "PANDA_GAZEBO_REPORT",
            "/workspace/artifacts/panda_physics/physics_report.json",
        )
        with open(path, "w", encoding="utf-8") as fp:
            json.dump(report, fp, ensure_ascii=False, indent=2)
        print("GAZEBO_PANDA_REPORT", json.dumps(report, indent=2), flush=True)
        return 0 if report["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
