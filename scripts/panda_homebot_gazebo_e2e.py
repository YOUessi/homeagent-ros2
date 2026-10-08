#!/usr/bin/env python3
"""Gazebo physical HomeBot+Panda end-to-end control smoke.

Tests the same combined URDF in one Gazebo model:
- all seven Panda physics joints under MoveIt2;
- independent Gazebo finger-link separation;
- HomeBot wheel-drive, odometry, and laser scan;
- physical model tilt, without claiming Nav2 or grasp success.
"""
import json
import math
import os
import time

import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from panda_gazebo_physics_e2e import PhysicalPandaProbe, sample_state
from panda_moveit_joint_demo import READY, INSPECT


class MobilePandaProbe(PhysicalPandaProbe):
    def __init__(self):
        super().__init__()
        self.odom = None
        self.scan = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scan = msg

    def wait_mobile(self, seconds=16):
        limit = time.monotonic() + seconds
        while time.monotonic() < limit:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None and self.scan is not None:
                return
        raise RuntimeError("combined Panda/HomeBot /odom or /scan not ready")

    def pose_and_tilt(self):
        request = GetEntityState.Request()
        request.name = "homebot_panda"
        request.reference_frame = "world"
        future = self.entity_state.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        res = future.result()
        if res is None or not res.success:
            raise RuntimeError("combined model state not available")
        p = res.state.pose.position
        q = res.state.pose.orientation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        roll = math.atan2(
            2 * (w * x + y * z),
            1 - 2 * (x * x + y * y),
        )
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        return [float(p.x), float(p.y), float(p.z)], [roll, pitch]

    def drive_short(self):
        p0 = self.odom.pose.pose.position
        start = [float(p0.x), float(p0.y)]
        cmd = Twist()
        cmd.linear.x = 0.12
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            self.vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.07)
        halt = Twist()
        for _ in range(8):
            self.vel_pub.publish(halt)
            rclpy.spin_once(self, timeout_sec=0.08)
        self.vel_pub.publish(halt)
        p = self.odom.pose.pose.position
        final = [float(p.x), float(p.y)]
        return start, final, math.hypot(final[0] - start[0], final[1] - start[1])


def main():
    rclpy.init()
    node = MobilePandaProbe()
    try:
        node.wait_ready()
        node.wait_mobile()
        init_code = node.initialize_ready()
        initial_ready = sample_state(node)
        init_error = max(abs(a - b) for a, b in zip(initial_ready, READY))
        if init_code != 0 or init_error > 0.10:
            raise RuntimeError("Panda mobile initial posture did not converge")

        model_start, model_start_tilt = node.pose_and_tilt()
        inspect = node.move_to(INSPECT, "mobile_panda_inspect")
        inspection = sample_state(node)
        inspect_error = max(abs(a - b) for a, b in zip(inspection, INSPECT))
        movement = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(inspection, initial_ready))
        )
        fingers_before = node.finger_world_gap()
        grip = node.set_gripper(0.03)
        sample_state(node, 0.6)
        fingers_after = node.finger_world_gap()
        ready = node.move_to(READY, "mobile_panda_ready")
        joint_return = sample_state(node)
        ready_error = max(abs(a - b) for a, b in zip(joint_return, READY))

        pos0, pos1, displacement = node.drive_short()
        model_final, model_final_tilt = node.pose_and_tilt()

        finite_lidar = sum(
            1 for value in self_scan_ranges(node.scan) if math.isfinite(value)
        )
        report = {
            "model": "one Gazebo entity: homebot_panda",
            "robot_description": "HomeBot + official Panda CAD meshes",
            "hardware": "gazebo_ros2_control/GazeboSystem",
            "platform": "physical diff drive + Panda 7DoF + dual fingers",
            "PandaMoveItInspect": inspect,
            "PandaMoveItReturn": ready,
            "joint_movement_l2_rad": movement,
            "inspect_max_error_rad": inspect_error,
            "return_max_error_rad": ready_error,
            "PandaGripperAction": grip,
            "GazeboFingerLinkGapBefore_m": fingers_before,
            "GazeboFingerLinkGapAfter_m": fingers_after,
            "GazeboFingerGapDelta_m": fingers_after - fingers_before,
            "LiDAR_finite_samples": finite_lidar,
            "base_start_xy": pos0,
            "base_final_xy": pos1,
            "base_displacement_m": displacement,
            "model_start_xyz": model_start,
            "model_final_xyz": model_final,
            "model_start_rollpitch": model_start_tilt,
            "model_final_rollpitch": model_final_tilt,
            "contact_grasp_tested": False,
            "nav2_integrated_tested": False,
            "passed": (
                inspect["success"]
                and ready["success"]
                and movement > 0.20
                and inspect_error < 0.07
                and ready_error < 0.07
                and int(grip["status"]) == 4
                and fingers_after - fingers_before > 0.035
                and finite_lidar > 180
                and displacement > 0.10
                and max(map(abs, model_final_tilt)) < 0.30
            ),
        }
        with open(
            "/workspace/artifacts/panda_mobile/e2e_report.json",
            "w", encoding="utf-8"
        ) as fp:
            json.dump(report, fp, ensure_ascii=False, indent=2)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        return 0 if report["passed"] else 2
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


def self_scan_ranges(scan):
    return scan.ranges if scan is not None else ()


if __name__ == "__main__":
    raise SystemExit(main())
