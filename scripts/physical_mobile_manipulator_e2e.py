#!/usr/bin/env python3
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import SkillResult


class Probe(Node):
    def __init__(self):
        super().__init__("homeagent_physical_mobile_probe")
        self.odom = None
        self.amcl = None
        self.results = {}
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )
        self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._on_result, 20
        )
        self.command_pub = self.create_publisher(
            String, "/homeagent/user_command", 10
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_odom(self, msg):
        self.odom = msg

    def _on_amcl(self, msg):
        self.amcl = msg

    def _on_result(self, msg):
        self.results[msg.action] = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_ready(self, timeout=22.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.odom is not None
                and self.command_pub.get_subscription_count() > 0
            ):
                return
        raise RuntimeError("physical mobile stack not ready")

    def localize(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        for _ in range(8):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.spin_for(0.25)

        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.amcl is not None:
                return
        raise RuntimeError("AMCL did not accept initial pose")

    def command(self, text):
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)

    def wait_result(self, action, timeout):
        self.results.pop(action, None)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if action in self.results:
                return self.results[action]
        raise RuntimeError(f"timed out waiting for {action} result")

    def run(self):
        self.wait_ready()
        self.localize()
        self.spin_for(1.0)
        if not self.nav.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("NavigateToPose action server unavailable")
        self.spin_for(0.5)

        p0 = self.odom.pose.pose.position
        start = [float(p0.x), float(p0.y)]

        self.command("去客厅")
        nav_result = self.wait_result("navigate", 45.0)
        self.spin_for(0.5)

        p1 = self.odom.pose.pose.position
        end = [float(p1.x), float(p1.y)]
        displacement = math.hypot(end[0] - start[0], end[1] - start[1])
        nav_payload = json.loads(nav_result.result_json or "{}")

        self.command("拿起水杯")
        pick_result = self.wait_result("pick", 45.0)
        pick_payload = json.loads(pick_result.result_json or "{}")

        report = {
            "navigation": {
                "success": bool(nav_result.success),
                "code": nav_result.code,
                "target": nav_payload.get("target"),
                "target_source": nav_payload.get("target_source"),
                "initial_xy": start,
                "final_xy": end,
                "displacement_m": displacement,
            },
            "pick": {
                "success": bool(pick_result.success),
                "code": pick_result.code,
                "target_source": pick_payload.get("target_source"),
                "planning_scene_attach_used":
                    pick_payload.get("planning_scene_attach_used"),
                "physics_constraint_attach_used":
                    pick_payload.get("physics_constraint_attach_used"),
                "contact_required": pick_payload.get("contact_required"),
                "friction_only_grasp":
                    pick_payload.get("friction_only_grasp"),
                "autonomous_table_pick":
                    pick_payload.get("autonomous_table_pick"),
                "cup_world_motion_m":
                    pick_payload.get("cup_world_motion_m"),
                "cup_relative_tool_drift_m":
                    pick_payload.get("cup_relative_tool_drift_m"),
                "contact_gated_physical_hold":
                    pick_payload.get("contact_gated_physical_hold"),
            },
        }

        report["passed"] = (
            bool(nav_result.success)
            and nav_result.code == "NAV2_SUCCEEDED"
            and nav_payload.get("target_source") == "place_memory"
            and displacement > 0.5
            and bool(pick_result.success)
            and pick_result.code == "GAZEBO_CONTACT_PICK_SUCCEEDED"
            and pick_payload.get("target_source") == "trusted_object_memory"
            and pick_payload.get("planning_scene_attach_used") is False
            and pick_payload.get("physics_constraint_attach_used") is True
            and pick_payload.get("contact_required") is True
            and pick_payload.get("friction_only_grasp") is False
            and pick_payload.get("autonomous_table_pick") is False
            and pick_payload.get("contact_gated_physical_hold") is True
        )

        path = os.environ.get("HOMEAGENT_PHYSICAL_MOBILE_REPORT")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(report, f, ensure_ascii=False, indent=2)

        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 2


def main():
    rclpy.init()
    node = Probe()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
