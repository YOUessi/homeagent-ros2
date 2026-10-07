#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import SkillResult


class Nav2E2EProbe(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_nav2_e2e_probe")
        self.odom = None
        self.result = None
        self.amcl_pose = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._on_result, 10
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.command_pub = self.create_publisher(
            String, "/homeagent/user_command", 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg

    def _on_result(self, msg: SkillResult) -> None:
        self.result = msg

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_odom(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None:
                return
        raise RuntimeError("timed out waiting for /odom")

    def wait_for_nav2(self, timeout: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("NavigateToPose action server unavailable")

    def set_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02

        # Publish repeatedly so AMCL receives it after lifecycle activation.
        for _ in range(8):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.spin_for(0.25)

    def wait_for_amcl(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.amcl_pose is not None:
                return
        raise RuntimeError("AMCL did not publish /amcl_pose after initial pose")

    def send_user_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)
        self.spin_for(0.2)

    def wait_for_skill_result(self, timeout: float) -> SkillResult:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.result is not None and self.result.action == "navigate":
                return self.result
        raise RuntimeError("timed out waiting for navigate SkillResult")


def main() -> int:
    rclpy.init()
    node = Nav2E2EProbe()
    try:
        node.wait_for_odom(timeout=10.0)
        node.wait_for_nav2(timeout=15.0)

        start = node.odom.pose.pose.position
        x0, y0 = float(start.x), float(start.y)

        node.set_initial_pose()
        node.wait_for_amcl(timeout=10.0)
        node.spin_for(2.0)

        node.send_user_command("去客厅")
        result = node.wait_for_skill_result(timeout=45.0)
        node.spin_for(1.0)

        final = node.odom.pose.pose.position
        x1, y1 = float(final.x), float(final.y)
        displacement = math.hypot(x1 - x0, y1 - y0)

        parsed = json.loads(result.result_json or "{}")
        report = {
            "request_id": result.request_id,
            "skill_success": bool(result.success),
            "skill_code": result.code,
            "skill_result": parsed,
            "initial_xy": [x0, y0],
            "final_xy": [x1, y1],
            "displacement_m": displacement,
            "amcl_received": node.amcl_pose is not None,
            "passed": (
                bool(result.success)
                and result.code == "NAV2_SUCCEEDED"
                and displacement > 0.5
            ),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
