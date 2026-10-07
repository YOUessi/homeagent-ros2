#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimProbe(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_sim_probe")
        self.odom = None
        self.scan = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def spin_until_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None and self.scan is not None:
                return
        raise RuntimeError("timed out waiting for /odom and /scan")

    def drive_forward(self, speed: float, duration: float) -> None:
        msg = Twist()
        msg.linear.x = speed
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)


def main() -> int:
    rclpy.init()
    node = SimProbe()
    try:
        node.spin_until_ready(timeout=12.0)
        initial = node.odom.pose.pose.position
        x0, y0 = float(initial.x), float(initial.y)

        finite_values = [
            float(value) for value in node.scan.ranges if math.isfinite(value)
        ]
        finite_ranges = len(finite_values)
        sample_count = len(node.scan.ranges)
        finite_values_sorted = sorted(finite_values)
        scan_min = finite_values_sorted[0] if finite_values_sorted else None
        scan_max = finite_values_sorted[-1] if finite_values_sorted else None
        close_returns = sum(1 for value in finite_values if value < 0.30)

        node.drive_forward(speed=0.20, duration=2.0)
        node.spin_until_ready(timeout=2.0)
        final = node.odom.pose.pose.position
        x1, y1 = float(final.x), float(final.y)
        displacement = math.hypot(x1 - x0, y1 - y0)

        result = {
            "scan_samples": sample_count,
            "finite_scan_samples": finite_ranges,
            "scan_min_m": scan_min,
            "scan_max_m": scan_max,
            "scan_returns_lt_0_30m": close_returns,
            "initial_xy": [x0, y0],
            "final_xy": [x1, y1],
            "displacement_m": displacement,
            "scan_ok": sample_count >= 180 and finite_ranges > 0,
            "motion_ok": displacement >= 0.10,
        }
        result["passed"] = result["scan_ok"] and result["motion_ok"]
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
