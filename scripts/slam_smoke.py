#!/usr/bin/env python3
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SlamProbe(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_slam_probe")
        self.grid = None
        self.odom = None
        self.scan = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.grid = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def command(self, linear: float, angular: float, seconds: float) -> None:
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def stop(self) -> None:
        zero = Twist()
        for _ in range(6):
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = SlamProbe()
    try:
        node.spin_for(5.0)
        if node.scan is None or node.odom is None:
            raise RuntimeError("simulation topics /scan or /odom are missing")

        # Explore a short safe arc inside the 6x6 room.
        node.command(0.16, 0.0, 3.0)
        node.command(0.0, 0.45, 2.5)
        node.command(0.16, 0.0, 2.5)
        node.stop()
        node.spin_for(4.0)

        if node.grid is None:
            raise RuntimeError("SLAM Toolbox did not publish /map")

        data = list(node.grid.data)
        known = sum(1 for value in data if value >= 0)
        occupied = sum(1 for value in data if value >= 50)
        free = sum(1 for value in data if 0 <= value < 50)
        result = {
            "map_width": int(node.grid.info.width),
            "map_height": int(node.grid.info.height),
            "resolution": float(node.grid.info.resolution),
            "known_cells": known,
            "free_cells": free,
            "occupied_cells": occupied,
            "map_ok": (
                node.grid.info.width > 10
                and node.grid.info.height > 10
                and known > 100
                and occupied > 5
            ),
        }
        result["passed"] = result["map_ok"]
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["passed"] else 2
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
