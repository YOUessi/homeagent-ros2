#!/usr/bin/env python3
"""Observe navigation actuation versus odometry without controlling the robot.

An independent telemetry subscriber in the same ROS_DOMAIN_ID collects
2 Hz samples of Nav2 commands and physical Gazebo odometry. Its output
provides evidence when Nav2 SimpleProgressChecker times out.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class Monitor(Node):
    def __init__(self, out_file):
        super().__init__("homeagent_nav2_telemetry")
        self.out_file = open(out_file, "w", encoding="utf-8", buffering=1)
        self.latest = {}
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_map, 20
        )
        self.map_to_odom_count = 0
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        for topic in ("/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed"):
            self.create_subscription(
                Twist, topic, lambda msg, k=topic: self.on_twist(k, msg), 20
            )
        self.create_timer(0.5, self.flush)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )
        self.latest["odom"] = {
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(yaw),
            "vx": float(msg.twist.twist.linear.x),
            "wz": float(msg.twist.twist.angular.z),
            "header_sim_time": float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9,
            "received_wall": time.time(),
        }

    def on_map(self, msg):
        p = msg.pose.pose.position
        self.latest["amcl"] = {
            "x": float(p.x), "y": float(p.y),
            "covariance_xy": [
                float(msg.pose.covariance[0]),
                float(msg.pose.covariance[7]),
            ],
            "received_wall": time.time(),
        }

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id.lstrip("/") == "map" and (
                tf.child_frame_id.lstrip("/") == "odom"
            ):
                self.map_to_odom_count += 1
                self.latest["map_to_odom_tf"] = {
                    "received_wall": time.time(),
                    "header_sim_time": float(tf.header.stamp.sec)
                    + float(tf.header.stamp.nanosec) * 1e-9,
                    "count": self.map_to_odom_count,
                }

    def on_twist(self, topic, msg):
        self.latest[topic] = {
            "vx": float(msg.linear.x),
            "wz": float(msg.angular.z),
            "received_wall": time.time(),
        }

    def flush(self):
        row = {
            "timestamp_wall": time.time(),
            "timestamp_sim": float(self.get_clock().now().nanoseconds) * 1e-9,
            **self.latest,
        }
        self.out_file.write(json.dumps(row, separators=(",", ":")) + "\n")

    def close(self):
        self.out_file.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    # Leave --ros-args / remappings for rclpy.
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Monitor(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
