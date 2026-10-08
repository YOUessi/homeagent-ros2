#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class LocalizationInitializer(Node):
    def __init__(self):
        super().__init__("homeagent_localization_initializer")
        self.odom = None
        self.amcl = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_amcl(self, msg):
        self.amcl = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        end = time.monotonic() + 12.0
        while self.odom is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom is None:
            raise RuntimeError("timed out waiting for /odom")

        p = self.odom.pose.pose.position
        yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        x, y = float(p.x), float(p.y)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02

        # AMCL may still be transitioning through lifecycle activation, so
        # publish repeatedly and only declare success when both pose and TF exist.
        deadline = time.monotonic() + 18.0
        published = 0
        while time.monotonic() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            published += 1
            self.spin_for(0.25)

            if self.amcl is None:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map",
                    "base_footprint",
                    rclpy.time.Time(),
                )
            except Exception:
                continue

            report = {
                "published_initialposes": published,
                "seed_xyyaw": [x, y, yaw],
                "amcl_xy": [
                    float(self.amcl.pose.pose.position.x),
                    float(self.amcl.pose.pose.position.y),
                ],
                "tf_translation": [
                    float(tf.transform.translation.x),
                    float(tf.transform.translation.y),
                    float(tf.transform.translation.z),
                ],
                "passed": True,
            }
            print(json.dumps(report, indent=2))
            return 0

        raise RuntimeError(
            "AMCL did not establish map->base_footprint within timeout"
        )


def main():
    rclpy.init()
    node = LocalizationInitializer()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
