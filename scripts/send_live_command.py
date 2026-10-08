#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: send_live_command.py <command text>")

    text = " ".join(sys.argv[1:])
    rclpy.init()
    node = Node("homeagent_live_command_sender")
    pub = node.create_publisher(String, "/homeagent/user_command", 10)

    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if pub.get_subscription_count() >= 2:
                break
        else:
            raise RuntimeError(
                f"expected >=2 subscribers, got {pub.get_subscription_count()}"
            )

        msg = String()
        msg.data = text
        pub.publish(msg)

        # Keep the publisher alive long enough for reliable delivery.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        print(
            f"published_once subscribers={pub.get_subscription_count()} "
            f"command={text}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
