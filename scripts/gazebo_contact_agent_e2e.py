#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import SkillResult


class Probe(Node):
    def __init__(self):
        super().__init__("homeagent_gazebo_contact_agent_probe")
        self.result = None
        self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._on_result, 10
        )
        self.command_pub = self.create_publisher(
            String, "/homeagent/user_command", 10
        )

    def _on_result(self, msg):
        if msg.action == "pick":
            self.result = msg

    def wait_ready(self, timeout=12.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.command_pub.get_subscription_count() > 0:
                return
        raise RuntimeError("planner not ready")

    def run(self):
        self.wait_ready()
        msg = String()
        msg.data = "拿起水杯"
        self.command_pub.publish(msg)

        end = time.monotonic() + 45.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.result is not None:
                break
        if self.result is None:
            raise RuntimeError("timed out waiting for pick SkillResult")

        payload = json.loads(self.result.result_json or "{}")
        passed = (
            bool(self.result.success)
            and self.result.code == "GAZEBO_CONTACT_PICK_SUCCEEDED"
            and payload.get("target_source") == "trusted_object_memory"
            and payload.get("planning_scene_attach_used") is False
            and payload.get("physics_constraint_attach_used") is True
            and payload.get("contact_required") is True
            and payload.get("friction_only_grasp") is False
            and payload.get("autonomous_table_pick") is False
            and payload.get("contact_gated_physical_hold") is True
            and float(payload.get("cup_world_motion_m", 0.0)) > 0.08
            and float(payload.get("cup_relative_tool_drift_m", 99.0)) < 0.035
        )

        report = {
            "command": "拿起水杯",
            "action": self.result.action,
            "skill_success": bool(self.result.success),
            "skill_code": self.result.code,
            "payload": payload,
            "passed": passed,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if passed else 2


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
