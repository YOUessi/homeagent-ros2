#!/usr/bin/env python3
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from homeagent_interfaces.msg import SkillResult


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


class PickApproachProbe(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_pick_approach_probe")
        self.positions = {}
        self.result = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._on_skill_result, 10
        )
        self.command_pub = self.create_publisher(
            String, "/homeagent/user_command", 10
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, value in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                self.positions[name] = float(value)

    def _on_skill_result(self, msg: SkillResult) -> None:
        if msg.action == "pick":
            self.result = msg

    def wait_ready(self, timeout: float = 12.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            joints_ready = all(name in self.positions for name in JOINT_NAMES)
            planner_ready = self.command_pub.get_subscription_count() > 0
            if joints_ready and planner_ready:
                return
        raise RuntimeError("timed out waiting for HomeArm / planner")

    def current(self):
        return [self.positions[name] for name in JOINT_NAMES]

    def send_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)

    def wait_result(self, timeout: float = 35.0) -> SkillResult:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.result is not None:
                return self.result
        raise RuntimeError("timed out waiting for pick SkillResult")


def main() -> int:
    rclpy.init()
    node = PickApproachProbe()
    try:
        node.wait_ready()
        initial = node.current()

        node.send_command("拿起水杯")
        result = node.wait_result()

        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        parsed = json.loads(result.result_json or "{}")
        target = [float(v) for v in parsed.get("joint_target") or []]
        final = node.current()

        if len(target) != len(JOINT_NAMES):
            raise RuntimeError("pick result did not contain a 4-joint target")

        movement = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(final, initial))
        )
        max_error = max(abs(a - b) for a, b in zip(final, target))

        report = {
            "command": "拿起水杯",
            "action": result.action,
            "skill_success": bool(result.success),
            "skill_code": result.code,
            "target_name": parsed.get("target_name"),
            "target_source": parsed.get("target_source"),
            "manipulation_stage": parsed.get("manipulation_stage"),
            "grasp_complete": parsed.get("grasp_complete"),
            "planned_points": parsed.get("planned_points"),
            "initial": initial,
            "target": target,
            "final": final,
            "movement_l2_rad": movement,
            "max_joint_error_rad": max_error,
            "passed": (
                bool(result.success)
                and result.code == "MOVEIT_PICK_APPROACH_SUCCEEDED"
                and parsed.get("target_name") == "pick_approach:cup"
                and parsed.get("target_source") == "trusted_object_memory"
                and parsed.get("manipulation_stage") == "pick_approach"
                and parsed.get("grasp_complete") is False
                and movement > 0.25
                and max_error < 0.05
            ),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
