#!/usr/bin/env python3
import json
import math
import time

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from homeagent_interfaces.msg import SkillResult


ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_JOINT = "gripper_joint"


class PickLogicalAttachProbe(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_pick_logical_attach_probe")
        self.positions = {}
        self.result = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._on_skill_result, 10
        )
        self.command_pub = self.create_publisher(
            String, "/homeagent/user_command", 10
        )
        self.scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, value in zip(msg.name, msg.position):
            if name in ARM_JOINT_NAMES or name == GRIPPER_JOINT:
                self.positions[name] = float(value)

    def _on_skill_result(self, msg: SkillResult) -> None:
        if msg.action == "pick":
            self.result = msg

    def wait_ready(self, timeout: float = 12.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            joints_ready = all(
                name in self.positions
                for name in ARM_JOINT_NAMES + [GRIPPER_JOINT]
            )
            planner_ready = self.command_pub.get_subscription_count() > 0
            scene_ready = self.scene_client.service_is_ready()
            if joints_ready and planner_ready and scene_ready:
                return
        raise RuntimeError(
            "timed out waiting for HomeArm / gripper / planner / planning scene"
        )

    def current_arm(self):
        return [self.positions[name] for name in ARM_JOINT_NAMES]

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

    def attached_object_ids(self) -> list:
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("get_planning_scene timed out")
        return [
            item.object.id
            for item in future.result().scene.robot_state.attached_collision_objects
        ]


def main() -> int:
    rclpy.init()
    node = PickLogicalAttachProbe()
    try:
        node.wait_ready()
        initial = node.current_arm()
        initial_gripper = float(node.positions[GRIPPER_JOINT])

        node.send_command("拿起水杯")
        result = node.wait_result()

        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        parsed = json.loads(result.result_json or "{}")
        target = [float(v) for v in parsed.get("joint_target") or []]
        final = node.current_arm()
        final_gripper = float(node.positions[GRIPPER_JOINT])
        attached_ids = node.attached_object_ids()

        if len(target) != len(ARM_JOINT_NAMES):
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
            "approach_completed": parsed.get("approach_completed"),
            "gripper_closed": parsed.get("gripper_closed"),
            "planning_scene_attached": parsed.get("planning_scene_attached"),
            "logical_attach_complete": parsed.get("logical_attach_complete"),
            "physical_grasp": parsed.get("physical_grasp"),
            "grasp_complete": parsed.get("grasp_complete"),
            "planned_points": parsed.get("planned_points"),
            "initial_arm": initial,
            "target_arm": target,
            "final_arm": final,
            "movement_l2_rad": movement,
            "max_joint_error_rad": max_error,
            "initial_gripper_m": initial_gripper,
            "final_gripper_m": final_gripper,
            "attached_object_ids": attached_ids,
            "passed": (
                bool(result.success)
                and result.code == "MOVEIT_PICK_LOGICAL_ATTACH_SUCCEEDED"
                and parsed.get("target_name") == "pick_approach:cup"
                and parsed.get("target_source") == "trusted_object_memory"
                and parsed.get("manipulation_stage") == "logical_attach"
                and parsed.get("approach_completed") is True
                and parsed.get("gripper_closed") is True
                and parsed.get("planning_scene_attached") is True
                and parsed.get("logical_attach_complete") is True
                and parsed.get("physical_grasp") is False
                and parsed.get("grasp_complete") is False
                and "cup" in attached_ids
                and initial_gripper > 0.03
                and final_gripper < 0.01
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
