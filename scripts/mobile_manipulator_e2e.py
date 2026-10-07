#!/usr/bin/env python3
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from homeagent_interfaces.msg import SkillResult


ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_JOINT = "gripper_joint"


class MobileManipulatorProbe(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_mobile_manipulator_probe")
        self.odom = None
        self.arm_positions = {}
        self.results = {}
        self.amcl_pose = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
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

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_joint_state(self, msg: JointState) -> None:
        for name, value in zip(msg.name, msg.position):
            if name in ARM_JOINT_NAMES or name == GRIPPER_JOINT:
                self.arm_positions[name] = float(value)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl_pose = msg

    def _on_result(self, msg: SkillResult) -> None:
        self.results[msg.action] = msg

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_ready(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            base_ready = self.odom is not None
            arm_ready = all(
                name in self.arm_positions
                for name in ARM_JOINT_NAMES + [GRIPPER_JOINT]
            )
            planner_ready = self.command_pub.get_subscription_count() > 0
            scene_ready = self.scene_client.service_is_ready()
            nav_ready = self.nav_client.server_is_ready()
            if base_ready and arm_ready and planner_ready and scene_ready and nav_ready:
                return
        raise RuntimeError(
            "timed out waiting for base + arm + HomeAgent + Nav2 + MoveIt"
        )

    def check_mount_tf(self):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link", "arm_base_footprint", Time()
                )
                return {
                    "x": float(tf.transform.translation.x),
                    "y": float(tf.transform.translation.y),
                    "z": float(tf.transform.translation.z),
                }
            except Exception:
                pass
        raise RuntimeError("base_link -> arm_base_footprint TF unavailable")

    def set_initial_pose(self) -> None:
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

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.amcl_pose is not None:
                return
        raise RuntimeError("AMCL initial pose was not accepted")

    def send_command(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)

    def wait_result(self, action: str, timeout: float):
        self.results.pop(action, None)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if action in self.results:
                return self.results[action]
        raise RuntimeError(f"timed out waiting for {action} SkillResult")

    def current_arm(self):
        return [self.arm_positions[name] for name in ARM_JOINT_NAMES]

    def attached_ids(self):
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
    node = MobileManipulatorProbe()
    try:
        node.wait_ready()
        mount = node.check_mount_tf()

        node.set_initial_pose()
        node.spin_for(1.5)

        start = node.odom.pose.pose.position
        x0, y0 = float(start.x), float(start.y)

        node.results.pop("navigate", None)
        node.send_command("去客厅")
        nav_result = node.wait_result("navigate", 45.0)
        node.spin_for(0.5)

        end = node.odom.pose.pose.position
        x1, y1 = float(end.x), float(end.y)
        base_displacement = math.hypot(x1 - x0, y1 - y0)
        nav_payload = json.loads(nav_result.result_json or "{}")

        initial_arm = node.current_arm()
        initial_gripper = float(node.arm_positions[GRIPPER_JOINT])

        node.results.pop("pick", None)
        node.send_command("拿起水杯")
        pick_result = node.wait_result("pick", 40.0)
        node.spin_for(0.5)

        pick_payload = json.loads(pick_result.result_json or "{}")
        target_arm = [float(v) for v in pick_payload.get("joint_target") or []]
        final_arm = node.current_arm()
        final_gripper = float(node.arm_positions[GRIPPER_JOINT])
        attached_ids = node.attached_ids()

        arm_error = (
            max(abs(a - b) for a, b in zip(final_arm, target_arm))
            if len(target_arm) == 4
            else 999.0
        )
        arm_motion = math.sqrt(
            sum((a - b) ** 2 for a, b in zip(final_arm, initial_arm))
        )

        report = {
            "mount_tf": mount,
            "navigation": {
                "success": bool(nav_result.success),
                "code": nav_result.code,
                "target": nav_payload.get("target"),
                "target_source": nav_payload.get("target_source"),
                "initial_xy": [x0, y0],
                "final_xy": [x1, y1],
                "displacement_m": base_displacement,
            },
            "pick": {
                "success": bool(pick_result.success),
                "code": pick_result.code,
                "target_source": pick_payload.get("target_source"),
                "initial_arm": initial_arm,
                "target_arm": target_arm,
                "final_arm": final_arm,
                "arm_motion_l2_rad": arm_motion,
                "max_joint_error_rad": arm_error,
                "initial_gripper_m": initial_gripper,
                "final_gripper_m": final_gripper,
                "attached_object_ids": attached_ids,
                "physical_grasp": pick_payload.get("physical_grasp"),
            },
        }

        report["passed"] = (
            abs(mount["z"] - 0.22) < 1e-6
            and bool(nav_result.success)
            and nav_result.code == "NAV2_SUCCEEDED"
            and nav_payload.get("target_source") == "trusted_memory"
            and base_displacement > 0.5
            and bool(pick_result.success)
            and pick_result.code == "MOVEIT_PICK_LOGICAL_ATTACH_SUCCEEDED"
            and pick_payload.get("target_source") == "trusted_object_memory"
            and arm_motion > 0.25
            and arm_error < 0.05
            and initial_gripper > 0.03
            and final_gripper < 0.01
            and "cup" in attached_ids
            and pick_payload.get("physical_grasp") is False
        )

        report_path = os.environ.get("HOMEAGENT_MOBILE_REPORT")
        if report_path:
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(report, f, ensure_ascii=False, indent=2)

        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
