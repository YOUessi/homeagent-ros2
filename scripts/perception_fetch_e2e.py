#!/usr/bin/env python3
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import SkillResult
from homeagent_interfaces.srv import MemoryQuery


COMMAND = "去拿水杯"


class PerceptionFetchProbe(Node):
    def __init__(self):
        super().__init__("homeagent_perception_fetch_probe")
        self.odom = None
        self.amcl = None
        self.results = {}

        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
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
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.memory = self.create_client(
            MemoryQuery, "/homeagent/memory/query"
        )

    def _on_odom(self, msg):
        self.odom = msg

    def _on_amcl(self, msg):
        self.amcl = msg

    def _on_result(self, msg):
        self.results[msg.action] = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_runtime(self, timeout=25.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.odom is not None
                and self.command_pub.get_subscription_count() > 0
                and self.memory.service_is_ready()
            ):
                return
        raise RuntimeError("fetch runtime not ready")

    def query_cup(self):
        request = MemoryQuery.Request()
        request.entity_type = "object"
        request.entity_id = ""
        request.name = "cup"
        future = self.memory.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            return None
        response = future.result()
        if not response.found:
            return None
        return json.loads(response.record_json or "{}")

    def wait_perception(self, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            record = self.query_cup()
            if record:
                payload = record.get("payload") or {}
                location = payload.get("location") or {}
                perception = payload.get("perception") or {}
                pose = location.get("pose") or {}
                if (
                    record.get("source") == "gazebo_ground_truth_perception"
                    and perception.get("backend") == "gazebo_ground_truth"
                    and pose.get("frame") == "map"
                    and "observed_at" in location
                ):
                    return record
            self.spin_for(0.15)
        raise RuntimeError("cup was not written to Memory by perception")

    def localize(self):
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

        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.amcl is not None:
                return
        raise RuntimeError("AMCL did not accept initial pose")

    def send_command(self):
        msg = String()
        msg.data = COMMAND
        self.command_pub.publish(msg)

    def wait_result(self, action, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if action in self.results:
                return self.results[action]
        raise RuntimeError(f"timed out waiting for {action} result")

    def run(self):
        self.wait_runtime()
        self.localize()
        self.spin_for(1.0)
        observed = self.wait_perception()

        payload = observed["payload"]
        observed_pose = payload["location"]["pose"]
        pregrasp = payload["manipulation"]["mobile_pregrasp_offset"]

        yaw = float(observed_pose.get("yaw", 0.0)) + float(
            pregrasp.get("yaw", 0.0)
        )
        c = math.cos(yaw)
        s = math.sin(yaw)
        expected_goal = [
            float(observed_pose["x"])
            - (c * float(pregrasp["x"]) - s * float(pregrasp["y"])),
            float(observed_pose["y"])
            - (s * float(pregrasp["x"]) + c * float(pregrasp["y"])),
            yaw,
        ]

        if not self.nav.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("NavigateToPose action server unavailable")
        p0 = self.odom.pose.pose.position
        start_xy = [float(p0.x), float(p0.y)]

        self.send_command()

        stow_result = self.wait_result("stow_arm", 35.0)
        stow_payload = json.loads(stow_result.result_json or "{}")

        nav_result = self.wait_result("navigate", 70.0)
        self.spin_for(0.3)

        p1 = self.odom.pose.pose.position
        nav_final_xy = [float(p1.x), float(p1.y)]
        nav_payload = json.loads(nav_result.result_json or "{}")
        nav_goal = [float(v) for v in nav_payload.get("goal_xyyaw") or []]

        pick_result = self.wait_result("pick", 55.0)
        pick_payload = json.loads(pick_result.result_json or "{}")

        displacement = math.hypot(
            nav_final_xy[0] - start_xy[0],
            nav_final_xy[1] - start_xy[1],
        )
        goal_error = (
            math.hypot(
                nav_final_xy[0] - nav_goal[0],
                nav_final_xy[1] - nav_goal[1],
            )
            if len(nav_goal) == 3
            else 999.0
        )
        dynamic_goal_error = (
            math.hypot(
                nav_goal[0] - expected_goal[0],
                nav_goal[1] - expected_goal[1],
            )
            if len(nav_goal) == 3
            else 999.0
        )

        report = {
            "command": COMMAND,
            "stow": {
                "success": bool(stow_result.success),
                "code": stow_result.code,
                "target_name": stow_payload.get("target_name"),
                "target_source": stow_payload.get("target_source"),
                "joint_target": stow_payload.get("joint_target"),
                "planned_points": stow_payload.get("planned_points"),
            },
            "perception": {
                "memory_source": observed.get("source"),
                "backend": payload["perception"]["backend"],
                "model_name": payload["perception"]["model_name"],
                "zone": payload["location"]["zone"],
                "observed_pose": observed_pose,
                "pregrasp_offset": pregrasp,
            },
            "navigation": {
                "success": bool(nav_result.success),
                "code": nav_result.code,
                "target": nav_payload.get("target"),
                "target_source": nav_payload.get("target_source"),
                "expected_dynamic_goal": expected_goal,
                "actual_goal": nav_goal,
                "dynamic_goal_error_m": dynamic_goal_error,
                "start_xy": start_xy,
                "final_xy": nav_final_xy,
                "goal_position_error_m": goal_error,
                "displacement_m": displacement,
            },
            "pick": {
                "success": bool(pick_result.success),
                "code": pick_result.code,
                "target_source": pick_payload.get("target_source"),
                "spawn_contact_object": pick_payload.get(
                    "spawn_contact_object"
                ),
                "object_preexisted": pick_payload.get("object_preexisted"),
                "perception_observed_target": pick_payload.get(
                    "perception_observed_target"
                ),
                "local_alignment_used": pick_payload.get(
                    "local_alignment_used"
                ),
                "local_alignment": pick_payload.get("local_alignment"),
                "planning_scene_attach_used": pick_payload.get(
                    "planning_scene_attach_used"
                ),
                "physics_constraint_attach_used": pick_payload.get(
                    "physics_constraint_attach_used"
                ),
                "contact_required": pick_payload.get("contact_required"),
                "friction_only_grasp": pick_payload.get(
                    "friction_only_grasp"
                ),
                "cup_before_carry_world": pick_payload.get(
                    "cup_before_carry_world"
                ),
                "cup_after_carry_world": pick_payload.get(
                    "cup_after_carry_world"
                ),
                "cup_before_carry_relative_tool": pick_payload.get(
                    "cup_before_carry_relative_tool"
                ),
                "cup_after_carry_relative_tool": pick_payload.get(
                    "cup_after_carry_relative_tool"
                ),
                "cup_world_motion_m": pick_payload.get(
                    "cup_world_motion_m"
                ),
                "cup_relative_tool_drift_m": pick_payload.get(
                    "cup_relative_tool_drift_m"
                ),
                "contact_gated_physical_hold": pick_payload.get(
                    "contact_gated_physical_hold"
                ),
                "autonomous_table_pick": pick_payload.get(
                    "autonomous_table_pick"
                ),
            },
        }

        report["passed"] = (
            bool(stow_result.success)
            and stow_result.code == "MOVEIT_SUCCEEDED"
            and stow_payload.get("target_name") == "navigation_stow"
            and stow_payload.get("target_source") == "trusted_skill_library"
            and report["perception"]["memory_source"]
            == "gazebo_ground_truth_perception"
            and report["perception"]["backend"] == "gazebo_ground_truth"
            and bool(nav_result.success)
            and nav_result.code == "NAV2_SUCCEEDED"
            and nav_payload.get("target") == "object_pregrasp:cup"
            and nav_payload.get("target_source")
            == "object_memory_observation"
            and dynamic_goal_error < 0.02
            and goal_error < 0.06
            and displacement > 0.20
            and bool(pick_result.success)
            and pick_result.code == "GAZEBO_CONTACT_PICK_SUCCEEDED"
            and pick_payload.get("spawn_contact_object") is False
            and pick_payload.get("object_preexisted") is True
            and pick_payload.get("perception_observed_target") is True
            and pick_payload.get("local_alignment_used") is True
            and float(
                (pick_payload.get("local_alignment") or {}).get(
                    "position_error_m", 999.0
                )
            ) < 0.01
            and float(
                (pick_payload.get("local_alignment") or {}).get(
                    "yaw_error_rad", 999.0
                )
            ) < 0.03
            and pick_payload.get("planning_scene_attach_used") is False
            and pick_payload.get("physics_constraint_attach_used") is True
            and pick_payload.get("contact_required") is True
            and pick_payload.get("contact_gated_physical_hold") is True
        )

        path = os.environ.get("HOMEAGENT_PERCEPTION_FETCH_REPORT")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(report, f, ensure_ascii=False, indent=2)

        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 2


def main():
    rclpy.init()
    node = PerceptionFetchProbe()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
