#!/usr/bin/env python3
"""True HomeAgent(Safety + Memory) -> Panda MoveIt + Nav2 + Gazebo E2E.

No claimed grasp here: only safe named Panda arm motions and semantic navigation.
"""
import json
import math
import os
import time
import uuid

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from homeagent_interfaces.msg import ActionProposal, SafetyDecision, SkillResult
from panda_gazebo_physics_e2e import PhysicalPandaProbe, sample_state
from panda_moveit_joint_demo import READY


class PandaAgentNavProbe(PhysicalPandaProbe):
    def __init__(self):
        super().__init__()
        self.odom = None
        self.amcl = None
        self.scan = None
        self.model_pose_samples = 0
        self.peak_abs_model_roll_pitch_rad = 0.0
        self.results = {}
        self.result_by_request = {}
        self.approvals = {}
        self.rejections = {}
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl, 10
        )
        self.create_subscription(LaserScan, "/scan", self._scan, 10)
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self._model_state, 10
        )
        self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._result, 20
        )
        self.create_subscription(
            SafetyDecision, "/homeagent/action_approved", self._approved, 20
        )
        self.create_subscription(
            SafetyDecision, "/homeagent/action_rejected", self._rejected, 20
        )
        self.command_pub = self.create_publisher(
            String, "/homeagent/user_command", 10
        )
        self.candidate_pub = self.create_publisher(
            ActionProposal, "/homeagent/action_candidate", 10
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.controller_state_client = self.create_client(
            GetState, "/controller_server/get_state"
        )
        self.navigator_state_client = self.create_client(
            GetState, "/bt_navigator/get_state"
        )

    def _odom(self, msg):
        self.odom = msg

    def _amcl(self, msg):
        self.amcl = msg

    def _scan(self, msg):
        self.scan = msg

    def _model_state(self, msg):
        try:
            index = msg.name.index("homebot_panda")
        except ValueError:
            return
        q = msg.pose[index].orientation
        roll = math.atan2(
            2 * (q.w * q.x + q.y * q.z),
            1 - 2 * (q.x * q.x + q.y * q.y),
        )
        pitch = math.asin(max(
            -1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))
        ))
        self.model_pose_samples += 1
        self.peak_abs_model_roll_pitch_rad = max(
            self.peak_abs_model_roll_pitch_rad,
            abs(roll), abs(pitch),
        )

    def _result(self, msg):
        self.results[msg.action] = msg
        self.result_by_request[msg.request_id] = msg

    def _approved(self, msg):
        self.approvals[msg.request_id] = msg

    def _rejected(self, msg):
        self.rejections[msg.request_id] = msg

    def wait_runtime(self):
        limit = time.monotonic() + 30.0
        while time.monotonic() < limit:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.odom is not None and self.scan is not None
                    and self.command_pub.get_subscription_count() > 0
                    and self.initial_pose_pub.get_subscription_count() > 0):
                return
        raise RuntimeError(
            "runtime readiness failed: "
            f"odom={self.odom is not None} "
            f"scan={self.scan is not None} "
            f"planner_subs={self.command_pub.get_subscription_count()} "
            f"amcl_initialpose_subs={self.initial_pose_pub.get_subscription_count()}"
        )

    @staticmethod
    def yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _lifecycle_is_active(self, client):
        if not client.service_is_ready():
            return False
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.8)
        try:
            return (
                future.done()
                and future.result() is not None
                and int(future.result().current_state.id) == 3
            )
        except Exception:
            return False

    def initialize_localization(self):
        if self.odom is None:
            raise RuntimeError("cannot localize without odom")

        # Nav2's costmap may itself require map->odom before it can activate.
        # Waiting for controller Active before the first initialpose would
        # deadlock activation. AMCL is already Active in the staged launcher;
        # publish localization once, THEN wait for downstream Nav2 lifecycle.
        print("AMCL_READY_INITIALPOSE_FIRST", flush=True)
        pose = self.odom.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = self.yaw(pose.orientation)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x = x
        initial.pose.pose.position.y = y
        initial.pose.pose.orientation.z = math.sin(yaw / 2)
        initial.pose.pose.orientation.w = math.cos(yaw / 2)
        initial.pose.covariance[0] = 0.05
        initial.pose.covariance[7] = 0.05
        initial.pose.covariance[35] = 0.02

        # Do not repeatedly reset AMCL while Nav2 is activating. Publish once,
        # then retry only if no AMCL/TF has appeared after several seconds.
        self.amcl = None
        deadline = time.monotonic() + 30.0
        last_publish = 0.0
        count = 0
        stable = 0
        first_valid = None
        last_log = 0.0
        latest_xy = None
        while time.monotonic() < deadline:
            now = time.monotonic()
            if count == 0 or (
                self.amcl is None
                and now - last_publish >= 5.0
                and count < 3
            ):
                initial.header.stamp = self.odom.header.stamp
                self.initial_pose_pub.publish(initial)
                last_publish = now
                count += 1
                print(f"AMCL_INITIALPOSE_PUBLISHED count={count}", flush=True)

            rclpy.spin_once(self, timeout_sec=0.12)
            fresh = False
            if self.amcl is not None:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        "map", "base_footprint", Time()
                    )
                    self.tf_buffer.lookup_transform("map", "odom", Time())
                    latest_xy = [
                        float(tf.transform.translation.x),
                        float(tf.transform.translation.y),
                    ]
                    stamp = (
                        float(tf.header.stamp.sec)
                        + float(tf.header.stamp.nanosec) * 1e-9
                    )
                    odom_stamp = (
                        float(self.odom.header.stamp.sec)
                        + float(self.odom.header.stamp.nanosec) * 1e-9
                    )
                    fresh = abs(stamp - odom_stamp) < 1.5
                except TransformException:
                    pass

            controller_active = (
                self._lifecycle_is_active(self.controller_state_client)
                if fresh else False
            )
            navigator_active = (
                self._lifecycle_is_active(self.navigator_state_client)
                if controller_active else False
            )
            nav_action_ready = (
                self.nav_client.server_is_ready()
                if navigator_active else False
            )

            if fresh and controller_active and navigator_active and nav_action_ready:
                stable += 1
                if first_valid is None:
                    first_valid = time.monotonic()
                if stable >= 12 and time.monotonic() - first_valid >= 2.0:
                    # A second quiet observation window after the last
                    # initialpose ensures that the pose was not ephemeral.
                    sample_state(self, seconds=1.0)
                    try:
                        self.tf_buffer.lookup_transform(
                            "map", "base_footprint", Time()
                        )
                    except TransformException:
                        stable = 0
                        continue
                    print(
                        "LOCALIZATION_READY " + json.dumps({
                            "initialpose_publications": count,
                            "controller_server_active": True,
                            "bt_navigator_active": True,
                            "map_xy": latest_xy,
                            "tf_timestamp_fresh": True,
                            "stable_tf_observations": stable,
                        }),
                        flush=True,
                    )
                    return latest_xy
            else:
                stable = 0
                first_valid = None

            now = time.monotonic()
            if now - last_log > 4.0:
                print(
                    "LOCALIZATION_WAIT " + json.dumps({
                        "published": count,
                        "amcl_received": self.amcl is not None,
                        "tf_fresh": fresh,
                        "controller_active": controller_active,
                        "navigator_active": navigator_active,
                        "nav_action_ready": nav_action_ready,
                        "stable_observations": stable,
                    }),
                    flush=True,
                )
                last_log = now
        raise RuntimeError(
            f"AMCL map TF did not stabilize: publications={count}, stable={stable}"
        )

    def command(self, text, result_action, timeout=45.0):
        self.results.pop(result_action, None)
        message = String()
        message.data = text
        self.command_pub.publish(message)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_action in self.results:
                return self.results[result_action]
        raise RuntimeError(f"timed out waiting for {result_action}: {text}")

    def reject_spoofed_forbidden_navigation(self):
        """An untrusted candidate cannot erase forbidden-zone policy."""
        if self.candidate_pub.get_subscription_count() < 1:
            raise RuntimeError("trusted Context Resolver not subscribed")

        rid = "panda-forbidden-" + uuid.uuid4().hex[:10]
        proposal = ActionProposal()
        proposal.request_id = rid
        proposal.action = "navigate"
        proposal.params_json = json.dumps({"target": "utility_room"})
        proposal.context_json = json.dumps({
            "safety_context_trusted": True,
            "forbidden_zones": [],
            "resolved_target_pose": [0.0, 0.0, 0.0],
        })
        proposal.source = "integration-negative-test"

        start = self.odom.pose.pose.position
        start_xy = [float(start.x), float(start.y)]
        self.candidate_pub.publish(proposal)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if rid in self.rejections:
                break
        sample_state(self, seconds=0.6)
        end = self.odom.pose.pose.position
        drift = math.hypot(
            float(end.x) - start_xy[0], float(end.y) - start_xy[1]
        )
        rejected = self.rejections.get(rid)
        return {
            "request_id": rid,
            "context_spoof_attempted": True,
            "rejected": bool(rejected is not None),
            "code": rejected.code if rejected is not None else None,
            "illegal_skill_executed": rid in self.result_by_request,
            "base_drift_after_reject_m": drift,
            "passed": (
                rejected is not None
                and rejected.code == "FORBIDDEN_ZONE"
                and rid not in self.result_by_request
                and drift < 0.05
            ),
        }

    def model_tilt(self):
        from gazebo_msgs.srv import GetEntityState
        req = GetEntityState.Request()
        req.name = "homebot_panda"
        req.reference_frame = "world"
        future = self.entity_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("missing Gazebo homebot_panda entity state")
        q = response.state.pose.orientation
        roll = math.atan2(
            2 * (q.w * q.x + q.y * q.z),
            1 - 2 * (q.x * q.x + q.y * q.y),
        )
        pitch = math.asin(max(-1.0, min(
            1.0, 2 * (q.w * q.y - q.z * q.x)
        )))
        return [float(roll), float(pitch)]

    def run(self):
        self.wait_ready()
        self.wait_runtime()

        bootstrap_code = self.initialize_ready()
        if bootstrap_code != 0:
            raise RuntimeError("initial Panda physical joint motion failed")
        baseline_joints = sample_state(self)
        bootstrap_error = max(
            abs(a - b) for a, b in zip(baseline_joints, READY)
        )
        if bootstrap_error > 0.07:
            raise RuntimeError("Panda failed to reach safe initial posture")

        initial_map_xy = self.initialize_localization()
        initial_odometry = self.odom.pose.pose.position
        initial_xy = [
            float(initial_odometry.x), float(initial_odometry.y)
        ]

        look_result = self.command(
            "机械臂检查一下", "look_at", timeout=35.0
        )
        inspect_joints = sample_state(self)
        motion_l2 = math.sqrt(
            sum((a - b)**2 for a, b in zip(
                inspect_joints, baseline_joints
            ))
        )
        # Safety negative test: while the 7-axis Panda is extended in inspect
        # posture, an otherwise approved 'go to living room' must not move the
        # physical base. The hardware-bound Nav2 skill guards against it.
        pre_block = self.odom.pose.pose.position
        pre_block_xy = [float(pre_block.x), float(pre_block.y)]
        blocked_navigation = self.command(
            "去客厅", "navigate", timeout=15.0
        )
        sample_state(self, seconds=0.3)
        post_block = self.odom.pose.pose.position
        blocked_displacement = math.hypot(
            float(post_block.x)-pre_block_xy[0],
            float(post_block.y)-pre_block_xy[1],
        )
        blocked_evidence = {
            "action": "navigate",
            "code": blocked_navigation.code,
            "success": bool(blocked_navigation.success),
            "safety_approved": (
                blocked_navigation.request_id in self.approvals
            ),
            "physical_base_displacement_m": blocked_displacement,
            "passed": (
                not blocked_navigation.success
                and blocked_navigation.code == "PANDA_ARM_NOT_STOWED"
                and blocked_displacement < 0.05
            ),
        }
        print("PANDA_STOW_NAV_BLOCK", json.dumps(blocked_evidence), flush=True)

        stow_result = self.command(
            "收拢机械臂", "stow_arm", timeout=35.0
        )
        stow_joints = sample_state(self)
        stow_error = max(
            abs(a - b) for a, b in zip(stow_joints, READY)
        )
        print("PANDA_AGENT_ARM_STOW", json.dumps({
            "look_success": bool(look_result.success),
            "stow_success": bool(stow_result.success),
            "actual_joint_movement_l2_rad": motion_l2,
            "stow_max_error_rad": stow_error,
        }), flush=True)

        navigation_result = self.command(
            "去客厅", "navigate", timeout=70.0
        )
        sample_state(self)
        final_odometry = self.odom.pose.pose.position
        final_xy = [
            float(final_odometry.x), float(final_odometry.y)
        ]
        distance = math.hypot(
            final_xy[0] - initial_xy[0],
            final_xy[1] - initial_xy[1],
        )
        nav_data = json.loads(navigation_result.result_json or "{}")

        forbidden_probe = self.reject_spoofed_forbidden_navigation()
        all_results = [look_result, stow_result, navigation_result]
        approvals = {
            result.action: (
                result.request_id in self.approvals
                and bool(self.approvals[result.request_id].allowed)
            ) for result in all_results
        }
        report = {
            "robot": "homebot_panda, one Gazebo physics model",
            "model_control": "PandaGazeboSystem; HomeBot Gazebo diff-drive",
            "bootstrap_direct_controller_only": True,
            "bootstrap_joint_error_rad": bootstrap_error,
            "source_command_look_at": "机械臂检查一下",
            "source_command_stow": "收拢机械臂",
            "source_command_navigate": "去客厅",
            "moveit_look_at_code": look_result.code,
            "moveit_look_at": json.loads(look_result.result_json or "{}"),
            "arm_motion_l2_rad": motion_l2,
            "stow_code": stow_result.code,
            "stow_joint_max_error_rad": stow_error,
            "nonstowed_navigation_block": blocked_evidence,
            "navigation_code": navigation_result.code,
            "navigation": nav_data,
            "initial_map_xy": initial_map_xy,
            "initial_odom_xy": initial_xy,
            "final_odom_xy": final_xy,
            "base_displacement_m": distance,
            "scan_samples": len(self.scan.ranges),
            "model_roll_pitch_rad": self.model_tilt(),
            "peak_abs_model_roll_pitch_rad": self.peak_abs_model_roll_pitch_rad,
            "model_pose_samples": self.model_pose_samples,
            "safety_approvals_correlated": approvals,
            "forbidden_zone_spoofing_probe": forbidden_probe,
            "grasp_tested": False,
            "passed": (
                all(bool(r.success) for r in all_results)
                and all(approvals.values())
                and bool(forbidden_probe["passed"])
                and bool(blocked_evidence["passed"])
                and look_result.code == "MOVEIT_SUCCEEDED"
                and stow_result.code == "MOVEIT_SUCCEEDED"
                and bool(json.loads(look_result.result_json or "{}").get(
                    "postcondition_verified"
                ))
                and motion_l2 > 0.2
                and stow_error < 0.06
                and navigation_result.code == "NAV2_SUCCEEDED"
                and nav_data.get("target_source") == "place_memory"
                and bool(nav_data.get("postcondition_verified"))
                and isinstance(nav_data.get("goal_position_error_m"), (int, float))
                and nav_data["goal_position_error_m"] < 0.15
                and distance > 0.4
                and len(self.scan.ranges) >= 180
                and max(map(abs, self.model_tilt())) < 0.15
                and self.model_pose_samples > 20
                and self.peak_abs_model_roll_pitch_rad < 0.30
            ),
        }
        path = os.environ.get(
            "HOMEAGENT_PANDA_AGENT_REPORT",
            "/workspace/artifacts/panda_agent_nav/e2e_report.json",
        )
        with open(path, "w", encoding="utf-8") as fp:
            json.dump(report, fp, ensure_ascii=False, indent=2)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        return 0 if report["passed"] else 2


def main():
    rclpy.init()
    probe = PandaAgentNavProbe()
    try:
        return probe.run()
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
