#!/usr/bin/env python3
import json
import math
import time

import rclpy
from gazebo_msgs.msg import LinkStates
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM = ["joint1", "joint2", "joint3", "joint4"]
PICK = [0.20, -0.75, 1.15, -0.35]
BASE_LINK = "homebot_arm::base_footprint"
TOOL_LINK = "homebot_arm::tool_link"
CUP_HALF_HEIGHT = 0.04
DESIRED_OBJECT_IN_TOOL = [0.12, 0.0060, 0.0]


def quat_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]


def pose_parts(pose):
    return (
        [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ],
        [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ],
    )


def transform_point(position, quaternion, point):
    delta = rotate(quaternion, point)
    return [position[i] + delta[i] for i in range(3)]


def relative_pose(parent_pose, child_pose):
    parent_pos, parent_q = pose_parts(parent_pose)
    child_pos, child_q = pose_parts(child_pose)

    inv_q = quat_conj(parent_q)
    delta = [
        child_pos[i] - parent_pos[i]
        for i in range(3)
    ]
    rel_pos = rotate(inv_q, delta)
    rel_q = quat_mul(inv_q, child_q)
    return rel_pos, rel_q


class Probe(Node):
    def __init__(self):
        super().__init__("homeagent_pick_geometry_calibration")
        self.q = {}
        self.link_poses = {}

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
        )
        self.create_subscription(
            LinkStates,
            "/gazebo/link_states",
            self._on_link_states,
            20,
        )
        self.move = ActionClient(self, MoveGroup, "/move_action")

    def _on_joint_state(self, msg):
        for name, value in zip(msg.name, msg.position):
            self.q[name] = float(value)

    def _on_link_states(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if name in {BASE_LINK, TOOL_LINK}:
                self.link_poses[name] = pose

    def wait_ready(self, timeout=18.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                all(name in self.q for name in ARM)
                and BASE_LINK in self.link_poses
                and TOOL_LINK in self.link_poses
                and self.move.server_is_ready()
            ):
                return
        raise RuntimeError(
            "calibration runtime not ready: missing joints/link states/MoveIt"
        )

    def move_pick(self):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20
        goal.request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = "pick_geometry_calibration"
        for name, position in zip(ARM, PICK):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        future = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("calibration MoveIt goal rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=35.0
        )
        wrapped = result_future.result()
        if (
            wrapped is None
            or int(wrapped.status) != 4
            or int(wrapped.result.error_code.val)
            != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError("calibration MoveIt execution failed")


def main():
    rclpy.init()
    node = Probe()
    try:
        node.wait_ready()
        node.move_pick()

        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        base_world = node.link_poses[BASE_LINK]
        tool_world = node.link_poses[TOOL_LINK]

        base_pos, base_q = pose_parts(base_world)
        tool_pos_world, tool_q_world = pose_parts(tool_world)
        tool_pos_base, tool_q_base = relative_pose(
            base_world,
            tool_world,
        )

        object_in_base = transform_point(
            tool_pos_base,
            tool_q_base,
            DESIRED_OBJECT_IN_TOOL,
        )
        object_in_world = transform_point(
            tool_pos_world,
            tool_q_world,
            DESIRED_OBJECT_IN_TOOL,
        )

        support_height = object_in_world[2] - CUP_HALF_HEIGHT

        report = {
            "pick_joint_target": PICK,
            "actual_joint_positions": [node.q[j] for j in ARM],
            "desired_object_in_tool": DESIRED_OBJECT_IN_TOOL,
            "base_in_world": {
                "position": base_pos,
                "quaternion_xyzw": base_q,
            },
            "tool_in_world": {
                "position": tool_pos_world,
                "quaternion_xyzw": tool_q_world,
            },
            "tool_in_base": {
                "position": tool_pos_base,
                "quaternion_xyzw": tool_q_base,
            },
            "calibrated_object_in_base": object_in_base,
            "calibrated_object_in_world_at_initial_base": object_in_world,
            "recommended_mobile_pregrasp_offset": {
                "x": object_in_base[0],
                "y": object_in_base[1],
                "yaw": 0.0,
                "expected_object_z": object_in_world[2],
            },
            "recommended_cup_center_z": object_in_world[2],
            "recommended_support_height": support_height,
        }
        print(json.dumps(report, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
