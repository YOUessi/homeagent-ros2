#!/usr/bin/env python3
import json
import math
import os
import re
import struct
import tempfile
import time
import zlib
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from homeagent_interfaces.msg import ActionProposal, SafetyDecision, SkillResult
from homeagent_perception.observation import build_map_pose, transform_world_pose_to_map


ROOT = Path("/workspace")
OUT = ROOT / "artifacts/live_web"
MAP_PGM = ROOT / "ros2_ws/src/homeagent_navigation/maps/home_room.pgm"
MAP_YAML = ROOT / "ros2_ws/src/homeagent_navigation/maps/home_room.yaml"


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def read_pgm(path):
    data = path.read_bytes()
    i = 0

    def token():
        nonlocal i
        while i < len(data):
            if data[i:i+1] == b"#":
                while i < len(data) and data[i:i+1] not in (b"\n", b"\r"):
                    i += 1
            elif data[i:i+1].isspace():
                i += 1
            else:
                break
        start = i
        while i < len(data) and not data[i:i+1].isspace():
            i += 1
        return data[start:i].decode("ascii")

    if token() != "P5":
        raise RuntimeError("expected P5 map")
    width, height, maxval = int(token()), int(token()), int(token())
    if maxval != 255:
        raise RuntimeError("unsupported map maxval")
    while i < len(data) and data[i:i+1].isspace():
        i += 1
    return width, height, data[i:i+width*height]


def map_meta(path):
    text = path.read_text(encoding="utf-8")
    resolution = float(re.search(r"^resolution:\s*([^\s]+)", text, re.M).group(1))
    m = re.search(r"^origin:\s*\[([^\]]+)\]", text, re.M)
    origin = [float(x.strip()) for x in m.group(1).split(",")]
    return resolution, origin


def write_png(path, width, height, gray):
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        row = gray[y*width:(y+1)*width]
        for v in row:
            raw.extend((v, v, v))

    def chunk(kind, payload):
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff)
        )

    png = bytearray(b"\x89PNG\r\n\x1a\n")
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


class DashboardState(Node):
    def __init__(self):
        super().__init__("homeagent_live_dashboard_state")
        self.odom = None
        self.amcl = None
        self.models = None
        self.plan = None
        self.joints = {}
        self.command = ""
        self.candidate = None
        self.decision = None
        self.skill = None
        self.last_update_wall = time.time()

        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._models, 10)
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(JointState, "/joint_states", self._joints, 20)
        self.create_subscription(String, "/homeagent/user_command", self._command, 10)
        self.create_subscription(ActionProposal, "/homeagent/action_candidate", self._candidate, 10)
        self.create_subscription(SafetyDecision, "/homeagent/action_approved", self._approved, 10)
        self.create_subscription(SafetyDecision, "/homeagent/action_rejected", self._rejected, 10)
        self.create_subscription(SkillResult, "/homeagent/skill_result", self._skill, 10)

        self.create_timer(0.10, self._flush)

    def _odom(self, msg): self.odom = msg
    def _amcl(self, msg): self.amcl = msg
    def _models(self, msg): self.models = msg
    def _plan(self, msg): self.plan = msg
    def _joints(self, msg):
        for n, v in zip(msg.name, msg.position):
            self.joints[n] = float(v)
    def _command(self, msg): self.command = msg.data
    def _candidate(self, msg):
        self.candidate = {
            "request_id": msg.request_id,
            "action": msg.action,
            "params_json": msg.params_json,
        }
    def _approved(self, msg):
        self.decision = {
            "allowed": True,
            "request_id": msg.request_id,
            "action": msg.action,
            "code": msg.code,
            "reason": msg.reason,
        }
    def _rejected(self, msg):
        self.decision = {
            "allowed": False,
            "request_id": msg.request_id,
            "action": msg.action,
            "code": msg.code,
            "reason": msg.reason,
        }
    def _skill(self, msg):
        try:
            payload = json.loads(msg.result_json or "{}")
        except Exception:
            payload = {}
        self.skill = {
            "request_id": msg.request_id,
            "action": msg.action,
            "success": bool(msg.success),
            "code": msg.code,
            "result": payload,
        }

    def _pose_dict(self, pose):
        if pose is None:
            return None
        p = pose.position
        q = pose.orientation
        return {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "yaw": yaw_from_quat(q),
        }

    def _flush(self):
        robot = None
        if self.amcl is not None:
            robot = self._pose_dict(self.amcl.pose.pose)
        elif self.odom is not None:
            robot = self._pose_dict(self.odom.pose.pose)

        cup = None
        support = None
        robot_world = None
        cup_world = None
        support_world = None
        if self.models is not None:
            for name, pose in zip(self.models.name, self.models.pose):
                if name == "homebot_arm":
                    robot_world = build_map_pose(
                        position=[
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                        ],
                        quaternion=[
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ],
                        frame="gazebo_world",
                    )
                elif name == "physical_cup":
                    cup_world = build_map_pose(
                        position=[
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                        ],
                        quaternion=[
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ],
                        frame="gazebo_world",
                    )
                elif name == "fetch_support":
                    support_world = build_map_pose(
                        position=[
                            pose.position.x,
                            pose.position.y,
                            pose.position.z,
                        ],
                        quaternion=[
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ],
                        frame="gazebo_world",
                    )

        if self.amcl is not None and robot_world is not None:
            robot_map = self._pose_dict(self.amcl.pose.pose)
            robot_map["frame"] = "map"
            if cup_world is not None:
                cup = transform_world_pose_to_map(
                    object_world=cup_world,
                    robot_world=robot_world,
                    robot_map=robot_map,
                )
            if support_world is not None:
                support = transform_world_pose_to_map(
                    object_world=support_world,
                    robot_world=robot_world,
                    robot_map=robot_map,
                )
        else:
            cup = cup_world
            support = support_world

        path = []
        if self.plan is not None:
            path = [
                [float(p.pose.position.x), float(p.pose.position.y)]
                for p in self.plan.poses
            ]

        pregrasp = None
        if cup is not None:
            yaw = float(cup["yaw"])
            c = math.cos(yaw)
            s = math.sin(yaw)
            dx = c * 0.54806 - s * 0.142303
            dy = s * 0.54806 + c * 0.142303
            pregrasp = {
                "x": cup["x"] - dx,
                "y": cup["y"] - dy,
                "yaw": yaw,
            }

        state = {
            "timestamp": time.time(),
            "command": self.command,
            "robot": robot,
            "cup": cup,
            "support": support,
            "pregrasp": pregrasp,
            "path": path,
            "joints": {
                k: self.joints.get(k)
                for k in ["joint1", "joint2", "joint3", "joint4", "gripper_joint"]
            },
            "candidate": self.candidate,
            "decision": self.decision,
            "skill": self.skill,
        }

        OUT.mkdir(parents=True, exist_ok=True)
        tmp = OUT / "state.json.tmp"
        tmp.write_text(json.dumps(state, ensure_ascii=False), encoding="utf-8")
        os.replace(tmp, OUT / "state.json")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    width, height, gray = read_pgm(MAP_PGM)
    write_png(OUT / "map.png", width, height, gray)
    resolution, origin = map_meta(MAP_YAML)
    (OUT / "map_meta.json").write_text(
        json.dumps(
            {
                "width": width,
                "height": height,
                "resolution": resolution,
                "origin": origin,
            }
        ),
        encoding="utf-8",
    )

    rclpy.init()
    node = DashboardState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
