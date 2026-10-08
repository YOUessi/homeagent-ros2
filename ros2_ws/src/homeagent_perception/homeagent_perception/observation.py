import math
from typing import Dict, Iterable, Optional, Sequence


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def build_map_pose(
    *,
    position: Sequence[float],
    quaternion: Sequence[float],
    frame: str = "map",
) -> Dict[str, float]:
    if len(position) != 3 or len(quaternion) != 4:
        raise ValueError("position must be xyz and quaternion must be xyzw")

    values = [float(v) for v in list(position) + list(quaternion)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("pose contains non-finite values")

    x, y, z, qx, qy, qz, qw = values
    return {
        "frame": frame,
        "x": x,
        "y": y,
        "z": z,
        "yaw": quaternion_to_yaw(qx, qy, qz, qw),
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
    }


def transform_world_pose_to_map(
    *,
    object_world: Dict[str, float],
    robot_world: Dict[str, float],
    robot_map: Dict[str, float],
) -> Dict[str, float]:
    """Project a Gazebo-world object pose into the localized ROS map frame.

    The robot is the shared reference observed in both frames:
        T_map_object = T_map_robot * inv(T_world_robot) * T_world_object

    Only planar x/y/yaw are transformed because Nav2's map is 2D. The
    physical z height is preserved from Gazebo world, whose ground plane is
    also z=0 in this simulation.
    """
    required = ("x", "y", "z", "yaw")
    for pose in (object_world, robot_world, robot_map):
        if not all(k in pose for k in required):
            raise ValueError("pose is missing x/y/z/yaw")
        if not all(math.isfinite(float(pose[k])) for k in required):
            raise ValueError("pose contains non-finite values")

    world_yaw = float(robot_world["yaw"])
    map_yaw = float(robot_map["yaw"])

    dx = float(object_world["x"]) - float(robot_world["x"])
    dy = float(object_world["y"]) - float(robot_world["y"])

    cw = math.cos(world_yaw)
    sw = math.sin(world_yaw)
    rel_x = cw * dx + sw * dy
    rel_y = -sw * dx + cw * dy

    cm = math.cos(map_yaw)
    sm = math.sin(map_yaw)
    map_x = float(robot_map["x"]) + cm * rel_x - sm * rel_y
    map_y = float(robot_map["y"]) + sm * rel_x + cm * rel_y

    object_map_yaw = wrap_angle(
        map_yaw
        + wrap_angle(float(object_world["yaw"]) - world_yaw)
    )

    return {
        "frame": "map",
        "x": map_x,
        "y": map_y,
        "z": float(object_world["z"]),
        "yaw": object_map_yaw,
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(object_map_yaw / 2.0),
        "qw": math.cos(object_map_yaw / 2.0),
    }


def find_model_index(names: Iterable[str], model_name: str) -> Optional[int]:
    for index, name in enumerate(names):
        if name == model_name:
            return index
    return None


def perception_payload(
    model_name: str,
    pose: Dict[str, float],
    *,
    raw_world_pose: Optional[Dict[str, float]] = None,
) -> Dict:
    payload = {
        "perception": {
            "backend": "gazebo_ground_truth",
            "model_name": model_name,
            "frame": pose["frame"],
            "frame_alignment": "robot_world_to_map_via_amcl",
        }
    }
    if raw_world_pose is not None:
        payload["perception"]["raw_world_pose"] = raw_world_pose
    return payload
