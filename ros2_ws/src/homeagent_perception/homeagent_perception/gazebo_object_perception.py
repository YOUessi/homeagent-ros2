import json
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

from homeagent_interfaces.srv import MemoryObserve

from .observation import (
    build_map_pose,
    find_model_index,
    perception_payload,
    transform_world_pose_to_map,
)


class GazeboObjectPerception(Node):
    """Simulation perception adapter: Gazebo model state -> Household Memory."""

    def __init__(self) -> None:
        super().__init__("homeagent_gazebo_object_perception")

        self.declare_parameter("model_name", "physical_cup")
        self.declare_parameter("robot_model_name", "homebot_arm")
        self.declare_parameter("entity_id", "object-cup")
        self.declare_parameter("zone_hint", "living_room")
        self.declare_parameter("source", "gazebo_ground_truth_perception")
        self.declare_parameter("min_update_interval_sec", 0.5)

        self._model_name = str(self.get_parameter("model_name").value)
        self._robot_model_name = str(
            self.get_parameter("robot_model_name").value
        )
        self._entity_id = str(self.get_parameter("entity_id").value)
        self._zone = str(self.get_parameter("zone_hint").value)
        self._source = str(self.get_parameter("source").value)
        self._interval = float(
            self.get_parameter("min_update_interval_sec").value
        )

        self._last_submit = 0.0
        self._pending = False
        self._observed_count = 0
        self._amcl_pose = None

        self._memory = self.create_client(
            MemoryObserve, "/homeagent/memory/observe"
        )
        self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self._on_models,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl,
            10,
        )
        self.get_logger().info(
            f"Gazebo object perception ready model={self._model_name} "
            f"robot={self._robot_model_name} entity={self._entity_id}"
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = msg

    @staticmethod
    def _pose_dict(pose_msg, frame: str) -> dict:
        return build_map_pose(
            position=[
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z,
            ],
            quaternion=[
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ],
            frame=frame,
        )

    def _on_models(self, msg: ModelStates) -> None:
        now = time.monotonic()
        if self._pending or now - self._last_submit < self._interval:
            return

        if self._amcl_pose is None:
            return

        object_index = find_model_index(msg.name, self._model_name)
        robot_index = find_model_index(msg.name, self._robot_model_name)
        if (
            object_index is None
            or robot_index is None
            or object_index >= len(msg.pose)
            or robot_index >= len(msg.pose)
        ):
            return

        if not self._memory.service_is_ready():
            self._memory.wait_for_service(timeout_sec=0.05)
        if not self._memory.service_is_ready():
            return

        object_world = self._pose_dict(
            msg.pose[object_index],
            "gazebo_world",
        )
        robot_world = self._pose_dict(
            msg.pose[robot_index],
            "gazebo_world",
        )
        robot_map = self._pose_dict(
            self._amcl_pose.pose.pose,
            "map",
        )
        pose = transform_world_pose_to_map(
            object_world=object_world,
            robot_world=robot_world,
            robot_map=robot_map,
        )

        request = MemoryObserve.Request()
        request.entity_type = "object"
        request.entity_id = self._entity_id
        request.zone = self._zone
        request.pose_json = json.dumps(pose, ensure_ascii=False)
        request.payload_json = json.dumps(
            perception_payload(
                self._model_name,
                pose,
                raw_world_pose=object_world,
            ),
            ensure_ascii=False,
        )
        request.confidence = 1.0
        request.source = self._source

        self._pending = True
        self._last_submit = now
        future = self._memory.call_async(request)
        future.add_done_callback(
            lambda done, observed_pose=pose: self._on_observed(
                done, observed_pose
            )
        )

    def _on_observed(self, future, pose: dict) -> None:
        self._pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"PERCEPTION_MEMORY_FAILED error={exc}")
            return

        if response is None or not response.success:
            code = getattr(response, "code", "NO_RESPONSE")
            self.get_logger().warning(
                f"PERCEPTION_MEMORY_REJECTED code={code}"
            )
            return

        self._observed_count += 1
        if self._observed_count <= 3 or self._observed_count % 10 == 0:
            self.get_logger().info(
                "PERCEPTION_OBSERVE "
                f"model={self._model_name} entity={self._entity_id} "
                f"frame=map x={pose['x']:.3f} y={pose['y']:.3f} "
                f"z={pose['z']:.3f} "
                f"observation_id={response.observation_id}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboObjectPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
