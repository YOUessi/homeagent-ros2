#!/usr/bin/env python3
import json
import sys
from typing import Iterable

import rclpy
from rclpy.node import Node

from homeagent_interfaces.srv import MemoryUpsert


DEFAULT_ENTITIES = [
    {
        "entity_type": "place",
        "entity_id": "place-living-room",
        "name": "living_room",
        "payload": {
            "display_name": "客厅",
            "map_pose": {"x": 0.80, "y": 0.00, "yaw": 0.0},
            "map_frame": "map",
            "kind": "room",
        },
        "confidence": 1.0,
        "source": "home_profile_seed",
    },
    {
        "entity_type": "place",
        "entity_id": "place-kitchen",
        "name": "kitchen",
        "payload": {
            "display_name": "厨房",
            "map_pose": {"x": 1.75, "y": -1.35, "yaw": 0.0},
            "map_frame": "map",
            "kind": "room",
        },
        "confidence": 1.0,
        "source": "home_profile_seed",
    },
    {
        "entity_type": "place",
        "entity_id": "place-bedroom",
        "name": "bedroom",
        "payload": {
            "display_name": "卧室",
            "map_pose": {"x": -1.75, "y": -1.35, "yaw": 0.0},
            "map_frame": "map",
            "kind": "room",
        },
        "confidence": 1.0,
        "source": "home_profile_seed",
    },
    {
        "entity_type": "place",
        "entity_id": "place-hallway",
        "name": "hallway",
        "payload": {
            "display_name": "走廊",
            "map_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "map_frame": "map",
            "kind": "passage",
        },
        "confidence": 1.0,
        "source": "home_profile_seed",
    },
    {
        "entity_type": "place",
        "entity_id": "place-utility-room",
        "name": "utility_room",
        "payload": {
            "display_name": "设备间",
            "map_pose": {"x": 2.2, "y": 2.2, "yaw": 0.0},
            "map_frame": "map",
            "kind": "restricted_room",
        },
        "confidence": 1.0,
        "source": "home_profile_seed",
    },
    {
        "entity_type": "object",
        "entity_id": "object-cup",
        "name": "cup",
        "payload": {"display_name": "水杯", "tags": []},
        "confidence": 0.95,
        "source": "home_profile_seed",
    },
    {
        "entity_type": "object",
        "entity_id": "object-kitchen-knife",
        "name": "kitchen_knife",
        "payload": {"display_name": "菜刀", "tags": ["sharp"]},
        "confidence": 1.0,
        "source": "home_profile_seed",
    },
    {
        "entity_type": "person",
        "entity_id": "person-child",
        "name": "child",
        "payload": {"display_name": "小孩", "age": 10},
        "confidence": 1.0,
        "source": "home_profile_seed",
    },
]


class Seeder(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_memory_seeder")
        self.client = self.create_client(
            MemoryUpsert, "/homeagent/memory/upsert"
        )

    def wait_ready(self, timeout_sec: float = 10.0) -> None:
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("memory upsert service unavailable")

    def upsert(self, record: dict) -> dict:
        request = MemoryUpsert.Request()
        request.entity_type = record["entity_type"]
        request.entity_id = record["entity_id"]
        request.name = record["name"]
        request.payload_json = json.dumps(record["payload"], ensure_ascii=False)
        request.confidence = float(record["confidence"])
        request.source = record["source"]

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError(f"upsert timed out: {record['entity_id']}")
        response = future.result()
        if response is None or not response.success:
            code = getattr(response, "code", "NO_RESPONSE")
            raise RuntimeError(f"upsert failed {record['entity_id']}: {code}")
        return json.loads(response.record_json or "{}")


def seed(records: Iterable[dict]) -> list:
    node = Seeder()
    try:
        node.wait_ready()
        result = []
        for record in records:
            stored = node.upsert(record)
            result.append(
                {
                    "entity_type": stored.get("entity_type"),
                    "entity_id": stored.get("entity_id"),
                    "name": stored.get("name"),
                    "revision": stored.get("revision"),
                }
            )
        return result
    finally:
        node.destroy_node()


def main() -> int:
    rclpy.init()
    try:
        result = seed(DEFAULT_ENTITIES)
        print(
            json.dumps(
                {
                    "seeded": len(result),
                    "entities": result,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0
    except Exception as exc:
        print(json.dumps({"error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
