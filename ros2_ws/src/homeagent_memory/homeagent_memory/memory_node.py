import json

import rclpy
from rclpy.node import Node

from homeagent_interfaces.srv import (
    MemoryCorrect,
    MemoryObserve,
    MemoryQuery,
    MemoryUpsert,
)

from .store import MemoryError, MemoryStore


class MemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("homeagent_memory")
        self.declare_parameter("database_path", "/tmp/homeagent_memory.sqlite3")
        database_path = (
            self.get_parameter("database_path").get_parameter_value().string_value
        )
        self._store = MemoryStore(database_path)

        self.create_service(
            MemoryUpsert, "/homeagent/memory/upsert", self._on_upsert
        )
        self.create_service(
            MemoryQuery, "/homeagent/memory/query", self._on_query
        )
        self.create_service(
            MemoryObserve, "/homeagent/memory/observe", self._on_observe
        )
        self.create_service(
            MemoryCorrect, "/homeagent/memory/correct", self._on_correct
        )
        self.get_logger().info(f"HomeAgent memory ready: {database_path}")

    def destroy_node(self):
        self._store.close()
        return super().destroy_node()

    def _on_upsert(self, request, response):
        try:
            record = self._store.upsert_entity(
                entity_type=request.entity_type,
                entity_id=request.entity_id,
                name=request.name,
                payload_json=request.payload_json,
                confidence=request.confidence,
                source=request.source,
            )
            response.success = True
            response.code = "OK"
            response.record_json = json.dumps(record, ensure_ascii=False)
        except MemoryError as exc:
            response.success = False
            response.code = "INVALID_MEMORY"
            response.record_json = json.dumps({"error": str(exc)}, ensure_ascii=False)
        return response

    def _on_query(self, request, response):
        try:
            record = self._store.get_entity(
                entity_type=request.entity_type,
                entity_id=request.entity_id,
                name=request.name,
            )
            response.found = record is not None
            response.code = "OK" if record is not None else "NOT_FOUND"
            response.record_json = json.dumps(record or {}, ensure_ascii=False)
        except MemoryError as exc:
            response.found = False
            response.code = "INVALID_QUERY"
            response.record_json = json.dumps({"error": str(exc)}, ensure_ascii=False)
        return response

    def _on_observe(self, request, response):
        try:
            observation_id = self._store.record_observation(
                entity_type=request.entity_type,
                entity_id=request.entity_id,
                zone=request.zone,
                pose_json=request.pose_json,
                payload_json=request.payload_json,
                confidence=request.confidence,
                source=request.source,
            )
            response.success = True
            response.code = "OK"
            response.observation_id = observation_id
        except MemoryError as exc:
            response.success = False
            response.code = "INVALID_OBSERVATION"
            response.observation_id = str(exc)
        return response

    def _on_correct(self, request, response):
        try:
            record = self._store.apply_correction(
                entity_type=request.entity_type,
                entity_id=request.entity_id,
                field=request.field,
                value_json=request.value_json,
                weight=request.weight,
                source=request.source,
            )
            response.success = True
            response.code = "OK"
            response.record_json = json.dumps(record, ensure_ascii=False)
        except MemoryError as exc:
            response.success = False
            response.code = "INVALID_CORRECTION"
            response.record_json = json.dumps({"error": str(exc)}, ensure_ascii=False)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
