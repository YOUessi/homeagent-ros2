import json
import sqlite3
import time
import uuid
from pathlib import Path
from typing import Any, Dict, Optional


class MemoryError(ValueError):
    pass


class MemoryStore:
    def __init__(self, database_path: str) -> None:
        path = Path(database_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(str(path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._init_schema()

    def close(self) -> None:
        self._conn.close()

    def _init_schema(self) -> None:
        self._conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS entities (
                entity_type TEXT NOT NULL,
                entity_id TEXT NOT NULL,
                name TEXT NOT NULL DEFAULT '',
                payload_json TEXT NOT NULL DEFAULT '{}',
                confidence REAL NOT NULL DEFAULT 0.0,
                source TEXT NOT NULL DEFAULT '',
                revision INTEGER NOT NULL DEFAULT 1,
                updated_at REAL NOT NULL,
                PRIMARY KEY (entity_type, entity_id)
            );

            CREATE TABLE IF NOT EXISTS observations (
                observation_id TEXT PRIMARY KEY,
                entity_type TEXT NOT NULL,
                entity_id TEXT NOT NULL,
                zone TEXT NOT NULL DEFAULT '',
                pose_json TEXT NOT NULL DEFAULT '{}',
                payload_json TEXT NOT NULL DEFAULT '{}',
                confidence REAL NOT NULL DEFAULT 0.0,
                source TEXT NOT NULL DEFAULT '',
                observed_at REAL NOT NULL
            );

            CREATE TABLE IF NOT EXISTS corrections (
                correction_id TEXT PRIMARY KEY,
                entity_type TEXT NOT NULL,
                entity_id TEXT NOT NULL,
                field TEXT NOT NULL,
                value_json TEXT NOT NULL,
                weight REAL NOT NULL,
                source TEXT NOT NULL DEFAULT '',
                created_at REAL NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_entities_name
                ON entities(entity_type, name);
            CREATE INDEX IF NOT EXISTS idx_observations_entity_time
                ON observations(entity_type, entity_id, observed_at DESC);
            """
        )
        self._conn.commit()

    @staticmethod
    def _confidence(value: float, field: str = "confidence") -> float:
        value = float(value)
        if not 0.0 <= value <= 1.0:
            raise MemoryError(f"{field} must be in [0, 1]")
        return value

    @staticmethod
    def _json_object(raw: str, field: str) -> Dict[str, Any]:
        try:
            value = json.loads(raw or "{}")
        except json.JSONDecodeError as exc:
            raise MemoryError(f"{field} is invalid JSON: {exc}") from exc
        if not isinstance(value, dict):
            raise MemoryError(f"{field} must be a JSON object")
        return value

    @staticmethod
    def _json_any(raw: str, field: str) -> Any:
        try:
            return json.loads(raw)
        except json.JSONDecodeError as exc:
            raise MemoryError(f"{field} is invalid JSON: {exc}") from exc

    def upsert_entity(
        self,
        *,
        entity_type: str,
        entity_id: str,
        name: str,
        payload_json: str,
        confidence: float,
        source: str,
    ) -> Dict[str, Any]:
        if not entity_type or not entity_id:
            raise MemoryError("entity_type and entity_id are required")

        payload = self._json_object(payload_json, "payload_json")
        confidence = self._confidence(confidence)
        now = time.time()
        current = self.get_entity(entity_type=entity_type, entity_id=entity_id)

        if current is None:
            revision = 1
            final_name = name or entity_id
        else:
            revision = int(current["revision"]) + 1
            final_name = name or current["name"]

        self._conn.execute(
            """
            INSERT INTO entities(
                entity_type, entity_id, name, payload_json,
                confidence, source, revision, updated_at
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(entity_type, entity_id) DO UPDATE SET
                name=excluded.name,
                payload_json=excluded.payload_json,
                confidence=excluded.confidence,
                source=excluded.source,
                revision=excluded.revision,
                updated_at=excluded.updated_at
            """,
            (
                entity_type,
                entity_id,
                final_name,
                json.dumps(payload, ensure_ascii=False),
                confidence,
                source,
                revision,
                now,
            ),
        )
        self._conn.commit()
        return self.get_entity(entity_type=entity_type, entity_id=entity_id)

    def get_entity(
        self,
        *,
        entity_type: str,
        entity_id: str = "",
        name: str = "",
    ) -> Optional[Dict[str, Any]]:
        if not entity_type:
            raise MemoryError("entity_type is required")
        if entity_id:
            row = self._conn.execute(
                """
                SELECT * FROM entities
                WHERE entity_type=? AND entity_id=?
                """,
                (entity_type, entity_id),
            ).fetchone()
        elif name:
            row = self._conn.execute(
                """
                SELECT * FROM entities
                WHERE entity_type=? AND name=?
                ORDER BY updated_at DESC
                LIMIT 1
                """,
                (entity_type, name),
            ).fetchone()
        else:
            raise MemoryError("entity_id or name is required")

        if row is None:
            return None
        return self._row_to_record(row)

    def record_observation(
        self,
        *,
        entity_type: str,
        entity_id: str,
        zone: str,
        pose_json: str,
        payload_json: str,
        confidence: float,
        source: str,
    ) -> str:
        if not entity_type or not entity_id:
            raise MemoryError("entity_type and entity_id are required")

        pose = self._json_any(pose_json or "{}", "pose_json")
        payload = self._json_object(payload_json, "payload_json")
        confidence = self._confidence(confidence)
        now = time.time()
        observation_id = str(uuid.uuid4())

        current = self.get_entity(entity_type=entity_type, entity_id=entity_id)
        merged = dict(current["payload"] if current else {})
        merged.update(payload)
        merged["location"] = {
            "zone": zone,
            "pose": pose,
            "observed_at": now,
        }

        self.upsert_entity(
            entity_type=entity_type,
            entity_id=entity_id,
            name=current["name"] if current else entity_id,
            payload_json=json.dumps(merged, ensure_ascii=False),
            confidence=max(current["confidence"] if current else 0.0, confidence),
            source=source,
        )

        self._conn.execute(
            """
            INSERT INTO observations(
                observation_id, entity_type, entity_id, zone, pose_json,
                payload_json, confidence, source, observed_at
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                observation_id,
                entity_type,
                entity_id,
                zone,
                json.dumps(pose, ensure_ascii=False),
                json.dumps(payload, ensure_ascii=False),
                confidence,
                source,
                now,
            ),
        )
        self._conn.commit()
        return observation_id

    def apply_correction(
        self,
        *,
        entity_type: str,
        entity_id: str,
        field: str,
        value_json: str,
        weight: float,
        source: str,
    ) -> Dict[str, Any]:
        if not field:
            raise MemoryError("field is required")
        weight = self._confidence(weight, "weight")
        value = self._json_any(value_json, "value_json")
        current = self.get_entity(entity_type=entity_type, entity_id=entity_id)
        if current is None:
            raise MemoryError("entity does not exist")

        now = time.time()
        payload = dict(current["payload"])
        self._set_dotted_path(payload, field, value)
        meta = payload.setdefault("_memory_meta", {})
        meta[field] = {
            "source": source or "human_correction",
            "weight": weight,
            "updated_at": now,
        }

        record = self.upsert_entity(
            entity_type=entity_type,
            entity_id=entity_id,
            name=current["name"],
            payload_json=json.dumps(payload, ensure_ascii=False),
            confidence=max(float(current["confidence"]), weight),
            source=source or "human_correction",
        )

        self._conn.execute(
            """
            INSERT INTO corrections(
                correction_id, entity_type, entity_id, field,
                value_json, weight, source, created_at
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                str(uuid.uuid4()),
                entity_type,
                entity_id,
                field,
                json.dumps(value, ensure_ascii=False),
                weight,
                source or "human_correction",
                now,
            ),
        )
        self._conn.commit()
        return record

    @staticmethod
    def _set_dotted_path(payload: Dict[str, Any], field: str, value: Any) -> None:
        parts = [part for part in field.split(".") if part]
        if not parts:
            raise MemoryError("field is required")
        cursor = payload
        for part in parts[:-1]:
            existing = cursor.get(part)
            if not isinstance(existing, dict):
                existing = {}
                cursor[part] = existing
            cursor = existing
        cursor[parts[-1]] = value

    @staticmethod
    def _row_to_record(row: sqlite3.Row) -> Dict[str, Any]:
        return {
            "entity_type": row["entity_type"],
            "entity_id": row["entity_id"],
            "name": row["name"],
            "payload": json.loads(row["payload_json"]),
            "confidence": float(row["confidence"]),
            "source": row["source"],
            "revision": int(row["revision"]),
            "updated_at": float(row["updated_at"]),
        }
