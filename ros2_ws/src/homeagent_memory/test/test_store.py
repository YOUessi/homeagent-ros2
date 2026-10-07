import json

from homeagent_memory.store import MemoryStore


def test_upsert_and_query(tmp_path):
    store = MemoryStore(str(tmp_path / "memory.sqlite3"))
    record = store.upsert_entity(
        entity_type="object",
        entity_id="cup-1",
        name="水杯",
        payload_json=json.dumps({"color": "blue"}),
        confidence=0.7,
        source="vision",
    )
    assert record["revision"] == 1

    loaded = store.get_entity(entity_type="object", entity_id="cup-1")
    assert loaded["name"] == "水杯"
    assert loaded["payload"]["color"] == "blue"
    store.close()


def test_observation_updates_location(tmp_path):
    store = MemoryStore(str(tmp_path / "memory.sqlite3"))
    observation_id = store.record_observation(
        entity_type="object",
        entity_id="cup-1",
        zone="living_room",
        pose_json=json.dumps({"x": 1.2, "y": -0.4}),
        payload_json=json.dumps({"color": "blue"}),
        confidence=0.82,
        source="camera",
    )
    assert observation_id

    loaded = store.get_entity(entity_type="object", entity_id="cup-1")
    assert loaded["payload"]["location"]["zone"] == "living_room"
    assert loaded["payload"]["location"]["pose"]["x"] == 1.2
    assert loaded["confidence"] == 0.82
    store.close()


def test_human_correction_is_auditable(tmp_path):
    store = MemoryStore(str(tmp_path / "memory.sqlite3"))
    store.record_observation(
        entity_type="object",
        entity_id="cup-1",
        zone="living_room",
        pose_json=json.dumps({"x": 0.0, "y": 0.0}),
        payload_json="{}",
        confidence=0.6,
        source="camera",
    )

    corrected = store.apply_correction(
        entity_type="object",
        entity_id="cup-1",
        field="location.zone",
        value_json=json.dumps("kitchen"),
        weight=0.95,
        source="human_correction",
    )

    assert corrected["payload"]["location"]["zone"] == "kitchen"
    assert corrected["payload"]["_memory_meta"]["location.zone"]["weight"] == 0.95
    assert corrected["confidence"] == 0.95
    assert corrected["revision"] == 2
    store.close()
