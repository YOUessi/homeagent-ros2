from homeagent_context.resolver import resolve_trusted_context


def test_navigation_pose_comes_from_place_memory():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "living_room"},
        place_record={
            "entity_id": "place-living-room",
            "name": "living_room",
            "payload": {"map_pose": {"x": 0.8, "y": 0.0, "yaw": 0.0}},
            "confidence": 1.0,
        },
        forbidden_zones=["utility_room"],
    )
    assert context["safety_context_trusted"]
    assert context["forbidden_zones"] == ["utility_room"]
    assert context["resolved_target_pose"] == [0.8, 0.0, 0.0]
    assert context["place_id"] == "place-living-room"


def test_navigation_unknown_place_is_untrusted():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "garage"},
        place_record=None,
        forbidden_zones=["utility_room"],
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "PLACE_NOT_IN_MEMORY"
    assert context["forbidden_zones"] == ["utility_room"]


def test_navigation_invalid_place_pose_is_untrusted():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "living_room"},
        place_record={
            "entity_id": "bad-place",
            "name": "living_room",
            "payload": {"map_pose": {"x": "bad", "y": 0.0, "yaw": 0.0}},
            "confidence": 1.0,
        },
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "PLACE_POSE_INVALID"


def test_pick_unknown_object_is_untrusted():
    context = resolve_trusted_context(
        action="pick",
        params={"object": "cup"},
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "OBJECT_NOT_IN_MEMORY"


def test_handover_uses_memory_not_agent_claims():
    object_record = {
        "payload": {"tags": ["sharp"]},
        "confidence": 0.91,
    }
    person_record = {
        "payload": {"age": 10},
        "confidence": 1.0,
    }
    context = resolve_trusted_context(
        action="handover",
        params={"object": "kitchen_knife", "recipient": "child"},
        object_record=object_record,
        person_record=person_record,
    )
    assert context["safety_context_trusted"]
    assert context["object_tags"] == ["sharp"]
    assert context["recipient_age"] == 10


def test_handover_missing_age_is_untrusted():
    context = resolve_trusted_context(
        action="handover",
        params={"object": "cup", "recipient": "visitor"},
        object_record={"payload": {"tags": []}, "confidence": 0.8},
        person_record={"payload": {}, "confidence": 0.5},
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "RECIPIENT_AGE_UNKNOWN"


def test_pick_approach_comes_from_object_memory():
    context = resolve_trusted_context(
        action="pick",
        params={"object": "cup"},
        object_record={
            "entity_id": "object-cup",
            "name": "cup",
            "payload": {
                "tags": [],
                "manipulation": {
                    "pick_approach_joint_target": [0.2, -0.75, 1.15, -0.35]
                },
            },
            "confidence": 0.95,
        },
    )
    assert context["safety_context_trusted"]
    assert context["resolved_arm_joint_target"] == [0.2, -0.75, 1.15, -0.35]
    assert context["manipulation_stage"] == "pick_approach"
    assert context["manipulation_source"] == "object_memory"


def test_pick_known_object_without_approach_is_untrusted():
    context = resolve_trusted_context(
        action="pick",
        params={"object": "cup"},
        object_record={
            "entity_id": "object-cup",
            "name": "cup",
            "payload": {"tags": []},
            "confidence": 0.95,
        },
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "PICK_APPROACH_UNKNOWN"


def test_object_pregrasp_uses_fresh_perception_pose():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "object_pregrasp:cup"},
        object_record={
            "entity_id": "object-cup",
            "name": "cup",
            "payload": {
                "location": {
                    "zone": "living_room",
                    "pose": {
                        "frame": "map",
                        "x": 1.4,
                        "y": 0.7,
                        "z": 0.448,
                        "yaw": 0.0,
                    },
                    "observed_at": 99.0,
                },
                "manipulation": {
                    "mobile_pregrasp_offset": {
                        "x": 0.55,
                        "y": 0.137,
                        "yaw": 0.0,
                        "expected_object_z": 0.448,
                    }
                },
            },
            "confidence": 1.0,
        },
        forbidden_zones=["utility_room"],
        observation_max_age_sec=5.0,
        now_wall_time=100.0,
    )
    assert context["safety_context_trusted"]
    assert context["navigation_source"] == "object_memory_observation"
    assert context["target_zone"] == "living_room"
    x, y, yaw = context["resolved_target_pose"]
    assert abs(x - 0.85) < 1e-9
    assert abs(y - 0.563) < 1e-9
    assert yaw == 0.0
    assert context["object_observation_age_sec"] == 1.0


def test_object_pregrasp_rejects_stale_observation():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "object_pregrasp:cup"},
        object_record={
            "entity_id": "object-cup",
            "name": "cup",
            "payload": {
                "location": {
                    "zone": "living_room",
                    "pose": {
                        "frame": "map",
                        "x": 1.4,
                        "y": 0.7,
                        "z": 0.448,
                        "yaw": 0.0,
                    },
                    "observed_at": 90.0,
                },
                "manipulation": {
                    "mobile_pregrasp_offset": {
                        "x": 0.55,
                        "y": 0.137,
                        "yaw": 0.0,
                    }
                },
            },
            "confidence": 1.0,
        },
        observation_max_age_sec=5.0,
        now_wall_time=100.0,
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "OBJECT_OBSERVATION_STALE"


def test_object_pregrasp_requires_observed_pose():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "object_pregrasp:cup"},
        object_record={
            "entity_id": "object-cup",
            "name": "cup",
            "payload": {
                "manipulation": {
                    "mobile_pregrasp_offset": {
                        "x": 0.55,
                        "y": 0.137,
                        "yaw": 0.0,
                    }
                }
            },
            "confidence": 1.0,
        },
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "OBJECT_POSE_NOT_OBSERVED"


def _fallen_cup_memory_record():
    return {
        "entity_id": "object-cup",
        "name": "cup",
        "confidence": 1.0,
        "payload": {
            "location": {
                "zone": "living_room",
                "pose": {
                    "frame": "map",
                    "x": 1.0,
                    "y": -0.1,
                    "z": 0.04,
                    "yaw": -3.0,
                },
                "observed_at": 100.0,
            },
            "manipulation": {
                "pick_approach_joint_target": [0.2, -0.75, 1.15, -0.35],
                "mobile_pregrasp_offset": {
                    "x": 0.54806,
                    "y": 0.142303,
                    "yaw": 0.0,
                    "expected_object_z": 0.620182,
                },
            },
        },
    }


def test_fallen_cup_blocks_recovery_navigation():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "object_pregrasp:cup"},
        object_record=_fallen_cup_memory_record(),
        observation_max_age_sec=5.0,
        now_wall_time=100.2,
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "OBJECT_HEIGHT_OUT_OF_RANGE"
    assert context["object_height_error_m"] > 0.5
    assert "resolved_target_pose" not in context


def test_fallen_cup_blocks_fixed_height_pick():
    context = resolve_trusted_context(
        action="pick",
        params={"object": "cup"},
        object_record=_fallen_cup_memory_record(),
    )
    assert not context["safety_context_trusted"]
    assert context["context_error"] == "OBJECT_HEIGHT_OUT_OF_RANGE"
    assert "resolved_arm_joint_target" not in context
