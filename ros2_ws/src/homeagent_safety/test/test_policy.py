from homeagent_safety.policy import evaluate_action


def test_safe_navigation_is_allowed():
    decision = evaluate_action(
        {
            "action": "navigate",
            "params": {"target": "living_room"},
            "context": {
                "forbidden_zones": ["garage"],
                "safety_context_trusted": True,
                "resolved_target_pose": [0.8, 0.0, 0.0],
            },
        }
    )
    assert decision.allowed
    assert decision.code == "ALLOW"


def test_forbidden_zone_is_rejected_before_execution():
    decision = evaluate_action(
        {
            "action": "navigate",
            "params": {"target": "garage"},
            "context": {"forbidden_zones": ["garage"]},
        }
    )
    assert not decision.allowed
    assert decision.code == "FORBIDDEN_ZONE"


def test_navigation_requires_trusted_place_memory():
    decision = evaluate_action(
        {
            "action": "navigate",
            "params": {"target": "unknown_room"},
            "context": {
                "safety_context_trusted": False,
                "context_error": "PLACE_NOT_IN_MEMORY",
                "forbidden_zones": [],
            },
        }
    )
    assert not decision.allowed
    assert decision.code == "UNTRUSTED_NAVIGATION_CONTEXT"


def test_navigation_requires_resolved_pose():
    decision = evaluate_action(
        {
            "action": "navigate",
            "params": {"target": "living_room"},
            "context": {
                "safety_context_trusted": True,
                "forbidden_zones": [],
            },
        }
    )
    assert not decision.allowed
    assert decision.code == "INVALID_NAVIGATION_CONTEXT"


def test_sharp_object_to_minor_is_rejected():
    decision = evaluate_action(
        {
            "action": "handover",
            "params": {"object": "kitchen_knife", "object_tags": ["sharp"]},
            "context": {"recipient_age": 10, "safety_context_trusted": True},
        }
    )
    assert not decision.allowed
    assert decision.code == "DANGEROUS_HANDOVER_MINOR"


def test_emergency_stop_blocks_motion():
    decision = evaluate_action(
        {
            "action": "navigate",
            "params": {"target": "living_room"},
            "context": {"emergency_stop": True},
        }
    )
    assert not decision.allowed
    assert decision.code == "E_STOP_ACTIVE"


def test_manipulation_requires_trusted_safety_context():
    decision = evaluate_action(
        {
            "action": "pick",
            "params": {"object": "cup"},
            "context": {},
        }
    )
    assert not decision.allowed
    assert decision.code == "UNTRUSTED_SAFETY_CONTEXT"


def test_pick_requires_resolved_trusted_arm_target():
    decision = evaluate_action(
        {
            "action": "pick",
            "params": {"object": "cup"},
            "context": {
                "safety_context_trusted": True,
                "object_tags": [],
            },
        }
    )
    assert not decision.allowed
    assert decision.code == "INVALID_MANIPULATION_CONTEXT"


def test_pick_with_trusted_arm_target_is_allowed():
    decision = evaluate_action(
        {
            "action": "pick",
            "params": {"object": "cup"},
            "context": {
                "safety_context_trusted": True,
                "object_tags": [],
                "resolved_arm_joint_target": [0.2, -0.75, 1.15, -0.35],
            },
        }
    )
    assert decision.allowed
    assert decision.code == "ALLOW"
