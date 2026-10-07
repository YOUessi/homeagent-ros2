from homeagent_safety.policy import evaluate_action


def test_safe_navigation_is_allowed():
    decision = evaluate_action(
        {
            "action": "navigate",
            "params": {"target": "living_room"},
            "context": {"forbidden_zones": ["garage"]},
        }
    )
    assert decision.allowed
    assert decision.code == "ALLOW"


def test_forbidden_zone_is_rejected():
    decision = evaluate_action(
        {
            "action": "navigate",
            "params": {"target": "garage"},
            "context": {"forbidden_zones": ["garage"]},
        }
    )
    assert not decision.allowed
    assert decision.code == "FORBIDDEN_ZONE"


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
