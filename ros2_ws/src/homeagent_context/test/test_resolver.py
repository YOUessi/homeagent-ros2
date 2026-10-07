from homeagent_context.resolver import resolve_trusted_context


def test_navigation_context_comes_from_policy():
    context = resolve_trusted_context(
        action="navigate",
        params={"target": "living_room"},
        forbidden_zones=["utility_room"],
    )
    assert context["safety_context_trusted"]
    assert context["forbidden_zones"] == ["utility_room"]


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
