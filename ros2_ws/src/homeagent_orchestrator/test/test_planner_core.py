import pytest

from homeagent_orchestrator.planner_core import PlannerOutputError, parse_plan_text


def test_parse_valid_navigation():
    plan = parse_plan_text(
        '{"action":"navigate","params":{"target":"living_room"}}'
    )
    assert plan["action"] == "navigate"
    assert plan["params"]["target"] == "living_room"
    assert plan["context"] == {}


def test_parse_markdown_fence():
    fence = chr(96) * 3
    plan = parse_plan_text(
        fence + 'json\n{"action":"observe","params":{"object":"cup"}}\n' + fence
    )
    assert plan["action"] == "observe"
    assert plan["params"]["object"] == "cup"


def test_llm_safety_context_is_not_trusted():
    plan = parse_plan_text(
        '{"action":"handover","params":{"object":"knife","recipient":"child"},'
        '"context":{"recipient_age":20,"object_tags":[]}}'
    )
    assert plan["context"] == {}


def test_reject_low_level_or_unknown_action():
    with pytest.raises(PlannerOutputError):
        parse_plan_text(
            '{"action":"publish_cmd_vel","params":{"linear_x":1.0}}'
        )


def test_reject_missing_required_params():
    with pytest.raises(PlannerOutputError):
        parse_plan_text('{"action":"navigate","params":{}}')


def test_parse_fetch_meta_task():
    plan = parse_plan_text(
        '{"action":"fetch","params":{"object":"cup"}}'
    )
    assert plan["action"] == "fetch"
    assert plan["params"]["object"] == "cup"
    assert plan["context"] == {}


def test_parse_navigation_stow_action():
    plan = parse_plan_text('{"action":"stow_arm","params":{}}')
    assert plan["action"] == "stow_arm"
    assert plan["params"] == {}
    assert plan["context"] == {}
