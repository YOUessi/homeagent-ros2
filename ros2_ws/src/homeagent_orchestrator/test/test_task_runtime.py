from homeagent_orchestrator.task_runtime import (
    CommandDeduplicator,
    FetchTask,
    FETCH_ABORTED,
    FETCH_COMPLETE,
    command_fingerprint,
    normalize_command,
)


def test_command_normalization_and_fingerprint_are_stable():
    assert normalize_command("  去拿   水杯  ") == "去拿 水杯"
    assert command_fingerprint("去拿   水杯") == command_fingerprint(" 去拿 水杯 ")


def test_short_window_duplicate_is_dropped():
    dedup = CommandDeduplicator(window_sec=2.0)
    assert dedup.accept("机械臂检查一下", now=10.0)
    assert not dedup.accept("机械臂检查一下", now=10.5)
    assert dedup.accept("机械臂检查一下", now=12.1)


def test_fetch_happy_path_completes_after_verified_pick():
    task = FetchTask(task_id="t1", text="去拿水杯", object_name="cup")

    first = task.action_for_stage()
    assert first.next_action == "stow_arm"

    nav = task.on_result(
        action="stow_arm",
        success=True,
        code="MOVEIT_SUCCEEDED",
    )
    assert nav.next_action == "navigate"

    pick = task.on_result(
        action="navigate",
        success=True,
        code="NAV2_SUCCEEDED",
    )
    assert pick.next_action == "pick"

    done = task.on_result(
        action="pick",
        success=True,
        code="GAZEBO_CONTACT_PICK_SUCCEEDED",
        result_json=(
            '{"contact_gated_physical_hold":true,'
            '"contact_required":true}'
        ),
    )
    assert done.status == FETCH_COMPLETE
    assert done.terminal


def test_navigation_failure_retries_once_then_aborts():
    task = FetchTask(task_id="t2", text="去拿水杯", object_name="cup")
    task.on_result(
        action="stow_arm",
        success=True,
        code="MOVEIT_SUCCEEDED",
    )

    retry = task.on_result(
        action="navigate",
        success=False,
        code="NAV2_TF_UNAVAILABLE",
    )
    assert retry.next_action == "navigate"
    assert retry.recovery
    assert task.nav_retries == 1

    aborted = task.on_result(
        action="navigate",
        success=False,
        code="NAV2_TF_UNAVAILABLE",
    )
    assert aborted.status == FETCH_ABORTED


def test_pick_failure_reobserves_via_navigation_once():
    task = FetchTask(task_id="t3", text="去拿水杯", object_name="cup")
    task.on_result(
        action="stow_arm",
        success=True,
        code="MOVEIT_SUCCEEDED",
    )
    task.on_result(
        action="navigate",
        success=True,
        code="NAV2_SUCCEEDED",
    )

    recovery = task.on_result(
        action="pick",
        success=False,
        code="GAZEBO_CONTACT_PICK_FAILED",
    )
    assert recovery.next_action == "navigate"
    assert recovery.recovery
    assert task.pick_recoveries == 1

    task.on_result(
        action="navigate",
        success=True,
        code="NAV2_SUCCEEDED",
    )
    aborted = task.on_result(
        action="pick",
        success=False,
        code="GAZEBO_CONTACT_PICK_FAILED",
    )
    assert aborted.status == FETCH_ABORTED


def test_false_contact_postcondition_is_not_accepted():
    task = FetchTask(task_id="t4", text="去拿水杯", object_name="cup")
    task.on_result(
        action="stow_arm",
        success=True,
        code="MOVEIT_SUCCEEDED",
    )
    task.on_result(
        action="navigate",
        success=True,
        code="NAV2_SUCCEEDED",
    )

    recovery = task.on_result(
        action="pick",
        success=True,
        code="GAZEBO_CONTACT_PICK_SUCCEEDED",
        result_json=(
            '{"contact_gated_physical_hold":false,'
            '"contact_required":true}'
        ),
    )
    assert recovery.next_action == "navigate"
    assert recovery.recovery


def test_safety_reject_aborts_task():
    task = FetchTask(task_id="t5", text="去拿水杯", object_name="cup")
    decision = task.on_safety_reject("FORBIDDEN_ZONE")
    assert decision.status == FETCH_ABORTED
    assert decision.terminal
