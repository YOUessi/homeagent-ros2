import hashlib
import json
import time
from dataclasses import dataclass, field
from typing import Dict, Optional


FETCH_STOW = "stow"
FETCH_NAVIGATE = "navigate"
FETCH_PICK = "pick"
FETCH_VERIFY = "verify"
FETCH_COMPLETE = "complete"
FETCH_ABORTED = "aborted"

RECOVERABLE_NAV_CODES = {
    "NAV2_FAILED",
    "NAV2_TF_UNAVAILABLE",
    "NAV2_UNAVAILABLE",
    "NAV2_SEND_FAILED",
    "NAV2_GOAL_REJECTED",
}
RECOVERABLE_PICK_CODES = {
    "GAZEBO_CONTACT_PICK_FAILED",
    "GAZEBO_GRASP_NOT_CONFIRMED",
    "GRASP_PROBE_MOVE_FAILED",
    "GRASP_PROBE_STATE_FAILED",
    "LOCAL_ALIGNMENT_TIMEOUT",
    "LOCAL_ALIGNMENT_STATE_FAILED",
    "GAZEBO_TARGET_QUERY_FAILED",
}


def normalize_command(text: str) -> str:
    return " ".join(str(text).strip().split())


def command_fingerprint(text: str) -> str:
    normalized = normalize_command(text)
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()[:16]


class CommandDeduplicator:
    """Drop short-window duplicate commands before they reach the planner."""

    def __init__(self, window_sec: float = 2.0) -> None:
        self.window_sec = float(window_sec)
        self._last_seen: Dict[str, float] = {}

    def accept(self, text: str, *, now: Optional[float] = None) -> bool:
        normalized = normalize_command(text)
        if not normalized:
            return False

        timestamp = time.monotonic() if now is None else float(now)
        key = command_fingerprint(normalized)
        previous = self._last_seen.get(key)
        if previous is not None and timestamp - previous < self.window_sec:
            return False

        self._last_seen[key] = timestamp
        self._prune(timestamp)
        return True

    def _prune(self, now: float) -> None:
        horizon = max(self.window_sec * 4.0, 30.0)
        expired = [
            key
            for key, timestamp in self._last_seen.items()
            if now - timestamp > horizon
        ]
        for key in expired:
            self._last_seen.pop(key, None)


@dataclass
class TaskDecision:
    status: str
    stage: str
    next_action: Optional[str] = None
    next_params: Dict = field(default_factory=dict)
    recovery: bool = False
    reason: str = ""

    @property
    def terminal(self) -> bool:
        return self.status in {FETCH_COMPLETE, FETCH_ABORTED}


@dataclass
class FetchTask:
    """Deterministic execution state for one fetch(object) task."""

    task_id: str
    text: str
    object_name: str
    stage: str = FETCH_STOW
    nav_retries: int = 0
    pick_recoveries: int = 0
    max_nav_retries: int = 1
    max_pick_recoveries: int = 1

    def action_for_stage(self) -> TaskDecision:
        if self.stage == FETCH_STOW:
            return TaskDecision(
                status="running",
                stage=self.stage,
                next_action="stow_arm",
                next_params={},
            )
        if self.stage == FETCH_NAVIGATE:
            return TaskDecision(
                status="running",
                stage=self.stage,
                next_action="navigate",
                next_params={
                    "target": f"object_pregrasp:{self.object_name}"
                },
            )
        if self.stage == FETCH_PICK:
            return TaskDecision(
                status="running",
                stage=self.stage,
                next_action="pick",
                next_params={"object": self.object_name},
            )
        if self.stage == FETCH_COMPLETE:
            return TaskDecision(status=FETCH_COMPLETE, stage=self.stage)
        return TaskDecision(
            status=FETCH_ABORTED,
            stage=FETCH_ABORTED,
            reason="task is not dispatchable",
        )

    def on_safety_reject(self, code: str) -> TaskDecision:
        self.stage = FETCH_ABORTED
        return TaskDecision(
            status=FETCH_ABORTED,
            stage=self.stage,
            reason=f"safety rejected task action: {code}",
        )

    def on_result(
        self,
        *,
        action: str,
        success: bool,
        code: str,
        result_json: str = "{}",
    ) -> TaskDecision:
        if action == "stow_arm" and self.stage == FETCH_STOW:
            if not success:
                return self._abort(
                    f"stow failed: {code}"
                )
            self.stage = FETCH_NAVIGATE
            return self.action_for_stage()

        if action == "navigate" and self.stage == FETCH_NAVIGATE:
            if success:
                self.stage = FETCH_PICK
                return self.action_for_stage()

            if (
                code in RECOVERABLE_NAV_CODES
                and self.nav_retries < self.max_nav_retries
            ):
                self.nav_retries += 1
                decision = self.action_for_stage()
                decision.recovery = True
                decision.reason = f"retry navigation after {code}"
                return decision

            return self._abort(f"navigation failed: {code}")

        if action == "pick" and self.stage == FETCH_PICK:
            if not success:
                return self._recover_or_abort_pick(
                    f"pick failed: {code}",
                    code=code,
                )

            self.stage = FETCH_VERIFY
            verified, reason = verify_pick_result(code, result_json)
            if verified:
                self.stage = FETCH_COMPLETE
                return TaskDecision(
                    status=FETCH_COMPLETE,
                    stage=self.stage,
                    reason=reason,
                )

            return self._recover_or_abort_pick(
                f"pick verification failed: {reason}",
                code="PICK_POSTCONDITION_FAILED",
            )

        return self._abort(
            f"unexpected result action={action} stage={self.stage}"
        )

    def _recover_or_abort_pick(
        self,
        reason: str,
        *,
        code: str,
    ) -> TaskDecision:
        recoverable = (
            code in RECOVERABLE_PICK_CODES
            or code == "PICK_POSTCONDITION_FAILED"
        )
        if recoverable and self.pick_recoveries < self.max_pick_recoveries:
            self.pick_recoveries += 1
            # Re-enter navigation so a fresh perception observation and
            # object_pregrasp are resolved before attempting the arm again.
            self.stage = FETCH_NAVIGATE
            decision = self.action_for_stage()
            decision.recovery = True
            decision.reason = reason
            return decision
        return self._abort(reason)

    def _abort(self, reason: str) -> TaskDecision:
        self.stage = FETCH_ABORTED
        return TaskDecision(
            status=FETCH_ABORTED,
            stage=self.stage,
            reason=reason,
        )


def verify_pick_result(code: str, result_json: str) -> tuple:
    """Verify execution postconditions instead of trusting action status alone."""
    try:
        payload = json.loads(result_json or "{}")
    except json.JSONDecodeError:
        return False, "invalid pick result JSON"

    if code == "GAZEBO_CONTACT_PICK_SUCCEEDED":
        if payload.get("contact_gated_physical_hold") is not True:
            return False, "Gazebo contact hold postcondition is false"
        if payload.get("contact_required") is not True:
            return False, "contact evidence is missing"
        return True, "contact-gated physical hold verified"

    # Hardware/other adapters may not expose Gazebo-specific contact evidence.
    # Their success has already been adapter-specific, so accept the result
    # while keeping this branch explicit.
    return True, "adapter success accepted"
