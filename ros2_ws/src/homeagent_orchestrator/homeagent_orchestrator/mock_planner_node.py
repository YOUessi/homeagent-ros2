import json
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import ActionProposal, SafetyDecision, SkillResult

from .task_runtime import (
    CommandDeduplicator,
    FetchTask,
    FETCH_ABORTED,
    FETCH_COMPLETE,
    command_fingerprint,
    normalize_command,
)


class MockPlannerNode(Node):
    """Deterministic high-level planner and task coordinator for integration tests."""

    def __init__(self) -> None:
        super().__init__("homeagent_mock_planner")
        self.declare_parameter("command_dedup_window_sec", 2.0)

        self._dedup = CommandDeduplicator(
            float(self.get_parameter("command_dedup_window_sec").value)
        )
        self._tasks = {}
        self._request_to_task = {}
        self._task_command_keys = {}
        self._active_command_tasks = {}

        self._command_sub = self.create_subscription(
            String, "/homeagent/user_command", self._on_command, 10
        )
        self._proposal_pub = self.create_publisher(
            ActionProposal, "/homeagent/action_candidate", 10
        )
        self._approved_sub = self.create_subscription(
            SafetyDecision, "/homeagent/action_approved", self._on_approved, 10
        )
        self._rejected_sub = self.create_subscription(
            SafetyDecision, "/homeagent/action_rejected", self._on_rejected, 10
        )
        self._skill_result_sub = self.create_subscription(
            SkillResult, "/homeagent/skill_result", self._on_skill_result, 10
        )

        self.get_logger().info(
            "HomeAgent mock planner ready with task state machine and dedup"
        )

    def _on_command(self, msg: String) -> None:
        text = normalize_command(msg.data)
        if not text:
            self.get_logger().warning("COMMAND_DROP reason=EMPTY")
            return

        if not self._dedup.accept(text):
            self.get_logger().warning(
                f"COMMAND_DEDUP_DROP key={command_fingerprint(text)} text={text}"
            )
            return

        if self._is_cup_fetch(text):
            self._start_fetch_task(text)
            return

        proposal = self._plan(text)
        self._proposal_pub.publish(proposal)
        self.get_logger().info(
            f"PROPOSE action={proposal.action} request_id={proposal.request_id}"
        )

    def _start_fetch_task(self, text: str) -> None:
        command_key = command_fingerprint(text)
        existing_task_id = self._active_command_tasks.get(command_key)
        if existing_task_id in self._tasks:
            self.get_logger().warning(
                f"COMMAND_DEDUP_ACTIVE task_id={existing_task_id} "
                f"key={command_key} text={text}"
            )
            return

        task_id = str(uuid.uuid4())
        task = FetchTask(
            task_id=task_id,
            text=text,
            object_name="cup",
        )
        self._tasks[task_id] = task
        self._task_command_keys[task_id] = command_key
        self._active_command_tasks[command_key] = task_id

        self.get_logger().info(
            f"TASK_START task_id={task_id} type=fetch object=cup stage=stow"
        )
        self._dispatch_task(task, task.action_for_stage())

    def _dispatch_task(self, task: FetchTask, decision) -> None:
        if decision.next_action is None:
            self._finish_task(task, decision)
            return

        proposal = self._make_proposal(
            task.text,
            decision.next_action,
            decision.next_params,
            {},
            task_id=task.task_id,
        )
        self._request_to_task[proposal.request_id] = task.task_id
        self._proposal_pub.publish(proposal)

        recovery = " recovery=true" if decision.recovery else ""
        reason = f" reason={decision.reason}" if decision.reason else ""
        self.get_logger().info(
            f"TASK_DISPATCH task_id={task.task_id} stage={decision.stage} "
            f"action={proposal.action} request_id={proposal.request_id}"
            f"{recovery}{reason}"
        )
        self.get_logger().info(
            f"PROPOSE action={proposal.action} request_id={proposal.request_id}"
        )

    def _make_proposal(
        self,
        text: str,
        action: str,
        params: dict,
        context: dict,
        *,
        task_id: str = "",
    ) -> ActionProposal:
        proposal = ActionProposal()
        proposal.request_id = str(uuid.uuid4())
        proposal.action = action
        proposal.params_json = json.dumps(params, ensure_ascii=False)
        proposal.context_json = json.dumps(context, ensure_ascii=False)
        source = {
            "type": "mock",
            "text": text,
            "command_key": command_fingerprint(text),
        }
        if task_id:
            source["task_id"] = task_id
        proposal.source = json.dumps(source, ensure_ascii=False)
        return proposal

    @staticmethod
    def _is_cup_fetch(text: str) -> bool:
        wants_cup = "水杯" in text or "杯子" in text
        wants_fetch = (
            "去拿" in text
            or "帮我拿" in text
            or "拿过来" in text
            or "取来" in text
        )
        return wants_cup and wants_fetch

    def _plan(self, text: str) -> ActionProposal:
        if "刀" in text and ("小孩" in text or "孩子" in text):
            return self._make_proposal(
                text,
                "handover",
                {
                    "object": "kitchen_knife",
                    "object_tags": ["sharp"],
                    "recipient": "child",
                },
                {"recipient_age": 10, "safety_context_trusted": True},
            )

        if self._is_cup_fetch(text):
            return self._make_proposal(text, "stow_arm", {}, {})

        if "客厅" in text:
            return self._make_proposal(
                text,
                "navigate",
                {"target": "living_room"},
                {"forbidden_zones": ["utility_room"]},
            )

        if "机械臂" in text or "检查姿态" in text or "检查一下" in text:
            return self._make_proposal(
                text,
                "look_at",
                {"target": "inspection_point"},
                {},
            )

        if ("拿起" in text or "拿" in text or "抓取" in text) and (
            "水杯" in text or "杯子" in text
        ):
            return self._make_proposal(
                text,
                "pick",
                {"object": "cup"},
                {},
            )

        if "水杯" in text or "杯子" in text:
            return self._make_proposal(
                text,
                "observe",
                {"object": "cup"},
                {},
            )

        return self._make_proposal(
            text,
            "speak",
            {"text": "当前 mock planner 还不能规划该任务。"},
            {},
        )

    def _on_approved(self, msg: SafetyDecision) -> None:
        task_id = self._request_to_task.get(msg.request_id, "")
        suffix = f" task_id={task_id}" if task_id else ""
        self.get_logger().info(
            f"APPROVED request_id={msg.request_id} "
            f"action={msg.action} code={msg.code}{suffix}"
        )

    def _on_rejected(self, msg: SafetyDecision) -> None:
        self.get_logger().warning(
            f"REJECTED request_id={msg.request_id} action={msg.action} "
            f"code={msg.code} reason={msg.reason}"
        )

        task_id = self._request_to_task.pop(msg.request_id, None)
        if task_id is None:
            return

        task = self._tasks.get(task_id)
        if task is None:
            return

        decision = task.on_safety_reject(msg.code)
        self._finish_task(task, decision)

    def _on_skill_result(self, msg: SkillResult) -> None:
        self.get_logger().info(
            f"RESULT request_id={msg.request_id} action={msg.action} "
            f"success={msg.success} code={msg.code} result={msg.result_json}"
        )

        task_id = self._request_to_task.pop(msg.request_id, None)
        if task_id is None:
            return

        task = self._tasks.get(task_id)
        if task is None:
            return

        decision = task.on_result(
            action=msg.action,
            success=bool(msg.success),
            code=msg.code,
            result_json=msg.result_json,
        )

        if decision.status in {FETCH_COMPLETE, FETCH_ABORTED}:
            self._finish_task(task, decision)
            return

        if decision.recovery:
            self.get_logger().warning(
                f"TASK_RECOVERY task_id={task.task_id} "
                f"stage={decision.stage} reason={decision.reason} "
                f"nav_retries={task.nav_retries} "
                f"pick_recoveries={task.pick_recoveries}"
            )

        self._dispatch_task(task, decision)

    def _finish_task(self, task: FetchTask, decision) -> None:
        if decision.status == FETCH_COMPLETE:
            self.get_logger().info(
                f"TASK_COMPLETE task_id={task.task_id} type=fetch "
                f"object={task.object_name} reason={decision.reason}"
            )
        else:
            self.get_logger().warning(
                f"TASK_ABORT task_id={task.task_id} type=fetch "
                f"object={task.object_name} stage={task.stage} "
                f"reason={decision.reason}"
            )

        command_key = self._task_command_keys.pop(task.task_id, None)
        if command_key is not None:
            self._active_command_tasks.pop(command_key, None)
        self._tasks.pop(task.task_id, None)

        stale_requests = [
            request_id
            for request_id, mapped_task_id in self._request_to_task.items()
            if mapped_task_id == task.task_id
        ]
        for request_id in stale_requests:
            self._request_to_task.pop(request_id, None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
