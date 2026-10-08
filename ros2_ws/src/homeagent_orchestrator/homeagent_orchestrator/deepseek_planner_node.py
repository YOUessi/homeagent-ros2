import json
import os
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from homeagent_interfaces.msg import ActionProposal, SafetyDecision, SkillResult

from .deepseek_client import DeepSeekClient, DeepSeekError
from .planner_core import PlannerOutputError
from .task_runtime import (
    CommandDeduplicator,
    FetchTask,
    FETCH_ABORTED,
    FETCH_COMPLETE,
    command_fingerprint,
    normalize_command,
)


class DeepSeekPlannerNode(Node):
    """LLM intent planner with deterministic execution task coordination."""

    def __init__(self) -> None:
        super().__init__("homeagent_deepseek_planner")
        self.declare_parameter("api_base", "https://api.deepseek.com")
        self.declare_parameter("model", "deepseek-chat")
        self.declare_parameter("api_key_env", "DEEPSEEK_API_KEY")
        self.declare_parameter("command_dedup_window_sec", 2.0)

        api_base = self.get_parameter("api_base").value
        model = self.get_parameter("model").value
        api_key_env = self.get_parameter("api_key_env").value
        api_key = os.getenv(api_key_env, "")

        self._client = None
        if api_key:
            self._client = DeepSeekClient(
                api_key=api_key,
                base_url=api_base,
                model=model,
            )
            self.get_logger().info(f"DeepSeek planner ready: model={model}")
        else:
            self.get_logger().warning(
                f"{api_key_env} is not set; planner will reject incoming commands locally"
            )

        self._model = model
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
        self._planner_error_pub = self.create_publisher(
            String, "/homeagent/planner_error", 10
        )
        self.create_subscription(
            SafetyDecision,
            "/homeagent/action_rejected",
            self._on_rejected,
            10,
        )
        self.create_subscription(
            SkillResult,
            "/homeagent/skill_result",
            self._on_skill_result,
            10,
        )

    def _on_command(self, msg: String) -> None:
        text = normalize_command(msg.data)
        if not text:
            self._publish_error("EMPTY_COMMAND", "empty command")
            return

        if not self._dedup.accept(text):
            self.get_logger().warning(
                f"COMMAND_DEDUP_DROP key={command_fingerprint(text)} text={text}"
            )
            return

        if self._client is None:
            self._publish_error(
                "NO_API_KEY",
                "DeepSeek API key is not configured",
            )
            return

        try:
            plan = self._client.plan(text)
        except (DeepSeekError, PlannerOutputError) as exc:
            self._publish_error("PLANNER_FAILED", str(exc))
            return

        if plan["action"] == "fetch":
            self._start_fetch_task(
                text=text,
                object_name=str(plan["params"]["object"]),
            )
            return

        proposal = self._make_proposal(
            text,
            plan["action"],
            plan["params"],
        )
        self._proposal_pub.publish(proposal)
        self.get_logger().info(
            f"PROPOSE action={proposal.action} request_id={proposal.request_id}"
        )

    def _start_fetch_task(self, *, text: str, object_name: str) -> None:
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
            object_name=object_name,
        )
        self._tasks[task_id] = task
        self._task_command_keys[task_id] = command_key
        self._active_command_tasks[command_key] = task_id

        self.get_logger().info(
            f"TASK_START task_id={task_id} source=deepseek "
            f"type=fetch object={object_name} stage=stow"
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
            task_id=task.task_id,
        )
        self._request_to_task[proposal.request_id] = task.task_id
        self._proposal_pub.publish(proposal)

        recovery = " recovery=true" if decision.recovery else ""
        self.get_logger().info(
            f"TASK_DISPATCH task_id={task.task_id} stage={decision.stage} "
            f"action={proposal.action} request_id={proposal.request_id}"
            f"{recovery}"
        )

    def _make_proposal(
        self,
        text: str,
        action: str,
        params: dict,
        *,
        task_id: str = "",
    ) -> ActionProposal:
        proposal = ActionProposal()
        proposal.request_id = str(uuid.uuid4())
        proposal.action = action
        proposal.params_json = json.dumps(params, ensure_ascii=False)
        proposal.context_json = "{}"

        source = {
            "type": "deepseek",
            "model": self._model,
            "command_key": command_fingerprint(text),
        }
        if task_id:
            source["task_id"] = task_id
        proposal.source = json.dumps(source, ensure_ascii=False)
        return proposal

    def _publish_error(self, code: str, message: str) -> None:
        out = String()
        out.data = json.dumps(
            {"code": code, "message": message},
            ensure_ascii=False,
        )
        self._planner_error_pub.publish(out)
        self.get_logger().error(f"{code}: {message}")

    def _on_rejected(self, msg: SafetyDecision) -> None:
        self.get_logger().warning(
            f"REJECTED request_id={msg.request_id} "
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
            f"success={msg.success} code={msg.code}"
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
                f"stage={decision.stage} reason={decision.reason}"
            )

        self._dispatch_task(task, decision)

    def _finish_task(self, task: FetchTask, decision) -> None:
        if decision.status == FETCH_COMPLETE:
            self.get_logger().info(
                f"TASK_COMPLETE task_id={task.task_id} "
                f"type=fetch object={task.object_name} "
                f"reason={decision.reason}"
            )
        else:
            self.get_logger().warning(
                f"TASK_ABORT task_id={task.task_id} "
                f"type=fetch object={task.object_name} "
                f"stage={task.stage} reason={decision.reason}"
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
    node = DeepSeekPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
