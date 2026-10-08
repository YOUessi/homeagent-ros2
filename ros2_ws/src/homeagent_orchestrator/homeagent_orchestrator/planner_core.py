import json
from typing import Any, Dict


SUPPORTED_ACTIONS = {
    "fetch",
    "navigate",
    "observe",
    "speak",
    "pick",
    "place",
    "handover",
    "look_at",
    "stow_arm",
    "stop",
}

FENCE = chr(96) * 3

SYSTEM_PROMPT = """你是家庭服务机器人的高层任务规划器。
你只能返回一个 JSON 对象，不要返回 Markdown、解释或代码块。

允许的 action:
- fetch: params.object；表示“去找到并拿取某物”的高层任务，由确定性状态机拆成多个受控技能
- navigate: params.target
- observe: params.object
- speak: params.text
- pick: params.object
- place: params.object, params.target
- handover: params.object, params.recipient
- look_at: params.target
- stow_arm: params 可为空；用于移动导航前把机械臂收回安全姿态
- stop: params 可为空

约束:
1. 只提出高层技能或 fetch 元任务，不得输出 /cmd_vel、轮速、关节角、轨迹点或任何底层控制量。
2. fetch 不会直接执行；系统会用确定性状态机拆成 stow/navigation/pick/verify。
3. 不要判断动作是否安全。安全性由独立 Safety Engine 决定。
3. 不要伪造 recipient_age、object_tags、forbidden_zones 等安全上下文。
4. 无法确定任务时使用 speak，向用户提出最小必要澄清。

输出格式:
{"action":"navigate","params":{"target":"living_room"}}
"""


class PlannerOutputError(ValueError):
    pass


def parse_plan_text(text: str) -> Dict[str, Any]:
    cleaned = text.strip()
    if cleaned.startswith(FENCE):
        if cleaned.startswith(FENCE + "json"):
            cleaned = cleaned[len(FENCE + "json") :].strip()
        else:
            cleaned = cleaned[len(FENCE) :].strip()
        if cleaned.endswith(FENCE):
            cleaned = cleaned[: -len(FENCE)].strip()

    try:
        plan = json.loads(cleaned)
    except json.JSONDecodeError:
        start = cleaned.find("{")
        end = cleaned.rfind("}")
        if start < 0 or end <= start:
            raise PlannerOutputError("LLM response contains no JSON object")
        try:
            plan = json.loads(cleaned[start : end + 1])
        except json.JSONDecodeError as exc:
            raise PlannerOutputError(f"invalid planner JSON: {exc}") from exc

    if not isinstance(plan, dict):
        raise PlannerOutputError("planner output must be a JSON object")

    action = plan.get("action")
    params = plan.get("params", {})
    if action not in SUPPORTED_ACTIONS:
        raise PlannerOutputError(f"unsupported action: {action!r}")
    if not isinstance(params, dict):
        raise PlannerOutputError("params must be a JSON object")

    if action == "fetch" and not params.get("object"):
        raise PlannerOutputError("fetch requires params.object")
    if action == "navigate" and not params.get("target"):
        raise PlannerOutputError("navigate requires params.target")
    if action in {"observe", "pick"} and not params.get("object"):
        raise PlannerOutputError(f"{action} requires params.object")
    if action == "place" and (not params.get("object") or not params.get("target")):
        raise PlannerOutputError("place requires params.object and params.target")
    if action == "handover" and (
        not params.get("object") or not params.get("recipient")
    ):
        raise PlannerOutputError("handover requires params.object and params.recipient")
    if action == "look_at" and not params.get("target"):
        raise PlannerOutputError("look_at requires params.target")
    if action == "speak" and not params.get("text"):
        raise PlannerOutputError("speak requires params.text")

    # Safety context is deliberately NOT accepted from the LLM.
    return {"action": action, "params": params, "context": {}}
