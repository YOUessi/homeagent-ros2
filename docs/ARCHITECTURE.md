# 系统架构

## 1. 分层

### A. 感知与设备层
- Camera / RGB-D
- LiDAR
- Microphone
- IMU
- Mobile base / manipulator

### B. ROS2 能力层
- TF2 / robot_state_publisher
- SLAM Toolbox
- Nav2
- ros2_control
- MoveIt2
- perception nodes

### C. 技能执行层
Agent 不接触低层速度/关节命令，只能调用高层技能：
- `navigate(target)`
- `observe(object)`
- `look_at(target)`
- `pick(object)`
- `place(object, target)`
- `handover(object, recipient)`
- `speak(text)`
- `stop()`

当前 `homeagent_skills` 已同时提供 deterministic mock adapter 与真实 Nav2 adapter。导航语义目标不再由 skill 内部硬编码：`Trusted Context Resolver` 先从 `homeagent_memory` 的 `place` 实体解析可信 `map_pose`，Safety 校验后 Nav2 adapter 只执行该可信 pose。MoveIt2 / perception adapter 后续保持相同 contract 接入。

### D. 主脑调度层
- task planner
- 状态机 / Behavior Tree
- skill registry
- timeout / cancel / retry
- execution feedback / replanning

### E. 安全层
所有 Agent 动作在技能执行之前进入 `homeagent_safety`：
- 动作白名单
- emergency stop
- 禁入区域
- 危险物体与未成年人规则
- 权限/上下文检查
- fail-closed

### F. 家庭记忆层
`homeagent_memory` 使用 SQLite 保存：
- object / person / place entity
- observation history
- location / pose
- confidence / source / revision
- human correction records
- field-level correction weight metadata

### G. Agent 层
- DeepSeek / 可替换 LLM
- structured tool calling
- task planning
- scene reasoning
- memory retrieval/update
- speech interface

## 2. 强类型 ROS2 协议

内部关键边界不再使用自由字符串控制动作：

```text
ActionProposal
  request_id
  action
  params_json
  context_json
  source

SafetyDecision
  request_id
  allowed
  code
  reason
  action
  proposal_json

SkillResult
  request_id
  action
  success
  code
  result_json
```

家庭记忆通过 `MemoryUpsert / MemoryQuery / MemoryObserve / MemoryCorrect` ROS2 service 暴露。

## 3. 执行闭环

```text
observation_t
    |
    v
Agent -> ActionCandidate (untrusted)
    |
    v
Trusted Context Resolver <- household memory / policy
    |                    |
    | navigate(target)   +--> place(target).map_pose
    v
ActionProposal + trusted context
    |
    v
Safety policy(state_t, action_t)
    |
    +---- reject ----> reason -> Agent replans
    |
    v
approved decision
    |
    v
Skill adapter
    |
    v
Nav2 / MoveIt2 / perception
    |
    v
SkillResult / observation_(t+1)
    |
    +----> memory update
    |
    +----> Agent next step
```

## 4. 设计约束

1. LLM 不发布 `/cmd_vel`、关节轨迹等低层控制量。
2. Agent 只输出 schema 约束的高层技能调用。
3. Safety 独立于 LLM、可单元测试、可审计、默认拒绝未知动作。
4. 只有 `action_approved` 能进入技能层；技能层再次检查 `allowed`，双重 fail-closed。
5. 仿真先行，真机与仿真保持同一 skill contract。
6. 每个关键请求都有 request id、结果码、执行反馈和日志。
