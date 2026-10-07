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

当前 `homeagent_skills` 已提供 deterministic mock、真实 Nav2 adapter 与真实 MoveIt2 adapter。导航语义目标不再由 skill 内部硬编码：`Trusted Context Resolver` 先从 `homeagent_memory` 的 `place` 实体解析可信 `map_pose`，Safety 校验后 Nav2 adapter 只执行该可信 pose。HomeArm 的 `look_at` 等固定动作从可信 skill library 解析 joint target；`pick(object)` 的接近姿态则从该 object 的 Household Memory 中解析。Safety-approved pick 会依次进入 MoveIt2/OMPL、`homearm_controller`、`gripper_controller`，并通过 MoveIt PlanningScene 建立逻辑 attach。当前使用 GenericSystem，因此逻辑 attach 与物理抓取严格区分。

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

## 5. Mobile Manipulator 联合拓扑

当前联合 Demo 使用同一 ROS graph：

```text
map
 └─ odom
     └─ base_footprint
         └─ base_link
             └─ arm_base_footprint   (static mount, z=0.22m)
                 └─ arm_base_link
                     └─ ... HomeArm ... -> tool_link
```

HomeBot 与 HomeArm 不再共享 root frame 名称；HomeArm 的 robot description topic 隔离为：

```text
/homearm/robot_description
```

联合运行时：

```text
HomeBot Gazebo / Nav2 / AMCL
        +
HomeArm MoveIt2 / ros2_control / gripper
        +
HomeAgent Context / Safety / Nav2Skill / MoveItSkill
```

当前 arm 与 base 的关系在 TF、规划和控制层已经连通；但 HomeArm 仍由 GenericSystem 驱动，尚未作为 Gazebo 动力学实体挂在 HomeBot 上。因此当前属于**控制/规划级 mobile-manipulator integration**，不是完整物理级联合仿真。
