# HomeAgent-ROS2

面向家庭服务机器人的安全具身智能 Agent 系统。项目目标直接对应“ROS2 + 具身智能 Agent 开发”岗位：从 ROS2 底层执行、主脑调度、安全法则、家庭记忆，到大模型 Agent、语音交互、场景推理和仿真联调。

## 核心原则

LLM 不能直接控制物理执行器。所有动作必须先经过结构化动作协议和独立安全法则引擎：

```text
User / Voice / Vision
        |
        v
  Agent Planner
        |
        v
 Action Candidate
        |
        v
 Trusted Context Resolver
        |
        v
 ActionProposal (typed ROS2 msg)
        |
        v
 Safety Engine
   |         |
 allow     reject
   |         |
   v         v
Skill Executor   Audit / Replan
   |
   v
Nav2 / MoveIt2 / Perception / Speech
   |
   v
SkillResult -> Agent feedback loop
```

## 当前模块

- `homeagent_interfaces`：强类型 ROS2 消息/服务协议。
- `homeagent_safety`：动作白名单、急停、禁区、危险物品/人员规则。
- `homeagent_orchestrator`：主脑调度骨架、可重复 mock planner，以及 DeepSeek 高层动作规划器。
- `homeagent_context`：把不可信 Agent candidate 与家庭档案/策略合并为可信 world-state safety context。
- `homeagent_skills`：安全门之后的技能执行适配层；已有 mock 与 Nav2 adapter，后续增加 MoveIt2。
- `homeagent_memory`：SQLite 家庭档案、物品/人员记忆、观测日志与人工纠正权重。
- `homeagent_description`：自研 HomeBot 差速底盘、LiDAR、Camera 与家庭房间 Gazebo 模型。
- `homeagent_navigation`：SLAM Toolbox / Nav2 配置与仿真导航入口。
- `homeagent_bringup`：核心服务一键启动与 planner 切换。

## 仓库结构

- `ros2_ws/`：ROS2 Humble 工作空间
- `agent/`：DeepSeek、tool calling、任务规划、场景推理
- `simulation/`：Gazebo / Isaac Sim
- `docker/`：可复现 ROS2/导航/机械臂依赖环境
- `docs/`：架构、JD 对照、环境、开发日志
- `scripts/`：构建、测试、环境检查和演示脚本

## 已验证闭环

```text
“去客厅”
 -> mock planner
 -> navigate(living_room)
 -> Safety ALLOW
 -> mock skill executor
 -> MOCK_NAVIGATION_COMPLETE
 -> planner 收到执行反馈
```

危险动作验证：

```text
“把刀给小孩”
 -> handover(kitchen_knife, sharp, recipient_age=10)
 -> Safety REJECT: DANGEROUS_HANDOVER_MINOR
 -> 技能执行层没有收到动作
```

当前 workspace 已有 9 个 HomeAgent ROS2 package，`colcon build` 全部通过；Safety + Memory + Planner + Context 单元测试 17/17 通过。Gazebo HomeBot、LiDAR/odometry、SLAM Toolbox 建图、地图保存和 Nav2 真实导航均已在隔离 Docker 环境中端到端验证通过。

DeepSeek 节点只读取环境变量 `DEEPSEEK_API_KEY`，密钥不会进入代码、ROS topic 或日志；没有密钥时节点 fail-closed，只发布 `NO_API_KEY` 错误，不产生机器人动作。

## 快速验证

```bash
./scripts/build_and_test.sh
./scripts/demo_core.sh

# 完整移动机器人 Demo：
# “去客厅” -> Context -> Safety -> Nav2 -> Gazebo 真移动 -> 轨迹图
./scripts/demo_nav2.sh
```

`demo_nav2.sh` 会额外生成 `artifacts/nav2_demo_path.png`、机器可读 JSON 和完整运行日志；运行产物默认被 Git 忽略。

开发过程按日期记录在 `docs/journal/YYYY-MM-DD.md`；`docs/DEVELOPMENT_LOG.md` 只保留阶段摘要。每天必须记录实际过程、验证结果、失败、问题定位与修复，不把“代码已写”混同为“已验证通过”。
