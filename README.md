# HomeAgent-ROS2

面向家庭服务机器人的安全具身智能 Agent 系统。项目目标直接对应“ROS2 + 具身智能 Agent 开发”岗位：从 ROS2 底层执行、主脑调度、安全法则、家庭记忆，到大模型 Agent、语音交互、场景推理和仿真联调。

## 核心原则

LLM 不能直接控制物理执行器。所有动作必须经过结构化动作协议与安全法则引擎：

```text
User / Voice / Vision
        |
        v
  Agent Planner
        |
        v
 Action Proposal
        |
        v
 Safety Engine
   |         |
 allow     reject
   |         |
   v         v
Skill Executor   Audit Log
   |
   v
ROS2 / Nav2 / MoveIt2 / Perception
```

## 仓库结构

- `ros2_ws/`：ROS2 Humble 工作空间
- `ros2_ws/src/homeagent_safety/`：安全动作校验节点
- `ros2_ws/src/homeagent_orchestrator/`：主脑调度/Agent 接入节点
- `agent/`：LLM、工具调用、任务规划与记忆层
- `simulation/`：Gazebo / Isaac Sim 场景与机器人模型
- `docs/`：架构、环境、JD 对照、开发日志
- `scripts/`：环境检查与开发脚本
- `tests/`：跨模块集成测试

## 当前状态

Phase 0 已启动：
- Tang 作为唯一 ROS2 / GPU / 仿真运行环境。
- ROS2 Humble 已确认可用。
- 创建 ROS2 workspace 和安全、调度两个基础包。
- 第一版安全规则引擎实现中。
- GitHub 作为唯一事实源（source of truth），本地 Tang 负责运行与测试。

详见 `docs/DEVELOPMENT_LOG.md`。
