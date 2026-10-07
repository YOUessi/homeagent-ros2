# 开发日志

## 2026-10-07 — Phase 0: 项目落地

### 决策
- 设备：Tang 作为 ROS2/GPU/仿真主环境。
- 代码与过程：GitHub 作为唯一事实源；Tang 只做运行、构建、仿真和测试。
- 路径：`/home/you/projects/homeagent-ros2`
- ROS2：使用系统 `/opt/ros/humble`，暂不将 ROS2 强行装进 Conda。

### 环境核验
- ROS2 Humble 可用。
- colcon 可用。
- RTX 4090 Laptop GPU 可用。
- Conda 可用，已有 `env_isaaclab`（Python 3.10.18）。
- Nav2、SLAM Toolbox、MoveIt2、ros2_control、Gazebo/Isaac Sim 尚未在当前 ROS2 环境发现。

### 已创建
- ROS2 workspace: `ros2_ws`
- package: `homeagent_safety`
- package: `homeagent_orchestrator`
- 架构、JD 对照、环境与开发日志文档。

### 下一步
1. 完成第一版 Safety Engine 并构建测试。
2. 实现 mock Agent -> Safety -> approval/rejection 闭环。
3. 补齐 Nav2 / SLAM / ros2_control / MoveIt2 / simulation 依赖。
4. 建立第一个室内导航仿真场景。

## 2026-10-07 — Phase 0.1: 首个可运行闭环

### 实现
- `homeagent_safety.policy`：纯 Python 可测试安全策略。
- `safety_node`：订阅 `/homeagent/action_proposal`，发布 approved/rejected。
- `mock_planner`：把测试用户指令转成结构化高层动作，用于 LLM 接入前的确定性验证。
- `scripts/demo_safety.sh`：可重复执行的安全闭环演示。

### 测试结果
- `colcon build --symlink-install`：2/2 ROS2 packages 构建成功。
- Safety policy 单元测试：4/4 passed。
- 端到端 ROS2 topic 测试：
  - `去客厅` -> `navigate(living_room)` -> ALLOW。
  - `把刀给小孩` -> `handover(kitchen_knife)` -> `DANGEROUS_HANDOVER_MINOR` 拦截。

### 环境问题记录
Tang 的用户级 Python 安装了 pytest 9.1.1，而 ROS2 Humble 的 `launch_testing` 与该版本存在 hook API 不兼容。ROS2 测试阶段暂时使用 `PYTHONNOUSERSITE=1` 调用 Ubuntu 自带 pytest 6.2.5；后续会把该约束固化到测试脚本/容器中，避免环境漂移。

## 2026-10-07 — Phase 0.2: 强类型协议 + 家庭记忆

### 强类型协议
- 新增 `homeagent_interfaces`。
- `ActionProposal / SafetyDecision / SkillResult` 替代核心动作链路中的自由字符串消息。
- 新增 `MemoryUpsert / MemoryQuery / MemoryObserve / MemoryCorrect` ROS2 service。

### 家庭记忆
- 新增 `homeagent_memory` SQLite 服务。
- 支持 object/person/place 等实体的版本化记录。
- 支持观测日志：zone、pose、confidence、source、timestamp。
- 支持人工纠正：field-level correction weight 与 source 元数据可审计保存。
- 端到端 ROS2 service 已验证：vision upsert -> camera observation -> human correction -> query。

### 测试
- 当前 ROS2 package 数：4（interfaces/safety/orchestrator/memory）。
- Safety + Memory 单元测试：7/7 passed。

## 2026-10-07 — Phase 0.3: 安全执行闭环

### 实现
- 新增 `homeagent_skills`。
- 所有技能只订阅 `/homeagent/action_approved`，不直接接受 Agent proposal。
- 技能层再次检查 `SafetyDecision.allowed`，形成双重 fail-closed。
- 当前使用 deterministic mock adapter；后续相同接口替换为 Nav2 / MoveIt2 / perception。
- planner 已接收 `SkillResult`，形成 proposal -> safety -> execution -> feedback 闭环。

### 实测
- `去客厅` -> navigate -> ALLOW -> `MOCK_NAVIGATION_COMPLETE` -> planner 收到结果。
- `把刀给小孩` -> handover -> `DANGEROUS_HANDOVER_MINOR` -> skill executor 无任何执行结果。
- 当前 `colcon build`：5/5 packages 成功。
- 当前单元测试：7/7 passed。

### 依赖环境处理
Tang 主机没有 passwordless sudo，因此不直接修改系统 ROS 安装。新增 Docker 可复现环境，通过本机已有 `ubuntu:22.04` 镜像安装 ROS2 Humble + Nav2 + SLAM Toolbox + ros2_control + MoveIt2 + Gazebo + TurtleBot3。这样既不污染主机，也能继续完成导航/建图/机械臂仿真。
