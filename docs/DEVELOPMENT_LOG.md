# 开发日志

> 本文件从 2026-10-07 起作为高层索引/阶段摘要。完整的按日期工作过程、验证结果、失败、问题定位与修复记录，以 `docs/journal/YYYY-MM-DD.md` 为准。日志规范见 `docs/journal/README.md`。

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

## 2026-10-07 — Phase 0.4: DeepSeek Agent 接入骨架

### 实现
- 新增 `planner_core.py`：严格解析单步高层动作 JSON，拒绝未知/底层控制动作。
- 新增 `deepseek_client.py`：OpenAI-compatible chat/completions HTTP client，仅从环境变量读取密钥。
- 新增 `deepseek_planner` ROS2 node。
- LLM 输出中的 `context` 被主动丢弃；recipient_age、object_tags、forbidden_zones 等安全上下文不允许由 LLM 自证。
- manipulation (`pick/place/handover`) 在没有 `safety_context_trusted=true` 时直接 fail-closed，后续由可信世界状态/记忆模块补齐。

### 测试
- Planner parser 新增 5 个测试：正常导航、Markdown JSON、丢弃 LLM safety context、拒绝低层控制、缺少必填参数。
- 总单元测试：13/13 passed。
- 无 `DEEPSEEK_API_KEY` 实测：节点发布 `NO_API_KEY` 并且不产生 ActionProposal。

## 2026-10-07 — Phase 0.5: 机器人模型、家庭场景与 SLAM 骨架

### 实现
- 新增 `homeagent_description`：自研 HomeBot 差速移动底盘。
- 机器人模型包含左右驱动轮、caster、2D LiDAR、前向 RGB Camera。
- 加入 Gazebo diff-drive、ray sensor、camera 插件。
- 新增 `home_room.world`：6m x 6m 家庭房间，包含墙体、沙发、桌子、柜体障碍物。
- 新增 headless `homebot_gazebo.launch.py`，用于自动化 CI/远程测试。
- 新增 `homeagent_navigation`，接入 SLAM Toolbox 参数和仿真 SLAM 启动文件。
- 新增 `homeagent_bringup`，可一键启动 safety / memory / skills / mock-or-DeepSeek planner。
- 新增 Gazebo smoke test：要求 `/scan` 有有效测距且 `/cmd_vel` 能产生可观测里程计位移。

### 当前验证
- workspace HomeAgent package 数：8。
- 8/8 package `colcon build --symlink-install` 通过。
- `homeagent_core.launch.py` 已实测：四个核心 node 正常拉起，`去客厅` 完成 proposal -> safety -> skill -> result 闭环，launch 退出后无残留节点。
- Gazebo/SLAM 运行时验证等待 Docker 完整依赖镜像构建完成后执行；主机不使用 sudo 安装，保持宿主环境不污染。

## 2026-10-07 — Phase 0.6: 可信世界状态边界 + Nav2 技能适配器

### 可信上下文
- 新增 `homeagent_context`，将 Agent 的 `/homeagent/action_candidate` 与家庭档案/固定策略合并后，才生成 `/homeagent/action_proposal`。
- LLM 不再有能力自行声明 `recipient_age`、`object_tags`、`forbidden_zones` 或 `safety_context_trusted`。
- manipulation 对象不存在家庭记忆时 fail-closed；已知对象才可进入安全引擎继续判定。
- forbidden zone 来自 context node 的可信策略参数，而不是 Agent 输出。

### 端到端验证
- 已知 `cup` 写入 memory 后：`pick(cup)` -> Context trusted=True -> Safety ALLOW -> mock skill 完成。
- 未知对象：Context trusted=False -> Safety `UNTRUSTED_SAFETY_CONTEXT`，不会进入 skill executor。
- `utility_room`：Context 从策略注入 forbidden zone -> Safety `FORBIDDEN_ZONE`，3 秒观察窗口内没有任何 skill result。
- `demo_core.sh` 已更新为完整 candidate -> context -> safety -> skill -> feedback 演示。

### Nav2 adapter
- 新增 `nav2_skill_executor`，只消费 Safety APPROVED 的 `navigate` 动作。
- semantic target 转为 `NavigateToPose` map-frame goal；当前内置 living_room / kitchen / bedroom / hallway 坐标。
- Nav2 server 不可用、目标未知、goal rejected、执行失败均显式返回 `SkillResult`，Agent 不直接发布 `/cmd_vel`。

### 回归
- workspace package 数：9。
- `colcon build`：9/9 成功。
- 单元测试：17/17 passed。
