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
- `homeagent_orchestrator`：主脑调度层。DeepSeek / mock planner 只产生 meta-task 或高层动作；共享的确定性任务状态机负责 `fetch` 的分步执行、命令幂等、恢复预算和 postcondition verification，避免 LLM 直接拥有执行控制权。
- `homeagent_context`：把不可信 Agent candidate 与家庭档案/策略/最新 perception observation 合并为可信 world-state context；支持房间语义位置与动态 `object_pregrasp:*` 目标，并对 stale / missing observation fail-closed。
- `homeagent_skills`：安全门之后的技能执行适配层；已有 mock、真实 Nav2 adapter 与真实 MoveIt2 adapter。Nav2 只执行 Household Memory 解析出的可信 `map_pose`；MoveIt2 只消费 Safety-approved 的机械臂高层动作，`pick` 的接近关节目标同样来自 Object Memory，而不是 Agent 输出。
- `homeagent_memory`：SQLite 家庭档案、物品/人员记忆、观测日志与人工纠正权重。
- `homeagent_description`：自研 HomeBot 差速底盘、LiDAR、Camera 与家庭房间 Gazebo 模型。
- `homeagent_navigation`：SLAM Toolbox / Nav2 配置与仿真导航入口。
- `homeagent_perception`：仿真/真实传感器感知适配边界；当前 Gazebo ground-truth adapter 会将 world pose 对齐到 ROS map frame 并写入 Household Memory，后续可替换 RGB-D/YOLO/VLM。
- `homeagent_manipulation`：自研 4-DOF HomeArm、MoveIt2/OMPL 与 ros2_control。保留 GenericSystem 快速回归路径，同时新增 `gazebo_ros2_control/GazeboSystem` 物理关节路径，可将 HomeArm 真实固定到 HomeBot 模型上运行。
- `homeagent_bringup`：核心服务一键启动与 planner / real-skill adapter 切换。

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

当前 workspace 的 11 个非 Gazebo-plugin package 均可 `colcon build`，最新纯逻辑/策略回归为 67/67 passed；Docker 物理路径同时构建 `homeagent_gazebo_plugins`，共 12 packages。系统已验证 Gazebo HomeBot、SLAM、Perception → Household Memory、Memory-backed Nav2、HomeArm MoveIt2 + gripper、`gazebo_ros2_control/GazeboSystem`、Agent fetch 状态机以及双指 contact-gated grasp。`fetch(cup)` 现在由确定性状态机执行 `stow → navigate(object_pregrasp:cup) → pick → verify`，并带命令幂等、Safety rejection abort、Nav2 retry、pick recovery 与 postcondition verification。抓取阶段在 full carry 前必须先通过小幅 grasp proof motion；未确认抓持时 `carry_executed=false`。当前 Gazebo 抓持严格表述为“双指接触门控后建立 fixed constraint”，不是纯摩擦维持；完整自主桌面视觉抓取仍未完成。

DeepSeek 节点只读取环境变量 `DEEPSEEK_API_KEY`，密钥不会进入代码、ROS topic 或日志；没有密钥时节点 fail-closed，只发布 `NO_API_KEY` 错误，不产生机器人动作。

## 快速验证

```bash
./scripts/build_and_test.sh
./scripts/demo_core.sh

# 完整移动机器人 Demo：
# “去客厅” -> Context -> Safety -> Nav2 -> Gazebo 真移动 -> 轨迹图
./scripts/demo_nav2.sh

# HomeArm MoveIt2 standalone：OMPL -> FollowJointTrajectory -> /joint_states
./scripts/docker_moveit_smoke.sh

# HomeAgent 机械臂闭环：
# “机械臂检查一下” -> Context -> Safety -> MoveIt2 -> ros2_control
./scripts/docker_moveit_agent_demo.sh

# Object Memory 驱动的抓取链：
# “拿起水杯” -> Memory -> Safety -> MoveIt2 approach
# -> gripper close -> PlanningScene logical attach
./scripts/docker_moveit_pick_demo.sh

# 当前最完整的联合 Demo：
# 去客厅 -> Nav2 真移动 -> 拿水杯 -> MoveIt2 + gripper + attach
./scripts/demo_mobile_manipulator.sh

# HomeBot + HomeArm 组合模型的 Gazebo 物理关节验证：
# MoveIt2 -> FollowJointTrajectory -> gazebo_ros2_control -> Gazebo joints
./scripts/docker_gazebo_homearm_physics_smoke.sh
```

`demo_nav2.sh` 会额外生成 `artifacts/nav2_demo_path.png`、机器可读 JSON 和完整运行日志；所有运行产物默认被 Git 忽略。

开发过程按日期记录在 `docs/journal/YYYY-MM-DD.md`；`docs/DEVELOPMENT_LOG.md` 只保留阶段摘要。每天必须记录实际过程、验证结果、失败、问题定位与修复，不把“代码已写”混同为“已验证通过”。

### Gazebo 接触门控约束 Demo

运行 `./scripts/demo_gazebo_contact_grasp.sh`。该实验要求两个夹爪手指都与目标杯子产生 Gazebo contact，并在夹爪达到闭合阈值后建立 Gazebo fixed constraint，再执行 carry。它不使用 MoveIt PlanningScene attach；同时明确不是纯摩擦维持，也不是自主桌面抓取。

### 当前最完整物理联合 Demo

运行 `./scripts/demo_physical_mobile_manipulator.sh`。同一 Gazebo 组合机器人依次执行“去客厅”和“拿起水杯”：Nav2 使用 Household Memory 的可信房间坐标完成真实移动；操作阶段使用 Object Memory 的可信目标、GazeboSystem 关节动力学与双指 contact gate 完成 carry。仍明确标记为非纯摩擦、非自主桌面获取。


## 三维机器人演示（2026-10-08，开发分支）

**请勿混淆二维监控页、Gazebo 物理世界和 RViz 机械臂运动演示。**

- 原生 **Gazebo Classic 3D**：运行 `./scripts/start_gazebo_3d_view.sh`，使用 Xvfb + noVNC 在浏览器中查看真实 Gazebo 3D 场景（沿用已运行的 HomeBot/HomeArm Gazebo gzserver）。[现场截图](docs/images/gazebo_native_20261008.png)。当前自制 HomeArm 只有四个转动关节，外形为 primitive geometry，不能代表工业机械臂外观。
- 官方 **Franka Panda 7-DOF**：先构建 `docker/Dockerfile.visual` 与 `docker/Dockerfile.panda` 所对应镜像，再运行 `./scripts/start_panda_3d_demo.sh` 启动隔离的 RViz + MoveIt2 + ros2_control `mock_components` demo。浏览器连接地址由脚本打印；[真实 CAD Mesh 截图](docs/images/panda_rviz_20261008.png)。
- Panda 的关节运动与夹爪开合可用 `docker exec homeagent-panda-demo bash -lc 'source /opt/ros/humble/setup.bash && python3 /workspace/scripts/panda_moveit_joint_demo.py'` 演示，MoveIt2 两次 15-point 规划执行及 gripper action 已实测通过。

**验证边界（已于 2026-10-08 更新）**：上方 Panda RViz 链接仍是模拟控制器展示；但新开发分支已额外完成 Panda 七轴 + 夹爪的真实 GazeboSystem 物理控制，以及与 HomeBot 底盘合并为单机器人后，受控的 Agent / Safety / Nav2 / MoveIt2 联合运动测试。**自主物品接触抓取仍未完成**。不要把 RViz 模拟控制器、Gazebo 物理运动、真实接触抓取混为一谈。


## Panda + HomeBot 同体物理运动 / Agent 安全导航（2026-10-08，开发分支）

本开发分支 feature/panda-gazebo-physics-20261008 在前述官方 Panda RViz 演示基础上，额外实现一个 Gazebo Classic 物理机器人 homebot_panda：HomeBot 底盘、两轮差速、激光雷达、里程计与官方 Panda 7 轴外观 Mesh、双指夹爪合并到**同一个机器人**。Panda 7 轴由 gazebo_ros2_control/GazeboSystem 和 MoveIt2 控制；不是 mock_components。

单条复现入口（均在 Tang Docker 中运行，与主实时 Demo 的 ROS Domain 隔离）：

~~~bash
./scripts/build_and_test.sh
./scripts/docker_panda_gazebo_physics_smoke.sh
./scripts/docker_homebot_panda_smoke.sh
./scripts/docker_panda_agent_nav_e2e.sh
~~~

其中 Agent 联合验收使用**确定性 mock planner**，按用户命令“机械臂检查一下 / 收拢机械臂 / 去客厅”逐一经过 Trusted Context → Safety Rule Engine → ROS2 Panda/导航 Skills。Panda 不收拢时不允许导航；禁区上下文伪造被拒绝。该验证**没有**宣称已经从桌面自主抓起杯子，也没有在线调用 DeepSeek 模型。

最新有效 r9：11 packages build、67/67 pytest；MoveIt2 七轴实际运动约 0.611rad；Nav2 一次导航成功，底盘实际运动约 0.683m、定位目标误差 0.116m，0 次进度恢复；Panda 未收拢/禁区指令均阻断。DWB 局部规划器与 GoalChecker 的停车容差已同步为 0.12m，停止判定速度设为 0.05m/s。

[完整过程与失败记录](docs/journal/2026-10-08.md) · [r9 端到端机器报告](docs/journal/traces/2026-10-08-panda-nav-e2e-r9.json) · [r8 问题遥测](docs/journal/traces/2026-10-08-nav2-telemetry-r8.json) · [r9 验收遥测](docs/journal/traces/2026-10-08-nav2-telemetry-r9.json)
