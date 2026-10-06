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
- SLAM
- Nav2
- ros2_control
- MoveIt2
- perception nodes

### C. 主脑调度层
- 状态机 / Behavior Tree
- skill registry
- task executor
- retry / timeout / cancel
- observation feedback

### D. 安全层
所有 Agent 动作先进入 `homeagent_safety`：
- 动作白名单
- 急停约束
- 危险物体与人员规则
- 禁入区域
- 权限与上下文检查
- 审计日志

### E. Agent 层
- DeepSeek / 可替换 LLM
- tool calling
- task planning
- scene reasoning
- memory retrieval/update
- speech interface

## 2. 执行闭环

```text
observation_t
    |
    v
Agent -> structured action proposal
    |
    v
Safety policy(state_t, action_t)
    |
    +---- reject ----> reason -> Agent replans
    |
    v
approved action
    |
    v
ROS2 skill executor
    |
    v
result / observation_(t+1)
```

## 3. 设计约束

1. LLM 不发布底盘速度、关节轨迹等低层控制量。
2. Agent 只输出受 schema 约束的高层技能调用。
3. 安全规则层独立于 LLM，可单元测试、可审计。
4. 仿真先行，真机接口与仿真接口保持同一 skill contract。
5. 所有关键动作具备 request id、结果状态和日志。
