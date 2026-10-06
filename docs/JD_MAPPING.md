# JD 对照与项目交付矩阵

| JD 能力 | HomeAgent-ROS2 对应模块 | 验收目标 |
|---|---|---|
| 多传感器接入 | sensor adapters | Camera/LiDAR/Mic topic 正常发布 |
| SLAM 室内建图 | simulation + SLAM Toolbox | 仿真室内地图生成与保存 |
| 自主导航避障 | Nav2 integration | 多目标点导航、动态障碍避让 |
| 机械臂运动控制 | MoveIt2 + ros2_control | 探头/俯仰/转向/抓取动作 |
| 主脑调度 | orchestrator | 状态、任务、取消、超时、重试 |
| 安全法则校验 | homeagent_safety | 危险/越权动作可解释拦截 |
| 家庭档案数据库 | memory service | 地图、物品、人、观测、记忆权重 |
| DeepSeek Agent | agent gateway | 结构化 tool calling，不直控硬件 |
| 语音唤醒与交互 | speech module | wake -> ASR -> Agent -> TTS |
| 场景推理 | perception + agent | 场景观测转结构化上下文 |
| 仿真先验验证 | Gazebo/Isaac | scripted scenarios 全通过 |
| 完整交付 | docs + tests + scripts | 源码、部署、运维、测试用例 |
