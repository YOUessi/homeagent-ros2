# 2026-10-09 — Isaac Sim branch kickoff

## Decision
Migrate high-fidelity rendering, physically simulated manipulation and sensor development to Isaac Sim. Keep Gazebo Classic regression baseline in its original branch, not replace its artifacts or claim equivalent physics.

## Verified machine environment (Tang)
Ubuntu 22.04.5 x86_64; RTX 4090 Laptop GPU (NOT desktop 24 GiB), 16,376 MiB VRAM, NVIDIA driver 580.178.04; 31 GiB RAM (around 16 GiB available at probe), 71 GiB available disk on root filesystem.

## Constraints and risks
- GPU VRAM is at the minimum of Isaac Sim 5.1, with little margin for several RTX sensors and elaborate environments.
- RAM is close to 5.1 minimum. Disk available (71 GB) is enough for a basic application but not unrestricted packs and caches.
- Need check official compatibility and effective runtime before choosing a working version. No Isaac Sim run is claimed yet.
- High-fidelity requirement includes genuine Panda and HomeBot assets, correct kinematic assembly, plausible contact and lighting, not a cosmetic primitive swap.
- Network/cloud asset access needs checking.
- Visual acceptance must include an inspected screenshot plus a time-continuous recording, not merely process logs.

## Artifacts
`isaac/franka_stage.py` is a first standalone scene smoke, not an integrated HomeBot nor grasp success.
`scripts/check_isaac_tang.sh` is a read-only environment check.

## Next tests
1. Confirm Isaac Sim installed or obtain package and run compatibility checker.
2. Verify official Franka USD asset can load, rendered screenshot looks correct, and joint articulation steps.
3. Mount Panda on HomeBot and test articulation stability.
4. Verify gripper friction-only grasp vs fixed-constraint baseline, drop/release.
5. Connect existing ROS2 Agent tools and publish browser-observable continuous replay.

## 实机第一次验证（02:14–02:17 +08:00）
- GPU: RTX 4090 Laptop 16376 MiB，5926 MiB 已用，9994 MiB 空闲；主占用 PID 556279 属于 **CodeRL-Lab EXP-004N**，未停止。
- 主内存 31 GiB，约 14 GiB 可用，磁盘剩余 71 GB；未启用 swap。
- **发现原本已有** /home/you/conda_env/env_isaaclab (Python 3.10.18, Isaac Sim 4.5.0.0, Isaac Lab 0.41.1)，以及 /home/you/programfiles/IsaacLab 官方示例。之前检查 /opt/isaac-sim 未检出旧版，现改为复用旧版，不先装完整 5.1。
- 已创建独立 homeagent-isaacsim Python 3.11 Conda 环境作为备用，但暂不往此环境下载完整 5.1，防止占用过多磁盘。
- 使用 `isaac/franka_45_smoke.py` 在现有 4.5 环境运行 `--headless --steps 30`，日志收到 **HOMEAGENT_ISAAC_READY** (9 joints, 11 bodies) 与 **HOMEAGENT_ISAAC_SMOKE_PASS**；未宣称 GUI、HomeBot 组合或夹爪物理抓取已经验证。
- Tang 首轮原始日志为 `/tmp/homeagent_isaac45_test.log`。该验证只证明 Isaac Sim 引擎加载与 30 步仿真；进程退出状态需独立检查。
- 下一验收必须以真实渲染截图+连续动作录像为凭证，不能以 headless 日志代替。不得中断 CodeRL-Lab GPU 任务。
