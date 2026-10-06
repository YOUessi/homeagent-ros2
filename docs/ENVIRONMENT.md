# Tang 环境基线

记录日期：2026-10-07

## 已确认

- Host: Tang
- Project root: `/home/you/projects/homeagent-ros2`
- GPU: NVIDIA GeForce RTX 4090 Laptop GPU, 16376 MiB
- NVIDIA Driver: 580.178.04
- ROS2: Humble
- ROS2 prefix: `/opt/ros/humble`
- `ros2`: `/opt/ros/humble/bin/ros2`
- `colcon`: `/usr/bin/colcon`
- System Python: `/usr/bin/python3`
- Conda: `/home/you/anaconda3/bin/conda`
- Existing conda env `env_isaaclab`: Python 3.10.18

## 当前缺失/尚未发现

ROS2 当前安装中尚未发现以下关键组件：
- Nav2
- SLAM Toolbox
- MoveIt2
- ros2_control / controller_manager
- Gazebo ROS bridge
- Isaac Sim executable

因此第一阶段先保持 ROS2 core 可构建，随后按模块补齐依赖。

## 环境策略

ROS2 Humble 的 apt 二进制包与 Ubuntu 系统 Python 3.10 绑定，因此 ROS2 核心节点优先使用系统 ROS 环境，不强行塞入 Conda。

Agent / perception / 本地模型后续可单独使用 Conda，与 ROS2 通过 DDS、ROS topic/service/action 或明确的 bridge 交互，避免 Python ABI 与环境污染问题。
