#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

echo "=== HomeAgent-ROS2 Environment ==="
echo "ROS_DISTRO=${ROS_DISTRO:-unset}"
echo "ros2=$(command -v ros2 || true)"
echo "colcon=$(command -v colcon || true)"
echo "python3=$(command -v python3 || true)"

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
fi

echo
echo "=== Required ROS package presence ==="
for pkg in robot_state_publisher tf2_ros nav2_bringup slam_toolbox moveit_ros_move_group controller_manager; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "[OK]      $pkg"
  else
    echo "[MISSING] $pkg"
  fi
done
