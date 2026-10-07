#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=48 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_moveit_colcon
    mkdir -p /tmp/homeagent_moveit_colcon
    colcon --log-base /tmp/homeagent_moveit_colcon/log build \
      --build-base /tmp/homeagent_moveit_colcon/build \
      --install-base /tmp/homeagent_moveit_colcon/install \
      --symlink-install
    source /tmp/homeagent_moveit_colcon/install/setup.bash

    xacro src/homeagent_manipulation/config/homearm.urdf.xacro >/tmp/homearm.urdf

    timeout 55 ros2 launch homeagent_manipulation homearm_moveit.launch.py \
      >/tmp/homeagent_moveit.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 7

    set +e
    python3 /workspace/scripts/moveit_smoke.py
    RC=$?
    set -e

    echo "=== MoveIt2 / HomeArm log tail ==="
    tail -220 /tmp/homeagent_moveit.log
    exit "$RC"
  '
