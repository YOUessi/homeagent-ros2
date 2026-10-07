#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=44 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_colcon
    mkdir -p /tmp/homeagent_colcon
    colcon --log-base /tmp/homeagent_colcon/log build \
      --build-base /tmp/homeagent_colcon/build \
      --install-base /tmp/homeagent_colcon/install \
      --symlink-install
    source /tmp/homeagent_colcon/install/setup.bash

    timeout 45 ros2 launch homeagent_navigation homebot_slam.launch.py \
      >/tmp/homeagent_slam.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 9
    set +e
    python3 /workspace/scripts/slam_smoke.py
    RC=$?
    set -e

    echo "=== SLAM log tail ==="
    tail -160 /tmp/homeagent_slam.log
    exit "$RC"
  '
