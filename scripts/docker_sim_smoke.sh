#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

docker run --rm \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=43 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    xacro src/homeagent_description/urdf/homebot.urdf.xacro >/tmp/homebot.urdf

    timeout 35 ros2 launch homeagent_description homebot_gazebo.launch.py gui:=false \
      >/tmp/homeagent_gazebo.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 8
    python3 /workspace/scripts/sim_smoke.py
    RC=$?

    echo "=== Gazebo log tail ==="
    tail -40 /tmp/homeagent_gazebo.log
    exit "$RC"
  '
