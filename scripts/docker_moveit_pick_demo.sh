#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=55 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_pick_colcon
    mkdir -p /tmp/homeagent_pick_colcon
    colcon --log-base /tmp/homeagent_pick_colcon/log build \
      --build-base /tmp/homeagent_pick_colcon/build \
      --install-base /tmp/homeagent_pick_colcon/install \
      --symlink-install
    source /tmp/homeagent_pick_colcon/install/setup.bash

    timeout 65 ros2 launch homeagent_manipulation homearm_moveit.launch.py \
      >/tmp/homearm_pick_moveit.log 2>&1 &
    ARM_PID=$!

    timeout 65 ros2 launch homeagent_bringup homeagent_core.launch.py \
      use_moveit:=true use_nav2:=false use_deepseek:=false \
      memory_db:=/tmp/homeagent_pick_memory.sqlite3 \
      >/tmp/homeagent_pick_core.log 2>&1 &
    CORE_PID=$!

    cleanup() {
      kill "$ARM_PID" "$CORE_PID" 2>/dev/null || true
      wait "$ARM_PID" "$CORE_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 8
    python3 /workspace/scripts/seed_home_memory.py >/tmp/homeagent_pick_seed.log

    set +e
    python3 /workspace/scripts/moveit_pick_e2e.py
    RC=$?
    set -e

    echo "=== Seeded trusted memory ==="
    cat /tmp/homeagent_pick_seed.log
    echo "=== HomeAgent pick trace ==="
    grep -E "PROPOSE|CONTEXT|ALLOW|REJECT|MOVEIT_SEND|SKILL_RESULT|RESULT" \
      /tmp/homeagent_pick_core.log | tail -100 || true
    echo "=== MoveIt / ros2_control log tail ==="
    tail -120 /tmp/homearm_pick_moveit.log
    exit "$RC"
  '
