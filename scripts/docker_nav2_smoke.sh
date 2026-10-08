#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=47 \
  -e HOMEAGENT_NAV2_REPORT=/workspace/artifacts/nav2_demo_report.json \
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

    test -s /tmp/homeagent_colcon/install/homeagent_navigation/share/homeagent_navigation/maps/home_room.yaml

    setsid timeout 75 ros2 launch homeagent_navigation homebot_nav2.launch.py \
      >/tmp/homeagent_nav2_stack.log 2>&1 &
    NAV_PID=$!

    setsid timeout 75 ros2 launch homeagent_bringup homeagent_core.launch.py \
      use_nav2:=true use_deepseek:=false use_sim_time:=true \
      memory_db:=/tmp/homeagent_nav2_memory.sqlite3 \
      >/tmp/homeagent_core_nav2.log 2>&1 &
    CORE_PID=$!

    cleanup() {
      kill -TERM -- "-$NAV_PID" "-$CORE_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$NAV_PID" "-$CORE_PID" 2>/dev/null || true
      wait "$NAV_PID" "$CORE_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 12
    python3 /workspace/scripts/seed_home_memory.py >/tmp/homeagent_seed.log

    set +e
    python3 /workspace/scripts/nav2_e2e_smoke.py --ros-args -p use_sim_time:=true
    RC=$?
    set -e

    echo "=== Seeded household memory ==="
    cat /tmp/homeagent_seed.log
    echo "=== HomeAgent core log tail ==="
    tail -100 /tmp/homeagent_core_nav2.log
    echo "=== Nav2 stack log tail ==="
    tail -180 /tmp/homeagent_nav2_stack.log
    exit "$RC"
  '
