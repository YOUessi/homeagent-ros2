#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=56 \
  -e HOMEAGENT_MOBILE_REPORT=/workspace/artifacts/mobile_manipulator_report.json \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_mobile_colcon
    mkdir -p /tmp/homeagent_mobile_colcon
    colcon --log-base /tmp/homeagent_mobile_colcon/log build \
      --build-base /tmp/homeagent_mobile_colcon/build \
      --install-base /tmp/homeagent_mobile_colcon/install \
      --symlink-install
    source /tmp/homeagent_mobile_colcon/install/setup.bash

    timeout 110 ros2 launch homeagent_bringup mobile_manipulator_demo.launch.py \
      >/tmp/homeagent_mobile_manipulator.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 14
    python3 /workspace/scripts/seed_home_memory.py >/tmp/homeagent_mobile_seed.log

    set +e
    python3 /workspace/scripts/mobile_manipulator_e2e.py \
      --ros-args -p use_sim_time:=true
    RC=$?
    set -e

    echo "=== Mobile manipulator HomeAgent trace ==="
    grep -E "PROPOSE|CONTEXT|ALLOW|REJECT|NAV2_SEND|MOVEIT_SEND|GRIPPER_CLOSE|PLANNING_SCENE_ATTACH|SKILL_RESULT|RESULT" \
      /tmp/homeagent_mobile_manipulator.log | tail -160 || true
    echo "=== Integrated stack log tail ==="
    tail -220 /tmp/homeagent_mobile_manipulator.log
    exit "$RC"
  '
