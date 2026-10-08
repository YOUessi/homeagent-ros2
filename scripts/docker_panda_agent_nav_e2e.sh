#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p artifacts/panda_agent_nav

# Isolated ROS2/Gazebo master; do not interfere with the existing live viewer.
docker run --rm \
  --entrypoint bash --network host --ipc host \
  -e ROS_DOMAIN_ID=81 \
  -e GAZEBO_MASTER_URI=http://127.0.0.1:11381 \
  -e HOMEAGENT_PANDA_MOUNT=homebot \
  -e PANDA_GAZEBO_ENTITY=homebot_panda \
  -e HOMEAGENT_PANDA_GAZEBO_URDF=/workspace/artifacts/panda_agent_nav/homebot_panda.urdf \
  -e HOMEAGENT_PANDA_AGENT_REPORT=/workspace/artifacts/panda_agent_nav/e2e_report.json \
  -v "$ROOT:/workspace" \
  homeagent-ros2:panda-visual -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    mkdir -p /tmp/homeagent_panda_agent_nav_colcon
    colcon --log-base /tmp/homeagent_panda_agent_nav_colcon/log build \
      --build-base /tmp/homeagent_panda_agent_nav_colcon/build \
      --install-base /tmp/homeagent_panda_agent_nav_colcon/install \
      --symlink-install --packages-skip homeagent_gazebo_plugins
    source /tmp/homeagent_panda_agent_nav_colcon/install/setup.bash

    python3 /workspace/scripts/generate_panda_physics_urdf.py \
      --output "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      --controllers /workspace/ros2_ws/src/homeagent_manipulation/config/panda_ros2_controllers.yaml \
      --mount homebot
    gz sdf -p "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      >/workspace/artifacts/panda_agent_nav/homebot_panda.sdf

    echo "=== Single ROS2 launch context, staggered stages ==="
    setsid timeout 165 ros2 launch homeagent_bringup \
      panda_agent_nav.launch.py \
      >/workspace/artifacts/panda_agent_nav/launch.log 2>&1 &
    LAUNCH_PID=$!
    cleanup() {
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 20
    python3 /workspace/scripts/seed_home_memory.py \
      >/workspace/artifacts/panda_agent_nav/seed_memory.log

    # Passive 2Hz measurements: controller output, smoothed velocity,
    # physical odometry, AMCL and map->odom timestamps. No robot commands.
    python3 /workspace/scripts/nav2_telemetry.py \
      --output /workspace/artifacts/panda_agent_nav/nav2_telemetry.jsonl \
      --ros-args -p use_sim_time:=true \
      >/workspace/artifacts/panda_agent_nav/nav2_telemetry.log 2>&1 &
    MONITOR_PID=$!
    sleep 1

    set +e
    python3 /workspace/scripts/panda_agent_nav_e2e.py \
      --ros-args -p use_sim_time:=true \
      > /workspace/artifacts/panda_agent_nav/e2e_probe.log 2>&1
    RC=$?
    set -e
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true

    timeout 8 python3 /workspace/scripts/summarize_nav2_telemetry.py \
      --telemetry /workspace/artifacts/panda_agent_nav/nav2_telemetry.jsonl \
      --nav-log /workspace/artifacts/panda_agent_nav/launch.log \
      --output /workspace/artifacts/panda_agent_nav/nav2_telemetry_summary.json \
      >/workspace/artifacts/panda_agent_nav/nav2_telemetry_summary.log \
      2>&1 || echo "WARN: passive telemetry summary unavailable"

    echo "=== E2E result ==="
    tail -100 /workspace/artifacts/panda_agent_nav/e2e_probe.log
    echo "=== Agent/Safety trace ==="
    grep -Ei "PROPOSE action|CONTEXT request|PANDA_MOVEIT_SEND|NAV2_SEND|SKILL_RESULT|FORBIDDEN_ZONE" \
      /workspace/artifacts/panda_agent_nav/launch.log | tail -75 || true
    echo "=== Nav2 control trace ==="
    grep -Ei "Failed to make progress|Aborting handle|Unable to transform robot pose|Reached the goal|Goal succeeded|NAV2_POSTCONDITION_FAILED" \
      /workspace/artifacts/panda_agent_nav/launch.log | tail -35 || true
    exit "$RC"
  '
