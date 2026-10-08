#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p artifacts/panda_mobile

docker run --rm \
  --entrypoint bash --network host --ipc host \
  -e ROS_DOMAIN_ID=80 \
  -e GAZEBO_MASTER_URI=http://127.0.0.1:11380 \
  -e HOMEAGENT_PANDA_MOUNT=homebot \
  -e PANDA_GAZEBO_ENTITY=homebot_panda \
  -e HOMEAGENT_PANDA_GAZEBO_URDF=/workspace/artifacts/panda_mobile/homebot_panda.urdf \
  -v "$ROOT:/workspace" \
  homeagent-ros2:panda-visual -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    mkdir -p /tmp/ha_panda_mobile
    colcon --log-base /tmp/ha_panda_mobile/log build \
      --packages-select homeagent_manipulation homeagent_description \
      --build-base /tmp/ha_panda_mobile/build \
      --install-base /tmp/ha_panda_mobile/install --symlink-install
    source /tmp/ha_panda_mobile/install/setup.bash

    python3 /workspace/scripts/generate_panda_physics_urdf.py \
      --output "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      --controllers /workspace/ros2_ws/src/homeagent_manipulation/config/panda_ros2_controllers.yaml \
      --mount homebot

    gz sdf -p "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      >/workspace/artifacts/panda_mobile/homebot_panda.sdf

    setsid timeout 135 ros2 launch homeagent_manipulation \
      panda_gazebo_physics.launch.py \
      >/workspace/artifacts/panda_mobile/launch.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 12
    set +e
    python3 /workspace/scripts/panda_homebot_gazebo_e2e.py \
      > /workspace/artifacts/panda_mobile/probe.log 2>&1
    RC=$?
    set -e
    echo "=== MOBILE PANDA PROBE ==="
    cat /workspace/artifacts/panda_mobile/probe.log
    echo "=== MOBILE PANDA GAZEBO EVENTS ==="
    grep -Ei "spawn|PandaGazeboSystem|panda_arm_controller|panda_hand_controller|error|failed|diff_drive|lidar" \
      /workspace/artifacts/panda_mobile/launch.log | tail -90 || true
    exit "$RC"
  '
