#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p artifacts/panda_physics

docker run --rm \
  --entrypoint bash \
  --network host --ipc host \
  -e ROS_DOMAIN_ID=79 \
  -e GAZEBO_MASTER_URI=http://127.0.0.1:11379 \
  -e HOMEAGENT_PANDA_GAZEBO_URDF=/workspace/artifacts/panda_physics/panda_gazebo.urdf \
  -v "$ROOT:/workspace" \
  homeagent-ros2:panda-visual -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws
    mkdir -p /tmp/ha_panda_colcon

    colcon --log-base /tmp/ha_panda_colcon/log build \
      --packages-select homeagent_manipulation homeagent_description \
      --build-base /tmp/ha_panda_colcon/build \
      --install-base /tmp/ha_panda_colcon/install \
      --symlink-install
    source /tmp/ha_panda_colcon/install/setup.bash

    python3 /workspace/scripts/generate_panda_physics_urdf.py \
      --output "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      --controllers /workspace/ros2_ws/src/homeagent_manipulation/config/panda_ros2_controllers.yaml

    gz sdf -p "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      >/workspace/artifacts/panda_physics/panda_gazebo.sdf

    setsid timeout 140 ros2 launch homeagent_manipulation \
      panda_gazebo_physics.launch.py \
      >/workspace/artifacts/panda_physics/launch.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 12
    echo "=== ROS2 topics before action ==="
    ros2 topic list -t | sort \
      >/workspace/artifacts/panda_physics/topics_before_action.txt
    echo "=== Gazebo finger topic candidates ==="
    grep -i -E "joint|finger|panda" \
      /workspace/artifacts/panda_physics/topics_before_action.txt || true
    set +e
    python3 /workspace/scripts/panda_gazebo_physics_e2e.py \
      > /workspace/artifacts/panda_physics/motion_report.log 2>&1
    RC=$?
    set -e

    echo "=== Panda Gazebo physics action test ==="
    cat /workspace/artifacts/panda_physics/motion_report.log
    echo "=== Gazebo/ros2_control key events ==="
    grep -Ei "gazebo_ros2_control|PandaGazeboSystem|panda_arm_controller|panda_hand_controller|spawned|spawn_entity|error|failed" \
      /workspace/artifacts/panda_physics/launch.log | tail -110 || true
    echo "=== Runtime log tail ==="
    tail -90 /workspace/artifacts/panda_physics/launch.log
    exit "$RC"
  '
