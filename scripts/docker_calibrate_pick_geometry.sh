#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 scripts/generate_homebot_arm_gazebo_urdf.py >/dev/null

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=64 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_calibration_colcon
    mkdir -p /tmp/homeagent_calibration_colcon
    colcon --log-base /tmp/homeagent_calibration_colcon/log build \
      --build-base /tmp/homeagent_calibration_colcon/build \
      --install-base /tmp/homeagent_calibration_colcon/install \
      --symlink-install
    source /tmp/homeagent_calibration_colcon/install/setup.bash
    export GAZEBO_PLUGIN_PATH="/tmp/homeagent_calibration_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"

    setsid timeout 65 ros2 launch homeagent_manipulation \
      homebot_arm_gazebo_moveit.launch.py \
      >/tmp/homeagent_pick_calibration.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    READY=0
    for _ in $(seq 1 50); do
      SERVICES="$(ros2 service list 2>/dev/null || true)"
      TOPICS="$(ros2 topic list 2>/dev/null || true)"
      if echo "$SERVICES" | grep -qx "/controller_manager/list_controllers" \
        && echo "$TOPICS" | grep -qx "/gazebo/link_states" \
        && echo "$TOPICS" | grep -qx "/joint_states"; then
        READY=1
        break
      fi
      sleep 0.5
    done

    if [ "$READY" -ne 1 ]; then
      echo "ERROR: calibration runtime did not become ready"
      tail -180 /tmp/homeagent_pick_calibration.log
      exit 3
    fi

    python3 /workspace/scripts/calibrate_pick_geometry.py
  '
