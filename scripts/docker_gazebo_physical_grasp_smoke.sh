#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 scripts/generate_homebot_arm_gazebo_urdf.py >/dev/null

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=58 \
  -e HOMEAGENT_CONTACT_REPORT=/workspace/artifacts/gazebo_contact_grasp_report.json \
  -e HOMEAGENT_GRASP_CLOSE="${HOMEAGENT_GRASP_CLOSE:-0.004}" \
  -e HOMEAGENT_CUP_Y="${HOMEAGENT_CUP_Y:-0.002}" \
  -e HOMEAGENT_CARRY_SCALE="${HOMEAGENT_CARRY_SCALE:-0.20}" \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_grasp_colcon
    mkdir -p /tmp/homeagent_grasp_colcon
    colcon --log-base /tmp/homeagent_grasp_colcon/log build \
      --build-base /tmp/homeagent_grasp_colcon/build \
      --install-base /tmp/homeagent_grasp_colcon/install \
      --symlink-install
    source /tmp/homeagent_grasp_colcon/install/setup.bash
    export GAZEBO_PLUGIN_PATH="/tmp/homeagent_grasp_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"

    timeout 90 ros2 launch homeagent_manipulation \
      homebot_arm_gazebo_moveit.launch.py \
      >/tmp/homeagent_physical_grasp.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 11
    set +e
    python3 /workspace/scripts/gazebo_contact_carry_smoke.py
    RC=$?
    set -e

    echo "=== Physical grasp Gazebo log tail ==="
    tail -260 /tmp/homeagent_physical_grasp.log
    exit "$RC"
  '
