#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 scripts/generate_homebot_arm_gazebo_urdf.py >/dev/null

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=61 \
  -e HOMEAGENT_PHYSICAL_MOBILE_REPORT=/workspace/artifacts/physical_mobile_manipulator_report.json \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_physical_mobile_colcon
    mkdir -p /tmp/homeagent_physical_mobile_colcon
    colcon --log-base /tmp/homeagent_physical_mobile_colcon/log build \
      --build-base /tmp/homeagent_physical_mobile_colcon/build \
      --install-base /tmp/homeagent_physical_mobile_colcon/install \
      --symlink-install
    source /tmp/homeagent_physical_mobile_colcon/install/setup.bash
    export GAZEBO_PLUGIN_PATH="/tmp/homeagent_physical_mobile_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"

    timeout 125 ros2 launch homeagent_bringup \
      physical_mobile_manipulator_demo.launch.py \
      >/tmp/homeagent_physical_mobile.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 16
    python3 /workspace/scripts/seed_home_memory.py >/tmp/homeagent_physical_mobile_seed.log

    set +e
    python3 /workspace/scripts/physical_mobile_manipulator_e2e.py \
      --ros-args -p use_sim_time:=true
    RC=$?
    set -e

    echo "=== Physical mobile-manipulator trace ==="
    grep -E "PROPOSE action=(navigate|pick)|CONTEXT .*action=(navigate|pick)|ALLOW action=(navigate|pick)|NAV2_SEND|GAZEBO_PICK_APPROACH|GAZEBO_CONTACT_OBJECT|HomeAgentContactGrasp.*ATTACHED|SKILL_RESULT .*action=(navigate|pick)" \
      /tmp/homeagent_physical_mobile.log | tail -180 || true
    echo "=== Integrated stack log tail ==="
    tail -220 /tmp/homeagent_physical_mobile.log
    exit "$RC"
  '
