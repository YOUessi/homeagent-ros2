#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 scripts/generate_homebot_arm_gazebo_urdf.py >/dev/null

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=60 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_contact_agent_colcon
    mkdir -p /tmp/homeagent_contact_agent_colcon
    colcon --log-base /tmp/homeagent_contact_agent_colcon/log build \
      --build-base /tmp/homeagent_contact_agent_colcon/build \
      --install-base /tmp/homeagent_contact_agent_colcon/install \
      --symlink-install
    source /tmp/homeagent_contact_agent_colcon/install/setup.bash
    export GAZEBO_PLUGIN_PATH="/tmp/homeagent_contact_agent_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"

    timeout 95 ros2 launch homeagent_bringup \
      gazebo_contact_pick_demo.launch.py \
      >/tmp/homeagent_contact_agent.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 12
    python3 /workspace/scripts/seed_home_memory.py >/tmp/homeagent_contact_seed.log

    set +e
    python3 /workspace/scripts/gazebo_contact_agent_e2e.py \
      --ros-args -p use_sim_time:=true
    RC=$?
    set -e

    echo "=== HomeAgent contact-pick trace ==="
    grep -E "PROPOSE action=pick|CONTEXT .*action=pick|ALLOW action=pick|GAZEBO_PICK_APPROACH|GAZEBO_CONTACT_OBJECT|SKILL_RESULT .*action=pick|RESULT .*action=pick|HomeAgentContactGrasp.*ATTACHED" \
      /tmp/homeagent_contact_agent.log | tail -120 || true
    echo "=== Integrated physical stack log tail ==="
    tail -180 /tmp/homeagent_contact_agent.log
    exit "$RC"
  '
