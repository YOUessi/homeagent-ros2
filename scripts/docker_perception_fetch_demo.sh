#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 scripts/generate_homebot_arm_gazebo_urdf.py >/dev/null

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=63 \
  -e HOMEAGENT_PERCEPTION_FETCH_REPORT=/workspace/artifacts/perception_fetch_report.json \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_perception_fetch_colcon
    mkdir -p /tmp/homeagent_perception_fetch_colcon
    colcon --log-base /tmp/homeagent_perception_fetch_colcon/log build \
      --build-base /tmp/homeagent_perception_fetch_colcon/build \
      --install-base /tmp/homeagent_perception_fetch_colcon/install \
      --symlink-install
    source /tmp/homeagent_perception_fetch_colcon/install/setup.bash

    export GAZEBO_PLUGIN_PATH="/tmp/homeagent_perception_fetch_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"

    setsid timeout 155 ros2 launch homeagent_bringup \
      perception_fetch_demo.launch.py \
      >/tmp/homeagent_perception_fetch.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    echo "=== Waiting for Gazebo/HomeArm runtime ==="
    READY=0
    for _ in $(seq 1 60); do
      SERVICES="$(ros2 service list 2>/dev/null || true)"
      TOPICS="$(ros2 topic list 2>/dev/null || true)"
      if echo "$SERVICES" | grep -qx "/spawn_entity" \
        && echo "$SERVICES" | grep -qx "/controller_manager/list_controllers" \
        && echo "$TOPICS" | grep -qx "/odom"; then
        READY=1
        break
      fi
      sleep 0.5
    done

    if [ "$READY" -ne 1 ]; then
      echo "ERROR: Gazebo/HomeArm runtime did not become ready"
      tail -220 /tmp/homeagent_perception_fetch.log
      exit 3
    fi

    echo "RUNTIME_READY"
    python3 /workspace/scripts/live_sim_visualizer.py \
      >/tmp/homeagent_live_sim_visualizer.log 2>&1 &
    VIZ_PID=$!

    python3 /workspace/scripts/seed_home_memory.py \
      >/tmp/homeagent_perception_fetch_seed.log

    ros2 run gazebo_ros spawn_entity.py \
      -entity fetch_support \
      -file /workspace/ros2_ws/src/homeagent_manipulation/config/fetch_support.sdf \
      -x 0.90 -y -0.20 -z 0.2900910 \
      >/tmp/homeagent_fetch_support_spawn.log 2>&1

    ros2 run gazebo_ros spawn_entity.py \
      -entity physical_cup \
      -file /workspace/ros2_ws/src/homeagent_manipulation/config/physical_cup.sdf \
      -x 0.90 -y -0.20 -z 0.6201820 \
      >/tmp/homeagent_fetch_cup_spawn.log 2>&1

    sleep 2

    set +e
    python3 /workspace/scripts/perception_fetch_e2e.py \
      --ros-args -p use_sim_time:=true
    RC=$?
    set -e

    echo "=== Perception fetch trace ==="
    grep -E "PERCEPTION_OBSERVE|TASK_START|TASK_STEP|PROPOSE action=(navigate|pick)|CONTEXT .*action=(navigate|pick)|ALLOW action=(navigate|pick)|NAV2_SEND|GAZEBO_EXISTING_TARGET|GAZEBO_PICK_APPROACH|GAZEBO_CONTACT_EXISTING_OBJECT|HomeAgentContactGrasp.*ATTACHED|SKILL_RESULT .*action=(navigate|pick)" \
      /tmp/homeagent_perception_fetch.log | tail -220 || true

    echo "=== Cup spawn ==="
    cat /tmp/homeagent_fetch_cup_spawn.log
    echo "=== Integrated log tail ==="
    tail -260 /tmp/homeagent_perception_fetch.log
    exit "$RC"
  '
