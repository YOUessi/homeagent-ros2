#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

docker rm -f homeagent-live-sim >/dev/null 2>&1 || true
rm -rf artifacts/live_web
mkdir -p artifacts/live_web

python3 scripts/generate_homebot_arm_gazebo_urdf.py >/dev/null

docker run -d \
  --name homeagent-live-sim \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=66 \
  -e HOMEAGENT_NAV2_PARAMS_FILE="${HOMEAGENT_NAV2_PARAMS_FILE:-/workspace/ros2_ws/src/homeagent_navigation/config/nav2_params_fetch.yaml}" \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_live_colcon
    mkdir -p /tmp/homeagent_live_colcon
    colcon --log-base /tmp/homeagent_live_colcon/log build \
      --build-base /tmp/homeagent_live_colcon/build \
      --install-base /tmp/homeagent_live_colcon/install \
      --symlink-install
    source /tmp/homeagent_live_colcon/install/setup.bash

    export GAZEBO_PLUGIN_PATH="/tmp/homeagent_live_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"

    setsid ros2 launch homeagent_bringup perception_fetch_demo.launch.py \
      nav2_params_file:="$HOMEAGENT_NAV2_PARAMS_FILE" \
      >/workspace/artifacts/live_web/backend.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    READY=0
    for _ in $(seq 1 80); do
      SERVICES="$(ros2 service list 2>/dev/null || true)"
      TOPICS="$(ros2 topic list 2>/dev/null || true)"
      if echo "$SERVICES" | grep -qx "/spawn_entity" \
        && echo "$SERVICES" | grep -qx "/controller_manager/list_controllers" \
        && echo "$TOPICS" | grep -qx "/odom" \
        && echo "$TOPICS" | grep -qx "/scan"; then
        READY=1
        break
      fi
      sleep 0.5
    done
    if [ "$READY" -ne 1 ]; then
      echo "runtime failed to become ready"
      tail -250 /workspace/artifacts/live_web/backend.log
      exit 3
    fi

    python3 /workspace/scripts/live_dashboard_state.py \
      >/workspace/artifacts/live_web/dashboard.log 2>&1 &
    DASH_PID=$!

    echo "=== Initializing AMCL / map->base_footprint ==="
    python3 /workspace/scripts/initialize_localization.py \
      --ros-args -p use_sim_time:=true \
      >/workspace/artifacts/live_web/localization.log 2>&1

    python3 /workspace/scripts/seed_home_memory.py >/tmp/homeagent_live_seed.log

    ros2 run gazebo_ros spawn_entity.py \
      -entity fetch_support \
      -file /workspace/ros2_ws/src/homeagent_manipulation/config/fetch_support.sdf \
      -x 0.90 -y -0.20 -z 0.2900910 \
      >/tmp/homeagent_live_support.log 2>&1

    ros2 run gazebo_ros spawn_entity.py \
      -entity physical_cup \
      -file /workspace/ros2_ws/src/homeagent_manipulation/config/physical_cup.sdf \
      -x 0.90 -y -0.20 -z 0.6201820 \
      >/tmp/homeagent_live_cup.log 2>&1

    touch /workspace/artifacts/live_web/runtime_ready
    echo "HOMEAGENT_LIVE_READY"

    while true; do
      sleep 60
    done
  '
