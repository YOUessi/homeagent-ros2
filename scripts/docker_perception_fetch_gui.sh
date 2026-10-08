#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 scripts/generate_homebot_arm_gazebo_urdf.py >/dev/null

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=65 \
  -e DISPLAY=:0 \
  -e XAUTHORITY=/tmp/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e HOMEAGENT_PERCEPTION_FETCH_REPORT=/workspace/artifacts/perception_fetch_gui_report.json \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /run/user/1000/gdm/Xauthority:/tmp/.Xauthority:ro \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_perception_fetch_gui_colcon
    mkdir -p /tmp/homeagent_perception_fetch_gui_colcon
    colcon --log-base /tmp/homeagent_perception_fetch_gui_colcon/log build \
      --build-base /tmp/homeagent_perception_fetch_gui_colcon/build \
      --install-base /tmp/homeagent_perception_fetch_gui_colcon/install \
      --symlink-install
    source /tmp/homeagent_perception_fetch_gui_colcon/install/setup.bash

    export GAZEBO_PLUGIN_PATH="/tmp/homeagent_perception_fetch_gui_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"

    setsid timeout 180 ros2 launch homeagent_bringup \
      perception_fetch_demo.launch.py gui:=true \
      >/tmp/homeagent_perception_fetch_gui.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
      sleep 0.5
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 18

    python3 /workspace/scripts/seed_home_memory.py \
      >/tmp/homeagent_perception_fetch_gui_seed.log

    ros2 run gazebo_ros spawn_entity.py \
      -entity fetch_support \
      -file /workspace/ros2_ws/src/homeagent_manipulation/config/fetch_support.sdf \
      -x 0.90 -y -0.20 -z 0.285718475 \
      >/tmp/homeagent_fetch_support_spawn_gui.log 2>&1

    ros2 run gazebo_ros spawn_entity.py \
      -entity physical_cup \
      -file /workspace/ros2_ws/src/homeagent_manipulation/config/physical_cup.sdf \
      -x 0.90 -y -0.20 -z 0.61143695 \
      >/tmp/homeagent_fetch_cup_spawn_gui.log 2>&1

    sleep 3

    set +e
    python3 /workspace/scripts/perception_fetch_e2e.py \
      --ros-args -p use_sim_time:=true
    RC=$?
    set -e

    echo "=== Visual perception fetch trace ==="
    grep -E "PERCEPTION_OBSERVE|TASK_START|TASK_STEP|NAV2_SEND|LOCAL_ALIGN|GAZEBO_PICK_PREGRASP|GAZEBO_PICK_APPROACH|HomeAgentContactGrasp.*ATTACHED|SKILL_RESULT .*action=(navigate|pick)" \
      /tmp/homeagent_perception_fetch_gui.log | tail -240 || true
    exit "$RC"
  '
