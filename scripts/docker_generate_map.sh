#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

MAP_BASE="/workspace/ros2_ws/src/homeagent_navigation/maps/home_room"

docker run --rm \
  --entrypoint bash \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=46 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble \
  -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    rm -rf /tmp/homeagent_colcon
    mkdir -p /tmp/homeagent_colcon
    colcon --log-base /tmp/homeagent_colcon/log build \
      --build-base /tmp/homeagent_colcon/build \
      --install-base /tmp/homeagent_colcon/install \
      --symlink-install
    source /tmp/homeagent_colcon/install/setup.bash

    rm -f /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.pgm \
          /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.yaml

    timeout 55 ros2 launch homeagent_navigation homebot_slam.launch.py \
      >/tmp/homeagent_generate_map.log 2>&1 &
    LAUNCH_PID=$!

    cleanup() {
      kill "$LAUNCH_PID" 2>/dev/null || true
      wait "$LAUNCH_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    sleep 9
    python3 /workspace/scripts/slam_smoke.py

    echo "=== Saving occupancy map ==="
    ros2 run nav2_map_server map_saver_cli \
      -f /workspace/ros2_ws/src/homeagent_navigation/maps/home_room \
      --occ 0.65 --free 0.25

    test -s /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.pgm
    test -s /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.yaml
    chmod a+rw \
      /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.pgm \
      /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.yaml

    echo "=== Map files ==="
    ls -lh /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.*
    echo "=== Map YAML ==="
    cat /workspace/ros2_ws/src/homeagent_navigation/maps/home_room.yaml
    echo "=== Mapping log tail ==="
    tail -60 /tmp/homeagent_generate_map.log
  '
