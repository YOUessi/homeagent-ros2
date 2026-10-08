#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p artifacts/panda_agent_nav

docker run --rm \
  --entrypoint bash --network host --ipc host \
  -e ROS_DOMAIN_ID=81 \
  -e GAZEBO_MASTER_URI=http://127.0.0.1:11381 \
  -e HOMEAGENT_PANDA_MOUNT=homebot \
  -e PANDA_GAZEBO_ENTITY=homebot_panda \
  -e HOMEAGENT_PANDA_GAZEBO_URDF=/workspace/artifacts/panda_agent_nav/homebot_panda.urdf \
  -e HOMEAGENT_PANDA_AGENT_REPORT=/workspace/artifacts/panda_agent_nav/e2e_report.json \
  -v "$ROOT:/workspace" \
  homeagent-ros2:panda-visual -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2_ws

    mkdir -p /tmp/homeagent_panda_agent_nav_colcon
    colcon --log-base /tmp/homeagent_panda_agent_nav_colcon/log build \
      --build-base /tmp/homeagent_panda_agent_nav_colcon/build \
      --install-base /tmp/homeagent_panda_agent_nav_colcon/install \
      --symlink-install --packages-skip homeagent_gazebo_plugins
    source /tmp/homeagent_panda_agent_nav_colcon/install/setup.bash

    python3 /workspace/scripts/generate_panda_physics_urdf.py \
      --output "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      --controllers /workspace/ros2_ws/src/homeagent_manipulation/config/panda_ros2_controllers.yaml \
      --mount homebot
    gz sdf -p "$HOMEAGENT_PANDA_GAZEBO_URDF" \
      >/workspace/artifacts/panda_agent_nav/homebot_panda.sdf

    LAUNCH_PIDS=()
    cleanup() {
      for pid in "${LAUNCH_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
      done
      sleep 0.5
      for pid in "${LAUNCH_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
      done
    }
    trap cleanup EXIT INT TERM

    echo "=== Stage 1: Gazebo physical Panda + HomeBot + MoveIt ==="
    setsid timeout 185 ros2 launch homeagent_manipulation \
      panda_gazebo_physics.launch.py \
      >/workspace/artifacts/panda_agent_nav/physics.log 2>&1 &
    PHYS_PID=$!
    LAUNCH_PIDS+=("$PHYS_PID")

    PHYS_READY=0
    for i in $(seq 1 45); do
      TOPICS="$(ros2 topic list 2>/dev/null || true)"
      SERVICES="$(ros2 service list 2>/dev/null || true)"
      ACTIONS="$(ros2 action list 2>/dev/null || true)"
      if echo "$TOPICS" | grep -qx "/odom" \
        && echo "$TOPICS" | grep -qx "/scan" \
        && echo "$SERVICES" | grep -qx "/controller_manager/list_controllers" \
        && echo "$ACTIONS" | grep -qx "/move_action"; then
        PHYS_READY=1
        break
      fi
      sleep 1
    done
    if [ "$PHYS_READY" -ne 1 ]; then
      echo "ERROR: Physical Panda/HomeBot never became ready"
      tail -150 /workspace/artifacts/panda_agent_nav/physics.log
      exit 3
    fi
    echo "PHYSICS_READY"
    sleep 2

    echo "=== Stage 2: Nav2 lifecycle after Gazebo TF is available ==="
    setsid timeout 160 ros2 launch homeagent_navigation \
      homebot_nav2_runtime.launch.py \
      params_file:=/workspace/ros2_ws/src/homeagent_navigation/config/nav2_params_panda.yaml \
      >/workspace/artifacts/panda_agent_nav/nav2.log 2>&1 &
    NAV_PID=$!
    LAUNCH_PIDS+=("$NAV_PID")

    AMCL_READY=0
    for i in $(seq 1 35); do
      if timeout 3 ros2 lifecycle get /amcl 2>/dev/null | grep -q "active"; then
        AMCL_READY=1
        break
      fi
      sleep 1
    done
    if [ "$AMCL_READY" -ne 1 ]; then
      echo "ERROR: AMCL never reached active lifecycle state"
      tail -150 /workspace/artifacts/panda_agent_nav/nav2.log
      exit 4
    fi
    echo "AMCL_ACTIVE"

    echo "=== Stage 3: Trusted Agent + Memory + Safety + Skills ==="
    setsid timeout 155 ros2 launch homeagent_bringup \
      homeagent_core.launch.py \
      use_nav2:=true nav_postcondition_tolerance_m:=0.15 \
      nav_require_panda_stow:=true \
      use_panda_moveit:=true use_moveit:=false \
      use_gazebo_contact_pick:=false use_deepseek:=false \
      use_sim_time:=true \
      memory_db:=/tmp/homeagent_panda_agent_nav_memory.sqlite3 \
      >/workspace/artifacts/panda_agent_nav/core.log 2>&1 &
    CORE_PID=$!
    LAUNCH_PIDS+=("$CORE_PID")
    sleep 3

    python3 /workspace/scripts/seed_home_memory.py \
      >/workspace/artifacts/panda_agent_nav/seed_memory.log

    echo "=== Stage 4: True ROS2 E2E test + independent telemetry ==="
    python3 /workspace/scripts/nav2_telemetry.py \
      --output /workspace/artifacts/panda_agent_nav/nav2_telemetry.jsonl \
      --ros-args -p use_sim_time:=true \
      >/workspace/artifacts/panda_agent_nav/nav2_telemetry.log 2>&1 &
    MONITOR_PID=$!
    sleep 1
    set +e
    python3 /workspace/scripts/panda_agent_nav_e2e.py --ros-args \
      -p use_sim_time:=true \
      > /workspace/artifacts/panda_agent_nav/e2e_probe.log 2>&1
    RC=$?
    set -e
    # Background Python ROS executors may ignore SIGINT inherited from bash.
    # TERM avoids hanging the Docker teardown; writes are line-buffered.
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
    timeout 8 ros2 topic info /tf -v \
      > /workspace/artifacts/panda_agent_nav/tf_graph_debug.log 2>&1 || true
    timeout 5 ros2 param get /controller_server use_sim_time \
      > /workspace/artifacts/panda_agent_nav/controller_time_param.log 2>&1 || true

    python3 /workspace/scripts/summarize_nav2_telemetry.py \
      --telemetry /workspace/artifacts/panda_agent_nav/nav2_telemetry.jsonl \
      --nav-log /workspace/artifacts/panda_agent_nav/nav2.log \
      --output /workspace/artifacts/panda_agent_nav/nav2_telemetry_summary.json \
      > /workspace/artifacts/panda_agent_nav/nav2_telemetry_summary.log

    echo "=== PANDA + AGENT + SAFETY + NAV2 E2E ==="
    tail -145 /workspace/artifacts/panda_agent_nav/e2e_probe.log
    echo "=== AGENT TRACE ==="
    grep -Ei "PROPOSE|CONTEXT|APPROVED|PANDA_MOVEIT_SEND|NAV2_SEND|SKILL_RESULT|FORBIDDEN_ZONE|REJECTED" \
      /workspace/artifacts/panda_agent_nav/core.log | tail -90 || true
    echo "=== NAV2 TRACE ==="
    grep -Ei "Failed to make progress|Aborting|Reached the goal|Goal succeeded|Goal failed|Activating amcl|Managed nodes are active" \
      /workspace/artifacts/panda_agent_nav/nav2.log | tail -70 || true

    # Keep the traditional combined log path for journal and diagnostics.
    cat /workspace/artifacts/panda_agent_nav/physics.log \
        /workspace/artifacts/panda_agent_nav/nav2.log \
        /workspace/artifacts/panda_agent_nav/core.log \
        >/workspace/artifacts/panda_agent_nav/launch.log
    exit "$RC"
  '
