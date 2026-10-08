#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$ROOT/ros2_ws"

colcon build --symlink-install --packages-skip homeagent_gazebo_plugins
source install/setup.bash
PYTHONNOUSERSITE=1 python3 -m pytest -q \
  src/homeagent_safety/test/test_policy.py \
  src/homeagent_memory/test/test_store.py \
  src/homeagent_orchestrator/test/test_planner_core.py \
  src/homeagent_orchestrator/test/test_task_runtime.py \
  src/homeagent_context/test/test_resolver.py \
  src/homeagent_skills/test/test_nav2_target.py \
  src/homeagent_skills/test/test_arm_targets.py \
  src/homeagent_perception/test/test_observation.py
