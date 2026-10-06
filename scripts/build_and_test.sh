#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$ROOT/ros2_ws"

colcon build --symlink-install
source install/setup.bash
PYTHONNOUSERSITE=1 python3 -m pytest -q src/homeagent_safety/test/test_policy.py
