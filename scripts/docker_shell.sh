#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
docker run --rm -it \
  --network host \
  --ipc host \
  -e ROS_DOMAIN_ID=42 \
  -e TURTLEBOT3_MODEL=waffle \
  -v "$ROOT:/workspace" \
  homeagent-ros2:humble bash
