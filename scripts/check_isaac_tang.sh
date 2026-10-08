#!/usr/bin/env bash
set -euo pipefail
echo "=== Isaac Sim readiness / read-only ==="
date -Is
uname -m
grep PRETTY_NAME /etc/os-release
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
free -h
df -h "$HOME"
echo "=== existing simulator launchers ==="
for root in "$HOME/isaacsim" /opt/isaac-sim "$HOME/.local/share/ov/pkg"; do
  if [ -e "$root" ]; then ls -ld "$root"; fi
done
echo "No installation or driver changes performed."
