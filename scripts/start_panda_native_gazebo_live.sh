#!/usr/bin/env bash
# Persistent Panda + HomeBot Gazebo Classic and native gzclient/noVNC.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
TS_IP="$(tailscale ip -4 | head -1)"
test -n "$TS_IP"
test -f artifacts/panda_mobile/homebot_panda.urdf
mkdir -p artifacts/panda_live
docker image inspect homeagent-ros2:panda-visual >/dev/null
docker rm -f homeagent-panda-live homeagent-panda-3d >/dev/null 2>&1 || true
docker run -d --name homeagent-panda-live --network host --ipc host \
  -e ROS_DOMAIN_ID=80 -e GAZEBO_MASTER_URI=http://127.0.0.1:11380 \
  -e HOMEAGENT_PANDA_MOUNT=homebot \
  -e HOMEAGENT_PANDA_GAZEBO_URDF=/workspace/artifacts/panda_mobile/homebot_panda.urdf \
  -v "$ROOT:/workspace" --entrypoint bash homeagent-ros2:panda-visual -lc '
  set -e
  source /opt/ros/humble/setup.bash
  source /tmp/ha_panda_mobile/install/setup.bash
  exec ros2 launch homeagent_manipulation panda_gazebo_physics.launch.py \
    > /workspace/artifacts/panda_live/launch.log 2>&1
'
docker run -d --name homeagent-panda-3d --network host --ipc host \
  -e TAILSCALE_IP="$TS_IP" -v "$ROOT:/workspace" \
  --entrypoint bash homeagent-ros2:panda-visual -lc '
  set -e
  export DISPLAY=:98 GAZEBO_MASTER_URI=http://127.0.0.1:11380
  export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
  export XDG_RUNTIME_DIR=/tmp/runtime-panda
  mkdir -p "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"
  Xvfb :98 -screen 0 1920x1080x24 +extension GLX +render -noreset > /workspace/artifacts/panda_live/xvfb.log 2>&1 &
  sleep 2
  fluxbox > /workspace/artifacts/panda_live/fluxbox.log 2>&1 &
  x11vnc -display :98 -localhost -rfbport 5906 -nopw -forever -shared -noxdamage -quiet > /workspace/artifacts/panda_live/vnc.log 2>&1 &
  websockify --web /usr/share/novnc "$TAILSCALE_IP":6081 127.0.0.1:5906 > /workspace/artifacts/panda_live/websockify.log 2>&1 &
  sleep 3
  gzclient --verbose > /workspace/artifacts/panda_live/gzclient.log 2>&1 &
  wait -n
'
echo "Live native Gazebo: http://$TS_IP:6081/vnc.html?autoconnect=true&resize=remote"
