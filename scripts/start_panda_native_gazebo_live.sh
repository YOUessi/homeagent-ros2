#!/usr/bin/env bash
# Persistent Panda + HomeBot Gazebo Classic and native gzclient/noVNC.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
TS_IP="$(tailscale ip -4 | head -1)"
test -n "$TS_IP"
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
  cd /workspace/ros2_ws
  colcon --log-base /tmp/panda_live_colcon/log build --packages-select homeagent_manipulation homeagent_description homeagent_gazebo_plugins --build-base /tmp/panda_live_colcon/build --install-base /tmp/panda_live_colcon/install --symlink-install > /workspace/artifacts/panda_live/build.log 2>&1
  source /tmp/panda_live_colcon/install/setup.bash
  export GAZEBO_PLUGIN_PATH="/tmp/panda_live_colcon/install/homeagent_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"
  python3 /workspace/scripts/generate_panda_physics_urdf.py --output /workspace/artifacts/panda_mobile/homebot_panda.urdf --controllers /workspace/ros2_ws/src/homeagent_manipulation/config/panda_ros2_controllers.yaml --mount homebot
  exec ros2 launch homeagent_manipulation panda_gazebo_physics.launch.py \
    > /workspace/artifacts/panda_live/launch.log 2>&1
'
docker run -d --name homeagent-panda-3d --network host --ipc host \
  -e TAILSCALE_IP="$TS_IP" -v "$ROOT:/workspace" \
  --entrypoint bash homeagent-ros2:panda-visual -lc '
  set -e
  export DISPLAY=:97 GAZEBO_MASTER_URI=http://127.0.0.1:11380
  export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
  export GAZEBO_MODEL_PATH=/root/.gazebo/models
  export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
  export XDG_RUNTIME_DIR=/tmp/runtime-panda
  mkdir -p "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"
  Xvfb :97 -screen 0 1920x1080x24 +extension GLX +render -noreset > /workspace/artifacts/panda_live/xvfb.log 2>&1 &
  sleep 2
  fluxbox > /workspace/artifacts/panda_live/fluxbox.log 2>&1 &
  x11vnc -display :97 -localhost -rfbport 5907 -nopw -forever -shared -noxdamage -quiet > /workspace/artifacts/panda_live/vnc.log 2>&1 &
  websockify --web /usr/share/novnc "$TAILSCALE_IP":6082 127.0.0.1:5907 > /workspace/artifacts/panda_live/websockify.log 2>&1 &
  sleep 3
  gzclient --verbose > /workspace/artifacts/panda_live/gzclient.log 2>&1 &
  wait -n
'
echo "Live native Gazebo: http://$TS_IP:6082/vnc.html?autoconnect=true&resize=remote"
