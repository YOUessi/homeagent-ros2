#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
TAILSCALE_IP="$(tailscale ip -4 | head -1)"
test -n "$TAILSCALE_IP"

mkdir -p artifacts/panda_demo
docker rm -f homeagent-panda-demo >/dev/null 2>&1 || true

docker run -d --name homeagent-panda-demo \
  --entrypoint bash --network host --ipc host \
  -e TAILSCALE_IP="$TAILSCALE_IP" -e ROS_DOMAIN_ID=78 \
  -v "$ROOT:/workspace" \
  homeagent-ros2:panda-visual -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    export DISPLAY=:98
    export LIBGL_ALWAYS_SOFTWARE=1
    export QT_X11_NO_MITSHM=1
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 700 "$XDG_RUNTIME_DIR"
    Xvfb :98 -screen 0 1440x900x24 +extension GLX +render -noreset \
      >/workspace/artifacts/panda_demo/xvfb.log 2>&1 &
    sleep 2
    DISPLAY=:98 xdpyinfo >/workspace/artifacts/panda_demo/display.log

    fluxbox >/workspace/artifacts/panda_demo/window_manager.log 2>&1 &
    x11vnc -display :98 -localhost -rfbport 5906 -nopw \
      -forever -shared -noxdamage -repeat -quiet \
      >/workspace/artifacts/panda_demo/vnc.log 2>&1 &
    websockify --web /usr/share/novnc \
      "$TAILSCALE_IP":6081 127.0.0.1:5906 \
      >/workspace/artifacts/panda_demo/websockify.log 2>&1 &

    ros2 launch moveit_resources_panda_moveit_config demo.launch.py \
      >/workspace/artifacts/panda_demo/moveit_demo.log 2>&1 &
    LAUNCH_PID=$!

    echo "PANDA_MOVEIT_3D_URL=http://$TAILSCALE_IP:6081/vnc.html?autoconnect=true"
    wait "$LAUNCH_PID"
  '
echo "Panda 7-DoF MoveIt 3D: http://$TAILSCALE_IP:6081/vnc.html?autoconnect=true"
