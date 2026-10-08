#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TAILSCALE_IP="$(tailscale ip -4 | head -1)"
if [[ -z "$TAILSCALE_IP" ]]; then
  echo "Tailscale IP is unavailable; not exposing a desktop port." >&2
  exit 2
fi

mkdir -p artifacts/3d_view
docker rm -f homeagent-3d-view >/dev/null 2>&1 || true

docker run -d --name homeagent-3d-view \
  --network host --ipc host \
  --entrypoint bash \
  -e HOME=/root \
  -e TAILSCALE_IP="$TAILSCALE_IP" \
  -v "$ROOT:/workspace" \
  homeagent-ros2:visual -lc '
    set -eo pipefail
    export DISPLAY=:99
    export GAZEBO_MASTER_URI=http://127.0.0.1:11345
    export LIBGL_ALWAYS_SOFTWARE=1
    export QT_X11_NO_MITSHM=1
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 700 "$XDG_RUNTIME_DIR"

    Xvfb :99 -screen 0 1440x900x24 +extension GLX +render -noreset \
      >/workspace/artifacts/3d_view/xvfb.log 2>&1 &
    XVFB_PID=$!
    sleep 2
    DISPLAY=:99 xdpyinfo >/workspace/artifacts/3d_view/display.log

    fluxbox >/workspace/artifacts/3d_view/window_manager.log 2>&1 &
    x11vnc -display :99 -localhost -rfbport 5905 -nopw \
      -forever -shared -noxdamage -repeat -quiet \
      >/workspace/artifacts/3d_view/vnc.log 2>&1 &
    VNC_PID=$!

    websockify --web /usr/share/novnc \
      "$TAILSCALE_IP":6080 127.0.0.1:5905 \
      >/workspace/artifacts/3d_view/websockify.log 2>&1 &
    WEB_PID=$!

    echo "Starting native Gazebo 3D client..."
    gzclient --verbose \
      >/workspace/artifacts/3d_view/gzclient.log 2>&1 &
    GZCLIENT_PID=$!

    echo "GZCLIENT_PID=$GZCLIENT_PID"
    echo "NOVNC_URL=http://$TAILSCALE_IP:6080/vnc.html?autoconnect=true"
    while kill -0 "$XVFB_PID" 2>/dev/null \
      && kill -0 "$VNC_PID" 2>/dev/null \
      && kill -0 "$WEB_PID" 2>/dev/null; do
      sleep 5
    done
    echo "One of the display processes exited."
    exit 1
  '

echo "3D viewer: http://$TAILSCALE_IP:6080/vnc.html?autoconnect=true"
