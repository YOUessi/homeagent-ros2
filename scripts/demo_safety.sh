#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

SAFETY_BIN="$ROOT/ros2_ws/install/homeagent_safety/lib/homeagent_safety/safety_node"
PLANNER_BIN="$ROOT/ros2_ws/install/homeagent_orchestrator/lib/homeagent_orchestrator/mock_planner"
SAFETY_LOG="/tmp/homeagent_safety.log"
PLANNER_LOG="/tmp/homeagent_planner.log"

"$SAFETY_BIN" >"$SAFETY_LOG" 2>&1 &
SAFETY_PID=$!
"$PLANNER_BIN" >"$PLANNER_LOG" 2>&1 &
PLANNER_PID=$!

cleanup() {
  kill "$SAFETY_PID" "$PLANNER_PID" 2>/dev/null || true
  wait "$SAFETY_PID" "$PLANNER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2

echo "[1/2] Safe command: 去客厅"
timeout 6 ros2 topic echo --once /homeagent/action_approved homeagent_interfaces/msg/SafetyDecision &
ECHO_PID=$!
sleep 1
ros2 topic pub --once /homeagent/user_command std_msgs/msg/String "{data: '去客厅'}" >/dev/null
wait "$ECHO_PID" || true

echo
echo "[2/2] Unsafe command: 把刀给小孩"
timeout 6 ros2 topic echo --once /homeagent/action_rejected homeagent_interfaces/msg/SafetyDecision &
ECHO_PID=$!
sleep 1
ros2 topic pub --once /homeagent/user_command std_msgs/msg/String "{data: '把刀给小孩'}" >/dev/null
wait "$ECHO_PID" || true

echo
echo "Safety engine log:"
tail -20 "$SAFETY_LOG"
