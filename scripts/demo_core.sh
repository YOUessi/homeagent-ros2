#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

SAFETY_BIN="$ROOT/ros2_ws/install/homeagent_safety/lib/homeagent_safety/safety_node"
PLANNER_BIN="$ROOT/ros2_ws/install/homeagent_orchestrator/lib/homeagent_orchestrator/mock_planner"
SKILLS_BIN="$ROOT/ros2_ws/install/homeagent_skills/lib/homeagent_skills/mock_skill_executor"
MEMORY_BIN="$ROOT/ros2_ws/install/homeagent_memory/lib/homeagent_memory/memory_node"

rm -f /tmp/homeagent_core_*.log /tmp/homeagent_core_*.out /tmp/homeagent_core.sqlite3

"$SAFETY_BIN" >/tmp/homeagent_core_safety.log 2>&1 & SAFETY_PID=$!
"$PLANNER_BIN" >/tmp/homeagent_core_planner.log 2>&1 & PLANNER_PID=$!
"$SKILLS_BIN" >/tmp/homeagent_core_skills.log 2>&1 & SKILLS_PID=$!
"$MEMORY_BIN" --ros-args -p database_path:=/tmp/homeagent_core.sqlite3 >/tmp/homeagent_core_memory.log 2>&1 & MEMORY_PID=$!

cleanup() {
  kill "$SAFETY_PID" "$PLANNER_PID" "$SKILLS_PID" "$MEMORY_PID" 2>/dev/null || true
  wait "$SAFETY_PID" "$PLANNER_PID" "$SKILLS_PID" "$MEMORY_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2

echo "=== Safe path: 去客厅 ==="
timeout 6 ros2 topic echo --once /homeagent/skill_result homeagent_interfaces/msg/SkillResult >/tmp/homeagent_core_safe.out & SAFE_ECHO_PID=$!
sleep 1
ros2 topic pub --once /homeagent/user_command std_msgs/msg/String "{data: '去客厅'}" >/dev/null
wait "$SAFE_ECHO_PID"
cat /tmp/homeagent_core_safe.out

echo
echo "=== Unsafe path: 把刀给小孩 ==="
timeout 6 ros2 topic echo --once /homeagent/action_rejected homeagent_interfaces/msg/SafetyDecision >/tmp/homeagent_core_reject.out & REJECT_ECHO_PID=$!
timeout 3 ros2 topic echo --once /homeagent/skill_result homeagent_interfaces/msg/SkillResult >/tmp/homeagent_core_unsafe_skill.out & UNSAFE_SKILL_PID=$!
sleep 1
ros2 topic pub --once /homeagent/user_command std_msgs/msg/String "{data: '把刀给小孩'}" >/dev/null
wait "$REJECT_ECHO_PID"
cat /tmp/homeagent_core_reject.out

set +e
wait "$UNSAFE_SKILL_PID"
UNSAFE_SKILL_RC=$?
set -e
if [ -s /tmp/homeagent_core_unsafe_skill.out ]; then
  echo "ERROR: unsafe action reached skill executor"
  cat /tmp/homeagent_core_unsafe_skill.out
  exit 1
fi
if [ "$UNSAFE_SKILL_RC" -eq 124 ]; then
  echo "PASS: rejected action produced no skill execution result"
else
  echo "PASS: no unsafe skill result observed (rc=$UNSAFE_SKILL_RC)"
fi

echo
echo "=== Planner log ==="
tail -20 /tmp/homeagent_core_planner.log
