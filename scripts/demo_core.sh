#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

rm -f /tmp/homeagent_demo_core.sqlite3 /tmp/homeagent_demo_core.log \
  /tmp/homeagent_demo_safe.out /tmp/homeagent_demo_reject.out \
  /tmp/homeagent_demo_forbidden_skill.out

ros2 launch homeagent_bringup homeagent_core.launch.py \
  memory_db:=/tmp/homeagent_demo_core.sqlite3 \
  >/tmp/homeagent_demo_core.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 3

echo "=== Safe path: 去客厅 ==="
timeout 6 ros2 topic echo --once \
  /homeagent/skill_result homeagent_interfaces/msg/SkillResult \
  >/tmp/homeagent_demo_safe.out &
SAFE_ECHO_PID=$!
sleep 1
ros2 topic pub --once /homeagent/user_command std_msgs/msg/String \
  "{data: '去客厅'}" >/dev/null
wait "$SAFE_ECHO_PID"
cat /tmp/homeagent_demo_safe.out

echo
echo "=== Rule-engine rejection: forbidden zone ==="
timeout 6 ros2 topic echo --once \
  /homeagent/action_rejected homeagent_interfaces/msg/SafetyDecision \
  >/tmp/homeagent_demo_reject.out &
REJECT_ECHO_PID=$!
timeout 3 ros2 topic echo --once \
  /homeagent/skill_result homeagent_interfaces/msg/SkillResult \
  >/tmp/homeagent_demo_forbidden_skill.out &
FORBIDDEN_SKILL_PID=$!
sleep 1
ros2 topic pub --once /homeagent/action_candidate \
  homeagent_interfaces/msg/ActionProposal \
  "{request_id: demo-forbidden-zone, action: navigate, params_json: '{\"target\":\"utility_room\"}', context_json: '{}', source: demo}" \
  >/dev/null
wait "$REJECT_ECHO_PID"
cat /tmp/homeagent_demo_reject.out

set +e
wait "$FORBIDDEN_SKILL_PID"
FORBIDDEN_SKILL_RC=$?
set -e
if [ -s /tmp/homeagent_demo_forbidden_skill.out ]; then
  echo "ERROR: rejected action reached the skill executor"
  cat /tmp/homeagent_demo_forbidden_skill.out
  exit 1
fi
if [ "$FORBIDDEN_SKILL_RC" -eq 124 ]; then
  echo "PASS: forbidden action produced no skill execution result"
else
  echo "PASS: no forbidden skill result observed (rc=$FORBIDDEN_SKILL_RC)"
fi

echo
echo "=== Context / Safety / Skill trace ==="
grep -E "CONTEXT|ALLOW|REJECT|SKILL_RESULT|RESULT" \
  /tmp/homeagent_demo_core.log | tail -30
