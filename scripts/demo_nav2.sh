#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

mkdir -p artifacts
LOG="artifacts/nav2_demo_full.log"
REPORT="artifacts/nav2_demo_report.json"
IMAGE="artifacts/nav2_demo_path.png"

rm -f "$LOG" "$REPORT" "$IMAGE"

echo "============================================================"
echo " HomeAgent-ROS2 Demo: 自然语言 -> Safety -> Nav2 -> 机器人移动"
echo "============================================================"
echo
echo "[1/5] 启动 Gazebo HomeBot + Map Server + AMCL + Nav2..."
echo "[2/5] 启动 HomeAgent Planner / Context / Safety / Nav2 Skill..."
echo "[3/5] 输入用户指令：去客厅"
echo

set +e
./scripts/docker_nav2_smoke.sh >"$LOG" 2>&1
RC=$?
set -e

if [[ ! -s "$REPORT" ]]; then
  echo "Demo failed before report generation. Full log: $LOG"
  tail -120 "$LOG"
  exit "$RC"
fi

python3 scripts/render_nav2_demo.py \
  ros2_ws/src/homeagent_navigation/maps/home_room.pgm \
  ros2_ws/src/homeagent_navigation/maps/home_room.yaml \
  "$REPORT" \
  "$IMAGE"

echo
echo "[4/5] Agent / Safety / Nav2 关键链路："
grep -E "PROPOSE action=navigate|CONTEXT .*action=navigate|ALLOW action=navigate|NAV2_SEND|SKILL_RESULT .*action=navigate" "$LOG" || true

echo
echo "[5/5] 真实运动验收："
python3 - "$REPORT" <<'PY'
import json, sys
r = json.load(open(sys.argv[1], encoding="utf-8"))
print(f"  target        : {r['skill_result']['target']}")
print(f"  goal_xy       : {r['skill_result']['goal_xyyaw'][:2]}")
print(f"  initial_xy    : {[round(v, 4) for v in r['initial_xy']]}")
print(f"  final_xy      : {[round(v, 4) for v in r['final_xy']]}")
print(f"  displacement  : {r['displacement_m']:.4f} m")
print(f"  trajectory pts: {r['trajectory_samples']}")
print(f"  skill result  : {r['skill_code']}")
print(f"  DEMO PASS     : {r['passed']}")
PY

echo
echo "可视化轨迹: $IMAGE"
echo "完整日志    : $LOG"
echo "机器结果    : $REPORT"

test "$RC" -eq 0
