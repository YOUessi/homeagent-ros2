#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

mkdir -p artifacts
LOG="artifacts/gazebo_contact_grasp_demo.log"
REPORT="artifacts/gazebo_contact_grasp_report.json"
rm -f "$LOG" "$REPORT"

echo "================================================================"
echo " HomeAgent Gazebo Contact-Gated Grasp Demo"
echo " 双指接触 -> close threshold -> Gazebo fixed constraint -> carry"
echo "================================================================"
echo
echo "说明："
echo "  - 不使用 MoveIt PlanningScene attach"
echo "  - 必须检测到两个夹爪手指都与杯子发生真实 Gazebo contact"
echo "  - 接触成立后创建 Gazebo fixed joint 作为仿真抓持约束"
echo "  - 这不是纯摩擦维持抓取，也不是自主桌面抓取"
echo

set +e
./scripts/docker_gazebo_physical_grasp_smoke.sh >"$LOG" 2>&1
RC=$?
set -e

if [[ ! -s "$REPORT" ]]; then
  echo "Demo failed before report generation. Full log: $LOG"
  tail -160 "$LOG"
  exit "$RC"
fi

python3 - "$REPORT" <<'PY'
import json, sys
r=json.load(open(sys.argv[1],encoding="utf-8"))
print("验收结果：")
print(f"  backend                  : {r['backend']}")
print(f"  contact required         : {r['contact_required']}")
print(f"  PlanningScene attach     : {r['planning_scene_attach_used']}")
print(f"  physics constraint attach: {r['physics_constraint_attach_used']}")
print(f"  friction-only grasp      : {r['friction_only_grasp']}")
print(f"  gravity enabled          : {r['gravity_enabled']}")
print(f"  cup world motion         : {r['cup_world_motion_m']:.4f} m")
print(f"  cup/tool relative drift  : {r['cup_relative_tool_drift_m']:.6f} m")
print(f"  contact-gated hold       : {r['contact_gated_physical_hold']}")
print(f"  autonomous table pick    : {r['autonomous_table_pick']}")
print(f"  DEMO PASS                : {r['passed']}")
PY

echo
echo "关键 Gazebo 事件："
grep -E "HomeAgentContactGrasp.*(ready|ATTACHED|DETACHED)" "$LOG" | tail -20 || true

echo
echo "完整日志 : $LOG"
echo "机器报告 : $REPORT"

test "$RC" -eq 0
