#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

mkdir -p artifacts
LOG="artifacts/physical_mobile_manipulator_demo.log"
REPORT="artifacts/physical_mobile_manipulator_report.json"
rm -f "$LOG" "$REPORT"

echo "================================================================"
echo " HomeAgent Physical Mobile Manipulator Demo"
echo " 去客厅 -> Nav2 -> GazeboSystem HomeArm -> bilateral contact -> carry"
echo "================================================================"

set +e
./scripts/docker_physical_mobile_manipulator_demo.sh >"$LOG" 2>&1
RC=$?
set -e

if [[ ! -s "$REPORT" ]]; then
  echo "Demo failed before report generation: $LOG"
  tail -180 "$LOG"
  exit "$RC"
fi

python3 - "$REPORT" <<'PY'
import json, sys
r=json.load(open(sys.argv[1],encoding="utf-8"))
n=r["navigation"]; p=r["pick"]
print("验收结果：")
print(f"  Nav2 result           : {n['code']}")
print(f"  Nav target source     : {n['target_source']}")
print(f"  Base displacement     : {n['displacement_m']:.4f} m")
print(f"  Pick result           : {p['code']}")
print(f"  Pick target source    : {p['target_source']}")
print(f"  PlanningScene attach  : {p['planning_scene_attach_used']}")
print(f"  Physics constraint    : {p['physics_constraint_attach_used']}")
print(f"  Bilateral contact req : {p['contact_required']}")
print(f"  Friction-only         : {p['friction_only_grasp']}")
print(f"  Cup world motion      : {p['cup_world_motion_m']:.4f} m")
print(f"  Cup/tool drift        : {p['cup_relative_tool_drift_m']:.6f} m")
print(f"  Autonomous table pick : {p['autonomous_table_pick']}")
print(f"  DEMO PASS             : {r['passed']}")
PY

echo
echo "关键 trace："
grep -E "PROPOSE action=(navigate|pick)|CONTEXT .*action=(navigate|pick)|ALLOW action=(navigate|pick)|NAV2_SEND|GAZEBO_PICK_APPROACH|GAZEBO_CONTACT_OBJECT|HomeAgentContactGrasp.*ATTACHED|SKILL_RESULT .*action=(navigate|pick)" "$LOG" | tail -100 || true

echo
echo "完整日志 : $LOG"
echo "机器报告 : $REPORT"
test "$RC" -eq 0
