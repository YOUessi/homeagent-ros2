#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

mkdir -p artifacts
LOG="artifacts/mobile_manipulator_demo.log"
REPORT="artifacts/mobile_manipulator_report.json"

rm -f "$LOG" "$REPORT"

echo "================================================================"
echo " HomeAgent Mobile Manipulator Demo"
echo " 去客厅 -> Nav2 真移动 -> 拿水杯 -> MoveIt2 -> Gripper -> Attach"
echo "================================================================"
echo
echo "[1/6] 启动 HomeBot Gazebo + Nav2 / AMCL"
echo "[2/6] 启动 HomeArm MoveIt2 + ros2_control + gripper"
echo "[3/6] 启动 HomeAgent Context / Safety / Nav2 + MoveIt adapters"
echo "[4/6] 执行：去客厅"
echo "[5/6] 执行：拿起水杯（逻辑抓取，不冒充物理抓取）"
echo

set +e
./scripts/docker_mobile_manipulator_demo.sh >"$LOG" 2>&1
RC=$?
set -e

if [[ ! -s "$REPORT" ]]; then
  echo "Demo failed before report generation. Full log: $LOG"
  tail -160 "$LOG"
  exit "$RC"
fi

echo "[6/6] 验收结果"
python3 - "$REPORT" <<'PY'
import json, sys
r = json.load(open(sys.argv[1], encoding="utf-8"))
n = r["navigation"]
p = r["pick"]
m = r["mount_tf"]

print(f"  arm mount TF       : base_link -> arm_base_footprint z={m['z']:.2f} m")
print(f"  Nav2 result        : {n['code']}")
print(f"  Nav target source  : {n['target_source']}")
print(f"  Base displacement  : {n['displacement_m']:.4f} m")
print(f"  Pick result        : {p['code']}")
print(f"  Pick target source : {p['target_source']}")
print(f"  Arm motion L2      : {p['arm_motion_l2_rad']:.4f} rad")
print(f"  Arm max error      : {p['max_joint_error_rad']:.5f} rad")
print(f"  Gripper            : {p['initial_gripper_m']:.3f} -> {p['final_gripper_m']:.3f} m")
print(f"  Attached objects   : {p['attached_object_ids']}")
print(f"  Physical grasp     : {p['physical_grasp']}")
print(f"  DEMO PASS          : {r['passed']}")
PY

echo
echo "关键 HomeAgent trace:"
grep -E "PROPOSE action=(navigate|pick)|CONTEXT .*action=(navigate|pick)|ALLOW action=(navigate|pick)|NAV2_SEND|MOVEIT_SEND|GRIPPER_CLOSE|PLANNING_SCENE_ATTACH|SKILL_RESULT .*action=(navigate|pick)" "$LOG" \
  | awk '!seen[$0]++' || true

echo
echo "完整日志 : $LOG"
echo "机器报告 : $REPORT"

test "$RC" -eq 0
