#!/usr/bin/env python3
"""Summarize Nav2 recovery events against independent Gazebo command/odom data."""
import argparse
import json
import math
import re
from pathlib import Path


FAILURE_RE = re.compile(
    r"\[([0-9]{10}\.[0-9]+)\].*Failed to make progress"
)


def movement(rows):
    points = [
        (r["odom"]["x"], r["odom"]["y"])
        for r in rows
        if r.get("odom") is not None
    ]
    if len(points) < 2:
        return None
    start, end = points[0], points[-1]
    return round(math.hypot(end[0]-start[0], end[1]-start[1]), 5)


def mean_abs(rows, key, field):
    values = [abs(row[key][field]) for row in rows if row.get(key)]
    return round(sum(values)/len(values), 5) if values else None


def window_report(rows, at, seconds=15.0):
    prior = [
        row for row in rows
        if at-seconds <= row["timestamp_wall"] <= at
    ]
    if not prior:
        return {"samples": 0}
    odom = [r["odom"] for r in prior if r.get("odom")]
    sim_start = prior[0]["timestamp_sim"]
    sim_end = prior[-1]["timestamp_sim"]
    return {
        "samples": len(prior),
        "wall_duration_sec": round(
            prior[-1]["timestamp_wall"]-prior[0]["timestamp_wall"], 2
        ),
        "sim_duration_sec": round(sim_end-sim_start, 2),
        "odom_displacement_m": movement(prior),
        "mean_abs_cmd_vel_nav_linear_x": mean_abs(
            prior, "/cmd_vel_nav", "vx"
        ),
        "mean_abs_cmd_vel_smoothed_linear_x": mean_abs(
            prior, "/cmd_vel_smoothed", "vx"
        ),
        "mean_abs_cmd_vel_linear_x": mean_abs(
            prior, "/cmd_vel", "vx"
        ),
        "mean_abs_measured_odom_linear_x": mean_abs(prior, "odom", "vx"),
        "latest_odom_xy": (
            [odom[-1]["x"], odom[-1]["y"]] if odom else None
        ),
        "latest_amcl_covariance": next(
            (r["amcl"]["covariance_xy"]
             for r in reversed(prior) if r.get("amcl")),
            None,
        ),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--telemetry", required=True)
    ap.add_argument("--nav-log", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()
    rows = [
        json.loads(line) for line in Path(args.telemetry).read_text().splitlines()
        if line.strip()
    ]
    nav_log = Path(args.nav_log).read_text(errors="replace")
    failures = [float(x) for x in FAILURE_RE.findall(nav_log)]
    summary = {
        "sample_count": len(rows),
        "start_wall": rows[0]["timestamp_wall"] if rows else None,
        "end_wall": rows[-1]["timestamp_wall"] if rows else None,
        "overall_odom_displacement_m": movement(rows),
        "map_to_odom_tf_total": next(
            (r["map_to_odom_tf"]["count"] for r in reversed(rows)
             if r.get("map_to_odom_tf")), 0
        ),
        "map_to_odom_tf_age_at_end_wall_sec": (
            round(rows[-1]["timestamp_wall"]
                  - next(r["map_to_odom_tf"]["received_wall"]
                         for r in reversed(rows) if r.get("map_to_odom_tf")), 2)
            if rows and any(r.get("map_to_odom_tf") for r in rows) else None
        ),
        "nav2_progress_failure_count": len(failures),
        "failures": [
            {"wall_timestamp": t, "preceding_15s": window_report(rows, t)}
            for t in failures
        ],
    }
    Path(args.output).write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
