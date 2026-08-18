#!/usr/bin/env python3
"""Summarise a parked-robot static eval directory.

Usage:
  python3 tools/summarize_static_eval.py /tmp/hunav_static_eval
"""

from __future__ import annotations

import csv
import math
import os
import sys


def _load_metrics_row(path: str) -> dict:
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f"FAIL: empty metrics CSV {path}")
    return rows[-1]


def _f(row: dict, key: str) -> float | None:
    raw = row.get(key)
    if raw is None or raw == "":
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def _reaction_stats(path: str) -> tuple[int, float, int, float]:
    """Return n_rows, min_dist, yaw90, mean_speed for Impassive agents."""
    if not os.path.isfile(path):
        return 0, math.nan, 0, math.nan
    n = 0
    min_d = math.inf
    yaw90 = 0
    speeds = []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            n += 1
            d = float(row.get("dist_robot", -1.0))
            if d >= 0.0:
                min_d = min(min_d, d)
            if float(row.get("yaw_jump_deg", 0.0)) > 90.0:
                yaw90 += 1
            speeds.append(float(row.get("speed_xy", 0.0)))
    mean_spd = sum(speeds) / len(speeds) if speeds else math.nan
    if min_d is math.inf:
        min_d = math.nan
    return n, min_d, yaw90, mean_spd


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/hunav_static_eval"
    metrics_path = os.path.join(out, "metrics.csv")
    reaction_path = os.path.join(out, "reaction.csv")
    if not os.path.isfile(metrics_path):
        print(f"FAIL: missing {metrics_path}")
        return 2

    row = _load_metrics_row(metrics_path)
    min_people = _f(row, "minimum_distance_to_people")
    avg_people = _f(row, "avg_distance_to_closest_person")
    person_hit = _f(row, "person_on_robot_collision")
    intimate = _f(row, "intimate_space_intrusions")
    personal = _f(row, "personal_space_intrusions")
    social = _f(row, "social_space_intrusions")
    ped_vel = _f(row, "avg_pedestrian_velocity")

    n_rx, rx_min, yaw90, mean_spd = _reaction_stats(reaction_path)

    print("=== static eval (parked robot, circulating people) ===")
    print(f"metrics: {metrics_path}")
    print(f"  min_distance_to_people     {min_people}")
    print(f"  avg_closest_person         {avg_people}")
    print(f"  person_on_robot_collision  {person_hit}")
    print(f"  intimate % ticks           {intimate}")
    print(f"  personal % ticks           {personal}")
    print(f"  social % ticks             {social}")
    print(f"  avg_pedestrian_velocity    {ped_vel}")
    if n_rx:
        print(f"reaction log: {reaction_path} rows={n_rx}")
        print(f"  min dist_robot             {rx_min:.2f}")
        print(f"  yaw_jump>90 count          {yaw90}")
        print(f"  mean speed_xy              {mean_spd:.3f}")
    else:
        print("reaction log: (missing — set HUNAV_REACTION_LOG on the Isaac launch)")

    fails = []
    # Surface-to-surface min; 0.02 m is the evaluator collision epsilon.
    if min_people is not None and min_people < 0.05:
        fails.append(f"people fused/clipped vs robot (min_d={min_people:.3f} m)")
    if person_hit is not None and person_hit > 0:
        fails.append(f"person_on_robot_collision={person_hit}")
    if ped_vel is not None and ped_vel < 0.15:
        fails.append(f"crowd looks stuck (avg_pedestrian_velocity={ped_vel:.3f})")
    if n_rx and yaw90 > max(20, int(0.05 * n_rx)):
        fails.append(f"thrash: {yaw90} yaw jumps >90° in {n_rx} samples")
    if n_rx and not math.isnan(mean_spd) and mean_spd < 0.10:
        fails.append(f"agents idle (mean speed_xy={mean_spd:.3f})")

    if fails:
        print("VERDICT FAIL")
        for f in fails:
            print(" -", f)
        return 1
    print("VERDICT PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
