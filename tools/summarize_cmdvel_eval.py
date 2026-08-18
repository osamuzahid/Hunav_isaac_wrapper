#!/usr/bin/env python3
"""Summarise a cmd_vel museum_eval directory.

Usage:
  python3 tools/summarize_cmdvel_eval.py /tmp/hunav_cmdvel_eval
"""

from __future__ import annotations

import csv
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


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/hunav_cmdvel_eval"
    metrics_path = os.path.join(out, "metrics.csv")
    if not os.path.isfile(metrics_path):
        print(f"FAIL: missing {metrics_path}")
        return 2

    row = _load_metrics_row(metrics_path)
    path_len = _f(row, "path_length")
    robot_spd = _f(row, "avg_robot_linear_speed")
    min_people = _f(row, "minimum_distance_to_people")
    avg_people = _f(row, "avg_distance_to_closest_person")
    robot_hit = _f(row, "robot_on_person_collision")
    person_hit = _f(row, "person_on_robot_collision")
    intimate = _f(row, "intimate_space_intrusions")
    personal = _f(row, "personal_space_intrusions")
    social = _f(row, "social_space_intrusions")
    ped_vel = _f(row, "avg_pedestrian_velocity")
    idle = _f(row, "time_not_moving")

    print("=== cmd_vel eval (moving Stretch, circulating people) ===")
    print(f"metrics: {metrics_path}")
    print(f"  path_length                {path_len}")
    print(f"  avg_robot_linear_speed     {robot_spd}")
    print(f"  time_not_moving            {idle}")
    print(f"  min_distance_to_people     {min_people}")
    print(f"  avg_closest_person         {avg_people}")
    print(f"  robot_on_person_collision  {robot_hit}")
    print(f"  person_on_robot_collision  {person_hit}")
    print(f"  intimate % ticks           {intimate}")
    print(f"  personal % ticks           {personal}")
    print(f"  social % ticks             {social}")
    print(f"  avg_pedestrian_velocity    {ped_vel}")

    fails = []
    if path_len is None or path_len < 0.5:
        fails.append(f"robot barely moved (path_length={path_len})")
    if robot_spd is not None and robot_spd < 0.02 and (path_len or 0) < 0.5:
        fails.append(f"avg_robot_linear_speed={robot_spd}")
    if ped_vel is not None and ped_vel < 0.15:
        fails.append(f"crowd stuck (avg_pedestrian_velocity={ped_vel:.3f})")
    if min_people is not None and min_people < 0.05:
        fails.append(f"interpenetration (min_d={min_people:.3f} m)")

    if fails:
        print("VERDICT FAIL")
        for f in fails:
            print(" -", f)
        return 1
    print("VERDICT PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
