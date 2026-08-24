#!/usr/bin/env python3
"""Summarise a Stretch Nav2 crowd-hop eval directory (#70 baseline).

Usage:
  python3 tools/summarize_nav2_crowd_eval.py /tmp/hunav_nav2_crowd_eval
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


def _b(row: dict, key: str) -> str:
    raw = (row.get(key) or "").strip().lower()
    if raw in ("1", "true", "yes"):
        return "true"
    if raw in ("0", "false", "no"):
        return "false"
    return raw or "?"


def _goal_line(out: str):
    summary = os.path.join(out, "nav2_smoke_summary.txt")
    if not os.path.isfile(summary):
        return "GOAL=?"
    text = open(summary, encoding="utf-8", errors="replace").read()
    for line in text.splitlines():
        if line.startswith("GOAL=") or line.startswith("OBSTACLE="):
            yield line


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/hunav_nav2_crowd_eval"
    metrics_path = os.path.join(out, "metrics.csv")
    if not os.path.isfile(metrics_path):
        print(f"FAIL: missing {metrics_path}")
        return 2

    row = _load_metrics_row(metrics_path)
    t_goal = _f(row, "time_to_reach_goal")
    path_len = _f(row, "path_length")
    completed = _b(row, "completed")
    min_tgt = _f(row, "minimum_distance_to_target")
    fin_tgt = _f(row, "final_distance_to_target")
    min_people = _f(row, "minimum_distance_to_people")
    avg_people = _f(row, "avg_distance_to_closest_person")
    robot_hit = _f(row, "robot_on_person_collision")
    person_hit = _f(row, "person_on_robot_collision")
    intimate = _f(row, "intimate_space_intrusions")
    personal = _f(row, "personal_space_intrusions")
    social = _f(row, "social_space_intrusions")
    robot_spd = _f(row, "avg_robot_linear_speed")
    idle = _f(row, "time_not_moving")
    ped_vel = _f(row, "avg_pedestrian_velocity")

    print("=== Nav2 crowd eval ===")
    print(f"metrics: {metrics_path}")
    for line in _goal_line(out):
        print(f"  {line}")
    print(f"  time_to_reach_goal         {t_goal}")
    print(f"  path_length                {path_len}")
    print(f"  completed                  {completed}")
    print(f"  min_distance_to_target     {min_tgt}")
    print(f"  final_distance_to_target   {fin_tgt}")
    print(f"  min_distance_to_people     {min_people}")
    print(f"  avg_closest_person         {avg_people}")
    print(f"  robot_on_person_collision  {robot_hit}")
    print(f"  person_on_robot_collision  {person_hit}")
    print(f"  intimate % ticks           {intimate}")
    print(f"  personal % ticks           {personal}")
    print(f"  social % ticks             {social}")
    print(f"  avg_robot_linear_speed     {robot_spd}")
    print(f"  time_not_moving            {idle}")
    print(f"  avg_pedestrian_velocity    {ped_vel}")

    fails = []
    if path_len is None or path_len < 7.0:
        fails.append(f"path too short for hop (path_length={path_len})")
    if ped_vel is not None and ped_vel < 0.05:
        fails.append(f"crowd stuck (avg_pedestrian_velocity={ped_vel:.3f})")
    if min_people is not None and min_people < 0.05:
        fails.append(f"interpenetration (min_d={min_people:.3f} m)")
    if (robot_hit or 0) > 0 or (person_hit or 0) > 0:
        fails.append(f"collision robot={robot_hit} person={person_hit}")

    if fails:
        print("VERDICT FAIL")
        for f in fails:
            print(" -", f)
        return 1
    print("VERDICT PASS  (CSV ok; personal-space % is expected with statues on the hop)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
