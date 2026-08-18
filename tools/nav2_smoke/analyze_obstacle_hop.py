#!/usr/bin/env python3
"""Compare a live /odom hop to occupancy A* vs standing people.

Empty-map A* (inflation 0.55 m, same as Stretch Nav2 global inflation) is the
path Nav2 would take with no people. If live odom stays that close to a
statue, the pass-by is corridor geometry, not lidar avoidance.

Exit 0 always (smoke still records GOAL=). Prints OBSTACLE=...
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys


def _load_xy(path: str) -> list[tuple[float, float]]:
    out = []
    with open(path, encoding="utf-8") as f:
        rows = csv.DictReader(f)
        for row in rows:
            out.append((float(row["x"]), float(row["y"])))
    return out


def _min_pt(pt: tuple[float, float], path: list[tuple[float, float]]) -> float:
    if not path:
        return float("inf")
    return min(math.hypot(pt[0] - x, pt[1] - y) for x, y in path)


def _score(d_map: float, d_live: float) -> str:
    extra = d_live - d_map
    # Staggered people sit ~0.2–0.5 m off the empty-map centerline on purpose.
    if d_map > 0.70:
        return "STATUE_OFF_PATH"
    if d_live < 0.50:
        return "THROUGH_OR_CLIP"
    if extra >= 0.40 and d_live >= 0.70:
        return "AVOID"
    return "AMBIGUOUS"


_RANK = {
    "TF_FAIL": 0,
    "NO_TRACE": 1,
    "STATUE_OFF_PATH": 2,
    "THROUGH_OR_CLIP": 3,
    "AMBIGUOUS": 4,
    "AVOID": 5,
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--odom", required=True)
    ap.add_argument("--map-yaml", required=True)
    ap.add_argument("--statue-x", type=float, required=True)
    ap.add_argument("--statue-y", type=float, required=True)
    ap.add_argument("--statue2-x", type=float, default=None)
    ap.add_argument("--statue2-y", type=float, default=None)
    ap.add_argument("--statue3-x", type=float, default=None)
    ap.add_argument("--statue3-y", type=float, default=None)
    ap.add_argument("--bringup-log", default="")
    ap.add_argument("--inflation", type=float, default=0.55)
    ap.add_argument("--start-x", type=float, default=2.0)
    ap.add_argument("--start-y", type=float, default=-8.0)
    ap.add_argument("--goal-x", type=float, default=1.5)
    ap.add_argument("--goal-y", type=float, default=6.5)
    args = ap.parse_args()

    statues = [(args.statue_x, args.statue_y)]
    if args.statue2_x is not None and args.statue2_y is not None:
        statues.append((args.statue2_x, args.statue2_y))
    if args.statue3_x is not None and args.statue3_y is not None:
        statues.append((args.statue3_x, args.statue3_y))

    odom = _load_xy(args.odom) if os.path.isfile(args.odom) else []
    n_origin = 0
    if args.bringup_log and os.path.isfile(args.bringup_log):
        text = open(args.bringup_log, encoding="utf-8", errors="replace").read()
        n_origin = text.count("Sensor origin")

    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "src"))
    sys.path.insert(0, root)
    from hunav_isaac_wrapper.occupancy_path import OccupancyMap

    occ = OccupancyMap.from_yaml(args.map_yaml, inflation_radius_m=args.inflation)
    expected = occ.plan(
        (args.start_x, args.start_y),
        (args.goal_x, args.goal_y),
        waypoint_spacing_m=0.25,
    ) or []

    print(f"odom_samples={len(odom)} expected_wp={len(expected)}")
    print(f"sensor_origin_warns={n_origin}")

    if n_origin > 5:
        print("OBSTACLE=TF_FAIL  (laser not on chassis; /scan not in local costmap)")
        return 0
    if len(odom) < 10:
        print("OBSTACLE=NO_TRACE  (odom CSV missing/short)")
        return 0

    verdicts = []
    for i, statue in enumerate(statues, start=1):
        d_map = _min_pt(statue, expected)
        d_live = _min_pt(statue, odom)
        tag = "" if i == 1 else str(i)
        label = "statue" if i == 1 else f"statue{i}"
        print(f"{label}=({statue[0]:.2f},{statue[1]:.2f})")
        print(f"min_dist_map_path_to_statue{tag}={d_map:.3f} m")
        print(f"min_dist_odom_to_statue{tag}={d_live:.3f} m")
        v = _score(d_map, d_live)
        print(f"{label}_verdict={v}")
        verdicts.append(v)

    overall = min(verdicts, key=lambda v: _RANK[v])
    notes = {
        "STATUE_OFF_PATH": "statue not on empty-map hop; move it",
        "THROUGH_OR_CLIP": "followed map path into a person",
        "AVOID": "live clearance > empty-map path; lidar likely acted",
        "AMBIGUOUS": "could still be controller wiggle",
    }
    print(f"OBSTACLE={overall}  ({notes[overall]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
