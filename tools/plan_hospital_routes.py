#!/usr/bin/env python3
"""Offline smoke: plan hospital routes without starting Isaac Sim.

Usage (from wrapper package root or any cwd):
  python3 tools/plan_hospital_routes.py
"""

from __future__ import annotations

import math
import os
import sys

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, ROOT)

from hunav_isaac_wrapper.occupancy_path import OccupancyMap  # noqa: E402

# Crowd-safety bars for hospital_behaviors (parked Stretch at reception).
CHOKE_M = 1.0
ROBOT_STANDOFF_M = 5.0


def _load_scenario(name: str) -> dict:
    path = os.path.join(ROOT, "scenarios", f"{name}.yaml")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)["hunav_loader"]["ros__parameters"]


def _path_length(path: list) -> float:
    return sum(
        math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    )


def _min_pt_path(pt: tuple, path: list) -> float:
    return min(math.hypot(pt[0] - x, pt[1] - y) for x, y in path)


def _min_path_path(p1: list, p2: list) -> float:
    best = float("inf")
    for a in p1:
        for b in p2:
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            if d < best:
                best = d
    return best


def main() -> int:
    maps = os.path.join(ROOT, "maps", "hospital.yaml")
    ok = True
    for scenario in ("hospital_agents", "hospital_behaviors"):
        params = _load_scenario(scenario)
        inflation = float(params.get("plan_inflation_radius_m", 0.35))
        m = OccupancyMap.from_yaml(maps, inflation_radius_m=inflation)
        goals = {
            str(k): (float(v["x"]), float(v["y"]))
            for k, v in params["global_goals"].items()
        }
        robot = (
            float(params["robot_init_pose"]["x"]),
            float(params["robot_init_pose"]["y"]),
        )
        paths = {}
        for aname in params["agents"]:
            a = params[aname]
            init = (float(a["init_pose"]["x"]), float(a["init_pose"]["y"]))
            gids = [str(g) for g in a["goals"]]
            keys = [init] + [goals[g] for g in gids] + [goals[gids[0]]]
            path = m.plan_route(keys, waypoint_spacing_m=1.0)
            label = f"{scenario}/{aname}"
            if path is None:
                print(f"FAIL {label}: no path")
                ok = False
                continue
            non_nav = [p for p in path if not m.is_navigable_world(*p)]
            d_robot = _min_pt_path(robot, path)
            print(
                f"OK   {label}: {len(path)} waypoints, "
                f"path_len≈{_path_length(path):.1f} m, "
                f"min_d_robot={d_robot:.2f} m"
            )
            if non_nav:
                print(f"FAIL {label}: {len(non_nav)} waypoints not navigable "
                      f"(first {non_nav[0]})")
                ok = False
            paths[aname] = (path, d_robot)

        names = list(paths)
        for i, a in enumerate(names):
            for b in names[i + 1 :]:
                d = _min_path_path(paths[a][0], paths[b][0])
                if d < CHOKE_M:
                    print(
                        f"FAIL {scenario}: {a} vs {b} share a "
                        f"{d:.2f} m choke (< {CHOKE_M:.1f} m)"
                    )
                    ok = False
                elif scenario == "hospital_behaviors":
                    print(f"     {a} vs {b}: min {d:.2f} m")

        if scenario == "hospital_behaviors":
            for aname, (_, d_robot) in paths.items():
                if d_robot < ROBOT_STANDOFF_M:
                    print(
                        f"FAIL {scenario}/{aname}: min distance to robot "
                        f"{d_robot:.2f} m < {ROBOT_STANDOFF_M:.1f} m standoff"
                    )
                    ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
