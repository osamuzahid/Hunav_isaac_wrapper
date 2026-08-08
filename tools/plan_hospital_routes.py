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


def _load_scenario(name: str) -> dict:
    path = os.path.join(ROOT, "scenarios", f"{name}.yaml")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)["hunav_loader"]["ros__parameters"]


def main() -> int:
    maps = os.path.join(ROOT, "maps", "hospital.yaml")
    m = OccupancyMap.from_yaml(maps, inflation_radius_m=0.35)
    ok = True
    for scenario in ("hospital_agents", "hospital_behaviors"):
        params = _load_scenario(scenario)
        goals = {
            str(k): (float(v["x"]), float(v["y"]))
            for k, v in params["global_goals"].items()
        }
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
            length = sum(
                math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                for i in range(1, len(path))
            )
            print(f"OK   {label}: {len(path)} waypoints, path_len≈{length:.1f} m")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
