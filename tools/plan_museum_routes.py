#!/usr/bin/env python3
"""Offline smoke: plan museum routes without starting Isaac Sim.

Usage (from wrapper package root or any cwd):
  python3 tools/plan_museum_routes.py
"""

from __future__ import annotations

import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, ROOT)

from hunav_isaac_wrapper.occupancy_path import OccupancyMap  # noqa: E402


def main() -> int:
    maps = os.path.join(ROOT, "maps", "museum.yaml")
    m = OccupancyMap.from_yaml(maps, inflation_radius_m=0.35)
    routes = {
        "museum_agents/agent1": [(-6.5, -4.5), (-6.5, -3.5), (-18.0, -5.0), (10.0, 5.0), (2.0, -8.0), (-6.5, -3.5)],
        "museum_agents/agent2": [(-5.0, -1.0), (8.0, -2.0), (-15.0, 8.0), (0.0, 10.0), (-20.0, 0.0), (8.0, -2.0)],
        "cross_museum": [(-20.0, 0.0), (11.0, -10.0), (-4.0, 14.0), (-20.0, 0.0)],
    }
    ok = True
    for name, keys in routes.items():
        path = m.plan_route(keys, waypoint_spacing_m=1.0)
        if path is None:
            print(f"FAIL {name}: no path")
            ok = False
            continue
        length = sum(
            math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            for i in range(1, len(path))
        )
        print(f"OK   {name}: {len(path)} waypoints, path_len≈{length:.1f} m")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
