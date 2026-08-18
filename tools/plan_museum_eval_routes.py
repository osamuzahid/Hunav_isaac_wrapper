#!/usr/bin/env python3
"""Offline A* + crowd-safety bars for museum_eval (10 Impassive loops).

Usage:
  python3 tools/plan_museum_eval_routes.py
"""

from __future__ import annotations

import math
import os
import sys

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, ROOT)

from hunav_isaac_wrapper.occupancy_path import OccupancyMap  # noqa: E402

CHOKE_M = 2.0
ROBOT_STANDOFF_M = 2.0
MAX_DETOUR = 1.35
INIT_SEP_M = 4.0


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
    s1 = max(1, len(p1) // 30)
    s2 = max(1, len(p2) // 30)
    for a in p1[::s1]:
        for b in p2[::s2]:
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            if d < best:
                best = d
    return best


def main() -> int:
    params = _load_scenario("museum_eval")
    inflation = float(params.get("plan_inflation_radius_m", 0.40))
    maps = os.path.join(ROOT, "maps", params.get("map_yaml", "museum.yaml"))
    m = OccupancyMap.from_yaml(maps, inflation_radius_m=inflation)
    goals = {
        str(k): (float(v["x"]), float(v["y"])) for k, v in params["global_goals"].items()
    }
    robot = (
        float(params["robot_init_pose"]["x"]),
        float(params["robot_init_pose"]["y"]),
    )
    ok = True
    paths = {}
    inits = {}
    for aname in params["agents"]:
        a = params[aname]
        if int(a.get("behavior", {}).get("type", 2)) != 2:
            print(f"FAIL {aname}: museum_eval must stay Impassive (type 2), "
                  f"got {a.get('behavior', {}).get('type')}")
            ok = False
        init = (float(a["init_pose"]["x"]), float(a["init_pose"]["y"]))
        inits[aname] = init
        gids = [str(g) for g in a["goals"]]
        keys = [init] + [goals[g] for g in gids] + [goals[gids[0]]]
        path = m.plan_route(keys, waypoint_spacing_m=1.0)
        if path is None:
            print(f"FAIL {aname}: no path")
            ok = False
            continue
        non_nav = [p for p in path if not m.is_navigable_world(*p)]
        length = _path_length(path)
        # rectangle perimeter ≈ 4 unique goals; detour vs that chord cycle
        key_len = _path_length([goals[g] for g in gids] + [goals[gids[0]]])
        detour = length / key_len if key_len > 1e-3 else 999.0
        d_robot = _min_pt_path(robot, path)
        print(
            f"OK   {aname}: {len(path)} wp, path_len≈{length:.1f} m, "
            f"detour={detour:.2f}, min_d_robot={d_robot:.2f} m"
        )
        if non_nav:
            print(f"FAIL {aname}: {len(non_nav)} waypoints not navigable "
                  f"(first {non_nav[0]})")
            ok = False
        if detour > MAX_DETOUR:
            print(f"FAIL {aname}: A* detour {detour:.2f} > {MAX_DETOUR} "
                  "(loop hits an exhibit; agents will wall-walk)")
            ok = False
        if d_robot < ROBOT_STANDOFF_M:
            print(f"FAIL {aname}: min distance to parked Stretch "
                  f"{d_robot:.2f} m < {ROBOT_STANDOFF_M:.1f} m")
            ok = False
        paths[aname] = path

    names = list(paths)
    pair_best = float("inf")
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            d = _min_path_path(paths[a], paths[b])
            pair_best = min(pair_best, d)
            if d < CHOKE_M:
                print(f"FAIL {a} vs {b}: share a {d:.2f} m choke (< {CHOKE_M:.1f} m)")
                ok = False
    print(f"     min pairwise path {pair_best:.2f} m (bar {CHOKE_M:.1f} m)")

    ns = list(inits)
    for i, a in enumerate(ns):
        for b in ns[i + 1 :]:
            d = math.hypot(inits[a][0] - inits[b][0], inits[a][1] - inits[b][1])
            if d < INIT_SEP_M:
                print(f"FAIL init {a} vs {b}: {d:.2f} m < {INIT_SEP_M:.1f} m")
                ok = False

    print("VERDICT", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
