#!/usr/bin/env python3
"""Offline A* + crowd-safety bars for campaign *_crowd.yaml files.

Reads optional campaign_goal from the scenario for the robot hop polyline.
Hospital stretcher-box check stays in plan_hospital_crowd_routes.py.

Usage:
  python3 tools/plan_crowd_routes.py office_crowd
  python3 tools/plan_crowd_routes.py hospital_crowd
"""

from __future__ import annotations

import math
import os
import sys

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, ROOT)

from hunav_isaac_wrapper.occupancy_path import OccupancyMap  # noqa: E402

CHOKE_M = 1.0
ROBOT_STANDOFF_M = 2.0
MAX_DETOUR = 1.70
INIT_SEP_M = 2.0


def _load_scenario(name: str) -> dict:
    path = os.path.join(ROOT, "scenarios", f"{name}.yaml")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)["hunav_loader"]["ros__parameters"]


def _path_length(path: list) -> float:
    return sum(
        math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    )


def _min_pt_seg(pt: tuple, a: tuple, b: tuple) -> float:
    ax, ay = a
    bx, by = b
    px, py = pt
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    if den < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _min_path_hop(path: list, hop: tuple) -> float:
    best = float("inf")
    s = max(1, len(path) // 40)
    for pt in path[::s]:
        for i in range(1, len(hop)):
            d = _min_pt_seg(pt, hop[i - 1], hop[i])
            if d < best:
                best = d
    return best


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


def _robot_hop(params: dict, occ: OccupancyMap) -> tuple | None:
    init = params.get("robot_init_pose", {})
    start = (float(init["x"]), float(init["y"]))
    cg = params.get("campaign_goal")
    if not cg:
        print("FAIL: missing campaign_goal in scenario yaml")
        return None
    goal = (float(cg["x"]), float(cg["y"]))
    path = occ.plan_route([start, goal], waypoint_spacing_m=1.0)
    if path is None:
        print(f"FAIL: no hop path {start} → {goal}")
        return None
    length = _path_length(path)
    print(f"hop: {start} → {goal}, A* {length:.1f} m ({len(path)} wp)")
    return tuple(path)


def main() -> int:
    name = sys.argv[1] if len(sys.argv) > 1 else "office_crowd"
    params = _load_scenario(name)
    inflation = float(params.get("plan_inflation_radius_m", 0.40))
    maps = os.path.join(ROOT, "maps", params.get("map_yaml", "office.yaml"))
    occ = OccupancyMap.from_yaml(maps, inflation_radius_m=inflation)
    hop = _robot_hop(params, occ)
    if hop is None:
        print("VERDICT FAIL")
        return 1

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
            print(
                f"FAIL {aname}: {name} must stay Impassive (type 2), "
                f"got {a.get('behavior', {}).get('type')}"
            )
            ok = False
        init = (float(a["init_pose"]["x"]), float(a["init_pose"]["y"]))
        inits[aname] = init
        gids = [str(g) for g in a["goals"]]
        standing = float(a.get("max_vel", 1.0)) < 0.05
        if standing:
            if not occ.is_navigable_world(*init):
                print(f"FAIL {aname}: standing pose not navigable {init}")
                ok = False
                continue
            d_robot = math.hypot(init[0] - robot[0], init[1] - robot[1])
            d_hop = _min_path_hop([init], hop)
            print(
                f"OK   {aname}: STANDING at ({init[0]:.2f},{init[1]:.2f}), "
                f"min_d_spawn={d_robot:.2f} m, min_d_hop={d_hop:.2f} m"
            )
            if d_robot < ROBOT_STANDOFF_M:
                print(
                    f"FAIL {aname}: standing on parked robot "
                    f"{d_robot:.2f} m < {ROBOT_STANDOFF_M:.1f} m"
                )
                ok = False
            paths[aname] = [init]
            continue
        keys = [init] + [goals[g] for g in gids] + [goals[gids[0]]]
        path = occ.plan_route(keys, waypoint_spacing_m=1.0)
        if path is None:
            print(f"FAIL {aname}: no path")
            ok = False
            continue
        non_nav = [p for p in path if not occ.is_navigable_world(*p)]
        length = _path_length(path)
        key_len = _path_length([goals[g] for g in gids] + [goals[gids[0]]])
        detour = length / key_len if key_len > 1e-3 else 999.0
        d_robot = math.hypot(init[0] - robot[0], init[1] - robot[1])
        d_hop = _min_path_hop(path, hop)
        print(
            f"OK   {aname}: {len(path)} wp, path_len≈{length:.1f} m, "
            f"detour={detour:.2f}, min_d_spawn={d_robot:.2f} m, "
            f"min_d_hop={d_hop:.2f} m"
        )
        if non_nav:
            print(
                f"FAIL {aname}: {len(non_nav)} waypoints not navigable "
                f"(first {non_nav[0]})"
            )
            ok = False
        if detour > MAX_DETOUR:
            print(
                f"FAIL {aname}: A* detour {detour:.2f} > {MAX_DETOUR} "
                "(loop hits a wall; agents will wall-walk)"
            )
            ok = False
        if d_robot < ROBOT_STANDOFF_M:
            print(
                f"FAIL {aname}: init on parked robot "
                f"{d_robot:.2f} m < {ROBOT_STANDOFF_M:.1f} m"
            )
            ok = False
        paths[aname] = path

    names = list(paths)
    pair_best = float("inf")
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            d = _min_path_path(paths[a], paths[b])
            pair_best = min(pair_best, d)
            if d < CHOKE_M:
                stand = (
                    float(params[a].get("max_vel", 1.0)) < 0.05
                    or float(params[b].get("max_vel", 1.0)) < 0.05
                )
                if stand:
                    print(f"NOTE {a} vs {b}: {d:.2f} m (standing blocker; ok)")
                    continue
                hop_share = (
                    _min_path_hop(paths[a], hop) < 0.5
                    and _min_path_hop(paths[b], hop) < 0.5
                )
                if hop_share:
                    print(f"NOTE {a} vs {b}: {d:.2f} m (same hop corridor; ok)")
                    continue
                print(f"FAIL {a} vs {b}: share a {d:.2f} m choke (< {CHOKE_M:.1f} m)")
                ok = False
    print(f"     min pairwise path {pair_best:.2f} m (bar {CHOKE_M:.1f} m)")

    ns = list(inits)
    for i, a in enumerate(ns):
        for b in ns[i + 1 :]:
            d = math.hypot(inits[a][0] - inits[b][0], inits[a][1] - inits[b][1])
            if d < INIT_SEP_M:
                stand = (
                    float(params[a].get("max_vel", 1.0)) < 0.05
                    or float(params[b].get("max_vel", 1.0)) < 0.05
                )
                if stand:
                    continue
                print(f"FAIL init {a} vs {b}: {d:.2f} m < {INIT_SEP_M:.1f} m")
                ok = False

    print("VERDICT", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
