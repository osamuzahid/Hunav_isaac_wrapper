#!/usr/bin/env python3
"""
Headless probe: museum_behaviors + Stretch, drive toward agents, log reactions.

Writes CSV via HUNAV_REACTION_LOG then prints analyze_reaction_log summary.

Usage (from wrapper repo root, ROS sourced):
  export OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=0 HUNAV_ISAAC_PROFILE=debug
  export HUNAV_REACTION_LOG=/tmp/hunav_reaction.csv
  # optional: kill leftover managers first
  ~/isaacsim/python.sh tools/near_robot_reaction_probe.py --seconds 45

Wrapper starts its own hunav_loader + agent_manager — do not start a second one.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

# Profile + log path must be set before SimulationApp / teleop import.
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("HUNAV_ISAAC_PROFILE", "debug")
os.environ.setdefault("ROS_DOMAIN_ID", "0")

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(REPO, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

LOG_DEFAULT = "/tmp/hunav_reaction.csv"


def _resolve_scenario() -> str:
    candidates = [
        os.path.join(SRC, "scenarios", "museum_behaviors.yaml"),
        os.path.join(
            REPO,
            "..",
            "..",
            "install",
            "hunav_isaac_wrapper",
            "share",
            "hunav_isaac_wrapper",
            "scenarios",
            "museum_behaviors.yaml",
        ),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError("museum_behaviors.yaml not found")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--log", default=os.environ.get("HUNAV_REACTION_LOG", LOG_DEFAULT))
    ap.add_argument("--lin-vel", type=float, default=0.65)
    args = ap.parse_args()

    os.environ["HUNAV_REACTION_LOG"] = args.log
    if os.path.isfile(args.log):
        os.remove(args.log)

    # Import after env: teleop builds SimulationApp at import time.
    from hunav_isaac_wrapper.teleop_hunav_sim import TeleopHuNavSim, simulation_app
    import rclpy

    if not rclpy.ok():
        rclpy.init()

    scenario = _resolve_scenario()
    print(f"[probe] scenario={scenario}")
    print(f"[probe] log={args.log} seconds={args.seconds}")

    node = TeleopHuNavSim(
        map_name="museum",
        hunav_config=scenario,
        robot_name="stretch",
    )

    # Reset first (same as TeleopHuNavSim.run), then wait for managers.
    node.world.reset()
    import omni
    import numpy as np

    _physx = getattr(omni.physx, "get_physx_interface", None) or getattr(
        omni.physx, "acquire_physx_interface", None
    )
    node.physx_interface = _physx()
    node.physx_sub = node.physx_interface.subscribe_physics_step_events(
        node._on_physics_step
    )

    deadline = time.time() + 90.0
    while time.time() < deadline:
        if node.hunav.compute_agents_client.wait_for_service(timeout_sec=1.0):
            print("[probe] /compute_agents ready")
            break
        node.world.step(render=False)
    else:
        print("FAIL: /compute_agents never became ready")
        try:
            node.hunav.close_hunav_nodes()
        except Exception:
            pass
        simulation_app.close()
        return 2

    node.hunav.send_agents_msg()

    # Approach points ~2.2 m from each spawn. Drive between them (no mid-tour
    # teleport — that flipped Scared face-away ~180°).
    waypoints = [
        (-6.5, -6.0),  # Impassive / Threatening
        (-7.0, -3.5),  # Scared (dwell)
        (-7.0, -3.5),  # Scared again
        (-5.0, -3.5),  # Curious (2.5 m south of spawn)
        (-4.0, -0.5),  # Surprised
        (-3.8, -6.0),  # Threatening
    ]
    wp_i = 0
    wp_dwell = max(12.0, args.seconds / max(1, len(waypoints)))
    wp_t0 = time.time()

    xform = getattr(node.robot, "_xform", None)
    if xform is not None:
        pos, ori = xform.get_world_pose()
        xform.set_world_pose(
            position=np.array(
                [waypoints[0][0], waypoints[0][1], float(pos[2])], dtype=float
            ),
            orientation=ori,
        )
        print(f"[probe] robot start at {waypoints[0]}")

    t0 = time.time()
    steps = 0
    ready_steps = 0
    try:
        while simulation_app.is_running() and (time.time() - t0) < args.seconds:
            if time.time() - wp_t0 >= wp_dwell:
                wp_i = (wp_i + 1) % len(waypoints)
                wp_t0 = time.time()
                print(f"[probe] waypoint -> {waypoints[wp_i]}")
                # Standoff teleport OK now that Scared LOS is omnidirectional.
                if xform is not None:
                    pos, ori = xform.get_world_pose()
                    xform.set_world_pose(
                        position=np.array(
                            [waypoints[wp_i][0], waypoints[wp_i][1], float(pos[2])],
                            dtype=float,
                        ),
                        orientation=ori,
                    )

            tx, ty = waypoints[wp_i]
            try:
                rpos, _ = node.robot.get_world_pose()
                rx, ry = float(rpos[0]), float(rpos[1])
                dx = tx - rx
                dy = ty - ry
                dist = math.hypot(dx, dy)
                if dist > 0.3:
                    yaw_cmd = math.atan2(dy, dx)
                    node.cmd_lin = min(args.lin_vel, 0.35 + 0.25 * dist)
                    node.cmd_ang = max(-1.2, min(1.2, 2.0 * yaw_cmd))
                else:
                    node.cmd_lin = 0.0
                    node.cmd_ang = 0.0
            except Exception:
                node.cmd_lin = 0.0
                node.cmd_ang = 0.0

            node.world.step(render=True)
            if getattr(node, "_chassis_drive", False):
                dt = node.world.get_physics_dt()
                node.robot.apply_cmd_vel(node.cmd_lin, node.cmd_ang, dt)
            steps += 1
            if node.hunav.compute_agents_client.service_is_ready():
                ready_steps += 1
    finally:
        try:
            node.hunav.close_hunav_nodes()
        except Exception:
            pass
        simulation_app.close()

    print(
        f"[probe] done steps={steps} ready_steps={ready_steps} "
        f"elapsed={time.time() - t0:.1f}s"
    )
    if not os.path.isfile(args.log) or os.path.getsize(args.log) == 0:
        print(f"FAIL: no reaction log at {args.log}")
        return 2

    analyze = os.path.join(os.path.dirname(__file__), "analyze_reaction_log.py")
    import subprocess

    return subprocess.call([sys.executable, analyze, args.log])


if __name__ == "__main__":
    raise SystemExit(main())
