#!/usr/bin/env python3
"""
Headless smoke: Stretch PhysX diff-drive must translate on the ground plane.

Usage:
  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh tools/stretch_wheeled_smoke.py

PATCH (isaac-social-nav): verifies wheel sphere colliders + disabled mesh sleds give
meaningful XY motion under DifferentialController (cmd_vel-equivalent).
"""

from __future__ import annotations

import math
import os
import sys

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "width": 960,
        "height": 540,
        "headless": True,
        "renderer": "RaytracedLighting",
        "sync_loads": True,
    }
)

import numpy as np
from isaacsim.core.api import World
from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
from isaacsim.robot.wheeled_robots.robots import WheeledRobot

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
USD = os.path.join(REPO, "src", "config", "robots", "stretch", "stretch.usd")
WHEEL_RADIUS = 0.0508
WHEEL_BASE = 0.3407
LIN_VEL = 0.5
SIM_DT = 1.0 / 60.0
SETTLE_STEPS = 90
DRIVE_STEPS = 240  # 4 s @ 60 Hz
PASS_XY_M = 0.5


def main() -> int:
    if not os.path.isfile(USD):
        print(f"FAIL: missing USD {USD}")
        return 2

    world = World(stage_units_in_meters=1.0, physics_dt=SIM_DT, rendering_dt=SIM_DT)
    world.scene.add_default_ground_plane()

    robot = world.scene.add(
        WheeledRobot(
            prim_path="/World/Stretch",
            name="Stretch",
            wheel_dof_names=["joint_left_wheel", "joint_right_wheel"],
            create_robot=True,
            usd_path=USD,
            position=np.array([0.0, 0.0, 0.002]),
        )
    )
    controller = DifferentialController(
        name="diff",
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE,
    )

    world.reset()
    stage = world.stage
    enabled = []
    for p in stage.Traverse():
        sp = str(p.GetPath())
        if not sp.startswith("/World/Stretch"):
            continue
        schemas = p.GetAppliedSchemas()
        if not any("Collision" in s for s in schemas):
            continue
        en = p.GetAttribute("physics:collisionEnabled")
        is_on = True if (en is None or not en.HasAuthoredValue() or en.Get() is None) else bool(en.Get())
        if is_on:
            enabled.append(f"{p.GetTypeName()}:{sp}")
    print(f"enabled_colliders={len(enabled)}")
    for e in enabled:
        print(f"  {e}")

    for _ in range(SETTLE_STEPS):
        world.step(render=False)

    pos0, _ = robot.get_world_pose()
    z0 = float(pos0[2])
    print(f"after_settle pos={pos0} z={z0:.4f}")

    cmd = controller.forward(command=np.array([LIN_VEL, 0.0]))
    print(f"cmd joint_velocities={cmd.joint_velocities}")

    for i in range(DRIVE_STEPS):
        robot.apply_wheel_actions(cmd)
        world.step(render=False)
        if i % 60 == 0:
            p, _ = robot.get_world_pose()
            names = list(robot.dof_names) if hasattr(robot, "dof_names") else []
            jv = robot.get_joint_velocities()
            wheel_jv = None
            if names and jv is not None:
                idxs = [
                    names.index(n)
                    for n in ("joint_left_wheel", "joint_right_wheel")
                    if n in names
                ]
                wheel_jv = [float(jv[i]) for i in idxs]
            print(f"  t={i * SIM_DT:.2f}s pos={p} wheel_jv={wheel_jv}")

    pos1, _ = robot.get_world_pose()
    moved_xy = math.hypot(float(pos1[0] - pos0[0]), float(pos1[1] - pos0[1]))
    z1 = float(pos1[2])
    print(
        f"RESULT moved_xy={moved_xy:.4f} m  z0={z0:.4f} z1={z1:.4f}  "
        f"pos0={pos0} pos1={pos1}"
    )

    ok = moved_xy >= PASS_XY_M and z1 > -0.05
    if ok:
        print(f"PASS: moved_xy={moved_xy:.4f} >= {PASS_XY_M}")
        return 0
    print(f"FAIL: moved_xy={moved_xy:.4f} (need >= {PASS_XY_M}) or sank z={z1:.4f}")
    return 1


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    finally:
        simulation_app.close()
    sys.exit(code)
