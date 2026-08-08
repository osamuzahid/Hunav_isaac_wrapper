#!/usr/bin/env python3
"""
Keep a HuNav Isaac scene stepping for Nav2 smoke (no internal ros2 CLI).

Usage:
  export OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=0 HUNAV_ISAAC_PROFILE=debug
  ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 360 \\
    --world empty_world --config empty_world_agents
  # museum (lighter agent preset):
  ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 400 \\
    --world museum --config museum_agents --disable-cameras
"""

from __future__ import annotations

import argparse
import os
import sys
import time

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("HUNAV_ISAAC_PROFILE", "debug")
os.environ.setdefault("ROS_DOMAIN_ID", "0")
# Avoid evaluator ABI crash under Isaac unless explicitly requested.
os.environ.setdefault("HUNAV_START_EVALUATOR", "0")
# Headless Nav2 smoke: no viewport behavior overlays.
os.environ.setdefault("HUNAV_BEHAVIOR_LABELS", "0")

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(REPO, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)


def _scenario(name: str) -> str:
    fname = name if name.endswith(".yaml") else f"{name}.yaml"
    for c in (
        os.path.join(SRC, "scenarios", fname),
        os.path.join(
            REPO,
            "..",
            "..",
            "install",
            "hunav_isaac_wrapper",
            "share",
            "hunav_isaac_wrapper",
            "scenarios",
            fname,
        ),
    ):
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError(fname)


def _disable_carter_cameras() -> int:
    """Deactivate Hawk/Owl camera prims to cut VRAM on laptop Nav2 runs."""
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath()).lower()
        if "/nova_carter/" not in path:
            continue
        if not any(k in path for k in ("hawk", "owl", "camera", "fisheye")):
            continue
        # Only leaf-ish camera-related Xforms / Camera prims
        t = prim.GetTypeName()
        if t in ("Camera", "Xform", "Mesh", "Scope") or "camera" in path:
            if prim.IsActive():
                prim.SetActive(False)
                n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--robot", default="carter_ROS")
    ap.add_argument("--world", default="empty_world")
    ap.add_argument("--config", default="empty_world_agents")
    ap.add_argument(
        "--disable-cameras",
        action="store_true",
        help="Deactivate Carter stereo/fisheye camera prims (laptop VRAM)",
    )
    args = ap.parse_args()

    from hunav_isaac_wrapper.teleop_hunav_sim import TeleopHuNavSim, simulation_app
    import omni
    import rclpy

    if not rclpy.ok():
        rclpy.init()

    scenario = _scenario(args.config)
    print(
        f"[nav2_keepalive] world={args.world} config={scenario} robot={args.robot}",
        flush=True,
    )

    node = TeleopHuNavSim(
        map_name=args.world,
        hunav_config=scenario,
        robot_name=args.robot,
    )
    if args.disable_cameras:
        n = _disable_carter_cameras()
        print(f"[nav2_keepalive] deactivated camera-related prims: {n}", flush=True)

    node.world.reset()
    _physx = getattr(omni.physx, "get_physx_interface", None) or getattr(
        omni.physx, "acquire_physx_interface", None
    )
    node.physx_interface = _physx()
    node.physx_sub = node.physx_interface.subscribe_physics_step_events(
        node._on_physics_step
    )

    # Prefer wrapper DifferentialController for /cmd_vel. Carter USD OmniGraph
    # also listens, but logs Invalid deltaTime and was unreliable for museum Nav2;
    # empty_world smoke succeeded with wrapper apply_wheel_actions.
    print(
        f"[nav2_keepalive] ready robot={args.robot} world={args.world} "
        f"wrapper_cmd_vel_drive=True",
        flush=True,
    )
    t_end = time.time() + args.seconds
    while time.time() < t_end and simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        if node._chassis_drive:
            node.robot.apply_cmd_vel(
                node.cmd_lin, node.cmd_ang, node.world.get_physics_dt()
            )
        else:
            wheel_action = node.diff_controller.forward([node.cmd_lin, node.cmd_ang])
            node.robot.apply_wheel_actions(wheel_action)
        node.world.step(render=True)

    try:
        node.hunav.close_hunav_nodes()
    except Exception:
        pass
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
