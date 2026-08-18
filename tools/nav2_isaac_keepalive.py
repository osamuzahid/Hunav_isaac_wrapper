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
  # windowed hospital (see the robot):
  export HUNAV_ISAAC_PROFILE=lab HUNAV_ISAAC_HEADLESS=0
  ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 600 \\
    --world hospital --config hospital_agents --frame-robot
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
    """Deactivate Hawk/Owl Camera prims only (keep chassis meshes visible)."""
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in stage.Traverse():
        path = str(prim.GetPath()).lower()
        if "/nova_carter/" not in path:
            continue
        if prim.GetTypeName() != "Camera":
            continue
        if not any(k in path for k in ("hawk", "owl", "camera", "fisheye")):
            continue
        if prim.IsActive():
            prim.SetActive(False)
            n += 1
    return n


def _disable_usd_diff_drive_graph() -> bool:
    """Mute Nova_Carter_ROS OmniGraph drive (wrapper apply_wheel_actions is used)."""
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath("/World/Nova_Carter/differential_drive")
    if prim and prim.IsValid() and prim.IsActive():
        prim.SetActive(False)
        print(
            "[nav2_keepalive] deactivated /World/Nova_Carter/differential_drive "
            "(wrapper drives wheels; silences Invalid deltaTime spam)",
            flush=True,
        )
        return True
    return False


def _frame_viewport_on_robot(robot, *, announce: bool = True) -> None:
    """Aim /OmniverseKit_Persp at physics pose via Isaac set_camera_view."""
    try:
        import numpy as np
        from isaacsim.core.utils.viewports import set_camera_view
    except Exception as exc:
        print(f"[nav2_keepalive] frame skip: {exc}", flush=True)
        return
    try:
        pos, _ori = robot.get_world_pose()
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        eye = np.array([x - 2.8, y - 2.8, z + 2.4], dtype=float)
        target = np.array([x, y, z + 0.35], dtype=float)
        set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")
        if announce:
            print(
                f"[nav2_keepalive] camera follow → Carter physics "
                f"({x:.2f},{y:.2f},{z:.2f}) — select chassis_link not Nova_Carter root",
                flush=True,
            )
    except Exception as exc:
        print(f"[nav2_keepalive] frame failed: {exc}", flush=True)


def _select_chassis_link() -> None:
    """Stage-tree click on /World/Nova_Carter is an empty wrapper — select the body."""
    try:
        import omni.kit.commands
        import omni.usd

        path = "/World/Nova_Carter/chassis_link"
        if not omni.usd.get_context().get_stage().GetPrimAtPath(path):
            return
        omni.kit.commands.execute(
            "SelectPrimsCommand",
            old_selected_paths=[],
            new_selected_paths=[path],
            expand_in_stage=True,
        )
        print(
            f"[nav2_keepalive] selected {path} (real body; Nova_Carter root gizmo is empty)",
            flush=True,
        )
    except Exception as exc:
        print(f"[nav2_keepalive] select chassis skipped: {exc}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--robot", default="carter_ROS")
    ap.add_argument("--world", default="empty_world")
    ap.add_argument("--config", default="empty_world_agents")
    ap.add_argument(
        "--disable-cameras",
        action="store_true",
        help="Deactivate Carter stereo/fisheye Camera prims (laptop VRAM)",
    )
    ap.add_argument(
        "--frame-robot",
        action="store_true",
        help="Follow Carter with Perspective camera (windowed demos)",
    )
    args = ap.parse_args()

    from hunav_isaac_wrapper.teleop_hunav_sim import (
        TeleopHuNavSim,
        _ensure_nova_carter_visual_config,
        _expand_nova_carter_body_instances,
        simulation_app,
    )
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

    # Expand body meshes before reset so PhysX/tensor views see real Mesh prims.
    if args.robot in ("carter", "carter_ROS"):
        _ensure_nova_carter_visual_config("/World/Nova_Carter", "Full_Merged")
        _expand_nova_carter_body_instances("/World/Nova_Carter")

    node.world.reset()

    if args.robot in ("carter", "carter_ROS"):
        sel = _ensure_nova_carter_visual_config("/World/Nova_Carter", "Full_Merged")
        n_exp = _expand_nova_carter_body_instances("/World/Nova_Carter")
        if not sel:
            print(
                "[nav2_keepalive] WARNING: could not set Nova_Carter Configuration "
                "variant (body may stay Skirt_only / invisible)",
                flush=True,
            )
        # One more reset if we expanded after PhysX had already bound instances.
        if n_exp:
            node.world.reset()
            _ensure_nova_carter_visual_config("/World/Nova_Carter", "Full_Merged")
            _expand_nova_carter_body_instances("/World/Nova_Carter")

    _disable_usd_diff_drive_graph()

    if args.disable_cameras:
        n = _disable_carter_cameras()
        print(f"[nav2_keepalive] deactivated Camera prims: {n}", flush=True)

    follow = bool(args.frame_robot)
    if follow:
        node.world.step(render=True)
        _select_chassis_link()
        _frame_viewport_on_robot(node.robot, announce=True)

    _physx = getattr(omni.physx, "get_physx_interface", None) or getattr(
        omni.physx, "acquire_physx_interface", None
    )
    node.physx_interface = _physx()
    node.physx_sub = node.physx_interface.subscribe_physics_step_events(
        node._on_physics_step
    )

    try:
        pos, _ori = node.robot.get_world_pose()
        pose_s = f"pose=({float(pos[0]):.2f},{float(pos[1]):.2f},{float(pos[2]):.2f})"
    except Exception:
        pose_s = "pose=?"
    print(
        f"[nav2_keepalive] ready robot={args.robot} world={args.world} "
        f"{pose_s} wrapper_cmd_vel_drive=True",
        flush=True,
    )
    if args.robot in ("carter", "carter_ROS"):
        print(
            "[nav2_keepalive] TIP: stage /World/Nova_Carter is an empty wrapper at origin. "
            "Body is /World/Nova_Carter/chassis_link (hospital ~10,-20).",
            flush=True,
        )

    t_end = time.time() + args.seconds
    step_i = 0
    while time.time() < t_end and simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        if getattr(node, "_lab_sensor_handles", None):
            try:
                from hunav_isaac_wrapper.lab_robot_sensors import tick_lab_sensor_handles

                tick_lab_sensor_handles(
                    node._lab_sensor_handles,
                    sim_time=float(node.world.current_time),
                )
            except Exception:
                pass
        if node._chassis_drive:
            node.robot.apply_cmd_vel(
                node.cmd_lin, node.cmd_ang, node.world.get_physics_dt()
            )
        elif getattr(node, "_static_drive", False):
            if hasattr(node.robot, "hold_joints"):
                node.robot.hold_joints()
        else:
            wheel_action = node.diff_controller.forward([node.cmd_lin, node.cmd_ang])
            node.robot.apply_wheel_actions(wheel_action)
        node.world.step(render=True)
        step_i += 1
        # Keep viewport locked on the moving robot (people are elsewhere on the map).
        if follow and step_i % 10 == 0:
            _frame_viewport_on_robot(node.robot, announce=False)

    try:
        node.hunav.close_hunav_nodes()
    except Exception:
        pass
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
