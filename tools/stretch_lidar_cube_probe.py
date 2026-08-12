#!/usr/bin/env python3
"""
stretch_lidar_cube_probe.py

PATCH (isaac-social-nav): control test for Stretch RTX /scan.

Spawns Stretch on empty_world, places a large opaque cube ahead of the base
lidar, and reports how many LaserScan hits land on that cube vs elsewhere.

If the cube shows a dense arc and museum walls do not, the lidar is fine and
the museum mesh/visibility path is the problem.

Usage (wrapper root):
  source /opt/ros/jazzy/setup.bash && source ../../../install/setup.bash
  export OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=0 HUNAV_ISAAC_PROFILE=debug
  export HUNAV_LAB_LIDAR=1 HUNAV_LAB_CAMERAS=0
  ~/isaacsim/python.sh tools/stretch_lidar_cube_probe.py \\
    --seconds 35 --summary /tmp/stretch_lidar_cube_summary.txt
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from collections import deque


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _spawn_cube(stage, path: str, xyz, size: float = 1.2) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(size)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    # Bright opaque PreviewSurface so RTX lidar has a solid target.
    if not stage.GetPrimAtPath("/World/Looks"):
        stage.DefinePrim("/World/Looks", "Scope")
    mat_path = "/World/Looks/LidarCubeRed"
    if not stage.GetPrimAtPath(mat_path):
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.9, 0.1, 0.1)
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    else:
        mat = UsdShade.Material(stage.GetPrimAtPath(mat_path))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
    print(f"[lidar_cube_probe] cube at {path} xyz={xyz} size={size}", flush=True)


def _analyze_scan(msg, cube_xy, cube_half: float) -> dict:
    """Count hits whose XY (laser frame ≈ robot for parked) fall near the cube."""
    hits = []
    a = msg.angle_min
    for r in msg.ranges:
        if math.isfinite(r) and msg.range_min < r < msg.range_max and r > 0:
            x = r * math.cos(a)
            y = r * math.sin(a)
            hits.append((x, y, r, a))
        a += msg.angle_increment

    cx, cy = cube_xy
    on_cube = []
    for x, y, r, ang in hits:
        # Axis-aligned square in laser/base XY (robot at origin, yaw 0).
        if abs(x - cx) <= cube_half + 0.15 and abs(y - cy) <= cube_half + 0.15:
            on_cube.append((x, y, r, ang))

    return {
        "n_ranges": len(msg.ranges),
        "n_hits": len(hits),
        "n_on_cube": len(on_cube),
        "hit_r_minmax": (
            (min(h[2] for h in hits), max(h[2] for h in hits)) if hits else None
        ),
        "cube_r_minmax": (
            (min(h[2] for h in on_cube), max(h[2] for h in on_cube))
            if on_cube
            else None
        ),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Stretch lidar cube control test")
    ap.add_argument("--seconds", type=float, default=35.0)
    ap.add_argument("--summary", default="/tmp/stretch_lidar_cube_summary.txt")
    ap.add_argument("--cube-x", type=float, default=2.5)
    ap.add_argument("--cube-size", type=float, default=1.2)
    args = ap.parse_args()

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    os.environ.setdefault("HUNAV_ISAAC_PROFILE", "debug")
    os.environ.setdefault("HUNAV_LAB_SENSORS", "1")
    os.environ.setdefault("HUNAV_LAB_LIDAR", "1")
    os.environ.setdefault("HUNAV_LAB_CAMERAS", "0")

    repo = _repo_root()
    src = os.path.join(repo, "src")
    if src not in sys.path:
        sys.path.insert(0, src)

    from hunav_isaac_wrapper.sim_app_config import apply_profile_to_environ

    apply_profile_to_environ(profile="debug", headless=True)

    scen = os.path.join(repo, "src", "scenarios", "empty_world_agents.yaml")
    print(
        f"[lidar_cube_probe] stretch empty_world cube_x={args.cube_x} "
        f"size={args.cube_size} seconds={args.seconds}",
        flush=True,
    )

    from hunav_isaac_wrapper.teleop_hunav_sim import TeleopHuNavSim, simulation_app
    import omni
    import omni.usd
    import rclpy
    from sensor_msgs.msg import LaserScan

    if not rclpy.ok():
        rclpy.init()

    node = TeleopHuNavSim(
        map_name="empty_world",
        hunav_config=scen,
        robot_name="stretch",
    )
    node.world.reset()

    stage = omni.usd.get_context().get_stage()
    # Cube center at lidar height (~0.2 m) so a horizontal 2D beam must hit it.
    cube_xyz = (float(args.cube_x), 0.0, 0.25)
    _spawn_cube(stage, "/World/LidarTestCube", cube_xyz, size=float(args.cube_size))

    _physx = getattr(omni.physx, "get_physx_interface", None) or getattr(
        omni.physx, "acquire_physx_interface", None
    )
    node.physx_interface = _physx()
    node.physx_sub = node.physx_interface.subscribe_physics_step_events(
        node._on_physics_step
    )

    # Isaac ROS bridge often uses a separate context; same-process rclpy may
    # never see /scan. Listen from a system-ROS subprocess instead.
    listener_path = "/tmp/stretch_lidar_cube_listener.py"
    listener_out = "/tmp/stretch_lidar_cube_scans.json"
    if os.path.isfile(listener_out):
        os.remove(listener_out)
    with open(listener_path, "w", encoding="utf-8") as f:
        f.write(
            """
import json, math, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

CUBE_X = float(%(cube_x)s)
CUBE_HALF = float(%(cube_half)s)
OUT = %(out)r
deadline = time.time() + float(%(seconds)s)

rclpy.init()
node = rclpy.create_node("stretch_lidar_cube_listener")
box = []

def cb(msg):
    hits = []
    on = []
    a = msg.angle_min
    for r in msg.ranges:
        if math.isfinite(r) and msg.range_min < r < msg.range_max and r > 0:
            x = r * math.cos(a)
            y = r * math.sin(a)
            hits.append((x, y, r))
            if abs(x - CUBE_X) <= CUBE_HALF + 0.15 and abs(y) <= CUBE_HALF + 0.15:
                on.append((x, y, r))
        a += msg.angle_increment
    box.append({
        "n_ranges": len(msg.ranges),
        "n_hits": len(hits),
        "n_on_cube": len(on),
        "hit_r_minmax": [min(h[2] for h in hits), max(h[2] for h in hits)] if hits else None,
        "cube_r_minmax": [min(h[2] for h in on), max(h[2] for h in on)] if on else None,
    })

node.create_subscription(LaserScan, "/scan", cb, qos_profile_sensor_data)
while time.time() < deadline and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
last = box[-1] if box else None
with open(OUT, "w", encoding="utf-8") as fh:
    json.dump({"n_msgs": len(box), "last": last, "samples": box[-5:]}, fh)
print("listener_done", len(box), last, flush=True)
"""
            % {
                "cube_x": args.cube_x,
                "cube_half": float(args.cube_size) * 0.5,
                "out": listener_out,
                "seconds": float(args.seconds) + 5.0,
            }
        )

    import subprocess

    listener = subprocess.Popen(
        [
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=0 && "
            f"python3 {listener_path} > /tmp/stretch_lidar_cube_listener.log 2>&1",
        ]
    )
    print(f"[lidar_cube_probe] external /scan listener pid={listener.pid}", flush=True)

    deadline = time.time() + float(args.seconds)
    steps = 0
    while simulation_app.is_running() and time.time() < deadline:
        node.world.step(render=True)
        steps += 1
        if getattr(node, "_lab_sensor_handles", None):
            from hunav_isaac_wrapper.lab_robot_sensors import tick_lab_sensor_handles

            tick_lab_sensor_handles(
                node._lab_sensor_handles,
                sim_time=float(node.world.current_time),
            )

    try:
        listener.wait(timeout=15)
    except subprocess.TimeoutExpired:
        listener.kill()

    last_stats = None
    n_msgs = 0
    if os.path.isfile(listener_out):
        import json

        with open(listener_out, encoding="utf-8") as fh:
            payload = json.load(fh)
        n_msgs = int(payload.get("n_msgs") or 0)
        last_stats = payload.get("last")

    ok = False
    if last_stats and last_stats.get("n_on_cube", 0) >= 20:
        ok = True
    elif (
        last_stats
        and last_stats.get("n_hits", 0) > 0
        and last_stats.get("n_on_cube", 0)
        >= max(10, int(0.15 * last_stats["n_hits"]))
    ):
        ok = True

    lines = [
        f"steps={steps}",
        f"scan_msgs={n_msgs}",
        f"cube_xyz={cube_xyz} size={args.cube_size}",
        f"stats={last_stats}",
        f"verdict: {'PASS — lidar sees opaque cube' if ok else 'FAIL — cube not clearly in /scan'}",
        "note: if PASS here but museum walls sparse → museum mesh/visibility issue",
    ]
    text = "\n".join(lines) + "\n"
    with open(args.summary, "w", encoding="utf-8") as f:
        f.write(text)
    print(text, flush=True)
    print(f"[lidar_cube_probe] wrote {args.summary}", flush=True)

    try:
        node.hunav.close_hunav_nodes()
    except Exception:
        pass
    simulation_app.close()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
