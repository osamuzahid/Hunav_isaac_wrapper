#!/usr/bin/env python3
"""
Headless Nav2 preflight: empty_world + carter_ROS, step sim, dump ROS topics.

Validates whether Nova_Carter_ROS publishes scan/odom/TF usable by Nav2.

Usage (ROS sourced; no separate agent manager):
  export OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=0 HUNAV_ISAAC_PROFILE=debug
  ~/isaacsim/python.sh tools/nav2_sensor_probe.py --seconds 45 \\
    --summary /tmp/nav2_smoke/sensor_summary.txt
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("HUNAV_ISAAC_PROFILE", "debug")
os.environ.setdefault("ROS_DOMAIN_ID", "0")

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC = os.path.join(REPO, "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)


def _resolve_scenario() -> str:
    candidates = [
        os.path.join(SRC, "scenarios", "empty_world_agents.yaml"),
        os.path.join(
            REPO,
            "..",
            "..",
            "install",
            "hunav_isaac_wrapper",
            "share",
            "hunav_isaac_wrapper",
            "scenarios",
            "empty_world_agents.yaml",
        ),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError("empty_world_agents.yaml not found")


def _ros2(*args: str, timeout: float = 8.0) -> str:
    try:
        out = subprocess.check_output(
            ["ros2", *args],
            stderr=subprocess.STDOUT,
            timeout=timeout,
            text=True,
        )
        return out
    except Exception as exc:
        return f"ERROR: {exc}\n"


def _echo_once(topic: str, timeout: float = 8.0) -> str:
    """Isaac ROS bridge often uses BEST_EFFORT; default echo is RELIABLE."""
    return _ros2(
        "topic",
        "echo",
        topic,
        "--once",
        "--qos-reliability",
        "best_effort",
        "--qos-durability",
        "volatile",
        timeout=timeout,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=45.0)
    ap.add_argument("--summary", default="/tmp/nav2_smoke/sensor_summary.txt")
    ap.add_argument("--robot", default="carter_ROS")
    ap.add_argument("--world", default="empty_world")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.summary) or ".", exist_ok=True)

    from hunav_isaac_wrapper.teleop_hunav_sim import TeleopHuNavSim, simulation_app
    import omni
    import rclpy

    if not rclpy.ok():
        rclpy.init()

    scenario = _resolve_scenario()
    print(f"[nav2_sensor] scenario={scenario}")
    print(f"[nav2_sensor] robot={args.robot} world={args.world} seconds={args.seconds}")

    node = TeleopHuNavSim(
        map_name=args.world,
        hunav_config=scenario,
        robot_name=args.robot,
    )

    node.world.reset()
    _physx = getattr(omni.physx, "get_physx_interface", None) or getattr(
        omni.physx, "acquire_physx_interface", None
    )
    node.physx_interface = _physx()
    node.physx_sub = node.physx_interface.subscribe_physics_step_events(
        node._on_physics_step
    )

    # Warm: let loader / agent manager / OmniGraphs come up.
    warm_end = time.time() + 12.0
    while time.time() < warm_end and simulation_app.is_running():
        node.world.step(render=True)

    lines: list[str] = []
    lines.append(f"robot={args.robot} world={args.world}")
    lines.append("--- topic list (after warm) ---")
    topics = _ros2("topic", "list")
    lines.append(topics)

    interesting = [
        "/clock",
        "/scan",
        "/front_2d_lidar/scan",
        "/back_2d_lidar/scan",
        "/front_3d_lidar/lidar_points",
        "/chassis/odom",
        "/odom",
        "/tf",
        "/tf_static",
        "/cmd_vel",
        "/joint_states",
    ]
    lines.append("--- topic type / echo sample (best_effort) ---")
    topic_set = set(topics.split())
    for t in interesting:
        if t not in topic_set:
            lines.append(f"{t} MISSING")
            continue
        typ = _ros2("topic", "type", t, timeout=5.0).strip()
        lines.append(f"{t} type={typ}")
        echo = _echo_once(t, timeout=8.0)
        lines.append("\n".join(echo.splitlines()[:16]))
        lines.append("---")

    lines.append("--- node list ---")
    lines.append(_ros2("node", "list"))

    # Drive via wrapper state (rclpy not spun; set cmd_* directly).
    node.cmd_lin = 0.35
    node.cmd_ang = 0.0
    t0 = time.time()
    while time.time() - t0 < args.seconds and simulation_app.is_running():
        if node._chassis_drive:
            dt = node.world.get_physics_dt()
            node.robot.apply_cmd_vel(node.cmd_lin, node.cmd_ang, dt)
        else:
            wheel_action = node.diff_controller.forward([node.cmd_lin, node.cmd_ang])
            node.robot.apply_wheel_actions(wheel_action)
        node.world.step(render=True)

    lines.append("--- topic list (end) ---")
    topics_end = _ros2("topic", "list")
    lines.append(topics_end)

    flat = topics_end if topics_end.strip() and not topics_end.startswith("ERROR") else topics
    has_clock = "/clock" in flat
    has_scan_2d = any(
        x in flat for x in ("/scan", "/front_2d_lidar/scan", "/back_2d_lidar/scan")
    )
    has_lidar3d = "/front_3d_lidar/lidar_points" in flat
    has_odom = any(x in flat for x in ("/odom", "/chassis/odom"))
    has_tf = "/tf" in flat
    # Isaac 6.0 Nova_Carter_ROS exposes 3D lidar; 2D /scan may need pointcloud_to_laserscan.
    has_ranging = has_scan_2d or has_lidar3d
    lines.append("--- GATES ---")
    lines.append(
        f"clock={has_clock} scan_2d={has_scan_2d} lidar3d={has_lidar3d} "
        f"odom={has_odom} tf={has_tf}"
    )
    verdict = (
        "PASS" if (has_clock and has_ranging and has_odom and has_tf) else "FAIL"
    )
    lines.append(f"VERDICT={verdict}")

    text = "\n".join(lines) + "\n"
    with open(args.summary, "w", encoding="utf-8") as fh:
        fh.write(text)
    print(text)
    print(f"[nav2_sensor] wrote {args.summary}")

    try:
        node.hunav.close_hunav_nodes()
    except Exception:
        pass
    simulation_app.close()
    return 0 if verdict == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
