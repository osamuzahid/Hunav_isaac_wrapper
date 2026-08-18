#!/usr/bin/env python3
"""Drive Stretch from the museum_eval alcove to the A10 plaza via /cmd_vel.

The previous sparse hop (2,-3.5) → (-5.5, 0.5) kept a forward component while
turning west, so kinematic Stretch drove into the first doorway at ~(2, 1).
This driver plans occupancy A* through the west gap (x≈-4) and steers with
chassis quaternion yaw — not Agent.yaw (character-axis / HuNav convention).

Kinematic Stretch (Physics=none) does not collide with walls; occupancy is
the keep-out. Do not treat a doorway stop as plaza arrival.
"""

from __future__ import annotations

import math
import os
import sys
import time

START = (2.0, -8.0)
GOAL = (1.5, 6.5)
# West of the y≈3 partition if A* cannot load the map.
FALLBACK = (
    (2.0, -8.0),
    (-0.9, -4.7),
    (-4.0, -1.5),
    (-4.0, 3.0),
    (-3.6, 4.2),
    (0.2, 5.9),
    (1.5, 6.5),
)
INFLATION_M = 0.50
SPACING_M = 1.2
LIN_MAX = 0.28
ANG_MAX = 0.50
ARRIVE_M = 0.70
GOAL_M = 0.90
TIMEOUT_S = 130.0
# museum_eval robot_init_pose.h — +Y, same as ChassisDriveRobot spawn.
YAW_SPAWN = 1.57
# First-doorway box (cream floor / wood pocket). Path must not enter this.
DOORWAY = (-3.0, 5.0, 1.5, 3.8)


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_from_quat(q) -> float:
    """geometry_msgs quaternion → yaw. Wrapper stores Isaac wxyz into xyzw."""
    w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _in_doorway(x: float, y: float) -> bool:
    x0, x1, y0, y1 = DOORWAY
    return x0 <= x <= x1 and y0 <= y <= y1


def plan_plaza_waypoints() -> tuple[tuple[float, float], ...]:
    """A* from alcove to A10 plaza; fallback is the west-gap polyline."""
    here = os.path.abspath(os.path.dirname(__file__))
    yaml_path = os.path.normpath(os.path.join(here, "..", "src", "maps", "museum.yaml"))
    try:
        sys.path.insert(0, os.path.normpath(os.path.join(here, "..", "src")))
        from hunav_isaac_wrapper.occupancy_path import OccupancyMap

        occ = OccupancyMap.from_yaml(yaml_path, inflation_radius_m=INFLATION_M)
        path = occ.plan(START, GOAL, waypoint_spacing_m=SPACING_M)
    except Exception as exc:
        print(f"[drive_stretch] occupancy plan failed ({exc}); using fallback", flush=True)
        return FALLBACK
    if not path or len(path) < 3:
        print("[drive_stretch] occupancy plan empty; using fallback", flush=True)
        return FALLBACK
    hits = [(x, y) for x, y in path if _in_doorway(x, y)]
    if hits:
        print(
            f"[drive_stretch] A* entered doorway {hits[0]}; using fallback",
            flush=True,
        )
        return FALLBACK
    wps = tuple((float(x), float(y)) for x, y in path[1:])  # skip spawn
    print(
        f"[drive_stretch] A* {len(wps)} wp inflation={INFLATION_M} "
        f"→ ({wps[-1][0]:.2f},{wps[-1][1]:.2f})",
        flush=True,
    )
    return wps


def _run_ros(waypoints: tuple[tuple[float, float], ...]) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from hunav_msgs.msg import Agent
    from rclpy.node import Node

    class WaypointDriver(Node):
        def __init__(self, wps: tuple[tuple[float, float], ...]) -> None:
            super().__init__("museum_eval_cmdvel_driver")
            self._wps = wps
            self._xy: tuple[float, float] | None = None
            self._quat_yaw = YAW_SPAWN
            self._motion_yaw = YAW_SPAWN
            self._prev: tuple[float, float] | None = None
            self._use_motion = False
            self._wp_i = 0
            self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(Agent, "/robot_states", self._on_robot, 10)
            self.create_timer(0.1, self._tick)
            self._t0 = time.monotonic()
            self._last_log = 0.0
            self._done = False
            self.get_logger().info(
                f"plaza hop n={len(wps)} last={wps[-1]} "
                "(quat yaw, not Agent.yaw; A* west gap)"
            )

        def _on_robot(self, msg: Agent) -> None:
            self._xy = (
                float(msg.position.position.x),
                float(msg.position.position.y),
            )
            self._quat_yaw = _yaw_from_quat(msg.position.orientation)

        def _stop(self) -> None:
            self._pub.publish(Twist())
            self._done = True

        def _heading(self) -> float:
            if self._use_motion:
                return self._motion_yaw
            return self._quat_yaw

        def _tick(self) -> None:
            if self._done:
                return
            now = time.monotonic()
            if now - self._t0 > TIMEOUT_S:
                self.get_logger().error(
                    "timeout — stopping (did not reach A10 plaza)"
                )
                self._stop()
                return
            if self._xy is None:
                return
            x, y = self._xy
            if self._prev is not None:
                ddx = x - self._prev[0]
                ddy = y - self._prev[1]
                if math.hypot(ddx, ddy) > 0.04:
                    self._motion_yaw = math.atan2(ddy, ddx)
                    # If quat yaw is ~π/2 off (old Agent.yaw failure), use motion.
                    if abs(_wrap(self._quat_yaw - self._motion_yaw)) > 1.0:
                        if not self._use_motion:
                            self.get_logger().warn(
                                "quat yaw disagrees with motion — steering from XY"
                            )
                        self._use_motion = True
            self._prev = (x, y)

            last = self._wp_i >= len(self._wps) - 1
            if self._wp_i >= len(self._wps):
                self.get_logger().info(f"arrived A10 plaza at ({x:.2f},{y:.2f})")
                self._stop()
                return
            gx, gy = self._wps[self._wp_i]
            dist = math.hypot(gx - x, gy - y)
            arrive = GOAL_M if last else ARRIVE_M
            if dist < arrive:
                self.get_logger().info(
                    f"reached wp{self._wp_i} ({gx:.1f},{gy:.1f}) "
                    f"at ({x:.2f},{y:.2f})"
                )
                self._wp_i += 1
                if self._wp_i >= len(self._wps):
                    self.get_logger().info(
                        f"arrived A10 plaza at ({x:.2f},{y:.2f})"
                    )
                    self._stop()
                return

            heading = self._heading()
            want = math.atan2(gy - y, gx - x)
            err = _wrap(want - heading)
            cmd = Twist()
            # Creep on large heading error so we do not drive into the doorway
            # while rotating, but never lin=0 (that froze motion-heading).
            if abs(err) > 0.70:
                cmd.linear.x = LIN_MAX * 0.25
            else:
                cmd.linear.x = LIN_MAX * max(
                    0.55, math.cos(max(-1.0, min(1.0, err)))
                )
            cmd.angular.z = max(-ANG_MAX, min(ANG_MAX, 1.6 * err))
            self._pub.publish(cmd)
            if now - self._last_log > 2.0:
                self._last_log = now
                self.get_logger().info(
                    f"wp{self._wp_i} pos=({x:.2f},{y:.2f}) dist={dist:.2f} "
                    f"quat={self._quat_yaw:.2f} motion={self._motion_yaw:.2f} "
                    f"err={err:.2f} lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}"
                    f"{' DOORWAY' if _in_doorway(x, y) else ''}"
                )

    rclpy.init()
    node = WaypointDriver(waypoints)
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    if "--plan-only" in sys.argv:
        wps = plan_plaza_waypoints()
        for i, (x, y) in enumerate(wps):
            flag = " DOORWAY" if _in_doorway(x, y) else ""
            print(f"  {i:2d}  {x:7.2f} {y:7.2f}{flag}")
        return 1 if any(_in_doorway(x, y) for x, y in wps) else 0
    return _run_ros(plan_plaza_waypoints())


if __name__ == "__main__":
    sys.exit(main())
