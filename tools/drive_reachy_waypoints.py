#!/usr/bin/env python3
"""Drive Reachy a short hospital corridor hop via /cmd_vel.

Spawn is hospital_behaviors / hospital_lab_park: (5.0, 0.0) yaw 2.9 (facing
west into the nurses desk). Occupancy at inflation 0.50 m is a N–S hall at
x≈5; west of spawn is occupied. Do **not** copy the museum plaza hop
(2,-8)→(1.5, 6.5). Do **not** closed-loop on Agent.yaw — chassis quaternion.

Kinematic Reachy (Physics=none) does not collide with walls; occupancy is
the keep-out. A west-heading /cmd_vel through the desk is the #77 leftover.
"""

from __future__ import annotations

import math
import os
import sys
import time

START = (5.0, 0.0)
GOAL = (5.0, -8.0)
# Straight N–S hall if A* cannot load the map.
FALLBACK = (
    (5.0, 0.0),
    (5.0, -2.0),
    (5.0, -4.0),
    (5.0, -6.0),
    (5.0, -8.0),
)
INFLATION_M = 0.50
SPACING_M = 1.2
LIN_MAX = 0.28
ANG_MAX = 0.50
ARRIVE_M = 0.70
GOAL_M = 0.90
TIMEOUT_S = 90.0
# hospital_lab_park / hospital_behaviors robot_init_pose.h
YAW_SPAWN = 2.9
# Nurses desk / reception west of the hall pinch at x≈5, y≈0.
DESK = (-2.0, 4.2, -1.5, 1.5)


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


def _in_desk(x: float, y: float) -> bool:
    x0, x1, y0, y1 = DESK
    return x0 <= x <= x1 and y0 <= y <= y1


def plan_corridor_waypoints() -> tuple[tuple[float, float], ...]:
    """A* south along the reception hall; fallback is the x≈5 polyline."""
    here = os.path.abspath(os.path.dirname(__file__))
    yaml_path = os.path.normpath(
        os.path.join(here, "..", "src", "maps", "hospital.yaml")
    )
    try:
        sys.path.insert(0, os.path.normpath(os.path.join(here, "..", "src")))
        from hunav_isaac_wrapper.occupancy_path import OccupancyMap

        occ = OccupancyMap.from_yaml(yaml_path, inflation_radius_m=INFLATION_M)
        path = occ.plan(START, GOAL, waypoint_spacing_m=SPACING_M)
    except Exception as exc:
        print(f"[drive_reachy] occupancy plan failed ({exc}); using fallback", flush=True)
        return FALLBACK
    if not path or len(path) < 3:
        print("[drive_reachy] occupancy plan empty; using fallback", flush=True)
        return FALLBACK
    hits = [(x, y) for x, y in path if _in_desk(x, y)]
    if hits:
        print(
            f"[drive_reachy] A* entered nurses desk {hits[0]}; using fallback",
            flush=True,
        )
        return FALLBACK
    wps = tuple((float(x), float(y)) for x, y in path[1:])  # skip spawn
    print(
        f"[drive_reachy] A* {len(wps)} wp inflation={INFLATION_M} "
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
            super().__init__("hospital_reachy_cmdvel_driver")
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
                f"corridor hop n={len(wps)} last={wps[-1]} "
                "(quat yaw, not Agent.yaw; hall x≈5, not west into desk)"
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
                    "timeout — stopping (did not reach south hall (5, -8))"
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
                    if abs(_wrap(self._quat_yaw - self._motion_yaw)) > 1.0:
                        if not self._use_motion:
                            self.get_logger().warn(
                                "quat yaw disagrees with motion — steering from XY"
                            )
                        self._use_motion = True
            self._prev = (x, y)

            last = self._wp_i >= len(self._wps) - 1
            if self._wp_i >= len(self._wps):
                self.get_logger().info(
                    f"arrived south hall at ({x:.2f},{y:.2f})"
                )
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
                        f"arrived south hall at ({x:.2f},{y:.2f})"
                    )
                    self._stop()
                return

            heading = self._heading()
            want = math.atan2(gy - y, gx - x)
            err = _wrap(want - heading)
            cmd = Twist()
            # Creep on large heading error so we do not drive into the desk
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
                    f"{' DESK' if _in_desk(x, y) else ''}"
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
        wps = plan_corridor_waypoints()
        for i, (x, y) in enumerate(wps):
            flag = " DESK" if _in_desk(x, y) else ""
            print(f"  {i:2d}  {x:7.2f} {y:7.2f}{flag}")
        return 1 if any(_in_desk(x, y) for x, y in wps) else 0
    return _run_ros(plan_corridor_waypoints())


if __name__ == "__main__":
    sys.exit(main())
