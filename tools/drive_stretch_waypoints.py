#!/usr/bin/env python3
"""Drive Stretch along museum_eval waypoints via /cmd_vel.

Do not closed-loop on Agent.yaw — kinematic Stretch's reported yaw does not
match chassis heading, which produced in-place spinning. Heading is estimated
from XY motion; always keep a forward component so that estimate can update.

Route: south alcove (2,-8) → cream floor → west of partition → A10 plaza (1.5, 6.5).
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from hunav_msgs.msg import Agent
from rclpy.node import Node

# Out of the south alcove onto the cream floor, around the y≈3 partition,
# into the open plaza by A10 (not the west-doorway spin at (2, 1)).
WAYPOINTS = (
    (2.0, -3.5),
    (-5.5, 0.5),
    (-5.5, 6.0),
    (1.5, 6.5),
)
LIN_MAX = 0.30
ANG_MAX = 0.45
ARRIVE_M = 0.90
TIMEOUT_S = 110.0
# museum_eval robot_init_pose.h — used only until the first 0.2 m of motion.
YAW_SPAWN = 1.57


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WaypointDriver(Node):
    def __init__(self) -> None:
        super().__init__("museum_eval_cmdvel_driver")
        self._xy: tuple[float, float] | None = None
        self._prev: tuple[float, float] | None = None
        self._heading = YAW_SPAWN
        self._wp_i = 0
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Agent, "/robot_states", self._on_robot, 10)
        self.create_timer(0.1, self._tick)
        self._t0 = time.monotonic()
        self._last_log = 0.0
        self._done = False
        self.get_logger().info(f"waypoints={WAYPOINTS} (heading from motion, not yaw)")

    def _on_robot(self, msg: Agent) -> None:
        self._xy = (
            float(msg.position.position.x),
            float(msg.position.position.y),
        )

    def _stop(self) -> None:
        self._pub.publish(Twist())
        self._done = True

    def _tick(self) -> None:
        if self._done:
            return
        now = time.monotonic()
        if now - self._t0 > TIMEOUT_S:
            self.get_logger().error("timeout — stopping")
            self._stop()
            return
        if self._xy is None:
            return
        x, y = self._xy
        if self._prev is not None:
            ddx = x - self._prev[0]
            ddy = y - self._prev[1]
            if math.hypot(ddx, ddy) > 0.04:
                self._heading = math.atan2(ddy, ddx)
        self._prev = (x, y)

        if self._wp_i >= len(WAYPOINTS):
            self.get_logger().info(f"arrived last wp at ({x:.2f},{y:.2f})")
            self._stop()
            return
        gx, gy = WAYPOINTS[self._wp_i]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < ARRIVE_M:
            self.get_logger().info(
                f"reached wp{self._wp_i} ({gx:.1f},{gy:.1f}) at ({x:.2f},{y:.2f})"
            )
            self._wp_i += 1
            self._pub.publish(Twist())
            return

        want = math.atan2(dy, dx)
        err = _wrap(want - self._heading)
        cmd = Twist()
        # Always roll: turn-in-place froze heading-from-motion and spun forever.
        cmd.linear.x = LIN_MAX * max(0.40, math.cos(max(-1.0, min(1.0, err))))
        cmd.angular.z = max(-ANG_MAX, min(ANG_MAX, 1.4 * err))
        self._pub.publish(cmd)
        if now - self._last_log > 2.0:
            self._last_log = now
            self.get_logger().info(
                f"wp{self._wp_i} pos=({x:.2f},{y:.2f}) dist={dist:.2f} "
                f"heading={self._heading:.2f} err={err:.2f} "
                f"lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}"
            )


def main() -> int:
    rclpy.init()
    node = WaypointDriver()
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


if __name__ == "__main__":
    sys.exit(main())
