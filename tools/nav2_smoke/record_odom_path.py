#!/usr/bin/env python3
"""Write /odom XY to CSV until SIGINT/SIGTERM (Nav2 hop trace)."""

from __future__ import annotations

import argparse
import signal
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/odom")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("record_odom_path")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=20,
    )
    fh = open(args.out, "w", encoding="utf-8")
    fh.write("t,x,y\n")
    fh.flush()
    n = [0]

    def cb(msg: Odometry) -> None:
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        fh.write(f"{t:.3f},{p.x:.4f},{p.y:.4f}\n")
        n[0] += 1
        if n[0] % 20 == 0:
            fh.flush()

    node.create_subscription(Odometry, args.topic, cb, qos)
    stop = [False]

    def _stop(*_a):
        stop[0] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    while rclpy.ok() and not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.2)
    fh.flush()
    fh.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f"[record_odom_path] wrote {n[0]} samples → {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
