#!/usr/bin/env python3
"""Dump global costmap, /scan, TF, and first /plan at Nav2 goal-send.

Box counts were identical on every squeeze. This writes the pinch layout,
lethal cell XY, a scan snapshot, and the first path poses.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path


def _box_stats(cells, x0, x1, y0, y1):
    vals = [c for x, y, c in cells if x0 <= x <= x1 and y0 <= y <= y1]
    if not vals:
        return "n=0"
    n = len(vals)
    mx = max(vals)
    n254 = sum(1 for v in vals if v >= 254)
    n253 = sum(1 for v in vals if v >= 253)
    n128 = sum(1 for v in vals if v >= 128)
    n0 = sum(1 for v in vals if v == 0)
    n255 = sum(1 for v in vals if v == 255)
    return (
        f"n={n} max={mx} lethal(>=254)={n254} inscribed(>=253)={n253} "
        f"high(>=128)={n128} free0={n0} unknown255={n255}"
    )


def _char(c: int) -> str:
    if c >= 254:
        return "#"
    if c >= 253:
        return "I"
    if c >= 128:
        return "+"
    if c == 0:
        return "."
    if c == 255:
        return "?"
    return "o"


def snapshot(out_path: str, timeout_s: float) -> int:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav2_msgs.msg import Costmap
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    out = Path(out_path)
    d = out.parent
    d.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("dump_nav2_grid_snap")
    cost_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    scan_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    cost_got: list = []
    scan_got: list = []

    def cost_cb(msg: Costmap) -> None:
        if not cost_got:
            cost_got.append(msg)

    def scan_cb(msg: LaserScan) -> None:
        if not scan_got:
            scan_got.append(msg)

    node.create_subscription(Costmap, "/global_costmap/costmap_raw", cost_cb, cost_qos)
    node.create_subscription(LaserScan, "/scan", scan_cb, scan_qos)
    tf_buf = Buffer()
    TransformListener(tf_buf, node)

    t0 = time.time()
    while rclpy.ok() and not cost_got and (time.time() - t0) < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.1)
    extra = time.time() + 1.5
    while rclpy.ok() and time.time() < extra:
        rclpy.spin_once(node, timeout_sec=0.1)

    tfs: dict[str, str] = {}
    for frame in ("base_link", "lidar_link", "laser"):
        try:
            t: TransformStamped = tf_buf.lookup_transform("map", frame, rclpy.time.Time())
            p = t.transform.translation
            tfs[frame] = f"{frame} ({p.x:.3f},{p.y:.3f},{p.z:.3f})"
        except Exception as exc:
            tfs[frame] = f"{frame} FAIL {exc}"

    node.destroy_node()
    rclpy.shutdown()

    if not cost_got:
        out.write_text("FAIL: no /global_costmap/costmap_raw\n")
        print("[dump_nav2_grid] FAIL: no costmap_raw", flush=True)
        return 1

    msg = cost_got[0]
    md = msg.metadata
    res = float(md.resolution)
    ox = float(md.origin.position.x)
    oy = float(md.origin.position.y)
    w, h = int(md.size_x), int(md.size_y)
    data = list(msg.data)
    cells = []
    for row in range(h):
        for col in range(w):
            x = ox + (col + 0.5) * res
            y = oy + (row + 0.5) * res
            cells.append((x, y, int(data[row * w + col])))

    pinch = [(x, y, c) for x, y, c in cells if 4.0 <= x <= 6.2 and -6.2 <= y <= -0.8]
    lethal = [(x, y, c) for x, y, c in pinch if c >= 254]
    inscribed = [(x, y, c) for x, y, c in pinch if c == 253]

    span_lines = []
    y = -1.0
    while y >= -6.0 - 1e-9:
        row = [(x, c) for x, yy, c in pinch if abs(yy - y) < res * 0.51]
        free = sorted(x for x, c in row if c == 0)
        hot = sorted(x for x, c in row if c >= 253)
        if free:
            span = f"free {min(free):.2f}..{max(free):.2f} w={max(free)-min(free)+res:.2f} n={len(free)}"
        else:
            span = "free NONE"
        span_lines.append(
            f"y={y:5.1f} {span}  insc/lethal_x={[round(x,2) for x in hot]}"
        )
        y = round(y - 0.5, 1)

    ascii_rows = []
    xs_set = sorted({round(x, 2) for x, _, _ in pinch})
    ys_set = sorted({round(y, 2) for _, y, _ in pinch}, reverse=True)
    lookup = {(round(x, 2), round(y, 2)): c for x, y, c in pinch}
    hdr = "y\\x " + "".join(f"{x:4.1f}"[-2:] for x in xs_set[::2])
    ascii_rows.append("pinch ASCII 0.1 m (. free  o mid  + high  I inscribed  # lethal)")
    ascii_rows.append("x from 4.0 to 6.2, y from -0.8 down to -6.2")
    for yv in ys_set:
        line = f"{yv:5.1f} "
        for xv in xs_set:
            line += _char(lookup.get((xv, yv), 255))
        ascii_rows.append(line)

    (d / "pinch_costs.csv").write_text(
        "x,y,cost\n" + "\n".join(f"{x:.3f},{y:.3f},{c}" for x, y, c in pinch) + "\n"
    )
    (d / "lethal_xy.csv").write_text(
        "x,y,cost\n" + "\n".join(f"{x:.3f},{y:.3f},{c}" for x, y, c in lethal) + "\n"
    )
    (d / "pinch_ascii.txt").write_text("\n".join(ascii_rows) + "\n")

    scan_lines = ["no /scan in snapshot window"]
    if scan_got:
        sc = scan_got[0]
        rngs = [float(r) for r in sc.ranges if math.isfinite(r)]
        close = [r for r in rngs if r < 6.0]
        southish = []
        a = float(sc.angle_min)
        inc = float(sc.angle_increment)
        for i, r in enumerate(sc.ranges):
            if not math.isfinite(r) or r > 8.0:
                a += inc
                continue
            ang = a
            # laser frame: 0 = forward. yaw 2.9 ≈ south-west; dump raw close beams.
            if r < 6.0:
                southish.append((ang, r))
            a += inc
        scan_lines = [
            f"n_beams={len(sc.ranges)} angle=[{sc.angle_min:.3f},{sc.angle_max:.3f}]",
            f"range_minmax finite=({min(rngs) if rngs else 'na'}, {max(rngs) if rngs else 'na'})",
            f"n_range_lt_6m={len(close)} n_range_lt_3m={sum(1 for r in rngs if r < 3.0)}",
            f"frame_id={sc.header.frame_id}",
        ]

    (d / "tf_at_goal.txt").write_text("\n".join(tfs.values()) + "\n")
    (d / "scan_at_goal.txt").write_text("\n".join(scan_lines) + "\n")

    lines = [
        f"source=/global_costmap/costmap_raw layer={md.layer!r}",
        f"size={w}x{h} res={res:.4f} origin=({ox:.3f},{oy:.3f})",
        "costs: 0 free, 253 inscribed, 254 lethal, 255 unknown",
        f"hall_pinch x[4.2,5.8] y[-6.0,-1.0]  {_box_stats(cells, 4.2, 5.8, -6.0, -1.0)}",
        f"spawn      x[4.5,5.5] y[-0.5,0.5]   {_box_stats(cells, 4.5, 5.5, -0.5, 0.5)}",
        f"west_wing  x[-4.8,-3.8] y[-1.0,1.0] {_box_stats(cells, -4.8, -3.8, -1.0, 1.0)}",
        f"pinch_lethal_n={len(lethal)} pinch_inscribed_n={len(inscribed)}",
        "tf: " + " | ".join(tfs.values()),
        "",
        "free-span along hall (0.5 m y steps):",
        *span_lines,
        "",
        "also: pinch_ascii.txt pinch_costs.csv lethal_xy.csv scan_at_goal.txt tf_at_goal.txt",
        "",
    ]
    out.write_text("\n".join(lines) + "\n")
    print(f"[dump_nav2_grid] wrote {out}", flush=True)
    print("\n".join(lines[:12]), flush=True)
    return 0


def watch_plan(out_path: str, n_paths: int) -> int:
    import rclpy
    from nav_msgs.msg import Path as NavPath
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("dump_nav2_grid_plan")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    rows = ["seq,n_poses,length_m,min_x,max_x,min_y,max_y,class"]
    count = [0]
    stop = [False]
    pose_dir = Path(out_path).parent

    def classify(min_x: float, length: float) -> str:
        if min_x < 0.0 and length > 16.0:
            return "WING"
        if min_x > 3.5 and length < 16.0:
            return "HALL"
        return "OTHER"

    def cb(msg: NavPath) -> None:
        if count[0] >= n_paths:
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        length = 0.0
        for i in range(1, len(pts)):
            length += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs), max(xs)
        count[0] += 1
        cls = classify(min_x, length)
        rows.append(
            f"{count[0]},{len(pts)},{length:.3f},{min_x:.3f},{max_x:.3f},"
            f"{min(ys):.3f},{max(ys):.3f},{cls}"
        )
        Path(out_path).write_text("\n".join(rows) + "\n")
        if count[0] == 1:
            (pose_dir / "plan_poses.csv").write_text(
                "i,x,y\n"
                + "\n".join(f"{i},{x:.4f},{y:.4f}" for i, (x, y) in enumerate(pts))
                + "\n"
            )
        print(f"[dump_nav2_grid] plan {count[0]} {rows[-1]}", flush=True)
        if count[0] >= n_paths:
            stop[0] = True

    node.create_subscription(NavPath, "/plan", cb, qos)

    def _stop(*_a):
        stop[0] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    while rclpy.ok() and not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not Path(out_path).is_file():
        Path(out_path).write_text("\n".join(rows) + "\n# no /plan received\n")
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--snapshot", metavar="FILE")
    ap.add_argument("--watch-plan", metavar="FILE")
    ap.add_argument("--timeout", type=float, default=12.0)
    ap.add_argument("--n-paths", type=int, default=5)
    args = ap.parse_args()
    if args.snapshot:
        return snapshot(args.snapshot, args.timeout)
    if args.watch_plan:
        return watch_plan(args.watch_plan, args.n_paths)
    ap.error("need --snapshot FILE or --watch-plan FILE")
    return 2


if __name__ == "__main__":
    sys.exit(main())
