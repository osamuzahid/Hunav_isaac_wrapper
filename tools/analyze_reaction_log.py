#!/usr/bin/env python3
"""Summarise HUNAV_REACTION_LOG CSV and exit non-zero if reactions look broken.

Usage:
  python3 tools/analyze_reaction_log.py /tmp/hunav_reaction.csv
"""

from __future__ import annotations

import csv
import sys
from collections import defaultdict


BEH = {
    1: "Regular",
    2: "Impassive",
    3: "Surprised",
    4: "Scared",
    5: "Curious",
    6: "Threatening",
}

# Pass gates when agent spent time near the robot (< 3.5 m).
NEAR_M = 4.0  # match Scared detect dist; successful flee may never go <3.5

# Pass gates when agent spent time near the robot.
NEAR_YAW90_MAX = {
    2: 8,   # Impassive
    3: 15,  # Surprised
    4: 15,  # Scared
    5: 10,  # Curious
    6: 15,  # Threatening
}
NEAR_REV_PCT_MAX = {
    2: 30.0,
    3: 35.0,
    4: 45.0,
    5: 25.0,
    6: 40.0,
}
REQUIRED_NEAR = (2, 4, 5, 6)


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/hunav_reaction.csv"
    rows_by_id = defaultdict(list)
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            rows_by_id[int(row["agent_id"])].append(row)

    if not rows_by_id:
        print(f"FAIL: no rows in {path}")
        return 2

    print(f"log={path}")
    print(
        f"{'id':>3} {'beh':<11} {'n':>5} {'near':>5} {'min_d':>6} {'yaw>90':>6} "
        f"{'rev%':>5} {'near_spd':>8} {'d_late':>6} {'verdict':<6}"
    )

    failures = []
    saw_near = {b: False for b in REQUIRED_NEAR}

    for aid in sorted(rows_by_id):
        rows = rows_by_id[aid]
        beh = int(float(rows[0]["beh"]))
        near_rows = [r for r in rows if 0.0 <= float(r["dist_robot"]) < NEAR_M]
        dists_all = [float(r["dist_robot"]) for r in rows if float(r["dist_robot"]) >= 0]
        min_d = min(dists_all) if dists_all else -1.0

        yaw90_near = sum(1 for r in near_rows if float(r["yaw_jump_deg"]) > 90.0)
        rev = 0
        near_speeds = []
        prev = None
        for r in near_rows:
            dx, dy = float(r["dx"]), float(r["dy"])
            near_speeds.append(float(r["speed_xy"]))
            if prev is not None and (abs(prev[0]) + abs(prev[1]) > 1e-4):
                if dx * prev[0] + dy * prev[1] < 0.0:
                    rev += 1
            prev = (dx, dy)
        near_n = len(near_rows)
        rev_pct = (100.0 * rev / near_n) if near_n else 0.0
        mean_near_spd = sum(near_speeds) / near_n if near_n else 0.0

        mid = max(1, near_n // 2)
        d_late = (
            sum(float(r["dist_robot"]) for r in near_rows[mid:]) / max(1, near_n - mid)
            if near_n
            else -1.0
        )
        d_early = (
            sum(float(r["dist_robot"]) for r in near_rows[:mid]) / mid if near_n else -1.0
        )

        verdict = "skip"
        if near_n >= 20:
            if beh in saw_near:
                saw_near[beh] = True
            verdict = "PASS"
            yaw_lim = NEAR_YAW90_MAX.get(beh, 15)
            rev_lim = NEAR_REV_PCT_MAX.get(beh, 35.0)
            reasons = []
            if yaw90_near > yaw_lim:
                reasons.append(f"yaw90={yaw90_near}>{yaw_lim}")
            if rev_pct > rev_lim:
                reasons.append(f"rev%={rev_pct:.1f}>{rev_lim}")
            # Curious should settle (low speed) near stop distance.
            if beh == 5 and mean_near_spd > 0.45 and d_late < 1.1:
                reasons.append(f"curious_bump spd={mean_near_spd:.2f} d={d_late:.2f}")
            # min_d uses 5th percentile so a single teleport frame can't FAIL.
            if near_n >= 20:
                near_ds = sorted(float(r["dist_robot"]) for r in near_rows)
                p5 = near_ds[max(0, int(0.05 * len(near_ds)))]
            else:
                p5 = min_d
            if beh == 5 and p5 < 1.0:
                reasons.append(f"curious_too_close p5={p5:.2f}")
            # Scared: late near-dist should grow or stay back (not collapse in).
            if beh == 4 and d_late + 0.2 < d_early and d_late < 2.5:
                reasons.append(f"scared_closing early={d_early:.2f} late={d_late:.2f}")
            if beh == 4 and mean_near_spd < 0.05 and d_late < 2.8:
                reasons.append(f"scared_frozen spd={mean_near_spd:.2f}")
            if beh == 4 and min_d < 1.2:
                reasons.append(f"scared_too_close min={min_d:.2f}")
            # Re-approach cycles: dist falls by >1 m after having risen (GUI loop).
            if beh == 4 and near_n >= 20:
                ds = [float(r["dist_robot"]) for r in near_rows]
                cycles = 0
                rising = False
                peak = ds[0]
                for d in ds[1:]:
                    if d > peak:
                        peak = d
                        rising = True
                    elif rising and d < peak - 1.0:
                        cycles += 1
                        rising = False
                        peak = d
                if cycles >= 2:
                    reasons.append(f"scared_reapproach_cycles={cycles}")
            # Curious approach weave
            if beh == 5 and rev_pct > 15.0:
                reasons.append(f"curious_weave rev%={rev_pct:.1f}")
            if reasons:
                verdict = "FAIL"
                failures.append(f"agent{aid}/{BEH.get(beh, beh)}: " + "; ".join(reasons))

        print(
            f"{aid:>3} {BEH.get(beh, str(beh)):<11} {len(rows):>5} {near_n:>5} "
            f"{min_d:>6.2f} {yaw90_near:>6} {rev_pct:>5.1f} {mean_near_spd:>8.3f} "
            f"{d_late:>6.2f} {verdict:<6}"
        )

    print()
    missing = [BEH[b] for b, ok in saw_near.items() if not ok]
    if missing:
        msg = "insufficient near samples for: " + ", ".join(missing)
        print("FAIL:", msg)
        failures.append(msg)

    if failures:
        print("VERDICT FAIL")
        for f in failures:
            print(" -", f)
        return 1

    print("VERDICT PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
