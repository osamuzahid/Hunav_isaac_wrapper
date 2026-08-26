#!/usr/bin/env bash
# Hospital Nav2 smoke on kinematic Reachy (native /scan + /odom, no Carter).
#
# Requires a running Isaac keepalive, e.g.:
#   ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 600 \
#     --robot reachy --world hospital --config hospital_lab_park --disable-cameras
#
# Same south-hall goal as occupancy #78: (5,0) → (5,-8). Not museum A10.
# Not Carter (10,-17). Live /scan should see USD stretchers (occupancy A* will not).
# Quiet hospital_lab_park. Crowd hop uses hospital_eval_5 + run_reachy_nav2_eval.sh.
# Global planner: Smac 2D (nav2_reachy_params.yaml). NavFn kept the 8 m
# stretcher squeeze; Smac prefers the ~27 m wing.
set -eo pipefail
SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SMOKE_DIR/../.." && pwd)"

export MAP_YAML="${MAP_YAML:-$ROOT/src/maps/hospital.yaml}"
export PARAMS="${PARAMS:-$SMOKE_DIR/nav2_reachy_params.yaml}"
export INIT_X="${INIT_X:-5.0}"
export INIT_Y="${INIT_Y:-0.0}"
export INIT_YAW="${INIT_YAW:-2.9}"
export GOAL_X="${GOAL_X:-5.0}"
export GOAL_Y="${GOAL_Y:--8.0}"
export GOAL_TIMEOUT="${GOAL_TIMEOUT:-240}"
export OUT_DIR="${OUT_DIR:-/tmp/nav2_smoke_reachy_hospital}"
export ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
export SKIP_PC2="${SKIP_PC2:-1}"
export USE_LOCALIZATION="${USE_LOCALIZATION:-False}"
export RECORD_ODOM="${RECORD_ODOM:-1}"
# Optional wait after Nav2 is up (quiet #79 had none). Crowd still squeezed with 6 s.
export GOAL_SETTLE_S="${GOAL_SETTLE_S:-0}"
export DUMP_NAV2_GRID="${DUMP_NAV2_GRID:-1}"

exec "$SMOKE_DIR/run_nav2_smoke.sh"
