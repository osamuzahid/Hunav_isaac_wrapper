#!/usr/bin/env bash
# Museum Nav2 smoke on kinematic Stretch (native /scan + /odom, no Carter).
#
# Requires a running Isaac keepalive, e.g.:
#   ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 600 \
#     --robot stretch --world museum --config museum_eval_5 --disable-cameras
#
# Same goal as plaza /cmd_vel #68: (2,-8) → A10 plaza (1.5, 6.5). Nav2 must
# plan west of the y≈3 partition (not the first doorway at ~(2,1)).
# Global planner: Smac 2D (nav2_stretch_params.yaml).
# Records /odom and scores clearance vs the three staggered standing people
# (STATUE / STATUE2 / STATUE3). Look for OBSTACLE=AVOID.
set -eo pipefail
SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SMOKE_DIR/../.." && pwd)"

export MAP_YAML="${MAP_YAML:-$ROOT/src/maps/museum.yaml}"
export PARAMS="${PARAMS:-$SMOKE_DIR/nav2_stretch_params.yaml}"
export INIT_X="${INIT_X:-2.0}"
export INIT_Y="${INIT_Y:--8.0}"
export INIT_YAW="${INIT_YAW:-1.57}"
export GOAL_X="${GOAL_X:-1.5}"
export GOAL_Y="${GOAL_Y:-6.5}"
export GOAL_TIMEOUT="${GOAL_TIMEOUT:-240}"
export OUT_DIR="${OUT_DIR:-/tmp/nav2_smoke_stretch_museum}"
export ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
export SKIP_PC2="${SKIP_PC2:-1}"
export USE_LOCALIZATION="${USE_LOCALIZATION:-False}"
export RECORD_ODOM="${RECORD_ODOM:-1}"
export STATUE_X="${STATUE_X:--3.25}"
export STATUE_Y="${STATUE_Y:--2.10}"
export STATUE2_X="${STATUE2_X:--4.55}"
export STATUE2_Y="${STATUE2_Y:-0.50}"
export STATUE3_X="${STATUE3_X:--4.20}"
export STATUE3_Y="${STATUE3_Y:-2.90}"

exec "$SMOKE_DIR/run_nav2_smoke.sh"
