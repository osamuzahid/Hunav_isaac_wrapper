#!/usr/bin/env bash
# Museum Nav2 smoke on kinematic Stretch (native /scan + /odom, no Carter).
#
# Requires a running Isaac keepalive, e.g.:
#   ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 600 \
#     --robot stretch --world museum --config museum_agents --disable-cameras
#
# Same goal as plaza /cmd_vel #68: (2,-8) → A10 plaza (1.5, 6.5). Nav2 must
# plan west of the y≈3 partition (not the first doorway at ~(2,1)).
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
export GOAL_TIMEOUT="${GOAL_TIMEOUT:-180}"
export OUT_DIR="${OUT_DIR:-/tmp/nav2_smoke_stretch_museum}"
export ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
export SKIP_PC2="${SKIP_PC2:-1}"
export USE_LOCALIZATION="${USE_LOCALIZATION:-False}"

exec "$SMOKE_DIR/run_nav2_smoke.sh"
