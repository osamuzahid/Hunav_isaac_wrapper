#!/usr/bin/env bash
# Hospital Nav2 smoke (carter_ROS) — short free-space goal near hospital_agents init.
#
# Requires a running Isaac keepalive, e.g.:
#   ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 400 \
#     --world hospital --config hospital_agents --disable-cameras
#
# Then:
#   ./tools/nav2_smoke/run_hospital_nav2_smoke.sh
set -eo pipefail
SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SMOKE_DIR/../.." && pwd)"

export MAP_YAML="${MAP_YAML:-$ROOT/src/maps/hospital.yaml}"
export INIT_X="${INIT_X:-10.0}"
export INIT_Y="${INIT_Y:--20.0}"
export INIT_YAW="${INIT_YAW:-2.5}"
# ~2 m along yaw 2.5 (hospital_agents facing). Do NOT use (10,-17): CUCR map has
# furniture/wall cells near y≈-18.8 that inflation_radius 0.55 seals off.
export GOAL_X="${GOAL_X:-8.4}"
export GOAL_Y="${GOAL_Y:--18.8}"
export GOAL_TIMEOUT="${GOAL_TIMEOUT:-120}"
export OUT_DIR="${OUT_DIR:-/tmp/nav2_smoke_hospital}"

exec "$SMOKE_DIR/run_nav2_smoke.sh"
