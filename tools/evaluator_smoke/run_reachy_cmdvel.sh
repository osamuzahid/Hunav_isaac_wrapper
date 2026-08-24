#!/usr/bin/env bash
# Scripted /cmd_vel hop for Reachy in the CUCR hospital N–S hall.
# No Nav2. No evaluator (that's B5). Occupancy A* (5,0) → (5,-8).
#
# Prerequisites: Isaac already up with --robot reachy --world hospital
#   --config hospital_lab_park (spawn (5,0) yaw 2.9).
#
# Env: OUT_DIR
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
source /home/osamuzahid/Projects/isaac-social-nav/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SMOKE_DIR/../.." && pwd)"
OUT="${OUT_DIR:-/tmp/hunav_reachy_cmdvel}"
mkdir -p "$OUT"
SUMMARY="$OUT/cmdvel_summary.txt"

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$SMOKE_DIR/../nav2_smoke/fastrtps_no_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

: >"$SUMMARY"
log() { echo "$@" | tee -a "$SUMMARY"; }

log "out=$OUT (Reachy occupancy /cmd_vel, no Nav2, no evaluator)"

wait_topic() {
  local t="$1"
  local n="${2:-90}"
  for i in $(seq 1 "$n"); do
    if timeout 3 ros2 topic echo "$t" --once >/dev/null 2>&1; then
      log "topic ok $t (attempt $i)"
      return 0
    fi
    sleep 2
  done
  log "FAIL: no data on $t"
  return 1
}

log "=== wait for Reachy state ==="
wait_topic /robot_states 90 || { ros2 topic list | tee -a "$SUMMARY" || true; exit 2; }
wait_topic /scan 30 || log "WARN: no /scan yet (RViz may still catch up)"

pkill -f teleop_twist_keyboard 2>/dev/null || true
pkill -f "topic pub .*/cmd_vel" 2>/dev/null || true
sleep 1

log "=== waypoint /cmd_vel: reception (5,0) → south hall (5,-8) ==="
set +e
python3 "$ROOT/tools/drive_reachy_waypoints.py" | tee -a "$SUMMARY"
DRIVE_RC=${PIPESTATUS[0]}
set -e

pkill -f drive_reachy_waypoints.py 2>/dev/null || true
pkill -f "topic pub .*/cmd_vel" 2>/dev/null || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null || true

if [[ "$DRIVE_RC" -ne 0 ]]; then
  log "FAIL: driver exit $DRIVE_RC (zeros still sent)"
  exit "$DRIVE_RC"
fi
log "PASS: driver finished; zeros sent on /cmd_vel"
exit 0
