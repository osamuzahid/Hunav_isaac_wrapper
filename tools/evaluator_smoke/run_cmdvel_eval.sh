#!/usr/bin/env bash
# Record hunav_evaluator while Stretch drives a scripted /cmd_vel hop through
# museum_eval (10 Impassive). No Nav2. Kinematic Stretch has no wall collision —
# occupancy A* goes west of the y≈3 partition to the A10 plaza (1.5, 6.5).
#
# Prerequisites: Isaac already up with --config museum_eval --robot stretch
#
# Env: OUT_DIR RECORD_SECS EXP_TAG RUN_ID
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
source /home/osamuzahid/Projects/isaac-social-nav/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SMOKE_DIR/../.." && pwd)"
OUT="${OUT_DIR:-/tmp/hunav_cmdvel_eval}"
mkdir -p "$OUT"
RESULT_FILE="$OUT/metrics"
RECORD_SECS="${RECORD_SECS:-120}"
EXP_TAG="${EXP_TAG:-museum_cmdvel_eval}"
RUN_ID="${RUN_ID:-1}"
SUMMARY="$OUT/cmdvel_eval_summary.txt"
PARAMS="${PARAMS:-$SMOKE_DIR/metrics_cmdvel.yaml}"

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$SMOKE_DIR/../nav2_smoke/fastrtps_no_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

: >"$SUMMARY"
log() { echo "$@" | tee -a "$SUMMARY"; }

log "out=$OUT record=${RECORD_SECS}s tag=$EXP_TAG (waypoint /cmd_vel into north hall, no Nav2)"

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

log "=== wait for HuNav state topics ==="
wait_topic /human_states 90 || { ros2 topic list | tee -a "$SUMMARY" || true; exit 2; }
wait_topic /robot_states 30 || { ros2 topic list | tee -a "$SUMMARY" || true; exit 3; }

pkill -f hunav_evaluator_node 2>/dev/null || true
pkill -f teleop_twist_keyboard 2>/dev/null || true
sleep 1

log "=== start hunav_evaluator_node ==="
ros2 run hunav_evaluator hunav_evaluator_node --ros-args \
  --params-file "$PARAMS" \
  -p "result_file:=$RESULT_FILE" \
  >"$OUT/evaluator_node.log" 2>&1 &
EVAL_PID=$!
trap 'kill "$EVAL_PID" 2>/dev/null || true; pkill -f "topic pub .*/cmd_vel" 2>/dev/null || true' EXIT

for i in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q '/hunav_start_recording'; then
    log "recording services ready (attempt $i)"
    break
  fi
  sleep 1
  if ! kill -0 "$EVAL_PID" 2>/dev/null; then
    log "FAIL: evaluator exited early"
    tail -n 80 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
    exit 4
  fi
  if [[ "$i" -eq 30 ]]; then
    log "FAIL: hunav_start_recording not advertised"
    tail -n 80 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
    exit 4
  fi
done

log "=== start recording ==="
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: '$EXP_TAG', run_id: $RUN_ID, robot_goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 6.5, z: 0.0}, orientation: {w: 1.0}}}}" \
  | tee -a "$SUMMARY"

log "=== waypoint /cmd_vel: alcove → west gap → A10 plaza (1.5, 6.5) ==="
python3 "$ROOT/tools/drive_stretch_waypoints.py" | tee -a "$SUMMARY" &
DRIVE_PID=$!

log "=== wait ${RECORD_SECS}s (drive ~30 m then idle; people still circulating) ==="
sleep "$RECORD_SECS"
wait "$DRIVE_PID" 2>/dev/null || true
pkill -f drive_stretch_waypoints.py 2>/dev/null || true
pkill -f "topic pub .*/cmd_vel" 2>/dev/null || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null || true

log "=== stop recording ==="
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}" | tee -a "$SUMMARY"
sleep 3

MAIN_CSV="${RESULT_FILE}.csv"
if [[ ! -f "$MAIN_CSV" ]]; then
  log "FAIL: no metrics CSV at $MAIN_CSV"
  ls -la "$OUT" | tee -a "$SUMMARY" || true
  tail -n 80 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
  exit 5
fi

log "PASS: $MAIN_CSV"
python3 "$ROOT/tools/summarize_cmdvel_eval.py" "$OUT" | tee -a "$SUMMARY"
exit 0
