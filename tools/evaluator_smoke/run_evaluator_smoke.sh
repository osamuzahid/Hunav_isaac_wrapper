#!/usr/bin/env bash
# End-to-end hunav_evaluator metrics CSV against a running Isaac keepalive.
#
# Prerequisites (separate terminal / process):
#   Isaac keepalive publishing /human_states + /robot_states via hunav_agent_manager
#   e.g. ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 240 \
#          --world museum --config museum_agents --disable-cameras
#
# Env:
#   OUT_DIR  RESULT_BASENAME  RECORD_SECS  GOAL_X GOAL_Y  EXP_TAG  RUN_ID
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
source /home/osamuzahid/Projects/isaac-social-nav/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT="${OUT_DIR:-/tmp/hunav_eval}"
mkdir -p "$OUT"
RESULT_BASENAME="${RESULT_BASENAME:-metrics}"
RESULT_FILE="$OUT/$RESULT_BASENAME"
RECORD_SECS="${RECORD_SECS:-45}"
GOAL_X="${GOAL_X:-2.0}"
GOAL_Y="${GOAL_Y:--6.5}"
EXP_TAG="${EXP_TAG:-museum_eval_smoke}"
RUN_ID="${RUN_ID:-1}"
SUMMARY="$OUT/evaluator_smoke_summary.txt"
PARAMS="${PARAMS:-$SMOKE_DIR/metrics_smoke.yaml}"

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$SMOKE_DIR/../nav2_smoke/fastrtps_no_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

: >"$SUMMARY"
log() { echo "$@" | tee -a "$SUMMARY"; }

log "out=$OUT result_file=$RESULT_FILE record=${RECORD_SECS}s goal=($GOAL_X,$GOAL_Y) tag=$EXP_TAG"

wait_topic() {
  local t="$1"
  local n="${2:-60}"
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
wait_topic /robot_states 30 || { ros2 topic list | tee -a "$SUMMARY" || true; exit 2; }

log "=== start hunav_evaluator_node (system Python) ==="
# Ensure pandas/numpy come from system, not Isaac.
python3 - <<'PY' | tee -a "$SUMMARY"
import pandas, numpy
print(f"pandas={pandas.__version__} numpy={numpy.__version__}")
PY

ros2 run hunav_evaluator hunav_evaluator_node --ros-args \
  --params-file "$PARAMS" \
  -p "result_file:=$RESULT_FILE" \
  >"$OUT/evaluator_node.log" 2>&1 &
EVAL_PID=$!
trap 'kill "$EVAL_PID" 2>/dev/null || true' EXIT

for i in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q '/hunav_start_recording'; then
    log "recording services ready (attempt $i)"
    break
  fi
  sleep 1
  if ! kill -0 "$EVAL_PID" 2>/dev/null; then
    log "FAIL: evaluator exited early"
    tail -n 80 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
    exit 3
  fi
  if [[ "$i" -eq 30 ]]; then
    log "FAIL: hunav_start_recording not advertised"
    tail -n 80 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
    exit 3
  fi
done

log "=== start recording ==="
# CLI request: robot_goal + experiment_tag + run_id
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: '$EXP_TAG', run_id: $RUN_ID, robot_goal: {header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}}" \
  | tee -a "$SUMMARY"

log "=== nudge robot on /cmd_vel for ${RECORD_SECS}s ==="
# Short forward/turn so path_length / speeds are non-trivial (teleop keepalive listens).
(
  end=$((SECONDS + RECORD_SECS))
  while (( SECONDS < end )); do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}" \
      >/dev/null 2>&1 || true
    sleep 0.5
  done
) &
DRIVE_PID=$!
wait "$DRIVE_PID" || true

log "=== stop recording (compute + write CSV) ==="
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}" | tee -a "$SUMMARY"

# Give pandas a moment to flush
sleep 3

MAIN_CSV="${RESULT_FILE}.csv"
if [[ ! -f "$MAIN_CSV" ]]; then
  # result_file may already include .csv
  MAIN_CSV="$RESULT_FILE"
fi

log "=== results ==="
if [[ -f "${RESULT_FILE}.csv" ]]; then
  MAIN_CSV="${RESULT_FILE}.csv"
elif [[ -f "$RESULT_FILE" && "$RESULT_FILE" == *.csv ]]; then
  MAIN_CSV="$RESULT_FILE"
else
  log "FAIL: no metrics CSV at ${RESULT_FILE}.csv"
  ls -la "$OUT" | tee -a "$SUMMARY" || true
  tail -n 100 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
  exit 4
fi

log "PASS: $MAIN_CSV"
wc -l "$MAIN_CSV" | tee -a "$SUMMARY"
head -n 5 "$MAIN_CSV" | tee -a "$SUMMARY"
ls -la "$OUT"/*"${EXP_TAG}"* "$OUT"/*metrics* 2>/dev/null | tee -a "$SUMMARY" || true
log "evaluator_node.log tail:"
tail -n 40 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
log "VERDICT PASS"
exit 0
