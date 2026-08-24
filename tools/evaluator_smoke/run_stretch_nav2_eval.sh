#!/usr/bin/env bash
# Record hunav_evaluator during the Stretch Nav2 A10 crowd hop (#70).
# System ROS Python only. Isaac keepalive must already be up:
#   ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py --seconds 600 \
#     --robot stretch --world museum --config museum_eval_5 --disable-cameras
#
# Env: OUT_DIR EXP_TAG RUN_ID
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
source /home/osamuzahid/Projects/isaac-social-nav/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SMOKE_DIR/../.." && pwd)"
NAV2_DIR="$ROOT/tools/nav2_smoke"
OUT="${OUT_DIR:-/tmp/hunav_nav2_crowd_eval}"
mkdir -p "$OUT"
RESULT_FILE="$OUT/metrics"
EXP_TAG="${EXP_TAG:-museum_nav2_crowd}"
RUN_ID="${RUN_ID:-1}"
GOAL_X="${GOAL_X:-1.5}"
GOAL_Y="${GOAL_Y:-6.5}"
SUMMARY="$OUT/nav2_eval_summary.txt"
PARAMS="${PARAMS:-$SMOKE_DIR/metrics_nav2_crowd.yaml}"
DESK="${DESK_COPY:-$HOME/Desktop/hunav_nav2_crowd_eval}"

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$NAV2_DIR/fastrtps_no_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

: >"$SUMMARY"
log() { echo "$@" | tee -a "$SUMMARY"; }

log "out=$OUT tag=$EXP_TAG goal=($GOAL_X,$GOAL_Y) (Nav2 crowd hop + evaluator)"

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

python3 - <<'PY' | tee -a "$SUMMARY"
import pandas, numpy
print(f"pandas={pandas.__version__} numpy={numpy.__version__}")
PY

pkill -f hunav_evaluator_node 2>/dev/null || true
sleep 1

log "=== start hunav_evaluator_node ==="
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
  "{experiment_tag: '$EXP_TAG', run_id: $RUN_ID, robot_goal: {header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}}" \
  | tee -a "$SUMMARY"

log "=== Stretch Nav2 smoke (2,-8) → ($GOAL_X,$GOAL_Y) ==="
set +e
OUT_DIR="$OUT" "$NAV2_DIR/run_stretch_nav2_smoke.sh"
NAV_RC=$?
set -e
log "nav2_smoke exit=$NAV_RC"

log "=== stop recording ==="
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}" | tee -a "$SUMMARY" || true
sleep 3

MAIN_CSV="${RESULT_FILE}.csv"
if [[ ! -f "$MAIN_CSV" ]]; then
  log "FAIL: no metrics CSV at $MAIN_CSV"
  ls -la "$OUT" | tee -a "$SUMMARY" || true
  tail -n 80 "$OUT/evaluator_node.log" | tee -a "$SUMMARY" || true
  exit 5
fi

log "PASS: $MAIN_CSV"
set +e
python3 "$ROOT/tools/summarize_nav2_crowd_eval.py" "$OUT" | tee -a "$SUMMARY"
SUM_RC=${PIPESTATUS[0]}
set -e

mkdir -p "$DESK"
cp -f "$MAIN_CSV" "$DESK/metrics.csv" 2>/dev/null || true
cp -f "$SUMMARY" "$DESK/nav2_eval_summary.txt" 2>/dev/null || true
[[ -f "$OUT/nav2_smoke_summary.txt" ]] && cp -f "$OUT/nav2_smoke_summary.txt" "$DESK/" || true
log "copied CSV → $DESK/metrics.csv"
if [[ -x /home/osamuzahid/Projects/isaac-social-nav/tools/archive_eval.sh ]]; then
  /home/osamuzahid/Projects/isaac-social-nav/tools/archive_eval.sh "$OUT" "${ARCHIVE_TAG:-nav2_crowd}" | tee -a "$SUMMARY" || true
fi

if [[ "$NAV_RC" -eq 0 ]] && grep -q 'GOAL=SUCCEEDED' "$OUT/nav2_smoke_summary.txt" 2>/dev/null && [[ "$SUM_RC" -eq 0 ]]; then
  log "VERDICT PASS (nav2 SUCCEEDED + metrics.csv)"
  exit 0
fi
log "VERDICT PARTIAL (csv=$MAIN_CSV nav2_rc=$NAV_RC summariser_rc=$SUM_RC)"
exit 0
