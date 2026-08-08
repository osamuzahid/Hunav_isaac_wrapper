#!/usr/bin/env bash
# Museum Nav2 run with hunav_evaluator recording (system ROS shell).
#
# Default goal = proven museum smoke (2,-6.5) so short E2E succeeds under laptop
# load. Override GOAL_* for longer traverses when the machine is quiet.
#
# Env overrides: OUT_DIR GOAL_X GOAL_Y INIT_* GOAL_TIMEOUT KEEPALIVE_SECS
#                EXP_TAG RUN_ID WORLD CONFIG MAP_YAML
#
# Examples:
#   ./tools/evaluator_smoke/run_nav2_eval.sh
#   GOAL_X=2.0 GOAL_Y=-4.0 GOAL_TIMEOUT=180 ./tools/evaluator_smoke/run_nav2_eval.sh
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
source /home/osamuzahid/Projects/isaac-social-nav/ros2_ws/install/setup.bash

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
NAV2_DIR="$ROOT/tools/nav2_smoke"
OUT="${OUT_DIR:-/tmp/hunav_eval_nav2}"
mkdir -p "$OUT"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export OMNI_KIT_ACCEPT_EULA=YES
export HUNAV_ISAAC_PROFILE="${HUNAV_ISAAC_PROFILE:-debug}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$NAV2_DIR/fastrtps_no_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

KEEPALIVE_SECS="${KEEPALIVE_SECS:-300}"
WORLD="${WORLD:-museum}"
# Hospital presets when WORLD=hospital (override any single field via env).
if [[ "$WORLD" == "hospital" ]]; then
  CONFIG="${CONFIG:-hospital_agents}"
  INIT_X="${INIT_X:-10.0}"
  INIT_Y="${INIT_Y:--20.0}"
  INIT_YAW="${INIT_YAW:-2.5}"
  # Forward ~2 m (yaw 2.5). (10,-17) is blocked on CUCR map under Nav2 inflation.
  GOAL_X="${GOAL_X:-8.4}"
  GOAL_Y="${GOAL_Y:--18.8}"
  EXP_TAG="${EXP_TAG:-hospital_nav2_eval}"
  MAP_YAML="${MAP_YAML:-$ROOT/src/maps/hospital.yaml}"
else
  CONFIG="${CONFIG:-museum_agents}"
  INIT_X="${INIT_X:-2.0}"
  INIT_Y="${INIT_Y:--8.0}"
  INIT_YAW="${INIT_YAW:-1.57}"
  # Proven museum Nav2 smoke goal (Validated #38).
  GOAL_X="${GOAL_X:-2.0}"
  GOAL_Y="${GOAL_Y:--6.5}"
  EXP_TAG="${EXP_TAG:-museum_nav2_eval}"
  MAP_YAML="${MAP_YAML:-$ROOT/src/maps/museum.yaml}"
fi
GOAL_TIMEOUT="${GOAL_TIMEOUT:-120}"
RUN_ID="${RUN_ID:-1}"
RESULT_FILE="$OUT/metrics"
SUMMARY="$OUT/nav2_eval_summary.txt"
PARAMS="$SMOKE_DIR/metrics_smoke.yaml"

: >"$SUMMARY"
log() { echo "$@" | tee -a "$SUMMARY"; }

cleanup() {
  log "cleanup..."
  [[ -n "${EVAL_PID:-}" ]] && kill "$EVAL_PID" 2>/dev/null || true
  [[ -n "${KEEP_PID:-}" ]] && kill "$KEEP_PID" 2>/dev/null || true
  pkill -x hunav_evaluator_node 2>/dev/null || true
  pkill -x pointcloud_to_laserscan_node 2>/dev/null || true
  pkill -x bt_navigator 2>/dev/null || true
  pkill -x controller_server 2>/dev/null || true
  pkill -x planner_server 2>/dev/null || true
  pkill -x behavior_server 2>/dev/null || true
  pkill -x lifecycle_manager 2>/dev/null || true
  pkill -x map_server 2>/dev/null || true
  pkill -x amcl 2>/dev/null || true
  pkill -x component_container_isolated 2>/dev/null || true
}
trap cleanup EXIT

log "out=$OUT world=$WORLD config=$CONFIG init=($INIT_X,$INIT_Y) goal=($GOAL_X,$GOAL_Y) timeout=${GOAL_TIMEOUT}s"

# --- Isaac keepalive (nohup; never pkill -f this script name) ---
cd "$ROOT"
nohup ~/isaacsim/python.sh tools/nav2_isaac_keepalive.py \
  --seconds "$KEEPALIVE_SECS" \
  --world "$WORLD" \
  --config "$CONFIG" \
  --disable-cameras \
  >"$OUT/keepalive.log" 2>&1 &
KEEP_PID=$!
echo "$KEEP_PID" >"$OUT/keepalive.pid"
log "keepalive pid=$KEEP_PID"

log "=== wait keepalive ready ==="
for i in $(seq 1 120); do
  if grep -q '\[nav2_keepalive\] ready' "$OUT/keepalive.log" 2>/dev/null; then
    log "keepalive ready (attempt $i)"
    break
  fi
  if ! kill -0 "$KEEP_PID" 2>/dev/null; then
    log "FAIL: keepalive died"
    tail -n 60 "$OUT/keepalive.log" | tee -a "$SUMMARY"
    exit 2
  fi
  sleep 3
  if [[ "$i" -eq 120 ]]; then
    log "FAIL: keepalive timeout"
    tail -n 60 "$OUT/keepalive.log" | tee -a "$SUMMARY"
    exit 2
  fi
done

log "=== wait /human_states + /robot_states ==="
for t in /human_states /robot_states; do
  ok=0
  for i in $(seq 1 60); do
    if timeout 3 ros2 topic echo "$t" --once >/dev/null 2>&1; then
      log "topic ok $t"
      ok=1
      break
    fi
    sleep 2
  done
  [[ "$ok" -eq 1 ]] || { log "FAIL: $t"; exit 2; }
done

log "=== start evaluator ==="
ros2 run hunav_evaluator hunav_evaluator_node --ros-args \
  --params-file "$PARAMS" \
  -p "result_file:=$RESULT_FILE" \
  >"$OUT/evaluator_node.log" 2>&1 &
EVAL_PID=$!
for i in $(seq 1 40); do
  if ros2 service list 2>/dev/null | grep -q '/hunav_start_recording'; then
    log "evaluator services ready"
    break
  fi
  sleep 1
  if [[ "$i" -eq 40 ]]; then
    log "FAIL: evaluator services"
    tail -n 40 "$OUT/evaluator_node.log" | tee -a "$SUMMARY"
    exit 3
  fi
done

log "=== start recording tag=$EXP_TAG ==="
ros2 service call /hunav_start_recording hunav_msgs/srv/StartEvaluation \
  "{experiment_tag: '$EXP_TAG', run_id: $RUN_ID, robot_goal: {header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}}" \
  | tee -a "$SUMMARY"

log "=== Nav2 (goal $GOAL_X,$GOAL_Y) ==="
set +e
OUT_DIR="$OUT" MAP_YAML="$MAP_YAML" \
  INIT_X="$INIT_X" INIT_Y="$INIT_Y" INIT_YAW="$INIT_YAW" \
  GOAL_X="$GOAL_X" GOAL_Y="$GOAL_Y" GOAL_TIMEOUT="$GOAL_TIMEOUT" \
  "$NAV2_DIR/run_nav2_smoke.sh"
NAV_RC=$?
set -e
log "nav2_smoke exit=$NAV_RC"
[[ -f "$OUT/nav2_smoke_summary.txt" ]] && tail -n 40 "$OUT/nav2_smoke_summary.txt" | tee -a "$SUMMARY" || true
[[ -f "$OUT/nav2_abort_diag.txt" ]] && {
  log "=== copied abort diag ==="
  tail -n 60 "$OUT/nav2_abort_diag.txt" | tee -a "$SUMMARY" || true
}

log "=== stop recording ==="
ros2 service call /hunav_stop_recording std_srvs/srv/Empty "{}" | tee -a "$SUMMARY"
sleep 3

MAIN_CSV="${RESULT_FILE}.csv"
if [[ ! -f "$MAIN_CSV" ]]; then
  log "FAIL: missing $MAIN_CSV"
  ls -la "$OUT" | tee -a "$SUMMARY"
  exit 4
fi

log "PASS: $MAIN_CSV"
wc -l "$MAIN_CSV" | tee -a "$SUMMARY"
head -n 5 "$MAIN_CSV" | tee -a "$SUMMARY"
ls -la "$OUT"/metrics*.csv 2>/dev/null | tee -a "$SUMMARY" || true
# CSV path works even if Nav2 aborted — call that out clearly.
if [[ "$NAV_RC" -eq 0 ]] && grep -q 'GOAL=SUCCEEDED' "$OUT/nav2_smoke_summary.txt" 2>/dev/null; then
  log "VERDICT PASS (nav2 SUCCEEDED, csv ok)"
  exit 0
fi
log "VERDICT PARTIAL (csv ok, nav2_rc=$NAV_RC — see abort diag)"
exit 0
