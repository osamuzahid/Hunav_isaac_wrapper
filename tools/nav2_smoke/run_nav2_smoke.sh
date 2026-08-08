#!/usr/bin/env bash
# Headless Nav2 smoke against a running Isaac keepalive (carter_ROS).
#
# Env overrides:
#   MAP_YAML  INIT_X INIT_Y GOAL_X GOAL_Y OUT_DIR
# Defaults = empty_world free map, goal (2,0).
set -eo pipefail
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/jazzy/setup.bash
source /home/osamuzahid/Projects/isaac-social-nav/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SMOKE_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT="${OUT_DIR:-/tmp/nav2_smoke}"
mkdir -p "$OUT"
MAP_YAML="${MAP_YAML:-$SMOKE_DIR/empty_free.yaml}"
PARAMS="${PARAMS:-$SMOKE_DIR/nav2_carter_params.yaml}"
INIT_X="${INIT_X:-0.0}"
INIT_Y="${INIT_Y:-0.0}"
INIT_YAW="${INIT_YAW:-0.0}"
GOAL_X="${GOAL_X:-2.0}"
GOAL_Y="${GOAL_Y:-0.0}"
SUMMARY="$OUT/nav2_smoke_summary.txt"

# Isaac Kit + many leftover ROS nodes can exhaust FastDDS SHM locks.
# Prefer UDP for this shell (and any child nav2 / laserscan processes).
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$SMOKE_DIR/fastrtps_no_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

: >"$SUMMARY"
log() { echo "$@" | tee -a "$SUMMARY"; }

# yaw → quaternion z,w (roll=pitch=0)
read -r INIT_QZ INIT_QW < <(python3 - <<PY
import math
y=float("$INIT_YAW")
print(f"{math.sin(y/2):.6f} {math.cos(y/2):.6f}")
PY
)

log "map=$MAP_YAML init=($INIT_X,$INIT_Y,yaw=$INIT_YAW) goal=($GOAL_X,$GOAL_Y)"
log "fastrtps_profile=$FASTRTPS_DEFAULT_PROFILES_FILE"

log "=== wait for /clock (reliable then best_effort) ==="
clock_ok=0
for i in $(seq 1 90); do
  if timeout 4 ros2 topic echo /clock --once --qos-reliability reliable \
      --qos-durability volatile >/dev/null 2>&1 \
    || timeout 4 ros2 topic echo /clock --once --qos-reliability best_effort \
      --qos-durability volatile >/dev/null 2>&1; then
    log "clock ok (attempt $i)"
    clock_ok=1
    break
  fi
  sleep 2
done
if [[ "$clock_ok" -ne 1 ]]; then
  log "FAIL: no /clock"
  ros2 topic list | tee -a "$SUMMARY" || true
  ros2 topic info /clock -v 2>&1 | tee -a "$SUMMARY" || true
  exit 2
fi

log "=== pointcloud_to_laserscan ==="
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/front_3d_lidar/lidar_points \
  -r scan:=/scan \
  -p target_frame:=base_link \
  -p transform_tolerance:=0.2 \
  -p min_height:=-0.3 \
  -p max_height:=0.5 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.1 \
  -p range_min:=0.2 \
  -p range_max:=30.0 \
  -p use_inf:=true \
  -p use_sim_time:=true \
  >"$OUT/laserscan.log" 2>&1 &
PC2_PID=$!
log "laserscan pid=$PC2_PID"
sleep 3
timeout 5 ros2 topic hz /scan --window 5 >"$OUT/scan_hz.txt" 2>&1 || true
log "scan hz:"
cat "$OUT/scan_hz.txt" | tee -a "$SUMMARY" || true

log "=== nav2_bringup ==="
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  map:="$MAP_YAML" \
  params_file:="$PARAMS" \
  autostart:=True \
  use_composition:=False \
  use_localization:=True \
  slam:=False \
  >"$OUT/nav2_bringup.log" 2>&1 &
NAV_PID=$!
log "nav2 pid=$NAV_PID"

log "=== wait navigate_to_pose ==="
for i in $(seq 1 90); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    log "navigate_to_pose available (attempt $i)"
    break
  fi
  sleep 2
  if [[ $i -eq 90 ]]; then
    log "FAIL: navigate_to_pose missing"
    tail -40 "$OUT/nav2_bringup.log" | tee -a "$SUMMARY"
    kill $PC2_PID $NAV_PID 2>/dev/null || true
    exit 3
  fi
done

log "=== initialpose ($INIT_X,$INIT_Y) qz=$INIT_QZ qw=$INIT_QW ==="
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: ${INIT_X}, y: ${INIT_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${INIT_QZ}, w: ${INIT_QW}}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}" \
  | tee -a "$SUMMARY" || true
sleep 4

log "=== NavigateToPose goal ($GOAL_X,$GOAL_Y) ==="
timeout 3 ros2 topic echo /chassis/odom --once --qos-reliability best_effort >"$OUT/odom_before.txt" 2>&1 || true

GOAL_TIMEOUT="${GOAL_TIMEOUT:-120}"
set +e
timeout "$GOAL_TIMEOUT" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${INIT_QZ}, w: ${INIT_QW}}}}}" \
  >"$OUT/nav_goal.txt" 2>&1
GOAL_RC=$?
set -e
log "send_goal exit=$GOAL_RC timeout=${GOAL_TIMEOUT}s"
tail -40 "$OUT/nav_goal.txt" | tee -a "$SUMMARY"

timeout 3 ros2 topic echo /chassis/odom --once --qos-reliability best_effort >"$OUT/odom_after.txt" 2>&1 || true
log "=== odom / goal verdict ==="
python3 - <<PY | tee -a "$SUMMARY"
import re
from pathlib import Path
out = Path("$OUT")

def xy(path):
    t = path.read_text() if path.is_file() else ""
    m = re.search(r"position:\n\s+x:\s*([-\d.eE]+)\n\s+y:\s*([-\d.eE]+)", t)
    return (float(m.group(1)), float(m.group(2))) if m else None

b, a = xy(out/"odom_before.txt"), xy(out/"odom_after.txt")
print("before", b, "after", a)
if b and a:
    dist = ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5
    print(f"delta_xy={dist:.3f} m")
    print("MOTION=PASS" if dist > 0.3 else "MOTION=WEAK")
else:
    print("MOTION=UNKNOWN")
goal = (out/"nav_goal.txt").read_text() if (out/"nav_goal.txt").is_file() else ""
if "STATUS_SUCCEEDED" in goal or "status: SUCCEEDED" in goal or "Goal finished with status: SUCCEEDED" in goal:
    print("GOAL=SUCCEEDED")
elif "ABORTED" in goal:
    print("GOAL=ABORTED")
elif "CANCELED" in goal:
    print("GOAL=CANCELED")
else:
    print("GOAL=UNKNOWN")
# Count progress failures in bringup log
blog = (out/"nav2_bringup.log").read_text() if (out/"nav2_bringup.log").is_file() else ""
n_prog = blog.count("Failed to make progress")
print(f"progress_fail_count={n_prog}")
PY

kill $PC2_PID $NAV_PID 2>/dev/null || true
wait $PC2_PID $NAV_PID 2>/dev/null || true
log "=== done ==="
cat "$SUMMARY"
