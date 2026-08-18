#!/usr/bin/env bash
# Launch parked Stretch + museum_eval (10 Impassive, unique loops).
# Cameras OFF — 10 agents + RGB-D on this laptop lags the sim and looks like thrash.
# Do not start a second hunav_agent_manager. Do not drive.
#
# Usage:
#   ./tools/run_museum_static_eval.sh
# Then in another ROS shell: ./tools/evaluator_smoke/run_static_eval.sh
set -eo pipefail

WS="/home/osamuzahid/Projects/isaac-social-nav/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS/install/setup.bash"
ISAAC_PY="${HOME}/isaacsim/python.sh"
MAIN_PY="$WS/src/Hunav_isaac_wrapper/src/scripts/main.py"
OUT="${OUT_DIR:-/tmp/hunav_static_eval}"
LOG="$OUT/isaac.log"

mkdir -p "$OUT"

pkill -f '/hunav_agent_manager/hunav_agent_manager' 2>/dev/null || true
pkill -f 'ros2 run hunav_agent_manager hunav_agent_manager' 2>/dev/null || true
pkill -f '/hunav_agent_manager/hunav_loader' 2>/dev/null || true
pkill -f hunav_evaluator_node 2>/dev/null || true
pkill -f teleop_twist_keyboard 2>/dev/null || true
# Do not pkill -f nav2_isaac_keepalive (kills this cmdline if it matched).
pkill -f '/isaacsim/kit/kit' 2>/dev/null || true
pkill -f '/isaacsim/kit/python/bin/python3' 2>/dev/null || true
sleep 1

export OMNI_KIT_ACCEPT_EULA=YES
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export HUNAV_ISAAC_PROFILE="${HUNAV_ISAAC_PROFILE:-debug}"
export HUNAV_ISAAC_HEADLESS="${HUNAV_ISAAC_HEADLESS:-0}"
export HUNAV_LAB_SENSORS=1
export HUNAV_LAB_LIDAR=1
export HUNAV_LAB_CAMERAS=0
export HUNAV_REACTION_LOG="$OUT/reaction.csv"
unset PYTHONEXE PYTHONHOME CONDA_PREFIX
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"

echo "=== $(date) parked Stretch + museum_eval (10 Impassive) ==="
echo "reaction log: $HUNAV_REACTION_LOG"
echo "cameras OFF; lidar ON. Do not teleop."
echo "Log: $LOG"
echo

exec > >(tee -a "$LOG") 2>&1
bash "$ISAAC_PY" "$MAIN_PY" --debug --no-headless --batch \
  --robot stretch --world museum --config museum_eval
