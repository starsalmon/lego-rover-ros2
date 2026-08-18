#!/usr/bin/env bash
# Stop demo, teleop, and micro-ROS agent. Safe to run anytime.
set +e

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"

echo "Stopping rover processes..."

pkill -f 'demo_showcase.py' 2>/dev/null
pkill -f 'demo_smooth_drive.py' 2>/dev/null
pkill -f 'autonomous_explore.py|wall_follow.py|rover_session.py' 2>/dev/null
pkill -f 'teleop_node' 2>/dev/null
pkill -f 'joy_node' 2>/dev/null
pkill -f 'micro_ros_agent' 2>/dev/null

sleep 1
rover_ir_close

echo "Stopped."
