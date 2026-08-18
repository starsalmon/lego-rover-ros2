#!/usr/bin/env bash
# Autonomous explore on the Pi (no Mac teleop).
# SSH:  ssh cain@lego-rover.local 'bash ~/lego-rover-ros2/go_auto.sh'
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"
rover_source_ros

echo "=== LEGO Rover — autonomous ==="
echo "Stops Mac teleop relay if running; Pi drives /cmd_vel directly."
echo "Cruise cap: ${ROVER_AUTO_MAX_LINEAR:-0.23}  Burst: ${ROVER_AUTO_BURST_MAX:-0.28} (~0.5% chance)"
echo "Linear sign: ${ROVER_LINEAR_SIGN:-1} (set ROVER_LINEAR_SIGN=-1 if forward is reversed on wire)"
echo "Front sonar: ESP scan+escape (IR front stop off)"
echo ""

export ROVER_AUTO_MAX_LINEAR="${ROVER_AUTO_MAX_LINEAR:-0.23}"
export ROVER_AUTO_BURST_MAX="${ROVER_AUTO_BURST_MAX:-0.28}"
export ROVER_AUTO_BURST_PROB="${ROVER_AUTO_BURST_PROB:-0.005}"
export ROVER_LINEAR_SIGN="${ROVER_LINEAR_SIGN:-1}"
export ROVER_SONAR="${ROVER_SONAR:-1}"
export ROVER_IR_FRONT_ESCAPE="${ROVER_IR_FRONT_ESCAPE:-0}"
export ROVER_IR_FRONT_STOP="${ROVER_IR_FRONT_STOP:-0}"
export ROVER_CRUISE_RADAR="${ROVER_CRUISE_RADAR:-1}"
export ROVER_CRUISE_RADAR_RING="${ROVER_CRUISE_RADAR_RING:-0}"
export ROVER_CRUISE_RADAR_SETTLE="${ROVER_CRUISE_RADAR_SETTLE:-0.05}"
export ROVER_AUTO_TICK="${ROVER_AUTO_TICK:-0.02}"
# Sonar escape recording: bash record_sonar_escape.sh 120  (or ROVER_SONAR_RECORD=1 in rover_session)
export ROVER_SONAR_REAR_SCAN="${ROVER_SONAR_REAR_SCAN:-1}"
# Front sonar ring: distance colours on forward arc (not rear IR radar).
# If sweep LED looks reversed on the ring, try ROVER_SONAR_PAN_MIRROR=1
export ROVER_SONAR_PAN_MIRROR="${ROVER_SONAR_PAN_MIRROR:-0}"
export ROVER_TWIST_SWAP="${ROVER_TWIST_SWAP:-0}"
export ROVER_BUTTON_SOURCE="${ROVER_BUTTON_SOURCE:-esp}"
export ROVER_BRIDGE_EXTERNAL=1
export ROVER_QUICK_START=1
export ROVER_SKIP_WAIT_KILL=1

if systemctl is-active --quiet rover-main.service 2>/dev/null; then
  echo "rover-main.service is running — using its ESP bridge (not killing standby waiter)."
fi

# Teleop UDP relay fights autonomous for /cmd_vel.
pkill -f 'cmd_vel_udp_relay.py' 2>/dev/null || true
pkill -f 'cmd_vel_ssh_relay.py' 2>/dev/null || true
sleep 0.3

exec bash "$DIR/go.sh"
