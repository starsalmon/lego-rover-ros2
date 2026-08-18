#!/usr/bin/env bash
# Restart Pi micro-ROS agent with FastDDS peer aimed at Mac (for Mac teleop session).
set -eo pipefail

MAC_IP="${1:-${MAC_IP:-}}"
MAC_IP="${MAC_IP:?Mac IP required}"
DIR="$(cd "$(dirname "$0")" && pwd)"

bash "$DIR/rover_kill.sh" --keep-agent
pkill -f autonomous_explore.py 2>/dev/null || true
systemctl --user stop rover-agent.service 2>/dev/null || true
pkill -f micro_ros_agent 2>/dev/null || true
sleep 1

PROFILE="/tmp/rover_fastdds_mac.xml"
bash "$DIR/write_fastdds_peer.sh" "$MAC_IP" "$PROFILE"

set +u
source /opt/ros/jazzy/setup.bash
source ~/uros_agent_ws/install/local_setup.bash
set -u

export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
export ROS_STATIC_PEERS="$MAC_IP"
export ROS_LOCALHOST_ONLY=0

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 460800 \
  >>/tmp/rover_agent.log 2>&1 &
AGENT_PID=$!

echo "Agent pid $AGENT_PID — waiting for ESP32..."
for i in $(seq 1 45); do
  if ros2 node list 2>/dev/null | grep -q 'lego_rover_esp32'; then
    echo "ESP32 linked."
    exit 0
  fi
  sleep 1
done

echo "WARN: ESP32 not on graph after 45s (check UART / power-cycle ESP once)"
exit 1
