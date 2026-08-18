#!/usr/bin/env bash
# Quick motor test — run while agent is NOT running, or stop demo first.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true

echo "Starting agent (only one should run)..."
bash "$(dirname "$0")/rover_kill.sh"
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 460800 &
AGENT_PID=$!
echo "Waiting 6s for ESP to connect (see 'micro-ROS ready' on USB serial)..."
sleep 6

echo "Pulsing forward 0.6 for 4 seconds (reliable)..."
timeout 4 ros2 topic pub --rate 10 --qos-reliability reliable \
  /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}" || true

timeout 2 ros2 topic pub --rate 10 --qos-reliability reliable \
  /cmd_vel geometry_msgs/msg/Twist "{}" || true
kill "$AGENT_PID" 2>/dev/null || true
echo "Done. If wheels did not move: check VMOT power to motor driver."
