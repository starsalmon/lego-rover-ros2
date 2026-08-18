#!/usr/bin/env bash
# Lightweight kill — demos/teleop; agent kept unless --full.
KEEP_AGENT=0
[[ "${1:-}" == "--keep-agent" ]] && KEEP_AGENT=1
[[ "${1:-}" == "--full" ]] && KEEP_AGENT=0

pkill -9 -f 'demo_showcase.py' 2>/dev/null || true
pkill -9 -f 'demo_smooth_drive.py' 2>/dev/null || true
pkill -9 -f 'teleop_ps4.py' 2>/dev/null || true
pkill -9 -f 'autonomous_explore.py' 2>/dev/null || true
pkill -9 -f 'cmd_vel_udp_relay.py' 2>/dev/null || true
pkill -9 -f 'cmd_vel_ssh_relay.py' 2>/dev/null || true

if (( KEEP_AGENT == 0 )); then
  pkill -9 -f 'micro_ros_agent' 2>/dev/null || true
  pkill -9 -f 'ros2 run micro_ros_agent' 2>/dev/null || true
  sleep 2
  if pgrep -f micro_ros_agent >/dev/null; then
    echo "WARN: agent still running — run: sudo fuser -k /dev/ttyAMA0"
  fi
fi
