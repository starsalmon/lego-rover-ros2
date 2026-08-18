#!/usr/bin/env bash
# Start UDP /cmd_vel relay on the Pi (kills stale SSH/UDP relays first).
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PIDFILE=/tmp/cmd_vel_udp_relay.pid

kill_stale() {
  for pat in 'python3.*cmd_vel_ssh_relay' 'python3.*cmd_vel_udp_relay'; do
    for _ in 1 2 3 4 5; do
      pids=$(pgrep -f "$pat" 2>/dev/null || true)
      [[ -z "$pids" ]] && break
      for pid in $pids; do
        kill "$pid" 2>/dev/null || true
      done
      sleep 0.3
    done
  done
  sleep 0.5
}

kill_stale

set +u
source /opt/ros/jazzy/setup.bash
source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true
set -u

cd "$DIR"
setsid env RELAY_DEBUG=1 python3 -u "$DIR/cmd_vel_udp_relay.py" </dev/null >/tmp/cmd_vel_udp.log 2>&1 &
echo $! >"$PIDFILE"
sleep 0.5
count=$(pgrep -cf 'python3.*cmd_vel_udp_relay' 2>/dev/null || echo 0)
if [[ "$count" -ne 1 ]]; then
  echo "ERROR: expected 1 UDP relay, found $count" >&2
  pgrep -af cmd_vel_udp_relay >&2 || true
  exit 1
fi
