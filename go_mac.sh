#!/usr/bin/env bash
# Drive LEGO rover from Mac with PS4 over WiFi.
#
# Uses SSH relay to Pi (reliable on macOS). Optional DDS mode: ROVER_MAC_DDS=1
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PI="${ROVER_PI:-cain@lego-rover.local}"
KEY="${ROVER_SSH_KEY:-$HOME/.ssh/lego_rover_cursor}"
SSH_OPTS=(-i "$KEY" -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8)

if [[ -x "${HOME}/micromamba/envs/rover/bin/python3" ]]; then
  PYTHON="${HOME}/micromamba/envs/rover/bin/python3"
else
  PYTHON="python3"
fi

_mac_ip() {
  ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || true
}

_pi_ip() {
  ssh "${SSH_OPTS[@]}" "$PI" "hostname -I 2>/dev/null | awk '{print \$1}'" 2>/dev/null || true
}

MAC_IP="$(_mac_ip)"
PI_IP="${ROVER_PI_IP:-$(_pi_ip)}"

if [[ -z "$PI_IP" ]]; then
  echo "ERROR: could not reach Pi at $PI"
  exit 1
fi

export TELEOP_CONFIG="${TELEOP_CONFIG:-$DIR/teleop_mac.yaml}"
export ROVER_PI="$PI"
export ROVER_PI_IP="$PI_IP"
export ROVER_SSH_KEY="$KEY"
export ROVER_TELEOP_UDP="${ROVER_TELEOP_UDP:-1}"
cd "$DIR"

echo "=== LEGO Rover — Mac PS4 teleop ==="
if [[ -n "$MAC_IP" ]]; then
  echo "Mac $MAC_IP  →  Pi $PI_IP"
else
  echo "Pi $PI_IP"
fi
echo ""

# Lightweight BT check — do NOT open pygame here; a second SDL open on macOS segfaults.
_detect_gamepad() {
  system_profiler SPBluetoothDataType 2>/dev/null \
    | grep -qiE 'wireless controller|dualshock|dualsense|playstation|DUALSHOCK|Wireless Controller'
}

echo "Looking for gamepad (Bluetooth)..."
FOUND_PAD=0
for _ in 1 2 3 4 5; do
  if _detect_gamepad; then
    FOUND_PAD=1
    break
  fi
  sleep 0.5
done
if (( FOUND_PAD == 0 )); then
  echo "  WARN: no BT gamepad in system_profiler — teleop will retry (USB OK)."
else
  echo "  Gamepad OK (Bluetooth visible)."
fi
echo ""

echo "[1/2] Stopping Pi autonomous (keeping agent)..."
ssh "${SSH_OPTS[@]}" "$PI" 'bash ~/lego-rover-ros2/rover_kill.sh --keep-agent; pkill -f autonomous_explore.py 2>/dev/null || true; source /opt/ros/jazzy/setup.bash; source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null; for i in 1 2 3; do ros2 topic pub --once --no-wait-for-subscribers --qos-reliability reliable /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true; done' || true

if [[ "${ROVER_MAC_DDS:-0}" == "1" ]] && [[ -n "$MAC_IP" ]]; then
  echo "[2/2] DDS mode — linking Mac ROS graph to Pi..."
  # shellcheck disable=SC1091
  source "$DIR/mac_ros_env.sh"
  export ROS_STATIC_PEERS="$PI_IP"
  bash "$DIR/write_fastdds_peer.sh" "$PI_IP" /tmp/rover_fastdds_pi.xml
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/rover_fastdds_pi.xml
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  ssh "${SSH_OPTS[@]}" "$PI" "MAC_IP='$MAC_IP' bash ~/lego-rover-ros2/rover_agent_mac_peer.sh" || true
  ros2 daemon stop 2>/dev/null || true; sleep 1
  ros2 daemon start 2>/dev/null || true; sleep 1
  if python3 "$DIR/mac_wait_cmd_vel.py" 30; then
    echo "  DDS linked — starting teleop."
    python3 "$DIR/teleop_ps4.py"
    exit 0
  fi
  echo "WARN: DDS failed — falling back to SSH relay"
fi

echo "[2/2] Starting Pi UDP relay + teleop → /cmd_vel"
ssh "${SSH_OPTS[@]}" "$PI" "bash ~/lego-rover-ros2/start_udp_relay.sh" || true
sleep 1
echo "  R2 = forward  L2 = reverse  Left stick = steer/spin"
echo "  TELEOP_DEBUG=1 for axis dump"
echo ""
"$PYTHON" "$DIR/teleop_mac_ssh.py"
