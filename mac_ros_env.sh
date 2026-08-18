#!/usr/bin/env bash
# Source ROS 2 Jazzy (RoboStack) for Mac teleop.
set -eo pipefail

MAMBA_ROOT="${MAMBA_ROOT:-$HOME/micromamba}"
ENV_NAME="${ROVER_ROS_ENV:-rover}"
ENV_PREFIX="$MAMBA_ROOT/envs/$ENV_NAME"

if [[ ! -f "$ENV_PREFIX/setup.bash" ]]; then
  echo "ERROR: ROS env missing at $ENV_PREFIX"
  echo "  Run: bash $(dirname "$0")/install_mac_teleop.sh"
  return 1 2>/dev/null || exit 1
fi

set +u
# shellcheck disable=SC1091
source "$ENV_PREFIX/setup.bash"
set -u
export PATH="$ENV_PREFIX/bin:$PATH"

if ! command -v ros2 >/dev/null; then
  echo "ERROR: ros2 not in env '$ENV_NAME' — re-run install_mac_teleop.sh"
  return 1 2>/dev/null || exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# macOS multicast discovery is flaky — go_mac.sh sets ROS_STATIC_PEERS to the Pi IP.
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-}"
