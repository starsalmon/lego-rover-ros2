#!/usr/bin/env bash
# One-time: ROS 2 Jazzy + pygame on Mac for PS4 teleop over WiFi.
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
MAMBA_ROOT="${MAMBA_ROOT:-$HOME/micromamba}"
export MAMBA_ROOT_PREFIX="$MAMBA_ROOT"
ENV_NAME="${ROVER_ROS_ENV:-rover}"
ENV_PREFIX="$MAMBA_ROOT/envs/$ENV_NAME"

ARCH="$(uname -m)"
case "$ARCH" in
  arm64) MM_URL="https://micro.mamba.pm/api/micromamba/osx-arm64/latest" ;;
  x86_64) MM_URL="https://micro.mamba.pm/api/micromamba/osx-64/latest" ;;
  *)
    echo "ERROR: unsupported Mac arch: $ARCH"
    exit 1
    ;;
esac

if [[ ! -x "$MAMBA_ROOT/bin/micromamba" ]]; then
  echo "=== Installing micromamba -> $MAMBA_ROOT ==="
  mkdir -p "$MAMBA_ROOT"
  curl -Ls "$MM_URL" | tar -xj -C "$MAMBA_ROOT" bin/micromamba
fi

if [[ ! -f "$ENV_PREFIX/setup.bash" ]]; then
  echo "=== Creating ROS 2 Jazzy env '$ENV_NAME' (RoboStack — may take 10–20 min) ==="
  "$MAMBA_ROOT/bin/micromamba" create -y -n "$ENV_NAME" \
    -c conda-forge -c robostack-jazzy \
    ros-jazzy-desktop python=3.11
else
  echo "Env '$ENV_NAME' already exists — skipping create"
fi

echo "=== Installing pygame in rover env ==="
# pygame-ce is a drop-in with newer SDL fixes on macOS (avoids some joystick segfaults).
if [[ "${ROVER_PYGAME:-ce}" == "ce" ]]; then
  "$ENV_PREFIX/bin/pip" install -q 'pygame-ce>=2.5'
else
  "$ENV_PREFIX/bin/pip" install -q pygame
fi

echo ""
echo "=== Mac teleop ready ==="
echo "  Pair PS4 to Mac via Bluetooth, then:"
echo "    bash $DIR/go_mac.sh"
echo ""
echo "  Or manually:"
echo "    source $DIR/mac_ros_env.sh"
echo "    python3 $DIR/teleop_ps4.py"
