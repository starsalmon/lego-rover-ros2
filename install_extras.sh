#!/usr/bin/env bash
# Gamepad teleop deps — run once on the Pi.
# Note: ros-jazzy-joy often fails on Ubuntu 24.04 point releases (held libs).
# We use python3-evdev instead — no ROS joy packages required.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive

sudo apt update
sudo apt install -y \
  joystick \
  bluez \
  python3-yaml

sudo systemctl enable bluetooth
sudo systemctl start bluetooth

sudo usermod -aG dialout "$USER" 2>/dev/null || true
sudo usermod -aG input "$USER" 2>/dev/null || true
sudo usermod -aG bluetooth "$USER" 2>/dev/null || true

# Optional: ROS joy packages (skip if apt reports held broken packages)
if sudo apt install -y ros-jazzy-joy ros-jazzy-teleop-twist-joy 2>/dev/null; then
  echo "ROS joy packages installed (optional legacy path)."
else
  echo "ROS joy packages skipped (apt conflict) — teleop_ps4.py uses evdev instead."
fi

# PS4 over BT on newer kernels
if modinfo hid-playstation &>/dev/null; then
  echo "hid-playstation module available."
else
  echo "Tip: if PS4 BT has no input device, try: sudo modprobe hid-playstation"
fi

echo ""
echo "Extras installed."
echo "  Pair PS4:   bash ~/lego-rover-ros2/pair_ps4.sh"
echo "  Drive:      bash ~/lego-rover-ros2/go_ps4.sh"
echo "  Or:         bash ~/lego-rover-ros2/run_teleop_ps4.sh"
if ! groups "$USER" | grep -q '\binput\b'; then
  echo "  Log out and back in for input group (or reboot)."
fi
