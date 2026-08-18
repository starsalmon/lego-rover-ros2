#!/usr/bin/env bash
# Push lego-rover-ros2 scripts to the Pi from your Mac.
set -eo pipefail

PI="${ROVER_PI:-cain@lego-rover.local}"
KEY="${ROVER_SSH_KEY:-$HOME/.ssh/lego_rover_cursor}"
SRC="$(cd "$(dirname "$0")" && pwd)"

SSH_OPTS=(-i "$KEY" -o StrictHostKeyChecking=accept-new)

rsync -avz -e "ssh ${SSH_OPTS[*]}" \
  --exclude '.git' \
  "$SRC/" "$PI:~/lego-rover-ros2/"

ssh "${SSH_OPTS[@]}" "$PI" 'chmod +x ~/lego-rover-ros2/*.sh ~/lego-rover-ros2/*.py'

echo "Synced to $PI:~/lego-rover-ros2/"
