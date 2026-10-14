#!/usr/bin/env bash
# Mac → sync fleet brains to dockerhost and start mini explore→chase.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST="${DOCKERHOST:-dockerhost}"
DEST="${ROBOT_FLEET_DIR:-robot-fleet}"

echo "=== Swarm go (mini on dockerhost) ==="
bash "$SCRIPT_DIR/sync-dockerhost.sh"

ssh "$HOST" "chmod +x ~/$DEST/ros.sh ~/$DEST/topic-once.sh ~/$DEST/swarm-explore.sh && sh ~/$DEST/swarm-explore.sh"

echo ""
echo "Rover (separate SSH):"
echo "  bash ~/Documents/cursor-esp32/lego-rover-ros2/sync_to_pi.sh   # if scripts changed"
echo "  ssh lego-rover 'bash ~/lego-rover-ros2/go_swarm.sh'"
