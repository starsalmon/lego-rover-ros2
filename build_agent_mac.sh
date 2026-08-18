#!/usr/bin/env bash
# Build micro-ROS agent on Mac (Docker arm64) and deploy to Pi — avoids Pi OOM.
# Requires: Docker Desktop on Apple Silicon (or any machine with Docker + arm64).
set -eo pipefail

PI="${ROVER_PI:-cain@lego-rover.local}"
KEY="${ROVER_SSH_KEY:-$HOME/.ssh/lego_rover_cursor}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/.agent_mac_build"
SSH_OPTS=(-i "$KEY" -o StrictHostKeyChecking=accept-new)

if ! command -v docker &>/dev/null; then
  echo "ERROR: Docker not found. Install Docker Desktop, or use build_agent_lowmem.sh on Pi."
  exit 1
fi

mkdir -p "$BUILD_DIR"
echo "Building arm64 agent in Docker (first run downloads image — may take a while)..."

docker run --rm --platform linux/arm64 \
  -v "$BUILD_DIR:/out" \
  ros:jazzy-ros-base \
  bash -c '
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1

apt-get update -qq
apt-get install -y -qq git python3-colcon-common-extensions python3-rosdep build-essential

mkdir -p /ws/src && cd /ws
if [ ! -d src/micro_ros_setup ]; then
  git clone -b jazzy --depth 1 https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

source /opt/ros/jazzy/setup.bash
rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select micro_ros_setup --parallel-workers 1
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh

cd agent_ws
colcon build \
  --install-base /ws/install_agent \
  --build-base /ws/build_agent \
  --packages-up-to micro_ros_agent \
  --parallel-workers 1 \
  --executor sequential \
  --cmake-args -DUAGENT_BUILD_EXECUTABLE=OFF -DUAGENT_P2P_PROFILE=OFF

test -f /ws/install_agent/local_setup.bash
tar -czf /out/install_agent.tar.gz -C /ws install_agent
echo MAC_BUILD_DONE
'

echo "Deploying to $PI..."
ssh "${SSH_OPTS[@]}" "$PI" 'mkdir -p ~/uros_agent_ws'
scp "${SSH_OPTS[@]}" "$BUILD_DIR/install_agent.tar.gz" "$PI:~/uros_agent_ws/"
ssh "${SSH_OPTS[@]}" "$PI" 'cd ~/uros_agent_ws && rm -rf install_agent && tar -xzf install_agent.tar.gz && test -f install_agent/local_setup.bash && echo AGENT_OK'

echo "Done. On Pi: bash ~/lego-rover-ros2/preflight.sh"
