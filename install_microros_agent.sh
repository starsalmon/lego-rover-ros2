#!/usr/bin/env bash
# Build micro-ROS agent from source (no apt package for Jazzy arm64)
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive

WS=~/uros_agent_ws
mkdir -p "$WS/src"
cd "$WS"

set +u
source /opt/ros/jazzy/setup.bash

if [ ! -d src/micro_ros_setup ]; then
  git clone -b jazzy --depth 1 https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1

bash "$(dirname "$0")/prepare_lowmem.sh"

rosdep install --from-paths src --ignore-src -y
colcon build --packages-select micro_ros_setup --parallel-workers 1 --executor sequential
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
bash "$(dirname "$0")/build_agent_lowmem.sh"

grep -q 'uros_agent_ws' ~/.bashrc || cat >> ~/.bashrc <<'EOF'

source ~/uros_agent_ws/install/local_setup.bash
EOF

if [[ -d "$WS/install/micro_ros_agent" ]]; then
  echo "AGENT_DONE"
  echo "micro-ROS agent built. Run: bash ~/lego-rover-ros2/preflight.sh"
else
  echo "AGENT_FAILED — check build output above"
  exit 1
fi
