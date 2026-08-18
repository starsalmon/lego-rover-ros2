#!/usr/bin/env bash
# Single-threaded agent build — survives 512MB Pi Zero 2W with swap.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
export NINJAFLAGS=-j1

WS=~/uros_agent_ws
LOG=~/agent_install.log
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

agent_ready() {
  [[ -f "$WS/install/local_setup.bash" ]] && \
    [[ -d "$WS/install/micro_ros_agent" ]] && \
    ros2 pkg prefix micro_ros_agent &>/dev/null
}

echo "=== build_agent_lowmem $(date) ===" | tee "$LOG"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f "$WS/install/local_setup.bash" ]] && source "$WS/install/local_setup.bash"
set -u

if agent_ready; then
  echo "AGENT_OK — already built." | tee -a "$LOG"
  exit 0
fi

bash "$SCRIPT_DIR/prepare_lowmem.sh" | tee -a "$LOG"

cd "$WS"
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u

if [[ ! -d src/uros/micro-ROS-Agent ]]; then
  echo "Creating agent workspace..." | tee -a "$LOG"
  ros2 run micro_ros_setup create_agent_ws.sh 2>&1 | tee -a "$LOG"
fi

echo "Building micro_ros_agent (1 worker, expect 20-40 min)..." | tee -a "$LOG"

colcon build \
  --packages-up-to micro_ros_agent \
  --parallel-workers 1 \
  --executor sequential \
  --cmake-args -DUAGENT_BUILD_EXECUTABLE=OFF -DUAGENT_P2P_PROFILE=OFF --no-warn-unused-cli \
  2>&1 | tee -a "$LOG"

set +u
source "$WS/install/local_setup.bash"
set -u

if agent_ready; then
  echo "AGENT_DONE" | tee -a "$LOG"
  grep -q 'uros_agent_ws/install/local_setup.bash' ~/.bashrc 2>/dev/null || cat >> ~/.bashrc <<'EOF'

source ~/uros_agent_ws/install/local_setup.bash
EOF
  bash "$SCRIPT_DIR/restore_services.sh" 2>/dev/null || true
else
  echo "AGENT_FAILED — see $LOG" | tee -a "$LOG"
  exit 1
fi
