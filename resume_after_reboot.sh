#!/usr/bin/env bash
# Run once after Pi reboot. Resumes agent build if needed; safe to re-run.
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "=== resume_after_reboot $(date) ==="

if [[ -f ~/uros_agent_ws/install/local_setup.bash ]] && [[ -d ~/uros_agent_ws/install/micro_ros_agent ]]; then
  echo "AGENT_OK — already built."
  bash ~/lego-rover-ros2/preflight.sh 2>/dev/null || true
  exit 0
fi

sudo usermod -aG dialout "$USER" 2>/dev/null || true

if [[ ! -d ~/uros_agent_ws/install ]]; then
  echo "Workspace incomplete — running full install_microros_agent.sh (lowmem)"
  nohup bash ~/lego-rover-ros2/install_microros_agent.sh > ~/agent_install.log 2>&1 &
  echo "BUILD_STARTED (full) — tail -f ~/agent_install.log"
  exit 0
fi

echo "Resuming low-memory build..."
nohup bash ~/lego-rover-ros2/build_agent_lowmem.sh > ~/agent_install.log 2>&1 &
echo "BUILD_RESUMED (lowmem) — tail -f ~/agent_install.log"
echo "When done: ros2 pkg prefix micro_ros_agent && echo AGENT_OK"
