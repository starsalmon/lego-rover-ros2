#!/usr/bin/env bash
# Free RAM before agent build on Pi Zero 2W (512MB). Run on Pi console.
set -eo pipefail

echo "=== prepare_lowmem $(date) ==="

# Kill stuck builds first — stops OOM spiral
sudo pkill -9 -f colcon 2>/dev/null || true
sudo pkill -9 -f 'cmake|ninja|g\+\+|cc1plus' 2>/dev/null || true
sleep 2

# 2GB swap (critical on Zero 2W)
if ! swapon --show | grep -q '/swapfile'; then
  echo "Adding 2GB swapfile..."
  sudo fallocate -l 2G /swapfile 2>/dev/null || sudo dd if=/dev/zero of=/swapfile bs=1M count=2048 status=progress
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  grep -q '/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
fi

# Stop non-essential services during build
for svc in bluetooth avahi-daemon snapd snapd.socket unattended-upgrades fwupd ModemManager; do
  sudo systemctl stop "$svc" 2>/dev/null || true
  sudo systemctl disable "$svc" 2>/dev/null || true
done

# Drop caches (needs root, frees a little RAM)
sync
echo 3 | sudo tee /proc/sys/vm/drop_caches >/dev/null || true

sudo usermod -aG dialout "$USER" 2>/dev/null || true

echo ""
free -h
swapon --show
echo ""
echo "Ready. Next: bash ~/lego-rover-ros2/build_agent_lowmem.sh"
