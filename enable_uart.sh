#!/usr/bin/env bash
# Enable Pi UART on /dev/ttyAMA0 for ESP32 (Ubuntu Server on Pi Zero 2W)
set -euo pipefail

echo "Disabling serial console on ttyAMA0..."
sudo sed -i 's/console=serial0,[0-9]* //g' /boot/firmware/cmdline.txt 2>/dev/null || true
sudo sed -i 's/console=ttyAMA0,[0-9]* //g' /boot/firmware/cmdline.txt 2>/dev/null || true

if ! grep -q '^enable_uart=1' /boot/firmware/config.txt 2>/dev/null; then
  echo 'enable_uart=1' | sudo tee -a /boot/firmware/config.txt
fi

if ! grep -q 'dtoverlay=disable-bt' /boot/firmware/config.txt 2>/dev/null; then
  echo 'dtoverlay=disable-bt' | sudo tee -a /boot/firmware/config.txt
fi

sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true

echo "Done. Reboot, then: ls -l /dev/ttyAMA0"
echo "  sudo reboot"
