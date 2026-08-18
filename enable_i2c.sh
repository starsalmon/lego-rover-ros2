#!/usr/bin/env bash
# Enable Pi I2C (PCA9685 servo) and grant access without sudo.
set -euo pipefail

CFG=/boot/firmware/config.txt
[[ -f "$CFG" ]] || CFG=/boot/config.txt

echo "=== Enable I2C ==="

if [[ -f "$CFG" ]] && ! grep -q '^dtparam=i2c_arm=on' "$CFG" 2>/dev/null; then
  echo 'dtparam=i2c_arm=on' | sudo tee -a "$CFG"
  echo "Added dtparam=i2c_arm=on — reboot required."
  NEED_REBOOT=1
fi

sudo apt-get install -y python3-smbus i2c-tools

if getent group i2c >/dev/null; then
  sudo usermod -aG i2c "$USER"
  echo "Added $USER to group i2c."
fi

if [[ -n "${NEED_REBOOT:-}" ]]; then
  echo "Reboot, then: i2cdetect -y 1  (expect 0x40 for PCA9685)"
else
  echo "Log out and back in (or reboot) so group i2c applies."
  echo "Then: i2cdetect -y 1  (expect 0x40 for PCA9685)"
fi
