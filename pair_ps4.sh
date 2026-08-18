#!/usr/bin/env bash
# Pair a PS4 DualShock 4 over Bluetooth on the Pi.
set -eo pipefail

MAC="${PS4_MAC:-}"
TIMEOUT="${SCAN_SECS:-40}"

echo "=== LEGO Rover — pair PS4 (DualShock 4) ==="
echo ""
echo "BEFORE you start:"
echo "  • UNPLUG the USB cable (controller must be wireless-only for BT pair)"
echo "  • Forget / disconnect the controller on your phone"
echo "  • Hold SHARE + PS until the light bar flashes RAPIDLY (not slow pulse)"
echo ""

sudo systemctl enable bluetooth >/dev/null 2>&1 || true
sudo systemctl start bluetooth
sleep 1

if ! systemctl is-active --quiet bluetooth; then
  echo "ERROR: bluetooth service will not start."
  echo "  Try: sudo systemctl restart bluetooth"
  echo "  Check: journalctl -u bluetooth -n 20"
  exit 1
fi

# PS4 uses Just Works pairing — no PIN prompt
sudo bluetoothctl <<'EOF'
power on
agent NoInputNoOutput
default-agent
pairable on
discoverable on
EOF

if [[ -n "$MAC" ]]; then
  echo "Using MAC from PS4_MAC=$MAC"
else
  echo "Scanning ${TIMEOUT}s — keep controller in pairing mode, close to the Pi..."
  echo ""

  # Fresh scan list
  sudo bluetoothctl scan on &
  SCAN_PID=$!
  sleep "$TIMEOUT"
  kill "$SCAN_PID" 2>/dev/null || true
  sudo bluetoothctl scan off 2>/dev/null || true

  echo "Devices seen:"
  bluetoothctl devices 2>/dev/null | sed 's/^/  /' || true
  echo ""

  MAC=$(bluetoothctl devices 2>/dev/null | grep -i 'Wireless Controller' | head -1 | awk '{print $2}')
  if [[ -z "$MAC" ]]; then
    MAC=$(bluetoothctl devices 2>/dev/null | grep -i 'DualShock' | head -1 | awk '{print $2}')
  fi
fi

if [[ -z "$MAC" ]]; then
  echo "No PS4 found."
  echo ""
  echo "Try:"
  echo "  1. Unplug USB, forget controller on phone, pairing mode again"
  echo "  2. Pi and controller within 30 cm (Zero 2W BT is weak)"
  echo "  3. Manual: bluetoothctl → scan on → pair <MAC> → trust → connect"
  echo "  4. If you see the MAC above: PS4_MAC=AA:BB:CC:DD:EE:FF bash ~/lego-rover-ros2/pair_ps4.sh"
  exit 1
fi

echo "Pairing $MAC ..."
sudo bluetoothctl remove "$MAC" 2>/dev/null || true
sleep 1

PAIR_OUT=$(sudo bluetoothctl <<EOF
agent NoInputNoOutput
default-agent
pair $MAC
trust $MAC
connect $MAC
EOF
)
echo "$PAIR_OUT"

sleep 3
sudo modprobe hid-playstation 2>/dev/null || sudo modprobe hid-sony 2>/dev/null || true
sleep 2

# Retry connect once
if ! bluetoothctl info "$MAC" 2>/dev/null | grep -q 'Connected: yes'; then
  echo "Retrying connect..."
  sudo bluetoothctl connect "$MAC" || true
  sleep 2
fi

sudo usermod -aG input "$USER" 2>/dev/null || true
mkdir -p ~/.config/lego-rover
echo "$MAC" > ~/.config/lego-rover/ps4_mac

if [[ -e /dev/input/js0 ]]; then
  echo ""
  echo "OK: /dev/input/js0 ready (Bluetooth)"
  echo "Saved MAC: $MAC"
  echo ""
  echo "Drive: bash ~/lego-rover-ros2/go_ps4.sh"
  echo "Test:  jstest /dev/input/js0"
else
  echo ""
  echo "Paired but no /dev/input/js0 yet."
  bluetoothctl info "$MAC" 2>/dev/null | grep -E 'Connected|Paired|Trusted' || true
  echo ""
  echo "Try:"
  echo "  sudo bluetoothctl connect $MAC"
  echo "  sudo modprobe hid-playstation"
  echo "  ls /dev/input/js*"
  echo ""
  echo "Pi Zero 2W onboard BT is weak — a USB BT 5.0 dongle is more reliable."
fi
