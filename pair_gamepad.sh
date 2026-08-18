#!/usr/bin/env bash
# Interactive Bluetooth pairing for a gamepad (Xbox, PS4, Switch Pro, etc.)
set -eo pipefail

echo "=== LEGO Rover — pair Bluetooth gamepad ==="
echo ""
echo "Put the controller in pairing mode, then follow prompts."
echo "Common pairing buttons:"
echo "  Xbox: hold pairing button on top until LED blinks"
echo "  PS4: Share + PS until light bar blinks"
echo "  Switch Pro: hold sync button on top"
echo ""
read -r -p "Press Enter when the controller is in pairing mode..."

sudo bluetoothctl <<'EOF'
power on
agent on
default-agent
scan on
EOF

echo ""
echo "Watch for your controller name in the scan list (about 10 seconds)..."
sleep 10

read -r -p "Paste the controller MAC address (AA:BB:CC:DD:EE:FF): " MAC
if [[ -z "$MAC" ]]; then
  echo "No MAC entered — exiting."
  exit 1
fi

sudo bluetoothctl <<EOF
scan off
pair $MAC
trust $MAC
connect $MAC
quit
EOF

sleep 2
if [[ -e /dev/input/js0 ]]; then
  echo "OK: /dev/input/js0 exists — gamepad ready."
  jstest --normal /dev/input/js0 2>/dev/null | head -5 || true
else
  echo "Paired but no /dev/input/js0 yet. Try: sudo bluetoothctl connect $MAC"
  echo "Then: ls -l /dev/input/js*"
fi
