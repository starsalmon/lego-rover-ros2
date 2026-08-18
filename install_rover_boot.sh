#!/usr/bin/env bash
# Boot setup: always-on micro-ROS agent + button-to-start main loop.
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"
MAIN_UNIT="$UNIT_DIR/rover-main.service"

echo "=== Install LEGO Rover boot services ==="

if ! sudo apt-get install -y python3-gpiozero python3-lgpio python3-smbus i2c-tools; then
  echo "ERROR: could not install python3-gpiozero / python3-lgpio / python3-smbus"
  exit 1
fi

if getent group i2c >/dev/null; then
  sudo usermod -aG i2c "$USER" 2>/dev/null || true
fi

if getent group gpio >/dev/null; then
  sudo usermod -aG gpio "$USER" 2>/dev/null || true
fi

# WS2812 ring — root systemd service (DMA)
bash "$DIR/install_rover_ring_service.sh" || echo "WARN: ring service install failed"

bash "$DIR/install_agent_service.sh"

SUDOERS_FILE="/etc/sudoers.d/rover-power"
if [[ ! -f "$SUDOERS_FILE" ]]; then
  echo "Installing passwordless shutdown/reboot for power menu..."
  echo "$USER ALL=(ALL) NOPASSWD: /usr/sbin/shutdown, /sbin/shutdown, /usr/sbin/reboot, /sbin/reboot" | sudo tee "$SUDOERS_FILE" >/dev/null
  sudo chmod 440 "$SUDOERS_FILE"
fi

mkdir -p "$UNIT_DIR"

cat >"$MAIN_UNIT" <<EOF
[Unit]
Description=LEGO Rover standby (button to start)
After=network-online.target rover-agent.service
Wants=rover-agent.service

[Service]
Type=simple
WorkingDirectory=$HOME/lego-rover-ros2
Environment=ROVER_BRIDGE_EXTERNAL=1
Environment=ROVER_BUTTON_SOURCE=esp
Environment=ROVER_IR_SOURCE=esp
Environment=ROVER_SERVO_SOURCE=esp
Environment=ROVER_SPEAKER_GPIO=17
Environment=ROVER_SPEAKER_VOL=0.38
Environment=ROVER_CRUISE_RADAR_PING=1
Environment=ROVER_RING_GPIO=18
Environment=ROVER_RING_COUNT=8
Environment=ROVER_RING_BRIGHTNESS=24
Environment=ROVER_AUTO_MAX_LINEAR=0.30
Environment=ROVER_IR_AUX_LED_GPIO=26
Environment=ROVER_IR_FRONT_LED_GPIO=6
Environment=ROVER_IR_FRONT_PULSE=1
Environment=ROVER_IR_FRONT_OFF_MS=50
Environment=ROVER_IR_FRONT_ON_MS=10
Environment=ROVER_PCA9685_OE_GPIO=24
Environment=ROVER_PCA9685_OE_ACTIVE_LOW=1
ExecStart=/bin/bash $HOME/lego-rover-ros2/rover_main.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
EOF

chmod +x "$DIR/rover_main.sh" "$DIR/wait_for_button.py" "$DIR/go.sh" \
  "$DIR/rover_session.py" "$DIR/rover_esp_button.py" "$DIR/rover_bridge_daemon.py" \
  "$DIR/rover_ring.py" "$DIR/rover_ring_daemon.py" \
  "$DIR/install_rover_ring_service.sh"

systemctl --user daemon-reload
systemctl --user enable rover-main.service
systemctl --user restart rover-main.service

if ! loginctl show-user "$USER" -p Linger 2>/dev/null | grep -q 'yes'; then
  sudo loginctl enable-linger "$USER"
  echo "Enabled linger (services run without login)."
fi

echo ""
echo "Installed."
echo "  Start/stop: ESP GPIO14 (top-right KEY) via /rover/button — not Pi GPIO"
echo "  Speaker: GPIO 17 (pin 11) — buzzer + to pin 11, − to GND"
echo "  LED ring: GPIO 18 (physical pin 12) — 8× WS2812 DIN, VCC 5V, GND"
echo "  Agent:  systemctl --user status rover-agent"
echo "  Main:   systemctl --user status rover-main"
echo "  Logs:   journalctl --user -u rover-main -f"
echo ""
echo "After reboot the rover waits idle until you press the button."
