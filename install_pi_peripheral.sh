#!/usr/bin/env bash
# Install Pi peripheral-only service (speaker + LED ring). Disables rover-main if present.
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT=rover-pi-peripheral.service
TMP="/tmp/$UNIT"

cat >"$TMP" <<EOF
[Unit]
Description=LEGO Rover Pi peripherals (speaker + LED ring)
After=local-fs.target
Conflicts=rover-main.service rover-agent.service

[Service]
Type=simple
User=cain
WorkingDirectory=$HOME/lego-rover-ros2
Environment=ROVER_SPEAKER_GPIO=17
Environment=ROVER_SPEAKER_VOL=0.38
Environment=ROVER_RING_GPIO=18
Environment=ROVER_PI_SERIAL=/dev/ttyAMA0
Environment=ROVER_PI_PERIPH_BAUD=115200
ExecStart=/usr/bin/python3 $HOME/lego-rover-ros2/pi_peripheral_daemon.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo cp "$TMP" "/etc/systemd/system/$UNIT"
sudo systemctl daemon-reload
sudo systemctl disable rover-main.service 2>/dev/null || true
sudo systemctl stop rover-main.service 2>/dev/null || true
sudo systemctl disable rover-agent.service 2>/dev/null || true
sudo systemctl stop rover-agent.service 2>/dev/null || true
sudo systemctl enable "$UNIT"
sudo systemctl restart "$UNIT"
echo "Installed $UNIT — ESP UART @ 115200 for chimes + ring only."
