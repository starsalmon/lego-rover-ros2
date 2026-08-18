#!/usr/bin/env bash
# System service for WS2812 ring (needs root for rpi_ws281x DMA).
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT=/etc/systemd/system/rover-ring.service

if ! python3 -c 'import rpi_ws281x' 2>/dev/null; then
  echo "Installing rpi-ws281x..."
  if ! sudo apt-get install -y python3-rpi-ws281x 2>/dev/null; then
    sudo python3 -m pip install rpi-ws281x --break-system-packages
  fi
fi

sudo tee "$UNIT" >/dev/null <<EOF
[Unit]
Description=LEGO Rover WS2812 LED ring
After=local-fs.target

[Service]
Type=simple
User=root
Environment=ROVER_RING_GPIO=${ROVER_RING_GPIO:-18}
Environment=ROVER_RING_COUNT=${ROVER_RING_COUNT:-8}
Environment=ROVER_RING_BRIGHTNESS=${ROVER_RING_BRIGHTNESS:-24}
Environment=ROVER_RING_CTL=/tmp/rover_ring_ctl
ExecStart=/usr/bin/python3 $DIR/rover_ring_daemon.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

sudo chmod +x "$DIR/rover_ring_daemon.py" "$DIR/rover_ring.py"
sudo systemctl daemon-reload
sudo systemctl enable rover-ring.service
sudo systemctl restart rover-ring.service

sleep 1
if sudo systemctl is-active --quiet rover-ring.service; then
  python3 "$DIR/rover_ring.py" standby
  echo "rover-ring.service running — ring should show standby rainbow."
  echo "Test: python3 $DIR/rover_ring.py test"
else
  echo "ERROR: rover-ring.service failed:"
  sudo journalctl -u rover-ring.service -n 20 --no-pager
  exit 1
fi
