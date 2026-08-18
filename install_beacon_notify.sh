#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"
mkdir -p "$UNIT_DIR"

cat > "$UNIT_DIR/rover-beacon-notify.service" <<EOF
[Unit]
Description=Rover mini-beacon lock notification
After=network-online.target

[Service]
ExecStart=/usr/bin/env python3 $DIR/rover_beacon_notify.py
WorkingDirectory=$DIR
Restart=always
RestartSec=2

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable --now rover-beacon-notify.service
echo "rover-beacon-notify.service listening on UDP 4242"
