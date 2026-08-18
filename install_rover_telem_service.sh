#!/usr/bin/env sh
# Install always-on rover UDP telemetry recorder on dockerhost (Alpine + Docker).
set -eu

DIR="$(cd "$(dirname "$0")" && pwd)"
TELEM_DIR="${ROVER_TELEM_DIR:-/root/rover-telem}"
CONTAINER=rover-telem

mkdir -p "$TELEM_DIR"

if command -v systemctl >/dev/null 2>&1; then
  USER_NAME="${SUDO_USER:-$USER}"
  HOME_DIR="$(eval echo "~$USER_NAME")"
  TELEM_DIR="${ROVER_TELEM_DIR:-$HOME_DIR/rover-telem}"
  cat >/etc/systemd/system/rover-telem.service <<EOF
[Unit]
Description=LEGO Rover ESP telemetry recorder (UDP 4243)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$DIR
Environment=ROVER_TELEM_DIR=$TELEM_DIR
Environment=ROVER_TELEM_PORT=4243
ExecStart=/usr/bin/python3 $DIR/rover_telem_daemon.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable rover-telem.service
  systemctl restart rover-telem.service
  echo "systemd rover-telem → $TELEM_DIR"
  exit 0
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Need systemd or docker" >&2
  exit 1
fi

docker rm -f "$CONTAINER" 2>/dev/null || true
docker run -d --name "$CONTAINER" --restart unless-stopped \
  --network host \
  -v "$TELEM_DIR:/data" \
  -v "$DIR:/app:ro" \
  -e ROVER_TELEM_DIR=/data \
  -e ROVER_TELEM_PORT=4243 \
  python:3-alpine \
  python3 /app/rover_telem_daemon.py

echo "Docker $CONTAINER (host network UDP 4243) → $TELEM_DIR"
docker logs "$CONTAINER" 2>&1 | tail -3
