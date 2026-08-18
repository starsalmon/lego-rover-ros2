#!/usr/bin/env bash
# Install micro-ROS agent as a systemd user service (starts on login / boot).
set -eo pipefail

UNIT_DIR="$HOME/.config/systemd/user"
UNIT_FILE="$UNIT_DIR/rover-agent.service"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
AGENT_SETUP="$HOME/uros_agent_ws/install/local_setup.bash"

mkdir -p "$UNIT_DIR"

cat >"$UNIT_FILE" <<EOF
[Unit]
Description=LEGO Rover micro-ROS serial agent
After=network.target

[Service]
Type=simple
Environment=SERIAL_DEV=/dev/ttyAMA0
Environment=BAUD=460800
ExecStart=/bin/bash -lc 'source $ROS_SETUP && source $AGENT_SETUP && exec ros2 run micro_ros_agent micro_ros_agent serial --dev \${SERIAL_DEV} -b \${BAUD}'
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable rover-agent.service
systemctl --user start rover-agent.service

echo "rover-agent.service installed and started."
echo "  status: systemctl --user status rover-agent"
echo "  logs:   journalctl --user -u rover-agent -f"
echo ""
echo "Enable lingering so it runs without login:"
echo "  sudo loginctl enable-linger \$USER"
