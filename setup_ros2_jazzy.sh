#!/usr/bin/env bash
# Ubuntu 24.04 Noble — ROS 2 Jazzy on Pi Zero 2W
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

sudo apt update
sudo apt install -y locales curl gnupg lsb-release software-properties-common git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-dev-tools \
  ros-jazzy-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  libcurl4-openssl-dev

sudo rosdep init 2>/dev/null || true
rosdep update

grep -q 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || cat >> ~/.bashrc <<'EOF'

source /opt/ros/jazzy/setup.bash
EOF

echo "ROS 2 Jazzy base installed. Next: bash ~/lego-rover-ros2/install_microros_agent.sh"
