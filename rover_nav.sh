#!/usr/bin/env bash
# Nav2 + slam_toolbox. Wheels still only move when rover_brain forwards cmd_vel.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
python3 /opt/fleet/prepare_nav2_params.py
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml /tmp/slam.yaml
sed -i 's/base_footprint/base_link/g' /tmp/slam.yaml
sed -i 's/minimum_travel_distance: 0.5/minimum_travel_distance: 0.15/' /tmp/slam.yaml
sed -i 's/minimum_travel_heading: 0.5/minimum_travel_heading: 0.15/' /tmp/slam.yaml
sed -i 's/map_update_interval: 5.0/map_update_interval: 2.0/' /tmp/slam.yaml
python3 /opt/fleet/rover_nav_sensors.py &
python3 /opt/fleet/rover_nav_goals.py &
exec ros2 launch nav2_bringup bringup_launch.py \
  slam:=True \
  use_sim_time:=False \
  autostart:=True \
  params_file:=/tmp/nav2_params.yaml \
  slam_params_file:=/tmp/slam.yaml
