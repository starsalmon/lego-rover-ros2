# 3-Hour Plan — See the robot move

## Goal
Smooth drive + spin with accel/decel. IMU publishing. ROS 2 cmd_vel over UART to ESP32.

## Hour 1 — Pi ROS + ESP32 motors spin (hardware proof)

**Mac:**
```bash
scp -r /Users/cain.davidson/Documents/cursor-esp32/lego-rover-ros2 cain@lego-rover.local:~/
scp -r /Users/cain.davidson/Documents/cursor-esp32/lego-rover-esp32 cain@lego-rover.local:~/
```

**Pi (one block, ~30-45 min install):**
```bash
bash ~/lego-rover-ros2/setup_ros2_jazzy.sh
source ~/.bashrc
```

**Mac — flash ESP32 motor test (no ROS yet):**
```bash
cd /Users/cain.davidson/Documents/cursor-esp32/lego-rover-esp32
pio run -e c3_motor_test -t upload
pio device monitor
```

You should see motors ramp forward / spin / stop. **If this fails, fix wiring before Hour 2.**

## Hour 2 — micro-ROS link + cmd_vel drives motors

**Wire UART:** Pi GPIO14(TX)→ESP RX, GPIO15(RX)→ESP TX, GND.

**Pi — enable UART (if needed):**
```bash
bash ~/lego-rover-ros2/enable_uart.sh
sudo reboot
```

**Flash ESP32 micro-ROS firmware (Mac):**
```bash
pio run -e c3_microros -t upload
```

**Pi — terminal 1:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 115200
```

**Pi — terminal 2 (after "agent connected"):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Robot should move.

## Hour 3 — Smooth demo + IMU

**Pi:**
```bash
bash ~/lego-rover-ros2/run_demo.sh
```

**Watch IMU:**
```bash
ros2 topic echo /imu/data --field angular_velocity
```

## What "smooth" means today
- ESP32 ramps PWM (trapezoid profile) — no jerk on start/stop
- Pi demo publishes smoothstep cmd_vel curves
- IMU streams orientation; firmware reduces speed if tilt > threshold

## Not in 3 hours (later)
- Full closed-loop balance
- Nav2, SLAM, Scratch bridge
