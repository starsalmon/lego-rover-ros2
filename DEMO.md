# LEGO Rover — demo runbook

Pi brain + ESP32 motors over ROS 2 micro-ROS (UART). Six-hour rebuild checklist.

## One-time setup (Pi)

```bash
bash ~/lego-rover-ros2/setup_ros2_jazzy.sh          # if not done
bash ~/lego-rover-ros2/enable_uart.sh && sudo reboot
bash ~/lego-rover-ros2/install_microros_agent.sh    # ~15–30 min on Zero 2W
bash ~/lego-rover-ros2/install_extras.sh            # joy + bluetooth
```

Log out/in after `install_extras.sh` (dialout group).

## Before every demo

```bash
bash ~/lego-rover-ros2/preflight.sh
```

Hardware:
- ESP32 on driver 5V (not USB to Mac during demo)
- UART: Pi pin 8 → ESP IO20, Pi pin 10 ← ESP IO21, GND common
- VMOT on for wheel motion

Flash ESP32 from Mac (once per firmware change):

```bash
cd lego-rover-esp32
pio run -e c3_microros -t upload
```

## Demo modes

### 1. Autonomous showcase (best for an audience)

```bash
bash ~/lego-rover-ros2/run_demo.sh
```

28s loop: pause → forward → S-curve → spin → reverse arc → finish.  
Classic 16s loop: `DEMO_MODE=classic bash ~/lego-rover-ros2/run_demo.sh`

### 2. PS4 gamepad drive (interactive)

```bash
bash ~/lego-rover-ros2/pair_ps4.sh          # first time only
bash ~/lego-rover-ros2/go_ps4.sh            # agent + teleop
```

Hold **L1** + left stick to drive. See `PS4_CONTROLLER.md`.

### 3. Plan B — no Pi

```bash
pio run -e c3_motor_test -t upload
```

## Show IMU live (optional second SSH window)

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /imu/data --field angular_velocity.z
```

## Sync scripts from Mac to Pi

```bash
bash lego-rover-ros2/sync_to_pi.sh
```

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Package micro_ros_agent not found` | Finish agent build: `tail -f ~/agent_install.log` until done |
| Agent exits / no motion | `groups` must include `dialout`; reboot after usermod |
| ESP retry loop on USB serial | Normal without Pi UART — start agent on Pi |
| Gamepad no `/dev/input/js0` | `pair_gamepad.sh` or `bluetoothctl connect MAC` |
