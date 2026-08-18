# Pi on the rover — ROS brain + peripherals

The **Raspberry Pi** runs autonomous explore, micro-ROS agent (UART to ESP), speaker, and LED ring.

The **ESP32-S3** is a thin client: motors, IMU, sonar pan, wheel IR, E-stop, battery, stall cutout, display.

## Pi responsibilities

| Function | Notes |
|----------|--------|
| micro-ROS agent | Serial to ESP @ **460800** (`rover-agent.service`) |
| `autonomous_explore.py` | Wander, escape, cruise radar — publishes `/cmd_vel` |
| Speaker + WS2812 ring | `rover_ring_daemon.py`, `rover_speaker.py` |
| Session / Go button bridge | `rover_session.py`, `go_auto.sh` |

## ESP responsibilities

| Function | Notes |
|----------|--------|
| `/cmd_vel` subscriber | Drives motors with ramp + heading hold |
| Sensor publishers | `/rover/sonar/*`, IMU, wheel ticks, IR |
| Emergency brake | Hard stop when closing on obstacle (< ~0.55 m trend or < ~0.20 m) |
| Safety | E-stop, low battery, stall lockout |

Escape manoeuvres (reverse, spin, drive-out) are on the **Pi**, not a multi-phase FSM on the ESP.

## Quick start

```bash
ssh lego-rover 'bash ~/lego-rover-ros2/go_auto.sh'
```

Or tap **Go** on the ESP if `rover-main.service` is in standby.

## Flash ESP (OTA)

```bash
cd lego-rover-esp32
pio run -e s3_tdisplay_microros_rover_ota -t upload
```

Builds `microros_main.cpp` (not standalone wander).

## Fleet context

Minis use dockerhost `fleet-brain`. The LEGO rover keeps its brain on the Pi (`/rover/…` topics). See `../c3-mini-bot/docs/SWARM.md`.
