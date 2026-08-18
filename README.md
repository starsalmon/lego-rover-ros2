# LEGO Rover — Pi scripts (legacy ROS stack)

**On-robot brain moved to ESP32.** The Pi is now a **peripheral co-processor** only.

## Current (use this)

| Doc / script | Purpose |
|--------------|---------|
| [PI_PERIPHERAL.md](PI_PERIPHERAL.md) | Architecture — ESP brain, Pi = speaker + ring |
| `pi_peripheral_daemon.py` | UART listener → chimes + WS2812 |
| `install_pi_peripheral.sh` | systemd service, disables old agent/main |
| [ROVER_BEHAVIOR.md](ROVER_BEHAVIOR.md) | Drive / sonar / stall logic (ESP) |

**ESP firmware:** `../lego-rover-esp32` — OTA env `s3_tdisplay_microros_rover_ota`.

## Legacy (Pi as ROS brain — deprecated on-robot)

Not used for normal driving anymore. Kept for dockerhost / Mac teleop / capture experiments.

| Area | Examples |
|------|----------|
| Autonomous | `go_auto.sh`, `autonomous_explore.py`, `rover_main.sh` |
| micro-ROS | `rover-agent.service`, `install_agent_service.sh` |
| Capture | `record_sonar_escape_session.sh` |
| Teleop | `go.sh`, `teleop_mac_ssh.py` |

Fleet ROS can run on **dockerhost** with ESP over **WiFi** later (same pattern as `c3-mini-bot`).
