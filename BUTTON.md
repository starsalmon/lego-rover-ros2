# Buttons + feedback (T-Display drive board)

Pins: [`lego-rover-esp32/WIRING.md`](../lego-rover-esp32/WIRING.md).

## Buttons

| Button | Action |
|--------|--------|
| **Go — tap** | Start / stop the brain session (`/rover/button`) |
| **Go — hold ~0.8 s** (standby) | Open/close **mode menu** (Explore / Wall) |
| **Go — double tap** (standby) | Open the power menu (Shutdown / Reboot / Cancel) |
| **Go — tap in a menu** | Cycle the option |
| **Go — hold in a menu** | Confirm |
| **E-stop (GPIO 0) — hold** | Cut motors on the ESP (release to drive again) |
| **E-stop — hold ~3 s** | Stop the brain session |

Go taps are ignored by the brain while `allow_drive` is OFF.

## ROS topics (drive board → brain)

| Topic | Type | Meaning |
|-------|------|---------|
| `/rover/button` | Bool | Go tap |
| `/rover/button_event` | UInt8 | 1 Go short, 2 Go long, 3 E-stop long, 4 Go multi-tap, 5 power shutdown, 6 power reboot, 7 menu cycle |
| `/rover/drive_mode` | UInt8 | 0 Explore, 1 Wall |

The brain acts on `/rover/button`, E-stop long (3) and `/rover/drive_mode`. The power-menu events (5/6) are published but nothing on the brain acts on them.

## Feedback (on the drive board)

- **Speaker (GPIO 17):** ready on agent connect, session start/stop, bump, stall, button/menu chirps.
- **LED ring (GPIO 18):** distance colours from the Pro S3 ToF (front LEDs from the L8 columns, side LEDs from the side ToF; near = orange, far = blue), plus bump/stall flashes.
- **Display:** link state, battery, cmd_vel, mode.
