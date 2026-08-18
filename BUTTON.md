# Start button + feedback

Wiring tables and pin conflicts: [`WIRING.md`](WIRING.md)

## Button actions (default: ESP)

| Button | Action |
|--------|--------|
| **Go — tap** | Start / stop autonomous session |
| **Go — hold ~0.8 s** (standby only) | Open/close **mode menu** (Explore / Wall) |
| **Go — double tap** (standby only) | Open **power menu** (Shutdown / Reboot / Cancel) |
| **Go — tap in mode menu** | Cycle Explore ↔ Wall |
| **Go — tap in power menu** | Cycle Shutdown → Reboot → Cancel |
| **Go — hold in menu** | **Confirm** (mode: save; power: run selected action) |
| **E-stop — hold** | Cut motors on ESP (release to drive again) |
| **E-stop — hold ~3 s** | Stop Pi session (return to standby) |

**Mode menu** — blue full-screen overlay. After **hold to exit**, standby shows `Mode: Wall` and **Tap Go -->** for 6 seconds.

**Power menu** — red full-screen overlay. Shuts down or reboots the **Pi only** (ESP stays powered). Requires passwordless `sudo` for shutdown/reboot — installed by `install_rover_boot.sh`.

Each button action plays a short beep on the Pi buzzer (GPIO **17**).

### ROS topics (ESP → Pi)

| Topic | Type | Meaning |
|-------|------|---------|
| `/rover/button` | `Bool` | Short Go press (start/stop) |
| `/rover/button_event` | `UInt8` | See event codes below |
| `/rover/drive_mode` | `UInt8` | `0`=explore, `1`=wall |

**`/rover/button_event` codes:** `1`=Go short, `2`=Go long, `3`=E-stop long, `4`=double tap (power menu open), `5`=shutdown confirm, `6`=reboot confirm

### Double-tap timing

Two quick taps on **Go (GPIO14)** within ~1.2 s opens the power menu immediately on the 2nd release. Standby only — a single tap waits ~1.2 s before start/stop (in case you are doing a double tap). While driving, Go tap stops immediately (no multi-tap detection).

## Hardware

| Button | ESP GPIO | Location |
|--------|----------|----------|
| **Go / start-stop** | **14** | Top-right KEY |
| **E-stop** | **0** | Bottom-right BOOT |

The ESP publishes button events via `rover_esp_button.py` (default `ROVER_BUTTON_SOURCE=esp`).

### Legacy Pi GPIO button (optional)

Set `ROVER_BUTTON_SOURCE=pi` and wire a momentary switch:

| Button leg | Pi pin |
|------------|--------|
| 1 | **BCM GPIO 17** — **physical pin 11** |
| 2 | **GND** (physical pin **9**) |

**Do not use** GPIO 14/15 (UART to ESP32). **Physical pin 18 is BCM GPIO 24** (PCA OE), not GPIO 18.

### Speaker (passive buzzer)

| Buzzer leg | Pi pin |
|------------|--------|
| + | **GPIO 17** (physical pin **11**) |
| − | **GND** (e.g. pin **9**) |

GPIO 17 is the production default (`ROVER_SPEAKER_GPIO=17` in `rover-main.service`). Use a **passive** piezo buzzer (not an active beeper that only clicks).

Disable: `export ROVER_SPEAKER_GPIO=0`

Volume: `export ROVER_SPEAKER_VOL=0.25` (default; above ~0.35 gets harsh on this buzzer)

### LED ring (8× WS2812)

Wraps around the start button. Data in on **GPIO 18** (physical pin **12**).

| Ring wire | Pi |
|-----------|-----|
| DIN | GPIO **18** (pin **12**) |
| VCC | **5V** (pin **2** or **4**) |
| GND | **GND** (pin **6** / **9** / **14** …) |

Add a **330Ω** resistor in series on DIN if you have one. For 8 LEDs at low brightness, Pi 5V is usually fine.

Disable: `export ROVER_RING_GPIO=0`  
Brightness: `export ROVER_RING_BRIGHTNESS=24` (default; max 255)

Test on Pi: `python3 ~/lego-rover-ros2/rover_ring.py test`  
If dark: `sudo python3 ~/lego-rover-ros2/rover_ring.py test` (DMA needs mem access on some images)

Install driver: `pip3 install rpi-ws281x` (or re-run `install_rover_boot.sh`)

## Sound cues

| Event | When |
|-------|------|
| **Ready** (boot) | Pi booted + ESP32 linked — C5→E5→G5 |
| **Autonomous start** | Button pressed to go | C4→E4→G4 |
| **Autonomous stop** | Button pressed to stop | G4→E4→C4 |
| **Bump** | Hit something while driving | A4→C5→A4 |
| **Stall** | Wheels on, not moving | A4→A4→F4 |

You only hear **start** if you weren't already driving. **Bump/stall** only when those events fire (not every few seconds).

Test: `python3 ~/lego-rover-ros2/rover_speaker.py ready`

## LED ring cues

| Pattern | When |
|---------|------|
| Slow rainbow breathe | Standby (waiting for button) |
| Green ripple | Boot ready (ESP linked) |
| Dual rainbow chase | Autonomous driving |
| Orange sweep | Escape (reverse / spin) |
| Red strobe flash | Bump |
| Yellow pulse flash | Stall |
| Red centre wipe | Stop |

## Boot flow

1. Pi boots → `rover-agent.service` connects ESP32 over UART (always on)
2. `rover-main.service` → ready chime + ring ripple when ESP linked
3. Ring shows slow rainbow — **hold Go** for mode, **double tap** for power, **tap Go** to start
4. `go.sh` or `go_wall.sh` / `rover_session.py` runs autonomous
5. Tap Go again → stop motors, descending chime, ring wipe → standby

## Install (once on Pi)

```bash
bash ~/lego-rover-ros2/sync_to_pi.sh   # from Mac
bash ~/lego-rover-ros2/install_rover_boot.sh
sudo reboot
```

## Manual test (no reboot)

```bash
python3 ~/lego-rover-ros2/wait_for_button.py   # press button
bash ~/lego-rover-ros2/go.sh
```

## Different button source

```bash
# Legacy wired Pi button on GPIO 17:
export ROVER_BUTTON_SOURCE=pi
export ROVER_BUTTON_GPIO=17
bash ~/lego-rover-ros2/install_rover_boot.sh
```
