# PS4 controller (DualShock 4) on LEGO Rover Pi

## One-time setup

```bash
bash ~/lego-rover-ros2/install_extras.sh
# log out/in once (dialout + input groups)
```

## Pair over Bluetooth

**Unplug USB first** — the controller cannot BT-pair while on the cable.

1. On the phone: forget / disconnect **Wireless Controller** if it appears there
2. **Pairing mode:** hold **Share** + **PS** until the light bar flashes **rapidly** (fast white blink)
3. Keep controller **within 30 cm** of the Pi (Zero 2W BT is weak)
4. On Pi:
   ```bash
   sudo systemctl start bluetooth   # must be active
   bash ~/lego-rover-ros2/pair_ps4.sh
   ```
5. Light bar **solid** = connected. Check `ls /dev/input/js0`

If scan finds nothing, run manual pairing:
```bash
bluetoothctl
power on
agent NoInputNoOutput
default-agent
scan on
# wait for [NEW] Device XX:XX:XX:XX:XX:XX Wireless Controller
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
scan off
quit
```

### Easier first time: USB

Plug PS4 into Pi USB (hub/OTG). Check:
```bash
ls /dev/input/js*
jstest /dev/input/js0
```
Then pair BT — controller remembers the Pi.

### Manual pairing

```bash
bluetoothctl
power on
agent on
default-agent
scan on
# wait for "Wireless Controller"
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
```

## Drive (one command)

```bash
bash ~/lego-rover-ros2/go_ps4.sh
```

Or if agent is already running:

```bash
bash ~/lego-rover-ros2/run_teleop_ps4.sh
```

- **Left stick** = drive
- **Hold L1** = deadman (must hold to move)
- **R1** = turbo speeds

Uses `teleop_ps4.py` (reads `/dev/input/js0` + RELIABLE `/cmd_vel`).

USB is easiest — plug PS4 into the Pi, verify with:
```bash
ls /dev/input/js0
jstest /dev/input/js0
```

## Reconnect later

PS4 sometimes won't auto-reconnect on Pi Zero 2W BT:

```bash
bluetoothctl connect $(cat ~/.config/lego-rover/ps4_mac)
# or run pair_ps4.sh again
```

If it connects then drops immediately, remove and re-pair:
```bash
bluetoothctl remove <MAC>
bash ~/lego-rover-ros2/pair_ps4.sh
```

## Tune sticks

Edit `teleop_ps4.yaml` on the Pi:
- `invert_linear` / `invert_angular` if directions feel wrong
- `scale_linear` / `scale_angular` for speed
- `enable_button: 4` is L1; try `6` for L2 if needed

## Known Pi Zero 2W quirks

- **Bluetooth service must be running:** `sudo systemctl start bluetooth`
- Built-in Bluetooth is weak; keep controller close for pairing
- **Unplug USB** before BT pair; phone must not steal the controller
- USB Bluetooth dongle (BT 5.0+) often more reliable than onboard radio
- Compile/load can make BT flaky — pair **after** agent build is done

## Drive from Mac (PS4 over Bluetooth + WiFi)

**Does not start on boot.** Pi boots to button standby (autonomous). Mac teleop is manual when you want to drive from your laptop.

### One-time Mac setup

```bash
bash lego-rover-ros2/install_mac_teleop.sh   # ~15 min, ROS 2 Jazzy via RoboStack
```

Pair the PS4 to your **Mac** (System Settings → Bluetooth). The controller should show as **PS4 Controller** or **Wireless Controller**.

### Each drive session

1. Pi powered on, same WiFi as Mac
2. PS4 connected to **Mac** via Bluetooth (solid light bar)
3. Run:

```bash
bash lego-rover-ros2/go_mac.sh
```

- Stops Pi autonomous mode (keeps ESP32 link)
- **R2** = forward (trigger = speed), **L2** = reverse (wins if both pressed)
- **Left stick X** = steer while moving, spin on the spot when stopped
- Full power (no speed caps). Ctrl+C stops motors

Mac and Pi must be on the same network. `go_mac.sh` sends stick/trigger input to the Pi over **UDP port 9999** (reliable); the Pi publishes `/cmd_vel` locally to the ESP32. Set `ROVER_TELEOP_UDP=0` to fall back to SSH stdin relay.

Optional native ROS mode (experimental): `ROVER_MAC_DDS=1 bash lego-rover-ros2/go_mac.sh`
