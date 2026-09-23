# LEGO Rover — Pi wiring (Raspberry Pi Zero 2W)

Motors, IMU, body MCP23008, pan servo (PCA9685 ch 0), and IR chase beacon are on the **ESP32** — see [`lego-rover-esp32/WIRING.md`](../lego-rover-esp32/WIRING.md).

This doc is **Pi GPIO only**: UART to ESP, buzzer, LED ring, and optional legacy wiring.

**Production defaults** (`install_rover_boot.sh` / `rover-main.service`):

| Setting | Value |
|---------|--------|
| `ROVER_BUTTON_SOURCE` | `esp` — Go/E-stop on ESP, not Pi GPIO |
| `ROVER_IR_SOURCE` | `esp` — wheel IR on body MCP (no front bumper board) |
| `ROVER_SERVO_SOURCE` | `esp` — rear pan sonar on PCA9685 ch 0 |

Always use **BCM GPIO numbers** in software.

---

## What the Pi actually wires today

| Function | BCM GPIO | Physical pin | Direction |
|----------|----------|--------------|-----------|
| UART TX → ESP | **14** | **8** | — |
| UART RX ← ESP | **15** | **10** | — |
| Passive buzzer **+** | **17** | **11** | OUTPUT (PWM) |
| WS2812 ring **DIN** | **18** | **12** | OUTPUT |
| GND | — | **6 / 9 / 14 / …** | — |
| 5 V (ring) | — | **2** or **4** | — |

**Do not repurpose GPIO 14 or 15** — micro-ROS serial to the ESP.

Start/stop is the **ESP Go button (GPIO14 on T-Display-S3)**, not a Pi pin. See [`BUTTON.md`](BUTTON.md).

---

## Pi ↔ ESP32 (UART / micro-ROS)

| Pi header | BCM | T-Display-S3 (production) |
|-----------|-----|---------------------------|
| Pin **8** (TX) | GPIO **14** | → **IO17** (ESP RX) |
| Pin **10** (RX) | GPIO **15** | ← **IO18** (ESP TX) |
| **GND** | GND | GND |

**Baud:** **460800** 8N1 — must match `rover-agent.service` and ESP firmware.

One-time: `bash ~/lego-rover-ros2/enable_uart.sh` then reboot.

*(Alternate board: ESP32-C3 Zero uses Pi TX→**IO20**, Pi RX←**IO21** — see ESP wiring doc.)*

---

## Passive buzzer (speaker)

| Buzzer | Pi |
|--------|-----|
| **+** | GPIO **17** (physical pin **11**) |
| **−** | **GND** |

Passive piezo only (PWM tones via gpiozero). Disable: `ROVER_SPEAKER_GPIO=0`

Volume: `ROVER_SPEAKER_VOL=0.38` (default in `rover-main.service`).

---

## LED ring (8× WS2812)

| Ring wire | Pi |
|-----------|-----|
| **DIN** | GPIO **18** (pin **12**) — 330 Ω in series if you have one |
| **VCC** | **5 V** (pin **2** or **4**) |
| **GND** | **GND** |

Disable: `ROVER_RING_GPIO=0`

Ring daemon: `rover-ring.service` (root). After wiring changes: `sudo systemctl restart rover-ring`.

---

## Not on the Pi (handled by ESP)

These appear in older notes / env vars but are **not used** when `ROVER_IR_SOURCE=esp` and `ROVER_SERVO_SOURCE=esp`:

| Legacy Pi GPIO | Was |
|----------------|-----|
| 6, 22 | Front IR emitter / detect |
| 26, 27 | Aux IR emitter / detect |
| 2, 3, 24 | Pi I2C → PCA9685 (servo) |

IR, wheel encoders, front bumper, aux sweep servo, and **38 kHz chase beacon** are on the **ESP I2C bus** (MPU6050 + MCP23008 + PCA9685). Pi software talks over ROS topics (`/rover/ir/front`, `/rover/servo/angle`, etc.).

---

## Legacy: Pi GPIO IR + PCA9685

Only if you deliberately set `ROVER_IR_SOURCE=pi` and/or `ROVER_SERVO_SOURCE=pi` (bench / old harness).

<details>
<summary>Click to expand legacy Pi IR + servo wiring</summary>

### Front IR (Pi GPIO)

| Signal | BCM | Physical pin |
|--------|-----|--------------|
| OUT (detect) | **22** | 15 |
| LED (emitter pulse) | **6** | 31 |

### Aux IR (Pi GPIO)

| Signal | BCM | Physical pin |
|--------|-----|--------------|
| OUT (detect) | **27** | 13 |
| LED (emitter) | **26** | 37 → N-MOSFET low-side switch |

### PCA9685 on Pi I2C bus 1

| PCA9685 | Pi |
|---------|-----|
| SDA | GPIO **2** (pin 3) |
| SCL | GPIO **3** (pin 5) |
| VCC | 3.3 V |
| V+ | 5 V (servo rail) |
| OE | GPIO **24** (pin 18) — or tie OE→GND and set `ROVER_PCA9685_OE_GPIO=0` |
| Servo | Channel **0** (OUT0 SIG) |

`enable_i2c.sh` + `i2cdetect -y 1` should show **0x40**.

</details>

---

## Legacy: Pi GPIO start button

Set `ROVER_BUTTON_SOURCE=pi`:

| Button leg | Pi |
|------------|-----|
| 1 | BCM **GPIO 17** (physical pin **11**) |
| 2 | **GND** (pin **9**) |

**Physical pin 11 is GPIO 17, not 18.** Pin 18 is BCM **24** (was PCA OE on old builds).

---

## 40-pin header (active pins only)

```
     3.3V  (1) (2)  5V      ← ring VCC
   GPIO 2  (3) (4)  5V
   GPIO 3  (5) (6)  GND
   GPIO 4  (7) (8)  GPIO 14  TX → ESP
     GND   (9)(10)  GPIO 15  RX ← ESP
  GPIO 17  (11)(12) GPIO 18  ← buzzer + / ring DIN
  GPIO 27  (13)(14) GND
  GPIO 22  (15)(16) GPIO 23
   ...
```

---

## Power

- **Pi 5 V** from motor driver **5Vo** or dedicated supply — not ESP USB while driving.
- **Common GND** with ESP, driver VMOT−, and sensors.
- Ring uses **5 V**; buzzer and UART use 3.3 V logic.

---

## Environment overrides (production)

| Variable | Default | Purpose |
|----------|---------|---------|
| `ROVER_BUTTON_SOURCE` | `esp` | `pi` = GPIO 17 button |
| `ROVER_IR_SOURCE` | `esp` | `pi` = GPIO 6/22/26/27 |
| `ROVER_SERVO_SOURCE` | `esp` | `pi` = PCA9685 on Pi I2C |
| `ROVER_SPEAKER_GPIO` | `17` | Buzzer (`0` = off) |
| `ROVER_RING_GPIO` | `18` | WS2812 data (`0` = off) |
| `ROVER_SPEAKER_VOL` | `0.38` | Buzzer volume |

After changing boot env:

```bash
bash ~/lego-rover-ros2/install_rover_boot.sh
systemctl --user restart rover-main.service
```

---

## Related docs

- [`BUTTON.md`](BUTTON.md) — Go / E-stop / menus
- [`lego-rover-esp32/WIRING.md`](../lego-rover-esp32/WIRING.md) — motors, IMU, MCP IR, servo, beacon
- [`DEMO.md`](DEMO.md) — preflight runbook
