# LEGO Rover — behavior & priority map

**Brain:** Raspberry Pi (`autonomous_explore.py`). **ESP:** motors, sensors, emergency brake — see [PI_PERIPHERAL.md](PI_PERIPHERAL.md).

Plain-English reference for **what the robot does**, **who decides**, and **what wins when things conflict**.

Hardware pins: `../lego-rover-esp32/WIRING.md`  
Firmware: `../lego-rover-esp32/src/microros_main.cpp`, `rover_sonar.cpp` (edge safety only)  
Pi stack: `autonomous_explore.py`, `rover_radar.py`, `go_auto.sh`

**Fleet:** bot0 = LEGO rover. ESP hostname `rover-esp.local`.

---

## Golden rule

**The Pi publishes motion. The ESP is the last word on whether the wheels spin.**

`autonomous_explore.py` sets cruise and escape via `/cmd_vel`. The ESP applies E-stop, battery, stall lockout, and **immediate reverse brake** when the **aft** sonar shows an imminent hit. Nose is VL53L8CX / front IR. Full escape runs on dockerhost.

---

## Priority ladder (highest wins)

| Priority | Source | What happens |
|----------|--------|----------------|
| **1** | **E-stop** (BOOT button held, GPIO 0) | Motors off every loop |
| **2** | **Low battery** (ADC on ESP) | Same as E-stop |
| **3** | **Stall lockout** (4 s after ESP stall) | `drive_blocked()` — motors off |
| **4** | **Sonar hard brake** (ESP) | Zeros **reverse** `cmd_vel` when the aft cone is too close |
| **5** | **Front IR cut** (ESP, when not braking) | Zeros forward if front beam tripped |
| **6** | **Heading hold** (ESP IMU) | Small trim to keep straight |
| **7** | **Pi `/cmd_vel`** | Teleop, autonomous explore, capture scripts |
| **8** | **Cmd timeout** (400 ms with no fresh `cmd_vel`) | Stop |

**Detectors (do not drive motors by themselves):**

| Event | Who detects | Who reacts |
|-------|-------------|------------|
| **Bump** | ESP IMU jerk | Pi `autonomous_explore` → escape |
| **Stall** | ESP IMU + wheel ticks | ESP cuts motors + lockout; Pi may also escape |
| **Sonar close** | ESP HC-SR04 | ESP brakes forward; Pi escape FSM handles manoeuvre |

---

## ESP main loop order

Each iteration when micro-ROS is connected (~5–10 ms):

```
1. E-stop / battery        → force_stop if bad
2. sonar.tick()            → may zero reverse lin (aft hard brake only)
3. apply_drive()           → commands motors (respects drive_blocked)
4. motion.update()         → bump / stall detection (IMU + wheel ticks)
5. on stall                → force_stop("stall"), 4 s lockout
```

Pan sonar (tail, 180° rotate) uses a ±70° sine sweep around aft during cruise. No 180° sit-and-scan on the ESP. Aux head servo is gone.

---

## `/cmd_vel` — what forward and turn actually mean

### Production (rover OTA env → `microros_main.cpp`)

**Standard ROS**, no axis swap:

| Wire field | Meaning |
|------------|---------|
| `linear.x` | Forward (+) / reverse (−) |
| `angular.z` | Turn left (+) / right (−) |

- **Pi:** `rover_twist.set_twist()` → `linear.x` / `angular.z` (`ROVER_TWIST_SWAP=0` default).
- **ESP:** `CMD_VEL_CROSS_FIX=0` in `platformio.ini` (`s3_tdisplay_microros*` / rover OTA).

Sign tweaks only (not swaps): `ROVER_LINEAR_SIGN`, `ROVER_ANGULAR_SIGN` on Pi; `MOTOR_INVERT_L/R` on ESP.

### Legacy `CMD_VEL_CROSS_FIX` (do not enable on rover)

Firmware still contains an optional swap:

```cpp
// CMD_VEL_CROSS_FIX=1 only (legacy):
last_lin = msg->angular.z;
last_ang = msg->linear.x;
```

This was early bring-up compensation when forward/steer appeared crossed (debugging micro-ROS + teleop + motor direction at once). **It is disabled on all current S3 rover builds.** The C++ default is still `1` if the flag is missing — `platformio.ini` explicitly sets `0`. Do not build rover firmware without that flag.

Pi mirror: `ROVER_TWIST_SWAP=1` — also off. Enabling swap on *one* side only is how you get “forward drives spin, steer drives straight.”

**Sanity check:** `python3 test_drive_dirs.py` on the Pi.

---

## ESP: sonar front avoidance

Config in `rover_sonar.cpp` / `rover_sonar.h`.

### Idle / cruise (not in escape)

- Pan glances while driving; stores left/right clearance for **bias steer**.
- Forward + range &lt; **0.55 m** (`SONAR_AVOID_M`): slow + steer toward clearer side (glance bins).
- Forward + range &lt; **0.20 m** (`SONAR_STOP_M`): **bias + slow for ~750 ms**, then full escape if still blocked.

**Normal explore** (standalone): wander + bias + escape only when truly stuck. **Capture mode** (legacy Pi scripts) deliberately hunted walls — not used on-robot anymore.

### Escape state machine

| Phase | Behavior |
|-------|----------|
| **Scan** | Pan 0°→180° (7 steps); record best clearance |
| **Reverse** | ~550 ms backward + slight turn toward escape side |
| **Turn** | Arc toward best angle; **re-reverse if still &lt; STOP** |
| **Drive out** | Forward ~900 ms; **range check every 60 ms** — reverse if blocked |
| **Cooldown** | ~1.2 s; no new escapes |

While in reverse / turn / drive-out: `sonar_drive_override = true` → Pi `cmd_vel`, heading hold, and front IR cut are bypassed for motion.

**Turn sign:** `escape_turn_sign` matches ROS — **+1 = left**, **−1 = right** (`apply_drive`: `l = v − turn`, `r = v + turn`). Pan &gt; center (robot left) → turn left (+1).

**Best bin:** max range across scan; when `scan_min_m` &lt; `SONAR_STOP_M`, ignore readings above `max(0.70 m, scan_min + 0.45 m)` (blocks down-hallway glances at 0°).

**Abort:** `abort_escape()` on stall (or any `force_stop` with a reason string). Clears phase, 1.5 s cooldown, pan to center.

Publishes: `/rover/sonar/range`, `/rover/sonar/pan_deg`, `/rover/sonar/avoid_active`, escape event topics.

---

## ESP: stall detection

`motion_detect.cpp` — “motors commanded but body not moving.”

| Signal | Threshold (approx.) |
|--------|---------------------|
| IMU stall | Motors &gt; 6% ~280 ms, avg motion &lt; 0.08 m/s² equivalent |
| Wheel stall | Motors &gt; 12% ~360 ms, &lt; 1 tick delta on MCP IR wheel sensors |
| Cooldown between stall events | 1.2 s |

**On stall:**

1. Publish `/rover/stall` = true  
2. `force_stop("stall")` — motors off, escape aborted  
3. `stall_block_until` = now + **4000 ms** — `on_cmd` ignored, `apply_drive` forces stop  

Does **not** require Pi acknowledgment.

---

## ESP: bump detection

Sudden **horizontal** jerk while driving → Pi chime/ring only. **No motor cut, no escape** for bump on standalone firmware.

---

## ESP: front / rear IR (MCP23008)

Custom front bumper (L/R, very close, angle vs head-on) and rear aux IR.

Firmware `tick()` continuously pulses emitters **off then on** and uses the **delta** (on && !off) — sunlight rejection and lower average LED power.

- Front hit while not reversing → zero forward (`apply_drive`).
- Rear hit while reversing → zero reverse.
- Topics: `/rover/ir/front` (Bool, any front), `/rover/ir/hits` (UInt8 bits FL/FR/rear).

Brain uses hits alongside sonar (sonar does not disable the bumper). Angled L-only or R-only hit peels away from that side. Rear hit aborts a reverse.

Front IR cut is **skipped** while `sonar_drive_override` is true (ESP-owned sonar manoeuvre). Rear cut still applies.

---

## ESP: other stops

| Condition | Action |
|-----------|--------|
| **Agent lost** | `force_stop("agent lost")`, tear down micro-ROS, wait for agent |
| **Cmd timeout** | 400 ms without `cmd_vel` → stop (unless sonar overriding) |
| **OTA in progress** | Loop blocks until flash complete |

---

## Pi: `autonomous_explore.py`

Runs when you launch explore / capture sessions. State machine:

**WANDER → CRUISE → BURST → AVOID → ESCAPE**

| Input | Pi reaction |
|-------|-------------|
| `/rover/bump` | Queue escape (reverse → radar scan → arc) |
| `/rover/stall` | Same |
| Front IR (if enabled) | AVOID mode — **off when `ROVER_SONAR=1`** |
| Cruise radar | Background sweep biases wander |

Escape flood protection: max 4 escapes/min → 8 s pause.

**Important:** Pi escape and ESP sonar escape are **separate**. Both publish `/cmd_vel`. When ESP sonar is overriding, **ESP motion wins** regardless of what Pi sends.

`ROVER_WHEEL_STALL=0` (default in `rover_common.sh`) — Pi-side wheel stall is **off**; ESP handles stall.

---

## Pi: capture session (`record_sonar_escape_session.sh`)

1. Preflight (`check_drive_ready.sh`)
2. Start `rover_sonar_escape_recorder.py`
3. Start `autonomous_explore.py` + `rover_sonar_ring.py`
4. Run N seconds, analyze with `analyze_sonar_escape.py`

Starts local `rover_esp_bridge` if not already up (rear IR scans).

---

## Two escape systems (common confusion)

```
Pi autonomous_explore          ESP sonar (local, fast)
        |                              |
        +-------- /cmd_vel -------------+
                      |
              ESP merge + priority
                      |
                   motors
```

- **Sonar escape:** sub-20 cm, pan scan, no Pi round-trip.  
- **Pi escape:** bump/stall events, radar/rear IR, longer sequences.  

They can both be active in a capture session. Sonar override outranks Pi `cmd_vel` for wheel commands.

---

## Power / stall → Pi reboot

Motor driver **VREF** limits H-bridge current only, not a separate cap on **5Vo**. Under stall, **VM** (2S pack) sags → **5Vo** (ESP + Pi) can brown out.

Mitigations:

1. **Software:** stall cut + escape abort (this doc).  
2. **Hardware:** separate Pi power bank; trim VREF to reduce stall current (helps sag, not isolation).

---

## Quick debug checklist

| Symptom | Check |
|---------|--------|
| Forward = spin | `CMD_VEL_CROSS_FIX` and `ROVER_TWIST_SWAP` — both must be **0** |
| Drives into wall | Serial: `SONAR escape` phases? `EVENT stall`? Escape aborted? |
| Pi reboot on push | Power rail — not a logic bug |
| No escape events in capture | Recorder running? `stream.jsonl` lines? Bridge up? |
| Wrong turn direction | Use capture + `analyze_sonar_escape.py` — don't guess invert flags |

---

## Change log

| Date | Change |
|------|--------|
| 2026-08-15 | Sonar escape: fix inverted turn sign; filter spurious long bins at wall. |
| 2026-08-15 | Initial doc. Stall+abort_escape fix. Sonar range checks in turn/drive-out. Stall block 4 s. |
