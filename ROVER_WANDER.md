# Rover wander / explore modes (current)

Rover explore runs on the NUC brain in `rover-brain` (`ROVER_MODE=explore`). Full behaviour, thresholds and env are in [ROVER_BEHAVIOR.md](ROVER_BEHAVIOR.md).

## Mode roll

| Mode | Chance | Duration | Motion |
|------|--------|----------|--------|
| Cruise | ≈70% | 4–8 s | Straight-ish at the cruise speed (cap 25% usable) |
| Wander | ≈30% | 3–5 s | Gentle random curve |
| Burst | 1% | 0.8–1.2 s | Ramp to the burst cap (25% usable, same as cruise) |

After an escape: straight cruise for 5–9 s.

The picture (L8 + side ToF) shapes every mode: it slows and curves toward the open side near obstacles and only reverses when boxed in (see ROVER_BEHAVIOR.md → Steer-around).

## Escapes

Bump or stall → reverse 1 s (10% usable) → IMU-limited 20–35° arc toward the gap → cruise. At most 3 per minute.

## Wall mode

Selected from the ESP mode menu (`/rover/drive_mode` = Wall). Uses `HallwayFollow` with the side ToF.
