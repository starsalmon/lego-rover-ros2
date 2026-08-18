#!/usr/bin/env python3
"""Analyze standalone rover serial capture (TELEM + SONAR escape lines)."""
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

TELEM_RE = re.compile(
    r'TELEM t=(\d+) lin=([-\d.]+) ang=([-\d.]+) L=([-\d.]+) R=([-\d.]+) '
    r'pan=([-\d.]+) rng=([-\d.]+) gz=([-\d.]+) yaw=([-\d.]+) '
    r'ovr=(\d+) ph=(\d+) esc=([-\d.]+)'
)
SONAR_TURN_RE = re.compile(
    r'SONAR scan best=([-\d.]+) deg ([-\d.]+)m → turn (left|right)'
)
SONAR_REVERSE_RE = re.compile(
    r'SONAR scan best=([-\d.]+) deg ([-\d.]+)m min=([-\d.]+)m → reverse turn (left|right)'
)
SONAR_BINS_RE = re.compile(r'SONAR scan bins:(.*)')
BIN_RE = re.compile(r' ([-\d.]+)=([-\d.]+)')


@dataclass
class Telem:
    t_ms: int
    lin: float
    ang: float
    ml: float
    mr: float
    pan: float
    rng: float
    gz: float
    yaw: float
    ovr: int
    ph: int
    esc: float


def parse_log(path: Path) -> tuple[list[Telem], list[dict]]:
    telem: list[Telem] = []
    escapes: list[dict] = []
    pending_bins: str | None = None

    for raw in path.read_text(encoding='utf-8', errors='replace').splitlines():
        line = raw.strip()
        m = TELEM_RE.search(line)
        if m:
            telem.append(
                Telem(
                    t_ms=int(m.group(1)),
                    lin=float(m.group(2)),
                    ang=float(m.group(3)),
                    ml=float(m.group(4)),
                    mr=float(m.group(5)),
                    pan=float(m.group(6)),
                    rng=float(m.group(7)),
                    gz=float(m.group(8)),
                    yaw=float(m.group(9)),
                    ovr=int(m.group(10)),
                    ph=int(m.group(11)),
                    esc=float(m.group(12)),
                )
            )
            continue
        bm = SONAR_BINS_RE.search(line)
        if bm:
            pending_bins = bm.group(1).strip()
            continue
        tm = SONAR_TURN_RE.search(line) or SONAR_REVERSE_RE.search(line)
        if tm:
            turn = tm.group(3) if tm.re is SONAR_TURN_RE else tm.group(4)
            bins: list[tuple[float, float]] = []
            if pending_bins:
                bins = [(float(a), float(b)) for a, b in BIN_RE.findall(pending_bins)]
                pending_bins = None
            escapes.append(
                {
                    'best_deg': float(tm.group(1)),
                    'best_m': float(tm.group(2)),
                    'turn': turn,
                    'bins': bins,
                }
            )

    return telem, escapes


def yaw_delta(telems: list[Telem], i: int, window_ms: int = 450) -> float | None:
    if i >= len(telems):
        return None
    t0 = telems[i].t_ms
    yaw0 = telems[i].yaw
    for j in range(i + 1, len(telems)):
        if telems[j].t_ms - t0 >= window_ms:
            return telems[j].yaw - yaw0
    return None


def sign(v: float, eps: float = 1e-6) -> int:
    if v > eps:
        return 1
    if v < -eps:
        return -1
    return 0


def analyze_steer(telems: list[Telem]) -> dict:
    """+ang should turn left (+yaw delta) per firmware convention."""
    checked = 0
    wrong = 0
    unknown = 0
    examples: list[str] = []

    for i, row in enumerate(telems):
        if abs(row.ang) < 0.035 or abs(row.lin) < 0.05:
            continue
        if row.ovr:
            continue
        dyaw = yaw_delta(telems, i)
        if dyaw is None:
            unknown += 1
            continue
        want = sign(row.ang)
        got = sign(dyaw, eps=0.8)
        if got == 0:
            unknown += 1
            continue
        checked += 1
        if want != got:
            wrong += 1
            if len(examples) < 5:
                examples.append(
                    f'  ang={row.ang:+.3f} → yaw Δ{dyaw:+.1f}° (wanted {"left" if want > 0 else "right"})'
                )

    return {'checked': checked, 'wrong': wrong, 'unknown': unknown, 'examples': examples}


def analyze_escape(telems: list[Telem], escapes: list[dict]) -> dict:
    results: list[str] = []
    wrong = 0
    ok = 0

    esc_idx = 0
    for i, row in enumerate(telems):
        if row.ovr and row.ph >= 4 and abs(row.esc) > 0.01:  # turn / drive-out phases
            dyaw = yaw_delta(telems, i, window_ms=700)
            if dyaw is None:
                continue
            want = sign(row.esc)
            got = sign(dyaw, eps=1.5)
            if got == 0:
                continue
            if want != got:
                wrong += 1
                meta = escapes[esc_idx] if esc_idx < len(escapes) else {}
                results.append(
                    f'  escape esc={row.esc:+.0f} → yaw Δ{dyaw:+.1f}° '
                    f'(logged turn {meta.get("turn", "?")})'
                )
            else:
                ok += 1
            esc_idx += 1

    return {'ok': ok, 'wrong': wrong, 'examples': results[:6]}


def analyze_bias(telems: list[Telem]) -> dict:
    """When pan off-center and cruising, ang should steer toward clearer side."""
    checked = 0
    suspect = 0
    center = 90.0
    for row in telems:
        if row.ovr or abs(row.lin) < 0.08:
            continue
        pan_err = row.pan - center
        if abs(pan_err) < 12:
            continue
        if abs(row.ang) < 0.01:
            continue
        checked += 1
        # pan high → gap on robot left → should steer +ang
        want = sign(pan_err)
        got = sign(row.ang)
        if want != got and abs(row.ang) > 0.02:
            suspect += 1
    return {'checked': checked, 'suspect': suspect}


def main() -> int:
    ap = argparse.ArgumentParser(description='Analyze standalone explore serial log')
    ap.add_argument('log', type=Path)
    args = ap.parse_args()

    if not args.log.is_file():
        print(f'Missing {args.log}', file=sys.stderr)
        return 1

    telems, escapes = parse_log(args.log)
    print(f'=== Standalone explore analysis: {args.log.name} ===')
    print(f'TELEM samples: {len(telems)}  escape events: {len(escapes)}')
    if not telems:
        print('No TELEM lines — re-flash with telemetry build and re-capture.')
        return 1

    dur_s = (telems[-1].t_ms - telems[0].t_ms) / 1000.0
    moving = sum(1 for t in telems if abs(t.ml) > 0.05 or abs(t.mr) > 0.05)
    ovr = sum(1 for t in telems if t.ovr)
    print(f'Duration: {dur_s:.1f}s  moving samples: {moving}/{len(telems)}  sonar override: {ovr}')
    print()

    steer = analyze_steer(telems)
    print('--- Cruise steer vs IMU (+ang = left) ---')
    print(f'Checked: {steer["checked"]}  WRONG: {steer["wrong"]}  inconclusive: {steer["unknown"]}')
    for ex in steer['examples']:
        print(ex)
    if steer['checked'] and steer['wrong'] / steer['checked'] > 0.4:
        print('>>> Cruise turns mostly inverted — check apply_drive sign or IMU_GYRO_YAW_SIGN')
    print()

    bias = analyze_bias(telems)
    print('--- Sonar pan bias (glance steer) ---')
    print(f'Off-center cruise samples: {bias["checked"]}  ang opposite pan: {bias["suspect"]}')
    if bias['checked'] and bias['suspect'] / bias['checked'] > 0.5:
        print('>>> Bias steer likely inverted — flip cruise_bias_ang or pan left/right mapping')
    print()

    esc = analyze_escape(telems, escapes)
    print('--- Sonar escape turn vs IMU ---')
    print(f'ok: {esc["ok"]}  WRONG: {esc["wrong"]}')
    for ex in esc['examples']:
        print(ex)
    print()

    for i, ev in enumerate(escapes[:8], 1):
        bins = ev.get('bins') or []
        if bins:
            summary = ', '.join(f'{d:.0f}°={m:.2f}m' for d, m in bins)
            print(f'Escape #{i}: turn {ev["turn"]} best={ev["best_deg"]:.0f}° {ev["best_m"]:.2f}m')
            print(f'  bins: {summary}')
    print()

    total_wrong = steer['wrong'] + esc['wrong']
    if total_wrong:
        print('>>> Turn direction mismatches detected — see sections above before flipping flags.')
        return 2
    if steer['checked'] or esc['ok']:
        print('>>> Turn directions consistent with +ang=left convention on this capture.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
