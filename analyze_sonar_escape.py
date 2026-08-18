#!/usr/bin/env python3
"""Summarise sonar escape recordings from rover_sonar_escape_recorder.py."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_events(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.is_file():
        return rows
    for line in path.read_text().splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description='Analyse sonar escape JSONL recordings')
    ap.add_argument('dir', type=Path, help='Recording directory')
    args = ap.parse_args()

    events_path = args.dir / 'escape_events.jsonl'
    rows = load_events(events_path)
    if not rows:
        print(f'No events in {events_path}')
        return 1

    captures = [r for r in rows if 'front_bins' in r]
    followups = [r for r in rows if r.get('followup_s') is not None]

    print(f'=== Sonar escape analysis: {args.dir} ===')
    print(f'Escape events: {len(captures)}  follow-ups: {len(followups)}')
    print()

    wrong = 0
    ok = 0
    unknown = 0
    for fu in followups:
        v = fu.get('verdict', 'unknown')
        if v == 'WRONG':
            wrong += 1
        elif v == 'ok':
            ok += 1
        else:
            unknown += 1

    if followups:
        print(f'IMU turn check: ok={ok} WRONG={wrong} unknown={unknown}')
        print('(Uses IMU yaw delta — not /cmd_vel, which Pi still publishes during ESP escape.)')
        print()

    for cap in captures:
        eid = cap.get('event_id', '?')
        esp_turn = cap.get('esp_turn_sign', 0.0)
        esp_dir = 'left' if esp_turn > 0 else 'right' if esp_turn < 0 else 'straight'
        best = cap.get('front_best') or {}
        print(f'--- event #{eid} ---')
        print(
            f'ESP decision: turn {esp_dir} (sign={esp_turn:+.2f}) '
            f'best={cap.get("esp_best_deg", "?")}° {cap.get("esp_best_m", "?")}m'
        )
        if best:
            print(
                f'Pi front best: pan={best.get("pan_deg")}° range={best.get("range_m")}m'
            )
        rear = cap.get('rear') or {}
        if rear.get('hits') is not None:
            hits = rear['hits']
            blocked = sum(1 for h in hits if h)
            print(
                f'Rear IR: {blocked}/{len(hits)} blocked, '
                f'gap bearing={rear.get("gap_bearing_deg", "?")}°'
            )
        elif rear.get('error'):
            print(f'Rear IR: error — {rear["error"]}')
        else:
            print('Rear IR: (not captured)')

        bins = cap.get('front_bins') or []
        if bins:
            summary = ', '.join(
                f'{b["pan_deg"]:.0f}°={b["range_m"]:.2f}m' for b in bins[-12:]
            )
            print(f'Front bins ({len(bins)}): {summary}')
        fu = next((f for f in followups if f.get('event_id') == eid), None)
        if fu:
            print(
                f'Follow-up: imu Δyaw={fu.get("imu_yaw_delta_deg")}° → '
                f'{fu.get("turn_observed")} (want {fu.get("front_best_want")}) '
                f'esp={fu.get("esp_turn")} → {fu.get("verdict")}'
            )
        print()

    if wrong:
        print('>>> At least one escape turned the wrong way per IMU — check scan bins vs esp_turn.')
        print('>>> Do NOT flip turn sign blindly; fix mapping or scan data first.')
        return 2
    if ok:
        print('>>> Turn direction matches front clearance on recorded events.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
