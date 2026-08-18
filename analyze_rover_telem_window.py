#!/usr/bin/env python3
"""Slice always-on rover-telem JSONL by wall-clock time and analyze behaviour."""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

DIR = Path(__file__).resolve().parent
TELEM_DIR = Path.home() / 'rover-telem'

TELEM_V2_RE = re.compile(
    r'TELEM t=(\d+) slin=([-\d.]+) sang=([-\d.]+) wlin=([-\d.]+) wang=([-\d.]+) '
    r'tL=([-\d.]+) tR=([-\d.]+) L=([-\d.]+) R=([-\d.]+) pan=([-\d.]+) rng=([-\d.]+) '
    r'gl=([-\d.]+) gr=([-\d.]+) yaw=([-\d.]+) ovr=(\d+) ph=(\d+) gs=(\d+) blk=(\d+) '
    r'wtl=(\d+) wtr=(\d+) sess=(\d+)'
)
TELEM_V1_RE = re.compile(
    r'TELEM t=(\d+) lin=([-\d.]+) ang=([-\d.]+) L=([-\d.]+) R=([-\d.]+) '
    r'pan=([-\d.]+) rng=([-\d.]+) gz=([-\d.]+) yaw=([-\d.]+) '
    r'ovr=(\d+) ph=(\d+) esc=([-\d.]+)'
)
EVT_RE = re.compile(r'EVT t=(\d+) tag=(\S+) (.*)')


def parse_when(s: str, tz: timezone) -> datetime:
    for fmt in ('%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M', '%H:%M:%S', '%H:%M'):
        try:
            dt = datetime.strptime(s, fmt)
            if fmt.startswith('%H'):
                today = datetime.now(tz).date()
                dt = dt.replace(year=today.year, month=today.month, day=today.day)
            return dt.replace(tzinfo=tz)
        except ValueError:
            continue
    raise SystemExit(f'Cannot parse time: {s!r}')


def iter_jsonl_files(telem_dir: Path, t0: datetime, t1: datetime) -> list[Path]:
    days = set()
    cur = t0.date()
    while cur <= t1.date():
        days.add(telem_dir / f'{cur.isoformat()}.jsonl')
        cur += timedelta(days=1)
    return [p for p in sorted(days) if p.is_file()]


def load_window(telem_dir: Path, t0: datetime, t1: datetime) -> list[dict]:
    rows: list[dict] = []
    for path in iter_jsonl_files(telem_dir, t0, t1):
        with path.open(encoding='utf-8') as f:
            for raw in f:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    rec = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                ts = rec.get('recv_ts')
                if ts is None:
                    continue
                recv = datetime.fromtimestamp(float(ts), tz=t0.tzinfo)
                if t0 <= recv <= t1:
                    rows.append(rec)
    return rows


def summarize(rows: list[dict]) -> None:
    if not rows:
        print('No lines in window.')
        return

    telem = 0
    evts: list[str] = []
    stops = 0
    stalls = 0
    blk_samples = 0
    pan_vals: list[float] = []
    pan_moves = 0
    prev_pan: float | None = None
    motor_cuts = 0
    prev_moving = False

    for rec in rows:
        line = rec.get('line', '')
        m2 = TELEM_V2_RE.search(line)
        m1 = TELEM_V1_RE.search(line)
        em = EVT_RE.search(line)
        if em:
            tag = em.group(2)
            detail = em.group(3)
            evts.append(f'  {rec["recv_iso"]} {tag} {detail}')
            if tag == 'STOP':
                stops += 1
            if tag == 'STALL':
                stalls += 1
            continue
        if not m2 and not m1:
            if line.startswith('SONAR') or line.startswith('DRIVE'):
                evts.append(f'  {rec["recv_iso"]} {line[:120]}')
            continue
        telem += 1
        if m2:
            pan = float(m2.group(10))
            ml = float(m2.group(8))
            mr = float(m2.group(9))
            blk = int(m2.group(17))
            if blk:
                blk_samples += 1
        else:
            pan = float(m1.group(6))
            ml = float(m1.group(4))
            mr = float(m1.group(5))
        pan_vals.append(pan)
        if prev_pan is not None and abs(pan - prev_pan) > 8:
            pan_moves += 1
        prev_pan = pan
        moving = abs(ml) > 0.05 or abs(mr) > 0.05
        if prev_moving and not moving:
            motor_cuts += 1
        prev_moving = moving

    t_first = rows[0]['recv_iso']
    t_last = rows[-1]['recv_iso']
    print(f'Window: {t_first} → {t_last}')
    print(f'Records: {len(rows)}  TELEM: {telem}  events: {len(evts)}')
    print()
    print('--- Stop / stall events ---')
    print(f'STOP events: {stops}  STALL events: {stalls}  drive_blocked samples: {blk_samples}')
    print(f'Motor cut transitions (was moving → stopped): {motor_cuts}')
    for e in evts[:25]:
        print(e)
    if len(evts) > 25:
        print(f'  … +{len(evts) - 25} more')
    print()
    if pan_vals:
        print('--- Pan sweep ---')
        print(
            f'pan min={min(pan_vals):.0f} max={max(pan_vals):.0f} '
            f'moves>8deg={pan_moves} samples={len(pan_vals)}'
        )
        if pan_moves < max(3, telem // 40):
            print('>>> Pan barely moved — glance/sweep may be stuck or too slow')
    print()


def main() -> int:
    ap = argparse.ArgumentParser(description='Analyze rover-telem JSONL for a time window')
    ap.add_argument('--dir', type=Path, default=TELEM_DIR, help='rover-telem directory')
    ap.add_argument('--at', help='Centre time (YYYY-MM-DD HH:MM or HH:MM today)')
    ap.add_argument('--seconds', type=int, default=90, help='Window length when using --at')
    ap.add_argument('--from', dest='from_', metavar='FROM', help='Window start (ISO-ish)')
    ap.add_argument('--to', metavar='TO', help='Window end')
    ap.add_argument('--full', action='store_true', help='Also run analyze_standalone_explore.py')
    args = ap.parse_args()

    tz = datetime.now().astimezone().tzinfo or timezone.utc
    if args.from_ and args.to:
        t0 = parse_when(args.from_, tz)
        t1 = parse_when(args.to, tz)
    elif args.at:
        centre = parse_when(args.at, tz)
        half = timedelta(seconds=args.seconds / 2)
        t0 = centre - half
        t1 = centre + half
    else:
        ap.error('Use --at or --from/--to')

    if not args.dir.is_dir():
        print(f'Missing telem dir {args.dir} — is rover-telem.service running?', file=sys.stderr)
        return 1

    rows = load_window(args.dir, t0, t1)
    print(f'=== Rover telem analysis ===')
    print(f'Dir: {args.dir}')
    summarize(rows)

    if args.full and rows:
        with tempfile.NamedTemporaryFile('w', suffix='.log', delete=False) as tmp:
            for rec in rows:
                tmp.write(rec.get('line', '') + '\n')
            tmp_path = Path(tmp.name)
        print('--- analyze_standalone_explore ---')
        subprocess.run(
            [sys.executable, str(DIR / 'analyze_standalone_explore.py'), str(tmp_path)],
            check=False,
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
