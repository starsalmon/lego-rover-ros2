#!/usr/bin/env python3
"""Summarise tune capture — rosbag2 or lightweight jsonl from record_tune_light.py."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

STRAIGHT_LIN = 0.25
STRAIGHT_ANG = 0.05


def twist_swap_enabled() -> bool:
    return os.environ.get('ROVER_TWIST_SWAP', '0').strip().lower() not in ('0', 'no', 'false')


def wire_to_logical(lin_wire: float, ang_wire: float) -> tuple[float, float]:
    if twist_swap_enabled():
        return ang_wire, lin_wire
    return lin_wire, ang_wire


def stats(vals: list[float]) -> str:
    if not vals:
        return 'n/a'
    mean = sum(vals) / len(vals)
    var = sum((v - mean) ** 2 for v in vals) / len(vals)
    return f'mean={mean:+.4f} std={math.sqrt(var):.4f} min={min(vals):+.4f} max={max(vals):+.4f}'


def _load_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.is_file():
        return rows
    for line in path.read_text().splitlines():
        line = line.strip()
        if line:
            rows.append(json.loads(line))
    return rows


def _analyze_series(
    gz_samples: list[float],
    lin_samples: list[float],
    ang_samples: list[float],
    paired: list[tuple[float, float, float, float]],
) -> int:
    if not gz_samples:
        print('No IMU samples')
        return 1

    print(f'Samples: imu={len(gz_samples)}  cmd_vel={len(lin_samples)}')
    if not twist_swap_enabled():
        print('cmd_vel: linear.x = forward, angular.z = steer')
    else:
        print('ROVER_TWIST_SWAP=1 — logical forward is on angular.z in /cmd_vel topic')
    print(f'angular_velocity.z: {stats(gz_samples)}')
    if lin_samples:
        print(f'cmd_vel logical forward:   {stats(lin_samples)}')
        print(f'cmd_vel logical steer:     {stats(ang_samples)}')

    last_lin = 0.0
    last_ang = 0.0
    straight_gz: list[float] = []
    yaw_deg = 0.0
    prev_t: float | None = None
    straight_secs = 0.0

    for lin, ang, gz, ts in paired:
        if not math.isnan(lin):
            last_lin = lin
            last_ang = ang
        if not math.isnan(gz):
            if prev_t is not None and ts > prev_t:
                dt = ts - prev_t
                straight = abs(last_lin) >= STRAIGHT_LIN and abs(last_ang) <= STRAIGHT_ANG
                if straight:
                    straight_gz.append(gz)
                    yaw_deg += math.degrees(gz * dt)
                    straight_secs += dt
        prev_t = ts

    print()
    print('--- straight-line capture (cmd forward, stick centred) ---')
    if straight_secs < 3.0:
        print(
            f'WARN: only {straight_secs:.1f}s of straight driving '
            f'(need |lin|>={STRAIGHT_LIN}, |ang|<={STRAIGHT_ANG}). '
            'Hold R2, keep stick centred for the full window.'
        )
    else:
        print(f'Straight segment: {straight_secs:.1f}s  samples={len(straight_gz)}')
        print(f'  gyro z during straight: {stats(straight_gz)}')
        print(f'  integrated heading change: {yaw_deg:+.1f} deg')
        drift_per_s = yaw_deg / straight_secs if straight_secs > 0 else 0.0
        print(f'  drift rate: {drift_per_s:+.2f} deg/s')

        gz_mean = sum(straight_gz) / len(straight_gz) if straight_gz else 0.0
        spinning = abs(gz_mean) > 1.0
        if spinning and lin_samples:
            mean_lin = sum(lin_samples) / len(lin_samples)
            mean_ang = sum(ang_samples) / len(ang_samples)
            if abs(mean_lin) >= STRAIGHT_LIN and abs(mean_ang) <= STRAIGHT_ANG:
                if twist_swap_enabled():
                    print(
                        '  → Still spinning with swap on — check ESP flash / agent; '
                        'try IMU_GYRO_YAW_SIGN=-1 only after drive direction is correct'
                    )
                else:
                    print(
                        '  → Rover spinning on forward cmd — check motor wiring / '
                        'ROVER_LINEAR_SIGN (or legacy ROVER_TWIST_SWAP)'
                    )
            else:
                print(
                    '  → Rover spinning — cmd_vel may not match intent (motor wiring / ROVER_LINEAR_SIGN)'
                )
        elif abs(yaw_deg) > 8.0:
            if abs(gz_mean) < 0.15:
                print(
                    '  → Slow drift — increase HEADING_ADJ_OUT_NORM or decrease '
                    'HEADING_ADJ_MAP_DEG in platformio.ini, then re-flash ESP'
                )
            elif (yaw_deg > 0 and gz_mean > 0.15) or (yaw_deg < 0 and gz_mean < -0.15):
                print(
                    '  → Gyro and heading same sign — try IMU_GYRO_YAW_SIGN=-1 '
                    '(or +1 if already -1) in platformio.ini'
                )
            else:
                print('  → Drift detected — tune HEADING_ADJ_MAP_DEG / HEADING_ADJ_OUT_NORM, re-flash ESP')
        else:
            print('  → Heading hold looks reasonable for this run.')

    print()
    print('Tip: re-run after changes; goal is |integrated heading| < ~5 deg over 30s straight.')
    return 0


def _from_jsonl(capture_dir: Path) -> int:
    imu_rows = _load_jsonl(capture_dir / 'imu.jsonl')
    cmd_rows = _load_jsonl(capture_dir / 'cmd_vel.jsonl')
    if not imu_rows:
        print(f'No imu.jsonl in {capture_dir}')
        return 1

    gz_samples = [float(r['gz']) for r in imu_rows]
    lin_samples = [float(r.get('lin_logical', r['lin'])) for r in cmd_rows]
    ang_samples = [float(r.get('ang_logical', r['ang'])) for r in cmd_rows]

    events: list[tuple[str, float, float, float, float]] = []
    for r in cmd_rows:
        lin = float(r.get('lin_logical', r['lin']))
        ang = float(r.get('ang_logical', r['ang']))
        events.append(('cmd', float(r['t']), lin, ang, float('nan')))
    for r in imu_rows:
        events.append(('imu', float(r['t']), float('nan'), float('nan'), float(r['gz'])))
    events.sort(key=lambda x: x[1])

    paired: list[tuple[float, float, float, float]] = []
    last_lin = 0.0
    last_ang = 0.0
    for kind, ts, lin, ang, gz in events:
        if kind == 'cmd':
            last_lin, last_ang = lin, ang
            paired.append((lin, ang, float('nan'), ts))
        else:
            paired.append((last_lin, last_ang, gz, ts))

    return _analyze_series(gz_samples, lin_samples, ang_samples, paired)


def _from_rosbag(bag_dir: Path) -> int:
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print('Run on Pi with: source /opt/ros/jazzy/setup.bash', file=sys.stderr)
        return 1

    meta = bag_dir / 'metadata.yaml'
    if not meta.is_file():
        print('Bag incomplete (no metadata.yaml)', file=sys.stderr)
        return 1

    storage = 'mcap' if list(bag_dir.glob('*.mcap')) else 'sqlite3'
    text = meta.read_text()
    for line in text.splitlines():
        if 'storage_identifier:' in line:
            storage = line.split(':', 1)[1].strip()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage),
        ConverterOptions('', ''),
    )

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    Imu = get_message(topics['/imu/data'])
    Twist = get_message(topics['/cmd_vel']) if '/cmd_vel' in topics else None

    gz_samples: list[float] = []
    lin_samples: list[float] = []
    ang_samples: list[float] = []
    paired: list[tuple[float, float, float, float]] = []

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        ts = ts_ns * 1e-9
        if topic == '/imu/data':
            msg = deserialize_message(data, Imu)
            gz = float(msg.angular_velocity.z)
            gz_samples.append(gz)
            paired.append((float('nan'), float('nan'), gz, ts))
        elif topic == '/cmd_vel' and Twist is not None:
            msg = deserialize_message(data, Twist)
            lin_w = float(msg.linear.x)
            ang_w = float(msg.angular.z)
            lin, ang = wire_to_logical(lin_w, ang_w)
            lin_samples.append(lin)
            ang_samples.append(ang)
            paired.append((lin, ang, float('nan'), ts))

    return _analyze_series(gz_samples, lin_samples, ang_samples, paired)


def main() -> int:
    ap = argparse.ArgumentParser(description='Analyse rover tune capture')
    ap.add_argument('capture_dir', type=Path, help='rosbag2 dir or jsonl capture dir')
    args = ap.parse_args()

    if not args.capture_dir.is_dir():
        print(f'No capture at {args.capture_dir}', file=sys.stderr)
        return 1

    if (args.capture_dir / 'imu.jsonl').is_file():
        return _from_jsonl(args.capture_dir)
    return _from_rosbag(args.capture_dir)


if __name__ == '__main__':
    sys.exit(main())
