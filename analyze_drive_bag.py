#!/usr/bin/env python3
"""Quick drive analysis from a rosbag2 mcap — cmd_vel jerk + L8 + hips."""
from __future__ import annotations

import argparse
import math
import sys
from collections import deque

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument('bag')
    p.add_argument('--t0', type=float, default=0.0, help='start offset sec from bag start')
    p.add_argument('--t1', type=float, default=0.0, help='end offset sec (0=all)')
    args = p.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id='mcap'),
        ConverterOptions('', ''),
    )

    topics = {
        '/cmd_vel': get_message('geometry_msgs/msg/Twist'),
        '/rover/session': get_message('std_msgs/msg/Bool'),
        '/rover/tof/l8_cols': get_message('std_msgs/msg/Float32MultiArray'),
        '/rover/tof/left': get_message('sensor_msgs/msg/Range'),
        '/rover/tof/right': get_message('sensor_msgs/msg/Range'),
        '/rover/tof/front': get_message('sensor_msgs/msg/Range'),
        '/rover/stall': get_message('std_msgs/msg/Bool'),
        '/rover/bump': get_message('std_msgs/msg/Bool'),
    }

    t_start: float | None = None
    t_end = float('inf') if args.t1 <= 0 else None
    session = args.t0 <= 0.0  # time window mode: analyze all cmd in range
    cmd: deque[tuple[float, float, float]] = deque(maxlen=5000)
    l8_min: list[float] = []
    hip_l: list[float] = []
    hip_r: list[float] = []
    stalls = bumps = 0
    zero_cmd = cmd_n = 0
    dlin_max = dang_max = 0.0
    prev_lin = prev_ang = None
    prev_t = None

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in topics:
            continue
        t = t_ns * 1e-9
        if t_start is None:
            t_start = t
        rel = t - t_start
        if rel < args.t0:
            continue
        if t_end is not None and rel > t_end:
            break

        msg = deserialize_message(raw, topics[topic])
        if topic == '/rover/session':
            if args.t0 <= 0.0:
                session = bool(msg.data)
            continue
        if not session:
            continue
        if topic == '/cmd_vel':
            lin, ang = float(msg.linear.x), float(msg.angular.z)
            cmd_n += 1
            if abs(lin) < 0.01 and abs(ang) < 0.01:
                zero_cmd += 1
            if prev_lin is not None and prev_t is not None and t > prev_t:
                dt = t - prev_t
                if dt < 0.15:
                    dlin_max = max(dlin_max, abs(lin - prev_lin) / dt)
                    dang_max = max(dang_max, abs(ang - prev_ang) / dt)
            prev_lin, prev_ang, prev_t = lin, ang, t
            cmd.append((t, lin, ang))
        elif topic == '/rover/tof/l8_cols':
            vals = [float(v) for v in msg.data if 0.05 < float(v) < 3.0]
            if vals:
                l8_min.append(min(vals))
        elif topic == '/rover/tof/left':
            hip_l.append(float(msg.range))
        elif topic == '/rover/tof/right':
            hip_r.append(float(msg.range))
        elif topic == '/rover/stall' and msg.data:
            stalls += 1
        elif topic == '/rover/bump' and msg.data:
            bumps += 1

    print(f'bag={args.bag}  window={args.t0:.0f}s..{args.t1 or "end"}s')
    print(f'session cmd samples={cmd_n}  zero_cmd={zero_cmd} ({100*zero_cmd/max(1,cmd_n):.0f}%)')
    print(f'cmd jerk max dlin/dt={dlin_max:.3f}  dang/dt={dang_max:.3f}')
    if l8_min:
        close = sum(1 for v in l8_min if v < 0.35)
        print(f'L8 min: mean={sum(l8_min)/len(l8_min):.2f}m  p10={sorted(l8_min)[len(l8_min)//10]:.2f}  close<35cm={close}/{len(l8_min)}')
    if hip_l and hip_r:
        print(f'hip L mean={sum(hip_l)/len(hip_l):.2f}  R mean={sum(hip_r)/len(hip_r):.2f}')
        both = sum(1 for l, r in zip(hip_l, hip_r) if l < 1.4 and r < 1.4)
        print(f'corridor samples (both<1.4m): {both}/{min(len(hip_l), len(hip_r))}')
    print(f'ESP stall events={stalls}  bump={bumps}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
