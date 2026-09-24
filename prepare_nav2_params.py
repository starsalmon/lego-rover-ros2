#!/usr/bin/env python3
"""Copy the Jazzy Nav2 params and slow them down for this rover."""
from __future__ import annotations

from pathlib import Path

SRC = Path('/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml')
DST = Path('/tmp/nav2_params.yaml')


def main() -> None:
    text = SRC.read_text()
    repls = {
        'cmd_vel_out_topic: "cmd_vel"': 'cmd_vel_out_topic: "cmd_vel_nav"',
        'base_frame_id: "base_footprint"': 'base_frame_id: "base_link"',
        'robot_radius: 0.22': 'robot_radius: 0.16',
        'max_velocity: [0.5, 0.0, 2.0]': 'max_velocity: [0.15, 0.0, 0.35]',
        'min_velocity: [-0.5, 0.0, -2.0]': 'min_velocity: [-0.10, 0.0, -0.35]',
        'max_accel: [2.5, 0.0, 3.2]': 'max_accel: [0.18, 0.0, 0.28]',
        'max_decel: [-2.5, 0.0, -3.2]': 'max_decel: [-0.15, 0.0, -0.22]',
        'max_rotational_vel: 1.0': 'max_rotational_vel: 0.35',
        'min_rotational_vel: 0.4': 'min_rotational_vel: 0.12',
        'rotational_acc_lim: 3.2': 'rotational_acc_lim: 0.28',
        'max_accel: [0.25, 0.0, 0.40]': 'max_accel: [0.18, 0.0, 0.28]',
        'max_decel: [-0.30, 0.0, -0.50]': 'max_decel: [-0.15, 0.0, -0.22]',
        # SLAM map is live — follow the robot instead of a fixed /map box.
        '      track_unknown_space: true\n      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]': (
            '      track_unknown_space: true\n'
            '      rolling_window: true\n'
            '      width: 15\n'
            '      height: 15\n'
            '      plugins: ["obstacle_layer", "inflation_layer"]'
        ),
    }
    for old, new in repls.items():
        if old not in text:
            print(f'missing: {old}')
        text = text.replace(old, new)
    DST.write_text(text)
    print(f'wrote {DST}')


if __name__ == '__main__':
    main()
