#!/usr/bin/env python3
"""Mac PS4 teleop → Pi /cmd_vel via UDP (reliable) or SSH stdin fallback."""
from __future__ import annotations

import errno
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

from joystick_device import open_joystick

# This Mac BT PS4 (pygame): stick X = axis 0, L2 = axis 4, R2 = axis 5 (-1 released).
AXIS_STEER = 0
AXIS_L2 = 4
AXIS_R2 = 5

DEFAULTS = {
    'scale_linear': 1.0,
    'scale_angular_drive': 0.62,
    'scale_angular_spin': 0.42,
    'invert_steer': False,
    'deadzone': 0.10,
    'steer_expo': 1.75,
    'steer_expo_spin': 1.9,
    'deadzone_spin': 0.05,
    'trigger_on': 0.08,
    'linear_slew': 5.0,
    'linear_brake_slew': 2.5,
    'publish_hz': 40.0,
}

UDP_PORT = int(os.environ.get('ROVER_CMD_UDP_PORT', '9999'))


def _load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError:
        return {}
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    block = data.get('/**') or data.get('teleop_mac') or {}
    return block.get('ros__parameters') or block.get('parameters') or {}


def _deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def _trigger_pull(raw: float) -> float:
    """Axis -1 released → 0, +1 fully pressed → 1."""
    if raw <= -0.92:
        return 0.0
    return max(0.0, min(1.0, (raw + 1.0) * 0.5))


def _steer_response(raw: float, cfg: dict, linear: float) -> float:
    """Smooth expo: gentle near centre, progressive to full at max stick."""
    spinning = abs(linear) < 0.06
    if spinning:
        max_out = float(cfg.get('scale_angular_spin', 0.42))
        expo = float(cfg.get('steer_expo_spin', 1.9))
        dz = float(cfg.get('deadzone_spin', 0.05))
    else:
        max_out = float(cfg.get('scale_angular_drive', 0.62))
        expo = float(cfg.get('steer_expo', 1.75))
        dz = float(cfg['deadzone'])
    v = _deadzone(raw, dz)
    if v == 0.0:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    out = (abs(v) ** expo) * max_out
    ang = sign * out
    if cfg.get('invert_steer'):
        ang = -ang
    return ang


class _DriveState:
    __slots__ = ('lin_ramped',)

    def __init__(self) -> None:
        self.lin_ramped = 0.0


def _slew_linear(target: float, state: _DriveState, dt: float, cfg: dict) -> float:
    cur = state.lin_ramped
    accel = float(cfg.get('linear_slew', 5.0))
    brake = float(cfg.get('linear_brake_slew', 2.5))

    # Brake to zero before reversing — no judder, but not glacial.
    opposite = (cur > 0.03 and target < -0.03) or (cur < -0.03 and target > 0.03)
    eff_target = 0.0 if opposite else target
    rate = brake if opposite or abs(eff_target) < abs(cur) else accel

    delta = eff_target - cur
    if abs(delta) < 1e-5:
        return cur
    step = rate * dt
    if delta > step:
        delta = step
    elif delta < -step:
        delta = -step
    state.lin_ramped = cur + delta
    return state.lin_ramped


def compute_cmd(js, cfg: dict, state: _DriveState, dt: float) -> tuple[float, float]:
    axes = js.axes
    trigger_on = float(cfg.get('trigger_on', 0.08))
    l2 = _trigger_pull(float(axes.get(AXIS_L2, -1.0)))
    r2 = _trigger_pull(float(axes.get(AXIS_R2, -1.0)))
    if l2 < trigger_on:
        l2 = 0.0
    if r2 < trigger_on:
        r2 = 0.0
    stick = float(axes.get(AXIS_STEER, 0.0))

    scale_lin = float(cfg['scale_linear'])
    if l2 > r2:
        lin_target = -l2 * scale_lin
    elif r2 > l2:
        lin_target = r2 * scale_lin
    else:
        lin_target = 0.0

    driving = abs(lin_target) >= trigger_on
    if driving:
        linear = _slew_linear(lin_target, state, dt, cfg)
        angular = _steer_response(stick, cfg, linear)
    else:
        stick_ang = _steer_response(stick, cfg, 0.0)
        if abs(stick_ang) > 0.02:
            # Spin — cut drive immediately so slew bleed doesn't drive forward.
            state.lin_ramped = 0.0
            linear = 0.0
            angular = stick_ang
        else:
            linear = _slew_linear(0.0, state, dt, cfg)
            if abs(linear) < 0.04:
                state.lin_ramped = 0.0
                linear = 0.0
            angular = 0.0

    return linear, angular


def _raw_axes(js) -> str:
    n = max(6, max(js.axes.keys(), default=0) + 1)
    return ' '.join(f'{i}:{js.axes.get(i, 0):+.2f}' for i in range(n))


def _ssh_cmd() -> list[str]:
    pi = os.environ.get('ROVER_PI', 'cain@lego-rover.local')
    key = os.environ.get('ROVER_SSH_KEY', os.path.expanduser('~/.ssh/lego_rover_cursor'))
    pi_dir = os.environ.get('ROVER_PI_DIR', '~/lego-rover-ros2')
    return [
        'ssh', '-i', key,
        '-o', 'StrictHostKeyChecking=accept-new',
        '-o', 'ConnectTimeout=8',
        pi,
        f'source /opt/ros/jazzy/setup.bash && '
        f'source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null; '
        f'cd {pi_dir} && PYTHONUNBUFFERED=1 python3 -u {pi_dir}/cmd_vel_ssh_relay.py',
    ]


def main() -> int:
    cfg_path = Path(
        os.environ.get(
            'TELEOP_CONFIG',
            Path(__file__).resolve().parent / 'teleop_mac.yaml',
        )
    )
    cfg = {**DEFAULTS, **_load_yaml(cfg_path)}
    debug = os.environ.get('TELEOP_DEBUG', '').strip() in ('1', 'yes', 'true')

    pi_ip = os.environ.get('ROVER_PI_IP', '').strip()
    use_udp = os.environ.get('ROVER_TELEOP_UDP', '1').strip() not in ('0', 'no', 'false')
    if use_udp and not pi_ip:
        use_udp = False

    opened = None
    for attempt in range(12):
        opened = open_joystick()
        if opened:
            break
        if attempt == 0:
            print('Waiting for gamepad...', flush=True)
        time.sleep(0.5)
    if not opened:
        print('ERROR: no gamepad — pair PS4 to Mac via Bluetooth', file=sys.stderr)
        return 1
    js, desc = opened

    print(f'Gamepad: {desc}')
    print(f'Map: stick X=axis{AXIS_STEER}  L2=axis{AXIS_L2}  R2=axis{AXIS_R2}')
    print('R2 = forward  L2 = reverse  Left stick X = steer while moving / spin when stopped')
    if use_udp:
        print(f'UDP → {pi_ip}:{UDP_PORT}')
    else:
        print('SSH stdin relay')
    print('Ctrl+C to stop')

    udp_sock: socket.socket | None = None
    ssh_proc: subprocess.Popen | None = None
    shutting_down = False

    if use_udp:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    else:
        ssh_proc = subprocess.Popen(
            _ssh_cmd(),
            stdin=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    def _send(lin: float, ang: float, *, force: bool = False) -> None:
        if shutting_down and not force:
            return
        line = f'{lin:.4f} {ang:.4f}'
        try:
            if udp_sock is not None and pi_ip:
                udp_sock.sendto(line.encode('ascii'), (pi_ip, UDP_PORT))
            elif ssh_proc and ssh_proc.stdin:
                ssh_proc.stdin.write(line + '\n')
                ssh_proc.stdin.flush()
        except OSError as exc:
            if shutting_down or exc.errno == errno.EBADF:
                return
            if debug:
                print(f'  WARN send failed: {exc}', flush=True)

    def _request_stop(*_args):
        nonlocal shutting_down
        shutting_down = True

    def _cleanup() -> None:
        nonlocal udp_sock
        try:
            if udp_sock is not None and pi_ip:
                udp_sock.sendto(b'0.0000 0.0000', (pi_ip, UDP_PORT))
        except OSError:
            pass
        if ssh_proc and ssh_proc.stdin:
            try:
                ssh_proc.stdin.close()
            except OSError:
                pass
            try:
                ssh_proc.terminate()
            except OSError:
                pass
        if udp_sock is not None:
            try:
                udp_sock.close()
            except OSError:
                pass
            udp_sock = None

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    hz = float(cfg['publish_hz'])
    period = 1.0 / hz
    last_hint = 0.0
    drive_state = _DriveState()
    last_t = time.monotonic()
    try:
        while not shutting_down and (ssh_proc is None or ssh_proc.poll() is None):
            t0 = time.monotonic()
            dt = min(0.12, t0 - last_t)
            last_t = t0
            try:
                js.poll()
            except Exception as exc:
                if debug and not shutting_down:
                    print(f'  WARN gamepad poll: {exc}', flush=True)
                time.sleep(0.1)
                continue
            lin, ang = compute_cmd(js, cfg, drive_state, dt)
            _send(lin, ang)

            if debug and not shutting_down:
                if abs(lin) > 0.01 or abs(ang) > 0.01:
                    print(f'  → lin {lin:.2f}  ang {ang:.2f}  | {_raw_axes(js)}', flush=True)
                elif t0 - last_hint > 3.0:
                    print(f'  (idle) {_raw_axes(js)}', flush=True)
                    last_hint = t0

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))
    finally:
        _send(0.0, 0.0, force=True)
        _cleanup()
        if ssh_proc:
            try:
                ssh_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                ssh_proc.kill()
        js.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
