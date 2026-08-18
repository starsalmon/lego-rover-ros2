#!/usr/bin/env python3
"""ESP buttons + session state over ROS (/rover/button, /rover/button_event, /rover/drive_mode)."""
from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path

_BRIDGE_LOCK = threading.Lock()
_PENDING = False
_SHUTDOWN_PENDING = False
_LAST_PUBLISHED_SESSION: bool | None = None
_THREAD: threading.Thread | None = None

# Keep in sync with lego-rover-esp32/src/rover_button_events.h
BTN_GO_SHORT = 1
BTN_GO_LONG = 2
BTN_ESTOP_LONG = 3
BTN_GO_TRIPLE = 4
BTN_POWER_SHUTDOWN = 5
BTN_POWER_REBOOT = 6
BTN_MENU_CYCLE = 7

DRIVE_EXPLORE = 0
DRIVE_WALL = 1

SESSION_FILE = Path(os.environ.get('ROVER_SESSION_FILE', '/tmp/rover_session_active'))
BUTTON_FILE = Path(os.environ.get('ROVER_BUTTON_FILE', '/tmp/rover_button_press'))
SHUTDOWN_FILE = Path(os.environ.get('ROVER_SHUTDOWN_FILE', '/tmp/rover_shutdown_request'))
MODE_FILE = Path(os.environ.get('ROVER_DRIVE_MODE_FILE', '/tmp/rover_drive_mode'))


def _button_source() -> str:
    return os.environ.get('ROVER_BUTTON_SOURCE', 'esp').strip().lower()


def use_esp_button() -> bool:
    return _button_source() in ('esp', 'esp32', '1', 'true', 'yes')


def _beep(name: str) -> None:
    try:
        from rover_speaker import play

        play(name)
    except Exception:
        pass


def _read_session_active() -> bool:
    try:
        return SESSION_FILE.read_text().strip() == '1'
    except OSError:
        return False


def set_session_active(active: bool) -> None:
    """Shared across processes via SESSION_FILE."""
    try:
        SESSION_FILE.write_text('1' if active else '0')
    except OSError:
        pass


def _write_drive_mode(mode: int) -> None:
    try:
        MODE_FILE.write_text(str(int(mode)))
    except OSError:
        pass


def read_drive_mode() -> int:
    try:
        mode = int(MODE_FILE.read_text().strip())
    except (OSError, ValueError):
        return DRIVE_EXPLORE
    return DRIVE_WALL if mode == DRIVE_WALL else DRIVE_EXPLORE


def drive_mode_name(mode: int | None = None) -> str:
    mode = DRIVE_EXPLORE if mode is None else mode
    return 'wall' if mode == DRIVE_WALL else 'explore'


def _handle_estop_long() -> None:
    global _SHUTDOWN_PENDING
    with _BRIDGE_LOCK:
        _SHUTDOWN_PENDING = True
    try:
        SHUTDOWN_FILE.touch()
    except OSError:
        pass

    set_session_active(False)
    print('Long E-stop — stopping rover session.', flush=True)
    _beep('menu_done')


def _stop_motors_quick() -> None:
    subprocess.run(
        [
            'bash',
            '-lc',
            'source ~/lego-rover-ros2/rover_common.sh && rover_source_ros && rover_stop_motors',
        ],
        check=False,
    )


def _handle_power_cmd(cmd: str) -> None:
    """Shutdown or reboot the Pi (ESP keeps running)."""
    set_session_active(False)
    _stop_motors_quick()
    print(f'Power menu — {cmd} requested.', flush=True)
    _beep('menu_done')
    time.sleep(0.8)
    if cmd == 'shutdown':
        subprocess.run(['sudo', '/usr/sbin/shutdown', '-h', 'now'], check=False)
    elif cmd == 'reboot':
        subprocess.run(['sudo', '/usr/sbin/reboot'], check=False)


def _notify_button_press() -> None:
    global _PENDING
    with _BRIDGE_LOCK:
        _PENDING = True
    try:
        BUTTON_FILE.touch()
    except OSError:
        pass
    _beep('button')


def _run_bridge() -> None:
    global _PENDING, _LAST_PUBLISHED_SESSION
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, UInt8, UInt32

    from rover_esp_ir import IR_AUX_REQ_FILE, IR_FRONT_REQ_FILE, set_front_hit, set_ir_sample_result
    from rover_esp_servo import SERVO_ANGLE_FILE

    class _Bridge(Node):
        def __init__(self) -> None:
            super().__init__('rover_esp_bridge')
            self.create_subscription(Bool, '/rover/button', self._on_button, 10)
            self.create_subscription(UInt8, '/rover/button_event', self._on_button_event, 10)
            self.create_subscription(UInt8, '/rover/drive_mode', self._on_drive_mode, 10)
            self.create_subscription(Bool, '/rover/ir/front', self._on_ir_front, 10)
            self.create_subscription(UInt8, '/rover/ir/aux_result', self._on_ir_aux_result, 10)
            self._session_pub = self.create_publisher(Bool, '/rover/session', 10)
            self._heartbeat_pub = self.create_publisher(UInt32, '/rover/heartbeat', 10)
            self._aux_sample_pub = self.create_publisher(UInt8, '/rover/ir/aux_sample', 10)
            self._servo_pub = self.create_publisher(Float32, '/rover/servo/angle', 10)
            self._heartbeat_seq = 0
            self._menu_open = False
            self._power_open = False
            self._timer = self.create_timer(0.5, self._tick)
            self._aux_poll = self.create_timer(0.02, self._poll_ir_sample_req)
            self._servo_poll = self.create_timer(0.02, self._poll_servo_req)
            self._tick()

        def _on_button(self, msg: Bool) -> None:
            if msg.data:
                _notify_button_press()

        def _on_button_event(self, msg: UInt8) -> None:
            code = int(msg.data)
            if code == BTN_GO_LONG:
                if self._power_open:
                    self._power_open = False
                    _beep('menu_done')
                    return
                self._menu_open = not self._menu_open
                _beep('menu' if self._menu_open else 'menu_done')
            elif code == BTN_GO_TRIPLE:
                self._power_open = True
                self._menu_open = False
                _beep('menu')
            elif code == BTN_POWER_SHUTDOWN:
                self._power_open = False
                _handle_power_cmd('shutdown')
            elif code == BTN_POWER_REBOOT:
                self._power_open = False
                _handle_power_cmd('reboot')
            elif code == BTN_ESTOP_LONG:
                _handle_estop_long()
            elif code == BTN_MENU_CYCLE:
                _beep('menu')
            elif code == BTN_GO_SHORT:
                _notify_button_press()

        def _on_drive_mode(self, msg: UInt8) -> None:
            _write_drive_mode(int(msg.data))

        def _on_ir_front(self, msg: Bool) -> None:
            set_front_hit(bool(msg.data))

        def _on_ir_aux_result(self, msg: UInt8) -> None:
            set_ir_sample_result(int(msg.data))

        def _poll_ir_sample_req(self) -> None:
            front = False
            try:
                if IR_FRONT_REQ_FILE.exists():
                    IR_FRONT_REQ_FILE.unlink(missing_ok=True)
                    front = True
                elif IR_AUX_REQ_FILE.exists():
                    IR_AUX_REQ_FILE.unlink(missing_ok=True)
                else:
                    return
            except OSError:
                return
            out = UInt8()
            out.data = 2 if front else 1
            self._aux_sample_pub.publish(out)

        def _poll_servo_req(self) -> None:
            try:
                if not SERVO_ANGLE_FILE.exists():
                    return
                text = SERVO_ANGLE_FILE.read_text().strip()
                SERVO_ANGLE_FILE.unlink(missing_ok=True)
            except OSError:
                return
            try:
                deg = float(text)
            except ValueError:
                return
            out = Float32()
            out.data = deg
            self._servo_pub.publish(out)

        def _tick(self) -> None:
            global _LAST_PUBLISHED_SESSION
            active = _read_session_active()

            self._heartbeat_seq = (self._heartbeat_seq + 1) % 0x7FFF_FFFF
            ping = UInt32()
            ping.data = self._heartbeat_seq
            if active:
                ping.data |= 0x8000_0000
            self._heartbeat_pub.publish(ping)

            if active != _LAST_PUBLISHED_SESSION:
                out = Bool()
                out.data = active
                self._session_pub.publish(out)
                _LAST_PUBLISHED_SESSION = active

    if not rclpy.ok():
        rclpy.init()
    node = _Bridge()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        try:
            if rclpy.ok():
                out = Bool()
                out.data = False
                node._session_pub.publish(out)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def ensure_bridge() -> None:
    global _THREAD
    with _BRIDGE_LOCK:
        if _THREAD is not None and _THREAD.is_alive():
            return
        _THREAD = threading.Thread(target=_run_bridge, name='rover-esp-bridge', daemon=True)
        _THREAD.start()


def bridge_managed_externally() -> bool:
    return os.environ.get('ROVER_BRIDGE_EXTERNAL', '').strip() == '1'


def ensure_listener() -> None:
    ensure_bridge()


def clear_button_state() -> None:
    """Drop stale Go / E-stop flags before starting or after stopping a session."""
    global _PENDING, _SHUTDOWN_PENDING
    with _BRIDGE_LOCK:
        _PENDING = False
        _SHUTDOWN_PENDING = False
    try:
        BUTTON_FILE.unlink(missing_ok=True)
        SHUTDOWN_FILE.unlink(missing_ok=True)
    except OSError:
        pass


def consume_press() -> bool:
    """Non-blocking — True once per ESP Go short press."""
    global _PENDING
    if not use_esp_button():
        return False
    with _BRIDGE_LOCK:
        if _PENDING:
            _PENDING = False
            try:
                BUTTON_FILE.unlink(missing_ok=True)
            except OSError:
                pass
            return True
    try:
        if BUTTON_FILE.exists():
            BUTTON_FILE.unlink()
            return True
    except OSError:
        pass
    return False


def consume_shutdown() -> bool:
    """Non-blocking — True once per long E-stop shutdown request."""
    global _SHUTDOWN_PENDING
    if not use_esp_button():
        return False
    with _BRIDGE_LOCK:
        pending = _SHUTDOWN_PENDING
        _SHUTDOWN_PENDING = False
    if pending:
        try:
            SHUTDOWN_FILE.unlink(missing_ok=True)
        except OSError:
            pass
        return True
    try:
        if SHUTDOWN_FILE.exists():
            SHUTDOWN_FILE.unlink()
            return True
    except OSError:
        pass
    return False


def wait_for_press() -> None:
    set_session_active(False)
    if not bridge_managed_externally():
        ensure_bridge()
    clear_button_state()
    time.sleep(0.12)
    clear_button_state()
    print(
        'Waiting for ESP Go (hold=mode, double=power, tap=start)...',
        flush=True,
    )
    while True:
        if consume_shutdown():
            print('Long E-stop received while waiting.', flush=True)
            continue
        if consume_press():
            mode = read_drive_mode()
            print(f'ESP Go pressed — starting {drive_mode_name(mode)}.', flush=True)
            return
        time.sleep(0.02)


def close_listener() -> None:
    global _PENDING, _LAST_PUBLISHED_SESSION, _SHUTDOWN_PENDING, _THREAD
    with _BRIDGE_LOCK:
        _PENDING = False
        _SHUTDOWN_PENDING = False
        _LAST_PUBLISHED_SESSION = None
        _THREAD = None
    set_session_active(False)
    try:
        BUTTON_FILE.unlink(missing_ok=True)
        SHUTDOWN_FILE.unlink(missing_ok=True)
    except OSError:
        pass
