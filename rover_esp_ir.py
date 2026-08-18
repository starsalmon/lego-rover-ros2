"""ESP MCP IR bridge state (files + ROS handoff)."""
from __future__ import annotations

import os
import threading
import time
from pathlib import Path

_LOCK = threading.Lock()
_FRONT_HIT = False
_FRONT_OFF_HIT = False
_FRONT_ON_HIT = False
_AUX_RESULT: tuple[bool, bool] | None = None
_FRONT_SAMPLE_RESULT: tuple[bool, bool] | None = None
_AUX_WAIT = threading.Event()
_FRONT_SAMPLE_WAIT = threading.Event()

IR_FRONT_FILE = Path(os.environ.get('ROVER_IR_FRONT_FILE', '/tmp/rover_ir_front'))
IR_FRONT_REQ_FILE = Path(os.environ.get('ROVER_IR_FRONT_REQ_FILE', '/tmp/rover_ir_front_req'))
IR_AUX_REQ_FILE = Path(os.environ.get('ROVER_IR_AUX_REQ_FILE', '/tmp/rover_ir_aux_req'))
IR_AUX_RESULT_FILE = Path(os.environ.get('ROVER_IR_AUX_RESULT_FILE', '/tmp/rover_ir_aux_result'))


def ir_source() -> str:
    return os.environ.get('ROVER_IR_SOURCE', 'esp').strip().lower()


def use_esp_ir() -> bool:
    return ir_source() in ('esp', 'esp32', 'shift', 'mcp', '1', 'true', 'yes')


def set_front_hit(hit: bool) -> None:
    global _FRONT_HIT
    with _LOCK:
        _FRONT_HIT = bool(hit)
    try:
        IR_FRONT_FILE.write_text('1' if hit else '0')
    except OSError:
        pass


def read_front_cached() -> bool:
    # rover_bridge_daemon.py (rover-main.service) updates the file in another process.
    try:
        text = IR_FRONT_FILE.read_text().strip()
        if text in ('0', '1'):
            return text == '1'
    except OSError:
        pass
    with _LOCK:
        return _FRONT_HIT


def front_pulse_state_cached() -> tuple[bool, bool, bool]:
    with _LOCK:
        off_hit = _FRONT_OFF_HIT
        on_hit = _FRONT_ON_HIT
        delta_hit = _FRONT_HIT
    return off_hit, on_hit, delta_hit


def _parse_aux_result_file(text: str) -> tuple[bool, bool] | None:
    parts = text.strip().split(',')
    if len(parts) < 2:
        return None
    try:
        return bool(int(parts[0])), bool(int(parts[1]))
    except ValueError:
        return None


def _parse_front_sample_file(text: str) -> tuple[bool, bool] | None:
    parts = text.strip().split(',')
    if len(parts) < 4:
        return None
    try:
        return bool(int(parts[2])), bool(int(parts[3]))
    except ValueError:
        return None


def _wait_aux_result(timeout_s: float) -> tuple[bool, bool]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _AUX_WAIT.wait(timeout=0.02):
            with _LOCK:
                if _AUX_RESULT is not None:
                    return _AUX_RESULT
        try:
            if IR_AUX_RESULT_FILE.exists():
                parsed = _parse_aux_result_file(IR_AUX_RESULT_FILE.read_text())
                if parsed is not None:
                    return parsed
        except OSError:
            pass
    raise TimeoutError('ESP aux IR sample timed out (is rover_main / bridge running?)')


def _wait_front_sample(timeout_s: float) -> tuple[bool, bool]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _FRONT_SAMPLE_WAIT.wait(timeout=0.02):
            with _LOCK:
                if _FRONT_SAMPLE_RESULT is not None:
                    return _FRONT_SAMPLE_RESULT
        try:
            if IR_AUX_RESULT_FILE.exists():
                parsed = _parse_front_sample_file(IR_AUX_RESULT_FILE.read_text())
                if parsed is not None:
                    return parsed
        except OSError:
            pass
    raise TimeoutError('ESP front IR sample timed out (is rover_main / bridge running?)')


def set_ir_sample_result(bits: int) -> None:
    """Decode /rover/ir/aux_result UInt8: aux off=1 on=2, front off=4 on=8."""
    global _AUX_RESULT, _FRONT_SAMPLE_RESULT, _FRONT_OFF_HIT, _FRONT_ON_HIT
    aux = (bool(bits & 1), bool(bits & 2))
    front = (bool(bits & 4), bool(bits & 8))
    with _LOCK:
        _AUX_RESULT = aux
        _FRONT_SAMPLE_RESULT = front
        _FRONT_OFF_HIT = front[0]
        _FRONT_ON_HIT = front[1]
    try:
        IR_AUX_RESULT_FILE.write_text(
            f'{int(aux[0])},{int(aux[1])},{int(front[0])},{int(front[1])}'
        )
    except OSError:
        pass
    _AUX_WAIT.set()
    _FRONT_SAMPLE_WAIT.set()


def request_aux_sample(timeout_s: float = 0.65) -> tuple[bool, bool]:
    """Ask ESP (via bridge) to run one aux pulse sample."""
    global _AUX_RESULT
    _AUX_WAIT.clear()
    with _LOCK:
        _AUX_RESULT = None
    try:
        IR_AUX_RESULT_FILE.unlink(missing_ok=True)
        IR_AUX_REQ_FILE.write_text('1')
    except OSError as exc:
        raise RuntimeError(f'cannot request ESP aux IR sample: {exc}') from exc

    result = _wait_aux_result(timeout_s)
    with _LOCK:
        _AUX_RESULT = result
    return result


def request_front_sample(timeout_s: float = 0.35) -> tuple[bool, bool]:
    """Ask ESP (via bridge) to run one front gate sample."""
    global _FRONT_SAMPLE_RESULT
    _FRONT_SAMPLE_WAIT.clear()
    with _LOCK:
        _FRONT_SAMPLE_RESULT = None
    try:
        IR_AUX_RESULT_FILE.unlink(missing_ok=True)
        IR_FRONT_REQ_FILE.write_text('1')
    except OSError as exc:
        raise RuntimeError(f'cannot request ESP front IR sample: {exc}') from exc

    result = _wait_front_sample(timeout_s)
    with _LOCK:
        _FRONT_SAMPLE_RESULT = result
    return result


def ensure_bridge_started() -> None:
    """Start rover_esp_bridge as its own process (never in-thread — breaks rclpy spin)."""
    if _bridge_node_running():
        return
    import subprocess
    import sys

    daemon = Path(__file__).resolve().parent / 'rover_bridge_daemon.py'
    subprocess.Popen(
        [sys.executable, str(daemon)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        time.sleep(0.25)
        if _bridge_node_running():
            return


def _bridge_node_running() -> bool:
    try:
        import subprocess

        out = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=4,
            check=False,
        )
        return 'rover_esp_bridge' in (out.stdout or '')
    except Exception:
        return False
