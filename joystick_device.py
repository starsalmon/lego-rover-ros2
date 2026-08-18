"""Joystick input — Linux /dev/input/js0 or pygame (macOS / fallback)."""
from __future__ import annotations

import atexit
import glob
import os
import struct
import sys
import time
from typing import Protocol

if sys.platform == 'darwin':
    # Must be set before pygame/SDL loads (macOS GameController needs a display).
    os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
    os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')
    # Tiny off-screen window — HIDDEN breaks GameController axis updates on some macOS/SDL builds.
    os.environ.setdefault('SDL_VIDEO_WINDOW_POS', '-2400,-2400')

JS_EVENT_FORMAT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02

_pygame_session: 'PygameDevice | None' = None


class JoystickDevice(Protocol):
    axes: dict[int, float]
    buttons: dict[int, int]

    def poll(self) -> None: ...
    def close(self) -> None: ...


def _find_js_path() -> str:
    want = os.environ.get('JOY_DEVICE', '').strip()
    if want and os.path.exists(want):
        return want
    if os.path.exists('/dev/input/js0'):
        return '/dev/input/js0'
    for path in sorted(glob.glob('/dev/input/by-id/*-joystick')):
        if 'event-joystick' not in path:
            return path
    for path in ('/dev/input/js1', '/dev/input/js2'):
        if os.path.exists(path):
            return path
    return ''


class LinuxJsDevice:
    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes: dict[int, float] = {}
        self.buttons: dict[int, int] = {}

    def poll(self) -> None:
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE)
            except BlockingIOError:
                break
            if len(data) < JS_EVENT_SIZE:
                break
            _time, value, evtype, number = struct.unpack(JS_EVENT_FORMAT, data)
            if evtype & JS_EVENT_BUTTON:
                self.buttons[number] = value
            elif evtype & JS_EVENT_AXIS:
                self.axes[number] = value / 32767.0

    def close(self) -> None:
        os.close(self.fd)


class PygameDevice:
    def __init__(self, index: int = 0):
        import pygame

        self._pygame = pygame
        self._index = index
        self.joy: pygame.joystick.Joystick | None = None
        self.name = ''
        self.axes: dict[int, float] = {}
        self.buttons: dict[int, int] = {}
        self._needs_reconnect = False
        self._last_reconnect = 0.0
        self._screen = None
        self._event_axes_tick: set[int] = set()
        self._attach(index)

    def _ensure_display(self) -> None:
        """macOS GameController.framework needs the video subsystem + event loop."""
        pg = self._pygame
        if self._screen is not None:
            return
        if not pg.display.get_init():
            pg.display.init()
        self._screen = pg.display.set_mode((64, 64))

    @staticmethod
    def _darwin_runloop_tick() -> None:
        """Nudge CoreFoundation so Bluetooth gamepad events reach SDL."""
        try:
            import ctypes

            cf = ctypes.CDLL(
                '/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation'
            )
            cf.CFRunLoopRunInMode(ctypes.c_void_p.in_dll(cf, 'kCFRunLoopDefaultMode'), 0.0, True)
        except Exception:
            pass

    def _event_instance(self, event) -> int:
        return int(getattr(event, 'instance_id', getattr(event, 'joy', -1)))

    def _pump_events(self) -> None:
        pg = self._pygame
        inst = None
        if self.joy is not None:
            try:
                inst = self.joy.get_instance_id()
            except pg.error:
                inst = None

        pg.event.pump()
        self._event_axes_tick.clear()
        for event in pg.event.get():
            et = event.type
            if et == pg.JOYAXISMOTION:
                if inst is None or self._event_instance(event) == inst:
                    ax = int(event.axis)
                    self.axes[ax] = float(event.value)
                    self._event_axes_tick.add(ax)
            elif et == pg.JOYBUTTONDOWN:
                if inst is None or self._event_instance(event) == inst:
                    self.buttons[event.button] = 1
            elif et == pg.JOYBUTTONUP:
                if inst is None or self._event_instance(event) == inst:
                    self.buttons[event.button] = 0
            elif et in (pg.JOYDEVICEREMOVED,):
                self._needs_reconnect = True

        if sys.platform == 'darwin':
            self._darwin_runloop_tick()

        if self._screen is not None:
            pg.display.flip()

    def _attach(self, index: int) -> bool:
        pg = self._pygame
        if not pg.get_init():
            pg.init()
        if sys.platform == 'darwin':
            self._ensure_display()

        if self.joy is not None:
            try:
                self.joy.quit()
            except pg.error:
                pass
            self.joy = None

        if not pg.joystick.get_init():
            pg.joystick.init()
        if pg.joystick.get_count() < 1:
            return False
        index = min(index, pg.joystick.get_count() - 1)
        self._index = index
        self.joy = pg.joystick.Joystick(index)
        self.joy.init()
        self.name = self.joy.get_name()
        self.axes.clear()
        self.buttons.clear()
        for _ in range(5):
            try:
                self._pump_events()
            except pg.error:
                break
        self._read_state()
        return True

    def _alive(self) -> bool:
        if self.joy is None:
            return False
        try:
            if self._pygame.joystick.get_count() < 1:
                return False
            _ = self.joy.get_numaxes()
            return bool(self.joy.get_init())
        except self._pygame.error:
            return False

    def _reconnect(self) -> bool:
        now = time.monotonic()
        if now - self._last_reconnect < 1.0:
            return self._alive()
        self._last_reconnect = now
        print('Gamepad reconnecting...', flush=True)
        if self._attach(self._index):
            print(f'Gamepad OK: {self.name}', flush=True)
            self._needs_reconnect = False
            return True
        return False

    def _read_state(self) -> bool:
        if self.joy is None:
            return False
        pg = self._pygame
        try:
            for i in range(self.joy.get_numaxes()):
                # Stick axes: always merge get_axis (Mac BT often misses negative X in events).
                if sys.platform == 'darwin' and i in self._event_axes_tick and i > 3:
                    continue
                self.axes[i] = float(self.joy.get_axis(i))
            for i in range(self.joy.get_numbuttons()):
                self.buttons[i] = int(self.joy.get_button(i))
            return True
        except (pg.error, SystemError, KeyError, OSError):
            return False

    def poll(self) -> None:
        pg = self._pygame
        try:
            self._pump_events()
        except (pg.error, SystemError, KeyError, OSError):
            self._needs_reconnect = True

        if not self._read_state():
            self._needs_reconnect = True

        if self._needs_reconnect or not self._alive():
            self._reconnect()
            try:
                self._pump_events()
            except (pg.error, SystemError, KeyError, OSError):
                pass
            self._read_state()

    def close(self) -> None:
        global _pygame_session
        pg = self._pygame
        if self.joy is not None:
            try:
                self.joy.quit()
            except pg.error:
                pass
            self.joy = None
        if self._screen is not None:
            try:
                pg.display.quit()
            except pg.error:
                pass
            self._screen = None
        if pg.joystick.get_init():
            try:
                pg.joystick.quit()
            except pg.error:
                pass
        if pg.get_init():
            try:
                pg.quit()
            except pg.error:
                pass
        if _pygame_session is self:
            _pygame_session = None


def _shutdown_pygame_session() -> None:
    global _pygame_session
    if _pygame_session is not None:
        _pygame_session.close()
        _pygame_session = None


atexit.register(_shutdown_pygame_session)


def open_joystick() -> tuple[JoystickDevice, str] | None:
    """Return (device, description) or None if no joystick found."""
    global _pygame_session

    path = _find_js_path()
    if path:
        try:
            return LinuxJsDevice(path), path
        except OSError:
            pass

    try:
        import pygame  # noqa: F401
    except ImportError:
        return None

    if _pygame_session is not None and _pygame_session._alive():
        return _pygame_session, f'pygame:{_pygame_session.name}'

    dev = PygameDevice(0)
    if dev.joy is None:
        dev.close()
        return None
    _pygame_session = dev
    return dev, f'pygame:{dev.name}'
