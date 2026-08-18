"""Shared gpiozero LGPIO factory — one chip handle for button + buzzer."""
from __future__ import annotations

_factory = None


def pin_factory():
    global _factory
    if _factory is not None:
        return _factory
    try:
        from gpiozero.pins.lgpio import LGPIOFactory

        _factory = LGPIOFactory()
    except ImportError:
        _factory = None
    return _factory
