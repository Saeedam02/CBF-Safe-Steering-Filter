from __future__ import annotations
import math


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
