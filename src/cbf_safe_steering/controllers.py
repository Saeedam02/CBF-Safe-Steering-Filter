from __future__ import annotations
from dataclasses import dataclass
import math

from .models import VehicleState
from .utils import clamp, wrap_angle


@dataclass(frozen=True)
class GoalSeekingController:
    """Simple nominal heading controller.

    It intentionally knows nothing about obstacles. This makes it a useful
    baseline for demonstrating how a CBF can wrap an imperfect controller.
    """
    heading_gain: float = 1.8

    def command(self, state: VehicleState, goal: tuple[float, float], max_yaw_rate: float) -> float:
        gx, gy = goal
        desired = math.atan2(gy - state.y, gx - state.x)
        error = wrap_angle(desired - state.psi)
        return clamp(self.heading_gain * error, -max_yaw_rate, max_yaw_rate)
