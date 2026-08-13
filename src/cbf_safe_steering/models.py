from __future__ import annotations
from dataclasses import dataclass
import math

from .utils import wrap_angle, clamp


@dataclass(frozen=True)
class VehicleState:
    x: float
    y: float
    psi: float
    t: float = 0.0


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float
    label: str = "obstacle"


@dataclass(frozen=True)
class VehicleParams:
    """Fixed-speed kinematic model parameters.

    The safety filter optimizes yaw rate omega. For a kinematic bicycle,
    omega and steering angle delta are related by
        omega = v/L * tan(delta).
    """
    speed: float = 2.0
    wheelbase: float = 2.7
    max_steer: float = math.radians(35.0)
    vehicle_radius: float = 0.35

    @property
    def max_yaw_rate(self) -> float:
        return self.speed / self.wheelbase * math.tan(self.max_steer)

    def yaw_rate_to_steer(self, omega: float) -> float:
        omega = clamp(omega, -self.max_yaw_rate, self.max_yaw_rate)
        if abs(self.speed) < 1e-12:
            return 0.0
        return math.atan(self.wheelbase * omega / self.speed)


def step_unicycle(state: VehicleState, omega: float, params: VehicleParams, dt: float) -> VehicleState:
    """Exact zero-order-hold step for fixed speed and constant yaw rate."""
    omega = clamp(omega, -params.max_yaw_rate, params.max_yaw_rate)
    v = params.speed
    psi0 = state.psi
    psi1 = psi0 + omega * dt

    if abs(omega) < 1e-10:
        x1 = state.x + v * math.cos(psi0) * dt
        y1 = state.y + v * math.sin(psi0) * dt
    else:
        x1 = state.x + (v / omega) * (math.sin(psi1) - math.sin(psi0))
        y1 = state.y - (v / omega) * (math.cos(psi1) - math.cos(psi0))

    return VehicleState(x=x1, y=y1, psi=wrap_angle(psi1), t=state.t + dt)
