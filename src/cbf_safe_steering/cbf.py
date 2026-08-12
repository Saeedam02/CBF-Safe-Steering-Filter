from __future__ import annotations
from dataclasses import dataclass
import math

from .models import VehicleState, VehicleParams, Obstacle


@dataclass(frozen=True)
class HOCBFConfig:
    """Second-order exponential/HOCBF gains and geometric margin."""
    k1: float = 1.0
    k2: float = 1.4
    safety_margin: float = 0.45


@dataclass(frozen=True)
class HOCBFConstraint:
    """One affine inequality a*omega + b >= 0."""
    a: float
    b: float
    h: float
    h_dot: float
    psi1: float
    clearance: float
    safe_radius: float
    obstacle_label: str

    def value(self, omega: float) -> float:
        return self.a * omega + self.b


def build_hocbf_constraint(
    state: VehicleState,
    obstacle: Obstacle,
    params: VehicleParams,
    config: HOCBFConfig,
) -> HOCBFConstraint:
    """Construct a relative-degree-2 obstacle-avoidance HOCBF constraint.

    Model:
        x_dot = v cos(psi)
        y_dot = v sin(psi)
        psi_dot = omega

    Barrier:
        h = (x-xo)^2 + (y-yo)^2 - R^2

    With p = -(x-xo) sin(psi) + (y-yo) cos(psi):
        h_dot  = 2v[(x-xo)cos(psi) + (y-yo)sin(psi)]
        h_ddot = 2v^2 + 2v p omega

    Second-order exponential CBF:
        h_ddot + (k1+k2) h_dot + k1*k2*h >= 0

    Therefore:
        a*omega + b >= 0.
    """
    dx = state.x - obstacle.x
    dy = state.y - obstacle.y
    dist = math.hypot(dx, dy)
    R = obstacle.radius + params.vehicle_radius + config.safety_margin

    h = dx * dx + dy * dy - R * R
    directional = dx * math.cos(state.psi) + dy * math.sin(state.psi)
    h_dot = 2.0 * params.speed * directional

    p = -dx * math.sin(state.psi) + dy * math.cos(state.psi)
    a = 2.0 * params.speed * p
    b = (
        2.0 * params.speed * params.speed
        + (config.k1 + config.k2) * h_dot
        + config.k1 * config.k2 * h
    )
    psi1 = h_dot + config.k1 * h
    clearance = dist - R
    return HOCBFConstraint(
        a=a,
        b=b,
        h=h,
        h_dot=h_dot,
        psi1=psi1,
        clearance=clearance,
        safe_radius=R,
        obstacle_label=obstacle.label,
    )
