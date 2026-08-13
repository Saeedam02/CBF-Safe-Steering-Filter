from __future__ import annotations
from dataclasses import dataclass
import math

from .models import VehicleState, VehicleParams, Obstacle
from .controllers import GoalSeekingController
from .cbf import HOCBFConfig


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    start: VehicleState
    goal: tuple[float, float]
    obstacles: tuple[Obstacle, ...]
    vehicle: VehicleParams
    controller: GoalSeekingController
    cbf: HOCBFConfig
    dt: float = 0.025
    horizon: float = 8.0
    goal_tolerance: float = 0.35


def _straight() -> Scenario:
    return Scenario(
        name="straight",
        description="A naive goal-seeking controller drives almost directly into a circular obstacle.",
        start=VehicleState(0.0, -0.35, 0.0),
        goal=(12.0, 0.0),
        obstacles=(Obstacle(6.0, 0.0, 0.95, "central obstacle"),),
        vehicle=VehicleParams(speed=1.8, wheelbase=2.7, max_steer=math.radians(50), vehicle_radius=0.35),
        controller=GoalSeekingController(heading_gain=1.8),
        cbf=HOCBFConfig(k1=1.0, k2=1.5, safety_margin=0.30),
        dt=0.025,
        horizon=8.0,
    )


def _slalom() -> Scenario:
    return Scenario(
        name="slalom",
        description="Three obstacles force repeated safety-filter interventions while the nominal controller keeps chasing the goal.",
        start=VehicleState(0.0, -0.65, 0.05),
        goal=(15.0, 0.2),
        obstacles=(
            Obstacle(5.0, -0.15, 0.85, "obstacle A"),
            Obstacle(8.3, 1.15, 0.80, "obstacle B"),
            Obstacle(11.2, -0.65, 0.80, "obstacle C"),
        ),
        vehicle=VehicleParams(speed=1.8, wheelbase=2.7, max_steer=math.radians(50), vehicle_radius=0.35),
        controller=GoalSeekingController(heading_gain=1.55),
        cbf=HOCBFConfig(k1=1.2, k2=1.5, safety_margin=0.15),
        dt=0.025,
        horizon=11.0,
    )


def _narrow_gate() -> Scenario:
    return Scenario(
        name="narrow-gate",
        description="Two obstacles create a narrow passage and illustrate QP feasibility limits.",
        start=VehicleState(0.0, -0.25, 0.0),
        goal=(12.0, 0.0),
        obstacles=(
            Obstacle(6.2, 1.35, 0.72, "upper obstacle"),
            Obstacle(6.0, -1.35, 0.72, "lower obstacle"),
        ),
        vehicle=VehicleParams(speed=1.6, wheelbase=2.7, max_steer=math.radians(35), vehicle_radius=0.32),
        controller=GoalSeekingController(heading_gain=1.6),
        cbf=HOCBFConfig(k1=1.0, k2=1.35, safety_margin=0.30),
        dt=0.025,
        horizon=9.0,
    )


_SCENARIOS = {s.name: s for s in (_straight(), _slalom(), _narrow_gate())}


def available_scenarios() -> tuple[str, ...]:
    return tuple(_SCENARIOS)


def get_scenario(name: str) -> Scenario:
    try:
        return _SCENARIOS[name]
    except KeyError as exc:
        raise ValueError(f"Unknown scenario {name!r}. Choose from: {', '.join(available_scenarios())}") from exc
