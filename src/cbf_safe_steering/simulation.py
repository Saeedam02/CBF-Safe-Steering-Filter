from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np

from .models import step_unicycle
from .cbf import build_hocbf_constraint
from .qp import solve_scalar_qp
from .scenarios import Scenario


@dataclass
class SimulationTrace:
    scenario_name: str
    use_cbf: bool
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray
    omega_nominal: np.ndarray
    omega_applied: np.ndarray
    steer_applied: np.ndarray
    min_clearance: np.ndarray
    min_h: np.ndarray
    qp_lower: np.ndarray
    qp_upper: np.ndarray
    qp_active: np.ndarray
    qp_feasible: np.ndarray
    collision: bool
    reached_goal: bool

    @property
    def minimum_clearance(self) -> float:
        return float(np.min(self.min_clearance))

    @property
    def intervention_fraction(self) -> float:
        return float(np.mean(self.qp_active)) if len(self.qp_active) else 0.0


def _goal_reached(x: float, y: float, goal: tuple[float, float], tol: float) -> bool:
    return math.hypot(x - goal[0], y - goal[1]) <= tol


def run_simulation(scenario: Scenario, use_cbf: bool = True) -> SimulationTrace:
    state = scenario.start
    max_steps = int(math.ceil(scenario.horizon / scenario.dt))

    records = []
    collision = False
    reached = False

    for _ in range(max_steps + 1):
        nominal = scenario.controller.command(state, scenario.goal, scenario.vehicle.max_yaw_rate)
        constraints = [build_hocbf_constraint(state, o, scenario.vehicle, scenario.cbf) for o in scenario.obstacles]

        min_clearance = min(c.clearance for c in constraints)
        min_h = min(c.h for c in constraints)

        if use_cbf:
            qp = solve_scalar_qp(
                nominal,
                constraints,
                -scenario.vehicle.max_yaw_rate,
                scenario.vehicle.max_yaw_rate,
            )
            applied = qp.omega
            qpl, qpu, active, feasible = qp.lower, qp.upper, qp.active, qp.feasible
        else:
            applied = nominal
            qpl, qpu, active, feasible = -scenario.vehicle.max_yaw_rate, scenario.vehicle.max_yaw_rate, False, True

        steer = scenario.vehicle.yaw_rate_to_steer(applied)
        records.append((state.t, state.x, state.y, state.psi, nominal, applied, steer, min_clearance, min_h, qpl, qpu, active, feasible))

        # Physical collision ignores the extra design margin; it uses obstacle + vehicle radii.
        for obstacle in scenario.obstacles:
            physical_clearance = math.hypot(state.x - obstacle.x, state.y - obstacle.y) - (obstacle.radius + scenario.vehicle.vehicle_radius)
            if physical_clearance < 0.0:
                collision = True
                break
        if collision:
            break
        if _goal_reached(state.x, state.y, scenario.goal, scenario.goal_tolerance):
            reached = True
            break

        state = step_unicycle(state, applied, scenario.vehicle, scenario.dt)

    arr = np.asarray(records, dtype=float)
    return SimulationTrace(
        scenario_name=scenario.name,
        use_cbf=use_cbf,
        t=arr[:, 0], x=arr[:, 1], y=arr[:, 2], psi=arr[:, 3],
        omega_nominal=arr[:, 4], omega_applied=arr[:, 5], steer_applied=arr[:, 6],
        min_clearance=arr[:, 7], min_h=arr[:, 8], qp_lower=arr[:, 9], qp_upper=arr[:, 10],
        qp_active=arr[:, 11].astype(bool), qp_feasible=arr[:, 12].astype(bool),
        collision=collision, reached_goal=reached,
    )
