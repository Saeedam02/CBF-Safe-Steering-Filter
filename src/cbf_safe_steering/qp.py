from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from .cbf import HOCBFConstraint
from .utils import clamp


@dataclass(frozen=True)
class QPResult:
    """Result of the exact one-dimensional hard-constrained CBF-QP."""

    omega: float
    nominal: float
    lower: float
    upper: float
    active: bool
    feasible: bool
    objective: float
    limiting_obstacles: tuple[str, ...]
    active_obstacles: tuple[str, ...]

    @property
    def binding_obstacles(self) -> tuple[str, ...]:
        """Backward-compatible alias for constraints active at the optimum.

        ``binding_obstacles`` was ambiguous in earlier versions because it also
        included constraints that merely tightened an intermediate interval.
        New code should use ``active_obstacles`` or ``limiting_obstacles``.
        """
        return self.active_obstacles


def _unique(labels: list[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(labels))


def solve_scalar_qp(
    nominal: float,
    constraints: Sequence[HOCBFConstraint],
    omega_min: float,
    omega_max: float,
    tol: float = 1e-10,
    active_tol: float = 1e-8,
) -> QPResult:
    """Solve the one-dimensional hard-constrained CBF-QP exactly.

    min_omega  0.5 * (omega - nominal)^2
    s.t.       a_i * omega + b_i >= 0
               omega_min <= omega <= omega_max

    In one dimension, every affine constraint clips the feasible set to an
    interval. The QP solution is the Euclidean projection of the nominal
    command onto that interval. This is the exact QP solution, not a heuristic.

    ``limiting_obstacles`` identifies obstacle constraints that define the final
    lower or upper feasible bound. ``active_obstacles`` identifies constraints
    whose residual is approximately zero at the returned optimum.
    """
    if omega_min > omega_max:
        raise ValueError("omega_min must not exceed omega_max")
    if tol <= 0.0 or active_tol <= 0.0:
        raise ValueError("tol and active_tol must be positive")

    lower, upper = omega_min, omega_max
    lower_labels: list[str] = []
    upper_labels: list[str] = []
    infeasible_constant_labels: list[str] = []

    for constraint in constraints:
        if abs(constraint.a) <= tol:
            if constraint.b < -tol:
                infeasible_constant_labels.append(constraint.obstacle_label)
            continue

        threshold = -constraint.b / constraint.a
        if constraint.a > 0.0:
            if threshold > lower + tol:
                lower = threshold
                lower_labels = [constraint.obstacle_label]
            elif abs(threshold - lower) <= active_tol:
                lower_labels.append(constraint.obstacle_label)
        else:
            if threshold < upper - tol:
                upper = threshold
                upper_labels = [constraint.obstacle_label]
            elif abs(threshold - upper) <= active_tol:
                upper_labels.append(constraint.obstacle_label)

    feasible = not infeasible_constant_labels and lower <= upper + tol
    limiting_labels = lower_labels + upper_labels + infeasible_constant_labels

    if feasible:
        omega = clamp(nominal, lower, upper)
        active_labels = [
            constraint.obstacle_label
            for constraint in constraints
            if abs(constraint.value(omega)) <= active_tol
        ]
    else:
        # Best-effort command used only so the simulator can keep producing
        # diagnostics. A formal safety guarantee no longer applies.
        candidates = [omega_min, omega_max, clamp(nominal, omega_min, omega_max)]

        def score(w: float) -> tuple[float, float]:
            worst = min((constraint.value(w) for constraint in constraints), default=0.0)
            return (worst, -abs(w - nominal))

        omega = max(candidates, key=score)
        active_labels = []
        # Preserve the historical plotting behavior: infeasible traces show the
        # actuator interval instead of an inverted/empty feasible interval.
        lower, upper = omega_min, omega_max

    objective = 0.5 * (omega - nominal) ** 2
    active = abs(omega - nominal) > active_tol
    return QPResult(
        omega=omega,
        nominal=nominal,
        lower=lower,
        upper=upper,
        active=active,
        feasible=feasible,
        objective=objective,
        limiting_obstacles=_unique(limiting_labels),
        active_obstacles=_unique(active_labels),
    )
