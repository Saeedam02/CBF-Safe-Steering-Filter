from __future__ import annotations
from dataclasses import dataclass
from typing import Sequence

from .cbf import HOCBFConstraint
from .utils import clamp


@dataclass(frozen=True)
class QPResult:
    omega: float
    nominal: float
    lower: float
    upper: float
    active: bool
    feasible: bool
    objective: float
    binding_obstacles: tuple[str, ...]


def solve_scalar_qp(
    nominal: float,
    constraints: Sequence[HOCBFConstraint],
    omega_min: float,
    omega_max: float,
    tol: float = 1e-10,
) -> QPResult:
    """Solve the one-dimensional hard-constrained CBF-QP exactly.

    min_omega  0.5 * (omega - nominal)^2
    s.t.       a_i * omega + b_i >= 0
               omega_min <= omega <= omega_max

    In one dimension, every affine constraint clips the feasible set to an
    interval. The QP solution is simply the Euclidean projection of the
    nominal command onto that interval. This is the exact QP solution, not a
    heuristic approximation.
    """
    lower, upper = omega_min, omega_max
    binding = []
    feasible = True

    for c in constraints:
        if abs(c.a) <= tol:
            if c.b < -tol:
                feasible = False
                binding.append(c.obstacle_label)
            continue
        threshold = -c.b / c.a
        if c.a > 0.0:
            if threshold > lower:
                lower = threshold
                binding.append(c.obstacle_label)
        else:
            if threshold < upper:
                upper = threshold
                binding.append(c.obstacle_label)

    if lower > upper + tol:
        feasible = False

    if feasible:
        omega = clamp(nominal, lower, upper)
    else:
        # Best-effort command used only so the simulator can keep producing
        # diagnostics. A formal guarantee no longer applies if the QP is infeasible.
        candidates = [omega_min, omega_max, clamp(nominal, omega_min, omega_max)]
        def score(w: float) -> tuple[float, float]:
            worst = min((c.value(w) for c in constraints), default=0.0)
            return (worst, -abs(w - nominal))
        omega = max(candidates, key=score)
        lower, upper = omega_min, omega_max

    objective = 0.5 * (omega - nominal) ** 2
    active = abs(omega - nominal) > 1e-8
    return QPResult(
        omega=omega,
        nominal=nominal,
        lower=lower,
        upper=upper,
        active=active,
        feasible=feasible,
        objective=objective,
        binding_obstacles=tuple(dict.fromkeys(binding)),
    )
