import random

import pytest

from cbf_safe_steering.cbf import HOCBFConstraint
from cbf_safe_steering.qp import solve_scalar_qp


def constraint(a: float, b: float, label: str = "o") -> HOCBFConstraint:
    return HOCBFConstraint(
        a=a,
        b=b,
        h=1.0,
        h_dot=0.0,
        psi1=1.0,
        clearance=1.0,
        safe_radius=1.0,
        obstacle_label=label,
    )


def test_qp_returns_nominal_when_feasible():
    result = solve_scalar_qp(0.2, [constraint(1.0, 1.0)], -1.0, 1.0)
    assert result.feasible and not result.active
    assert result.omega == pytest.approx(0.2)
    assert result.active_obstacles == ()


def test_qp_projects_to_lower_bound_and_reports_active_constraint():
    result = solve_scalar_qp(0.0, [constraint(1.0, -0.5, "lower")], -1.0, 1.0)
    assert result.feasible and result.active
    assert result.omega == pytest.approx(0.5)
    assert result.limiting_obstacles == ("lower",)
    assert result.active_obstacles == ("lower",)
    assert result.binding_obstacles == result.active_obstacles


def test_qp_projects_to_upper_bound():
    result = solve_scalar_qp(0.9, [constraint(-1.0, 0.25, "upper")], -1.0, 1.0)
    assert result.feasible and result.active
    assert result.omega == pytest.approx(0.25)
    assert result.active_obstacles == ("upper",)


def test_nonbinding_constraint_is_not_reported_as_active():
    constraints = [
        constraint(1.0, -0.5, "final lower"),
        constraint(1.0, 0.25, "loose lower"),
    ]
    result = solve_scalar_qp(0.8, constraints, -1.0, 1.0)
    assert result.omega == pytest.approx(0.8)
    assert result.limiting_obstacles == ("final lower",)
    assert result.active_obstacles == ()


def test_tied_limiting_constraints_are_both_reported():
    constraints = [
        constraint(1.0, -0.5, "a"),
        constraint(2.0, -1.0, "b"),
    ]
    result = solve_scalar_qp(0.0, constraints, -1.0, 1.0)
    assert result.omega == pytest.approx(0.5)
    assert result.limiting_obstacles == ("a", "b")
    assert result.active_obstacles == ("a", "b")


def test_qp_detects_infeasible_constant_constraint():
    result = solve_scalar_qp(0.0, [constraint(0.0, -1.0, "constant")], -1.0, 1.0)
    assert not result.feasible
    assert result.limiting_obstacles == ("constant",)
    assert result.active_obstacles == ()


def test_qp_detects_conflicting_bounds():
    constraints = [
        constraint(1.0, -0.75, "turn left"),
        constraint(-1.0, 0.25, "turn right"),
    ]
    result = solve_scalar_qp(0.0, constraints, -1.0, 1.0)
    assert not result.feasible
    assert result.limiting_obstacles == ("turn left", "turn right")
    assert -1.0 <= result.omega <= 1.0


def test_qp_validates_inputs():
    with pytest.raises(ValueError):
        solve_scalar_qp(0.0, [], 1.0, -1.0)
    with pytest.raises(ValueError):
        solve_scalar_qp(0.0, [], -1.0, 1.0, tol=0.0)


def test_random_feasible_qp_solution_satisfies_every_constraint():
    rng = random.Random(20260816)
    checked = 0

    for _ in range(1000):
        constraints = []
        for index in range(rng.randint(0, 6)):
            a = rng.uniform(-3.0, 3.0)
            if abs(a) < 0.05:
                a = 0.05 if a >= 0.0 else -0.05
            b = rng.uniform(-2.0, 2.0)
            constraints.append(constraint(a, b, f"o{index}"))

        result = solve_scalar_qp(
            nominal=rng.uniform(-1.5, 1.5),
            constraints=constraints,
            omega_min=-1.0,
            omega_max=1.0,
        )
        if not result.feasible:
            continue

        checked += 1
        assert -1.0 - 1e-10 <= result.omega <= 1.0 + 1e-10
        assert all(c.value(result.omega) >= -1e-8 for c in constraints)

    assert checked > 100
