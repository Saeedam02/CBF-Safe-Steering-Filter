import math

import pytest

from cbf_safe_steering.cbf import HOCBFConfig, build_hocbf_constraint
from cbf_safe_steering.models import Obstacle, VehicleParams, VehicleState


def test_barrier_sign_outside_boundary_and_inside():
    params = VehicleParams(vehicle_radius=0.3)
    obstacle = Obstacle(0.0, 0.0, 1.0)
    config = HOCBFConfig(safety_margin=0.2)
    safe_radius = obstacle.radius + params.vehicle_radius + config.safety_margin

    outside = build_hocbf_constraint(
        VehicleState(safe_radius + 0.2, 0.0, 0.0), obstacle, params, config
    )
    boundary = build_hocbf_constraint(
        VehicleState(safe_radius, 0.0, 0.0), obstacle, params, config
    )
    inside = build_hocbf_constraint(
        VehicleState(safe_radius - 0.2, 0.0, 0.0), obstacle, params, config
    )

    assert outside.h > 0.0
    assert boundary.h == pytest.approx(0.0, abs=1e-12)
    assert inside.h < 0.0
    assert boundary.clearance == pytest.approx(0.0, abs=1e-12)


def test_hdot_matches_finite_difference():
    params = VehicleParams(speed=1.7)
    obstacle = Obstacle(3.0, -1.0, 0.8)
    config = HOCBFConfig()
    state = VehicleState(0.7, 0.2, 0.43)
    c0 = build_hocbf_constraint(state, obstacle, params, config)

    dt = 1e-6
    state_next = VehicleState(
        state.x + params.speed * math.cos(state.psi) * dt,
        state.y + params.speed * math.sin(state.psi) * dt,
        state.psi,
    )
    c1 = build_hocbf_constraint(state_next, obstacle, params, config)
    approx = (c1.h - c0.h) / dt
    assert approx == pytest.approx(c0.h_dot, abs=1e-4)


def test_psi1_definition_is_consistent():
    params = VehicleParams(speed=2.1)
    obstacle = Obstacle(4.0, 1.0, 0.5)
    config = HOCBFConfig(k1=1.3, k2=1.7, safety_margin=0.25)
    result = build_hocbf_constraint(
        VehicleState(1.0, -0.3, 0.2), obstacle, params, config
    )
    assert result.psi1 == pytest.approx(result.h_dot + config.k1 * result.h)


def test_affine_constraint_matches_second_derivative_formula():
    params = VehicleParams(speed=1.9)
    obstacle = Obstacle(2.5, -0.4, 0.7)
    config = HOCBFConfig(k1=0.9, k2=1.4, safety_margin=0.2)
    state = VehicleState(0.3, 0.8, -0.35)
    omega = 0.27

    result = build_hocbf_constraint(state, obstacle, params, config)
    dx = state.x - obstacle.x
    dy = state.y - obstacle.y
    p = -dx * math.sin(state.psi) + dy * math.cos(state.psi)
    h_ddot = 2.0 * params.speed**2 + 2.0 * params.speed * p * omega
    expected = (
        h_ddot
        + (config.k1 + config.k2) * result.h_dot
        + config.k1 * config.k2 * result.h
    )
    assert result.value(omega) == pytest.approx(expected)
