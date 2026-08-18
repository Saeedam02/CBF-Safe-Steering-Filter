import math

import pytest

from cbf_safe_steering.models import VehicleParams, VehicleState, step_unicycle


def test_yaw_rate_to_steer_saturates_to_physical_limit():
    params = VehicleParams(speed=2.0, wheelbase=2.7, max_steer=math.radians(35.0))
    assert params.yaw_rate_to_steer(1e6) == pytest.approx(params.max_steer)
    assert params.yaw_rate_to_steer(-1e6) == pytest.approx(-params.max_steer)


def test_zero_speed_yaw_rate_conversion_returns_zero():
    params = VehicleParams(speed=0.0)
    assert params.yaw_rate_to_steer(1.0) == 0.0


def test_exact_straight_line_step():
    params = VehicleParams(speed=2.0)
    state = VehicleState(1.0, -2.0, math.pi / 2.0)
    next_state = step_unicycle(state, 0.0, params, 0.25)
    assert next_state.x == pytest.approx(1.0)
    assert next_state.y == pytest.approx(-1.5)
    assert next_state.t == pytest.approx(0.25)


def test_constant_yaw_rate_step_matches_circle_geometry():
    params = VehicleParams(speed=2.0, max_steer=math.radians(60.0))
    state = VehicleState(0.0, 0.0, 0.0)
    omega = 0.4
    dt = 0.5
    next_state = step_unicycle(state, omega, params, dt)
    expected_psi = omega * dt
    assert next_state.x == pytest.approx((params.speed / omega) * math.sin(expected_psi))
    assert next_state.y == pytest.approx(
        -(params.speed / omega) * (math.cos(expected_psi) - 1.0)
    )
