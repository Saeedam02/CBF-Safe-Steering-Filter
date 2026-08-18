from dataclasses import replace
import math

import pytest

from cbf_safe_steering.scenarios import available_scenarios, get_scenario
from cbf_safe_steering.simulation import run_simulation


def test_default_nominal_collides_but_cbf_stays_safe():
    scenario = get_scenario("straight")
    nominal = run_simulation(scenario, False)
    safe = run_simulation(scenario, True)

    assert nominal.collision
    assert not safe.collision
    assert safe.qp_feasible.all()
    # Sampled numerical implementation should retain essentially all design margin.
    assert safe.minimum_clearance > -0.03
    assert safe.qp_active.any()


def test_safe_steering_never_exceeds_actuator_limit():
    scenario = get_scenario("straight")
    safe = run_simulation(scenario, True)
    assert max(abs(safe.steer_applied)) <= scenario.vehicle.max_steer + 1e-10


def test_far_from_obstacle_filter_is_initially_inactive():
    scenario = get_scenario("straight")
    safe = run_simulation(scenario, True)
    assert not safe.qp_active[0]


@pytest.mark.parametrize("name", available_scenarios())
def test_all_checked_in_scenarios_produce_finite_consistent_traces(name):
    scenario = get_scenario(name)
    safe = run_simulation(scenario, True)

    assert len(safe.t) > 1
    assert len(safe.t) == len(safe.x) == len(safe.qp_feasible)
    assert all(math.isfinite(value) for value in safe.x)
    assert all(math.isfinite(value) for value in safe.y)
    assert max(abs(safe.steer_applied)) <= scenario.vehicle.max_steer + 1e-10


def test_straight_scenario_is_stable_under_smaller_sample_period():
    baseline = get_scenario("straight")
    finer = replace(baseline, dt=baseline.dt / 2.0)
    safe = run_simulation(finer, True)

    assert not safe.collision
    assert safe.qp_feasible.all()
    assert safe.minimum_clearance > -0.03

