import math
from cbf_safe_steering.scenarios import get_scenario
from cbf_safe_steering.simulation import run_simulation


def test_default_nominal_collides_but_cbf_stays_safe():
    s=get_scenario("straight")
    nominal=run_simulation(s,False)
    safe=run_simulation(s,True)
    assert nominal.collision
    assert not safe.collision
    assert safe.qp_feasible.all()
    # Sampled numerical implementation should retain essentially all design margin.
    assert safe.minimum_clearance > -0.03
    assert safe.qp_active.any()


def test_safe_steering_never_exceeds_actuator_limit():
    s=get_scenario("straight")
    safe=run_simulation(s,True)
    assert max(abs(safe.steer_applied)) <= s.vehicle.max_steer + 1e-10


def test_far_from_obstacle_filter_is_initially_inactive():
    s=get_scenario("straight")
    safe=run_simulation(s,True)
    assert not safe.qp_active[0]
