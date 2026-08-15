from cbf_safe_steering.cbf import HOCBFConstraint
from cbf_safe_steering.qp import solve_scalar_qp


def C(a,b,label="o"):
    return HOCBFConstraint(a=a,b=b,h=1,h_dot=0,psi1=1,clearance=1,safe_radius=1,obstacle_label=label)


def test_qp_returns_nominal_when_feasible():
    r=solve_scalar_qp(0.2,[C(1,1)],-1,1)
    assert r.feasible and not r.active
    assert abs(r.omega-0.2)<1e-12


def test_qp_projects_to_lower_bound():
    # omega - 0.5 >= 0
    r=solve_scalar_qp(0.0,[C(1,-0.5)],-1,1)
    assert r.feasible and r.active
    assert abs(r.omega-0.5)<1e-12


def test_qp_detects_infeasible_constant_constraint():
    r=solve_scalar_qp(0.0,[C(0,-1)],-1,1)
    assert not r.feasible
