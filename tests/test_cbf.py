import math
from cbf_safe_steering.models import VehicleState, VehicleParams, Obstacle
from cbf_safe_steering.cbf import HOCBFConfig, build_hocbf_constraint


def test_barrier_sign_outside_and_inside():
    params = VehicleParams(vehicle_radius=0.3)
    obs = Obstacle(0.0, 0.0, 1.0)
    cfg = HOCBFConfig(safety_margin=0.2)
    outside = build_hocbf_constraint(VehicleState(2.0,0.0,0.0), obs, params, cfg)
    inside = build_hocbf_constraint(VehicleState(0.5,0.0,0.0), obs, params, cfg)
    assert outside.h > 0
    assert inside.h < 0


def test_hdot_matches_finite_difference():
    params = VehicleParams(speed=1.7)
    obs = Obstacle(3.0,-1.0,0.8)
    cfg = HOCBFConfig()
    s = VehicleState(0.7,0.2,0.43)
    c = build_hocbf_constraint(s,obs,params,cfg)
    dt=1e-6
    s2=VehicleState(s.x+params.speed*math.cos(s.psi)*dt, s.y+params.speed*math.sin(s.psi)*dt, s.psi)
    c2=build_hocbf_constraint(s2,obs,params,cfg)
    approx=(c2.h-c.h)/dt
    assert abs(approx-c.h_dot) < 1e-4
