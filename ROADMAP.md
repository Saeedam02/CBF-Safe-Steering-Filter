# Technical roadmap

The current `v1.x` line intentionally keeps the control input one-dimensional: fixed longitudinal speed with yaw rate as the QP decision variable. That makes the safety filter analytically solvable and easy to inspect.

## v2.0 — joint braking + steering

The next major technical version should promote longitudinal speed to a state and optimize longitudinal acceleration and yaw rate together.

### Proposed model

For state \(z=[x,y,\psi,v]^T\) and control \(u=[a,\omega]^T\):

$$
\dot x=v\cos\psi,\qquad
\dot y=v\sin\psi,\qquad
\dot\psi=\omega,\qquad
\dot v=a.
$$

For the same circular-obstacle barrier

$$
h=(x-x_o)^2+(y-y_o)^2-R^2,
$$

define

$$
q=(x-x_o)\cos\psi+(y-y_o)\sin\psi,
$$

$$
p=-(x-x_o)\sin\psi+(y-y_o)\cos\psi.
$$

Then

$$
\dot h=2vq,
$$

and

$$
\ddot h=2v^2+2qa+2vp\omega.
$$

The second-order HOCBF condition therefore remains affine in the two control inputs:

$$
2q\,a+2vp\,\omega
+2v^2+(k_1+k_2)\dot h+k_1k_2h\ge0.
$$

### Proposed QP

Use a weighted least-change objective around nominal acceleration and yaw rate:

$$
\min_{a,\omega}\;
\frac12 w_a(a-a_{nom})^2+
\frac12 w_\omega(\omega-\omega_{nom})^2,
$$

subject to one HOCBF inequality per obstacle and physical bounds such as

$$
a_{min}\le a\le a_{max},
\qquad
-\omega_{max}(v)\le\omega\le\omega_{max}(v).
$$

At this point a general convex QP solver such as OSQP becomes appropriate; the scalar interval-projection solver should remain available for the educational v1 mode.

### Acceptance criteria

- Nominal steering-only baseline still reproduces the v1 results.
- Joint control can choose braking, steering, or both depending on which causes the smallest weighted intervention.
- Hard-QP infeasibility is explicit and tested.
- Acceleration, speed, steering, and yaw-rate limits are tested at boundaries.
- HOCBF residuals are checked numerically at every feasible sampled step.
- New plots show nominal/applied acceleration, speed, and steering activity.
- README safety claims are updated to match the new model and sampling assumptions.

## Later milestones

1. Moving obstacles with obstacle velocity in the barrier derivatives.
2. Lane-boundary CBFs.
3. Robust CBF margins for bounded state/model uncertainty.
4. Sampled-data or discrete-time safety conditions for inter-sample guarantees.
5. Dynamic bicycle dynamics with lateral velocity, yaw dynamics, and tire-force limits.
6. Steering-angle/rate state constraints with appropriate higher-order barriers.
7. Hardware-in-the-loop integration and timing/solver diagnostics.
