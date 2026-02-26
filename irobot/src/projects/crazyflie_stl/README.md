# crazyflie_stl — Self-Contained STL Mission Module

## Status

- [x] scenario.py   — mission geometry
- [x] belief.py     — Gaussian belief state
- [x] stl.py        — STL predicates and temporal operators
- [x] controller.py — reactive STL controller
- [x] mission.py    — simulation runner + plot
- [x] cf_runner.py  — hardware runner (real Crazyflie)
- [ ] Phase 2: gradient-based STL optimisation

---

## Mission geometry

```text
x: -1.0          0.0   0.25  0.5   0.75   1.0
    |              |     |    |      |      |
y= 1.0  S (start)
y= 0.75            [─────OBS-1──────]
y= 0.50            [  x[0,0.5]      ]
y= 0.25            [────────────────]
y= 0.0                   ● (through)
y=-1.25                        [────OBS-2────]
y=-1.50                        [ x[0.25,0.75]]
y=-2.0                                        G (goal)
```

- **Start:** (-1.0, 1.0)
- **Goal:**  ( 1.0, -2.0)
- **Through-point:** (0.0, 0.0)
- **OBS-1:** x[0.00, 0.50]  y[0.25, 0.75]
- **OBS-2:** x[0.25, 0.75]  y[-1.50, -1.25]
- **Flight altitude:** z = 0.5 m (constant)

---

## STL specification

```text
φ = φ_reach ∧ φ_safe

φ_reach = ♢[0,T] (position in goal region)
φ_safe  = □[0,T] (outside OBS-1  ∧  outside OBS-2)
```

Evaluated over a Gaussian belief trajectory — returns (lower, upper) probability bounds.

---

## Belief model

2D Gaussian `N(μ, Σ)` where:

- `μ` = estimated (x, y)
- `Σ` = 2×2 position covariance
- Updated each step via a simple Kalman filter (constant-velocity model)
- `Q` = process noise (velocity uncertainty)
- `R` = measurement noise (Lighthouse: ~0.01–0.02 m std)

---

## Controller (Phase 1 — reactive, no optimisation)

Each step:

1. Predict belief forward by one step (process model)
2. Receive position measurement → Kalman update
3. Evaluate STL predicates at current belief
4. Compute velocity:
   - **Nominal:** proportional attraction toward next spline waypoint
   - **Safety:** if P(inside obstacle) > δ, add repulsion from obstacle centre
   - Clamp to `v_max`
5. Advance waypoint pointer when within `wp_tol` of target
6. At goal: stop, hover `hover_time` seconds, land

---

## Phase 2 (planned)

Replace reactive repulsion with gradient-based control:

- Parameterise velocity sequence as differentiable variables
- Loss = −STL_robustness_lower_bound
- Optimise with Adam over a receding horizon
