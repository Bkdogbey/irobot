"""
Probabilistic-STL path optimiser for the Crazyflie sinusoidal path.

Workflow
--------
1. Compute the original 10-waypoint sine path from crazyflie.py.
2. Propagate a Gaussian covariance along it, including fan noise.
3. Evaluate pDSTL robustness: rho = P(always outside all obstacles).
4. Optimise the 8 interior waypoint positions to maximise rho.
5. Plot before/after with 2-sigma covariance ellipses.
6. Print the optimised waypoints, ready to paste into crazyflie.py.

Run
---
    python irobot/src/projects/optimise_cf_path.py

Adjust the constants under "TUNABLE CONSTANTS" to match your physical setup.
"""

from __future__ import annotations

import json
import pathlib
import sys
from datetime import datetime, timezone

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Rectangle
from scipy.optimize import minimize

# ── Import crazyflie_stl (self-contained numpy module) ────────────────────────
_CRAZYFLIE_STL = pathlib.Path(__file__).parent / 'crazyflie_stl'
sys.path.insert(0, str(_CRAZYFLIE_STL))
from belief import GaussianBelief2D  # noqa: E402
from stl import OutsideObstacle, stl_and  # noqa: E402

# =============================================================================
# TUNABLE CONSTANTS  <--  adjust these to match your physical setup
# =============================================================================

# ── Coordinate frame: metres, same as crazyflie.py ───────────────────────────
START = np.array([0.0, -1.5])  # matches go_to(0, -start_0) in crazyflie.py
GOAL = np.array([0.489, 0.65])  # last sine waypoint: x = 0.5*sin(pi*0.65/1.5)
N_WP = 10  # total waypoints including start and goal
Z = 0.2  # flight altitude (m) — not optimised here

# ── Obstacles (x_min, x_max, y_min, y_max) in metres ─────────────────────────
#    ADJUST THESE to your physical lab obstacles
OBSTACLES = [
    (0.15, 0.55, -0.90, -0.50),  # OBS-1 placeholder
    (-0.45, 0.05, 0.00, 0.40),  # OBS-2 placeholder
]

# ── Noise model ───────────────────────────────────────────────────────────────
#    sigma_step = std dev of position error added each DT seconds.
#    The predict() call uses  Q = (q_std * dt)^2 * I,  so q_std = sigma_step / dt.
SIGMA0 = np.eye(2) * (0.010**2)  # initial covariance (0.5 cm sigma)
SIGMA_DRONE_STEP = 0.010  # m  drone process noise std dev per step
SIGMA_FAN_STEP = 0.020  # m  fan disturbance std dev per step  <-- ADJUST
SIGMA_TOTAL_STEP = float(np.sqrt(SIGMA_DRONE_STEP**2 + SIGMA_FAN_STEP**2))

DT = 0.1  # seconds between consecutive waypoints

# ── pDSTL satisfaction threshold ──────────────────────────────────────────────
ALPHA = 0.90  # require P(always safe) >= ALPHA after optimisation

# ── Optimiser loss weights ────────────────────────────────────────────────────
W_PHI = 20.0  # pDSTL satisfaction  (primary objective)
W_SMOOTH = 1.0  # second-difference smoothness penalty
W_DEV = 0.5  # deviation from original sine path

# =============================================================================


def original_sine_waypoints() -> np.ndarray:
    """Replicate the sine path defined in crazyflie.py.  Shape: (N_WP, 2)."""
    start_0 = 1.5
    y_pos = np.linspace(-start_0, 0.65, N_WP)
    x_pos = 0.5 * np.sin(np.pi * y_pos / start_0)
    return np.stack([x_pos, y_pos], axis=1)


def propagate_covariance(waypoints: np.ndarray) -> list[GaussianBelief2D]:
    """
    Open-loop Gaussian covariance propagation along a fixed mean path.

    No measurement updates are applied — this gives the worst-case (maximum)
    uncertainty growth, making the safety constraint conservative.

    Args:
        waypoints: (N_WP, 2) array of mean positions in metres.

    Returns:
        List of N_WP GaussianBelief2D objects, one per waypoint.
    """
    q_std = SIGMA_TOTAL_STEP / DT  # convert per-step sigma to predict()'s q_std
    beliefs: list[GaussianBelief2D] = [GaussianBelief2D(mean=waypoints[0].copy(), cov=SIGMA0.copy())]
    for i in range(1, len(waypoints)):
        vel = (waypoints[i] - waypoints[i - 1]) / DT
        beliefs.append(beliefs[-1].predict(vel, DT, q_std=q_std))
    return beliefs


def eval_pdstl(beliefs: list[GaussianBelief2D]) -> float:
    """
    Lower bound of P(□[0,T] safe):  probability that the drone stays outside
    all obstacles at every waypoint along the trajectory.

    Implements:  min_t  AND_i P(outside obs_i | belief_t)
    using Frechet bounds throughout.

    Returns:
        Scalar in [0, 1].  Higher is safer.
    """
    predicates = [OutsideObstacle(*obs) for obs in OBSTACLES]
    per_step: list[float] = []
    for b in beliefs:
        p: tuple[float, float] = (1.0, 1.0)
        for pred in predicates:
            p = stl_and(p, pred(b))
        per_step.append(p[0])  # lower bound of joint safety
    return float(min(per_step))  # Always = min over time


# ── Optimiser internals ───────────────────────────────────────────────────────


def _assemble(x_flat: np.ndarray) -> np.ndarray:
    """Reconstruct (N_WP, 2) waypoints from flattened interior decision vars."""
    interior = x_flat.reshape(N_WP - 2, 2)
    return np.vstack([START, interior, GOAL])


def _loss(x_flat: np.ndarray, orig: np.ndarray) -> float:
    """Scalar loss for scipy.minimize."""
    wps = _assemble(x_flat)

    rho = eval_pdstl(propagate_covariance(wps))

    # Second-difference smoothness (penalises sharp bends)
    diffs = np.diff(wps, axis=0)
    second_diffs = np.diff(diffs, axis=0)
    smooth = float(np.sum(second_diffs**2))

    # Deviation from original sine path (interior points only)
    dev = float(np.sum((wps[1:-1] - orig[1:-1]) ** 2))

    return -W_PHI * rho + W_SMOOTH * smooth + W_DEV * dev


def optimise() -> tuple[np.ndarray, np.ndarray]:
    """
    Optimise the 8 interior waypoints to maximise pDSTL safety robustness.

    Warm-starts from the original sine path so the optimiser begins with the
    best currently known path.

    Returns:
        (original_waypoints, optimised_waypoints)  both shape (N_WP, 2).
    """
    orig = original_sine_waypoints()

    rho_before = eval_pdstl(propagate_covariance(orig))
    print(f'rho before: {rho_before:.4f}   (threshold alpha = {ALPHA})')

    x0 = orig[1:-1].flatten()  # warm-start: interior of original path
    result = minimize(
        _loss,
        x0=x0,
        args=(orig,),
        method='L-BFGS-B',
        options={'maxiter': 1000, 'ftol': 1e-10, 'gtol': 1e-7},
    )

    opt_wps = _assemble(result.x)
    rho_after = eval_pdstl(propagate_covariance(opt_wps))

    status = 'satisfied' if rho_after >= ALPHA else 'NOT satisfied — try increasing W_PHI or relaxing ALPHA'
    print(f'rho after:  {rho_after:.4f}   ({status})')
    print(f'Converged: {result.success}  |  iterations: {result.nit}  |  message: {result.message}')

    return orig, opt_wps


# ── Plotting ──────────────────────────────────────────────────────────────────


def _ellipse_patch(belief: GaussianBelief2D, n_std: float = 2.0, **kwargs) -> Ellipse:
    """Return a matplotlib Ellipse for the n_std-sigma confidence region."""
    vals, vecs = np.linalg.eigh(belief.cov)
    order = vals.argsort()[::-1]
    vals = vals[order]
    vecs = vecs[:, order]
    angle = float(np.degrees(np.arctan2(*vecs[:, 0][::-1])))
    width, height = 2.0 * n_std * np.sqrt(np.maximum(vals, 0.0))
    return Ellipse(xy=(belief.mean[0], belief.mean[1]), width=width, height=height, angle=angle, **kwargs)


def plot_result(orig: np.ndarray, opt: np.ndarray) -> None:
    """Side-by-side before / after plot with obstacles and 2-sigma ellipses."""
    beliefs_orig = propagate_covariance(orig)
    beliefs_opt = propagate_covariance(opt)
    rho_orig = eval_pdstl(beliefs_orig)
    rho_opt = eval_pdstl(beliefs_opt)

    fig, axes = plt.subplots(1, 2, figsize=(12, 8), sharey=True)

    configs = [
        ('Original sine path', orig, beliefs_orig, rho_orig, 'tab:blue'),
        ('Optimised path', opt, beliefs_opt, rho_opt, 'tab:green'),
    ]

    for ax, (label, wps, beliefs, rho, col) in zip(axes, configs):
        # Obstacles
        for x0, x1, y0, y1 in OBSTACLES:
            ax.add_patch(
                Rectangle(
                    (x0, y0),
                    x1 - x0,
                    y1 - y0,
                    facecolor='red',
                    alpha=0.35,
                    edgecolor='darkred',
                    lw=1.2,
                    zorder=2,
                )
            )

        # 2-sigma covariance ellipses
        for b in beliefs:
            ax.add_patch(
                _ellipse_patch(
                    b,
                    n_std=2.0,
                    facecolor=col,
                    alpha=0.15,
                    edgecolor=col,
                    linewidth=0.8,
                    zorder=3,
                )
            )

        # Waypoints and path
        ax.plot(wps[:, 0], wps[:, 1], 'o-', color=col, lw=2, ms=5, zorder=4)
        ax.plot(*START, 'g^', ms=10, zorder=5, label='start')
        ax.plot(*GOAL, 'r*', ms=12, zorder=5, label='goal')

        ax.set_title(f'{label}\nrho = {rho:.4f}  (alpha = {ALPHA})', fontsize=11)
        ax.set_xlabel('x  (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='lower right', fontsize=8)

    axes[0].set_ylabel('y  (m)')
    fig.suptitle(
        f'pDSTL path optimisation — fan noise sigma = {SIGMA_FAN_STEP * 100:.0f} cm/step',
        fontsize=12,
    )
    plt.tight_layout()
    plt.show()


# ── Waypoint save / log ───────────────────────────────────────────────────────

# Written next to crazyflie.py so it can be imported directly.
_COMPONENTS = pathlib.Path(__file__).parent / 'probabilistic_stl' / 'components'
WAYPOINTS_FILE = _COMPONENTS / 'opt_waypoints.py'
LOG_FILE       = _COMPONENTS / 'opt_waypoints.log'


def save_waypoints(waypoints: np.ndarray, rho_before: float, rho_after: float) -> None:
    """
    Write the optimised waypoints to WAYPOINTS_FILE as a plain Python module
    and append a one-line JSON summary to LOG_FILE for every run.
    """
    _COMPONENTS.mkdir(parents=True, exist_ok=True)

    ts = datetime.now(timezone.utc).isoformat(timespec='seconds')

    lines = [
        '# Auto-generated by optimise_cf_path.py — do not edit by hand.',
        f'# generated={ts}  rho_before={rho_before:.4f}  rho_after={rho_after:.4f}'
        f'  alpha={ALPHA}  sigma_fan={SIGMA_FAN_STEP}',
        '',
        'WAYPOINTS: list[tuple[float, float, float]] = [',
    ]
    for x, y in waypoints:
        lines.append(f'    ({x:.6f}, {y:.6f}, {Z}),')
    lines += [']', '']

    WAYPOINTS_FILE.write_text('\n'.join(lines))
    print(f'Waypoints saved  →  {WAYPOINTS_FILE}')

    with LOG_FILE.open('a') as fh:
        fh.write(json.dumps({
            'ts':         ts,
            'rho_before': round(rho_before, 6),
            'rho_after':  round(rho_after,  6),
            'converged':  rho_after >= ALPHA,
        }) + '\n')
    print(f'Run logged       →  {LOG_FILE}')


def print_waypoints(waypoints: np.ndarray) -> None:
    """Print a Python list ready to paste into crazyflie.py."""
    print()
    print('# ── Optimised waypoints ─────────────────────────────────────────')
    print('OPT_WAYPOINTS = [  # (x, y, z)')
    for x, y in waypoints:
        print(f'    ({x: .5f}, {y: .5f}, {Z}),')
    print(']')
    print('# ────────────────────────────────────────────────────────────────')
    print()


# =============================================================================
if __name__ == '__main__':
    orig, opt = optimise()
    rho_b = eval_pdstl(propagate_covariance(orig))
    rho_a = eval_pdstl(propagate_covariance(opt))
    save_waypoints(opt, rho_b, rho_a)
    print_waypoints(opt)
    plot_result(orig, opt)
