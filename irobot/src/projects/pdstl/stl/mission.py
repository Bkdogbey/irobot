"""
Top-level STL mission runner (simulation / dry-run).

Simulates the Crazyflie running the reactive STL controller over the
Catmull-Rom nominal path, with Gaussian process + measurement noise.

Run:
    python -m irobot.src.projects.pdstl.stl.mission

Outputs:
  - Trajectory plot with obstacles, nominal path, actual path, belief ellipses
  - STL safety probability over time
"""

from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Ellipse

from .scenario import (
    START, GOAL, GOAL_REGION, OBSTACLES, Z_FLIGHT,
    get_nominal_waypoints,
)
from .belief import GaussianBelief2D
from .controller import STLController
from .stl import OutsideObstacle, InsideGoal

# ── Simulation parameters ─────────────────────────────────────────────────────
DT          = 0.1     # time step (s)
MAX_STEPS   = 600     # safety cutoff
HOVER_STEPS = 50      # steps to hover at goal before "landing"

Q_STD       = 0.02    # process noise std dev (m)
R_STD       = 0.01    # measurement noise std dev (m) — Lighthouse quality

V_MAX       = 0.25    # max speed (m/s)
WP_TOL      = 0.08    # waypoint advance tolerance (m)
SAFETY_THRESH = 0.90  # STL safety threshold


# ── Helpers ───────────────────────────────────────────────────────────────────

def _draw_cov_ellipse(ax, belief: GaussianBelief2D, n_std: float = 2.0, **kwargs):
    """Draw a 2-sigma covariance ellipse for the belief."""
    vals, vecs = np.linalg.eigh(belief.cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    angle = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)
    ell = Ellipse(
        xy=belief.mean, width=width, height=height, angle=angle,
        **kwargs,
    )
    ax.add_patch(ell)


def _in_goal(pos: np.ndarray) -> bool:
    x0, x1, y0, y1 = GOAL_REGION
    return x0 <= pos[0] <= x1 and y0 <= pos[1] <= y1


# ── Main simulation ───────────────────────────────────────────────────────────

def run_mission(seed: int = 0) -> dict:
    """
    Simulate the mission and return a results dict.

    Returns:
        {
          'positions':   (N, 2) actual positions,
          'means':       (N, 2) belief means,
          'safety_obs1': (N,)  lower P(outside OBS-1),
          'safety_obs2': (N,)  lower P(outside OBS-2),
          'reached_goal': bool,
          'steps':        int,
        }
    """
    rng = np.random.default_rng(seed)

    waypoints = get_nominal_waypoints(n_points=60)
    ctrl = STLController(
        waypoints=waypoints,
        obstacles=OBSTACLES,
        goal_region=GOAL_REGION,
        v_max=V_MAX,
        wp_tol=WP_TOL,
        safety_thresh=SAFETY_THRESH,
    )

    obs1_pred = OutsideObstacle(*OBSTACLES[0], name='obs1')
    obs2_pred = OutsideObstacle(*OBSTACLES[1], name='obs2')
    goal_pred = InsideGoal(*GOAL_REGION)

    # Initialise belief at start
    belief = GaussianBelief2D(
        mean=START.copy(),
        cov=np.eye(2) * (R_STD * 2) ** 2,   # slightly uncertain at start
    )

    positions   = []
    means       = []
    safety_obs1 = []
    safety_obs2 = []
    true_pos    = START.copy()    # ground truth (for simulation)
    reached     = False
    hover_count = 0

    for step in range(MAX_STEPS):
        # ── Record ────────────────────────────────────────────────────────────
        positions.append(true_pos.copy())
        means.append(belief.mean.copy())
        safety_obs1.append(obs1_pred(belief)[0])
        safety_obs2.append(obs2_pred(belief)[0])

        # ── Check goal ────────────────────────────────────────────────────────
        if _in_goal(true_pos):
            if not reached:
                reached = True
                print(f'[step {step}] Goal reached. Hovering for {HOVER_STEPS} steps.')
            hover_count += 1
            if hover_count >= HOVER_STEPS:
                print(f'[step {step}] Landing.')
                break
            # hover: zero velocity, continue belief update
            vel = np.zeros(2)
        else:
            # ── Controller ────────────────────────────────────────────────────
            vel = ctrl.step(belief)

        # ── Simulate true motion (with process noise) ─────────────────────────
        true_pos = true_pos + vel * DT + rng.normal(0, Q_STD * DT, size=2)

        # ── Simulate measurement ──────────────────────────────────────────────
        measurement = true_pos + rng.normal(0, R_STD, size=2)

        # ── Belief update ─────────────────────────────────────────────────────
        belief = belief.predict(vel, DT, q_std=Q_STD)
        belief = belief.update(measurement, r_std=R_STD)

    return {
        'positions':    np.array(positions),
        'means':        np.array(means),
        'safety_obs1':  np.array(safety_obs1),
        'safety_obs2':  np.array(safety_obs2),
        'reached_goal': reached,
        'steps':        step + 1,
        'waypoints':    waypoints,
    }


# ── Plotting ──────────────────────────────────────────────────────────────────

def plot_mission(results: dict) -> None:
    pos      = results['positions']
    means    = results['means']
    wps      = results['waypoints']
    s_obs1   = results['safety_obs1']
    s_obs2   = results['safety_obs2']
    n_steps  = results['steps']

    fig, (ax_map, ax_stl) = plt.subplots(1, 2, figsize=(12, 8))

    # ── Map ───────────────────────────────────────────────────────────────────
    ax_map.set_title('Mission trajectory')

    # Workspace
    ax_map.plot([-1,1,1,-1,-1],[1,1,-2,-2,1], 'k--', lw=1, label='Workspace')

    # Obstacles
    for i, (x0, x1, y0, y1) in enumerate(OBSTACLES):
        ax_map.add_patch(
            mpatches.Rectangle((x0, y0), x1-x0, y1-y0,
                                color='red', alpha=0.35, label=f'OBS-{i+1}')
        )

    # Goal region
    gx0, gx1, gy0, gy1 = GOAL_REGION
    ax_map.add_patch(
        mpatches.Rectangle((gx0, gy0), gx1-gx0, gy1-gy0,
                            color='green', alpha=0.3, label='Goal')
    )

    # Nominal path
    ax_map.plot(wps[:, 0], wps[:, 1], 'b--', lw=1, alpha=0.5, label='Nominal path')

    # Actual trajectory
    ax_map.plot(pos[:, 0], pos[:, 1], 'b-', lw=2, label='Actual trajectory')

    # Belief means
    ax_map.plot(means[:, 0], means[:, 1], 'c:', lw=1, alpha=0.7, label='Belief mean')

    # Start / goal markers
    ax_map.plot(*START, 'go', ms=10, label='Start')
    ax_map.plot(*GOAL,  'r*', ms=12, label='Goal')

    ax_map.set_xlim(-1.1, 1.1)
    ax_map.set_ylim(-2.1, 1.1)
    ax_map.set_aspect('equal')
    ax_map.grid(True, alpha=0.3)
    ax_map.legend(loc='upper right', fontsize=7)

    # ── STL probability over time ─────────────────────────────────────────────
    ax_stl.set_title('STL safety probability (lower bound)')
    t = np.arange(n_steps) * DT
    ax_stl.plot(t, s_obs1, label='P(outside OBS-1)', color='orange')
    ax_stl.plot(t, s_obs2, label='P(outside OBS-2)', color='purple')
    ax_stl.axhline(SAFETY_THRESH, color='red', ls='--', lw=1, label='Safety threshold')
    ax_stl.set_xlabel('Time (s)')
    ax_stl.set_ylabel('Probability')
    ax_stl.set_ylim(-0.05, 1.05)
    ax_stl.grid(True, alpha=0.3)
    ax_stl.legend(fontsize=8)

    title = 'REACHED GOAL' if results['reached_goal'] else 'DID NOT REACH GOAL'
    fig.suptitle(f'Crazyflie STL Mission — {title}  ({n_steps} steps)')
    plt.tight_layout()
    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    results = run_mission(seed=42)
    print(f"Reached goal: {results['reached_goal']}  |  Steps: {results['steps']}")
    plot_mission(results)
