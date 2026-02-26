"""
S-curve paths from (-1, 1) to (1, -2) around two rectangular obstacles.

Top-view layout (constant altitude z throughout):
  Start: (-1, 1)
  Goal:  ( 1, -2)

  Obstacles:
    1. x[ 0.00, 0.50] y[0.25, 0.75]   (upper, centre-right)
    2. x[ 0.25, 0.75] y[-1.50,-1.25]  (lower-right)
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from scipy.interpolate import CubicHermiteSpline, CubicSpline

# ── Obstacles (x_min, x_max, y_min, y_max) ──────────────────────────────────
OBSTACLES = [
    (0.00, 0.50, 0.25, 0.75),  # OBS-1: upper centre-right
    (0.25, 0.75, -1.50, -1.25),  # OBS-2: lower-right
]

# ── Start / goal ─────────────────────────────────────────────────────────────
START = (-1.00, 1.00)
GOAL = (1.00, -2.00)

# ── Guide knots (original build_path) ────────────────────────────────────────
_KNOTS = np.array(
    [
        [-1.00, 1.00],
        [-0.80, -0.50],  # Drop down
        [-0.20, -1.60],  # Low and left (below both obstacles)
        [1.00, -2.00],
    ]
)


# ── S-curve cubic spline knots ────────────────────────────────────────────────
#
# Upper arc bows RIGHT around OBS-1 (x[-0.75,-0.25], y[0.25,0.75]):
#   knot 1: above OBS-1 top (y=0.90 > 0.75, x=0.00 > -0.25) — sweeps right early
#   knot 2: right & below OBS-1 (x=0.70, y=0.10 < 0.25) — peak of rightward bow
#   knot 3: back to midpoint (0, 0) — creates visible return bend
#
# Lower arc bows LEFT around OBS-2 (x[0.25,0.75], y[-1.50,-1.25]):
#   knot 4: far left at OBS-2 mid-y (x=-0.60 << 0.25) — peak of leftward bow
#              by the time x re-enters [0.25,0.75] en route to goal, y < -1.50 ✓
_S_KNOTS = np.array(
    [
        [-1.00, 1.000],  # Start
        [0.00, 0.900],  # Above OBS-1 (y > 0.75), already right of x=-0.25
        [0.70, 0.100],  # Right bow peak: x >> -0.25, y < 0.25 → fully clears OBS-1
        [0.00, 0.000],  # Required through-point (return swing to centre)
        [-0.60, -1.375],  # Left bow peak: x=-0.60 < 0.25 → fully clears OBS-2
        [1.00, -2.000],  # Goal
    ]
)


# ── Catmull-Rom knots (corrected OBS-1: x[0.00,0.50], y[0.25,0.75]) ─────────
#
# Upper arc bows RIGHT, hugging OUTSIDE of OBS-1:
#   knot 1: (0.60, 0.90) — right of OBS-1 right edge (x=0.6 > 0.5),
#                           above OBS-1 top (y=0.9 > 0.75)
#   knot 2: (0.60, 0.10) — same x=0.6 (still right of OBS-1),
#                           below OBS-1 bottom (y=0.1 < 0.25)
#   knot 3: (0.00, 0.00) — through-point; chord from knot 2 here stays
#                           at y < 0.25 while x crosses [0,0.5] ✓
#
# Lower arc bows LEFT around OBS-2 (x[0.25,0.75], y[-1.50,-1.25]):
#   knot 4: (-0.60, -1.375) — x=-0.60 < 0.25 throughout OBS-2 y range;
#                              on the swing to goal x re-enters [0.25,0.75]
#                              only after y < -1.50 ✓
_CR_KNOTS = np.array(
    [
        [-1.00, 1.000],  # Start
        [0.60, 0.900],  # Right of OBS-1 (x > 0.5) and above it (y > 0.75)
        [0.60, 0.100],  # Right of OBS-1 (x > 0.5) and below it (y < 0.25)
        [0.00, 0.000],  # Required through-point
        [-0.60, -1.375],  # Left bow peak: clears OBS-2
        [1.00, -2.000],  # Goal
    ]
)


def build_s_path(z: float, n_points: int = 20) -> list[tuple[float, float, float]]:
    """
    S-curve from (-1, 1) to (1, -2) passing through (0, 0).

    Upper arc bows east, clearing OBS-1 (x[-0.75,-0.25], y[0.25,0.75]).
    Lower arc bows west, clearing OBS-2 (x[0.25,0.75], y[-1.50,-1.25]).

    Chord-length parameterisation for even sample spacing.

    Args:
        z:        Constant flight altitude in metres.
        n_points: Number of (x, y, z) waypoints to return.

    Returns:
        List of (x, y, z) tuples — feed directly to FlightCommander.go_to().

    """
    diffs = np.diff(_S_KNOTS, axis=0)
    chord = np.concatenate([[0.0], np.cumsum(np.hypot(diffs[:, 0], diffs[:, 1]))])

    cs_x = CubicSpline(chord, _S_KNOTS[:, 0])
    cs_y = CubicSpline(chord, _S_KNOTS[:, 1])

    s_samples = np.linspace(0.0, chord[-1], n_points)
    return [(float(cs_x(s)), float(cs_y(s)), z) for s in s_samples]


def build_cr_path(z: float, n_points: int = 20) -> list[tuple[float, float, float]]:
    """
    Catmull-Rom S-curve from (-1, 1) to (1, -2) passing through (0, 0).

    Upper arc bows east, skirting OBS-1 right edge (x[0.00,0.50], y[0.25,0.75])
    by staying at x=0.60 for the entire OBS-1 y-range.
    Lower arc bows west, clearing OBS-2 (x[0.25,0.75], y[-1.50,-1.25]).

    Tangents are derived from the standard centripetal Catmull-Rom formula
    (chord-length parameterisation) and fed into CubicHermiteSpline.

    Args:
        z:        Constant flight altitude in metres.
        n_points: Number of (x, y, z) waypoints to return.

    Returns:
        List of (x, y, z) tuples — feed directly to FlightCommander.go_to().

    """
    diffs = np.diff(_CR_KNOTS, axis=0)
    chord = np.concatenate([[0.0], np.cumsum(np.hypot(diffs[:, 0], diffs[:, 1]))])

    n = len(_CR_KNOTS)
    tangents = np.zeros_like(_CR_KNOTS)

    # Interior: centripetal Catmull-Rom tangent
    for i in range(1, n - 1):
        tangents[i] = (_CR_KNOTS[i + 1] - _CR_KNOTS[i - 1]) / (chord[i + 1] - chord[i - 1])

    # Endpoints: one-sided finite difference
    tangents[0] = (_CR_KNOTS[1] - _CR_KNOTS[0]) / (chord[1] - chord[0])
    tangents[-1] = (_CR_KNOTS[-1] - _CR_KNOTS[-2]) / (chord[-1] - chord[-2])

    cs_x = CubicHermiteSpline(chord, _CR_KNOTS[:, 0], tangents[:, 0])
    cs_y = CubicHermiteSpline(chord, _CR_KNOTS[:, 1], tangents[:, 1])

    s_samples = np.linspace(0.0, chord[-1], n_points)
    return [(float(cs_x(s)), float(cs_y(s)), z) for s in s_samples]


def _plot_s_path() -> None:
    traj = build_s_path(z=0.0, n_points=200)
    xs = [p[0] for p in traj]
    ys = [p[1] for p in traj]

    _, ax = plt.subplots(figsize=(5, 8))
    ax.plot([-1, 1, 1, -1, -1], [1, 1, -2, -2, 1], 'k--', lw=1, label='Workspace')

    for x0, x1, y0, y1 in OBSTACLES:
        ax.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0, color='red', alpha=0.4))

    kx, ky = _S_KNOTS[:, 0], _S_KNOTS[:, 1]
    ax.scatter(kx, ky, color='black', zorder=5, label='Knots')
    ax.plot(xs, ys, 'b-', lw=2, label='S-path')
    ax.plot(xs[0], ys[0], 'go', ms=8, label='Start')
    ax.plot(xs[-1], ys[-1], 'r*', ms=10, label='Goal')

    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-2.1, 1.1)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('S-curve spline')
    plt.tight_layout()
    plt.show()


def _plot_cr_path() -> None:
    traj = build_cr_path(z=0.0, n_points=200)
    xs = [p[0] for p in traj]
    ys = [p[1] for p in traj]

    _, ax = plt.subplots(figsize=(5, 8))
    ax.plot([-1, 1, 1, -1, -1], [1, 1, -2, -2, 1], 'k--', lw=1, label='Workspace')

    for x0, x1, y0, y1 in OBSTACLES:
        ax.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0, color='red', alpha=0.4))

    kx, ky = _CR_KNOTS[:, 0], _CR_KNOTS[:, 1]
    ax.scatter(kx, ky, color='black', zorder=5, label='Knots')
    ax.plot(xs, ys, 'b-', lw=2, label='CR-path')
    ax.plot(xs[0], ys[0], 'go', ms=8, label='Start')
    ax.plot(xs[-1], ys[-1], 'r*', ms=10, label='Goal')

    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-2.1, 1.1)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Catmull-Rom S-curve')
    plt.tight_layout()
    plt.show()


def build_path(z: float, n_points: int = 20) -> list[tuple[float, float, float]]:
    """
    Fit a cubic spline through the guide knots and return sampled positions.

    Chord-length parameterisation ensures samples are evenly spread along the
    actual curve rather than bunched near sharp turns.

    Args:
        z:        Constant flight altitude in metres.
        n_points: Number of (x, y, z) waypoints to return.

    Returns:
        List of (x, y, z) tuples — feed directly to FlightCommander.go_to().

    """
    diffs = np.diff(_KNOTS, axis=0)
    chord = np.concatenate([[0.0], np.cumsum(np.hypot(diffs[:, 0], diffs[:, 1]))])

    cs_x = CubicSpline(chord, _KNOTS[:, 0])
    cs_y = CubicSpline(chord, _KNOTS[:, 1])

    s_samples = np.linspace(0.0, chord[-1], n_points)
    return [(float(cs_x(s)), float(cs_y(s)), z) for s in s_samples]


if __name__ == '__main__':
    _plot_s_path()
    _plot_cr_path()
