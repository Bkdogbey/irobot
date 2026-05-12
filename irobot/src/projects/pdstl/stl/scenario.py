"""
Mission geometry for the Crazyflie STL experiment.

Mission geometry for the pDSTL Crazyflie experiment.

Arena: x ∈ [-1, 1],  y ∈ [-2, 1]

Obstacles (x_min, x_max, y_min, y_max):
  OBS-1: x[0.00, 0.50]  y[ 0.25,  0.75]   upper centre-right
  OBS-2: x[0.25, 0.75]  y[-1.50, -1.25]   lower right

Path (Catmull-Rom CR knots):
  Start  → above/right of OBS-1 → through (0,0) → left of OBS-2 → Goal
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicHermiteSpline

# ── Mission constants ─────────────────────────────────────────────────────────
START = np.array([-1.0,  1.0])
GOAL  = np.array([ 1.0, -2.0])
Z_FLIGHT = 0.5          # constant altitude (metres)

GOAL_REGION   = (0.80,  1.00, -2.00, -1.80)   # x0,x1,y0,y1  (tight box around goal)

# Obstacles: (x_min, x_max, y_min, y_max)
OBSTACLES = [
    (0.00, 0.50,  0.25,  0.75),   # OBS-1
    (0.25, 0.75, -1.50, -1.25),   # OBS-2
]

# ── Catmull-Rom knots ─────────────────────────────────────────────────────────
_CR_KNOTS = np.array(
    [
        [-1.00,  1.000],   # Start
        [ 0.60,  0.900],   # Right of OBS-1 (x > 0.5), above OBS-1 (y > 0.75)
        [ 0.60,  0.100],   # Right of OBS-1 (x > 0.5), below OBS-1 (y < 0.25)
        [ 0.00,  0.000],   # Through-point
        [-0.60, -1.375],   # Left bow: clears OBS-2 (x < 0.25)
        [ 1.00, -2.000],   # Goal
    ]
)


def get_nominal_waypoints(n_points: int = 40) -> np.ndarray:
    """
    Return the Catmull-Rom spline as (n_points, 2) xy waypoints.

    These are the nominal positions the drone should follow.
    The reactive controller may deviate and then return to the nearest
    remaining waypoint.
    """
    diffs = np.diff(_CR_KNOTS, axis=0)
    chord = np.concatenate([[0.0], np.cumsum(np.hypot(diffs[:, 0], diffs[:, 1]))])

    n = len(_CR_KNOTS)
    tangents = np.zeros_like(_CR_KNOTS)
    for i in range(1, n - 1):
        tangents[i] = (_CR_KNOTS[i + 1] - _CR_KNOTS[i - 1]) / (chord[i + 1] - chord[i - 1])
    tangents[0]  = (_CR_KNOTS[1]  - _CR_KNOTS[0])  / (chord[1]  - chord[0])
    tangents[-1] = (_CR_KNOTS[-1] - _CR_KNOTS[-2]) / (chord[-1] - chord[-2])

    cs_x = CubicHermiteSpline(chord, _CR_KNOTS[:, 0], tangents[:, 0])
    cs_y = CubicHermiteSpline(chord, _CR_KNOTS[:, 1], tangents[:, 1])

    s = np.linspace(0.0, chord[-1], n_points)
    return np.column_stack([cs_x(s), cs_y(s)])
