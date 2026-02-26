"""
S-curve path from (-1, 1) to (1, -2) around three rectangular obstacles.

Top-view layout (constant altitude z throughout):

       x
  -1   0  0.35 0.65 0.75   1
   |   |   |    |    |     |
1  S───┬───┬────┬────┬─────┤   S = start (-1,  1)
   |   │#1#│    │    │#1###│   G = goal  ( 1, -2)
0  |   │#1#│    │    │#1###│   obs1 = x:[0.00,0.75]  y:[-0.75, 1.00]
   |   │#1#│#2# │    │#1###│   obs2 = x:[0.00,0.35]  y:[-0.35, 0.00]
-.35───┤   └────┘    │#1###│   obs3 = x:[0.35,0.65]  y:[-1.50,-1.25]
   |   │             │#1###│
-.75───┴─────────────┴─────┤
   |
-1.25 ──────────[obs3]─────┤
-1.5  ──────────[obs3]─────┤
   |
-2 |                       G

Route: stay x < 0 until below obs1+obs2, sweep right below obs3 to goal.
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline

# ── Obstacles (x_min, x_max, y_min, y_max) ──────────────────────────────────
OBSTACLES = [
    (0.00, 0.75, -0.75, 1.00),   # obs1: main column
    (0.00, 0.35, -0.35, 0.00),   # obs2: lower-left block
    (0.35, 0.65, -1.50, -1.25),  # obs3: block near goal
]

# ── Start / goal ─────────────────────────────────────────────────────────────
START = (-1.00,  1.00)
GOAL  = ( 1.00, -2.00)

# ── Guide knots ──────────────────────────────────────────────────────────────
# knot 0  (-1.00,  1.00)  start
# knot 1  (-0.40, -1.10)  left of all obstacles; below obs1 bottom (y<-1.0)
# knot 2  ( 0.20, -1.80)  just left of obs3 (x<0.35); below obs3 (y<-1.75)
# knot 3  ( 1.00, -2.00)  goal — right and below obs3
#
_KNOTS = np.array([
    [-1.00,  1.00],
    [-0.40, -1.10],
    [ 0.20, -1.80],
    [ 1.00, -2.00],
])


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
