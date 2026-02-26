"""
STL specification for the current mission.

  phi = Eventually[0,T]( reach goal )
      AND
        Always[0,T]( avoid obstacle )

No optimisation yet — this file just defines the constraints.
The spline in spline_path.py handles execution for now.

Coordinate frame: lighthouse origin at Crazyflie power-on position.
  Start:    (-1.0,  1.0)
  Obstacle:  x = [0.00, 0.75],  y = [-0.75, 1.00]
  Goal:      x = [0.70, 1.00],  y = [-2.00, -1.70]
"""

from __future__ import annotations

import sys
from pathlib import Path

# planning / pdstl use bare imports — expose the package root
sys.path.insert(0, str(Path(__file__).parents[1]))  # → probabilistic_stl/

from constraints.reach_avoid import build_reach_avoid_spec
from planning.environment import Environment

# ── Mission geometry ──────────────────────────────────────────────────────────
OBSTACLE_X = [0.00, 0.75]
OBSTACLE_Y = [-0.75, 1.00]
GOAL_X = [0.70, 1.00]
GOAL_Y = [-2.00, -1.70]

T = 100  # time horizon (steps) — used when the planner is added later


def build_environment() -> Environment:
    """Create the Environment with the obstacle and goal for this mission."""
    env = Environment()
    env.set_goal(x_range=GOAL_X, y_range=GOAL_Y)
    env.add_obstacle(x_range=OBSTACLE_X, y_range=OBSTACLE_Y)
    return env


def build_spec():
    """
    Return the STL formula for this mission.

        phi = Eventually[0,T](reach goal)
              AND
              Always[0,T](avoid obstacle)
    """
    env = build_environment()
    return build_reach_avoid_spec(env, T)
