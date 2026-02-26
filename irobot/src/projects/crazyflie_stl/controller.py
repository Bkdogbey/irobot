"""
Reactive STL controller for the Crazyflie S-curve mission.

Phase 1 — no optimisation.  At each time step:
  1. Compute STL safety probability at current belief.
  2. If any obstacle constraint is at risk (P_outside < safety_thresh),
     add repulsion velocity from that obstacle.
  3. Otherwise follow the nominal spline waypoint (attraction).
  4. Clamp to v_max and return the velocity command.

The returned velocity is in the xy-plane; z is held constant externally.
"""

from __future__ import annotations

import numpy as np

from belief import GaussianBelief2D
from stl import OutsideObstacle


class STLController:
    """
    Reactive STL controller.

    Args:
        waypoints:      (N, 2) nominal xy path from scenario.get_nominal_waypoints()
        obstacles:      list of (x_min, x_max, y_min, y_max) — same as scenario.OBSTACLES
        goal_region:    (x_min, x_max, y_min, y_max)
        v_max:          maximum speed in m/s
        wp_tol:         distance (m) at which a waypoint is considered reached
        safety_thresh:  P(outside_obs) below this triggers repulsion
    """

    def __init__(
        self,
        waypoints:      np.ndarray,
        obstacles:      list[tuple],
        goal_region:    tuple,
        v_max:          float = 0.20,
        wp_tol:         float = 0.08,
        safety_thresh:  float = 0.90,
    ):
        self.waypoints     = waypoints
        self.goal_region   = goal_region
        self.v_max         = v_max
        self.wp_tol        = wp_tol
        self.safety_thresh = safety_thresh
        self.wp_idx        = 0

        self._avoid_preds = [
            OutsideObstacle(*obs, name=f'obs{i+1}')
            for i, obs in enumerate(obstacles)
        ]

    # ── Public API ────────────────────────────────────────────────────────────

    @property
    def at_goal(self) -> bool:
        """True once all waypoints have been passed."""
        return self.wp_idx >= len(self.waypoints)

    def step(self, belief: GaussianBelief2D) -> np.ndarray:
        """
        Compute xy velocity command given the current belief.

        Returns:
            (2,) velocity vector in m/s, magnitude ≤ v_max.
        """
        if self.at_goal:
            return np.zeros(2)

        self._advance_waypoint(belief)

        v_nominal  = self._attraction(belief)
        v_repulse  = self._repulsion(belief)
        v_cmd      = v_nominal + v_repulse

        speed = np.linalg.norm(v_cmd)
        if speed > self.v_max:
            v_cmd = v_cmd / speed * self.v_max

        return v_cmd

    def stl_safety(self, belief: GaussianBelief2D) -> list[tuple[str, float, float]]:
        """
        Evaluate all obstacle-avoidance predicates at the current belief.

        Returns list of (name, lower_prob, upper_prob) for logging / plotting.
        """
        return [
            (pred.name, *pred(belief))
            for pred in self._avoid_preds
        ]

    # ── Private helpers ───────────────────────────────────────────────────────

    def _advance_waypoint(self, belief: GaussianBelief2D) -> None:
        """Move the waypoint pointer forward when the drone is close enough."""
        while self.wp_idx < len(self.waypoints):
            target = self.waypoints[self.wp_idx]
            if np.linalg.norm(belief.mean - target) < self.wp_tol:
                self.wp_idx += 1
            else:
                break

    def _attraction(self, belief: GaussianBelief2D) -> np.ndarray:
        """Proportional attraction toward the current target waypoint."""
        if self.at_goal:
            return np.zeros(2)
        target   = self.waypoints[self.wp_idx]
        to_wp    = target - belief.mean
        dist     = np.linalg.norm(to_wp)
        speed    = min(dist, self.v_max)
        return (to_wp / (dist + 1e-8)) * speed

    def _repulsion(self, belief: GaussianBelief2D) -> np.ndarray:
        """
        Sum of repulsion velocities from obstacles whose safety probability
        has dropped below safety_thresh.

        Direction: away from obstacle centre.
        Magnitude: v_max, scaled by how far below the threshold we are.
        """
        v_rep = np.zeros(2)
        for pred in self._avoid_preds:
            lower, _ = pred(belief)
            if lower < self.safety_thresh:
                x0, x1, y0, y1 = pred.bounds
                obs_centre = np.array([(x0 + x1) / 2.0, (y0 + y1) / 2.0])
                away       = belief.mean - obs_centre
                away_norm  = away / (np.linalg.norm(away) + 1e-8)
                deficit    = self.safety_thresh - lower   # 0 → 1
                v_rep     += away_norm * self.v_max * deficit
        return v_rep
