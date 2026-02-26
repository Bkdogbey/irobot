"""
2D Gaussian belief state for the Crazyflie position.

Models position uncertainty as N(mean, cov) and provides:
  - Kalman predict step  (process noise Q)
  - Kalman update step   (measurement noise R)
  - Marginal probability that position falls inside a rectangle
    (used by the STL predicates in stl.py)

No imports from probabilistic_stl.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.stats import norm


@dataclass
class GaussianBelief2D:
    """
    2D Gaussian belief: position ~ N(mean, cov).

    Attributes:
        mean: (2,) array — estimated [x, y]
        cov:  (2,2) array — position covariance
    """

    mean: np.ndarray
    cov: np.ndarray = field(default_factory=lambda: np.eye(2) * 0.01)

    # ── Kalman steps ──────────────────────────────────────────────────────────

    def predict(self, velocity: np.ndarray, dt: float, q_std: float = 0.02) -> GaussianBelief2D:
        """
        Constant-velocity prediction step.

        Args:
            velocity: (2,) commanded [vx, vy] in m/s
            dt:       time step in seconds
            q_std:    std dev of process noise (models tracking error)

        Returns:
            New GaussianBelief2D after propagation.
        """
        mean_new = self.mean + velocity * dt
        Q = np.eye(2) * (q_std * dt) ** 2
        cov_new = self.cov + Q
        return GaussianBelief2D(mean=mean_new, cov=cov_new)

    def update(self, measurement: np.ndarray, r_std: float = 0.01) -> GaussianBelief2D:
        """
        Kalman measurement update (identity observation model).

        Args:
            measurement: (2,) observed [x, y]
            r_std:        std dev of measurement noise (Lighthouse: ~0.01 m)

        Returns:
            New GaussianBelief2D after incorporating measurement.
        """
        R = np.eye(2) * r_std**2
        S = self.cov + R  # innovation covariance
        K = self.cov @ np.linalg.inv(S)  # Kalman gain
        innovation = measurement - self.mean
        mean_new = self.mean + K @ innovation
        cov_new = (np.eye(2) - K) @ self.cov
        return GaussianBelief2D(mean=mean_new, cov=cov_new)

    # ── Probability helpers ───────────────────────────────────────────────────

    def std(self) -> np.ndarray:
        """(2,) marginal standard deviations [σ_x, σ_y]."""
        return np.sqrt(np.diag(self.cov))

    def prob_inside_rect(
        self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
    ) -> tuple[float, float]:
        """
        (lower, upper) probability that position is inside [x_min,x_max] × [y_min,y_max].

        Uses Gaussian marginal CDFs (x and y treated as independent —
        conservative when correlated). Fréchet bounds give the joint interval:
            lower = max(P_x + P_y − 1, 0)
            upper = min(P_x, P_y)
        """
        sx, sy = self.std()
        px = norm.cdf(x_max, self.mean[0], sx) - norm.cdf(x_min, self.mean[0], sx)
        py = norm.cdf(y_max, self.mean[1], sy) - norm.cdf(y_min, self.mean[1], sy)
        lower = float(max(px + py - 1.0, 0.0))
        upper = float(min(px, py))
        return lower, upper

    def prob_outside_rect(
        self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
    ) -> tuple[float, float]:
        """
        (lower, upper) probability that position is outside the rectangle.

        Complement of prob_inside_rect: [1 − upper_in, 1 − lower_in].
        """
        lower_in, upper_in = self.prob_inside_rect(x_min, x_max, y_min, y_max)
        return 1.0 - upper_in, 1.0 - lower_in
