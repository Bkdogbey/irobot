"""
pdSTL Formula Primitives
========================
Signal Temporal Logic with smooth robustness for gradient-based control.

Robustness ρ(x, t) > 0  →  formula satisfied at time t
            ρ(x, t) < 0  →  formula violated

Smooth min/max use the log-sum-exp approximation so that ρ is
differentiable everywhere, enabling gradient-based control synthesis.

Operators
---------
    Predicate(mu)           atomic: ρ = mu(x, y, z)
    Always(phi, a, b)       G[a,b] phi   (smooth-min over window)
    Eventually(phi, a, b)   F[a,b] phi   (smooth-max over window)
    And(*phis)              phi₁ ∧ phi₂ ∧ …  (smooth-min)
    Or(*phis)               phi₁ ∨ phi₂ ∨ …  (smooth-max)

Probabilistic extension (planned)
----------------------------------
When pose uncertainty (covariance) is available, E[ρ] and Var[ρ] can be
computed via first-order linearisation:

    E[ρ] ≈ ρ(μ)
    Var[ρ] ≈ (∇ρ)ᵀ Σ (∇ρ)

and the probability of satisfaction estimated as P(ρ > 0) ≈ Φ(E[ρ]/√Var[ρ]).
Hook-in point: add an `uncertainty(sig, t, Sigma)` method alongside
`robustness(sig, t)` once Lighthouse covariance is available.

Example
-------
    import math
    from formula import Predicate, Always, Eventually, And

    safe_z  = Predicate(lambda x, y, z: z - 0.2,          name="z>0.2m")
    at_goal = Predicate(lambda x, y, z: 0.15 - math.hypot(x - 1.0, y - 0.0),
                        name="at_goal")
    phi = And(Always(safe_z, 0, 30), Eventually(at_goal, 0, 10))
"""

from __future__ import annotations

import math
from collections import deque
from typing import Callable, NamedTuple


# ── Signal ────────────────────────────────────────────────────────────────────

class Sample(NamedTuple):
    t: float
    x: float
    y: float
    z: float


class Signal3D:
    """Rolling buffer of 3-D position samples, queryable by time window."""

    def __init__(self, maxlen: int = 1000):
        self._buf: deque[Sample] = deque(maxlen=maxlen)

    def push(self, t: float, x: float, y: float, z: float) -> None:
        self._buf.append(Sample(t, x, y, z))

    def window(self, t_lo: float, t_hi: float) -> list[Sample]:
        """Return all samples with timestamp in [t_lo, t_hi]."""
        return [s for s in self._buf if t_lo <= s.t <= t_hi]

    def closest(self, t: float) -> Sample | None:
        """Return the sample nearest to time t."""
        if not self._buf:
            return None
        return min(self._buf, key=lambda s: abs(s.t - t))

    @property
    def latest(self) -> Sample | None:
        return self._buf[-1] if self._buf else None


# ── Smooth min / max ──────────────────────────────────────────────────────────

# Higher β → closer to true min/max but more numerically extreme.
# β = 10 is a practical default for metre-scale drone control.
_BETA = 10.0


def _smin(vals: list[float], beta: float = _BETA) -> float:
    """Smooth min via log-sum-exp: lim_{β→∞} = true min."""
    if not vals:
        return 0.0
    scaled = [-beta * v for v in vals]
    m = max(scaled)
    return -1.0 / beta * (m + math.log(sum(math.exp(s - m) for s in scaled)))


def _smax(vals: list[float], beta: float = _BETA) -> float:
    """Smooth max via log-sum-exp: lim_{β→∞} = true max."""
    if not vals:
        return 0.0
    scaled = [beta * v for v in vals]
    m = max(scaled)
    return 1.0 / beta * (m + math.log(sum(math.exp(s - m) for s in scaled)))


# ── Formula classes ───────────────────────────────────────────────────────────

class Predicate:
    """
    Atomic predicate: ρ = mu(x, y, z).

    mu should return positive values when the predicate holds and
    negative values when it is violated.  The magnitude encodes margin.

    Example — "altitude above 20 cm":
        Predicate(lambda x, y, z: z - 0.2, name="z>0.2m")
    """

    def __init__(
        self,
        mu: Callable[[float, float, float], float],
        name: str = "",
    ):
        self._mu  = mu
        self.name = name

    def robustness(self, sig: Signal3D, t: float) -> float:
        s = sig.closest(t)
        if s is None:
            return 0.0
        return self._mu(s.x, s.y, s.z)


class Always:
    """
    G[a,b] phi — formula must hold at every instant in the window.

    Robustness = smooth-min of phi over samples in [t-b, t-a].
    Returns +100 for an empty window (vacuously true — no samples yet).

    Parameters
    ----------
    phi : formula
        Sub-formula.
    a, b : float
        Time window bounds in seconds (0 ≤ a ≤ b).
    """

    def __init__(self, phi, a: float = 0.0, b: float = 10.0):
        self.phi, self.a, self.b = phi, a, b

    def robustness(self, sig: Signal3D, t: float) -> float:
        samples = sig.window(t - self.b, t - self.a)
        if not samples:
            return 100.0  # vacuously true — no samples in window yet
        return _smin([self.phi.robustness(sig, s.t) for s in samples])


class Eventually:
    """
    F[a,b] phi — formula must hold at some instant in the window.

    Robustness = smooth-max of phi over samples in [t-b, t-a].
    Returns -100 for an empty window (no witness observed yet).

    Parameters
    ----------
    phi : formula
        Sub-formula.
    a, b : float
        Time window bounds in seconds (0 ≤ a ≤ b).
    """

    def __init__(self, phi, a: float = 0.0, b: float = 10.0):
        self.phi, self.a, self.b = phi, a, b

    def robustness(self, sig: Signal3D, t: float) -> float:
        samples = sig.window(t - self.b, t - self.a)
        if not samples:
            return -100.0  # no witness yet
        return _smax([self.phi.robustness(sig, s.t) for s in samples])


class And:
    """phi₁ ∧ phi₂ ∧ … — robustness = smooth-min of all sub-formulas."""

    def __init__(self, *phis):
        self._phis = phis

    def robustness(self, sig: Signal3D, t: float) -> float:
        return _smin([phi.robustness(sig, t) for phi in self._phis])


class Or:
    """phi₁ ∨ phi₂ ∨ … — robustness = smooth-max of all sub-formulas."""

    def __init__(self, *phis):
        self._phis = phis

    def robustness(self, sig: Signal3D, t: float) -> float:
        return _smax([phi.robustness(sig, t) for phi in self._phis])
