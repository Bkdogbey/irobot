"""
Numpy-based STL predicates and temporal operators for the Crazyflie mission.

Self-contained — no PyTorch, no imports from probabilistic_stl.
Semantics mirror probabilistic_stl/pdstl/operators.py but operate directly
on lists of GaussianBelief2D objects.

Probability bounds follow the Fréchet / StoRI convention:
    trace[t] = (lower, upper)  ∈ [0,1]²

Temporal operators:
    Always[a,b]    — min over the window  (□)
    Eventually[a,b] — max over the window  (♢)
    And            — Fréchet conjunction
    Or             — Fréchet disjunction
"""

from __future__ import annotations

from typing import Sequence

from belief import GaussianBelief2D

Bounds = tuple[float, float]
Trace  = list[Bounds]          # one (lower, upper) per time step


# ── Predicates ────────────────────────────────────────────────────────────────

class OutsideObstacle:
    """
    P(position is outside obstacle rectangle) at a single belief.

    Returns (lower, upper) probability bounds.
    """

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float, name: str = 'obs'):
        self.bounds = (x_min, x_max, y_min, y_max)
        self.name   = name

    def __call__(self, belief: GaussianBelief2D) -> Bounds:
        return belief.prob_outside_rect(*self.bounds)

    def __str__(self) -> str:
        return f'outside({self.name})'


class InsideGoal:
    """
    P(position is inside goal region) at a single belief.
    """

    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float, name: str = 'goal'):
        self.bounds = (x_min, x_max, y_min, y_max)
        self.name   = name

    def __call__(self, belief: GaussianBelief2D) -> Bounds:
        return belief.prob_inside_rect(*self.bounds)

    def __str__(self) -> str:
        return f'in_goal({self.name})'


# ── Boolean connectives ───────────────────────────────────────────────────────

def stl_and(a: Bounds, b: Bounds) -> Bounds:
    """Fréchet conjunction: lower = max(la+lb−1, 0), upper = min(ua, ub)."""
    lower = max(a[0] + b[0] - 1.0, 0.0)
    upper = min(a[1], b[1])
    return lower, upper


def stl_or(a: Bounds, b: Bounds) -> Bounds:
    """Fréchet disjunction: lower = max(la, lb), upper = min(ua+ub, 1)."""
    lower = max(a[0], b[0])
    upper = min(a[1] + b[1], 1.0)
    return lower, upper


def stl_not(a: Bounds) -> Bounds:
    """Negation: [1−upper, 1−lower]."""
    return 1.0 - a[1], 1.0 - a[0]


# ── Temporal operators ────────────────────────────────────────────────────────

class Always:
    """
    □[a,b] φ — probability of φ holding at every step in the window.

    Evaluated as min (lower) and min (upper) over the belief trace.
    Time is in discrete steps; interval defaults to [0, len(trace)−1].
    """

    def __init__(self, predicate, interval: list[int] | None = None):
        self.predicate = predicate
        self.interval  = interval

    def evaluate(self, belief_trace: Sequence[GaussianBelief2D]) -> Trace:
        """
        Returns a Trace — one (lower, upper) per time step t.

        At step t the operator looks forward over [t+a, t+b].
        """
        T   = len(belief_trace)
        a   = self.interval[0] if self.interval else 0
        b   = self.interval[1] if self.interval else T - 1

        result: Trace = []
        for t in range(T):
            lo, hi = t + a, min(t + b, T - 1)
            if lo > hi:
                result.append((1.0, 1.0))   # vacuously true
                continue
            probs = [self.predicate(belief_trace[k]) for k in range(lo, hi + 1)]
            lower = min(p[0] for p in probs)
            upper = min(p[1] for p in probs)
            result.append((lower, upper))
        return result

    def __str__(self) -> str:
        interval_str = str(self.interval) if self.interval else ''
        return f'□{interval_str}({self.predicate})'


class Eventually:
    """
    ♢[a,b] φ — probability that φ holds at some step in the window.

    Evaluated as max (lower) and max (upper) over the belief trace.
    """

    def __init__(self, predicate, interval: list[int] | None = None):
        self.predicate = predicate
        self.interval  = interval

    def evaluate(self, belief_trace: Sequence[GaussianBelief2D]) -> Trace:
        T   = len(belief_trace)
        a   = self.interval[0] if self.interval else 0
        b   = self.interval[1] if self.interval else T - 1

        result: Trace = []
        for t in range(T):
            lo, hi = t + a, min(t + b, T - 1)
            if lo > hi:
                result.append((0.0, 0.0))   # impossible in empty window
                continue
            probs = [self.predicate(belief_trace[k]) for k in range(lo, hi + 1)]
            lower = max(p[0] for p in probs)
            upper = max(p[1] for p in probs)
            result.append((lower, upper))
        return result

    def __str__(self) -> str:
        interval_str = str(self.interval) if self.interval else ''
        return f'♢{interval_str}({self.predicate})'


# ── Mission spec builder ──────────────────────────────────────────────────────

def build_mission_spec(obstacles: list[tuple], goal_region: tuple, T: int):
    """
    φ = ♢[0,T](in_goal)  ∧  □[0,T](outside_obs1 ∧ outside_obs2 ∧ ...)

    Args:
        obstacles:   list of (x_min, x_max, y_min, y_max)
        goal_region: (x_min, x_max, y_min, y_max)
        T:           horizon in steps

    Returns:
        (phi_reach, phi_safe, phi_mission)
        Each is a callable that takes a belief trace and returns a Trace.
    """
    avoid_preds = [
        OutsideObstacle(*obs, name=f'obs{i+1}')
        for i, obs in enumerate(obstacles)
    ]
    goal_pred = InsideGoal(*goal_region)

    phi_reach = Eventually(goal_pred,   interval=[0, T])
    phi_safe  = Always(
        lambda b: _eval_all_avoid(b, avoid_preds),
        interval=[0, T],
    )

    def phi_mission(belief_trace: Sequence[GaussianBelief2D]) -> Trace:
        reach_trace = phi_reach.evaluate(belief_trace)
        safe_trace  = phi_safe.evaluate(belief_trace)
        return [stl_and(r, s) for r, s in zip(reach_trace, safe_trace)]

    return phi_reach, phi_safe, phi_mission


def _eval_all_avoid(
    belief: GaussianBelief2D,
    avoid_preds: list[OutsideObstacle],
) -> Bounds:
    """Conjunction of all obstacle-avoidance predicates at a single belief."""
    result: Bounds = (1.0, 1.0)
    for pred in avoid_preds:
        result = stl_and(result, pred(belief))
    return result
