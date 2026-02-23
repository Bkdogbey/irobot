"""
PdSTL Planner Component
=======================
A ros_sugar BaseComponent that closes the loop between pose observations
and velocity commands using a pdSTL robustness-gradient controller.

Subscribes:  /cf/pose  (geometry_msgs/PoseStamped)
Publishes:   /cf/cmd   (geometry_msgs/Twist)

Control law
-----------
At each pose update:
    1. Push (t, x, y, z) into the signal buffer.
    2. Evaluate ρ = formula.robustness(signal, t_now).
    3. If ρ ≥ margin: publish zero velocity — formula already satisfied.
    4. Else: compute ∇ρ via central differences and publish
                v = clip(k_p · ∇ρ, −v_max, v_max)

Gradient computation
--------------------
The finite-difference probes temporarily append a test sample, evaluate
the formula, then restore the buffer — no allocation, O(1) overhead:

    rho_fwd = probe(x+ε, y, z)
    rho_bwd = probe(x−ε, y, z)
    ∂ρ/∂x  = (rho_fwd − rho_bwd) / 2ε

Note on ros_sugar callback naming
----------------------------------
ros_sugar auto-discovers callbacks by stripping the leading namespace
from the topic name and appending `_callback`.  With topic "/cf/pose"
the expected method name is `pose_callback`.  This matches cf_node.py's
`cmd_callback` convention for "/cf/cmd".
"""

from __future__ import annotations

import copy
import time

from geometry_msgs.msg import PoseStamped, Twist
from ros_sugar.core import BaseComponent
from ros_sugar.io import Topic

from projects.probabilistic_stl.formula import Sample, Signal3D


# ── Controller constants (override via constructor kwargs) ────────────────────

_EPS    = 0.01   # finite-difference step, metres
_K_P    = 0.5    # proportional gain (m/s per unit robustness gradient)
_V_MAX  = 0.3    # velocity saturation per axis, m/s
_MARGIN = 0.05   # stop controlling when ρ exceeds this — formula satisfied


def _clip(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


# ─────────────────────────────────────────────────────────────────────────────

class PdSTLComponent(BaseComponent):
    """
    pdSTL feedback controller as a ros_sugar component.

    Parameters
    ----------
    formula : Any formula from formula.py (Predicate, And, Always, …)
        The STL specification to satisfy.
    k_p : float
        Proportional gain on the robustness gradient.
    v_max : float
        Velocity saturation per axis (m/s).
    margin : float
        Robustness threshold above which no command is sent (formula met).
    component_name : str
        ROS2 node name.

    Example
    -------
    See recipes/cf_pdstl.py for a full two-component launcher.
    """

    def __init__(
        self,
        formula,
        *,
        k_p: float     = _K_P,
        v_max: float   = _V_MAX,
        margin: float  = _MARGIN,
        component_name: str = "pdstl_planner",
        **kwargs,
    ):
        self._formula = formula
        self._k_p     = k_p
        self._v_max   = v_max
        self._margin  = margin
        self._sig     = Signal3D()
        self._t0: float | None = None

        inputs  = [Topic(PoseStamped, "/cf/pose")]
        outputs = [Topic(Twist,       "/cf/cmd")]

        super().__init__(component_name, inputs=inputs, outputs=outputs, **kwargs)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def on_start(self):
        self._t0 = time.time()
        print("[pdSTL] Planner started.")

    # ── ROS2 callback ─────────────────────────────────────────────────────────

    def pose_callback(self, msg: PoseStamped):
        """Receive /cf/pose, update signal buffer, publish /cf/cmd."""
        if self._t0 is None:
            self._t0 = time.time()

        p = msg.pose.position
        t = time.time() - self._t0
        self._sig.push(t, p.x, p.y, p.z)
        self._step(t, p.x, p.y, p.z)

    # ── Control step ──────────────────────────────────────────────────────────

    def _step(self, t: float, x: float, y: float, z: float) -> None:
        rho = self._formula.robustness(self._sig, t)

        if rho >= self._margin:
            cmd = Twist()  # zero — formula satisfied
            print(f"[pdSTL] ρ={rho:+.3f}  SATISFIED — holding position")
        else:
            dx = self._grad(t, x, y, z, axis=0)
            dy = self._grad(t, x, y, z, axis=1)
            dz = self._grad(t, x, y, z, axis=2)

            cmd = Twist()
            cmd.linear.x = _clip(self._k_p * dx, self._v_max)
            cmd.linear.y = _clip(self._k_p * dy, self._v_max)
            cmd.linear.z = _clip(self._k_p * dz, self._v_max)

            print(
                f"[pdSTL] ρ={rho:+.3f}  "
                f"∇ρ=({dx:+.3f},{dy:+.3f},{dz:+.3f})  "
                f"v=({cmd.linear.x:+.2f},{cmd.linear.y:+.2f},{cmd.linear.z:+.2f})"
            )

        self.publishers["/cf/cmd"].publish(cmd)

    # ── Gradient ──────────────────────────────────────────────────────────────

    def _grad(self, t: float, x: float, y: float, z: float, axis: int) -> float:
        """Central-difference ∂ρ/∂axis at (x, y, z)."""
        delta = [0.0, 0.0, 0.0]
        delta[axis] = _EPS
        rho_fwd = self._probe(t, x + delta[0], y + delta[1], z + delta[2])
        rho_bwd = self._probe(t, x - delta[0], y - delta[1], z - delta[2])
        return (rho_fwd - rho_bwd) / (2 * _EPS)

    def _probe(self, t: float, x: float, y: float, z: float) -> float:
        """
        Evaluate formula at a hypothetical position without permanently
        modifying the signal buffer.

        Strategy: save a shallow copy of the deque, append the test sample,
        evaluate, then restore.  O(n) copy but allocates no extra Sample
        objects (deque holds references).
        """
        saved = copy.copy(self._sig._buf)   # shallow-copy the deque (O(n))
        self._sig.push(t, x, y, z)
        rho = self._formula.robustness(self._sig, t)
        self._sig._buf = saved              # restore
        return rho
