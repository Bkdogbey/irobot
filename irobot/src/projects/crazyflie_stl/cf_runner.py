"""
Hardware runner — Crazyflie STL mission on real hardware.

Replaces the simple spline-follower in probabilistic_stl/components/crazyflie.py
with the reactive STL belief-update loop.

Control interface (matches CrazyflieBase):
    takeoff(z_hold)                          — ramp to hover altitude
    send_velocity_setpoint(vx, vy, 0, z)    — send_hover_setpoint under the hood
    land(z_hold)                             — ramp down and cut motors

Measurement source:
    cf.current_x, cf.current_y              — Lighthouse state at 20 Hz
    cf.state_ready                           — True once first Lighthouse packet arrives

Loop rate: DT = 0.05 s  (20 Hz — matches Lighthouse logging period)

Run (from repo root):
    python -m irobot.src.projects.crazyflie_stl.cf_runner
"""

from __future__ import annotations

import logging
import time

import numpy as np
from belief import GaussianBelief2D
from controller import STLController

# STL module — self-contained, no imports from probabilistic_stl
from scenario import GOAL_REGION, OBSTACLES, Z_FLIGHT, get_nominal_waypoints

from irobot.src.projects.probabilistic_stl.components.config import CrazyflieConfig
from irobot.src.robots.crazyflie.core.base import CrazyflieBase

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── Mission parameters ────────────────────────────────────────────────────────
DT = 0.05  # control loop period (s) — 20 Hz
MAX_STEPS = 1200  # hard cutoff: 60 s
HOVER_STEPS = 100  # 5 s hover at goal before landing

Q_STD = 0.02  # process noise std dev (m)
R_STD = 0.01  # Lighthouse measurement noise std dev (m)
V_MAX = 0.20  # max speed (m/s) — conservative for indoor flight
WP_TOL = 0.10  # waypoint advance distance (m)
SAFETY_THRESH = 0.90  # P(outside_obs) below this triggers repulsion


# ── Helper ────────────────────────────────────────────────────────────────────


def _wait_for_state(cf: CrazyflieBase, timeout: float = 10.0) -> bool:
    """Block until the Lighthouse state estimate is ready."""
    deadline = time.time() + timeout
    while not cf.state_ready and time.time() < deadline:
        time.sleep(0.05)
    if not cf.state_ready:
        logger.error('Lighthouse state not ready after %.1f s — aborting', timeout)
        return False
    logger.info('Lighthouse ready: x=%.3f  y=%.3f  z=%.3f', cf.current_x, cf.current_y, cf.current_z)
    return True


def _in_goal(x: float, y: float) -> bool:
    x0, x1, y0, y1 = GOAL_REGION
    return x0 <= x <= x1 and y0 <= y <= y1


# ── Main mission ──────────────────────────────────────────────────────────────


def run_hardware_mission() -> None:
    """
    Full hardware mission loop.

    1. Connect + wait for Lighthouse lock.
    2. Take off to Z_FLIGHT.
    3. Reactive STL loop:
       a. Read Lighthouse position → Kalman update.
       b. Compute STL-gated velocity.
       c. Send velocity setpoint.
       d. Kalman predict.
    4. Hover at goal for HOVER_STEPS.
    5. Land unconditionally (try/finally).
    """
    config = CrazyflieConfig()
    cf = CrazyflieBase(config)

    if not cf.is_connected:
        logger.error('Could not connect to Crazyflie. Exiting.')
        return

    if not _wait_for_state(cf):
        cf.disconnect()
        return

    # ── Initialise belief at current Lighthouse position ──────────────────────
    start_pos = np.array([cf.current_x, cf.current_y])
    belief = GaussianBelief2D(
        mean=start_pos.copy(),
        cov=np.eye(2) * (R_STD * 3) ** 2,  # slightly uncertain at start
    )

    waypoints = get_nominal_waypoints(n_points=60)
    ctrl = STLController(
        waypoints=waypoints,
        obstacles=OBSTACLES,
        goal_region=GOAL_REGION,
        v_max=V_MAX,
        wp_tol=WP_TOL,
        safety_thresh=SAFETY_THRESH,
    )

    reached = False
    hover_count = 0

    try:
        # ── Takeoff ───────────────────────────────────────────────────────────
        logger.info('Taking off to z=%.2f m', Z_FLIGHT)
        cf.takeoff(z_hold=Z_FLIGHT, duration=3.0)
        time.sleep(0.5)  # settle

        # ── Control loop ──────────────────────────────────────────────────────
        for step in range(MAX_STEPS):
            t0 = time.time()

            # 1. Measurement from Lighthouse
            meas = np.array([cf.current_x, cf.current_y])

            # 2. Kalman update
            belief = belief.update(meas, r_std=R_STD)

            # 3. STL safety log (every 2 s)
            if step % 40 == 0:
                for name, lo, hi in ctrl.stl_safety(belief):
                    logger.info('  STL %s: P_outside=[%.3f, %.3f]', name, lo, hi)

            # 4. Goal check
            if _in_goal(meas[0], meas[1]):
                if not reached:
                    reached = True
                    logger.info('[step %d] Goal reached — hovering for %.1f s', step, HOVER_STEPS * DT)
                hover_count += 1
                vel = np.zeros(2)
                if hover_count >= HOVER_STEPS:
                    logger.info('[step %d] Hover complete.', step)
                    break
            else:
                # 5. Reactive STL velocity
                vel = ctrl.step(belief)

            # 6. Send velocity setpoint
            cf.send_velocity_setpoint(float(vel[0]), float(vel[1]), 0.0, Z_FLIGHT)

            # 7. Kalman predict (forward one step)
            belief = belief.predict(vel, DT, q_std=Q_STD)

            # 8. Pace the loop
            elapsed = time.time() - t0
            sleep_t = max(0.0, DT - elapsed)
            time.sleep(sleep_t)

    finally:
        # Always land, whether we completed normally, hit MAX_STEPS, or crashed
        logger.info('Landing.')
        cf.land(z_hold=Z_FLIGHT, duration=3.0)
        cf.disconnect()
        if not reached:
            logger.warning('Mission ended without reaching goal.')


if __name__ == '__main__':
    run_hardware_mission()
