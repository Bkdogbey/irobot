"""Crazyflie 3D figure-8: a vertical figure-eight that also climbs and descends.

Same in-plane shape as figure_eight_flight.py (a Lemniscate of Gerono, symmetric
about x = 0.5, spanning y in [-2, 0] with the gate at (0.5, -1) and total lobe
width 0.5 m), but the altitude now varies along the path instead of staying flat —
so the curve is a true 3D space curve rather than a planar one.

    x(φ) = 0.5 + HALF_WIDTH * sin(2φ)                       # x in [0.25, 0.75]
    y(φ) = -1.0 + V_HALF     * cos(φ)                        # y in [-2, 0]
    z    = Z_MIN + (Z_MAX - Z_MIN) * (y - BOTTOM_Y) / span   # z tracks height on y

Altitude leans with the climb: low at the bottom tips (Z_MIN), mid at the gate,
high at the top apex (Z_MAX) — both safely under the 0.60 m ceiling. For φ from π
to 3π the drone starts low at the bottom tip (0.5,-2,Z_MIN), rises through the
lower lobe to the gate, over the apex at (0.5,0,Z_MAX), and back down to the
bottom tip, where it lands in place.

The curve is smooth by construction (no spline needed), sampled densely and flown
with PositionHlCommander (each leg's duration sized from distance / velocity). The
drone takes off wherever it is, flies to the bottom tip at TRANSIT_HEIGHT, descends
to Z_MIN, traces the 3D 8, and lands.

    python3 irobot/src/robots/crazyflie/examples/figure_eight_3d_flight.py   # from repo root

Requires cflib + a Crazyradio and absolute positioning (Lighthouse). Does not
depend on ros_sugar. Override the drone with e.g.
    CFLIB_URI=radio://0/85/2M/E7E7E7E85 python3 ...figure_eight_3d_flight.py
"""

from __future__ import annotations

import time

import cflib.crtp
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

# --- Mission parameters -----------------------------------------------------
URI = uri_helper.uri_from_env(default="radio://0/85/2M/E7E7E7E85")

Z_MIN = 0.15               # m — altitude at the bottom tips (start / end)
Z_MAX = 0.65               # m — altitude at the top apex (hard cap: never exceed 0.60)
TAKEOFF_HEIGHT = 0.20      # m — brief initial climb before transiting to the start
TRANSIT_HEIGHT = 0.60      # m — fly to the start at this height, then descend to Z_MIN
CRUISE_VELOCITY = 0.30     # m/s — PositionHlCommander sizes each leg from this
TAKEOFF_VELOCITY = 0.30    # m/s for the vertical takeoff
WAYPOINT_DELAY = 0.10      # s pause between waypoints (keeps the motion flowing)

ESTIMATOR_SPREAD_LIMIT = 0.08     # max x/y estimate jitter (m) before we call it settled
ESTIMATOR_SETTLE_TIMEOUT = 12.0   # give up if the estimate never settles (s)

# --- Path definition --------------------------------------------------------
# Vertical Lemniscate of Gerono, symmetric about x = 0.5, with a leaning altitude
# profile. Traced parametrically (smooth by construction) over one full loop,
# φ from π (bottom tip) to 3π (back to the bottom tip).
GATE = (0.5, -1.0)         # figure centre / self-crossing point (x, y)
TOP_Y = 0.0                # top apex y
BOTTOM_Y = -2.0            # bottom tip y (= start / end)
V_HALF = (TOP_Y - BOTTOM_Y) / 2.0   # 1.0 — vertical half-height about the gate
WIDTH = 0.8                # total side-to-side width of the lobes
HALF_WIDTH = WIDTH / 2.0   # 0.25 — bulge each side of x = 0.5
N_SAMPLES = 100            # samples around the full loop (denser = smoother)


def _figure_eight_xyz(n: int = N_SAMPLES) -> np.ndarray:
    """Sample the 3D figure-8; return (n, 3) [x, y, z] in flight order."""
    phi = np.linspace(np.pi, 3 * np.pi, n)
    x = GATE[0] + HALF_WIDTH * np.sin(2 * phi)
    y = GATE[1] + V_HALF * np.cos(phi)
    z = Z_MIN + (Z_MAX - Z_MIN) * (y - BOTTOM_Y) / (TOP_Y - BOTTOM_Y)
    return np.column_stack([x, y, z])


FIGURE8_WAYPOINTS = _figure_eight_xyz()


def reset_estimator(scf: SyncCrazyflie) -> None:
    scf.cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    scf.cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2.0)


def measure_position(
    scf: SyncCrazyflie,
    sample_count: int = 10,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Sample the estimate; return (mean xyz, xyz spread, base-station bitmask).

    The bitmask ORs lighthouse.bsReceive across the samples, so each set bit is a
    base station seen at least once in the window (0 if the deck reports none).
    """
    log_config = LogConfig(
        name="Position check",
        period_in_ms=100,
    )
    log_config.add_variable("stateEstimate.x", "float")
    log_config.add_variable("stateEstimate.y", "float")
    log_config.add_variable("stateEstimate.z", "float")
    have_bs = "bsReceive" in scf.cf.log.toc.toc.get("lighthouse", {})
    if have_bs:
        log_config.add_variable("lighthouse.bsReceive")

    samples = []
    bs_seen = 0

    with SyncLogger(scf, log_config) as logger:
        for _, data, _ in logger:
            samples.append(
                [
                    data["stateEstimate.x"],
                    data["stateEstimate.y"],
                    data["stateEstimate.z"],
                ],
            )
            if have_bs:
                bs_seen |= int(data["lighthouse.bsReceive"])

            if len(samples) >= sample_count:
                break

    sample_array = np.asarray(samples)
    return np.mean(sample_array, axis=0), np.ptp(sample_array, axis=0), bs_seen


def wait_for_stable_estimate(scf: SyncCrazyflie) -> np.ndarray:
    """Reset the Kalman estimator and wait until the position estimate settles.

    Returns the settled (x, y, z) estimate. The drone can be sitting anywhere;
    we only require a stable lock, not a specific start coordinate. Prints live
    feedback (position spread + how many base stations are seen) each poll, and
    raises if it never stabilises within ESTIMATOR_SETTLE_TIMEOUT.
    """
    reset_estimator(scf)

    deadline = time.time() + ESTIMATOR_SETTLE_TIMEOUT
    while True:
        measured, spread, bs_seen = measure_position(scf)
        n_bs = bin(bs_seen).count("1")
        print(
            f"  settling: pos=({measured[0]:+.2f}, {measured[1]:+.2f}, {measured[2]:+.2f})  "
            f"x/y spread=({spread[0]:.3f}, {spread[1]:.3f})  "
            f"base stations seen={n_bs} (mask {bs_seen:#06b})",
        )

        if np.all(spread[:2] <= ESTIMATOR_SPREAD_LIMIT):
            print(
                f"Stable estimate at ({measured[0]:+.2f}, {measured[1]:+.2f}, {measured[2]:+.2f}).",
            )
            return measured

        if time.time() > deadline:
            hint = (
                "Only one base station is being received — you need 2 for a solid lock. "
                if n_bs < 2
                else ""
            )
            raise RuntimeError(
                f"Position estimate never stabilised (x/y spread={spread[:2]}, "
                f"base stations seen={n_bs}). {hint}Check Lighthouse coverage/placement.",
            )
        time.sleep(0.5)


def _send_arming_request(cf: Crazyflie, do_arm: bool) -> None:
    """Arm/disarm via the current supervisor API, falling back to platform."""
    try:
        cf.supervisor.send_arming_request(do_arm)
    except AttributeError:
        cf.platform.send_arming_request(do_arm)


def run_flight(scf: SyncCrazyflie) -> None:
    airborne = False
    commander = None

    try:
        settled = wait_for_stable_estimate(scf)

        # Initialise the commander at the settled position and let it select the
        # PID controller (does not reset the estimator).
        commander = PositionHlCommander(
            scf,
            x=float(settled[0]),
            y=float(settled[1]),
            z=float(settled[2]),
            default_velocity=CRUISE_VELOCITY,
            default_height=TAKEOFF_HEIGHT,
            controller=PositionHlCommander.CONTROLLER_PID,
        )

        _send_arming_request(scf.cf, True)
        time.sleep(1.0)

        print(f"Taking off to {TAKEOFF_HEIGHT:.2f} m")
        commander.take_off(TAKEOFF_HEIGHT, TAKEOFF_VELOCITY)
        airborne = True

        # Fly from wherever we took off to the bottom tip (FIGURE8_WAYPOINTS[0] =
        # (0.5,-2)) at TRANSIT_HEIGHT, then descend to the tip's low altitude (Z_MIN).
        start_x, start_y, start_z = (float(v) for v in FIGURE8_WAYPOINTS[0])
        print(f"Flying to start ({start_x:.2f}, {start_y:.2f}) at {TRANSIT_HEIGHT:.2f} m")
        commander.go_to(start_x, start_y, TRANSIT_HEIGHT)
        commander.go_to(start_x, start_y, start_z)

        # Trace the 3D figure-8 — each waypoint carries its own altitude, so the drone
        # climbs to the apex and descends back as it goes. PositionHlCommander sizes
        # each leg's duration from the 3D distance and CRUISE_VELOCITY.
        print(f"Tracing 3D figure-8 through {len(FIGURE8_WAYPOINTS)} waypoints "
              f"(z {Z_MIN:.2f}–{Z_MAX:.2f} m)")
        for i in range(1, len(FIGURE8_WAYPOINTS)):
            x, y, z = (float(v) for v in FIGURE8_WAYPOINTS[i])
            commander.go_to(x, y, z)
            time.sleep(WAYPOINT_DELAY)

        # The 8 closes low at the bottom tip (0.5,-2) — land in place.
        print("Landing")
        commander.land()
        airborne = False

    except KeyboardInterrupt:
        print("Flight interrupted.")

    finally:
        if airborne and commander is not None:
            try:
                commander.land()
            except Exception:
                pass
        _send_arming_request(scf.cf, False)


def main() -> None:
    print(f"3D figure-8 ({len(FIGURE8_WAYPOINTS)} pts, width {WIDTH:.2f} m, "
          f"z {Z_MIN:.2f}–{Z_MAX:.2f} m): "
          f"start {tuple(round(v, 2) for v in FIGURE8_WAYPOINTS[0])} -> "
          f"gate ({GATE[0]:.2f}, {GATE[1]:.2f}) -> "
          f"apex ({GATE[0]:.2f}, {TOP_Y:.2f}, {Z_MAX:.2f}) -> "
          f"end {tuple(round(v, 2) for v in FIGURE8_WAYPOINTS[-1])}")

    cflib.crtp.init_drivers()

    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache"),
    ) as scf:
        run_flight(scf)


if __name__ == "__main__":
    main()
