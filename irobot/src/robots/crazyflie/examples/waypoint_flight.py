"""Crazyflie loop flight: a smooth curve out and a raised curve back.

FORWARD (0.30 m): from START (1,-2) to (0.5,0), passing through (0.5,-1.62), the
midpoint (0.35,-1), and (0.4,-0.5) — bulging left.
RETURN: from (0.5,0) curve up via (0.25,0.25) to (0.1,0); there climb to
RAISED_HEIGHT (0.60 m) and fly the straight leg down to (0.1,-0.65); descend back
to 0.30 m and curve through (0.35,-1), (0.15,-1.5) to the END (0,-2).

Each leg is a shape-preserving PCHIP spline through its control points, sampled
densely and flown with PositionHlCommander (each leg's duration sized from
distance / velocity, so motion stays smooth). The drone takes off wherever it is,
flies to the start at TRANSIT_HEIGHT, descends to the cruise height, traces the
forward curve, climbs to the return lane, traces the way back, descends, and lands
at (0,-2).

    python3 irobot/src/robots/crazyflie/examples/waypoint_flight.py   # from repo root

Requires cflib + a Crazyradio and absolute positioning (Lighthouse). Does not
depend on ros_sugar. Override the drone with e.g.
    CFLIB_URI=radio://0/85/2M/E7E7E7E85 python3 ...waypoint_flight.py
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
from scipy.interpolate import PchipInterpolator

# --- Mission parameters -----------------------------------------------------
URI = uri_helper.uri_from_env(default="radio://0/85/2M/E7E7E7E85")

HEIGHT = 0.20              # constant cruise altitude (m) for the curve
TRANSIT_HEIGHT = 0.60      # m — fly to the start at this height, then descend to HEIGHT
CRUISE_VELOCITY = 0.30     # m/s — PositionHlCommander sizes each leg from this
TAKEOFF_VELOCITY = 0.30    # m/s for the vertical takeoff
WAYPOINT_DELAY = 0.10      # s pause between waypoints (keeps the motion flowing)

ESTIMATOR_SPREAD_LIMIT = 0.08     # max x/y estimate jitter (m) before we call it settled
ESTIMATOR_SETTLE_TIMEOUT = 12.0   # give up if the estimate never settles (s)

# --- Path definition --------------------------------------------------------
# Each curved segment is a shape-preserving PCHIP spline through its control
# points, sampled densely so PositionHlCommander traces one smooth curve that hits
# each point with no overshoot between them.
_N = 16   # samples per curve segment (denser = smoother)


def _pchip_xy(control: np.ndarray, axis: str, n: int = _N) -> np.ndarray:
    """Sample a PCHIP through `control` (Kx2 [x, y]); return (M, 2) [x, y].

    axis='y' fits x(y); axis='x' fits y(x). Samples span the independent axis'
    range (including the exact control values), ascending in that axis — reverse
    the result to flip flight direction.
    """
    col = 1 if axis == "y" else 0
    other = 1 - col
    fit = PchipInterpolator(control[:, col], control[:, other])
    s = np.union1d(np.linspace(control[:, col].min(), control[:, col].max(), n), control[:, col])
    out = np.empty((len(s), 2))
    out[:, col] = s
    out[:, other] = fit(s)
    return out


def _with_z(xy: np.ndarray, z: float) -> np.ndarray:
    return np.column_stack([xy, np.full(len(xy), z)])


# FORWARD curve (flat HEIGHT): START (1,-2) -> (0.5,0), bulging left. Control
# points in flight order (y increasing).
FORWARD_CONTROL_POINTS = np.array([
    [1.00, -2.00],   # START
    [0.50, -1.62],   # curves the first half more
    [0.35, -1.00],   # midpoint
    [0.40, -0.50],   # shapes the second half
    [0.50,  0.00],   # forward end
])
FORWARD_WAYPOINTS = _with_z(_pchip_xy(FORWARD_CONTROL_POINTS, "y"), HEIGHT)

# RETURN leg, as (x, y, z) waypoints (the shape is not single-valued in y and the
# altitude changes, so it is built from segments):
#   A: (0.5,0) -> curve up via (0.25,0.25) -> (0.1,0)         at HEIGHT
#   B: climb in place, straight to (0.1,-0.65), descend        RAISED_HEIGHT -> HEIGHT
#   C: curve through (0.35,-1), (0.15,-1.5) to the END (0,-2)  at HEIGHT
RAISED_HEIGHT = 0.60   # m — raised altitude (>0.5) for the (0.1,0) -> (0.1,-0.65) leg
_ret_a = _with_z(_pchip_xy(np.array([[0.10, 0.00], [0.25, 0.25], [0.50, 0.00]]), "x")[::-1], HEIGHT)
_ret_b = np.array([
    [0.10,  0.00, RAISED_HEIGHT],   # climb in place at (0.1, 0)
    [0.10, -0.65, RAISED_HEIGHT],   # straight leg at the raised altitude
    [0.10, -0.65, HEIGHT],          # descend in place
])
_ret_c = _with_z(_pchip_xy(np.array([[0.00, -2.00], [0.15, -1.50], [0.35, -1.00], [0.10, -0.65]]), "y")[::-1], HEIGHT)
RETURN_WAYPOINTS = np.vstack([_ret_a, _ret_b, _ret_c[1:]])   # _ret_c[0] duplicates (0.1,-0.65)


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
            default_height=HEIGHT,
            controller=PositionHlCommander.CONTROLLER_PID,
        )

        _send_arming_request(scf.cf, True)
        time.sleep(1.0)

        print(f"Taking off to {HEIGHT:.2f} m")
        commander.take_off(HEIGHT, TAKEOFF_VELOCITY)
        airborne = True

        # Fly from wherever we took off to the start (FORWARD_WAYPOINTS[0] = (1,-2))
        # at TRANSIT_HEIGHT, then descend to the cruise height at the start.
        start_x = float(FORWARD_WAYPOINTS[0, 0])
        start_y = float(FORWARD_WAYPOINTS[0, 1])
        print(f"Flying to start ({start_x:.2f}, {start_y:.2f}) at {TRANSIT_HEIGHT:.2f} m")
        commander.go_to(start_x, start_y, TRANSIT_HEIGHT)
        commander.go_to(start_x, start_y, HEIGHT)

        # Trace the forward curve. PositionHlCommander sizes each leg's duration from
        # the distance and CRUISE_VELOCITY, so the motion stays smooth and continuous.
        print(f"Tracing forward curve through {len(FORWARD_WAYPOINTS)} waypoints")
        for i in range(1, len(FORWARD_WAYPOINTS)):
            x, y, z = (float(v) for v in FORWARD_WAYPOINTS[i])
            commander.go_to(x, y, z)
            time.sleep(WAYPOINT_DELAY)

        # Trace the return leg — it carries its own altitude changes (curve to
        # (0.1,0), raised straight leg down to (0.1,-0.65), then a curve to the END).
        print(f"Tracing return through {len(RETURN_WAYPOINTS)} waypoints")
        for i in range(1, len(RETURN_WAYPOINTS)):
            x, y, z = (float(v) for v in RETURN_WAYPOINTS[i])
            commander.go_to(x, y, z)
            time.sleep(WAYPOINT_DELAY)

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
    print(f"Forward ({len(FORWARD_WAYPOINTS)} pts @ {HEIGHT:.2f} m): "
          f"{tuple(round(v, 2) for v in FORWARD_WAYPOINTS[0])} -> {tuple(round(v, 2) for v in FORWARD_WAYPOINTS[-1])}")
    print(f"Return ({len(RETURN_WAYPOINTS)} pts, raised leg @ {RAISED_HEIGHT:.2f} m): "
          f"{tuple(round(v, 2) for v in RETURN_WAYPOINTS[0])} -> {tuple(round(v, 2) for v in RETURN_WAYPOINTS[-1])}")

    cflib.crtp.init_drivers()

    with SyncCrazyflie(
        URI,
        cf=Crazyflie(rw_cache="./cache"),
    ) as scf:
        run_flight(scf)


if __name__ == "__main__":
    main()
