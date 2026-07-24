"""Diagnose why the Crazyflie has no stable position estimate.

Run this (drone sitting still at a known spot, radio + positioning powered) BEFORE
attempting a positioned flight. It reports: which decks are detected, the active
estimator/controller, Lighthouse reception, and whether stateEstimate converges
when the drone is stationary.

    python check_positioning.py            # from the repo root

Override the drone with:  CFLIB_URI=radio://0/85/2M/E7E7E7E85 python check_positioning.py
"""

from __future__ import annotations

import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default="radio://0/85/2M/E7E7E7E85")

EST_NAMES = {"1": "complementary", "2": "kalman (EKF)", "3": "ukf", "4": "out-of-tree"}
CTRL_NAMES = {"1": "PID", "2": "mellinger", "3": "INDI", "4": "brescianini"}


def _rng(values: list[float]) -> float:
    return max(values) - min(values) if values else float("nan")


def main() -> None:
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf

        # --- decks that report themselves present ---------------------------
        print("== Decks detected ==")
        deck_group = cf.param.toc.toc.get("deck", {})
        found = []
        for name in sorted(deck_group):
            try:
                val = cf.param.get_value(f"deck.{name}")
            except Exception:
                continue
            if str(val) not in ("0", "0.0", "None"):
                print(f"  deck.{name} = {val}")
                found.append(name)
        if not found:
            print("  (no deck reports present — a Lighthouse/Flow/LPS deck is required for x/y)")

        # --- estimator + controller ----------------------------------------
        est = str(cf.param.get_value("stabilizer.estimator"))
        ctrl = str(cf.param.get_value("stabilizer.controller"))
        print(f"\nstabilizer.estimator  = {est}  ({EST_NAMES.get(est, '?')})")
        print(f"stabilizer.controller = {ctrl}  ({CTRL_NAMES.get(ctrl, '?')})")
        if est != "2":
            print("  !! Absolute x/y position needs the Kalman estimator (2).")
            print("     The complementary estimator cannot hold an absolute position.")

        # --- Lighthouse reception, if the deck driver is loaded ------------
        lh = cf.log.toc.toc.get("lighthouse", {})
        if lh:
            names = [n for n in ("bsActive", "bsReceive", "bsAvailable", "status") if n in lh]
            if names:
                print("\n== Lighthouse status (2 s) — bsActive/bsReceive are per-base-station bitmasks ==")
                lc = LogConfig(name="lh", period_in_ms=200)
                for n in names:
                    lc.add_variable(f"lighthouse.{n}")
                with SyncLogger(scf, lc) as logger:
                    for i, (_, data, _) in enumerate(logger):
                        print("  ", {k: data[k] for k in data})
                        if i >= 9:
                            break
        else:
            print("\n(No 'lighthouse' log group — Lighthouse deck driver is not active.)")

        # --- does the estimate converge when the drone is still? -----------
        print("\n== Reset Kalman, watch estimate for 8 s (keep the drone perfectly still) ==")
        cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        cf.param.set_value("kalman.resetEstimation", "0")

        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        lc = LogConfig(name="pos", period_in_ms=200)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
            lc.add_variable(v, "float")
        with SyncLogger(scf, lc) as logger:
            t0 = time.time()
            for _, data, _ in logger:
                xs.append(data["stateEstimate.x"])
                ys.append(data["stateEstimate.y"])
                zs.append(data["stateEstimate.z"])
                if time.time() - t0 > 8.0:
                    break

        print(f"  final estimate     : x={xs[-1]:+.3f} y={ys[-1]:+.3f} z={zs[-1]:+.3f}")
        print(f"  spread (full 8 s)  : x={_rng(xs):.3f} y={_rng(ys):.3f} z={_rng(zs):.3f}")
        print(f"  spread (last 10)   : x={_rng(xs[-10:]):.3f} y={_rng(ys[-10:]):.3f} z={_rng(zs[-10:]):.3f}")
        stable = max(_rng(xs[-10:]), _rng(ys[-10:]), _rng(zs[-10:])) < 0.05
        print("\n  ==> " + ("Estimate is STABLE — positioning looks usable."
                            if stable else
                            "Estimate is NOT stable — no usable position lock. "
                            "Check the deck seating, base-station power/visibility, and geometry calibration."))


if __name__ == "__main__":
    main()
