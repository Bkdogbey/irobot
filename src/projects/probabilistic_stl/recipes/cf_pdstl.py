"""
cf_pdstl.py — Crazyflie + pdSTL Launcher
=========================================
Two-component setup:
    CrazyflieComponent  — hardware driver, publishes /cf/pose, subscribes /cf/cmd
    PdSTLComponent      — STL planner,    subscribes /cf/pose, publishes /cf/cmd

Formula
-------
    φ = G[0,30](z > 0.2)                        always stay above 20 cm
      ∧ F[0,10](‖pos_xy − goal_xy‖ < GOAL_R)   reach XY goal within 10 s

The controller gradient-ascends the robustness surface until φ is satisfied,
then holds position (zero velocity command).

Usage
-----
    cd src
    python -m projects.probabilistic_stl.recipes.cf_pdstl
"""

from __future__ import annotations

import math
import os
import sys

# Allow running from repo root or from src/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

from ros_sugar import Launcher

from robots.crazyflie.cf_node import CrazyflieComponent
from projects.probabilistic_stl.formula import (
    Always,
    And,
    Eventually,
    Predicate,
)
from projects.probabilistic_stl.pdstl_node import PdSTLComponent


# ── Mission parameters ────────────────────────────────────────────────────────

URI     = "radio://0/80/2M/E7E7E7E7E7"

GOAL_X  = 1.0    # target x, metres (Lighthouse frame)
GOAL_Y  = 0.0    # target y, metres
HOVER_Z = 0.5    # cruise / takeoff altitude, metres
GOAL_R  = 0.15   # acceptance radius in XY, metres

SAFE_Z_MIN   = 0.2    # G constraint lower bound, metres
ALWAYS_WIN   = 30.0   # G window width, seconds
FINALLY_WIN  = 10.0   # F window width, seconds


# ── Formula factory ───────────────────────────────────────────────────────────

def make_formula():
    """
    Build the pdSTL specification for this mission.

    φ = G[0,30](z > 0.2)  ∧  F[0,10](‖pos_xy − goal‖ < GOAL_R)

    To extend (e.g. add a second waypoint or velocity constraint):
        phi2 = Eventually(Predicate(...), 0, 20)
        return And(safe_floor, phi2, ...)
    """
    safe_floor = Predicate(
        lambda x, y, z: z - SAFE_Z_MIN,
        name=f"z>{SAFE_Z_MIN}m",
    )
    at_goal_xy = Predicate(
        lambda x, y, z: GOAL_R - math.hypot(x - GOAL_X, y - GOAL_Y),
        name=f"goal_xy(r={GOAL_R}m)",
    )
    return And(
        Always(safe_floor,  a=0, b=ALWAYS_WIN),
        Eventually(at_goal_xy, a=0, b=FINALLY_WIN),
    )


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    print("=== Crazyflie + pdSTL Planner ===")
    print(f"    Goal: ({GOAL_X}, {GOAL_Y}) ± {GOAL_R} m  |  z > {SAFE_Z_MIN} m")

    drone = CrazyflieComponent(
        uri=URI,
        hover_z=HOVER_Z,
        component_name="crazyflie",
    )
    planner = PdSTLComponent(
        formula=make_formula(),
        k_p=0.5,
        v_max=0.3,
        margin=0.05,
        component_name="pdstl_planner",
    )

    launcher = Launcher()
    launcher.add_pkg(components=[drone, planner])
    launcher.bringup()


if __name__ == "__main__":
    main()
