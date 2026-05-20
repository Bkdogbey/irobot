"""
Swarm demo — Formation Flying

4 Crazyflies fly in a square formation using absolute positions.
The sequence demonstrates coordinated multi-drone movement:

  1. All 4 drones take off and fly to the corners of a 1m square.
  2. All converge to the centre simultaneously.
  3. Each drone rotates one position clockwise around the square.
  4. All return to their original corners.
  5. All land.

Requirements:
  - Crazyradio PA USB dongle
  - Lighthouse or other absolute positioning system (calibrated and running)
  - cflib:  pip install cflib

Setup:
  Place each drone near its designated corner before running. The corner
  assignments are printed when the script starts. URIs and positions are
  configured in the URIS / POSITIONS constants below.

Run:
  python irobot/src/robots/crazyflie/examples/swarm_formation.py
"""
from __future__ import annotations

import logging
import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

logging.basicConfig(level=logging.ERROR)

# ── Hardware configuration ────────────────────────────────────────────────────
# Edit these URIs to match your Crazyflies.
# All URIs in a swarm must share the same radio channel (80 here).

URI1 = 'radio://0/80/2M/E7E7E7E7E1'  # front-left
URI2 = 'radio://0/80/2M/E7E7E7E7E2'  # front-right
URI3 = 'radio://0/80/2M/E7E7E7E7E3'  # back-right
URI4 = 'radio://0/80/2M/E7E7E7E7E4'  # back-left

URIS = {URI1, URI2, URI3, URI4}

# Corner positions (x, y) in metres, centred on the origin.
# Layout (top view):
#
#   URI1 (-0.5, +0.5)      URI2 (+0.5, +0.5)
#          ●                      ●
#
#          ●                      ●
#   URI4 (-0.5, -0.5)      URI3 (+0.5, -0.5)
#
POSITIONS: dict[str, tuple[float, float]] = {
    URI1: (-0.5,  0.5),
    URI2: ( 0.5,  0.5),
    URI3: ( 0.5, -0.5),
    URI4: (-0.5, -0.5),
}

# Clockwise rotation: each drone moves to the next drone's corner.
ROTATED: dict[str, tuple[float, float]] = {
    URI1: POSITIONS[URI2],
    URI2: POSITIONS[URI3],
    URI3: POSITIONS[URI4],
    URI4: POSITIONS[URI1],
}

# ── Flight parameters ─────────────────────────────────────────────────────────

HOVER_HEIGHT = 0.5   # metres — keep low for indoor safety
TRAVEL_TIME = 3.0    # seconds per move


# ── Per-drone functions ───────────────────────────────────────────────────────


def _use_pid_controller(scf) -> None:
    scf.cf.param.set_value('stabilizer.controller', '1')


def arm(scf) -> None:
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)


def takeoff(scf) -> None:
    _use_pid_controller(scf)
    scf.cf.high_level_commander.takeoff(HOVER_HEIGHT, 2.0)
    time.sleep(3.0)


def fly_to(scf, x: float, y: float) -> None:
    """Fly to absolute position (x, y) at HOVER_HEIGHT."""
    scf.cf.high_level_commander.go_to(x, y, HOVER_HEIGHT, 0, TRAVEL_TIME, relative=False)
    time.sleep(TRAVEL_TIME + 0.5)


def converge(scf) -> None:
    """All drones fly to the centre of the formation."""
    fly_to(scf, 0.0, 0.0)


def land(scf) -> None:
    scf.cf.high_level_commander.land(0.0, 2.0)
    time.sleep(2.5)
    scf.cf.high_level_commander.stop()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Corner assignments:')
    for uri, (x, y) in POSITIONS.items():
        print(f'  {uri}  →  ({x:+.1f}, {y:+.1f})')
    print()

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # Build per-drone argument dicts for the two position phases
    corner_args = {uri: list(pos) for uri, pos in POSITIONS.items()}
    rotated_args = {uri: list(pos) for uri, pos in ROTATED.items()}

    with Swarm(URIS, factory=factory) as swarm:
        # Wait for all Kalman filters to converge before flying
        swarm.reset_estimators()

        swarm.parallel_safe(arm)
        swarm.parallel_safe(takeoff)

        # Phase 1: spread to corners
        swarm.parallel_safe(fly_to, args_dict=corner_args)

        # Phase 2: converge to centre
        swarm.parallel_safe(converge)
        time.sleep(2.0)

        # Phase 3: rotate formation clockwise
        swarm.parallel_safe(fly_to, args_dict=rotated_args)
        time.sleep(2.0)

        # Phase 4: return to original corners
        swarm.parallel_safe(fly_to, args_dict=corner_args)

        # Phase 5: land
        swarm.parallel_safe(land)
