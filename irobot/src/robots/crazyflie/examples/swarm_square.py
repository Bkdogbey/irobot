"""
Swarm demo — Synchronised Square

4 Crazyflies each fly a 1m square from their starting position.
All drones execute the same sequence in parallel using the High Level Commander
with relative movements, so they can be placed anywhere on the floor.

Requirements:
  - Crazyradio PA USB dongle
  - Any absolute positioning system (Lighthouse, motion capture, etc.)
  - cflib:  pip install cflib

Run:
  python irobot/src/robots/crazyflie/examples/swarm_square.py
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

URIS = {
    'radio://0/80/2M/E7E7E7E784',  # drone 1
    'radio://0/80/2M/E7E7E7E780',  # drone 2
    'radio://0/80/2M/E7E7E7E782',  # drone 3
    'radio://0/80/2M/E7E7E7E783',  # drone 4
}

# ── Flight parameters ─────────────────────────────────────────────────────────

TAKEOFF_HEIGHT = 1.0  # metres
BOX_SIZE = 1.0        # side length of the square in metres
FLIGHT_TIME = 2.0     # seconds per leg of the square


# ── Per-drone functions ───────────────────────────────────────────────────────
# Each function receives a SyncCrazyflie (scf) as its first argument.
# parallel_safe() calls these on all drones simultaneously.


def _use_pid_controller(scf) -> None:
    scf.cf.param.set_value('stabilizer.controller', '1')


def arm(scf) -> None:
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)


def run_square(scf) -> None:
    """Takeoff, fly a 1m square, land — all movements relative to start."""
    _use_pid_controller(scf)

    commander = scf.cf.high_level_commander

    commander.takeoff(TAKEOFF_HEIGHT, 2.0)
    time.sleep(3.0)

    # Four legs of the square, each relative to the previous position
    for dx, dy in [(BOX_SIZE, 0), (0, BOX_SIZE), (-BOX_SIZE, 0), (0, -BOX_SIZE)]:
        commander.go_to(dx, dy, 0, 0, FLIGHT_TIME, relative=True)
        time.sleep(FLIGHT_TIME + 0.5)

    commander.land(0.0, 2.0)
    time.sleep(2.5)
    commander.stop()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(URIS, factory=factory) as swarm:
        # Wait for all Kalman filters to converge before flying
        swarm.reset_estimators()

        swarm.parallel_safe(arm)
        swarm.parallel_safe(run_square)
