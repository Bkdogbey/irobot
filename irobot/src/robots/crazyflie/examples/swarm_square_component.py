"""
Swarm demo — Synchronised Square (ROS component wrapper)

Wraps the swarm-square mission as a ros_sugar BaseComponent so it can be
launched from main.py alongside other components.

Each drone flies a 1 m square from its own starting position using the
High Level Commander with relative movements. All drones execute in parallel.
"""

from __future__ import annotations

import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from ros_sugar.core import BaseComponent

from irobot.src.robots.crazyflie import SwarmSquareConfig


class SwarmSquareDemo(BaseComponent):
    def __init__(self, *, component_name: str, config: SwarmSquareConfig, **kwargs) -> None:
        self.swarm_config = config
        super().__init__(component_name=component_name, **kwargs)

    # ── Per-drone helpers (called by Swarm.parallel_safe) ─────────────────────

    def _use_pid_controller(self, scf) -> None:
        scf.cf.param.set_value('stabilizer.controller', '1')

    def _arm(self, scf) -> None:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def _run_square(self, scf) -> None:
        self._use_pid_controller(scf)
        cfg = self.swarm_config
        commander = scf.cf.high_level_commander

        commander.takeoff(cfg.takeoff_height, 2.0)
        time.sleep(3.0)

        for dx, dy in [
            (cfg.box_size, 0),
            (0, cfg.box_size),
            (-cfg.box_size, 0),
            (0, -cfg.box_size),
        ]:
            commander.go_to(dx, dy, 0, 0, cfg.flight_time, relative=True)
            time.sleep(cfg.flight_time + 0.5)

        commander.land(0.0, 2.0)
        time.sleep(2.5)
        commander.stop()

    # ── Component lifecycle ───────────────────────────────────────────────────

    def _execute_once(self) -> None:
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(self.swarm_config.uris, factory=factory) as swarm:
            swarm.reset_estimators()
            swarm.parallel_safe(self._arm)
            swarm.parallel_safe(self._run_square)

    def _execution_step(self) -> None:
        pass
