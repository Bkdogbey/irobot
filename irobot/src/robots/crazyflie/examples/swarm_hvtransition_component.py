from __future__ import annotations

import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from ros_sugar.core import BaseComponent

from irobot.src.robots.crazyflie import SwarmHVConfig


class SwarmHVTransitionDemo(BaseComponent):
    """3 drones: horizontal line → vertical line → horizontal → vertical → land.

    Workspace: x[0,1] m, y[0,-2] m.

    Initial floor placement (horizontal line at y=-0.5):
      D1=(1,-0.5)  D2=(0.5,-0.5)  D3=(0,-0.5)

    Seq 1 — horizontal → vertical (x=0.5):
      Step A (D1+D2 simultaneous):  D1→(0.5,0)  D2→(0.5,-2)
      Step B (D3 after A clears):   D3→(0.5,-0.5)

    Seq 2 — vertical → horizontal (y=-0.5):
      D1+D2 simultaneous:  D1→(0,-0.5)  D2→(1,-0.5)
      D3 hovers at (0.5,-0.5)

    Final — horizontal → vertical (x=0.5):
      Step A (D3 first, clears (0.5,-0.5)):  D3→(0.5,0)
      Step B (D1+D2 simultaneous):           D2→(0.5,-0.5)  D1→(0.5,-2)

    Land all.
    """

    def __init__(self, *, component_name: str, config: SwarmHVConfig, **kwargs) -> None:
        self.swarm_config = config
        super().__init__(component_name=component_name, **kwargs)

    # ── Per-drone helpers ─────────────────────────────────────────────────────

    def _arm(self, scf) -> None:
        scf.cf.param.set_value('stabilizer.controller', '1')
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def _takeoff(self, scf) -> None:
        scf.cf.high_level_commander.takeoff(self.swarm_config.height, 2.0)
        time.sleep(3.0)

    def _move(self, scf, x: float, y: float, z: float) -> None:
        cfg = self.swarm_config
        scf.cf.high_level_commander.go_to(x, y, z, 0, cfg.move_time, relative=False)
        time.sleep(cfg.move_time + 0.3)

    def _land(self, scf) -> None:
        scf.cf.high_level_commander.land(0.0, 2.0)
        time.sleep(2.5)
        scf.cf.high_level_commander.stop()

    # ── Mission ───────────────────────────────────────────────────────────────

    def _execute_once(self) -> None:
        cfg = self.swarm_config
        d1, d2, d3 = cfg.uris
        h = cfg.height

        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(cfg.uris, factory=factory) as swarm:
            swarm.reset_estimators()
            swarm.parallel_safe(self._arm)
            swarm.parallel_safe(self._takeoff)

            # ── Seq 1A: D1 diagonal, D2 down, D3 holds ───────────────────────
            swarm.parallel_safe(self._move, args_dict={
                d1: [0.5,  0.0, h],
                d2: [0.5, -2.0, h],
                d3: [0.0, -0.5, h],   # hold
            })
            # ── Seq 1B: D3 slides to D2's vacated spot; D1/D2 hold ───────────
            swarm.parallel_safe(self._move, args_dict={
                d1: [0.5,  0.0, h],   # hold
                d2: [0.5, -2.0, h],   # hold
                d3: [0.5, -0.5, h],
            })
            # Vertical line: D1=(0.5,0)  D3=(0.5,-0.5)  D2=(0.5,-2)
            time.sleep(cfg.dwell_time)

            # ── Seq 2: D1+D2 swing to y=-0.5, D3 holds ───────────────────────
            swarm.parallel_safe(self._move, args_dict={
                d1: [0.0, -0.5, h],
                d2: [1.0, -0.5, h],
                d3: [0.5, -0.5, h],   # hold
            })
            # Horizontal line: D1=(0,-0.5)  D3=(0.5,-0.5)  D2=(1,-0.5)
            time.sleep(cfg.dwell_time)

            # ── Final A: D3 clears (0.5,-0.5); D1/D2 hold ────────────────────
            swarm.parallel_safe(self._move, args_dict={
                d1: [0.0, -0.5, h],   # hold
                d2: [1.0, -0.5, h],   # hold
                d3: [0.5,  0.0, h],
            })
            # ── Final B: D1 drops, D2 takes centre; D3 holds ─────────────────
            swarm.parallel_safe(self._move, args_dict={
                d1: [0.5, -2.0, h],
                d2: [0.5, -0.5, h],
                d3: [0.5,  0.0, h],   # hold
            })
            # Vertical line: D3=(0.5,0)  D2=(0.5,-0.5)  D1=(0.5,-2)
            time.sleep(cfg.dwell_time)

            swarm.parallel_safe(self._land)

    def _execution_step(self) -> None:
        pass
