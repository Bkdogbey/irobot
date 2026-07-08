from __future__ import annotations

import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from ros_sugar.core import BaseComponent

from irobot.src.robots.crazyflie import SwarmTransitionConfig


class SwarmTransitionDemo(BaseComponent):
    """4 drones: vertical line → square → line → land. All within ±0.5 m.

    URI order: top, upper-mid, lower-mid, bottom (of the vertical line).

    Collision-free 2-step transition (line → square):
      Step A — x-adjust only: rows stay at their line y-values (≥ 0.3 m apart).
      Step B — y-adjust only: columns move to square y-values; adjacent columns
               move in opposite directions so separation only increases.
    Square → line is the exact reverse (Step B, then Step A).
    """

    def __init__(self, *, component_name: str, config: SwarmTransitionConfig, **kwargs) -> None:
        self.swarm_config = config
        super().__init__(component_name=component_name, **kwargs)

    def _compute_positions(self) -> dict[str, dict[str, tuple[float, float, float]]]:
        cfg = self.swarm_config
        d0, d1, d2, d3 = cfg.uris
        h = cfg.height
        s = cfg.square_half
        sp = cfg.line_spacing

        # Vertical line: 4 drones at x=0, evenly spaced along y
        # y values: +1.5*sp, +0.5*sp, -0.5*sp, -1.5*sp
        y0, y1, y2, y3 = 1.5 * sp, 0.5 * sp, -0.5 * sp, -1.5 * sp

        line = {
            d0: (0.0,  y0, h),
            d1: (0.0,  y1, h),
            d2: (0.0,  y2, h),
            d3: (0.0,  y3, h),
        }
        # Step A: x-adjust to square x-columns, keep line y-rows
        step_a = {
            d0: (-s,  y0, h),   # front-left column, top row
            d1: ( s,  y1, h),   # front-right column, upper-mid row
            d2: ( s,  y2, h),   # back-right column, lower-mid row
            d3: (-s,  y3, h),   # back-left column, bottom row
        }
        # Step B: y-adjust to square y-values
        square = {
            d0: (-s,  s, h),
            d1: ( s,  s, h),
            d2: ( s, -s, h),
            d3: (-s, -s, h),
        }
        return {'line': line, 'step_a': step_a, 'square': square}

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

    def _execute_once(self) -> None:
        cfg = self.swarm_config
        pos = self._compute_positions()

        def args(stage: str) -> dict[str, list[float]]:
            return {uri: list(xyz) for uri, xyz in pos[stage].items()}

        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm(cfg.uris, factory=factory) as swarm:
            swarm.reset_estimators()
            swarm.parallel_safe(self._arm)
            swarm.parallel_safe(self._takeoff)

            swarm.parallel_safe(self._move, args_dict=args('line'))
            time.sleep(cfg.dwell_time)

            swarm.parallel_safe(self._move, args_dict=args('step_a'))   # x-adjust
            swarm.parallel_safe(self._move, args_dict=args('square'))   # y-adjust → square
            time.sleep(cfg.dwell_time)

            swarm.parallel_safe(self._move, args_dict=args('step_a'))   # y-restore
            swarm.parallel_safe(self._move, args_dict=args('line'))     # x-restore → line
            time.sleep(cfg.dwell_time)

            swarm.parallel_safe(self._land)

    def _execution_step(self) -> None:
        pass
