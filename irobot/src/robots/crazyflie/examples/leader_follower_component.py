from __future__ import annotations

import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from ros_sugar.core import BaseComponent

from irobot.src.robots.crazyflie import LeaderFollowerConfig


class LeaderFollowerDemo(BaseComponent):
    """2 drones: leader traces an S-curve, follower mirrors it 0.35 m north.

    Workspace: x[0,1] m, y[0,-2] m.  Height: 0.3 m.

    Leader S-curve waypoints:
      (0.2,-0.5) → (0.8,-0.5) → (0.8,-1.0) → (0.2,-1.0)
      → (0.2,-1.5) → (0.8,-1.5) → (0.5,-1.0) [land]

    Follower is always follower_offset_y metres north (+y) of the leader,
    maintaining ≥0.35 m separation throughout.

    Smooth flight via blend_time: each go_to is issued blend_time seconds before
    the previous one expires, so the high_level_commander curves smoothly into
    the next waypoint instead of decelerating to a full stop first.
    """

    def __init__(self, *, component_name: str, config: LeaderFollowerConfig, **kwargs) -> None:
        self.swarm_config = config
        super().__init__(component_name=component_name, **kwargs)

    def _build_follower_waypoints(self) -> list[tuple[float, float]]:
        cfg = self.swarm_config
        return [
            (x, y + cfg.follower_offset_y)
            for x, y in cfg.leader_waypoints
        ]

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
        scf.cf.high_level_commander.go_to(x, y, z, 0, cfg.leg_time, relative=False)
        # Sleep less than leg_time so the next go_to is issued while the drone is
        # still moving — the planner curves smoothly into the new target.
        time.sleep(cfg.leg_time - cfg.blend_time)

    def _land(self, scf) -> None:
        scf.cf.high_level_commander.land(0.0, 2.0)
        time.sleep(2.5)
        scf.cf.high_level_commander.stop()

    # ── Mission ───────────────────────────────────────────────────────────────

    def _execute_once(self) -> None:
        cfg = self.swarm_config
        h = cfg.height
        follower_wps = self._build_follower_waypoints()

        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        with Swarm([cfg.leader_uri, cfg.follower_uri], factory=factory) as swarm:
            swarm.reset_estimators()
            swarm.parallel_safe(self._arm)
            swarm.parallel_safe(self._takeoff)

            for (lx, ly), (fx, fy) in zip(cfg.leader_waypoints, follower_wps):
                swarm.parallel_safe(self._move, args_dict={
                    cfg.leader_uri:   [lx, ly, h],
                    cfg.follower_uri: [fx, fy, h],
                })

            swarm.parallel_safe(self._land)

    def _execution_step(self) -> None:
        pass
