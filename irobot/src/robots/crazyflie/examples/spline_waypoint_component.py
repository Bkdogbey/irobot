from __future__ import annotations

import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from ros_sugar.core import BaseComponent

from irobot.src.robots.crazyflie import SplineWaypointConfig


class SplineWaypointDemo(BaseComponent):
    """Single drone: sine S-curve from (1,-1.75) to (1,0) through (0.5,-0.5).

    Workspace: x[0,1] m, y[0,-2] m.  Height: 0.3 m.

    Waypoints are pre-computed in SplineWaypointConfig from:
      x(t) = 1.0 - 0.64 * sin(π * t)
      y(t) = -1.75 + 1.75 * t

    blend_time causes the next go_to to be issued before the current leg
    expires, so the drone curves smoothly through each point with no dead stops.
    """

    def __init__(self, *, component_name: str, config: SplineWaypointConfig, **kwargs) -> None:
        self.drone_config = config
        super().__init__(component_name=component_name, **kwargs)

    def _execute_once(self) -> None:
        cfg = self.drone_config

        cflib.crtp.init_drivers()
        with SyncCrazyflie(cfg.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            # Reset Kalman position estimator and wait for convergence
            scf.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            scf.cf.param.set_value('kalman.resetEstimation', '0')
            time.sleep(2.0)

            # Arm with PID controller
            scf.cf.param.set_value('stabilizer.controller', '1')
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            commander = scf.cf.high_level_commander
            commander.takeoff(cfg.height, 2.0)
            time.sleep(3.0)

            # Fly sine S-curve waypoints with smooth blending
            for i, (x, y) in enumerate(cfg.waypoints):
                commander.go_to(x, y, cfg.height, 0, cfg.leg_time, relative=False)
                is_last = (i == len(cfg.waypoints) - 1)
                time.sleep(cfg.leg_time if is_last else cfg.leg_time - cfg.blend_time)

            commander.land(0.0, 2.0)
            time.sleep(2.5)
            commander.stop()

    def _execution_step(self) -> None:
        pass
