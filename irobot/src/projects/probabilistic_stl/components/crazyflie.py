from __future__ import annotations

import time

import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.reset_estimator import reset_estimator
from ros_sugar.core import BaseComponent

from irobot.src.projects.probabilistic_stl.components.flight_logger import FlightLogger
from irobot.src.projects.probabilistic_stl.components.opt_waypoints import WAYPOINTS
from irobot.src.projects.probabilistic_stl.components.spline_path import build_cr_path
from irobot.src.robots.crazyflie.core.base import CrazyflieBase

# ── Trial configuration  <-- edit these two lines before each run ──────────
# Path selector: True → pDSTL-optimised path, False → original sine path
USE_OPTIMISED = False
# Condition label: 'deterministic' (no optimisation) or 'pdstl' (optimised)
CONDITION = 'deterministic'
# Fan speed integer: 0 = off (nominal), 6 / 12 / 18 for wind levels
FAN_SPEED = 18
#
# Quick reference:
#   Condition      USE_OPTIMISED   CONDITION          FAN_SPEED
#   Deterministic  False           'deterministic'    0 / 6 / 12 / 18
#   pDSTL          True            'pdstl'            0 / 6 / 12 / 18


def _sine_waypoints() -> list[tuple[float, float, float]]:
    start_0 = 1.5
    y_pos = np.linspace(-start_0, 0.65, 10)
    x_pos = 0.5 * np.sin(np.pi * y_pos / start_0)
    return [(float(x), float(y), 0.2) for x, y in zip(x_pos, y_pos)]


class CrazyfliePlanning(BaseComponent):
    def __init__(self, *, component_name, config, **kwargs):
        self.crazyflie = CrazyflieBase(config)

        super().__init__(
            component_name=component_name,
            config=config,
            **kwargs,
        )
        self.position_commander = PositionHlCommander(self.crazyflie.cf)

    def _go_to_origin(self):
        self.position_commander.take_off()
        time.sleep(1.0)
        self.position_commander._cf.commander.send_position_setpoint(0.0, 1.5, 0.5, 0.0)
        time.sleep(1.0)
        self.position_commander.land()

    def _execute_once(self):
        # self._go_to_origin()
        start_0 = 1.5
        logger = FlightLogger(CONDITION, fan_speed=FAN_SPEED)

        self.position_commander.take_off(height=0.2)
        time.sleep(1.0)
        self.position_commander.go_to(0, -start_0, 0.2)
        time.sleep(0.1)

        logger.start()
        logger.start_actual_logging(
            lambda: (self.crazyflie.current_x, self.crazyflie.current_y, self.crazyflie.current_z)
        )
        waypoints = WAYPOINTS if USE_OPTIMISED else _sine_waypoints()
        try:
            for x, y, z in waypoints:
                print('Setting position {} {}'.format(x, y))
                self.position_commander.go_to(x, y, z)
                logger.log_waypoint(x, y, z)
                time.sleep(0.1)

            self.position_commander.go_to(x, y, 0.65)
            time.sleep(1.0)
            self.position_commander.go_to(0, -start_0, 0.65)
            time.sleep(1.0)
            self.position_commander.go_to(0, -start_0, 0.1)
            time.sleep(1.0)
        except Exception as exc:
            print(f'[CrazyfliePlanning] Exception during flight: {exc}')
            logger.mark_crashed()
            raise
        finally:
            logger.stop_actual_logging()
            logger.save()
            self.position_commander.land()

    def _execution_step(self):
        pass
