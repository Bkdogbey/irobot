from __future__ import annotations

import time

import numpy as np
from cflib.positioning.position_hl_commander import PositionHlCommander
from ros_sugar.core import BaseComponent

from irobot.src.projects.pdstl.component.flight_logger import FlightLogger
from irobot.src.projects.pdstl.component.opt_waypoints import WAYPOINTS
from irobot.src.projects.pdstl.config import CONDITION, FAN_SPEED, USE_OPTIMISED
from irobot.src.robots.crazyflie.core.base import CrazyflieBase


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

    def _execute_once(self):
        start_0 = 1.5
        logger = FlightLogger(CONDITION, fan_speed=FAN_SPEED)

        logger.start()
        logger.start_actual_logging(
            lambda: (self.crazyflie.current_x, self.crazyflie.current_y, self.crazyflie.current_z)
        )
        waypoints = WAYPOINTS if USE_OPTIMISED else _sine_waypoints()
        try:
            for x, y, z in waypoints:
                self.position_commander.go_to(x, y, z)
                logger.log_waypoint(x, y, z)
                time.sleep(0.1)

            self.position_commander.go_to(x, y, 0.65)
            time.sleep(1.0)
            self.position_commander.go_to(0, -start_0, 0.65)
            time.sleep(1.0)
            self.position_commander.go_to(0, -start_0, 0.1)
            time.sleep(1.0)
            self.position_commander._hl_commander.stop()
            time.sleep(1.0)

        except Exception as exc:
            logger.mark_crashed()
            raise
        finally:
            logger.stop_actual_logging()
            logger.save()
            self.position_commander.land()

    def _execution_step(self):
        pass
