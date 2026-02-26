from __future__ import annotations

import time

from cflib.positioning.position_hl_commander import PositionHlCommander
from ros_sugar.core import BaseComponent

from irobot.src.robots.crazyflie.core.base import CrazyflieBase


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
        self.position_commander.take_off(height=0.4)
        time.sleep(1)
        self.position_commander.go_to(x=-1.0, y=1.0)
        time.sleep(1)
        self.position_commander.land()
        time.sleep(1)

    def _execution_step(self):
        pass
