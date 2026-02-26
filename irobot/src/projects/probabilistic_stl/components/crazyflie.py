from __future__ import annotations

import time

from cflib.positioning.position_hl_commander import PositionHlCommander
from ros_sugar.core import BaseComponent

from irobot.src.projects.probabilistic_stl.components.spline_path import build_cr_path
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

        self.position_commander.take_off(height=0.5)
        time.sleep(1.0)

        try:
            path = build_cr_path(z=0.5, n_points=20)
            for x, y, z in path:
                self.position_commander.go_to(x, y, z)
            time.sleep(5.0)  # hover at goal for 5 s
        finally:
            self.position_commander.land()

    def _execution_step(self):
        pass
