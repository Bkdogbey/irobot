from __future__ import annotations

import time

import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.reset_estimator import reset_estimator
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

    def _go_to_origin(self):
        self.position_commander.take_off()
        time.sleep(1.0)
        self.position_commander._cf.commander.send_position_setpoint(0.0, 1.5, 0.5, 0.0)
        time.sleep(1.0)
        self.position_commander.land()

    def _execute_once(self):
        # self._go_to_origin()
        start_0 = 1.5
        self.position_commander.take_off(height=0.2)
        time.sleep(1.0)
        self.position_commander.go_to(0, -start_0, 0.2)
        time.sleep(0.1)

        y_pos = np.linspace(-start_0, 0.65, 10)
        x_pos = 0.5 * np.sin(np.pi * y_pos / start_0)

        for x, y in zip(x_pos, y_pos):
            print('Setting position {} {}'.format(x, y))
            # 1. Send the position setpoint (required for active control)
            self.position_commander.go_to(x, y, 0.2)
            time.sleep(0.1)

        self.position_commander.go_to(x, y, 0.65)
        time.sleep(1.0)
        self.position_commander.go_to(0, -start_0, 0.65)
        time.sleep(1.0)
        self.position_commander.go_to(0, -start_0, 0.1)
        time.sleep(1.0)
        self.position_commander.land()
        # time.sleep(1.0)

        # for x1, y1 in zip(x, y):
        #     self.position_commander.go_to(x1, y1)
        #     time.sleep(0.1)

        # self.position_commander.land()
        # self._go_to_origin()
        # self.position_commander.take_off(height=0.5)
        # time.sleep(1.0)

        # try:
        #     path = build_cr_path(z=0.5, n_points=20)
        #     for x, y, z in path:
        #         self.position_commander.go_to(x, y, z)
        #     time.sleep(5.0)  # hover at goal for 5 s
        # finally:
        #     self.position_commander.land()

    def _execution_step(self):
        pass
