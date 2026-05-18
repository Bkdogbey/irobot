"""
Example: Crazyflie ROS component using ros_sugar.

Shows how to build a BaseComponent that wraps CrazyflieController
and flies a simple path. Copy this into your own project and extend
_execute_once with your mission logic.
"""

from __future__ import annotations

from cflib.positioning.position_hl_commander import PositionHlCommander
from ros_sugar.core import BaseComponent

from irobot.src.robots.crazyflie import CrazyflieConfig, CrazyflieController


class CrazyflieDemo(BaseComponent):
    def __init__(self, *, component_name: str, config: CrazyflieConfig, **kwargs) -> None:
        self.robot = CrazyflieController(config)
        super().__init__(component_name=component_name, config=config, **kwargs)
        self.commander = PositionHlCommander(self.robot.cf)

    def _execute_once(self) -> None:
        waypoints = [
            (0.5, 0.0, 0.3),
            (0.5, 0.5, 0.3),
            (0.0, 0.5, 0.3),
            (0.0, 0.0, 0.3),
        ]
        for x, y, z in waypoints:
            self.commander.go_to(x, y, z)
        self.commander.land()

    def _execution_step(self) -> None:
        pass
