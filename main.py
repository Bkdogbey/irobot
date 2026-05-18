from __future__ import annotations

from ros_sugar import Launcher

from irobot.src.robots.crazyflie.examples.crazyflie_ros_component import CrazyflieDemo
from irobot.src.robots.crazyflie import CrazyflieConfig

component = CrazyflieDemo(
    component_name='crazyflie_demo',
    config=CrazyflieConfig(),
)

launcher = Launcher()
launcher.add_pkg(components=[component], activate_all_components_on_start=True)
launcher.bringup()
