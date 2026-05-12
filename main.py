from __future__ import annotations

from ros_sugar import Launcher

from irobot.src.projects.pdstl.config import CrazyflieConfig
from irobot.src.projects.pdstl.component.crazyflie import CrazyfliePlanning

my_component = CrazyfliePlanning(
    component_name='crazyflie_planning',
    config=CrazyflieConfig(),
)

launcher = Launcher()
launcher.add_pkg(components=[my_component], activate_all_components_on_start=True)
launcher.bringup()
