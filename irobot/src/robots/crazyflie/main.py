from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[4]))  # → repo root (irobot/)

from ros_sugar import Launcher

from irobot.src.projects.probabilistic_stl.components.config import CrazyflieConfig
from irobot.src.projects.probabilistic_stl.components.crazyflie import CrazyfliePlanning

my_component = CrazyfliePlanning(
    component_name='crazyflie_planning',
    config=CrazyflieConfig(z_hold=0.5),
)

launcher = Launcher()
launcher.add_pkg(components=[my_component], activate_all_components_on_start=True)
launcher.bringup()
