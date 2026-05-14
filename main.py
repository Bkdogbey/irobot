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

# ── Trossen Arm template (uncomment to use instead of the Crazyflie above) ───
# from irobot.src.projects.trossen_arm_template.config import TrossenArmComponentConfig
# from irobot.src.projects.trossen_arm_template.component.arm_component import TrossenArmTemplate
#
# arm = TrossenArmTemplate(
#     component_name='trossen_arm_template',
#     config=TrossenArmComponentConfig(),
# )
# launcher = Launcher()
# launcher.add_pkg(components=[arm], activate_all_components_on_start=True)
# launcher.bringup()
