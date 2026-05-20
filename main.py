from __future__ import annotations

from ros_sugar import Launcher

# ── Crazyflie ─────────────────────────────────────────────────────────────────
from irobot.src.robots.crazyflie.examples.crazyflie_ros_component import CrazyflieDemo
from irobot.src.robots.crazyflie import CrazyflieConfig

component = CrazyflieDemo(
    component_name='crazyflie_demo',
    config=CrazyflieConfig(),
)

# ── Trossen Arm ───────────────────────────────────────────────────────────────
# from irobot.src.robots.trossen_arm.examples.trossen_ros_component import TrossenArmDemo
# from irobot.src.robots.trossen_arm.examples.trossen_ros_component import TrossenArmDemoConfig
# from irobot.src.robots.trossen_arm.config import FOLLOWER_1
#
# component = TrossenArmDemo(
#     component_name='trossen_arm_demo',
#     config=TrossenArmDemoConfig(arm_config=FOLLOWER_1),
# )

launcher = Launcher()
launcher.add_pkg(components=[component], activate_all_components_on_start=True)
launcher.bringup()
