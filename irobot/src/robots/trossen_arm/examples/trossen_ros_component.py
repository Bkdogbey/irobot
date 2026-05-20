"""
Example: Trossen arm ROS component using ros_sugar.

Shows how to build a BaseComponent that wraps TrossenArmController
and runs a simple manipulation sequence. Copy this into your own project
and extend _execute_once with your mission logic.
"""

from __future__ import annotations

from attrs import define
from ros_sugar.config import BaseComponentConfig
from ros_sugar.core import BaseComponent

from irobot.src.robots.trossen_arm import TrossenArmConfig, TrossenArmController
from irobot.src.robots.trossen_arm.config import FOLLOWER_1


@define(kw_only=True)
class TrossenArmDemoConfig(BaseComponentConfig):
    arm_config: TrossenArmConfig = FOLLOWER_1


class TrossenArmDemo(BaseComponent):
    def __init__(
        self,
        *,
        component_name: str,
        config: TrossenArmDemoConfig,
        **kwargs,
    ) -> None:
        self.arm = TrossenArmController(config.arm_config)
        super().__init__(component_name=component_name, config=config, **kwargs)

    def _execute_once(self) -> None:
        try:
            self.arm.go_home()

            # Read current end-effector pose
            self.arm._poll_state()
            print(f'EE pose after home: {self.arm.ee_pose}')

            # Open and close the gripper
            self.arm.open_gripper()
            self.arm.close_gripper()

            # Move to a Cartesian pose [x, y, z, roll, pitch, yaw]
            self.arm.set_cartesian_positions([0.3, 0.0, 0.2, 0.0, 0.0, 0.0], duration=2.0)

        finally:
            self.arm.disconnect()   # parks to sleep then cleans up

    def _execution_step(self) -> None:
        pass
