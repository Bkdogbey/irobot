from __future__ import annotations

from ros_sugar.core import BaseComponent

from irobot.src.robots.trossen_arm.core.base import TrossenArmBase
from irobot.src.projects.trossen_arm_template.config import TrossenArmComponentConfig


class TrossenArmTemplate(BaseComponent):
    """
    Minimal template showing how to use TrossenArmBase inside a ros_sugar component.

    Copy this project folder and modify _execute_once() with your experiment sequence.
    Leave _execution_step() empty unless you need a continuous control loop.
    """

    def __init__(self, *, component_name: str, config: TrossenArmComponentConfig, **kwargs):
        self.arm = TrossenArmBase(config.arm_config)
        super().__init__(
            component_name=component_name,
            config=config,
            **kwargs,
        )

    def _execute_once(self):
        """
        Single-shot task example. Override with your experiment sequence.

        Pattern: connect → home → experiment logic → sleep → disconnect.
        The finally block ensures the arm parks safely even on exception.
        """
        try:
            self.arm.connect()
            self.arm.go_home()

            # ── Your experiment logic goes here ──────────────────────────────
            # Example: read current end-effector pose
            self.arm.get_state()
            print(f'EE pose: {self.arm.ee_pose}')

            # Example: open and close gripper
            # self.arm.open_gripper()
            # self.arm.close_gripper()

            # Example: move to a Cartesian pose [x, y, z, roll, pitch, yaw]
            # self.arm.set_cartesian_positions([0.3, 0.0, 0.2, 0.0, 0.0, 0.0], duration=2.0)
            # ────────────────────────────────────────────────────────────────

        except Exception as exc:
            print(f'Error during execution: {exc}')
            raise
        finally:
            self.arm.disconnect()   # goes to sleep then cleans up

    def _execution_step(self):
        """Continuous loop body (called at loop_rate Hz). Leave empty for single-shot tasks."""
        pass
