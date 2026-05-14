from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np
import trossen_arm

logger = logging.getLogger(__name__)


@dataclass
class TrossenArmConfig:
    """Configuration for one Trossen WXAI arm."""

    ip_address: str = '192.168.1.2'
    model: trossen_arm.Model = field(default_factory=lambda: trossen_arm.Model.wxai_v0)
    end_effector: trossen_arm.StandardEndEffector = field(
        default_factory=lambda: trossen_arm.StandardEndEffector.wxai_v0_follower
    )
    clear_error_on_connect: bool = True
    gripper_open_pos: float = 0.035   # metres; 0.035 = fully open for WXAI V0
    gripper_close_pos: float = 0.0    # metres; 0.0 = fully closed


class TrossenArmBase:
    """
    Hardware abstraction for one Trossen WXAI arm.

    Wraps the trossen_arm.TrossenArmDriver into a clean Python class with
    connect/disconnect lifecycle, cached state, and convenience methods.
    No ROS2, no ros_sugar, no experiment logic.

    One instance = one physical arm. For multi-arm setups, instantiate one
    TrossenArmBase per arm.
    """

    HOME_POSITIONS = np.array([0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0, 0.0])
    SLEEP_POSITIONS = np.zeros(7)

    def __init__(self, config: TrossenArmConfig) -> None:
        self.config = config
        self.driver = trossen_arm.TrossenArmDriver()
        self.is_connected = False

        # Cached state — updated by get_state()
        self.joint_positions: list[float] = [0.0] * 7
        self.joint_velocities: list[float] = [0.0] * 7
        self.joint_efforts: list[float] = [0.0] * 7
        self.joint_external_efforts: list[float] = [0.0] * 7
        self.ee_pose: list[float] = [0.0] * 6   # [x, y, z, roll, pitch, yaw]
        self.gripper_position: float = 0.0

    # ── Lifecycle ───────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """Configure the driver and establish connection to the arm.

        Returns:
            True on success, False on failure.
        """
        try:
            logger.info('Connecting to Trossen arm at %s', self.config.ip_address)
            self.driver.configure(
                self.config.model,
                self.config.end_effector,
                self.config.ip_address,
                self.config.clear_error_on_connect,
            )
            self.is_connected = True
            logger.info('Connected to Trossen arm at %s', self.config.ip_address)
        except Exception:
            logger.exception('Failed to connect to Trossen arm at %s', self.config.ip_address)
            return False
        return True

    def disconnect(self):
        """Park the arm and release the connection. Safe to call even if not connected."""
        if self.is_connected:
            try:
                self.go_sleep()
            except Exception:
                logger.warning('go_sleep() failed during disconnect — continuing cleanup')
            try:
                self.driver.cleanup()
            except Exception:
                logger.warning('driver.cleanup() failed during disconnect')
            self.is_connected = False
            logger.info('Disconnected from Trossen arm at %s', self.config.ip_address)

    # ── State ───────────────────────────────────────────────────────────────

    def get_state(self):
        """Poll the driver and cache joint and Cartesian state.

        Returns:
            The raw RobotOutput for callers that need full data.
        """
        output = self.driver.get_robot_output()
        self.joint_positions = list(output.joint.all.positions)
        self.joint_velocities = list(output.joint.all.velocities)
        self.joint_efforts = list(output.joint.all.efforts)
        self.joint_external_efforts = list(output.joint.all.external_efforts)
        self.ee_pose = list(output.cartesian.positions)
        self.gripper_position = list(output.joint.gripper.positions)[0]
        return output

    # ── Named poses ─────────────────────────────────────────────────────────

    def go_home(self, duration: float = 3.0):
        """Move all joints to the upright home position."""
        self._require_connected('go_home')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(self.HOME_POSITIONS, duration, True)
        logger.info('Arm at %s moved to home', self.config.ip_address)

    def go_sleep(self, duration: float = 3.0):
        """Move all joints to zeros (parked/sleep position)."""
        self._require_connected('go_sleep')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(self.SLEEP_POSITIONS, duration, True)
        logger.info('Arm at %s moved to sleep', self.config.ip_address)

    # ── Joint commands ──────────────────────────────────────────────────────

    def set_joint_positions(
        self,
        positions,
        duration: float = 1.0,
        blocking: bool = True,
    ):
        """Set all 7 joint positions (includes gripper joint).

        Args:
            positions: 7-element array of joint positions in radians.
            duration: Time to reach the goal in seconds.
            blocking: If True, wait until motion completes.
        """
        self._require_connected('set_joint_positions')
        self.driver.set_all_positions(np.asarray(positions, dtype=float), duration, blocking)

    def set_arm_positions(
        self,
        positions,
        duration: float = 1.0,
        blocking: bool = True,
    ):
        """Set arm joint positions only (6 joints, no gripper).

        Args:
            positions: 6-element array of arm joint positions in radians.
            duration: Time to reach the goal in seconds.
            blocking: If True, wait until motion completes.
        """
        self._require_connected('set_arm_positions')
        self.driver.set_arm_positions(np.asarray(positions, dtype=float), duration, blocking)

    # ── Cartesian commands ──────────────────────────────────────────────────

    def set_cartesian_positions(
        self,
        pose,
        duration: float = 1.0,
        blocking: bool = True,
        interp_space: trossen_arm.InterpolationSpace = trossen_arm.InterpolationSpace.joint,
    ):
        """Set Cartesian end-effector pose.

        Args:
            pose: 6-element array [x, y, z, roll, pitch, yaw] in metres/radians.
            duration: Time to reach the goal in seconds.
            blocking: If True, wait until motion completes.
            interp_space: Interpolation space (joint or cartesian).
        """
        self._require_connected('set_cartesian_positions')
        self.driver.set_cartesian_positions(np.asarray(pose, dtype=float), interp_space, duration, blocking)

    # ── Gripper ─────────────────────────────────────────────────────────────

    def set_gripper(self, position: float, duration: float = 1.0):
        """Set gripper position in metres (0.0 = closed, 0.035 = open for WXAI V0).

        Args:
            position: Gripper opening in metres.
            duration: Time to reach the position in seconds.
        """
        self._require_connected('set_gripper')
        self.driver.set_gripper_position(position, duration, True)

    def open_gripper(self, duration: float = 1.0):
        """Open the gripper to config.gripper_open_pos."""
        self.set_gripper(self.config.gripper_open_pos, duration)

    def close_gripper(self, duration: float = 1.0):
        """Close the gripper to config.gripper_close_pos."""
        self.set_gripper(self.config.gripper_close_pos, duration)

    # ── Mode control ────────────────────────────────────────────────────────

    def set_mode(self, mode: trossen_arm.Mode):
        """Set all joints to the given control mode.

        Args:
            mode: trossen_arm.Mode enum value (position, velocity, effort, external_effort).
        """
        self._require_connected('set_mode')
        self.driver.set_all_modes(mode)

    def gravity_comp(self):
        """Enter gravity compensation mode (external_effort, zero efforts).

        Useful for kinesthetic teaching: the arm floats under its own weight.
        """
        self._require_connected('gravity_comp')
        self.driver.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver.set_all_external_efforts([0.0] * 7, 0.0, False)

    # ── Teleoperation helpers ───────────────────────────────────────────────

    def get_positions(self) -> list[float]:
        """Return current all-joint positions (for leader → follower streaming)."""
        return self.driver.get_all_positions()

    def get_velocities(self) -> list[float]:
        """Return current all-joint velocities (for velocity feedforward)."""
        return self.driver.get_all_velocities()

    def get_external_efforts(self) -> list[float]:
        """Return current all-joint external efforts (for force feedback)."""
        return self.driver.get_all_external_efforts()

    def stream_positions(self, positions, velocities=None):
        """Non-blocking position command for real-time control loops.

        Args:
            positions: 7-element array of target joint positions.
            velocities: Optional 7-element array of velocity feedforward values.
        """
        self._require_connected('stream_positions')
        if velocities is not None:
            self.driver.set_all_positions(np.asarray(positions, dtype=float), 0.0, False, velocities)
        else:
            self.driver.set_all_positions(np.asarray(positions, dtype=float), 0.0, False)

    def stream_external_efforts(self, efforts):
        """Non-blocking effort command for real-time control loops.

        Args:
            efforts: 7-element array of external effort values.
        """
        self._require_connected('stream_external_efforts')
        self.driver.set_all_external_efforts(list(efforts), 0.0, False)

    # ── Safety ──────────────────────────────────────────────────────────────

    def e_stop(self):
        """Emergency stop: immediately cleanup the driver, ignoring all errors."""
        logger.warning('E-STOP called for arm at %s', self.config.ip_address)
        try:
            self.driver.cleanup()
        except Exception:
            pass
        self.is_connected = False

    # ── Internal ────────────────────────────────────────────────────────────

    def _require_connected(self, method: str):
        if not self.is_connected:
            raise RuntimeError(f'TrossenArmBase.{method}: arm at {self.config.ip_address} is not connected')
