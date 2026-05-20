"""Trossen arm motion control and manipulation primitives."""

from __future__ import annotations

import logging
import math

import numpy as np
import trossen_arm

from irobot.src.robots.trossen_arm.core.base import TrossenArmBase

logger = logging.getLogger(__name__)


class TrossenArmController(TrossenArmBase):
    """Motion control built on top of TrossenArmBase."""

    HOME_POSITIONS: list[float] = [0.0, math.pi / 2, math.pi / 2, 0.0, 0.0, 0.0, 0.0]
    SLEEP_POSITIONS: list[float] = [0.0] * 7

    # ── Named poses ───────────────────────────────────────────────────────────

    def go_home(self, duration: float = 3.0, blocking: bool = True) -> None:
        """Move all joints to the upright home position."""
        self._require_connected('go_home')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(np.asarray(self.HOME_POSITIONS), duration, blocking)
        logger.info('Arm at %s moved to home', self.config.ip_address)

    def go_sleep(self, duration: float = 3.0, blocking: bool = True) -> None:
        """Move all joints to the parked (zeros) position."""
        self._require_connected('go_sleep')
        self.driver.set_all_modes(trossen_arm.Mode.position)
        self.driver.set_all_positions(np.asarray(self.SLEEP_POSITIONS), duration, blocking)
        logger.info('Arm at %s moved to sleep', self.config.ip_address)

    def disconnect(self) -> None:
        """Park the arm to sleep position, then release the connection."""
        if self.is_connected:
            try:
                self.go_sleep()
            except Exception:
                logger.warning('go_sleep() failed during disconnect — continuing cleanup')
        super().disconnect()

    # ── Mode control ──────────────────────────────────────────────────────────

    def set_mode(self, mode: trossen_arm.Mode) -> None:
        """Set all joints to the given control mode."""
        self._require_connected('set_mode')
        self.driver.set_all_modes(mode)

    # ── Joint commands ────────────────────────────────────────────────────────

    def set_joint_positions(
        self,
        positions: list[float] | np.ndarray,
        duration: float = 1.0,
        blocking: bool = True,
    ) -> None:
        """Set all 7 joint positions (6 arm joints + gripper).

        Args:
            positions: 7-element sequence of joint positions in radians.
            duration: Time to reach the goal in seconds.
            blocking: If True, wait until motion completes.
        """
        self._require_connected('set_joint_positions')
        self.driver.set_all_positions(np.asarray(positions, dtype=float), duration, blocking)

    def set_arm_positions(
        self,
        positions: list[float] | np.ndarray,
        duration: float = 1.0,
        blocking: bool = True,
    ) -> None:
        """Set the 6 arm joint positions only (no gripper).

        Args:
            positions: 6-element sequence of arm joint positions in radians.
            duration: Time to reach the goal in seconds.
            blocking: If True, wait until motion completes.
        """
        self._require_connected('set_arm_positions')
        self.driver.set_arm_positions(np.asarray(positions, dtype=float), duration, blocking)

    # ── Cartesian commands ────────────────────────────────────────────────────

    def set_cartesian_positions(
        self,
        pose: list[float] | np.ndarray,
        duration: float = 1.0,
        blocking: bool = True,
        interp_space: trossen_arm.InterpolationSpace = trossen_arm.InterpolationSpace.joint,
    ) -> None:
        """Set the end-effector Cartesian pose.

        Args:
            pose: 6-element sequence [x, y, z, roll, pitch, yaw] in metres/radians.
            duration: Time to reach the goal in seconds.
            blocking: If True, wait until motion completes.
            interp_space: Joint-space (default, avoids singularities) or Cartesian-space
                interpolation.
        """
        self._require_connected('set_cartesian_positions')
        self.driver.set_cartesian_positions(
            np.asarray(pose, dtype=float), interp_space, duration, blocking
        )

    # ── Gripper ───────────────────────────────────────────────────────────────

    def set_gripper(self, position: float, duration: float = 1.0) -> None:
        """Set gripper opening in metres (0.0 = closed, 0.035 = open for WXAI V0)."""
        self._require_connected('set_gripper')
        self.driver.set_gripper_position(position, duration, True)

    def open_gripper(self, duration: float = 1.0) -> None:
        """Open the gripper to config.gripper_open_pos."""
        self.set_gripper(self.config.gripper_open_pos, duration)

    def close_gripper(self, duration: float = 1.0) -> None:
        """Close the gripper to config.gripper_close_pos."""
        self.set_gripper(self.config.gripper_close_pos, duration)

    # ── Teaching ──────────────────────────────────────────────────────────────

    def gravity_comp(self) -> None:
        """Enter gravity compensation mode for kinesthetic teaching.

        The arm floats under its own weight — manually guide it to record poses.
        Call disconnect() or go_home() to exit.
        """
        self._require_connected('gravity_comp')
        self.driver.set_all_modes(trossen_arm.Mode.external_effort)
        self.driver.set_all_external_efforts([0.0] * 7, 0.0, False)
        logger.info('Arm at %s in gravity compensation mode', self.config.ip_address)

    # ── Teleoperation helpers ─────────────────────────────────────────────────

    def stream_positions(
        self,
        positions: list[float] | np.ndarray,
        velocities: list[float] | np.ndarray | None = None,
    ) -> None:
        """Non-blocking position command for real-time control loops.

        Args:
            positions: 7-element target joint positions.
            velocities: Optional 7-element velocity feedforward values.
        """
        self._require_connected('stream_positions')
        if velocities is not None:
            self.driver.set_all_positions(
                np.asarray(positions, dtype=float), 0.0, False,
                np.asarray(velocities, dtype=float),
            )
        else:
            self.driver.set_all_positions(np.asarray(positions, dtype=float), 0.0, False)

    def stream_external_efforts(self, efforts: list[float] | np.ndarray) -> None:
        """Non-blocking effort command for real-time force feedback loops."""
        self._require_connected('stream_external_efforts')
        self.driver.set_all_external_efforts(list(efforts), 0.0, False)

    def get_positions(self) -> list[float]:
        """Return current all-joint positions (poll driver directly — no cache)."""
        return list(self.driver.get_all_positions())

    def get_velocities(self) -> list[float]:
        """Return current all-joint velocities (poll driver directly — no cache)."""
        return list(self.driver.get_all_velocities())

    def get_external_efforts(self) -> list[float]:
        """Return current all-joint external efforts (poll driver directly — no cache)."""
        return list(self.driver.get_all_external_efforts())
