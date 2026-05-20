"""Trossen arm connection lifecycle and state management."""

from __future__ import annotations

import logging
import time
from typing import Any

import trossen_arm

from irobot.src.robots.trossen_arm.config import TrossenArmConfig

logger = logging.getLogger(__name__)


class TrossenArmBase:
    """Manages connection to a Trossen WXAI V0 arm and exposes live state.

    One instance = one physical arm. For multi-arm setups, instantiate one
    TrossenArmBase (or TrossenArmController) per arm.
    """

    def __init__(self, config: TrossenArmConfig | None = None) -> None:
        self.config = config or TrossenArmConfig()
        self.driver = trossen_arm.TrossenArmDriver()
        self.is_connected = False

        # Cached state — populated by _poll_state()
        self.joint_positions: list[float] = [0.0] * 7
        self.joint_velocities: list[float] = [0.0] * 7
        self.joint_efforts: list[float] = [0.0] * 7
        self.joint_external_efforts: list[float] = [0.0] * 7
        self.ee_pose: list[float] = [0.0] * 6   # [x, y, z, roll, pitch, yaw]
        self.gripper_position: float = 0.0
        self.state_ready: bool = False

        self.connect()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

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

    def disconnect(self) -> None:
        """Release the driver connection. Safe to call even if not connected."""
        if self.is_connected:
            try:
                self.driver.cleanup()
            except Exception:
                logger.warning('driver.cleanup() failed during disconnect')
            self.is_connected = False
            logger.info('Disconnected from Trossen arm at %s', self.config.ip_address)

    def e_stop(self) -> None:
        """Emergency stop: immediately clean up the driver, ignoring all errors."""
        logger.warning('E-STOP called for arm at %s', self.config.ip_address)
        try:
            self.driver.cleanup()
        except Exception:
            pass
        self.is_connected = False

    def __del__(self) -> None:
        if self.is_connected:
            self.disconnect()

    # ── State ─────────────────────────────────────────────────────────────────

    def _poll_state(self) -> None:
        """Poll the driver and cache all joint and Cartesian state."""
        output = self.driver.get_robot_output()
        self.joint_positions = list(output.joint.all.positions)
        self.joint_velocities = list(output.joint.all.velocities)
        self.joint_efforts = list(output.joint.all.efforts)
        self.joint_external_efforts = list(output.joint.all.external_efforts)
        self.ee_pose = list(output.cartesian.positions)
        self.gripper_position = list(output.joint.gripper.positions)[0]
        self.state_ready = True

    def get_status(self) -> dict[str, Any]:
        return {
            'ip_address': self.config.ip_address,
            'connected': self.is_connected,
            'timestamp': time.time(),
        }

    # ── Internal ──────────────────────────────────────────────────────────────

    def _require_connected(self, method: str) -> None:
        if not self.is_connected:
            raise RuntimeError(
                f'TrossenArmBase.{method}: arm at {self.config.ip_address} is not connected'
            )
