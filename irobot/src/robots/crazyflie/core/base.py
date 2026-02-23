"""
Sugarcoat Component for Crazyflie drone connectivity.

This module provides a Sugarcoat Component interface for connecting to and controlling
Crazyflie nano quadcopters, enabling seamless integration with the irobot
ecosystem for robotics research.
"""

from __future__ import annotations

import logging
import time
from typing import Any

import cflib
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper
from ros_sugar.core import BaseComponent

logger = logging.getLogger(__name__)


MIN_THRUST = 20000
MAX_THRUST = 25000


class CrazyflieComponent(BaseComponent):
    """
    Sugarcoat Component for Crazyflie drone control.

    This class implements a Sugarcoat Component that provides a high-level interface
    for connecting to and controlling Crazyflie drones, following the Sugarcoat
    design patterns for component-based robotics development.
    """

    def __init__(
        self,
        *,
        component_name: str,
        config: None,
        **kwargs,
    ) -> None:
        # Set default config if config is not provided
        self.config = config

        super().__init__(
            component_name=component_name,
            config=self.config,
            **kwargs,
        )

        self.uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
        self.cf = None
        self.is_connected = False

        # Initialize the low-level drivers
        cflib.crtp.init_drivers()
        self.connect()

    def connect(self) -> bool:
        """
        Connect to the Crazyflie drone.

        Returns:
            True if connection was successful, False otherwise

        """
        try:
            logger.info('Connecting to Crazyflie at %s', self.uri)

            self.cf = Crazyflie(rw_cache='./cache')

            # Setup callbacks
            self.cf.connected.add_callback(self._connected)
            self.cf.disconnected.add_callback(self._disconnected)
            self.cf.connection_failed.add_callback(self._connection_failed)
            self.cf.connection_lost.add_callback(self._connection_lost)

            # Open the connection
            self.cf.open_link(self.uri)

            # Wait for connection (simple timeout mechanism)
            timeout = 10
            start_time = time.time()
            while not self.is_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)

        except Exception:
            logger.exception('Failed to connect to Crazyflie')
            return False
        else:
            return self.is_connected

    def _connected(self, link_uri: str):
        """Handle successful connection to Crazyflie."""
        logger.info('Successfully connected to Crazyflie at %s', link_uri)
        self.is_connected = True

        # Arm the Crazyflie
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

    def _connection_failed(self, link_uri: str, msg: str):
        """Handle connection failure."""
        logger.error('Connection to %s failed: %s', link_uri, msg)

    def _connection_lost(self, link_uri: str, msg: str):
        """Handle connection loss."""
        logger.warning('Connection to %s lost: %s', link_uri, msg)
        self.is_connected = False

    def _disconnected(self, link_uri: str):
        """Handle disconnection."""
        logger.info('Disconnected from %s', link_uri)
        self.is_connected = False

    def disconnect(self):
        """Disconnect from the Crazyflie drone."""
        if self.cf and self.is_connected:
            self.cf.close_link()
            self.is_connected = False
            logger.info('Disconnected from Crazyflie')

    def send_setpoint(self, roll: float, pitch: float, yaw_rate: float, thrust: int):
        """
        Send a setpoint to the Crazyflie drone.

        Args:
            roll: Roll angle in degrees
            pitch: Pitch angle in degrees
            yaw_rate: Yaw rate in degrees/second
            thrust: Thrust value (0-65535)

        """
        if self.cf and self.is_connected:
            self.cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)
        else:
            logger.warning('Cannot send setpoint: Not connected to Crazyflie')

    def ramp_motors(self):
        """
        Ramp up and down the motors (similar to the example).
        This is a basic demonstration of motor control.
        """
        if not self.is_connected:
            logger.warning('Cannot ramp motors: Not connected to Crazyflie')
            return

        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        self.cf.commander.send_setpoint(0, 0, 0, 0)

        while thrust >= MIN_THRUST:
            self.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if thrust >= MAX_THRUST:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult

        # Land the drone
        for _ in range(30):
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

    def _execution_step(self):
        self.ramp_motors()

    def arm(self):
        """Arm the Crazyflie drone."""
        if self.cf and self.is_connected:
            self.cf.platform.send_arming_request(True)
            logger.info('Crazyflie armed')

    def disarm(self):
        """Disarm the Crazyflie drone."""
        if self.cf and self.is_connected:
            self.cf.platform.send_arming_request(False)
            logger.info('Crazyflie disarmed')

    def get_status(self) -> dict[str, Any]:
        """
        Get the current status of the Crazyflie.

        Returns:
            Dictionary containing current status information

        """
        return {'name': self.name, 'uri': self.uri, 'connected': self.is_connected, 'timestamp': time.time()}

    def __del__(self):
        """Cleanup when object is destroyed."""
        if self.cf and self.is_connected:
            self.disconnect()
