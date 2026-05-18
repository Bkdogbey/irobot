"""Crazyflie connection lifecycle."""

from __future__ import annotations

import logging
import time
from typing import Any

import cflib
from cflib.crazyflie import Crazyflie

from irobot.src.robots.crazyflie.config import CrazyflieConfig
from irobot.src.robots.crazyflie.core.logging import LighthouseLogging

logger = logging.getLogger(__name__)


class CrazyflieBase(LighthouseLogging):
    """Manages connection to a Crazyflie drone and exposes live state."""

    def __init__(self, config: CrazyflieConfig | None = None) -> None:
        self.config = config or CrazyflieConfig()
        self.cf: Crazyflie | None = None
        self.is_connected = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_yaw = 0.0
        self.state_ready = False

        cflib.crtp.init_drivers()
        self.connect()

    def connect(self) -> bool:
        """Open the radio link; returns True when connected."""
        try:
            logger.info('Connecting to %s', self.config.uri)
            self.cf = Crazyflie(rw_cache='./cache')
            self.cf.connected.add_callback(self._connected)
            self.cf.disconnected.add_callback(self._disconnected)
            self.cf.connection_failed.add_callback(self._connection_failed)
            self.cf.connection_lost.add_callback(self._connection_lost)
            self.cf.open_link(self.config.uri)

            start = time.time()
            while not self.is_connected and (time.time() - start) < self.config.connect_timeout:
                time.sleep(0.1)
        except Exception:
            logger.exception('Failed to connect')
            return False
        return self.is_connected

    def _connected(self, link_uri: str) -> None:
        logger.info('Connected to %s', link_uri)
        self.is_connected = True
        self.cf.platform.send_arming_request(True)  # type: ignore[union-attr]
        time.sleep(1.0)
        self._setup_lighthouse_logging()

    def _connection_failed(self, link_uri: str, msg: str) -> None:
        logger.error('Connection to %s failed: %s', link_uri, msg)

    def _connection_lost(self, link_uri: str, msg: str) -> None:
        logger.warning('Connection to %s lost: %s', link_uri, msg)
        self.is_connected = False

    def _disconnected(self, link_uri: str) -> None:
        logger.info('Disconnected from %s', link_uri)
        self.is_connected = False

    def disconnect(self) -> None:
        if self.cf and self.is_connected:
            self.cf.close_link()
            self.is_connected = False

    def arm(self) -> None:
        if self.cf and self.is_connected:
            self.cf.platform.send_arming_request(True)

    def disarm(self) -> None:
        if self.cf and self.is_connected:
            self.cf.platform.send_arming_request(False)

    def get_status(self) -> dict[str, Any]:
        return {'uri': self.config.uri, 'connected': self.is_connected, 'timestamp': time.time()}

    def __del__(self) -> None:
        if self.cf and self.is_connected:
            self.disconnect()
