"""Lighthouse position logging mixin for Crazyflie."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from cflib.crazyflie.log import LogConfig

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class LighthouseLogging:
    """Mixin that adds Lighthouse state logging to a Crazyflie connection class.

    Expects the host class to provide:
      self.cf         — cflib Crazyflie instance
      self.config     — CrazyflieConfig (for log_rate_ms)
      self.current_*  — state fields initialised in the host's __init__
    """

    def _setup_lighthouse_logging(self) -> None:
        log_conf = LogConfig(name='LighthouseState', period_in_ms=self.config.log_rate_ms)  # type: ignore[attr-defined]
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        log_conf.add_variable('stateEstimate.vx', 'float')
        log_conf.add_variable('stateEstimate.vy', 'float')
        log_conf.add_variable('stateEstimate.yaw', 'float')
        self.cf.log.add_config(log_conf)  # type: ignore[attr-defined]
        log_conf.data_received_cb.add_callback(self._state_callback)
        log_conf.start()
        logger.info('Lighthouse logging started at %d ms period', self.config.log_rate_ms)  # type: ignore[attr-defined]

    def _state_callback(self, _timestamp: Any, data: dict, _logconf: Any) -> None:
        self.current_x = data['stateEstimate.x']  # type: ignore[attr-defined]
        self.current_y = data['stateEstimate.y']  # type: ignore[attr-defined]
        self.current_z = data['stateEstimate.z']  # type: ignore[attr-defined]
        self.current_vx = data['stateEstimate.vx']  # type: ignore[attr-defined]
        self.current_vy = data['stateEstimate.vy']  # type: ignore[attr-defined]
        self.current_yaw = data['stateEstimate.yaw']  # type: ignore[attr-defined]
        self.state_ready = True  # type: ignore[attr-defined]
        self._publish_state()

    def _publish_state(self) -> None:
        """Override in a subclass to publish state to ROS or another transport."""
