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
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

logger = logging.getLogger(__name__)


MIN_THRUST = 20000
MAX_THRUST = 25000

DEFAULT_HOVER_Z = 0.3  # meters


class CrazyflieBase:
    """
    Sugarcoat Component for Crazyflie drone control.

    This class implements a Sugarcoat Component that provides a high-level interface
    for connecting to and controlling Crazyflie drones, following the Sugarcoat
    design patterns for component-based robotics development.
    """

    def __init__(self, config) -> None:
        # Set default config if config is not provided
        self.config = config

        self.uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E781')
        self.cf = None
        self.is_connected = False

        # Initialize the low-level drivers
        cflib.crtp.init_drivers()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_yaw = 0.0  # radians, from stateEstimate.yaw
        self.state_ready = False

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
        self._setup_lighthouse_logging()

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

    def _move_to(
        self,
        p0: tuple[float, float, float],
        p1: tuple[float, float, float],
        yaw: float = 0.0,
        duration: float = 2.0,
    ):
        """Linearly interpolate from p0 to p1 over duration seconds."""
        x0, y0, z0 = p0
        x1, y1, z1 = p1
        dt = 0.05  # 20 Hz
        steps = max(1, int(duration / dt))
        for i in range(steps):
            t = i / steps
            xi = x0 + (x1 - x0) * t
            yi = y0 + (y1 - y0) * t
            zi = z0 + (z1 - z0) * t
            self.cf.commander.send_position_setpoint(xi, yi, zi, yaw)
            time.sleep(dt)
        # Hold the final position for one step
        self.cf.commander.send_position_setpoint(x1, y1, z1, yaw)

    def fly_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
        duration: float = 2.0,
    ):
        """
        Fly to a position using smooth interpolation from the current lighthouse position.

        Requires a positioning deck (e.g. Flow deck or Lighthouse).

        Args:
            x: Target x position in meters
            y: Target y position in meters
            z: Target z position in meters (altitude)
            yaw: Target yaw in degrees
            duration: Time to reach the target in seconds

        """
        if not self.cf or not self.is_connected:
            logger.warning('Cannot fly_to: Not connected to Crazyflie')
            return

        start = (self.current_x, self.current_y, self.current_z)
        logger.info('Flying to (%.2f, %.2f, %.2f) yaw=%.1f over %.1fs', x, y, z, yaw, duration)
        self._move_to(start, (x, y, z), yaw=yaw, duration=duration)

    def fly_from_to(
        self,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        travel_time: float = 6.0,
        hover_time: float = 3.0,
    ):
        """
        Fly from one point to another with smooth interpolation.

        Sequence: takeoff → hover at start → fly to end → hover → land.

        Args:
            start: (x, y, z) start position in meters
            end: (x, y, z) end position in meters
            travel_time: Seconds to travel between waypoints
            hover_time: Seconds to hover at each waypoint

        """
        if not self.cf or not self.is_connected:
            logger.warning('Cannot fly_from_to: Not connected to Crazyflie')
            return

        # Unlock startup thrust protection
        self.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        sx, sy, sz = start
        ex, ey, ez = end

        logger.info('Flying from %s to %s', start, end)

        # Takeoff: rise slowly — use dedicated takeoff_time, not travel_time
        takeoff_time = max(travel_time, travel_time * (sz / max(sz, 0.01)))
        self._move_to((sx, sy, 0.0), (sx, sy, sz), duration=takeoff_time)

        # Hover at start
        self.fly_to(sx, sy, sz, duration=hover_time)

        # Fly to end
        self.fly_to(ex, ey, ez, duration=travel_time)

        # Hover at end
        self.fly_to(ex, ey, ez, duration=hover_time)

        # Land: descend slowly to near-ground, then cut motors
        logger.info('Landing')
        self._move_to((ex, ey, ez), (ex, ey, 0.05), duration=takeoff_time)
        for _ in range(20):
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)

    def _execution_step(self):

        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0

        # Unlock startup thrust protection
        self.cf.commander.send_setpoint(0, 0, 0, 0)

        while thrust >= 20000:
            self.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 25000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        for _ in range(30):
            # Continuously send the zero setpoint until the drone is recognized as landed
            # to prevent the supervisor from intervening due to missing regular setpoints
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            # Sleeping before closing the link makes sure the last
            # packet leaves before the link is closed, since the
            # message queue is not flushed before closing
            time.sleep(0.1)

    def _setup_lighthouse_logging(self):
        """
        Start cflib LogConfig for Lighthouse state estimates at 20 Hz.
        Called inside _connected() after the arming delay.
        stateEstimate.x/y/z are in the lighthouse world frame (meters).
        """
        log_conf = LogConfig(name='LighthouseState', period_in_ms=50)
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        log_conf.add_variable('stateEstimate.vx', 'float')
        log_conf.add_variable('stateEstimate.vy', 'float')
        log_conf.add_variable('stateEstimate.yaw', 'float')  # radians

        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self._state_callback)
        log_conf.start()
        logger.info('Lighthouse state logging started at 20 Hz')

    def _state_callback(self, _timestamp, data, _logconf):
        """Receives lighthouse state estimates, updates local fields, and publishes to ROS."""
        self.current_x = data['stateEstimate.x']
        self.current_y = data['stateEstimate.y']
        self.current_z = data['stateEstimate.z']
        self.current_vx = data['stateEstimate.vx']
        self.current_vy = data['stateEstimate.vy']
        self.current_yaw = data['stateEstimate.yaw']
        self.state_ready = True
        self._publish_state()

    def _publish_state(self):
        """Placeholder for ROS state publishing (odom/TF). Override in subclass if needed."""

    def send_velocity_setpoint(self, vx: float, vy: float, yaw_rate: float, z_hold: float):
        """
        Send 2D velocity command at fixed hover height.
        Maps directly to SingleIntegrator planner output u = [vx, vy].
        Firmware PID handles z independently.
        """
        if self.cf and self.is_connected:
            self.cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z_hold)
        else:
            logger.warning('Cannot send velocity setpoint: not connected')

    def takeoff(self, z_hold: float = 0.5, duration: float = 3.0):
        """
        Ramp to hover height before handing control to the planner.
        Sends hover setpoint at z_hold for the full duration.
        """
        if not self.is_connected:
            logger.warning('Cannot takeoff: not connected')
            return
        logger.info('Taking off to z=%.2f over %.1fs', z_hold, duration)
        steps = int(duration / 0.1)
        for _ in range(steps):
            self.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_hold)
            time.sleep(0.1)
        logger.info('Takeoff complete')

    def land(self, z_hold: float = 0.5, duration: float = 2.5):
        """
        Ramp z down to zero then cut motors.
        """
        if not self.is_connected:
            return
        logger.info('Landing from z=%.2f over %.1fs', z_hold, duration)
        steps = int(duration / 0.1)
        for i in range(steps):
            z = z_hold * (1.0 - (i / steps))
            z = max(0.05, z)
            self.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)
            time.sleep(0.1)
        # Cut motors
        self.cf.commander.send_setpoint(0, 0, 0, 0)
        logger.info('Landed')

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
