"""Crazyflie flight control and movement primitives."""

from __future__ import annotations

import logging
import time

from irobot.src.robots.crazyflie.core.base import CrazyflieBase

logger = logging.getLogger(__name__)

MIN_THRUST = 20000
MAX_THRUST = 25000
DEFAULT_HOVER_Z = 0.3  # meters


class CrazyflieController(CrazyflieBase):
    """Flight control built on top of CrazyflieBase."""

    def send_setpoint(self, roll: float, pitch: float, yaw_rate: float, thrust: int) -> None:
        """Send a raw roll/pitch/yaw-rate/thrust setpoint."""
        if self.cf and self.is_connected:
            self.cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)
        else:
            logger.warning('Cannot send setpoint: not connected')

    def send_velocity_setpoint(self, vx: float, vy: float, yaw_rate: float, z_hold: float) -> None:
        """Send a 2D velocity command at fixed hover height."""
        if self.cf and self.is_connected:
            self.cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z_hold)
        else:
            logger.warning('Cannot send velocity setpoint: not connected')

    def ramp_motors(self) -> None:
        """Ramp motors up and back down — basic motor demo."""
        if not self.is_connected:
            logger.warning('Cannot ramp motors: not connected')
            return

        thrust_mult = 1
        thrust_step = 500
        thrust = MIN_THRUST
        self.cf.commander.send_setpoint(0, 0, 0, 0)  # unlock startup protection

        while thrust >= MIN_THRUST:
            self.cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(0.1)
            if thrust >= MAX_THRUST:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult

        for _ in range(30):
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)

    def _move_to(
        self,
        p0: tuple[float, float, float],
        p1: tuple[float, float, float],
        yaw: float = 0.0,
        duration: float = 2.0,
    ) -> None:
        """Linearly interpolate from p0 to p1 over duration seconds at 20 Hz."""
        x0, y0, z0 = p0
        x1, y1, z1 = p1
        dt = 0.05
        steps = max(1, int(duration / dt))
        for i in range(steps):
            t = i / steps
            self.cf.commander.send_position_setpoint(
                x0 + (x1 - x0) * t,
                y0 + (y1 - y0) * t,
                z0 + (z1 - z0) * t,
                yaw,
            )
            time.sleep(dt)
        self.cf.commander.send_position_setpoint(x1, y1, z1, yaw)

    def fly_to(self, x: float, y: float, z: float, yaw: float = 0.0, duration: float = 2.0) -> None:
        """Fly to (x, y, z) from current position using smooth interpolation."""
        if not self.cf or not self.is_connected:
            logger.warning('Cannot fly_to: not connected')
            return
        logger.info('Flying to (%.2f, %.2f, %.2f) over %.1fs', x, y, z, duration)
        self._move_to((self.current_x, self.current_y, self.current_z), (x, y, z), yaw=yaw, duration=duration)

    def fly_from_to(
        self,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        travel_time: float = 6.0,
        hover_time: float = 3.0,
    ) -> None:
        """Takeoff → hover at start → fly to end → hover → land."""
        if not self.cf or not self.is_connected:
            logger.warning('Cannot fly_from_to: not connected')
            return

        sx, sy, sz = start
        ex, ey, ez = end
        self.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        logger.info('Flying from %s to %s', start, end)
        takeoff_time = max(travel_time, travel_time * (sz / max(sz, 0.01)))
        self._move_to((sx, sy, 0.0), (sx, sy, sz), duration=takeoff_time)
        self.fly_to(sx, sy, sz, duration=hover_time)
        self.fly_to(ex, ey, ez, duration=travel_time)
        self.fly_to(ex, ey, ez, duration=hover_time)

        logger.info('Landing')
        self._move_to((ex, ey, ez), (ex, ey, 0.05), duration=takeoff_time)
        for _ in range(20):
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)

    def takeoff(self, z_hold: float = 0.5, duration: float = 3.0) -> None:
        """Hover at z_hold for duration seconds to establish stable flight."""
        if not self.is_connected:
            logger.warning('Cannot takeoff: not connected')
            return
        logger.info('Taking off to z=%.2f over %.1fs', z_hold, duration)
        steps = int(duration / 0.1)
        for _ in range(steps):
            self.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_hold)
            time.sleep(0.1)
        logger.info('Takeoff complete')

    def land(self, z_hold: float = 0.5, duration: float = 2.5) -> None:
        """Ramp z down to zero then cut motors."""
        if not self.is_connected:
            return
        logger.info('Landing from z=%.2f over %.1fs', z_hold, duration)
        steps = int(duration / 0.1)
        for i in range(steps):
            z = max(0.05, z_hold * (1.0 - (i / steps)))
            self.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)
            time.sleep(0.1)
        self.cf.commander.send_setpoint(0, 0, 0, 0)
        logger.info('Landed')
