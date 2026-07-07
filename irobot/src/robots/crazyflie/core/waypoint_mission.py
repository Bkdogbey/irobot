"""Reusable waypoint mission primitive: takeoff -> waypoints -> return/land.

The mission is deliberately hardware-agnostic: it drives any *commander*
object exposing ``take_off(z)``, ``go_to(x, y, z)`` and ``land()`` (the
cflib ``PositionHlCommander`` satisfies this), so it can be exercised in
tests — or simulation — with a fake commander.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol, Sequence

logger = logging.getLogger(__name__)

Waypoint = tuple[float, float, float]


class Commander(Protocol):
    """Minimal flight-command interface required by WaypointMission."""

    def take_off(self, height: float) -> None: ...

    def go_to(self, x: float, y: float, z: float) -> None: ...

    def land(self) -> None: ...


@dataclass
class Geofence:
    """Axis-aligned flight boundary; positions outside it are rejected.

    ``margin`` shrinks the box inward, so a 0.1 m margin on a [0, 2] range
    accepts only [0.1, 1.9].
    """

    x_range: tuple[float, float]
    y_range: tuple[float, float]
    z_range: tuple[float, float]
    margin: float = 0.0

    def contains(self, point: Sequence[float]) -> bool:
        x, y, z = point
        return (
            self.x_range[0] + self.margin <= x <= self.x_range[1] - self.margin
            and self.y_range[0] + self.margin <= y <= self.y_range[1] - self.margin
            and self.z_range[0] + self.margin <= z <= self.z_range[1] - self.margin
        )

    def check_waypoints(self, waypoints: Sequence[Sequence[float]]) -> list[int]:
        """Return the indices of waypoints that violate the geofence."""
        return [i for i, wp in enumerate(waypoints) if not self.contains(wp)]


class GeofenceViolationError(ValueError):
    """Raised when a mission contains waypoints outside the geofence."""


@dataclass
class WaypointMission:
    """Flies a waypoint sequence with pre-checks and emergency landing.

    Sequence: geofence precheck -> takeoff -> waypoints -> optional return
    to the first waypoint's xy at ``return_z`` -> land. Any exception during
    flight triggers an emergency landing before the exception is re-raised.

    Args:
        commander: object with take_off/go_to/land (e.g. PositionHlCommander).
        geofence: optional flight boundary checked before takeoff.
        takeoff_z: altitude reached before the first waypoint (m).
        return_z: altitude for the optional return-to-start leg (m).
        waypoint_delay: settle time after each go_to (s).
        return_to_start: fly back over the first waypoint before landing.
        on_waypoint: callback fired as on_waypoint(x, y, z) after each go_to
            command is issued — hook for commanded-trajectory logging.
        sleep: injectable sleep function (tests pass a no-op).
    """

    commander: Commander
    geofence: Geofence | None = None
    takeoff_z: float = 0.3
    return_z: float = 0.5
    waypoint_delay: float = 0.1
    return_to_start: bool = True
    on_waypoint: Callable[[float, float, float], None] | None = None
    sleep: Callable[[float], None] = field(default=time.sleep)

    def precheck(self, waypoints: Sequence[Waypoint]) -> None:
        """Validate the mission before flight; raises on any violation."""
        if not waypoints:
            raise ValueError('Mission has no waypoints')
        if self.geofence is not None:
            bad = self.geofence.check_waypoints(waypoints)
            if bad:
                raise GeofenceViolationError(
                    f'Waypoints {bad} lie outside the geofence (margin={self.geofence.margin} m)'
                )

    def _go_to(self, x: float, y: float, z: float) -> None:
        self.commander.go_to(x, y, z)
        if self.on_waypoint is not None:
            self.on_waypoint(x, y, z)
        self.sleep(self.waypoint_delay)

    def run(self, waypoints: Sequence[Waypoint]) -> None:
        """Execute the full mission; emergency-lands on any exception."""
        self.precheck(waypoints)

        logger.info('Mission start: %d waypoints, takeoff to %.2f m', len(waypoints), self.takeoff_z)
        self.commander.take_off(self.takeoff_z)
        try:
            for x, y, z in waypoints:
                self._go_to(x, y, z)

            if self.return_to_start:
                last_x, last_y, _ = waypoints[-1]
                start_x, start_y, _ = waypoints[0]
                self._go_to(last_x, last_y, self.return_z)
                self._go_to(start_x, start_y, self.return_z)
        except Exception:
            logger.exception('Mission failed — emergency landing')
            self._emergency_land()
            raise
        else:
            logger.info('Mission complete — landing')
            self.commander.land()

    def _emergency_land(self) -> None:
        try:
            self.commander.land()
        except Exception:
            logger.exception('Emergency landing itself failed')
