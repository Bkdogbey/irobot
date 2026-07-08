from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class CrazyflieConfig:
    """Hardware configuration for a single Crazyflie drone."""

    uri: str = 'radio://0/80/2M/E7E7E7E780'
    log_rate_ms: int = 50
    connect_timeout: float = 10.0
    hover_z: float = 0.3


@dataclass
class SwarmSquareConfig:
    """4 drones each fly a 0.3 m square from their own start position (relative moves).

    Place drones ≥ 0.5 m apart on the floor so individual squares don't overlap.
    """

    uris: set[str] = field(default_factory=lambda: {
        'radio://0/84/2M/E7E7E7E784',
        'radio://0/80/2M/E7E7E7E780',
        'radio://0/82/2M/E7E7E7E782',
        'radio://0/83/2M/E7E7E7E783',
    })
    takeoff_height: float = 0.5   # metres — hard cap: never exceed 0.6
    box_size: float = 0.3         # side length of the square in metres
    flight_time: float = 2.0      # seconds per leg


@dataclass
class SwarmHVConfig:
    """3 drones: horizontal line → vertical line → horizontal → vertical → land.

    Workspace: x[0,1] m, y[0,-2] m.
    URI order: D1, D2, D3.
    Initial floor placement (horizontal line at y=-0.5):
      D1=(1,-0.5)  D2=(0.5,-0.5)  D3=(0,-0.5)
    Collision-free sequencing: steps that share a waypoint use two sub-steps
    (departing drone moves first, arriving drone moves second).
    """

    uris: list[str] = field(default_factory=lambda: [
        'radio://0/80/2M/E7E7E7E780',   # D1 — starts at (1, -0.5)
        'radio://0/83/2M/E7E7E7E783',   # D2 — starts at (0.5, -0.5)
        'radio://0/84/2M/E7E7E7E784',   # D3 — starts at (0, -0.5)
    ])
    height: float = 0.5       # metres — hard cap: never exceed 0.6
    move_time: float = 2.5    # seconds per formation step
    dwell_time: float = 2.0   # seconds to hold each formation


@dataclass
class LeaderFollowerConfig:
    """2 drones flying an S-curve through x[0,1] y[0,-2] workspace.

    Follower maintains a fixed +0.35 m y-offset (north) of the leader at
    every waypoint, ensuring ≥0.35 m separation throughout the flight.
    Leader waypoints trace an S-curve; follower mirrors them offset north.
    """

    leader_uri: str = 'radio://0/80/2M/E7E7E7E780'
    follower_uri: str = 'radio://0/84/2M/E7E7E7E784'
    height: float = 0.3            # metres — hard cap: never exceed 0.6
    follower_offset_y: float = 0.35
    leg_time: float = 3.0          # seconds allocated per waypoint leg
    blend_time: float = 0.5        # issue next go_to this many seconds early for smooth curves

    # S-curve through the workspace — (x, y) pairs, height applied separately
    leader_waypoints: list = field(default_factory=lambda: [
        (0.2, -0.5),   # upper left
        (0.8, -0.5),   # upper right
        (0.8, -1.0),   # mid right
        (0.2, -1.0),   # mid left
        (0.2, -1.5),   # lower left
        (0.8, -1.5),   # lower right
        (0.5, -1.0),   # centre — land here
    ])


@dataclass
class SplineWaypointConfig:
    """Single drone: sine S-curve from (1,-1.75) to (1,0) passing through (0.5,-0.5).

    Workspace: x[0,1] m, y[0,-2] m.

    Parametrisation (t ∈ [0,1]):
      x(t) = 1.0 - 0.64 * sin(π * t)
      y(t) = -1.75 + 1.75 * t
    Amplitude 0.64 ensures x = 0.5 at t = 5/7 (y = -0.5).
    blend_time: next go_to issued this many seconds before current leg ends,
                so the drone curves smoothly through each waypoint.
    """

    uri: str = 'radio://0/80/2M/E7E7E7E780'
    height: float = 0.2          # metres — hard cap: never exceed 0.6
    leg_time: float = 2.5        # seconds allocated per waypoint segment
    blend_time: float = 0.6      # seconds of early overlap for smooth curves
    waypoints: list = field(default_factory=lambda: [
        (round(1.0 - 0.64 * math.sin(math.pi * i / 7), 3),
         round(-1.75 + 1.75 * i / 7, 3))
        for i in range(8)
    ])


@dataclass
class SwarmTransitionConfig:
    """4 drones: vertical line → square → line → land. All positions within ±0.5 m.

    URI order: top, upper-mid, lower-mid, bottom of the vertical line.
    Transition uses a guaranteed collision-free 2-step move:
      Step A — x-adjust only (y rows stay separated ≥ 0.3 m).
      Step B — y-adjust only (x columns never share a row).
    Requires absolute positioning (Lighthouse / MoCap).
    """

    uris: list[str] = field(default_factory=lambda: [
        'radio://0/80/2M/E7E7E7E780',   # top of line / front-left of square
        #'radio://0/82/2M/E7E7E7E782',   # upper-mid  / front-right
        'radio://0/83/2M/E7E7E7E783',   # lower-mid  / back-right
        'radio://0/84/2M/E7E7E7E784',   # bottom     / back-left
    ])
    height: float = 0.5          # metres — hard cap: never exceed 0.6
    line_spacing: float = 0.3    # gap between adjacent drones in vertical line
    square_half: float = 0.3     # drones at (±square_half, ±square_half)
    move_time: float = 2.5       # seconds per formation change
    dwell_time: float = 2.0      # seconds to hold each formation
