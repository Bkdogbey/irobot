from __future__ import annotations

from dataclasses import dataclass


@dataclass
class CrazyflieConfig:
    """Hardware configuration for a Crazyflie drone.

    This is the single place to change when targeting a different radio
    channel, address, logging rate, or hover altitude.
    """

    uri: str = 'radio://0/80/2M/E7E7E7E781'  # radio://<channel>/<datarate>/<address>
    log_rate_ms: int = 50                      # Lighthouse log period; 50 ms = 20 Hz
    connect_timeout: float = 10.0              # seconds before giving up on connection
    hover_z: float = 0.3                       # default hover altitude (m)
