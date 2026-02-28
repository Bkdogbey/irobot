"""
Flight data logger for Crazyflie experiment trials.

Four conditions:
    deterministic_nominal  — original sine path, no fan
    deterministic_wind     — original sine path, fan on
    pdstl_nominal          — pDSTL-optimised path, no fan
    pdstl_wind             — pDSTL-optimised path, fan on

Usage (from crazyflie.py):
    logger = FlightLogger("pdstl_wind")
    logger.start()
    ...
    logger.log_waypoint(x, y, z)
    ...
    logger.save()

Output: logs/<condition>_<timestamp>.csv
Columns: condition, t, x, y, z, outside_obs1, outside_obs2, safe
"""

from __future__ import annotations

import csv
import pathlib
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Sequence

# ── Valid condition labels ──────────────────────────────────────────────────
CONDITIONS = (
    "deterministic_nominal",
    "deterministic_wind",
    "pdstl_nominal",
    "pdstl_wind",
)

# ── Obstacles (x_min, x_max, y_min, y_max) — must match optimise_cf_path.py ─
_OBSTACLES: list[tuple[float, float, float, float]] = [
    (0.15, 0.55, -0.90, -0.50),  # OBS-1
    (-0.45, 0.05, 0.00, 0.40),   # OBS-2
]

# ── Log directory: next to this file ────────────────────────────────────────
_LOGS_DIR = pathlib.Path(__file__).parent / "logs"


def _inside(x: float, y: float, obs: tuple[float, float, float, float]) -> bool:
    """Return True if point (x, y) is inside rectangular obstacle."""
    x0, x1, y0, y1 = obs
    return x0 <= x <= x1 and y0 <= y <= y1


@dataclass
class FlightLogger:
    """Logs commanded waypoints and per-step geometric safety for one trial."""

    condition: str
    obstacles: list[tuple[float, float, float, float]] = field(
        default_factory=lambda: list(_OBSTACLES)
    )

    _t0: float = field(init=False, default=0.0)
    _rows: list[dict] = field(init=False, default_factory=list)

    def __post_init__(self) -> None:
        if self.condition not in CONDITIONS:
            raise ValueError(
                f"Unknown condition {self.condition!r}. "
                f"Choose from: {CONDITIONS}"
            )

    def start(self) -> None:
        """Call just before the first go_to."""
        self._t0 = time.monotonic()
        self._rows = []

    def log_waypoint(self, x: float, y: float, z: float) -> None:
        """Log one commanded waypoint with elapsed time and safety flags."""
        t = round(time.monotonic() - self._t0, 4)
        outside = [not _inside(x, y, obs) for obs in self.obstacles]
        safe = all(outside)
        row: dict = {
            "condition": self.condition,
            "t": t,
            "x": round(x, 6),
            "y": round(y, 6),
            "z": round(z, 6),
        }
        for i, out in enumerate(outside, start=1):
            row[f"outside_obs{i}"] = int(out)
        row["safe"] = int(safe)
        self._rows.append(row)

    def save(self) -> pathlib.Path:
        """Write CSV and return the file path."""
        _LOGS_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        path = _LOGS_DIR / f"{self.condition}_{ts}.csv"

        if not self._rows:
            print("[FlightLogger] No data to save.")
            return path

        fieldnames = list(self._rows[0].keys())
        with path.open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

        n_unsafe = sum(1 for r in self._rows if not r["safe"])
        print(
            f"[FlightLogger] Saved {len(self._rows)} waypoints "
            f"({n_unsafe} unsafe) → {path}"
        )
        return path
