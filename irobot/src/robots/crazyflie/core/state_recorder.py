"""Fixed-rate recorder for raw Crazyflie state.

Records *measured* state only (time, position, velocity, source label); it
makes no judgement about safety, obstacles, or specifications — that belongs
to whatever experiment consumes the recording.
"""

from __future__ import annotations

import csv
import pathlib
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Mapping

FIELDNAMES = ('time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'source')


@dataclass
class StateRecorder:
    """Samples a state provider at a fixed rate in a background thread.

    Args:
        get_state: callable returning a mapping with at least ``x``/``y``/``z``
            (``vx``/``vy``/``vz`` optional — left blank when absent). For a
            connected drone: ``lambda: {'x': cf.current_x, ...}``.
        rate_hz: sampling frequency.
        source: label written to every row (e.g. 'lighthouse', 'sim').
    """

    get_state: Callable[[], Mapping[str, float]]
    rate_hz: float = 10.0
    source: str = 'lighthouse'

    _t0: float = field(init=False, default=0.0)
    _rows: list[dict] = field(init=False, default_factory=list)
    _stop_event: threading.Event = field(init=False, default_factory=threading.Event)
    _thread: threading.Thread | None = field(init=False, default=None)

    @property
    def rows(self) -> list[dict]:
        return list(self._rows)

    def start(self) -> None:
        """Reset the clock and begin sampling in a daemon thread."""
        if self._thread is not None:
            raise RuntimeError('StateRecorder is already running')
        self._t0 = time.monotonic()
        self._rows = []
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _sample_once(self) -> None:
        t = round(time.monotonic() - self._t0, 4)
        state = self.get_state()
        row = {'time': t}
        for key in ('x', 'y', 'z', 'vx', 'vy', 'vz'):
            value = state.get(key)
            row[key] = round(float(value), 6) if value is not None else ''
        row['source'] = self.source
        self._rows.append(row)

    def _loop(self) -> None:
        interval = 1.0 / self.rate_hz
        while not self._stop_event.is_set():
            tick = time.monotonic()
            self._sample_once()
            remaining = interval - (time.monotonic() - tick)
            if remaining > 0:
                self._stop_event.wait(timeout=remaining)

    def stop(self) -> None:
        """Stop sampling; safe to call twice."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def save(self, path: str | pathlib.Path) -> pathlib.Path:
        """Write all sampled rows as CSV and return the path."""
        path = pathlib.Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=FIELDNAMES)
            writer.writeheader()
            writer.writerows(self._rows)
        return path
