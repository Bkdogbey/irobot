from __future__ import annotations

import cflib
from attrs import define
from ros_sugar.config import BaseComponentConfig

# ── Hardware ───────────────────────────────────────────────────────────────────
Z_HOLD: float = 0.3             # hover altitude (m)

# ── Trial — edit these before each run ────────────────────────────────────────
USE_OPTIMISED: bool = False      # True → pDSTL-optimised path, False → sine path
CONDITION: str = 'deterministic' # 'deterministic' | 'pdstl'
FAN_SPEED: int = 0               # 0 | 2 | 6 | 12 | 16

# ── ROS Sugar component config ─────────────────────────────────────────────────
@define(kw_only=True)
class CrazyflieConfig(BaseComponentConfig):
    cf = None
    is_connected: bool = False
    z_hold: float = Z_HOLD

cflib.crtp.init_drivers()
