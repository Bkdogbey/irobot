from __future__ import annotations

import cflib
from attrs import define
from ros_sugar.config import BaseComponentConfig


@define(kw_only=True)
class CrazyflieConfig(BaseComponentConfig):
    """
    Component configuration parameters
    """

    cf = None
    is_connected = False
    z_hold: float = 0.3
