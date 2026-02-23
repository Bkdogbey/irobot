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

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
