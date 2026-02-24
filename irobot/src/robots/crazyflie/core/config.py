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
<<<<<<< HEAD
    cflib.crtp.init_drivers()
=======
    cflib.crtp.init_drivers()
>>>>>>> 92b566d (restructure)
