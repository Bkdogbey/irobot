"""iRobot: iHuman Lab robot software platform.

Robot subpackages are imported lazily so that using one robot does not
require the SDKs of the others (e.g. Crazyflie users don't need
``trossen_arm`` installed, and vice versa).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

__all__ = [
    'CrazyflieBase',
    'CrazyflieConfig',
    'CrazyflieController',
    'TrossenArmBase',
    'TrossenArmConfig',
    'TrossenArmController',
]

_CRAZYFLIE = {'CrazyflieBase', 'CrazyflieConfig', 'CrazyflieController'}
_TROSSEN = {'TrossenArmBase', 'TrossenArmConfig', 'TrossenArmController'}


def __getattr__(name: str):
    if name in _CRAZYFLIE:
        from irobot.src.robots import crazyflie

        return getattr(crazyflie, name)
    if name in _TROSSEN:
        from irobot.src.robots import trossen_arm

        return getattr(trossen_arm, name)
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


if TYPE_CHECKING:
    from irobot.src.robots.crazyflie import (
        CrazyflieBase,
        CrazyflieConfig,
        CrazyflieController,
    )
    from irobot.src.robots.trossen_arm import (
        TrossenArmBase,
        TrossenArmConfig,
        TrossenArmController,
    )
