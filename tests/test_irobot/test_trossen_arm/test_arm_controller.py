"""Safety tests for TrossenArmController.disconnect().

These tests verify the two most consequential correctness properties of the
controller without requiring physical hardware:

1. go_sleep() is called before driver.cleanup() on a normal disconnect.
2. driver.cleanup() still runs even if go_sleep() raises an exception.
"""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, patch

import pytest

# Stub both vendor libraries before any irobot imports so the modules load
# without the real hardware packages being installed.
_trossen_arm_stub = MagicMock()
_trossen_arm_stub.Model.wxai_v0 = 'wxai_v0'
_trossen_arm_stub.StandardEndEffector.wxai_v0_follower = 'wxai_v0_follower'
_trossen_arm_stub.Mode.position = 'position'
_trossen_arm_stub.InterpolationSpace.joint = 'joint'

_cflib_stub = MagicMock()

_stubs = {
    'trossen_arm': _trossen_arm_stub,
    'cflib': _cflib_stub,
    'cflib.crazyflie': _cflib_stub.crazyflie,
    'cflib.crazyflie.log': _cflib_stub.crazyflie.log,
    'cflib.crtp': _cflib_stub.crtp,
}

for _name, _mod in _stubs.items():
    sys.modules.setdefault(_name, _mod)

from irobot.src.robots.trossen_arm.core.controller import TrossenArmController  # noqa: E402


@pytest.fixture()
def controller() -> TrossenArmController:
    """Return a TrossenArmController wired up as if already connected."""
    with patch.object(TrossenArmController, 'connect', return_value=True):
        arm = TrossenArmController.__new__(TrossenArmController)
    arm.config = MagicMock()
    arm.config.ip_address = '192.168.1.3'
    arm.driver = MagicMock()
    arm.is_connected = True
    return arm


def test_go_sleep_called_before_cleanup(controller: TrossenArmController) -> None:
    """go_sleep() must complete before driver.cleanup() is called."""
    call_order: list[str] = []
    controller.go_sleep = MagicMock(side_effect=lambda *a, **kw: call_order.append('go_sleep'))
    controller.driver.cleanup = MagicMock(side_effect=lambda: call_order.append('cleanup'))

    controller.disconnect()

    assert call_order == ['go_sleep', 'cleanup'], (
        f'Expected go_sleep before cleanup, got: {call_order}'
    )


def test_cleanup_runs_even_if_go_sleep_raises(controller: TrossenArmController) -> None:
    """driver.cleanup() must be called even when go_sleep() raises."""
    controller.go_sleep = MagicMock(side_effect=RuntimeError('motion failed'))
    controller.driver.cleanup = MagicMock()

    controller.disconnect()   # must not propagate the RuntimeError

    controller.driver.cleanup.assert_called_once()
