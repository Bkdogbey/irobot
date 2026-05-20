from __future__ import annotations

from dataclasses import dataclass, field

import trossen_arm


@dataclass
class TrossenArmConfig:
    """Hardware configuration for one Trossen WXAI V0 arm.

    This is the single place to change when targeting a different arm,
    IP address, end-effector type, or gripper range.
    """

    ip_address: str = '192.168.1.2'
    model: trossen_arm.Model = field(default_factory=lambda: trossen_arm.Model.wxai_v0)
    end_effector: trossen_arm.StandardEndEffector = field(
        default_factory=lambda: trossen_arm.StandardEndEffector.wxai_v0_follower
    )
    clear_error_on_connect: bool = True
    gripper_open_pos: float = 0.035   # metres; fully open for WXAI V0
    gripper_close_pos: float = 0.0    # metres; fully closed


# ── Lab arm presets ───────────────────────────────────────────────────────────
# The iHuman Lab has four arms on 192.168.1.0/24. Your PC must have a static IP
# on the same subnet (e.g. 192.168.1.1). See README.md for network setup.

LEADER_1 = TrossenArmConfig(
    ip_address='192.168.1.2',
    end_effector=trossen_arm.StandardEndEffector.wxai_v0_leader,
)
FOLLOWER_1 = TrossenArmConfig(
    ip_address='192.168.1.3',
    end_effector=trossen_arm.StandardEndEffector.wxai_v0_follower,
)
LEADER_2 = TrossenArmConfig(
    ip_address='192.168.1.4',
    end_effector=trossen_arm.StandardEndEffector.wxai_v0_leader,
)
FOLLOWER_2 = TrossenArmConfig(
    ip_address='192.168.1.5',
    end_effector=trossen_arm.StandardEndEffector.wxai_v0_follower,
)
