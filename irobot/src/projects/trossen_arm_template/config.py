from __future__ import annotations

import trossen_arm
from attrs import define
from ros_sugar.config import BaseComponentConfig

from irobot.src.robots.trossen_arm.core.base import TrossenArmConfig

# ── Lab arms ─────────────────────────────────────────────────────────────────
# All 4 physical arms in the iHuman Lab. Set ACTIVE_ARM to the one you want.
# PC must be on 192.168.1.0/24 (e.g. 192.168.1.1). See robots/trossen_arm/README.md.

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

# ── Select arm ───────────────────────────────────────────────────────────────
ACTIVE_ARM: TrossenArmConfig = FOLLOWER_1  # ← change this before each run

# ── Experiment parameters ─────────────────────────────────────────────────────
TASK_DURATION: float = 30.0   # seconds

# ── ROS Sugar component config ────────────────────────────────────────────────
@define(kw_only=True)
class TrossenArmComponentConfig(BaseComponentConfig):
    arm_config: TrossenArmConfig = ACTIVE_ARM
