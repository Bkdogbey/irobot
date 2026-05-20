"""
Example: Single leader→follower teleoperation with force feedback.

The leader arm (input device) drives the follower arm (actuator). Force
feedback is applied back to the leader so the operator feels resistance
when the follower encounters external forces.

Hardware:
  Leader:   WXAI V0 with wxai_v0_leader end effector  at 192.168.1.4 (LEADER_2)
  Follower: WXAI V0 with wxai_v0_follower end effector at 192.168.1.5 (FOLLOWER_2)
"""

from __future__ import annotations

import time

import numpy as np
import trossen_arm

from irobot.src.robots.trossen_arm import TrossenArmController
from irobot.src.robots.trossen_arm.config import FOLLOWER_2, LEADER_2

TELEOPERATION_DURATION: float = 30.0   # seconds
FORCE_FEEDBACK_GAIN: float = 0.1

if __name__ == '__main__':
    print('Connecting to arms...')
    leader = TrossenArmController(LEADER_2)
    follower = TrossenArmController(FOLLOWER_2)

    try:
        print('Moving to home positions...')
        leader.go_home()
        follower.go_home()

        print('Starting teleoperation. Release the leader when time expires.')
        time.sleep(1.0)

        leader.set_mode(trossen_arm.Mode.external_effort)
        follower.set_mode(trossen_arm.Mode.position)

        end_time = time.time() + TELEOPERATION_DURATION
        while time.time() < end_time:
            # Force feedback: scale follower external efforts back to leader
            leader.stream_external_efforts(
                -FORCE_FEEDBACK_GAIN * np.array(follower.get_external_efforts())
            )
            # Position mirror: send leader positions + velocity feedforward to follower
            follower.stream_positions(
                leader.get_positions(),
                velocities=leader.get_velocities(),
            )

        print('Teleoperation complete.')

    finally:
        leader.disconnect()
        follower.disconnect()
