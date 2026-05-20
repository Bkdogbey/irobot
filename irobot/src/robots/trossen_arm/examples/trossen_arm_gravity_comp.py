"""
Example: Kinesthetic teaching via gravity compensation.

Connects to an arm, enters gravity compensation mode so the arm floats
under its own weight, then waits for Enter before parking and disconnecting.
Use this to manually guide the arm to record poses.
"""

from __future__ import annotations

from irobot.src.robots.trossen_arm import TrossenArmController
from irobot.src.robots.trossen_arm.config import FOLLOWER_1

if __name__ == '__main__':
    arm = TrossenArmController(FOLLOWER_1)

    try:
        arm.go_home()
        arm.gravity_comp()
        print('Gravity compensation active. Guide the arm to desired poses.')
        print('Press Enter to exit and park the arm.')
        input()
    finally:
        arm.disconnect()
