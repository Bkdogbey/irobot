# General arm connectivity and motion test for the Trossen WXAI V0.
#
# Connects to the arm, moves to the home position, prints the current
# Cartesian end-effector pose, then parks the arm at sleep position.
# Run this first to confirm the arm is reachable and motion-ready.
#
# Usage:
#   conda run -n trossen_ai_env python setup/test_arm.py

import numpy as np
import trossen_arm

# ── Parameters — edit to match your arm ──────────────────────────────
ARM_IP       = "192.168.1.3"   # IP of the arm to test
END_EFFECTOR = trossen_arm.StandardEndEffector.wxai_v0_follower

HOME_POSITION  = np.array([0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0, 0.0])
SLEEP_POSITION = np.zeros(7)


if __name__ == '__main__':
    print(f"Connecting to arm at {ARM_IP}...")
    arm = trossen_arm.TrossenArmDriver()
    arm.configure(
        trossen_arm.Model.wxai_v0,
        END_EFFECTOR,
        ARM_IP,
        True,   # clear any pre-existing error state
    )
    print("Connected.")

    print("\nCurrent state:")
    cart = arm.get_cartesian_positions()
    joints = arm.get_all_positions()
    print(f"  Cartesian pose : {[round(v, 4) for v in cart]}")
    print(f"  Joint positions: {[round(v, 4) for v in joints]}")

    print("\nMoving to home position...")
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_all_positions(HOME_POSITION, 3.0, True)

    cart = arm.get_cartesian_positions()
    print(f"  End-effector at home: {[round(v, 4) for v in cart]}")

    print("\nMoving to sleep position (parking arm)...")
    arm.set_all_positions(SLEEP_POSITION, 3.0, True)

    print("\nArm test complete — arm is connected and motion-ready.")
