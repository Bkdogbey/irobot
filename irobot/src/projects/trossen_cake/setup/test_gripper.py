# General gripper test for the Trossen WXAI V0 arm.
#
# Cycles the gripper open → closed → open and prints the actual position
# after each step. Run this to verify the gripper motor is responding and
# that the open/closed range looks correct before running any project script.
#
# Usage:
#   conda run -n trossen_ai_env python setup/test_gripper.py

import trossen_arm

# ── Parameters — edit to match your arm ──────────────────────────────
ARM_IP     = "192.168.1.3"   # IP of the arm to test
OPEN_POS   = 0.035           # m — wide open (adjust if your gripper range differs)
CLOSED_POS = 0.0             # m — fully closed
DURATION   = 1.5             # seconds per gripper command


def get_gripper_pos(arm: trossen_arm.TrossenArmDriver) -> float:
    return arm.get_robot_output().joint.all.positions[-1]


if __name__ == '__main__':
    print(f"Connecting to arm at {ARM_IP}...")
    arm = trossen_arm.TrossenArmDriver()
    arm.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        ARM_IP,
        True,
    )
    arm.set_all_modes(trossen_arm.Mode.position)

    print(f"\nInitial gripper position : {get_gripper_pos(arm):.4f} m")

    print(f"\nOpening gripper  → target {OPEN_POS} m ...")
    arm.set_gripper_position(OPEN_POS, DURATION, True)
    print(f"  Actual position : {get_gripper_pos(arm):.4f} m")

    print(f"\nClosing gripper  → target {CLOSED_POS} m ...")
    arm.set_gripper_position(CLOSED_POS, DURATION, True)
    print(f"  Actual position : {get_gripper_pos(arm):.4f} m")

    print(f"\nOpening gripper  → target {OPEN_POS} m ...")
    arm.set_gripper_position(OPEN_POS, DURATION, True)
    print(f"  Actual position : {get_gripper_pos(arm):.4f} m")

    print("\nGripper test complete.")
