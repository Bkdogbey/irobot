import numpy as np
import trossen_arm
from cake_config import (
    GRIPPER_OPEN, GRIPPER_CLOSED,
    KNIFE_HOVER_JOINTS,
    KNIFE_REST_X, KNIFE_REST_Y, KNIFE_REST_Z, KNIFE_REST_ORI,
    Z_APPROACH, KNIFE_ORI,
    CAKE_CX, CAKE_CY,
)

HOME_POSITION = np.array([0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0, 0.0])

# How far to dip below hover during the grip test (metres)
DIP_DEPTH = 0.15


def go_home(arm):
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_all_positions(HOME_POSITION, 4.0, True)


def go_sleep(arm):
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_all_positions(np.zeros(arm.get_num_joints()), 4.0, True)


def move_cart(arm, pose, duration):
    arm.set_cartesian_positions(pose, trossen_arm.InterpolationSpace.joint, duration, True)


def move_joints(arm, joints, duration):
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_all_positions(joints, duration, True)


def set_gripper(arm, pos):
    arm.set_gripper_position(pos, 1.5, True)


if __name__ == '__main__':
    print("Initializing driver...")
    arm = trossen_arm.TrossenArmDriver()

    print("Configuring driver...")
    arm.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        "192.168.1.3",
        True,
    )

    try:
        print("Moving to home...")
        go_home(arm)

        # ── Pick up knife ──────────────────────────────────────────────
        print("Opening gripper...")
        set_gripper(arm, GRIPPER_OPEN)

        print("Moving to knife hover (joint space)...")
        move_joints(arm, KNIFE_HOVER_JOINTS, 3.0)

        print("Descending to knife rest (Cartesian)...")
        move_cart(arm, np.array([KNIFE_REST_X, KNIFE_REST_Y, KNIFE_REST_Z, *KNIFE_REST_ORI]), 2.0)

        print("Closing gripper...")
        set_gripper(arm, GRIPPER_CLOSED)

        print("Lifting back to hover (joint space)...")
        move_joints(arm, KNIFE_HOVER_JOINTS, 2.0)

        # ── Grip test: dip down and back up ───────────────────────────
        hover_pose = np.array([CAKE_CX, CAKE_CY, Z_APPROACH, *KNIFE_ORI])
        dip_pose   = np.array([CAKE_CX, CAKE_CY, Z_APPROACH - DIP_DEPTH, *KNIFE_ORI])

        print("Moving to test position (above cake centre)...")
        move_cart(arm, hover_pose, 2.0)

        print("Dipping down...")
        move_cart(arm, dip_pose, 1.5)

        print("Rising back up...")
        move_cart(arm, hover_pose, 1.5)

        print("Grip test complete — knife held through dip cycle.")

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        print("Returning to home and sleep...")
        try:
            go_home(arm)
            go_sleep(arm)
        except Exception:
            pass
