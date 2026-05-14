import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import trossen_arm
from cake_config import (
    GRIPPER_OPEN, GRIPPER_CLOSED,
    KNIFE_HOVER_JOINTS, KNIFE_GRIP_JOINTS, CAKE_APPROACH_JOINTS,
    CAKE_CX, CAKE_CY,
    PLATFORM_Z, CAKE_Z_TOP, CAKE_Z_CUT, Z_APPROACH,
    KNIFE_ORI,
)

# ── Round cake geometry ───────────────────────────────────────────────
# 6.5" diameter, 2" tall — same height as rectangular cake so Z constants unchanged
CAKE_R = (6.5 * 0.0254) / 2   # 0.08255 m

# ── Timing ────────────────────────────────────────────────────────────
# Slower than cake_cut.py to reduce knife deflection on long blade
PLUNGE_DURATION = 2.5
CUT_DURATION    = 5.0

# ── Fixed home ────────────────────────────────────────────────────────
HOME_POSITION = np.array([0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0, 0.0])


# ── Helpers ──────────────────────────────────────────────────────────

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
    arm.set_gripper_position(pos, 0.8, True)


# ── Validation ────────────────────────────────────────────────────────

def validate_constants():
    errors = []
    if Z_APPROACH < CAKE_Z_TOP + 0.05:
        errors.append(
            f"Z_APPROACH ({Z_APPROACH:.4f} m) must be >= CAKE_Z_TOP ({CAKE_Z_TOP:.4f} m) + 5 cm."
        )
    if CAKE_Z_TOP <= CAKE_Z_CUT:
        errors.append(
            f"CAKE_Z_TOP ({CAKE_Z_TOP:.4f} m) must be above CAKE_Z_CUT ({CAKE_Z_CUT:.4f} m)."
        )
    if CAKE_Z_CUT <= PLATFORM_Z:
        errors.append(
            f"CAKE_Z_CUT ({CAKE_Z_CUT:.4f} m) must be above PLATFORM_Z ({PLATFORM_Z:.4f} m)."
        )

    print("\n── Calibration constants ────────────────────────────────")
    print(f"  PLATFORM_Z  = {PLATFORM_Z:.4f} m")
    print(f"  CAKE_Z_CUT  = {CAKE_Z_CUT:.4f} m")
    print(f"  CAKE_Z_TOP  = {CAKE_Z_TOP:.4f} m")
    print(f"  Z_APPROACH  = {Z_APPROACH:.4f} m")
    print(f"  CAKE centre = ({CAKE_CX:.4f}, {CAKE_CY:.4f}) m")
    print(f"  CAKE_R      = {CAKE_R:.4f} m  (6.5\" diameter)")

    if errors:
        print("\nCALIBRATION ERROR:")
        for e in errors:
            print(f"  ✗ {e}")
        raise ValueError("Invalid calibration constants — see above.")

    print("  Constants look valid.\n")


# ── Cutting ───────────────────────────────────────────────────────────

def make_cut(arm, x0, y0, x1, y1):
    move_cart(arm, np.array([x0, y0, Z_APPROACH,  *KNIFE_ORI]), 2.0)
    move_cart(arm, np.array([x0, y0, CAKE_Z_TOP,  *KNIFE_ORI]), 1.0)
    move_cart(arm, np.array([x0, y0, CAKE_Z_CUT,  *KNIFE_ORI]), PLUNGE_DURATION)
    move_cart(arm, np.array([x1, y1, CAKE_Z_CUT,  *KNIFE_ORI]), CUT_DURATION)
    move_cart(arm, np.array([x1, y1, Z_APPROACH,  *KNIFE_ORI]), 1.5)


# ── Main sequence ─────────────────────────────────────────────────────

if __name__ == '__main__':
    validate_constants()

    cx, cy, r = CAKE_CX, CAKE_CY, CAKE_R
    d = r * np.sqrt(2) / 2   # r·cos45 = r·sin45

    # Uncomment cuts progressively to increase slice count:
    cuts = [
        (cx - r,  cy,      cx + r,  cy     ),   # cut 1 — 0°    → 2 slices
        # (cx - d,  cy - d,  cx + d,  cy + d),  # cut 2 — 45°   → 4 slices
        # (cx,      cy - r,  cx,      cy + r),   # cut 3 — 90°   → 6 slices
        # (cx + d,  cy - d,  cx - d,  cy + d),   # cut 4 — 135°  → 8 slices
    ]

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

        print("Picking up knife...")
        set_gripper(arm, GRIPPER_OPEN)
        move_joints(arm, KNIFE_HOVER_JOINTS, 3.0)
        move_joints(arm, KNIFE_GRIP_JOINTS, 2.0)
        set_gripper(arm, GRIPPER_CLOSED)
        move_joints(arm, KNIFE_HOVER_JOINTS, 2.0)

        print("Moving to cake approach position...")
        move_joints(arm, CAKE_APPROACH_JOINTS, 3.0)

        for i, (x0, y0, x1, y1) in enumerate(cuts, 1):
            print(f"Cut {i}/4...")
            make_cut(arm, x0, y0, x1, y1)

        print("Done — round cake cut into 8 equal slices.")

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        print("Returning to home and sleep...")
        try:
            go_home(arm)
            go_sleep(arm)
        except Exception:
            pass
