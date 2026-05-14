# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Purpose:
# follower1 autonomously picks up a triangular cake-server knife, then cuts a
# rectangular sheet cake (11" x 5" x 2") into 5 equal strips with 4 straight cuts.
#
# Hardware: follower1 only — WXAI V0 at 192.168.1.3
#
# Knife pre-placement:
#   Lay the Farberware cake server flat on a stable surface near the arm,
#   handle pointing toward the arm so the gripper can close around it.
#   Record its exact pose with gravity_comp + get_cartesian_positions() and
#   set the CALIBRATE constants below.
#
# Dry-run order:
#   1. Set CAKE_Z_CUT = Z_APPROACH  → knife hovers only, verify paths.
#   2. Set CAKE_Z_CUT = CAKE_Z_TOP + 0.01 → skim top, check alignment.
#   3. Set CAKE_Z_CUT = PLATFORM_Z + 0.005 → full cut.

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import time
import numpy as np
import trossen_arm
from cake_config import (
    GRIPPER_OPEN, GRIPPER_CLOSED,
    KNIFE_HOVER_JOINTS, CAKE_APPROACH_JOINTS,
    KNIFE_REST_X, KNIFE_REST_Y, KNIFE_REST_Z, KNIFE_REST_ORI,
    CAKE_CX, CAKE_CY, CAKE_HL, CAKE_HW,
    PLATFORM_Z, CAKE_Z_TOP, CAKE_Z_CUT, Z_APPROACH,
    KNIFE_ORI,
)

# ── Home / sleep — fixed for WXAI V0 ─────────────────────────────────
HOME_POSITION = np.array([0.0, np.pi / 2, np.pi / 2, 0.0, 0.0, 0.0, 0.0])


# ── Helpers ──────────────────────────────────────────────────────────

def go_home(arm: trossen_arm.TrossenArmDriver) -> None:
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_all_positions(HOME_POSITION, 4.0, True)


def go_sleep(arm: trossen_arm.TrossenArmDriver) -> None:
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_all_positions(np.zeros(arm.get_num_joints()), 4.0, True)


def move_cart(arm: trossen_arm.TrossenArmDriver,
              pose: np.ndarray,
              duration: float,
              interp: trossen_arm.InterpolationSpace = trossen_arm.InterpolationSpace.joint) -> None:
    arm.set_cartesian_positions(pose, interp, duration, True)


def move_joints(arm: trossen_arm.TrossenArmDriver,
                joints: np.ndarray,
                duration: float) -> None:
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_all_positions(joints, duration, True)


def set_gripper(arm: trossen_arm.TrossenArmDriver, pos: float) -> None:
    arm.set_all_modes(trossen_arm.Mode.position)
    arm.set_gripper_position(pos, 1.5, True)


# ── Knife pickup ──────────────────────────────────────────────────────

def pickup_knife(arm: trossen_arm.TrossenArmDriver) -> None:
    """Open gripper, move to knife in joint space, descend in Cartesian, grip, lift."""
    rx, ry, rz = KNIFE_REST_X, KNIFE_REST_Y, KNIFE_REST_Z

    set_gripper(arm, GRIPPER_OPEN)

    # Large move in joint space — guaranteed reachable
    move_joints(arm, KNIFE_HOVER_JOINTS, 3.0)
    # Short vertical descend in Cartesian — small move, IK reliable from this config
    move_cart(arm, np.array([rx, ry, rz, *KNIFE_REST_ORI]), 2.0)
    # Grip
    set_gripper(arm, GRIPPER_CLOSED)
    # Lift back to hover in joint space
    move_joints(arm, KNIFE_HOVER_JOINTS, 2.0)


# ── Cutting ───────────────────────────────────────────────────────────

def make_cut(arm: trossen_arm.TrossenArmDriver,
             x0: float, y0: float,
             x1: float, y1: float) -> None:
    """One straight cut from (x0,y0) to (x1,y1), blade perpendicular to cake."""
    move_cart(arm, np.array([x0, y0, Z_APPROACH,  *KNIFE_ORI]), 2.0)  # hover above start
    move_cart(arm, np.array([x0, y0, CAKE_Z_TOP,  *KNIFE_ORI]), 1.0)  # touch cake top
    move_cart(arm, np.array([x0, y0, CAKE_Z_CUT,  *KNIFE_ORI]), 1.5)  # plunge through
    move_cart(arm, np.array([x1, y1, CAKE_Z_CUT,  *KNIFE_ORI]), 3.5,
              trossen_arm.InterpolationSpace.cartesian)  # straight-line drag
    move_cart(arm, np.array([x1, y1, Z_APPROACH,  *KNIFE_ORI]), 1.5)  # lift clear


# ── Constant validation ───────────────────────────────────────────────

def validate_constants() -> None:
    """Catch obviously wrong calibration values before the arm moves."""
    errors = []

    if Z_APPROACH < CAKE_Z_TOP + 0.05:
        errors.append(
            f"Z_APPROACH ({Z_APPROACH:.4f} m) must be at least 5 cm above "
            f"CAKE_Z_TOP ({CAKE_Z_TOP:.4f} m).\n"
            f"    → Re-run calibrate_arm.py and record CAKE_APPROACH with the "
            f"knife hovering well above the cake, not near the surface."
        )
    if CAKE_Z_TOP <= CAKE_Z_CUT:
        errors.append(
            f"CAKE_Z_TOP ({CAKE_Z_TOP:.4f} m) must be higher than "
            f"CAKE_Z_CUT ({CAKE_Z_CUT:.4f} m)."
        )
    if CAKE_Z_CUT <= PLATFORM_Z:
        errors.append(
            f"CAKE_Z_CUT ({CAKE_Z_CUT:.4f} m) must be above "
            f"PLATFORM_Z ({PLATFORM_Z:.4f} m)."
        )

    print("\n── Calibration constants ────────────────────────────────")
    print(f"  PLATFORM_Z  = {PLATFORM_Z:.4f} m  (table surface)")
    print(f"  CAKE_Z_CUT  = {CAKE_Z_CUT:.4f} m  (knife fully through cake)")
    print(f"  CAKE_Z_TOP  = {CAKE_Z_TOP:.4f} m  (top of cake)")
    print(f"  Z_APPROACH  = {Z_APPROACH:.4f} m  (safe hover height)")
    print(f"  CAKE centre = ({CAKE_CX:.4f}, {CAKE_CY:.4f}) m")
    print(f"  KNIFE_ORI   = {list(KNIFE_ORI)}")

    if errors:
        print("\nCALIBRATION ERROR — fix these before running:")
        for e in errors:
            print(f"  ✗ {e}")
        raise ValueError("Invalid calibration constants — see above.")

    print("  Constants look valid.\n")


# ── Main sequence ─────────────────────────────────────────────────────

if __name__ == '__main__':
    cx, cy = CAKE_CX, CAKE_CY
    hl, hw = CAKE_HL, CAKE_HW

    validate_constants()

    print("Initializing driver...")
    arm = trossen_arm.TrossenArmDriver()

    print("Configuring driver...")
    arm.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        "192.168.1.3",
        True,   # clear any pre-existing error state
    )

    try:
        print("Gripping knife...")
        set_gripper(arm, GRIPPER_OPEN)
        set_gripper(arm, GRIPPER_CLOSED)
        time.sleep(2)
        set_gripper(arm, GRIPPER_CLOSED)  # re-squeeze after settle
        time.sleep(2)

        print("Moving to home position...")
        go_home(arm)

        # Move to above-cake position in joint space before handing off to Cartesian cuts
        print("Moving to cake approach position...")
        move_joints(arm, CAKE_APPROACH_JOINTS, 3.0)

        # 4 cuts along the 10.5" length, evenly spaced across the 8.5" width
        # Divides cake into 5 equal strips

        print("Cut 1/4 — along length at y = cy - 3*HW/5...")
        make_cut(arm, cx + hl, cy - 3*hw/5, cx - hl, cy - 3*hw/5)

        print("Cut 2/4 — along length at y = cy - HW/5...")
        make_cut(arm, cx + hl, cy - hw/5, cx - hl, cy - hw/5)

        print("Cut 3/4 — along length at y = cy + HW/5...")
        make_cut(arm, cx + hl, cy + hw/5, cx - hl, cy + hw/5)

        print("Cut 4/4 — along length at y = cy + 3*HW/5...")
        make_cut(arm, cx + hl, cy + 3*hw/5, cx - hl, cy + 3*hw/5)

        print("Done — cake cut into 5 equal strips.")

    except Exception as e:
        print(f"ERROR during sequence: {e}")

    finally:
        print("Returning to home and sleep...")
        try:
            go_home(arm)
            go_sleep(arm)
        except Exception:
            pass
