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
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Hardware setup:
# Pair 1: leader at 192.168.1.2, follower at 192.168.1.3
# Pair 2: leader at 192.168.1.4, follower at 192.168.1.5

import time
import numpy as np
import trossen_arm

HOME_POSITION = np.array([0.0, np.pi/2, np.pi/2, 0.0, 0.0, 0.0, 0.0])

if __name__ == '__main__':
    teleoperation_time = 120
    force_feedback_gain = 0.1

    print("Initializing drivers...")
    leader1   = trossen_arm.TrossenArmDriver()
    follower1 = trossen_arm.TrossenArmDriver()
    leader2   = trossen_arm.TrossenArmDriver()
    follower2 = trossen_arm.TrossenArmDriver()

    print("Configuring drivers...")
    leader1.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_leader,
        "192.168.1.2",
        False
    )
    follower1.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        "192.168.1.3",
        False
    )
    leader2.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_leader,
        "192.168.1.4",
        False
    )
    follower2.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        "192.168.1.5",
        False
    )

    print("Moving to home positions...")
    for driver in (leader1, follower1, leader2, follower2):
        driver.set_all_modes(trossen_arm.Mode.position)
        driver.set_all_positions(HOME_POSITION, 2.0, True)

    print("Starting teleoperation (both pairs)...")
    time.sleep(1)
    leader1.set_all_modes(trossen_arm.Mode.external_effort)
    follower1.set_all_modes(trossen_arm.Mode.position)
    leader2.set_all_modes(trossen_arm.Mode.external_effort)
    follower2.set_all_modes(trossen_arm.Mode.position)

    end_time = time.time() + teleoperation_time
    while time.time() < end_time:
        # Pair 1
        leader1.set_all_external_efforts(
            -force_feedback_gain * np.array(follower1.get_all_external_efforts()),
            0.0,
            False,
        )
        follower1.set_all_positions(
            leader1.get_all_positions(),
            0.0,
            False,
            leader1.get_all_velocities()
        )
        # Pair 2
        leader2.set_all_external_efforts(
            -force_feedback_gain * np.array(follower2.get_all_external_efforts()),
            0.0,
            False,
        )
        follower2.set_all_positions(
            leader2.get_all_positions(),
            0.0,
            False,
            leader2.get_all_velocities()
        )

    print("Moving to home positions...")
    for driver in (leader1, follower1, leader2, follower2):
        driver.set_all_modes(trossen_arm.Mode.position)
        driver.set_all_positions(HOME_POSITION, 2.0, True)

    print("Moving to sleep positions...")
    for driver in (leader1, follower1, leader2, follower2):
        driver.set_all_modes(trossen_arm.Mode.position)
        driver.set_all_positions(np.zeros(driver.get_num_joints()), 2.0, True)
