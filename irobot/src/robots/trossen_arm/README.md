# Trossen WXAI V0 Arm — Hardware Setup Guide

## What is this?

The Trossen WidowX AI (WXAI) V0 is a 6-DOF robotic manipulator arm with an integrated gripper (7 joints total). The iHuman Lab has **four arms** on a shared Ethernet subnet, configured as two leader-follower pairs for bimanual teleoperation and learning from demonstration experiments.

The `TrossenArmBase` class in `core/base.py` wraps the vendor Python driver into a clean interface with connect/disconnect lifecycle, cached state, and convenience methods. No ROS2 required — everything runs with plain Python.

---

## 1. Install the Python driver

```bash
pip install trossen-arm
```

Supported: Python 3.10–3.13, Ubuntu 20.04/22.04/24.04.

**Firmware compatibility:** The driver's major.minor version must match the controller firmware exactly (e.g. driver v1.3.x only works with firmware v1.3.x). Downgrading firmware resets calibration — do not do it without lab approval.

---

## 2. Ethernet setup

The arms communicate over Ethernet. The PC must have a **manual static IP** on the `192.168.1.0/24` subnet.

### Ubuntu (Network Manager GUI)

1. Open **Settings → Network → Wired** (or the relevant Ethernet connection)
2. Click the gear icon → **IPv4** tab
3. Set **Method** to `Manual`
4. Add an address: IP `192.168.1.1`, Netmask `255.255.255.0`, Gateway (leave blank)
5. Apply and reconnect the Ethernet cable

### Ubuntu (command line)

```bash
# Replace eth0 with your interface name (check with: ip link)
sudo ip addr add 192.168.1.1/24 dev eth0
sudo ip link set eth0 up
```

To make this permanent, add a Netplan config or use the GUI method above.

---

## 3. Lab arm reference

| Label | Role | End Effector | IP Address |
|---|---|---|---|
| Leader 1 (left) | leader | `wxai_v0_leader` | `192.168.1.2` |
| Follower 1 (right) | follower | `wxai_v0_follower` | `192.168.1.3` |
| Leader 2 (left) | leader | `wxai_v0_leader` | `192.168.1.4` |
| Follower 2 (right) | follower | `wxai_v0_follower` | `192.168.1.5` |

Leader arms have a different end-effector configuration (no grasping — used as input devices). Follower arms are the actuating arms.

---

## 4. Verify connectivity

Before running any code, confirm the arm is reachable:

```bash
ping 192.168.1.3   # replace with your arm's IP
```

You can also run the Trossen SDK discovery demo to check all arms on the subnet:

```bash
python -m trossen_arm.demos.arm_discovery
```

---

## 5. Quick test

This snippet configures one arm, moves it to the home position, parks it, and disconnects:

```python
import trossen_arm
import numpy as np

arm = trossen_arm.TrossenArmDriver()
arm.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_follower,
    '192.168.1.3',   # IP of Follower 1
    True,            # clear error state on connect
)
arm.set_all_modes(trossen_arm.Mode.position)
arm.set_all_positions(np.array([0.0, np.pi/2, np.pi/2, 0.0, 0.0, 0.0, 0.0]), 3.0, True)  # home
arm.set_all_positions(np.zeros(7), 3.0, True)  # sleep/park
arm.cleanup()
```

Or using `TrossenArmBase`:

```python
from irobot.src.robots.trossen_arm.core.base import TrossenArmBase, TrossenArmConfig
import trossen_arm

arm = TrossenArmBase(TrossenArmConfig(
    ip_address='192.168.1.3',
    end_effector=trossen_arm.StandardEndEffector.wxai_v0_follower,
))
arm.connect()
arm.go_home()
arm.disconnect()   # automatically goes to sleep before disconnecting
```

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `connect()` returns `False` | Wrong subnet or arm not powered | Check PC IP is `192.168.1.x`; verify arm power LED |
| `ping` times out | Firewall blocking ICMP | `sudo ufw allow from 192.168.1.0/24` |
| Driver raises version error | Firmware/driver mismatch | Check `pip show trossen-arm` and compare to controller firmware version |
| Arm moves erratically | Error state not cleared | Set `clear_error_on_connect=True` in config (default) |
| `cleanup()` hangs | Driver in bad state | Call `arm.e_stop()` which forcefully cleans up |
