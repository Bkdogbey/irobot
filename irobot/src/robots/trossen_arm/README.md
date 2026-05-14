# Trossen WXAI V0 Arm — Setup Guide

## What is this?

The Trossen WidowX AI (WXAI) V0 is a 6-DOF robotic manipulator arm with an integrated
gripper (7 joints total). The iHuman Lab has **four arms** on a shared Ethernet subnet,
configured as two leader-follower pairs for bimanual teleoperation and learning from
demonstration experiments.

The `TrossenArmBase` class in `core/base.py` wraps the vendor Python driver into a clean
interface with connect/disconnect lifecycle, cached state, and convenience methods. No
ROS2 required — everything runs with plain Python.

---

## 1. Install the Python driver

### Using conda (lab default)

The lab environment is `trossen_ai_env`. Activate it and install the driver:

```bash
conda activate trossen_ai_env
pip install trossen-arm
```

### Using pip directly

```bash
pip install trossen-arm
```

**Firmware compatibility:** The driver's `major.minor` version must match the controller
firmware exactly — for example, driver `v1.3.x` only works with firmware `v1.3.x`. Check
your installed version with:

```bash
pip show trossen-arm
```

If the versions don't match, either upgrade the driver (`pip install --upgrade trossen-arm`)
or upgrade the firmware (see [Section 4](#4-firmware-upgrade)).

---

## 2. Ethernet setup

The arms communicate over a wired Ethernet connection. Your PC needs a **static IP** on
the `192.168.1.0/24` subnet to reach them.

### Controller factory defaults

Every arm ships with these network settings:

| Setting | Value |
|---|---|
| IP address | `192.168.1.2` |
| Subnet mask | `255.255.255.0` |
| Gateway | `192.168.1.1` |
| DNS | `8.8.8.8` |

> Arms 3, 4, and 5 in the lab have been reassigned from the factory default using
> `trossen_cake/setup/set_manual_ip.py`. A new arm out of the box will always appear at
> `192.168.1.2` until reconfigured.

### Set PC static IP — Ubuntu (Network Manager GUI)

1. Open **Settings → Network → Wired** (or your Ethernet connection)
2. Click the gear icon → **IPv4** tab
3. Set **Method** to `Manual`
4. Add address: IP `192.168.1.1`, Netmask `255.255.255.0`, Gateway (leave blank)
5. Apply and reconnect the Ethernet cable

### Set PC static IP — Ubuntu (command line)

```bash
# Replace eth0 with your interface name (check with: ip link)
sudo ip addr add 192.168.1.1/24 dev eth0
sudo ip link set eth0 up
```

> This is temporary and resets on reboot. To make it permanent, use the GUI method or
> add a Netplan configuration.

---

## 3. Verify connectivity

Before running any code, confirm your PC can reach the arm:

```bash
ping 192.168.1.3   # replace with your arm's IP
```

To discover all arms currently reachable on the subnet:

```bash
python -m trossen_arm.demos.arm_discovery
```

---

## 4. Firmware upgrade

Only needed when the driver and controller firmware versions diverge. Do **not** downgrade
firmware without lab approval — downgrading resets arm calibration.

### Step 1 — Back up configuration

Before flashing, use the compatible version of the driver to save joint calibration and
PID gains (use `setup/configure_trossen.py` in any project to print current settings).

### Step 2 — Install Teensy Loader CLI

```bash
sudo apt install build-essential libusb-dev
```

Then install the CLI tool following the
[Teensy Loader CLI instructions](https://www.pjrc.com/teensy/loader_cli.html).

### Step 3 — Flash the firmware

Download the firmware `.hex` file for your target version from the Trossen releases page,
then run:

```bash
teensy_loader_cli --mcu=TEENSY41 -s firmware-wxai_v0.hex
```

### Step 4 — Reinstall matching driver

```bash
pip install trossen-arm==<version>   # must match firmware major.minor
```

---

## 5. Lab arm reference

| Label | Role | End Effector | IP Address |
|---|---|---|---|
| Leader 1 | leader (input) | `wxai_v0_leader` | `192.168.1.2` |
| Follower 1 | follower (actuating) | `wxai_v0_follower` | `192.168.1.3` |
| Leader 2 | leader (input) | `wxai_v0_leader` | `192.168.1.4` |
| Follower 2 | follower (actuating) | `wxai_v0_follower` | `192.168.1.5` |

Leader arms have a different end-effector configuration — they are used as input devices,
not for grasping. Follower arms are the actuating arms used in experiments.

---

## 6. Quick test

### Using the raw vendor driver

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

### Using `TrossenArmBase` (recommended for projects)

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

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `connect()` returns `False` | Wrong subnet or arm not powered | Check PC IP is `192.168.1.x`; verify arm power LED |
| `ping` times out | Firewall blocking ICMP | `sudo ufw allow from 192.168.1.0/24` |
| Driver raises version error | Firmware/driver version mismatch | Check `pip show trossen-arm` and compare to controller firmware; see Section 4 |
| Arm moves erratically | Error state not cleared | Set `clear_error_on_connect=True` in `TrossenArmConfig` (default) |
| `cleanup()` hangs | Driver in bad state | Call `arm.e_stop()` which forcefully cleans up |
| New arm not reachable | Still on factory IP `192.168.1.2` | Use `set_manual_ip.py` to reassign before connecting |
