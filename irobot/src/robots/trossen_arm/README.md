# Trossen WXAI V0 Arm

The Trossen WidowX AI (WXAI) V0 is a 6-DOF robotic manipulator with an integrated gripper (7 joints total). The iHuman Lab has **four arms** on a shared Ethernet subnet, configured as two leader-follower pairs for bimanual teleoperation and learning from demonstration experiments.

`TrossenArmController` (in `core/controller.py`) is the main entry point: it wraps the vendor Python driver with a clean connect/disconnect lifecycle, cached state, and all motion primitives. No ROS2 required.

---

## Architecture

```
TrossenArmBase (core/base.py)
  └── TrossenArmController (core/controller.py)
```

- **`TrossenArmBase`** — connection lifecycle (`connect`, `disconnect`, `e_stop`), state polling (`_poll_state`), cached state fields
- **`TrossenArmController`** — named poses (`go_home`, `go_sleep`), joint/Cartesian/gripper commands, gravity compensation, streaming helpers for teleoperation

---

## 1. Install the Python driver

```bash
pip install trossen-arm
```

**Firmware compatibility:** The driver's `major.minor` version must match the controller firmware exactly. Check your installed version:

```bash
pip show trossen-arm
```

---

## 2. Ethernet setup

Arms communicate over wired Ethernet. Your PC needs a **static IP** on `192.168.1.0/24`.

**Controller factory defaults:**

| Setting      | Value           |
|---|---|
| IP address   | `192.168.1.2`   |
| Subnet mask  | `255.255.255.0` |
| Gateway      | `192.168.1.1`   |

**Set PC static IP (Ubuntu GUI):**
1. Settings → Network → Wired → gear → IPv4
2. Method: Manual — IP: `192.168.1.1`, Netmask: `255.255.255.0`

**Set PC static IP (command line, temporary):**
```bash
sudo ip addr add 192.168.1.1/24 dev eth0   # replace eth0 with your interface
sudo ip link set eth0 up
```

---

## 3. Lab arm reference

| Label      | Role           | End Effector         | IP           | Config preset |
|---|---|---|---|---|
| Leader 1   | input device   | `wxai_v0_leader`     | `192.168.1.2` | `LEADER_1`   |
| Follower 1 | actuating arm  | `wxai_v0_follower`   | `192.168.1.3` | `FOLLOWER_1` |
| Leader 2   | input device   | `wxai_v0_leader`     | `192.168.1.4` | `LEADER_2`   |
| Follower 2 | actuating arm  | `wxai_v0_follower`   | `192.168.1.5` | `FOLLOWER_2` |

Import presets from `config.py`:
```python
from irobot.src.robots.trossen_arm.config import FOLLOWER_1, LEADER_2
```

---

## 4. Quick start

```python
from irobot.src.robots.trossen_arm import TrossenArmController
from irobot.src.robots.trossen_arm.config import FOLLOWER_1

# Auto-connects on construction
arm = TrossenArmController(FOLLOWER_1)

arm.go_home()
arm.open_gripper()
arm.set_cartesian_positions([0.3, 0.0, 0.25, 0.0, 0.0, 0.0], duration=2.0)
arm.close_gripper()

arm.disconnect()   # parks to sleep, then releases connection
```

---

## 5. API summary

### `TrossenArmBase`
| Method | Description |
|---|---|
| `connect() → bool` | Configure driver and establish connection |
| `disconnect()` | Release connection (Controller overrides to park first) |
| `e_stop()` | Emergency stop — force cleanup, no homing |
| `_poll_state()` | Update cached state fields from driver |
| `get_status() → dict` | IP, connection flag, timestamp |

Cached state fields: `joint_positions`, `joint_velocities`, `joint_efforts`, `joint_external_efforts` (7-element lists), `ee_pose` (6-element `[x, y, z, roll, pitch, yaw]`), `gripper_position`, `state_ready`.

### `TrossenArmController`
| Method | Description |
|---|---|
| `go_home(duration=3.0)` | Move to upright home position |
| `go_sleep(duration=3.0)` | Move to parked (zeros) position |
| `set_mode(mode)` | Set all joints to a `trossen_arm.Mode` |
| `set_joint_positions(positions, duration=1.0, blocking=True)` | 7-joint command |
| `set_arm_positions(positions, duration=1.0, blocking=True)` | 6 arm joints only |
| `set_cartesian_positions(pose, duration=1.0, blocking=True, interp_space=...)` | EE pose `[x,y,z,r,p,y]` |
| `set_gripper(position, duration=1.0)` | Raw gripper position in metres |
| `open_gripper(duration=1.0)` | Open to `config.gripper_open_pos` |
| `close_gripper(duration=1.0)` | Close to `config.gripper_close_pos` |
| `gravity_comp()` | Kinesthetic teaching mode |
| `stream_positions(positions, velocities=None)` | Non-blocking, for real-time loops |
| `stream_external_efforts(efforts)` | Non-blocking effort command |
| `get_positions() → list[float]` | Poll driver for joint positions |
| `get_velocities() → list[float]` | Poll driver for joint velocities |
| `get_external_efforts() → list[float]` | Poll driver for external efforts |

---

## 6. Firmware upgrade

Only needed when driver and firmware versions diverge.

```bash
# Install Teensy Loader CLI
sudo apt install build-essential libusb-dev

# Flash firmware (obtain .hex from Trossen releases)
teensy_loader_cli --mcu=TEENSY41 -s firmware-wxai_v0.hex

# Reinstall matching driver
pip install trossen-arm==<version>
```

---

## 7. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `connect()` returns `False` | Wrong subnet or arm not powered | Check PC IP is `192.168.1.x`; verify arm power LED |
| `ping` times out | Firewall blocking ICMP | `sudo ufw allow from 192.168.1.0/24` |
| Driver raises version error | Firmware/driver mismatch | `pip show trossen-arm`; see Section 6 |
| Arm moves erratically | Error state not cleared | Set `clear_error_on_connect=True` (default) |
| `cleanup()` hangs | Driver in bad state | Call `arm.e_stop()` |
| New arm not reachable | Still on factory IP `192.168.1.2` | Use `trossen_arm` EEPROM API to reassign IP |
