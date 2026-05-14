# Trossen Arm Template Project

This is a copy-and-adapt starting point for any new arm experiment. It shows the minimal wiring between a `TrossenArmBase` hardware object and a `ros_sugar` component.

For hardware setup (Ethernet config, IP table, quick test), see [`robots/trossen_arm/README.md`](../../robots/trossen_arm/README.md).

---

## How to start a new project from this template

**Step 1 — Copy this folder and rename it:**

```bash
cp -r irobot/src/projects/trossen_arm_template irobot/src/projects/my_arm_project
```

**Step 2 — Pick your arm in `config.py`:**

Open `config.py` and change the `ACTIVE_ARM` line to the arm you want:

```python
# Options: LEADER_1, FOLLOWER_1, LEADER_2, FOLLOWER_2
ACTIVE_ARM: TrossenArmConfig = FOLLOWER_1   # ← change this
```

Also update `TASK_DURATION` and any other parameters for your experiment.

**Step 3 — Add your experiment logic in `component/arm_component.py`:**

Edit `_execute_once()`. The arm is already connected and homed when your code runs. Put your sequence between the comment markers:

```python
def _execute_once(self):
    try:
        self.arm.connect()
        self.arm.go_home()

        # ── Your experiment logic goes here ──────────────────────────────
        self.arm.open_gripper()
        self.arm.set_cartesian_positions([0.3, 0.0, 0.25, 0.0, 0.0, 0.0], duration=2.0)
        self.arm.close_gripper()
        # ────────────────────────────────────────────────────────────────

    except Exception as exc:
        print(f'Error: {exc}')
        raise
    finally:
        self.arm.disconnect()   # parks the arm and cleans up
```

**Step 4 — Wire into `main.py`:**

Replace the imports and component name at the top of `main.py`:

```python
from ros_sugar import Launcher
from irobot.src.projects.my_arm_project.config import TrossenArmComponentConfig
from irobot.src.projects.my_arm_project.component.arm_component import TrossenArmTemplate

arm = TrossenArmTemplate(
    component_name='my_arm_project',
    config=TrossenArmComponentConfig(),
)

launcher = Launcher()
launcher.add_pkg(components=[arm], activate_all_components_on_start=True)
launcher.bringup()
```

**Step 5 — Run:**

```bash
python main.py
```

---

## Useful TrossenArmBase methods

| Method | What it does |
|---|---|
| `arm.connect()` | Connects to the arm over Ethernet |
| `arm.disconnect()` | Parks (sleeps) then disconnects |
| `arm.go_home()` | Moves to upright home position |
| `arm.go_sleep()` | Moves to zeros (parked position) |
| `arm.get_state()` | Polls and caches joint/Cartesian state |
| `arm.ee_pose` | Last cached EE pose `[x, y, z, roll, pitch, yaw]` |
| `arm.joint_positions` | Last cached joint positions (7 values) |
| `arm.set_joint_positions(q, duration)` | Move all 7 joints |
| `arm.set_arm_positions(q, duration)` | Move arm joints only (6, no gripper) |
| `arm.set_cartesian_positions(pose, duration)` | Move to Cartesian pose |
| `arm.open_gripper()` / `arm.close_gripper()` | Gripper control |
| `arm.gravity_comp()` | Float under gravity (for teaching) |
| `arm.e_stop()` | Emergency stop |

For teleoperation (leader → follower streaming), see the `stream_positions()` and `stream_external_efforts()` methods in `core/base.py`.
