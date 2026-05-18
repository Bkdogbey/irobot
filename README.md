# irobot
### *Intelligent. Modular. Build-Free.*

Central robot software platform for the **iHuman Lab**.
Built on [ROS2](https://docs.ros.org) + [ros_sugar](https://github.com/automatika-robotics/ros-sugar) —
write robot components in pure Python, no `colcon build` required.

Install `irobot`, import the robot driver you need, and build your application on top.

---

## 📦 Installation

```bash
git clone https://github.com/iHumanLab/irobot.git
cd irobot
pip install -e .
```

> ROS2 and ros_sugar must be installed in your environment for ROS-based component usage.

---

## 🔌 Usage

**Standalone (no ROS):**

```python
from irobot import CrazyflieController, CrazyflieConfig

drone = CrazyflieController(CrazyflieConfig(uri='radio://0/80/2M/E7E7E7E781'))
drone.takeoff()
drone.fly_to(0.5, 0.5, 0.3)
drone.land()
```

**As a ROS2 component (ros_sugar):**

Copy `irobot/src/robots/crazyflie/examples/crazyflie_ros_component.py` into your project and extend `_execute_once` with your mission logic.

---

## 🚀 Quick start (development)

```bash
# 1. Clone and install (see above)
# 2. Set your radio URI in irobot/src/robots/crazyflie/config.py
# 3. Run the demo
python main.py
```

---

## 📁 Structure

```
irobot/
├── main.py                    ← demo launcher (development use)
│
└── irobot/src/
    └── robots/                ← one folder per supported robot
        └── <robot_name>/
            ├── core/          ← driver: base, controller, logging
            ├── examples/      ← runnable examples and ROS component templates
            ├── config.py      ← hardware configuration dataclass
            └── README.md      ← hardware setup guide (udev rules, pairing, etc.)
```

Each robot folder wraps a hardware SDK into a clean Python class.
Examples live next to the robot they demo — copy them into your own project as a starting point.

---

## 🤖 Adding a new robot

1. Create `irobot/src/robots/<robot_name>/`
2. Add `config.py` — a dataclass with all hardware parameters (URI, timeouts, rates)
3. Add `core/base.py` — a Python class that wraps the robot's SDK
4. Add `core/controller.py` — high-level movement and control primitives
5. Add `examples/` — at least one runnable example or ROS component template
6. Add a `README.md` with hardware setup instructions (udev rules, pairing, etc.)
7. Add `__init__.py` files and export from `irobot/__init__.py`

---

## 📌 Conventions

- **One `config.py` per robot** — all hardware parameters live there, nowhere else.
- **`robots/` is hardware-only** — no experiment logic, no application-specific constants.
- **`examples/` stays inside the robot folder** — examples are robot-specific and ship with the driver.

---

*Intelligent Human-Machine Nexus Lab*
