# irobot
### *Intelligent. Modular. Build-Free.*

**irobot** is a Python library of robot drivers developed at the **iHuman Lab**. Each driver wraps a robot's hardware SDK into a clean, importable Python class. Install irobot, import the driver for your robot, and build your application on top — no framework lock-in required.

Drivers work standalone (plain Python) or as ROS2 components via [ros_sugar](https://github.com/automatika-robotics/ros-sugar), with no `colcon` build step.

---

## 🤖 Supported Robots

| Robot | Description | Driver Docs |
|-------|-------------|-------------|
| [Crazyflie 2.x](https://www.bitcraze.io/products/crazyflie-2-1/) | Nano quadrotor by Bitcraze | [irobot/src/robots/crazyflie/README.md](irobot/src/robots/crazyflie/README.md) |

---

## Prerequisites

- Python 3.11 or higher
- The hardware SDK for your robot (see the robot's README for installation)
- **ROS2 + ros_sugar** — only required if you are using ROS2-based components (optional)

---

## 📦 Installation

Clone the repository and install in editable mode:

```bash
git clone https://github.com/iHumanLab/irobot.git
cd irobot
pip install -e .
```

Then install the SDK for your specific robot. See the robot's README for exact instructions.

---

## 🚀 Quick Start

Import the controller and configuration class for your robot, configure it, and call the driver's methods:

```python
from irobot import RobotController, RobotConfig

robot = RobotController(RobotConfig(...))
robot.connect()

# Use the robot-specific API
# See your robot's README for full usage and examples
```

Each robot exports its classes at the top level of the `irobot` package. See the [Supported Robots](#-supported-robots) table above for links to robot-specific documentation.

---

## 📁 Repository Structure

```
irobot/
├── main.py                    ← demo launcher (development use only)
│
└── irobot/src/
    └── robots/                ← one folder per supported robot
        └── <robot_name>/
            ├── core/          ← driver internals (connection, controller, logging)
            ├── examples/      ← usage examples and ROS2 component templates
            ├── config.py      ← all hardware parameters in one dataclass
            └── README.md      ← setup guide and full API documentation
```

Each robot folder is self-contained: the driver, its configuration, its examples, and its hardware setup guide all live together. When you want to use a robot, start with its `README.md`.

---

## 🤖 Adding a New Robot

Contributions are welcome. Follow these steps to add a new robot driver:

1. Create a folder at `irobot/src/robots/<robot_name>/`
2. Add `config.py` — a Python dataclass holding all hardware parameters (addresses, timeouts, rates). This is the only place parameters should live.
3. Add `core/base.py` — a class that wraps the robot's SDK, manages the connection lifecycle, and exposes live state.
4. Add `core/controller.py` — high-level motion primitives built on top of the base class.
5. Add `core/logging.py` (optional) — a mixin for onboard sensor logging, if applicable.
6. Add `examples/` — at least one runnable Python example and, if applicable, a ROS2 component template.
7. Add a `README.md` — hardware setup instructions (drivers, udev rules, pairing) plus configuration and API documentation.
8. Add `__init__.py` files throughout and export the main classes from `irobot/__init__.py`.

---

## 📌 Design Conventions

- **One `config.py` per robot** — all hardware parameters belong there and nowhere else.
- **`robots/` is hardware-only** — drivers wrap hardware SDKs. No experiment logic, application state, or project-specific constants belong here.
- **`examples/` travels with the robot** — examples are co-located with the robot they demonstrate and ship as part of the driver.

---

*iHuman Lab — Intelligent Human-Machine Nexus*
