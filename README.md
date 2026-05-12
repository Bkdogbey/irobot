# irobot

Central robot software platform for the **iHuman Lab**.
Built on [ROS2](https://docs.ros.org) + [ros_sugar](https://github.com/automatika-robotics/ros-sugar) —
write robot components in pure Python, no `colcon build` required.

---

## Structure

```
irobot/
├── main.py                   ← active launch entry point
│
└── irobot/src/
    ├── robots/               ← hardware abstraction layers, one folder per robot
    │   └── <robot_name>/     ← driver + setup guide (README.md)
    └── projects/             ← research projects, one folder per project
        └── <project_name>/   ← config.py + algorithm + component subfolders
```

Each robot folder wraps a hardware SDK into a Python class.
Each project folder holds all the logic for one experiment, with its own `config.py`
as the single place to edit parameters before a run.

---

## Quick start

```bash
# 1. Install system dependencies (ROS2, ros_sugar, and the relevant robot SDK)
# 2. Edit the active project's config.py with your run parameters
# 3. Launch
python main.py
```

---

## Adding a new robot

1. Create `irobot/src/robots/<robot_name>/`
2. Add `core/base.py` — a Python class that wraps the robot's SDK
3. Add a `README.md` with hardware setup instructions (udev rules, pairing, etc.)
4. Add `__init__.py` files for Python packaging

## Adding a new project

1. Create `irobot/src/projects/<project_name>/`
2. Add `config.py` — all tunable parameters in one file
3. Add your algorithm and component code in subfolders
4. Update `main.py` to import and launch your component
5. Add a project-level `README.md` describing the experiment and how to run it

---

## Conventions

- **One `config.py` per project** — all parameters that change between runs live there, nowhere else.
- **`robots/` is hardware-only** — no experiment logic, no project-specific constants.
- **`projects/` is sandboxed** — each project is self-contained and does not import from another project.

---

*Intelligent Human-Machine Nexus Lab*
