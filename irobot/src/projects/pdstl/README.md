# pDSTL Crazyflie Project

Probabilistic Signal Temporal Logic (pDSTL) flight control for the Crazyflie nano-drone,
using Lighthouse positioning for state estimation.

The project has two layers: an **algorithm layer** (`stl/`) that is pure numpy/scipy
and runs independently of any robot, and a **component layer** (`component/`) that wraps
it in a ROS Sugar component with logging and waypoint management.

---

## Config — edit before every run

All tunable parameters live in one file:

```
irobot/src/projects/pdstl/config.py
```

| Parameter       | Description                           | Values                             |
|-----------------|---------------------------------------|------------------------------------|
| `Z_HOLD`        | Hover altitude (m)                    | float, default `0.3`               |
| `USE_OPTIMISED` | Path selector                         | `False` = sine, `True` = pDSTL     |
| `CONDITION`     | Trial label written to log filenames  | `'deterministic'` \| `'pdstl'`     |
| `FAN_SPEED`     | Fan disturbance level                 | `0` \| `2` \| `6` \| `12` \| `16` |

---

## Two ways to run

### 1. ROS Sugar Launcher — `main.py`

Runs `CrazyfliePlanning` as a managed ROS2 component with CSV flight logging.

```bash
python main.py
```

### 2. Direct hardware runner — `stl/runner.py`

Runs the reactive STL belief-update loop directly, with no ROS2 component lifecycle.
Useful for quick hardware tests or when ros_sugar is not available.

```bash
python -m irobot.src.projects.pdstl.stl.runner
```

---

## Layout

```
pdstl/
├── config.py            ← all experiment parameters + CrazyflieConfig class
│
├── stl/                 ← algorithm layer (numpy/scipy, no hardware dependency)
│   ├── belief.py        ← 2D Gaussian belief state (Kalman predict + update)
│   ├── stl.py           ← STL predicates and temporal operators (Always, Eventually)
│   ├── scenario.py      ← mission geometry (obstacles, goal region, nominal waypoints)
│   ├── controller.py    ← reactive STL velocity controller
│   ├── mission.py       ← Monte-Carlo simulation + visualisation (dry-run, no hardware)
│   └── runner.py        ← hardware runner (direct cflib, no ROS2)
│
└── component/           ← ROS Sugar wrapper layer
    ├── crazyflie.py     ← CrazyfliePlanning BaseComponent (reads config.py at import)
    ├── flight_logger.py ← CSV logger for actual and commanded positions
    └── opt_waypoints.py ← pre-computed pDSTL-optimised waypoints
```

---

## Hardware requirements

- Crazyflie 2.x nano-drone
- Crazyradio PA USB dongle
- Lighthouse v2 base stations (for position estimation)
- cflib Python library installed

See `irobot/src/robots/crazyflie/README.md` for USB setup and udev rules.
