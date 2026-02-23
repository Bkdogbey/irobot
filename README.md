<div align="center">

# 🤖 irobot

### *Intelligent. Modular. Build-Free.*

The central software ecosystem for the **iHuman Lab**, designed to decouple robotics hardware from algorithmic research.

</div>

---

## 🎯 What is irobot?

In the **Intelligent Human-Machine Nexus Lab**, we believe researchers should spend their time on *intelligence*, not *infrastructure*.

**irobot** is a "Nexus" repository. It uses **Sugarcoat** to allow students to build complex robotics projects in **pure Python**. No more waiting for `colcon build` every time you change a line of code.

## ✨ Key Features

| Feature                   | Benefit                                                             |
| ------------------------- | ------------------------------------------------------------------- |
| ⚡ **Zero-Build Momentum** | Save your Python file and run. No compilation required.             |
| 🧱 **Project Isolation**   | Independent sandboxes ensure Student A never breaks Student B.      |
| 🤖 **Hardware Agnostic**   | Write an HMI algorithm once; deploy it on Sawyer, Aloha, or Drones. |
| 🎛️ **Live Dashboards**     | Auto-generated Web UIs to tweak parameters during human trials.     |
| 🧩 **Core Nexus Library**  | A shared "language" of HMI messages and safety utilities.           |

## 🚀 Quick Start

### 1. Setup the Nexus

```bash
# 1. Clone the nexus
git clone https://github.com/iHuman-Lab/irobot.git
cd irobot

# 2. Pull in the latest robot drivers (Requires vcstool)
# sudo apt install python3-vcstool
vcs import src < .repos

# 3. Install Lab Dependencies (One-time setup)
# This script handles pip installs and rosdep for the lab
chmod +x scripts/bootstrap.sh
./scripts/bootstrap.sh

# 4. Build the core messages (Required for HMI language)
colcon build --packages-select msgs
source install/setup.bash

# 5. Run a Research Recipe
python3 src/projects/human_intent/recipes/run_trial.py

```

### 2. Run a Research Recipe

```python
# Each project owns its own "Launch Recipe"
python3 src/projects/human_intent/recipes/run_trial.py

```

## 📁 Project Structure

```text
irobot/
├── src/
│   ├── core/            # 🤝 SHARED: Common HMI messages & Nexus utilities.
│   ├── drivers/         # 🏗️ WRAPPERS: Sugarcoated interfaces for lab hardware.
│   ├── projects/        # 🚨 SANDBOX: Student-owned research packages.
│   │   ├── human_intent/
│   │   └── [your_project]/
│   └── third_party/     # 📦 VENDOR: Raw drivers (Sawyer SDK, Crazyswarm, etc.)
├── recipes/             # 📔 LAB MENU: Top-level demo and multi-robot recipes.
├── docker/              # 🐳 CONTAINER: The universal lab environment.
└── .repos               # 📑 MANIFEST: External dependencies list.

```

## 🧪 The iHuman Workflow

We utilize **Branch Protection** to keep the lab stable while allowing rapid iteration:

1. **Sandbox:** Create your folder in `src/projects/`. This is your private kingdom.
2. **Standardize:** If you need a new data type, contribute it to `src/core/msgs`.
3. **Connect:** Use a **Sugarcoat Recipe** to bridge a `driver` to your `project` logic.
4. **Deploy:** Run your recipe. **Iterate in seconds, not minutes.**

---

### 🛠️ Nexus Development Rules

* **Don't Touch the Drivers:** Unless you are fixing a hardware bug.
* **Namespacing is Key:** Always name your components relative to your project.
* **PRs for Core:** Any changes to `core/` or `drivers/` require a Peer Review.

---

<div align="center">

**Intelligent Human-Machine Nexus Lab**

*Bridging the gap between humans and machine*

</div>
