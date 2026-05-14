# Trossen Cake Cutting

Autonomous cake cutting using a single Trossen Robotics WXAI V0 follower arm.
The arm picks up a Farberware triangular cake server and cuts rectangular or round
cakes into equal pieces with straight cuts.

---

## Hardware Required

| Item | Details |
|---|---|
| Robot arm | Trossen WXAI V0 — **follower1** at `192.168.1.3` |
| Knife | Farberware triangular cake/pie server (kite-shaped serrated blade) |
| Cake (rectangular) | Sheet cake — **11" × 5" × 2"** |
| Cake (round) | Round cake — **6.5" diameter × 2"** |
| Platform | Stable flat surface to place the cake on |

---

## Project Structure

```
trossen_cake/
├── README.md                  ← this file
├── config.py                  ← project parameters (edit before running)
├── cake_config.py             ← calibrated constants (auto-generated — do not edit)
├── calibrate_arm.py           ← interactive calibration tool
├── gravity_comp.py            ← gravity compensation mode (for manual teaching)
├── test_grip.py               ← gripper strength / knife pickup test
├── configure_trossen.py       ← arm inspection utility
├── set_manual_ip.py           ← arm IP address utility
├── algorithms/
│   ├── cake_cut.py            ← autonomous rectangular cake cutting (5 strips)
│   └── cut_round_cake.py      ← autonomous round cake cutting (up to 8 slices)
└── teleoperation/
    ├── teleoperation.py       ← single leader-follower pair (30 s)
    └── teleoperation_4arm.py  ← two simultaneous leader-follower pairs (120 s)
```

Run all scripts from the project root:

```bash
cd irobot/src/projects/trossen_cake
conda activate trossen_ai_env
```

---

## One-Time Setup

### 1. Physical arrangement

```
          [ARM base]
               |
     ┌─────────┼──────────┐
     │  knife  │          │
     │  rest   │  table   │
     │         │          │
     │         │  [CAKE]  │
     └─────────┴──────────┘
```

- Place the arm on a stable surface.
- Place the **knife** flat on the table near the arm, handle pointing toward the arm,
  blade pointing away. The arm's gripper will descend from above onto the handle.
- Place the **cake** on its platform within the arm's reach (roughly 40–50 cm in front).
- Keep the arm near its **sleep position** (all joints at zero) before running any script.

### 2. Set gripper width (one time only)

Open `cake_config.py` and adjust the two gripper constants to fit the knife handle:

```python
GRIPPER_OPEN   = 0.035   # m — wide enough to slide over the handle
GRIPPER_CLOSED = 0.015   # m — firm grip; increase by 0.002 if the knife slips
```

These are the **only** values you ever edit manually in `cake_config.py`. Everything
else is written by `calibrate_arm.py`.

---

## Step 1 — Calibration

Run whenever the arm, knife rest position, or cake position changes.

```bash
python calibrate_arm.py
```

The arm connects, prints its home Cartesian pose, then enters **gravity compensation**
— you can push the arm freely by hand.

The script walks you through **5 poses** one at a time:

| # | Pose | What to do |
|---|---|---|
| 1 | **KNIFE_HOVER** | Gripper open, hovering **4–5 cm above** the knife handle |
| 2 | **KNIFE_GRIP** | Lower straight down until the gripper jaws are **level with the handle centre** |
| 3 | **CAKE_APPROACH** | Knife gripped, blade vertical, hovering above cake centre at **safe clearance height (10–15 cm above cake top)** |
| 4 | **CAKE_TOP** | Lower until the knife tip **just rests on the top surface** of the cake |
| 5 | **CAKE_CUT** | Lower until the knife is **fully through** the cake, tip **1–2 mm above** the platform |

> **Pose 3 is critical.** The z value recorded here becomes `Z_APPROACH`. It must be
> clearly above the cake top — if in doubt, go higher. A value close to `CAKE_Z_TOP`
> will cause an IK failure when the arm tries to approach the cut start.

For each pose:
1. Push the arm to the position by hand and hold it still
2. Press **Enter**
3. Check the printed values look reasonable, then type **Y** to confirm
   (or move and press Enter again to re-capture)

When all 5 poses are captured, `calibrate_arm.py` **writes `cake_config.py`
automatically**. There is nothing to copy or paste.

---

## Cutting a Rectangular Cake

### Step 2 — Dry Run (no real cutting)

Before the first full cut, verify the trajectory with the knife hovering only.
Open `cake_config.py` and temporarily set:

```python
CAKE_Z_CUT = Z_APPROACH   # knife never descends into the cake
```

Then run:

```bash
python algorithms/cake_cut.py
```

The script prints a **constant summary and validation check** before the arm moves.
Watch the arm trace all 4 cut paths in the air above the cake.

### Step 3 — Skim Test (optional but recommended)

In `cake_config.py` set:

```python
CAKE_Z_CUT = CAKE_Z_TOP + 0.01   # knife barely skims the top surface
```

Run again and confirm the knife touches the correct positions on the cake.

### Step 4 — Full Cut

Restore `CAKE_Z_CUT` to its calibrated value in `cake_config.py`, then run:

```bash
python algorithms/cake_cut.py
```

#### What the arm does

```
1. Validate constants → print summary
2. Connect to arm at 192.168.1.3
3. Move to home position
4. Open gripper → move to knife → descend → grip → lift
5. Move to cake approach position (joint space)
6. Cut 1–4 — four parallel cuts along the length, evenly spaced across the width
7. Return to home → sleep
```

#### Cut pattern — 5 equal strips

```
    ┌─────────────────────────────────┐
    │           │     │     │         │
    │           │     │     │         │
    │           │     │     │         │
    └─────────────────────────────────┘
    ←────────────── 11" ─────────────→
```

#### Cut timing

| Step | Duration |
|---|---|
| Home → knife hover | ~3 s |
| Descend + grip + lift | ~5 s |
| Move to cake approach | ~3 s |
| Per cut (hover → plunge → drag → lift) | ~10 s |
| 4 cuts total | ~40 s |
| Return home + sleep | ~8 s |
| **Total** | **~60 s** |

---

## Cutting a Round Cake

Uses the same `cake_config.py` calibration. The round cake must be centred at
(`CAKE_CX`, `CAKE_CY`) — recalibrate if the position changes.

### Dry run

Open `algorithms/cut_round_cake.py` and temporarily override `CAKE_Z_CUT`:

```python
CAKE_Z_CUT = Z_APPROACH   # knife hovers only
```

Run:

```bash
python algorithms/cut_round_cake.py
```

Verify the 4 cut paths trace correctly above the cake, then restore `CAKE_Z_CUT`.

### Real cut

```bash
python algorithms/cut_round_cake.py
```

The script executes cuts at 0°, 45°, 90°, 135° → 8 equal slices.

By default only the first cut (0°) is active — uncomment additional lines in the
`cuts` list inside `cut_round_cake.py` to increase the slice count.

#### Tuning (top of `cut_round_cake.py`)

| Constant | Default | Effect |
|---|---|---|
| `PLUNGE_DURATION` | 2.5 s | Increase if knife deflects on entry |
| `CUT_DURATION` | 5.0 s | Increase for cleaner cut on dense cake |
| `CAKE_R` | 0.08255 m | Change only if using a different cake diameter |

---

## Teleoperation

For manual control or demonstration before a cut session.

### Single pair (leader 192.168.1.4 → follower 192.168.1.5)

```bash
python teleoperation/teleoperation.py
```

Runs for 30 seconds with force feedback. Let go of the leader when the time expires
— the arm will lock and move to home.

### Two pairs simultaneously

```bash
python teleoperation/teleoperation_4arm.py
```

Pair 1: leader `192.168.1.2` → follower `192.168.1.3`
Pair 2: leader `192.168.1.4` → follower `192.168.1.5`
Runs for 120 seconds.

---

## Troubleshooting

### `CALIBRATION ERROR` printed before the arm moves
`Z_APPROACH` is too low — the CAKE_APPROACH pose was recorded near the surface.
Re-run `calibrate_arm.py` with pose 3 at least **10 cm above the cake top**.

### `Joint limit exceeded`
The arm was not near sleep position when the script started.
Move all joints close to zero by hand, or power-cycle the arm.

### `Cannot find inverse kinematics solution`
The arm cannot reach the commanded pose from its current configuration.
Do not move the arm or cake between calibration and cutting.
Re-run `calibrate_arm.py` to recapture all waypoints.

### Knife slips during cut
Increase `GRIPPER_CLOSED` in `cake_config.py` by `0.002` m at a time.
Do not exceed the physical width of the knife handle.

### Cut is misaligned
`CAKE_CX` or `CAKE_CY` is off. Re-run `calibrate_arm.py` and recapture
**CAKE_APPROACH** with the knife tip directly above the exact cake centre.

### Script crashes mid-cut
The `finally` block always attempts to return the arm to home and sleep.
If the arm is frozen, power-cycle it, then restart from the full cut step.
