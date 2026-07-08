from __future__ import annotations

from ros_sugar import Launcher

from irobot.src.utils import skip_run

# ── Swarm 1 — each drone flies its own 0.3 m square (relative) ───────────────
with skip_run('run', 'swarm_square') as check:
    with check():
        from irobot.src.robots.crazyflie.examples.swarm_square_component import SwarmSquareDemo
        from irobot.src.robots.crazyflie import SwarmSquareConfig
        component = SwarmSquareDemo(component_name='swarm_square', config=SwarmSquareConfig())

# ── Swarm 2 — vertical line → square → line → land ───────────────────────────
with skip_run('skip', 'swarm_transition') as check:
    with check():
        from irobot.src.robots.crazyflie.examples.swarm_transition_component import SwarmTransitionDemo
        from irobot.src.robots.crazyflie import SwarmTransitionConfig
        component = SwarmTransitionDemo(component_name='swarm_transition', config=SwarmTransitionConfig())

# ── Swarm 3 — 3 drones: horizontal line ↔ vertical line (workspace x[0,1] y[0,-2]) ──
with skip_run('skip', 'swarm_hv') as check:
    with check():
        from irobot.src.robots.crazyflie.examples.swarm_hvtransition_component import SwarmHVTransitionDemo
        from irobot.src.robots.crazyflie import SwarmHVConfig
        component = SwarmHVTransitionDemo(component_name='swarm_hv', config=SwarmHVConfig())

# ── Leader-Follower — 2 drones, S-curve through workspace x[0,1] y[0,-2] ─────
with skip_run('skip', 'leader_follower') as check:
    with check():
        from irobot.src.robots.crazyflie.examples.leader_follower_component import LeaderFollowerDemo
        from irobot.src.robots.crazyflie import LeaderFollowerConfig
        component = LeaderFollowerDemo(component_name='leader_follower', config=LeaderFollowerConfig())

# ── Spline — single drone, sine S-curve (workspace x[0,1] y[0,-2]) ───────────
with skip_run('skip', 'spline_waypoint') as check:
    with check():
        from irobot.src.robots.crazyflie.examples.spline_waypoint_component import SplineWaypointDemo
        from irobot.src.robots.crazyflie import SplineWaypointConfig
        component = SplineWaypointDemo(component_name='spline_waypoint', config=SplineWaypointConfig())

with skip_run('skip', 'crazyflie_ros') as check:
    with check():
        from irobot.src.robots.crazyflie.examples.crazyflie_ros_component import CrazyflieDemo
        from irobot.src.robots.crazyflie import CrazyflieConfig
        component = CrazyflieDemo(component_name='crazyflie_ros', config=CrazyflieConfig())


# ── Launch ────────────────────────────────────────────────────────────────────
launcher = Launcher()
launcher.add_pkg(components=[component], activate_all_components_on_start=True)
launcher.bringup()
