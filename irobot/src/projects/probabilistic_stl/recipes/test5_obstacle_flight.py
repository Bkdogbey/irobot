"""
TASK 7 — Test 5: Obstacle Avoidance Flight
Full pdSTL reach-avoid spec. Physical obstacle between start and goal.

Prerequisites:
  - Task 6 (waypoint flight) PASS
  - Sim check for obstacle spec PASS
  - Physical obstacle placed and measured

Net area: ~4m x 2.5m
Start:    (0.0, 0.0)
Obstacle: x=[0.6, 1.0], y=[-0.3, 0.3]  — WE can update depending on measurements
Goal:     x=[1.6, 2.0], y=[-0.2, 0.2]
"""

import logging
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))           # probabilistic_stl/
sys.path.insert(0, os.path.join(_HERE, '../../..'))     # src/ — robots.crazyflie.*

logging.basicConfig(level=logging.INFO)

import rclpy
from constraints.reach_avoid import build_reach_avoid_spec
from executor.cf_executor import CrazyflieSTLExecutor
from planning.environment import build_test5_environment
from robots.crazyflie.core.base import CrazyflieComponent
from robots.crazyflie.core.config import CrazyflieConfig

Z_HOLD = 0.5
T = 70  # longer horizon for obstacle detour

PLANNER_CFG = {
    'w_u': 0.5,
    'w_du': 0.05,
    'w_phi': 100.0,
    'lr': 0.05,
    'max_iters': 400,
    'alpha': 0.85,
    'w_dist': 4.0,
    'w_obs': 3.0,  # active for obstacle avoidance
    'w_visit': 0.0,
}


def _print_env_summary(env):
    """Print software-defined environment so the pilot can verify it matches the physical setup."""
    print('\n--- Software environment (cross-check against physical setup) ---')
    if env.goal:
        print(f'  Goal:     x={env.goal["x"]}  y={env.goal["y"]}')
    for i, obs in enumerate(env.obstacles):
        print(f'  Obstacle {i}: x={obs["x"]}  y={obs["y"]}')
    print('-----------------------------------------------------------------')


def _plot_trajectory(mean_trace, env, p_sat, filename='test5_planned_path.png'):
    """Save top-down view of planned trajectory. Non-fatal if matplotlib unavailable."""
    try:
        import matplotlib.patches as patches
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 5))
        path = mean_trace[0].detach().numpy()
        ax.plot(path[:, 0], path[:, 1], 'b-o', markersize=3, label='Planned path')

        if env.goal:
            gx, gy = env.goal['x'], env.goal['y']
            ax.add_patch(patches.Rectangle(
                (gx[0], gy[0]), gx[1] - gx[0], gy[1] - gy[0],
                color='green', alpha=0.4, label='Goal',
            ))

        for i, obs in enumerate(env.obstacles):
            ox, oy = obs['x'], obs['y']
            ax.add_patch(patches.Rectangle(
                (ox[0], oy[0]), ox[1] - ox[0], oy[1] - oy[0],
                color='red', alpha=0.5,
                label='Obstacle' if i == 0 else '_',
            ))

        ax.scatter([0], [0], color='red', s=100, zorder=5, label='Start')
        ax.set_xlim(-0.5, 2.5)
        ax.set_ylim(-1.0, 1.0)
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title(f'Test 5 Planned Path | P(sat)={p_sat:.4f}')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(filename, dpi=150)
        print(f'Trajectory plot saved to {filename}')
        plt.show()
    except Exception as e:
        print(f'Plot failed (non-fatal): {e}')


def main():
    print('=== TASK 7: Obstacle Avoidance Flight ===')

    # Print the software environment first so the pilot can verify it matches the physical setup
    env = build_test5_environment()
    _print_env_summary(env)

    print('CONFIRM: Physical obstacle matches the coordinates printed above.')
    print('CONFIRM: test3_sim_check.py (obstacle spec) printed PASS in this session.')
    input('Press ENTER to continue or Ctrl+C to abort...')

    rclpy.init()
    cf = None
    try:
        print('Connecting...')
        cf = CrazyflieComponent(component_name='test5_cf', config=CrazyflieConfig())

        timeout = 10
        start = time.time()
        while not cf.is_connected and (time.time() - start) < timeout:
            time.sleep(0.2)

        if not cf.is_connected:
            print('FAIL: Could not connect')
            sys.exit(1)

        executor = CrazyflieSTLExecutor(cf, dt=0.1, z_hold=Z_HOLD, u_max=0.3)

        # wait_for_state raises RuntimeError if lighthouse state is not received in time
        print('Waiting for lighthouse state...')
        executor.wait_for_state(timeout=8.0)
        print(f'Lighthouse lock: x={cf.current_x:.3f}  y={cf.current_y:.3f}  z={cf.current_z:.3f}')

        # Sanity-check: position stuck at (0,0,0) means lighthouse is not tracking
        if abs(cf.current_x) < 1e-4 and abs(cf.current_y) < 1e-4 and abs(cf.current_z) < 1e-4:
            print('WARN: Position is (0, 0, 0) — lighthouse may not be locked.')
            choice = input('Continue anyway? Type "yes" to proceed: ').strip().lower()
            if choice != 'yes':
                sys.exit(1)

        print(f'\nPlanning obstacle avoidance trajectory (T={T} steps = {T * 0.1:.1f}s)...')
        u_trace, mean_trace, cov_trace, p_sat, history = executor.plan(
            env, T, PLANNER_CFG, build_reach_avoid_spec, verbose=True
        )

        final_pos = mean_trace[0, -1, :].detach().numpy()
        print(f'\nPlan results:')
        print(f'  P(satisfaction) = {p_sat:.4f}')
        print(f'  Planned final position = [{final_pos[0]:.3f}, {final_pos[1]:.3f}]')

        if p_sat < 0.85:
            print('ABORT: P(sat) below 0.85 — not safe to fly')
            sys.exit(1)

        # Show planned trajectory before asking for flight confirmation
        _plot_trajectory(mean_trace, env, p_sat)

        print('\nFlight area clear? Obstacle in position? Kill switch ready?')
        input('Press ENTER to TAKEOFF and FLY, or Ctrl+C to abort...')

        print(f'Taking off to z={Z_HOLD}m...')
        cf.takeoff(z_hold=Z_HOLD, duration=3.0)
        print('Hover stable. Executing trajectory in 2 seconds...')
        time.sleep(2.0)

        print('Executing planned trajectory...')
        executor.execute_open_loop(u_trace)

        print('Trajectory complete. Holding position...')
        time.sleep(2.0)

        print('Landing...')
        cf.land(z_hold=Z_HOLD)

    except KeyboardInterrupt:
        print('\nAborted by user')
        if cf and cf.is_connected:
            print('Emergency land...')
            cf.land(z_hold=Z_HOLD)

    except Exception as e:
        print(f'\nERROR: {e}')
        if cf and cf.is_connected:
            print('Emergency land...')
            try:
                cf.land(z_hold=Z_HOLD)
            except Exception:
                pass

    finally:
        if cf and cf.is_connected:
            cf.disconnect()
        rclpy.shutdown()
        print('Disconnected.')

    print('=== TASK 7 COMPLETE ===')


if __name__ == '__main__':
    main()
