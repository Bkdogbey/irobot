"""
TASK 6 — Test 4: Waypoint Flight
First live autonomous flight. Point-to-point, no obstacles, open-loop.

Prerequisites:
  - Task 1 (hardware setup) complete
  - Task 3 (hardware ping) PASS
  - Task 4 (lighthouse lock) PASS
  - Task 5 (sim check) PASS in this session

Net area: ~4m x 2.5m
Start:    (0.0, 0.0) — lighthouse origin at CF power-on
Goal:     x=[1.3, 1.7], y=[-0.2, 0.2] — place marker at 1.5m forward
"""

import os
import sys
import time
import logging

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))  # probabilistic_stl/
sys.path.insert(0, os.path.join(_HERE, '../../..'))  # src/ — robots.crazyflie.*

logging.basicConfig(level=logging.INFO)

import rclpy
from robots.crazyflie.core.base import CrazyflieComponent
from robots.crazyflie.core.config import CrazyflieConfig

from planning.environment import build_test4_environment
from constraints.reach_only import build_reach_spec
from executor.cf_executor import CrazyflieSTLExecutor

Z_HOLD = 0.5
T = 50  # 50 steps x 0.1s = 5 second trajectory

PLANNER_CFG = {
    'w_u': 0.5,
    'w_du': 0.05,
    'w_phi': 100.0,
    'lr': 0.05,
    'max_iters': 300,
    'alpha': 0.85,
    'w_dist': 5.0,
    'w_obs': 0.0,
    'w_visit': 0.0,
}


def main():
    print('=== TASK 6: Waypoint Flight ===')
    print('CONFIRM: test3_sim_check.py printed PASS in this session')
    input('Press ENTER to continue or Ctrl+C to abort...')

    rclpy.init()
    cf = None
    try:
        # 1. Connect
        print('Connecting to Crazyflie...')
        cf = CrazyflieComponent(component_name='test4_cf', config=CrazyflieConfig())

        timeout = 10
        start = time.time()
        while not cf.is_connected and (time.time() - start) < timeout:
            time.sleep(0.2)

        if not cf.is_connected:
            print('FAIL: Could not connect')
            sys.exit(1)
        print(f'Connected: {cf.uri}')

        # 2. Build environment and executor
        env = build_test4_environment()
        executor = CrazyflieSTLExecutor(cf, dt=0.1, z_hold=Z_HOLD, u_max=0.3)

        # 3. Wait for lighthouse lock
        print('Waiting for lighthouse state...')
        executor.wait_for_state(timeout=8.0)
        print(f'Lighthouse lock: x={cf.current_x:.3f}  y={cf.current_y:.3f}  z={cf.current_z:.3f}')

        # 4. Plan from current position
        print(f'\nPlanning trajectory (T={T} steps = {T * 0.1:.1f}s)...')
        u_trace, mean_trace, cov_trace, p_sat, history = executor.plan(
            env, T, PLANNER_CFG, build_reach_spec, verbose=True
        )

        final_pos = mean_trace[0, -1, :].detach().numpy()
        print(f'\nPlan results:')
        print(f'  P(satisfaction) = {p_sat:.4f}')
        print(f'  Planned final position = [{final_pos[0]:.3f}, {final_pos[1]:.3f}]')

        if p_sat < 0.85:
            print('ABORT: P(sat) below 0.85 — not safe to fly')
            sys.exit(1)

        # 5. Human confirmation gate
        print(f'\nGoal marker should be at approximately x=1.5m from CF start position')
        print(f'Flight area clear? Kill switch ready?')
        input('Press ENTER to TAKEOFF and FLY, or Ctrl+C to abort...')

        # 6. Takeoff
        print(f'Taking off to z={Z_HOLD}m...')
        cf.takeoff(z_hold=Z_HOLD, duration=3.0)
        print('Hover stable. Executing trajectory in 2 seconds...')
        time.sleep(2.0)

        # 7. Execute
        print('Executing planned trajectory...')
        executor.execute_open_loop(u_trace)

        # 8. Hold position briefly
        print('Trajectory complete. Holding position...')
        time.sleep(2.0)

        # 9. Land
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

    print('=== TASK 6 COMPLETE ===')


if __name__ == '__main__':
    main()
