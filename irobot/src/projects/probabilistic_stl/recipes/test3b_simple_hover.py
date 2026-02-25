"""
TASK 5b — Simple Hover Test
First motors-on test. Takes off, holds hover at fixed height, lands.
No planning, no trajectory — pure hardware validation.

Prerequisites:
  - Task 4 (lighthouse lock) PASS
  
  - Task 5 (sim check) PASS in this session

Pass criteria:
  - Drone hovers stably for 5 seconds without drifting > 0.5m from start position
  - Lands cleanly
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
from robots.crazyflie.core.base import CrazyflieComponent
from robots.crazyflie.core.config import CrazyflieConfig

Z_HOLD      = 0.4   # conservative hover height for first flight
HOVER_TIME  = 5.0   # seconds to hold hover
DRIFT_LIMIT = 0.5   # meters — abort if position drifts beyond this


def main():
    print('=== TASK 5b: Simple Hover Test ===')
    print(f'Will hover at z={Z_HOLD}m for {HOVER_TIME:.0f}s then land.')
    print('Flight area clear? Kill switch ready?')
    input('Press ENTER to continue or Ctrl+C to abort...')

    rclpy.init()
    cf = None
    try:
        print('Connecting to Crazyflie...')
        cf = CrazyflieComponent(component_name='hover_test', config=CrazyflieConfig())

        timeout = 10
        start = time.time()
        while not cf.is_connected and (time.time() - start) < timeout:
            time.sleep(0.2)

        if not cf.is_connected:
            print('FAIL: Could not connect within 10 seconds')
            sys.exit(1)
        print(f'Connected: {cf.uri}')

        # Wait for lighthouse state
        print('Waiting for lighthouse state...')
        timeout = 8.0
        start = time.time()
        while not cf.state_ready and (time.time() - start) < timeout:
            time.sleep(0.1)

        if not cf.state_ready:
            print('FAIL: Lighthouse state not received within 8 seconds')
            print('      Check Task 1A geometry calibration')
            sys.exit(1)

        x0 = cf.current_x
        y0 = cf.current_y
        print(f'Lighthouse lock: x={x0:.3f}  y={y0:.3f}  z={cf.current_z:.3f}')

        if abs(x0) < 1e-4 and abs(y0) < 1e-4 and abs(cf.current_z) < 1e-4:
            print('WARN: Position is (0, 0, 0) — lighthouse may not be tracking.')
            choice = input('Continue anyway? Type "yes" to proceed: ').strip().lower()
            if choice != 'yes':
                sys.exit(1)

        print('\nFlight area clear? Confirm before takeoff.')
        input('Press ENTER to TAKEOFF, or Ctrl+C to abort...')

        # Takeoff
        print(f'Taking off to z={Z_HOLD}m...')
        cf.takeoff(z_hold=Z_HOLD, duration=3.0)
        print('Airborne.')

        # Hover and monitor drift
        print(f'Hovering for {HOVER_TIME:.0f}s — monitoring position drift...')
        t_start = time.time()
        passed = True
        while time.time() - t_start < HOVER_TIME:
            dx = cf.current_x - x0
            dy = cf.current_y - y0
            drift = (dx ** 2 + dy ** 2) ** 0.5
            elapsed = time.time() - t_start
            print(f'  t={elapsed:.1f}s  x={cf.current_x:.3f}  y={cf.current_y:.3f}  '
                  f'z={cf.current_z:.3f}  drift={drift:.3f}m')

            # Send a zero velocity setpoint to maintain hover
            cf.send_velocity_setpoint(0.0, 0.0, 0.0, Z_HOLD)

            if drift > DRIFT_LIMIT:
                print(f'WARN: Drift {drift:.3f}m exceeds limit {DRIFT_LIMIT}m')
                passed = False

            time.sleep(1.0)

        # Land
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
        raise

    finally:
        if cf and cf.is_connected:
            cf.disconnect()
        rclpy.shutdown()
        print('Disconnected.')

    if passed:
        print('\n=== TASK 5b PASS — Safe to proceed to waypoint flight ===')
    else:
        print('\n=== TASK 5b WARN: Drift exceeded limit — review lighthouse stability before Task 6 ===')


if __name__ == '__main__':
    main()
