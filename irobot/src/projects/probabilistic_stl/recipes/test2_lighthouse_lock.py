"""
TASK 4 — Test 2: Lighthouse Lock Validation
Confirm lighthouse tracking is stable and accurate enough to trust for flight.
No motors spin.
"""

import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))           # probabilistic_stl/
sys.path.insert(0, os.path.join(_HERE, '../../..'))     # src/ — robots.crazyflie.*

import rclpy
from robots.crazyflie.core.base import CrazyflieComponent
from robots.crazyflie.core.config import CrazyflieConfig


def main():
    print('=== TASK 4: Lighthouse Lock Validation ===')

    rclpy.init()
    cf = CrazyflieComponent(component_name='lh_test', config=CrazyflieConfig())

    timeout = 10
    start = time.time()
    while not cf.is_connected and (time.time() - start) < timeout:
        time.sleep(0.2)

    if not cf.is_connected:
        print('FAIL: Could not connect')
        sys.exit(1)

    timeout = 5
    start = time.time()
    while not cf.state_ready and (time.time() - start) < timeout:
        time.sleep(0.1)

    if not cf.state_ready:
        print('FAIL: Lighthouse state not received')
        cf.disconnect()
        sys.exit(1)

    # Phase 1: Stationary stability test
    print('\nPhase 1: Stationary stability (10 seconds) — do not move the drone')
    xs, ys, zs = [], [], []
    for _ in range(100):  # 10 Hz for 10 seconds
        xs.append(cf.current_x)
        ys.append(cf.current_y)
        zs.append(cf.current_z)
        time.sleep(0.1)

    xs, ys, zs = np.array(xs), np.array(ys), np.array(zs)
    print(f'  x: mean={xs.mean():.4f}  std={xs.std():.4f}  range=[{xs.min():.4f}, {xs.max():.4f}]')
    print(f'  y: mean={ys.mean():.4f}  std={ys.std():.4f}  range=[{ys.min():.4f}, {ys.max():.4f}]')
    print(f'  z: mean={zs.mean():.4f}  std={zs.std():.4f}  range=[{zs.min():.4f}, {zs.max():.4f}]')

    stable = xs.std() < 0.05 and ys.std() < 0.05
    if not stable:
        print('WARN: Position noise exceeds 5cm std — lighthouse may not be fully locked')
        print('      Consider re-running Task 1A geometry calibration')

    # Phase 2: Manual movement check
    input('\nPhase 2: Pick up the drone and move it ~1m in X, press ENTER when moved...')
    x_moved = cf.current_x
    y_moved = cf.current_y
    print(f'  Position after move: x={x_moved:.4f}  y={y_moved:.4f}')
    print(f'  Delta from origin:   dx={x_moved - xs.mean():.4f}  dy={y_moved - ys.mean():.4f}')

    input('Move it back to start position, press ENTER when returned...')
    x_back = cf.current_x
    y_back = cf.current_y
    drift = np.sqrt((x_back - xs.mean()) ** 2 + (y_back - ys.mean()) ** 2)
    print(f'  Return drift: {drift:.4f}m')

    cf.disconnect()
    rclpy.shutdown()

    if stable:
        print('=== TASK 4 PASS ===')
    else:
        print('=== TASK 4 WARN: Passed with noise warning — review before flight ===')


if __name__ == '__main__':
    main()

