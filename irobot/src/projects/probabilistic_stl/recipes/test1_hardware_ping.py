"""
TASK 3 — Test 1: Hardware Ping
Confirm cflib can connect, arm, and cleanly disconnect without any flight.
No motors spin.
"""

import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))  # noqa: PTH100, PTH120
sys.path.insert(0, os.path.join(_HERE, '..'))  # probabilistic_stl/  # noqa: PTH118
sys.path.insert(0, os.path.join(_HERE, '../../..'))  # src/ — robots.crazyflie.*  # noqa: PTH118

import rclpy
from robots.crazyflie.core.base import CrazyflieComponent
from robots.crazyflie.core.config import CrazyflieConfig


def main():
    print('=== TASK 3: Hardware Ping ===')
    print('Connecting to Crazyflie...')

    rclpy.init()
    cf = CrazyflieComponent(component_name='ping_test', config=CrazyflieConfig())

    # Wait for connection
    timeout = 10
    start = time.time()
    while not cf.is_connected and (time.time() - start) < timeout:
        time.sleep(0.2)

    if not cf.is_connected:
        print('FAIL: Could not connect within 10 seconds')
        sys.exit(1)

    print(f'Connected at URI: {cf.uri}')

    # Wait for lighthouse state
    timeout = 5
    start = time.time()
    while not cf.state_ready and (time.time() - start) < timeout:
        time.sleep(0.1)

    if not cf.state_ready:
        print('FAIL: Lighthouse state not received within 5 seconds')
        print('Check: Lighthouse geometry calibrated? (Task 1A)')
        cf.disconnect()
        sys.exit(1)

    print(f'Lighthouse state received:')
    print(f'  x = {cf.current_x:.4f} m')
    print(f'  y = {cf.current_y:.4f} m')
    print(f'  z = {cf.current_z:.4f} m')

    cf.disconnect()
    rclpy.shutdown()
    print('=== TASK 3 PASS ===')


if __name__ == '__main__':
    main()
