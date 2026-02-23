import sys
import os
import time
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../.."))

from ros_sugar import Launcher
from robots.crazyflie.cf_node import CrazyflieComponent

URI        = "radio://0/80/2M/E7E7E7E7E7"
HOVER_Z    = 0.5
HOVER_SECS = 3.0

def flight_sequence(drone):
    time.sleep(2.0)  # wait for node to fully activate
    print("[Flight] Starting sequence ...")
    drone.takeoff(height=HOVER_Z)
    print(f"[Flight] Hovering {HOVER_SECS}s ...")
    time.sleep(HOVER_SECS)
    drone.land()
    print("[Flight] Done.")

def main():
    print("=== Hello Crazyflie (Sugarcoat) ===")

    drone = CrazyflieComponent(
        uri=URI,
        hover_z=HOVER_Z,
        component_name="crazyflie",
    )

    # Start flight in background thread
    t = threading.Thread(target=flight_sequence, args=(drone,), daemon=True)
    t.start()

    launcher = Launcher()
    launcher.add_pkg(components=[drone])
    launcher.bringup()

if __name__ == "__main__":
    main()