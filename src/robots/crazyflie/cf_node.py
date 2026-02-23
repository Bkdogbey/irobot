"""
Crazyflie Component
===================
A ros_sugar BaseComponent that wraps cflib.

Publishes:
    /cf/pose  (geometry_msgs/PoseStamped)  — position at 50Hz
Subscribes:
    /cf/cmd   (geometry_msgs/Twist)        — velocity commands
"""

import time
import logging
from ros_sugar.core import BaseComponent
from ros_sugar.io import Topic

from geometry_msgs.msg import PoseStamped, Twist

logging.basicConfig(level=logging.ERROR)

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    HAS_CFLIB = True
except ImportError:
    HAS_CFLIB = False


class CrazyflieComponent(BaseComponent):

    def __init__(
        self,
        uri: str = "radio://0/80/2M/E7E7E7E7E7",
        hover_z: float = 0.5,
        component_name: str = "crazyflie",
        **kwargs,
    ):
        self.uri     = uri
        self.hover_z = hover_z
        self._scf    = None
        self._cf     = None

        # Current pose (updated from Lighthouse or mock)
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0

        # Define ROS2 topics
        inputs  = [Topic(Twist,       "/cf/cmd")]
        outputs = [Topic(PoseStamped, "/cf/pose")]

        super().__init__(
            component_name,
            inputs=inputs,
            outputs=outputs,
            **kwargs
        )

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def on_start(self):
        if not HAS_CFLIB:
            print("[MOCK] cflib not found — mock mode")
            return
        cflib.crtp.init_drivers()
        self._scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self._scf.open_link()
        self._cf = self._scf.cf
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        print("Connected to Crazyflie!")

    def on_stop(self):
        if self._scf:
            self._scf.close_link()

    def _execution_step(self):
        """Called on each timer tick — publish current pose."""
        self._publish_pose()

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def cmd_callback(self, msg: Twist):
        """Receive velocity command from /cf/cmd."""
        vx = msg.linear.x
        vy = msg.linear.y
        print(f"[CFNode] Cmd received: vx={vx:.2f} vy={vy:.2f}")
        # TODO: forward to drone when hardware is connected

    # ── Publishing ────────────────────────────────────────────────────────────

    def _publish_pose(self):
        """Publish current position to /cf/pose."""
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = self._z
        msg.pose.orientation.w = 1.0

        # Publish via Sugarcoat output
        self.publishers["/cf/pose"].publish(msg)

    # ── Flight commands ───────────────────────────────────────────────────────

    def takeoff(self, height=None, duration=3.0):
        height = height or self.hover_z
        print(f"[CFNode] Takeoff → {height}m")
        if not self._cf:
            print(f"[MOCK] Takeoff to {height}m")
            self._z = height
            return
        steps = 50
        for i in range(steps):
            thrust = int(10001 + (45000 - 10001) * (i / steps))
            self._cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(duration / steps)
        self._z = height

    def land(self, duration=2.0):
        print("[CFNode] Landing ...")
        if not self._cf:
            print("[MOCK] Landing")
            self._z = 0.0
            return
        steps = 50
        for i in range(steps):
            thrust = int(45000 * (1 - i / steps))
            self._cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(duration / steps)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        self._z = 0.0
        print("Landed.")