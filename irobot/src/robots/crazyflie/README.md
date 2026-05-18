# Crazyflie Driver

The Crazyflie driver wraps [Bitcraze's cflib](https://github.com/bitcraze/crazyflie-lib-python) into a clean Python class hierarchy that handles connection management, Lighthouse position tracking, and flight control. It works standalone or as a ROS2 component via ros_sugar.

---

## Hardware Requirements

- **Crazyflie 2.x** — the drone itself
- **Crazyradio PA** — the USB radio dongle used to communicate with the drone

---

## Prerequisites

Install the Crazyflie Python library:

```bash
pip install cflib
```

On Linux, you also need to configure USB device permissions so the radio can be accessed without root. See [Hardware Setup (Linux)](#hardware-setup-linux) below.

---

## Hardware Setup (Linux)

By default, USB devices require root privileges. The steps below grant your user account access to the Crazyradio and Crazyflie without `sudo`.

### 1. Add your user to the `plugdev` group

```bash
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
```

Log out and back in after running these commands for the group change to take effect.

### 2. Create the udev rules file

Create `/etc/udev/rules.d/99-bitcraze.rules` with the following content:

```plaintext
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
```

You can create it with any editor, for example:

```bash
sudo nano /etc/udev/rules.d/99-bitcraze.rules
```

### 3. Reload udev rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The Crazyradio and Crazyflie should now be accessible without root privileges.

---

## Configuration

All hardware parameters are collected in a single `CrazyflieConfig` dataclass. Create one and pass it to the controller:

```python
from irobot import CrazyflieConfig

config = CrazyflieConfig(
    uri='radio://0/80/2M/E7E7E7E781',  # match your drone's address
    connect_timeout=15.0,
)
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `uri` | `str` | `'radio://0/80/2M/E7E7E7E781'` | Radio URI identifying the drone |
| `log_rate_ms` | `int` | `50` | State logging period in milliseconds (50 ms = 20 Hz) |
| `connect_timeout` | `float` | `10.0` | Maximum seconds to wait for a connection |
| `hover_z` | `float` | `0.3` | Default hover altitude in metres |

To find your drone's URI, use the `cflib` scanner:

```bash
python -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

---

## Usage

### Standalone Python

```python
from irobot import CrazyflieController, CrazyflieConfig

config = CrazyflieConfig(uri='radio://0/80/2M/E7E7E7E781')
drone = CrazyflieController(config)   # connects automatically on construction

drone.takeoff(z_hold=0.5, duration=3.0)
drone.fly_to(x=1.0, y=0.0, z=0.5)
drone.fly_to(x=1.0, y=1.0, z=0.5)
drone.land()

drone.disconnect()
```

### ROS2 Component (ros_sugar)

Copy `examples/crazyflie_ros_component.py` into your project and extend the `_execute_once` method with your mission logic:

```python
from irobot.src.robots.crazyflie.examples.crazyflie_ros_component import CrazyflieDemo

# Or copy the file and subclass it:
class MyMission(CrazyflieDemo):
    def _execute_once(self) -> None:
        # your waypoints and logic here
        ...
```

The example uses `PositionHlCommander` from cflib for high-level position commands and integrates with the ros_sugar `Launcher` in `main.py`.

---

## API Reference

### `CrazyflieConfig`
Dataclass. All hardware parameters. See [Configuration](#configuration) above.

---

### `CrazyflieBase`
Connection lifecycle manager. Inherits from `LighthouseLogging`.

| Method / Property | Description |
|-------------------|-------------|
| `connect() → bool` | Opens the radio link; returns `True` when connected |
| `disconnect()` | Closes the radio link |
| `arm()` | Sends arming request to the platform |
| `disarm()` | Sends disarming request to the platform |
| `get_status() → dict` | Returns `uri`, `connected`, and current timestamp |
| `is_connected: bool` | `True` when the link is active |
| `current_x/y/z: float` | Live position estimate from Lighthouse (metres) |
| `current_vx/vy: float` | Live velocity estimate (m/s) |
| `current_yaw: float` | Live yaw estimate (degrees) |
| `state_ready: bool` | `True` once the first Lighthouse log packet has arrived |

Override `_publish_state()` in a subclass to forward state to ROS or any other transport.

---

### `CrazyflieController`
Flight control built on top of `CrazyflieBase`.

| Method | Description |
|--------|-------------|
| `takeoff(z_hold, duration)` | Hovers at `z_hold` metres for `duration` seconds to establish stable flight |
| `land(z_hold, duration)` | Ramps altitude from `z_hold` to zero and cuts motors |
| `fly_to(x, y, z, yaw, duration)` | Flies to a position from the current location using smooth interpolation |
| `fly_from_to(start, end, travel_time, hover_time)` | Full sequence: takeoff → hover at start → fly to end → hover → land |
| `send_setpoint(roll, pitch, yaw_rate, thrust)` | Sends a raw attitude setpoint |
| `send_velocity_setpoint(vx, vy, yaw_rate, z_hold)` | Sends a 2D velocity command at a fixed hover height |
| `ramp_motors()` | Ramps motors up and back down — useful for a quick motor health check |

---

### `LighthouseLogging`
Mixin that sets up Lighthouse position logging via cflib's log framework. Configured automatically on connection. Override `_publish_state()` to act on each state update.

---

For more on the Crazyflie hardware and cflib, see the [Bitcraze documentation](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/).
