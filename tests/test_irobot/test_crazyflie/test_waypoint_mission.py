import pytest

from irobot.src.robots.crazyflie.core.waypoint_mission import (
    Geofence,
    GeofenceViolationError,
    WaypointMission,
)


class FakeCommander:
    def __init__(self, fail_at=None):
        self.calls = []
        self.fail_at = fail_at

    def take_off(self, height):
        self.calls.append(('take_off', height))

    def go_to(self, x, y, z):
        if self.fail_at is not None and len([c for c in self.calls if c[0] == 'go_to']) == self.fail_at:
            raise RuntimeError('link lost')
        self.calls.append(('go_to', x, y, z))

    def land(self):
        self.calls.append(('land',))


FENCE = Geofence(x_range=(-1.0, 1.0), y_range=(-2.0, 1.0), z_range=(0.0, 1.0), margin=0.1)
WAYPOINTS = [(0.0, -1.5, 0.3), (0.2, -1.0, 0.3), (0.0, 0.5, 0.3)]


def make_mission(commander, **kwargs):
    defaults = dict(geofence=FENCE, takeoff_z=0.3, return_z=0.5, sleep=lambda _: None)
    defaults.update(kwargs)
    return WaypointMission(commander, **defaults)


def test_full_mission_sequence():
    cmd = FakeCommander()
    make_mission(cmd).run(WAYPOINTS)

    assert cmd.calls[0] == ('take_off', 0.3)
    go_tos = [c for c in cmd.calls if c[0] == 'go_to']
    assert go_tos[:3] == [('go_to', *wp) for wp in WAYPOINTS]
    # return-to-start leg: over the last waypoint, then over the first, at return_z
    assert go_tos[3] == ('go_to', 0.0, 0.5, 0.5)
    assert go_tos[4] == ('go_to', 0.0, -1.5, 0.5)
    assert cmd.calls[-1] == ('land',)


def test_no_return_to_start():
    cmd = FakeCommander()
    make_mission(cmd, return_to_start=False).run(WAYPOINTS)
    go_tos = [c for c in cmd.calls if c[0] == 'go_to']
    assert len(go_tos) == 3


def test_geofence_precheck_rejects_before_takeoff():
    cmd = FakeCommander()
    bad = WAYPOINTS + [(5.0, 0.0, 0.3)]
    with pytest.raises(GeofenceViolationError):
        make_mission(cmd).run(bad)
    assert cmd.calls == []  # never took off


def test_geofence_margin():
    fence = Geofence(x_range=(0.0, 2.0), y_range=(0.0, 2.0), z_range=(0.0, 1.0), margin=0.1)
    assert fence.contains((0.1, 1.0, 0.5))
    assert not fence.contains((0.05, 1.0, 0.5))
    assert fence.check_waypoints([(0.5, 0.5, 0.5), (1.95, 0.5, 0.5)]) == [1]


def test_emergency_land_on_exception():
    cmd = FakeCommander(fail_at=2)
    with pytest.raises(RuntimeError, match='link lost'):
        make_mission(cmd).run(WAYPOINTS)
    assert cmd.calls[-1] == ('land',)  # emergency landing still happened


def test_on_waypoint_callback():
    cmd = FakeCommander()
    logged = []
    make_mission(cmd, on_waypoint=lambda x, y, z: logged.append((x, y, z)), return_to_start=False).run(WAYPOINTS)
    assert logged == WAYPOINTS


def test_empty_mission_rejected():
    with pytest.raises(ValueError):
        make_mission(FakeCommander()).run([])
