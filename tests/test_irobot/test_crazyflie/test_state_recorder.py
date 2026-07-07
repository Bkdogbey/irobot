import time

from irobot.src.robots.crazyflie.core.state_recorder import FIELDNAMES, StateRecorder


def test_records_position_and_velocity(tmp_path):
    recorder = StateRecorder(
        get_state=lambda: {'x': 1.0, 'y': -0.5, 'z': 0.3, 'vx': 0.1, 'vy': 0.0, 'vz': 0.0},
        rate_hz=100.0,
        source='sim',
    )
    recorder.start()
    time.sleep(0.1)
    recorder.stop()

    rows = recorder.rows
    assert len(rows) >= 5
    assert rows[0]['x'] == 1.0
    assert rows[0]['vx'] == 0.1
    assert rows[0]['source'] == 'sim'
    assert rows[-1]['time'] >= rows[0]['time']

    out = recorder.save(tmp_path / 'actual.csv')
    header = out.read_text().splitlines()[0]
    assert header == ','.join(FIELDNAMES)


def test_velocity_optional():
    recorder = StateRecorder(get_state=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.2}, rate_hz=100.0)
    recorder.start()
    time.sleep(0.05)
    recorder.stop()
    assert recorder.rows[0]['vx'] == ''


def test_restart_resets_rows():
    recorder = StateRecorder(get_state=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.0}, rate_hz=100.0)
    recorder.start()
    time.sleep(0.05)
    recorder.stop()
    first = len(recorder.rows)
    assert first > 0

    recorder.start()
    recorder.stop()
    assert len(recorder.rows) < first or recorder.rows[0]['time'] < 0.05
