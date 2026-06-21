# On-board test for mission.py: launch.config load, live update (launch id / position / RTC time
# setup), persistence, and Inspector integration. Positive + negative.

import json
import os

import inspector
import mission

PATH = 'test_launch.config'


def _cleanup():
    for p in (PATH, PATH + '.tmp'):
        try:
            os.remove(p)
        except OSError:
            pass


def test_number():
    # positive: in-range numbers pass through; negative: bool / out-of-range / non-number -> None
    assert mission._number(45.0, -90, 90) == 45.0
    assert mission._number(0, -90, 90) == 0
    assert mission._number(200, -90, 90) is None
    assert mission._number(True, -90, 90) is None  # bool is not a coordinate
    assert mission._number('45', -90, 90) is None


def test_load_missing():
    _cleanup()
    assert mission._load(PATH) == {}  # missing file -> empty mission, never raises
    with open(PATH, 'w') as f:
        f.write('{ not json')
    assert mission._load(PATH) == {}  # corrupt file -> empty too
    _cleanup()


def test_defaults_and_register():
    _cleanup()
    launch = mission.Mission(PATH)
    assert launch.launch_id == '' and launch.site == ''
    assert launch.latitude is None and launch.longitude is None
    # self-registered for `inspect mission`
    assert 'mission' in inspector.Inspector.names()
    assert inspector.Inspector.get('mission') is launch
    snap = inspector.Inspector.inspect('mission')
    assert snap['launch_id'] == '' and len(snap['clock']) == 19


def test_load_from_file():
    with open(PATH, 'w') as f:
        json.dump({'launch_id': 'hprc-t1', 'site': 'pad-a', 'latitude': 45.5, 'longitude': -73.5, 'altitude': 120}, f)
    launch = mission.Mission(PATH)
    assert launch.launch_id == 'hprc-t1' and launch.site == 'pad-a'
    assert launch.latitude == 45.5 and launch.longitude == -73.5 and launch.altitude == 120
    # an out-of-range coordinate in the file is dropped to None
    with open(PATH, 'w') as f:
        json.dump({'latitude': 200, 'longitude': -73.5}, f)
    bad = mission.Mission(PATH)
    assert bad.latitude is None and bad.longitude == -73.5
    _cleanup()


def test_update_launch_id():
    launch = mission.Mission(PATH)
    assert launch.update({'launch_id': 'flight.7'}) == ['launch_id']
    assert launch.launch_id == 'flight.7'
    assert inspector.Inspector.inspect('mission')['launch_id'] == 'flight.7'


def test_update_positive_and_negative():
    launch = mission.Mission(PATH)
    # positive: valid coordinates stored, reported changed
    changed = launch.update({'latitude': 10.0, 'longitude': 20.0, 'site': 'home'})
    assert sorted(changed) == ['latitude', 'longitude', 'site']
    assert launch.latitude == 10.0 and launch.site == 'home'
    # negative: re-applying the same values changes nothing
    assert launch.update({'latitude': 10.0, 'site': 'home'}) == []
    # negative: out-of-range coordinates are ignored, value unchanged
    assert launch.update({'latitude': 999}) == []
    assert launch.latitude == 10.0


def test_time_setup():
    launch = mission.Mission(PATH)
    # positive: a Unix epoch sets the RTC; clock + epoch round-trip back to it
    epoch = 1781000000  # some moment in 2026
    assert launch.set_time(epoch) is True
    assert abs(launch.epoch() - epoch) <= 3
    assert len(launch.clock()) == 19 and launch.clock()[:2] == '20'
    # via update(), 'epoch' is reported changed but never stored as a field
    assert 'epoch' in launch.update({'epoch': epoch + 100})
    assert not hasattr(launch, 'epoch_value')
    # negative: non-int / bool epochs are rejected
    assert launch.set_time(1.5) is False
    assert launch.set_time(True) is False
    assert launch.update({'epoch': 'now'}) == []


def test_save_roundtrip():
    _cleanup()
    launch = mission.Mission(PATH)
    launch.update({'launch_id': 'save-me', 'latitude': 1.0, 'longitude': 2.0, 'altitude': 5})
    launch.save()
    # a fresh Mission reads the persisted launch.config back
    reloaded = mission.Mission(PATH)
    assert reloaded.launch_id == 'save-me'
    assert reloaded.latitude == 1.0 and reloaded.longitude == 2.0 and reloaded.altitude == 5
    # the clock is never persisted
    with open(PATH) as f:
        assert 'clock' not in json.load(f)
    _cleanup()


def test_landing_zone():
    _cleanup()
    launch = mission.Mission(PATH)
    assert launch.zone is None  # unset by default

    # a valid 2-corner rectangle is stored (tuples) + reported changed
    assert launch.update({'zone': [[48.001, 11.000], [48.000, 11.010]]}) == ['zone']
    assert launch.zone == ((48.001, 11.000), (48.000, 11.010))
    assert launch.inspect()['zone'] == ((48.001, 11.000), (48.000, 11.010))
    # negative: malformed / out-of-range zones are ignored (the valid one stays)
    assert launch.update({'zone': [[48.0, 11.0]]}) == []  # only one corner
    assert launch.update({'zone': [[200.0, 11.0], [48.0, 11.0]]}) == []  # bad latitude
    assert launch.zone == ((48.001, 11.000), (48.000, 11.010))

    # round-trips through launch.config (JSON lists -> tuples on reload)
    launch.save()
    assert mission.Mission(PATH).zone == ((48.001, 11.000), (48.000, 11.010))
    _cleanup()


def main():
    assert mission._EPOCH_OFFSET == 946684800
    try:
        test_number()
        test_load_missing()
        test_defaults_and_register()
        test_load_from_file()
        test_update_launch_id()
        test_update_positive_and_negative()
        test_time_setup()
        test_save_roundtrip()
        test_landing_zone()
    finally:
        _cleanup()
    print('ok: mission load/update/time-setup/launch-prefix/save + landing-zone + Inspector')


main()
