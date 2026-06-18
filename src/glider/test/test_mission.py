# On-board test for mission.py: launch.config load, live update (launch id / position / RTC time
# setup), persistence, and Inspector integration. Positive + negative.

import json
import os

from inspector import Inspector
from mission import _EPOCH_OFFSET, Mission, _load, _number

PATH = 'test_launch.config'


def _cleanup():
    for p in (PATH, PATH + '.tmp'):
        try:
            os.remove(p)
        except OSError:
            pass


def test_number():
    # positive: in-range numbers pass through; negative: bool / out-of-range / non-number -> None
    assert _number(45.0, -90, 90) == 45.0
    assert _number(0, -90, 90) == 0
    assert _number(200, -90, 90) is None
    assert _number(True, -90, 90) is None  # bool is not a coordinate
    assert _number('45', -90, 90) is None


def test_load_missing():
    _cleanup()
    assert _load(PATH) == {}  # missing file -> empty mission, never raises
    with open(PATH, 'w') as f:
        f.write('{ not json')
    assert _load(PATH) == {}  # corrupt file -> empty too
    _cleanup()


def test_defaults_and_register():
    _cleanup()
    mission = Mission(PATH)
    assert mission.launch_id == '' and mission.site == ''
    assert mission.latitude is None and mission.longitude is None
    # self-registered for `inspect mission`
    assert 'mission' in Inspector.names()
    assert Inspector.get('mission') is mission
    snap = Inspector.inspect('mission')
    assert snap['launch_id'] == '' and len(snap['clock']) == 19


def test_load_from_file():
    with open(PATH, 'w') as f:
        json.dump({'launch_id': 'hprc-t1', 'site': 'pad-a', 'latitude': 45.5, 'longitude': -73.5, 'altitude': 120}, f)
    mission = Mission(PATH)
    assert mission.launch_id == 'hprc-t1' and mission.site == 'pad-a'
    assert mission.latitude == 45.5 and mission.longitude == -73.5 and mission.altitude == 120
    # an out-of-range coordinate in the file is dropped to None
    with open(PATH, 'w') as f:
        json.dump({'latitude': 200, 'longitude': -73.5}, f)
    bad = Mission(PATH)
    assert bad.latitude is None and bad.longitude == -73.5
    _cleanup()


def test_update_launch_id():
    mission = Mission(PATH)
    assert mission.update({'launch_id': 'flight.7'}) == ['launch_id']
    assert mission.launch_id == 'flight.7'
    assert Inspector.inspect('mission')['launch_id'] == 'flight.7'


def test_update_positive_and_negative():
    mission = Mission(PATH)
    # positive: valid coordinates stored, reported changed
    changed = mission.update({'latitude': 10.0, 'longitude': 20.0, 'site': 'home'})
    assert sorted(changed) == ['latitude', 'longitude', 'site']
    assert mission.latitude == 10.0 and mission.site == 'home'
    # negative: re-applying the same values changes nothing
    assert mission.update({'latitude': 10.0, 'site': 'home'}) == []
    # negative: out-of-range coordinates are ignored, value unchanged
    assert mission.update({'latitude': 999}) == []
    assert mission.latitude == 10.0


def test_time_setup():
    mission = Mission(PATH)
    # positive: a Unix epoch sets the RTC; clock + epoch round-trip back to it
    epoch = 1781000000  # some moment in 2026
    assert mission.set_time(epoch) is True
    assert abs(mission.epoch() - epoch) <= 3
    assert len(mission.clock()) == 19 and mission.clock()[:2] == '20'
    # via update(), 'epoch' is reported changed but never stored as a field
    assert 'epoch' in mission.update({'epoch': epoch + 100})
    assert not hasattr(mission, 'epoch_value')
    # negative: non-int / bool epochs are rejected
    assert mission.set_time(1.5) is False
    assert mission.set_time(True) is False
    assert mission.update({'epoch': 'now'}) == []


def test_save_roundtrip():
    _cleanup()
    mission = Mission(PATH)
    mission.update({'launch_id': 'save-me', 'latitude': 1.0, 'longitude': 2.0, 'altitude': 5})
    mission.save()
    # a fresh Mission reads the persisted launch.config back
    reloaded = Mission(PATH)
    assert reloaded.launch_id == 'save-me'
    assert reloaded.latitude == 1.0 and reloaded.longitude == 2.0 and reloaded.altitude == 5
    # the clock is never persisted
    with open(PATH) as f:
        assert 'clock' not in json.load(f)
    _cleanup()


def main():
    assert _EPOCH_OFFSET == 946684800
    try:
        test_number()
        test_load_missing()
        test_defaults_and_register()
        test_load_from_file()
        test_update_launch_id()
        test_update_positive_and_negative()
        test_time_setup()
        test_save_roundtrip()
    finally:
        _cleanup()
    print('ok: mission load/update/time-setup/launch-prefix/save + Inspector')


main()
