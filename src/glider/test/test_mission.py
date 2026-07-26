"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for mission.py: launch.config load, live update (launch id / position / RTC time setup),
persistence, and Inspector integration. Positive + negative.
"""

import asyncio
import json
import os

import databoard
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
    assert mission._load(PATH) == mission._HPRC_DEFAULT  # missing file -> HPRC default, never raises
    with open(PATH, 'w') as f:
        f.write('{ not json')
    assert mission._load(PATH) == mission._HPRC_DEFAULT  # corrupt file -> HPRC default too
    _cleanup()


def test_defaults_and_register():
    _cleanup()
    launch = mission.Mission(PATH)
    assert launch.launch_id == '' and launch.site == 'HPRC'  # no file -> HPRC pad default
    assert launch.latitude == 25.514379 and launch.longitude == -80.391795 and launch.zone is not None
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
    # negative: an epoch before 2000-01-01 is rejected (would set a pre-2000 RTC)
    assert launch.set_time(100) is False
    assert launch.set_time(mission._EPOCH_OFFSET - 1) is False


def test_save_roundtrip():
    _cleanup()
    launch = mission.Mission(PATH)
    # set every persisted field explicitly, incl. zone -- a cleaned file falls back to the HPRC default
    # mission (which carries a zone), so do not assume a blank start; assert the values we set round-trip.
    launch.update({'launch_id': 'save-me', 'latitude': 1.0, 'longitude': 2.0, 'altitude': 5,
                   'zone': [[1.0, 2.0], [0.0, 3.0]]})
    launch.save()
    # a fresh Mission reads the persisted launch.config back
    reloaded = mission.Mission(PATH)
    assert reloaded.launch_id == 'save-me'
    assert reloaded.latitude == 1.0 and reloaded.longitude == 2.0 and reloaded.altitude == 5
    # the clock is never persisted
    with open(PATH) as f:
        assert 'clock' not in json.load(f)

    # persisted() is the editable launch.config the dashboard loads (get-config launch): the launch fields
    # + zone, no computed geometry/clock; and it round-trips through save()
    snapshot = launch.persisted()
    assert snapshot['launch_id'] == 'save-me' and snapshot['latitude'] == 1.0
    assert snapshot['zone'] == [[1.0, 2.0], [0.0, 3.0]]  # zone round-trips as plain lists
    assert 'clock' not in snapshot and 'target' not in snapshot
    with open(PATH) as f:
        assert json.load(f) == snapshot  # save() writes exactly persisted()
    _cleanup()


def test_landing_zone():
    _cleanup()
    launch = mission.Mission(PATH)
    assert launch.zone is not None  # a cleaned (missing) file falls back to the HPRC default, which has a zone

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


def test_zone_geometry_and_range():
    _cleanup()
    launch = mission.Mission(PATH)
    launch.update({'latitude': 48.00025, 'longitude': 11.0005})  # launch at the zone centre
    launch.update({'zone': [[48.0005, 11.000], [48.0000, 11.001]]})  # ~74 m wide -> gates ~37 m away
    snap = launch.inspect()
    # all 3 points (target + both gates) and their range from the launch point are exposed
    assert len(snap['gates']) == 2 and set(snap['distances_m']) == {'target', 'gate_a', 'gate_b'}
    assert snap['distances_m']['target'] < 2.0  # centre ~ the launch point
    assert 30.0 < snap['distances_m']['gate_a'] < 45.0  # ~37 m to a short-side gate
    assert snap['in_range'] is True

    # move the launch point far -> all points blow the range -> in_range flag + probe both fail
    launch.update({'latitude': 48.005})  # ~530 m north of the zone
    assert launch.inspect()['in_range'] is False
    assert 'out of range' in asyncio.run(launch.probe())

    # the threshold is board config (airframe glide range), not hardcoded: a tighter range flags it
    tight = mission.Mission(PATH, max_range_m=10.0)
    tight.update({'latitude': 48.00025, 'longitude': 11.0005, 'zone': [[48.0005, 11.000], [48.0000, 11.001]]})
    assert tight.inspect()['in_range'] is False  # ~37 m gates exceed the 10 m range
    _cleanup()


def test_launch_point_from_gnss():
    _cleanup()
    # present-but-empty config -> blank mission (a MISSING file loads the HPRC default, which has a position)
    with open(PATH, 'w') as f:
        json.dump({}, f)
    launch = mission.Mission(PATH)  # no CC-set position
    assert launch.launch_point() is None  # nothing from CC, no fix yet
    launch.freeze_launch()  # arming without a fix freezes nothing (stays live-fallthrough)
    assert launch.latitude is None and launch.longitude is None
    assert launch.inspect()['launch_point'] is None and launch.inspect()['launch_source'] is None
    channel = databoard.Databoard.provide('gnss', {'position': {'priority': 0, 'timeout_ms': 1000}}, 'position')
    channel.push((48.0, 11.0))
    assert launch.launch_point() == (48.0, 11.0)  # set by GPS (databoard) when CC has not
    # inspect shows the EFFECTIVE origin + its source even while latitude/longitude read None
    snapshot = launch.inspect()
    assert snapshot['launch_point'] == (48.0, 11.0) and snapshot['launch_source'] == 'gnss'
    assert snapshot['latitude'] is None

    # freeze_launch (at arm): the live fix becomes the PERSISTENT launch point, so a mid-flight fix
    # loss no longer costs the tier-2 heading; an operator-set position is never overwritten
    launch.freeze_launch()
    assert (launch.latitude, launch.longitude) == (48.0, 11.0)
    assert launch.inspect()['launch_source'] == 'set'  # frozen reads as the persistent position
    channel.push((49.0, 12.0))  # the fix moves on -- the frozen pad point holds
    assert launch.launch_point() == (48.0, 11.0)
    launch.freeze_launch()  # a second arm never overwrites (operator-set wins the same way)
    assert (launch.latitude, launch.longitude) == (48.0, 11.0)
    _cleanup()


def test_sites_and_fallback():
    """
    CC-less site selection (doc/specs/coludo.md "Field operation without CC"): sites parse (invalid
    entries dropped), the nearest in-range pad wins, none in range -> None; the fallback zone is
    centred offset_m from the fix at bearing_deg and adopted as the mission zone.
    """
    _cleanup()
    with open(PATH, 'w') as f:
        f.write(json.dumps({'sites': [
            {'name': 'north', 'pad': [48.0010, 11.0], 'zone': [[48.0020, 10.999], [48.0015, 11.001]]},
            {'name': 'south', 'pad': [48.0000, 11.0], 'zone': [[47.9990, 10.999], [47.9985, 11.001]]},
            {'pad': [200.0, 11.0], 'zone': [[1, 1], [2, 2]]},   # bad latitude -> dropped
            {'name': 'no-zone', 'pad': [48.0, 11.0]},           # missing zone -> dropped
        ]}))
    launch = mission.Mission(PATH, max_range_m=200)
    assert len(launch.sites) == 2  # the two invalid entries never made the list
    # a fix ~55 m south of 'south': south (~55 m) beats north (~166 m) -> south's zone adopted
    assert launch.select_site((47.9995, 11.0)) == 'south'
    assert launch.site == 'south' and abs(launch.zone[0][0] - 47.9990) < 1e-9
    assert launch.latitude is None  # the launch POINT stays the live fix (not the stored pad)
    # a fix 1 km away: nothing within max_range_m -> None, the zone is untouched
    before = launch.zone
    assert launch.select_site((48.0090, 11.0)) is None and launch.zone == before
    # fallback: a GENEROUS 100 m (wide, E-W) x 90 m (deep, N-S) box NORTH of the fix (bearing 0),
    # near edge 50 m out (safety), centre near+depth/2 = 95 m from the pad
    import navigation
    fix = (48.0, 11.0)
    zone = launch.fallback_zone(fix, bearing_deg=0.0, near_m=50.0, width_m=100.0, depth_m=90.0)
    assert launch.site == 'fallback' and launch.zone == zone
    centre_lat = (zone[0][0] + zone[1][0]) / 2
    centre_lon = (zone[0][1] + zone[1][1]) / 2
    assert abs(navigation.distance(fix[0], fix[1], centre_lat, centre_lon) - 95.0) < 1.0  # centre ~95 m N
    assert centre_lat > fix[0] and abs(centre_lon - fix[1]) < 1e-9  # due north
    # near edge (south) ~50 m from the pad; box ~90 m deep (N-S) x ~100 m wide (E-W)
    near = navigation.distance(fix[0], fix[1], zone[1][0], centre_lon)  # BR lat = south edge
    assert abs(near - 50.0) < 1.0
    depth = navigation.distance(zone[0][0], centre_lon, zone[1][0], centre_lon)
    width = navigation.distance(centre_lat, zone[0][1], centre_lat, zone[1][1])
    assert abs(depth - 90.0) < 1.0 and abs(width - 100.0) < 1.0
    assert navigation.inside((centre_lat, centre_lon), zone[0], zone[1])
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
        test_zone_geometry_and_range()
        test_launch_point_from_gnss()
        test_sites_and_fallback()
    finally:
        _cleanup()
    print('ok: mission load/update/time/save + landing-zone geometry/range + GNSS launch-point + '
          'site-by-GPS + fallback zone + Inspector')


main()
