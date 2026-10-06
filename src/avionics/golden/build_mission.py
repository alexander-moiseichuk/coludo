"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture mission.json: plan B's Mission (src/glider/mission.py) as the Rust port (crates/mission) must
answer it -- launch.config load, update(), the time-set rule, persisted(), probe(), inspect() and the board clock
text.

Three members carry cases:
  * scenarios -- stateful: a launch.config text (or none) and max_range_m to start from, then steps. Each step
    is an action with the GNSS fix it runs under, and plan B's answer after it: the changed list, persisted(),
    the in-memory sites, probe(), and inspect() minus clock and epoch (the caller's clock). The actions:
      observe     nothing; the state is read under the step's fix
      update      `update mission` (Mission.update; with an accepted epoch, plan B lists 'epoch' last)
      set_config  `set-config launch`: Mission.update, then save() -- what cc_client's set_config does
      reboot      a new Mission from the launch.config plan B last saved (or was started from)
  * time_sets -- one update each on a fresh HPRC mission: the changed list, the epoch set_time accepted, and the
    momentary inputs plan B hands its session.csv row.
  * clocks -- an epoch set through set_time, and the text clock() then reads back.

The fixture compares under abs 5.01e-8 (see `tolerance`): degrees to half a 1e-7 degree unit (the Rust units;
a zone centre is the integer mean, at most half a unit off), so a one-unit slip in a corner, gate or pad fails;
and every metre value must agree exactly. That holds because each reported distance is rounded to 0.1 m, and the
builder refuses a case whose unrounded distance lies closer to a rounding boundary than the Rust answer can
differ from plan B's (_check_boundaries).

DELIBERATE DIVERGENCES (marked, so the Rust test expects them; a step where the Rust answer differs carries
`divergence` and a note):
  * sites -- plan B's persisted() drops `sites`, so a set-config launch erases the CC-less site list at the next
    boot, and its set-config ignores a `sites` payload. The Rust persisted() always carries `sites`, and its
    set-config path (Mission::merge) replaces them with the payload's valid entries when `sites` is a list. The
    step carries the Rust values (`sites`, `changed`).
  * number kind -- plan B echoes the JSON number it was given, so an altitude, coordinate or zone corner sent
    as an integer reads back as one. The Rust port holds metres as f32 and degrees as 1e-7 units and writes
    every one as a JSON float. The step carries `floats: true`: the integers in its persisted, inspect and sites
    answers go out as floats there (the values are equal; only the JSON kind differs, and the golden comparison
    holds an integer exact).
Values whose JSON type plan B stores as-is but the Rust port does not (a numeric launch_id, a text altitude, a
non-text site name) are left to the Rust unit tests.

The clock is the shims' virtual one: set_time writes machine.RTC, and clock() reads that time back. Two more
things the builder stands in, without touching the Mission's logic:
  * recorder.Recorder.index_session -- records the row's inputs instead of queueing a row (no Recorder runs);
  * the databoard 'position' channel -- pushed with the step's fix, or dated back past its window for none.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import json
import math
import os
import random
import tempfile
import time

import databoard
import mission
import navigation
import recorder

_SEED: int = 20261006  # the clock sweep's random.Random seed
_CLOCK_SWEEP: int = 40  # seeded clock cases on top of the explicit ones
_CLOCK_SWEEP_END: int = 4102444800  # 2100-01-01: the sweep covers the 21st century the RTC lives in
_UNITS_PER_DEGREE: float = 1e7  # a Rust Position counts 1e-7 degrees
_FIX_WINDOW_MS: int = 60000  # the GNSS stand-in's freshness window: roomy, a fix pushed for a step is fresh in it
_F32_RELATIVE: float = 3e-07  # the Rust f32 distance's error, relative (worst measured over 5 km: 2.6e-7)...
_F32_LATITUDE_LIMIT: float = 50.0  # ...within this many degrees of the equator; it grows with tan(latitude)
_F32_FLOOR_M: float = 1e-04  # ...plus this absolute floor, for the short ones
_HALF_UNIT_M: float = 0.0056  # how far half a unit moves a point along one axis, at most (half of 1.1132 cm)
_TOLERANCE: float = 5.01e-08  # the fixture's abs tolerance: half a unit plus float slack (see build())
_SITES_NOTE: str = ('plan B drops sites from persisted() and ignores them in set-config; the Rust port keeps them '
                    'and replaces them with a set-config list')  # the sites divergence, as a step states it
_FLOATS_NOTE: str = ('plan B echoes the integer it was given; the Rust port holds metres as f32 and degrees as 1e-7 '
                     'units, and writes every one as a JSON float')  # the number-kind divergence, as a step states it

"""The session rows plan B asked for: (utc_offset, source, cc_position) per time set."""
_index_calls: list = []


def _record_index(seconds: int, utc_offset: int | None, source: object, cc_position: tuple | None) -> bool:
    """recorder.Recorder.index_session on the host: record the row's inputs."""
    _index_calls.append((utc_offset, source, cc_position))
    return True


def _install() -> object:
    """
    Put the stand-ins in place.

    Args:
        (none)

    Returns:
        The databoard channel that plays the GNSS 'position' provider.
    """
    recorder.Recorder.index_session = _record_index
    return databoard.Databoard.provide('gnss', {'position': {'priority': 0, 'timeout_ms': _FIX_WINDOW_MS}},
                                       'position')


def _degrees(units: list) -> tuple:
    """Plan B's float (latitude, longitude) for a position in 1e-7 degree units."""
    return units[0] / _UNITS_PER_DEGREE, units[1] / _UNITS_PER_DEGREE


def _set_fix(channel: object, fix: list | None) -> None:
    """
    Make the GNSS fix this step runs under.

    Args:
        channel - the 'position' channel.
        fix - [latitude, longitude] in units, or None for no fresh fix.

    Returns:
        None; pushes the fix, or dates the channel's last push back past its window so it reads stale.
    """
    if fix is not None:
        channel.push(_degrees(fix))
        return
    channel.t1 = time.ticks_add(time.ticks_us(), -2 * _FIX_WINDOW_MS * 1000)


def _plain(value: object) -> object:
    """A value as JSON gives it back: tuples become lists."""
    return json.loads(json.dumps(value))


def _holds_integer(value: object) -> bool:
    """
    Whether a JSON answer holds an integer number anywhere (a bool is no number).

    Every number in a step's persisted, inspect and sites answers is a metre or a degree, which the Rust port
    writes as a JSON float; an integer there is plan B echoing the kind it was given.

    Args:
        value - the answer, as JSON gives it back (dicts, lists, scalars).

    Returns:
        True when an int other than a bool is in it, at any depth.
    """
    if isinstance(value, dict):
        return any(_holds_integer(item) for item in value.values())
    if isinstance(value, list):
        return any(_holds_integer(item) for item in value)
    return isinstance(value, int) and not isinstance(value, bool)


def _sites_json(sites: list) -> list:
    """Plan B's validated sites, [(name, (lat, lon), zone)], in the launch.config shape the Rust port writes."""
    return [{'name': name, 'pad': list(pad), 'zone': [list(zone[0]), list(zone[1])]} for name, pad, zone in sites]


def _check_boundaries(launch: object, name: str) -> None:
    """
    Refuse a step whose geometry lies near a rounding boundary.

    The distances are reported rounded to 0.1 m, and the Rust answer differs from plan B's before rounding: by
    its f32 error (3e-7 of the distance, at least 0.1 mm; measured only within 50 degrees of the equator, since
    it grows with tan(latitude)), and by how far the zone's centre moves when a corner sum is odd: the Rust port
    rounds that half unit toward zero, up to 5.6 mm along each axis whose sum is odd, so up to 7.9 mm when both
    are. A distance closer than that to a boundary could round the other way there. (The HPRC default's
    pad-to-target distance is 49.450247 m: 0.25 mm from one, even sums, and safe.)

    Args:
        launch - the Mission.
        name - the scenario and step, for the message.

    Returns:
        None.

    Raises:
        ValueError - a distance lies too close to a rounding boundary (move the case), or the geometry lies past
            the latitude the f32 error was measured to (measure it there first).
    """
    origin = launch.launch_point()
    if launch.zone is None or origin is None:
        return
    if max(abs(latitude) for latitude in (origin[0], launch.zone[0][0], launch.zone[1][0])) > _F32_LATITUDE_LIMIT:
        raise ValueError('%s: past %.0f degrees of latitude the f32 distance error is not measured'
                         % (name, _F32_LATITUDE_LIMIT))
    corners = [round(coordinate * _UNITS_PER_DEGREE) for corner in launch.zone for coordinate in corner]
    latitude_odd = (corners[0] + corners[2]) % 2
    longitude_odd = (corners[1] + corners[3]) % 2
    centre_shift = _HALF_UNIT_M * math.hypot(latitude_odd, longitude_odd)
    for point in navigation.zone(launch.zone[0], launch.zone[1]):
        distance = navigation.distance(origin[0], origin[1], point[0], point[1])
        margin = _F32_RELATIVE * distance + _F32_FLOOR_M + centre_shift
        if abs(distance - (int(distance * 10.0) + 0.5) / 10.0) < margin:
            raise ValueError('%s: distance %.6f m is within %.4f m of a rounding boundary' % (name, distance, margin))


def _state(launch: object, name: str) -> dict:
    """
    What the Rust port is compared on after a step.

    Args:
        launch - the Mission.
        name - the scenario and step, for a boundary refusal.

    Returns:
        {persisted, sites, probe, inspect}: inspect without clock and epoch.
    """
    _check_boundaries(launch, name)
    snapshot = _plain(launch.inspect())
    del snapshot['clock'], snapshot['epoch']
    return {'persisted': _plain(launch.persisted()), 'sites': _sites_json(launch.sites),
            'probe': asyncio.run(launch.probe()), 'inspect': snapshot}


"""
Sites for the divergence scenario: test_mission's list (two valid, one with a bad latitude, one without a zone)
plus an unnamed valid one, which plan B names site<n> by its place among the valid ones.
"""
_SITES: list = [
    {'name': 'north', 'pad': [48.0010, 11.0], 'zone': [[48.0020, 10.999], [48.0015, 11.001]]},
    {'name': 'south', 'pad': [48.0000, 11.0], 'zone': [[47.9990, 10.999], [47.9985, 11.001]]},
    {'pad': [200.0, 11.0], 'zone': [[1, 1], [2, 2]]},
    {'name': 'no-zone', 'pad': [48.0, 11.0]},
    {'pad': [48.0005, 11.0005], 'zone': [[48.0008, 11.0], [48.0006, 11.001]]},
]
# a valid site of its own, for set-config to replace the list with
_EAST_SITE: dict = {'name': 'east', 'pad': [48.0, 11.002], 'zone': [[48.001, 11.003], [48.0005, 11.005]]}
# a landing zone centred ~95 m north-west of the HPRC pad, in range: an operator moving the zone at HPRC
_HPRC_NEAR_ZONE: list = [[25.5153, -80.3933], [25.5149, -80.3913]]
_TEST_ZONE: list = [[48.0005, 11.0], [48.0, 11.001]]  # test_mission's ~74 m wide zone

"""
The scenarios: (name, stored launch.config text or None, max_range_m, steps). A step is (action, properties or
None, fix in units or None).
"""
_SCENARIOS: tuple = (
    ('hprc_default_updates', None, 200.0, (
        ('observe', None, None),
        ('update', {'launch_id': 'flight.7'}, None),
        ('update', {'launch_id': 'flight.7'}, None),
        ('update', {'latitude': 25.5141, 'longitude': -80.3921, 'site': 'HPRC west pad'}, None),
        ('update', {'latitude': 999}, None),
        ('update', {'latitude': -90.5}, None),
        ('update', {'longitude': 180.25}, None),
        ('update', {'latitude': True}, None),
        ('update', {'longitude': '-80.39'}, None),
        ('update', {'latitude': None, 'longitude': None}, None),
        ('update', {'altitude': 5}, None),
        ('update', {'altitude': 5.0}, None),
        ('update', {'altitude': None}, None),
        ('update', {'altitude': 2.5}, None),
        ('update', {'zone': _HPRC_NEAR_ZONE}, None),
        ('update', {'zone': [[48.0, 11.0]]}, None),
        ('update', {'zone': [[200.0, 11.0], [48.0, 11.0]]}, None),
        ('update', {'zone': [[25.5, False], [25.4, -80.3]]}, None),
        ('update', {'zone': 'north'}, None),
        ('update', {'zone': None}, None),
        ('update', {'zone': _HPRC_NEAR_ZONE}, None),
        ('update', {'unknown': 1, 'clock': 'x', 'target': [0, 0]}, None),
        ('update', {'sites': [_EAST_SITE]}, None),
        ('update', {'epoch': 1781000000, 'utc_offset': -240, 'cc_position': [25.5, -80.25], 'source': 'cc-auto'},
         None),
        ('update', {'epoch': 'now', 'source': 'cc-auto'}, None),
        ('update', {'epoch': 1781000060, 'launch_id': 'L2', 'site': 'pad-b', 'latitude': 25.5144,
                    'longitude': -80.3916, 'altitude': 3.25, 'zone': [[25.5152, -80.3931], [25.5147, -80.3910]]},
         None),
    )),
    ('zone_range', None, 200.0, (
        ('update', {'latitude': 48.00025, 'longitude': 11.0005, 'zone': _TEST_ZONE}, None),
        ('update', {'zone': [[48.0005, 11.0], [48.0, 11.0011]]}, None),
        ('update', {'latitude': 48.005}, None),
        ('update', {'latitude': 48.0012}, None),
    )),
    ('tight_range', None, 10.0, (
        ('update', {'latitude': 48.00025, 'longitude': 11.0005, 'zone': _TEST_ZONE}, None),
        ('update', {'zone': [[48.00027, 11.0004], [48.00023, 11.0006]]}, None),
    )),
    ('gnss_launch_point', '{}', 200.0, (
        ('observe', None, None),
        ('observe', None, [480000000, 110000000]),
        ('update', {'zone': _TEST_ZONE}, [480003100, 110005200]),
        ('update', {'latitude': 48.0002}, [480003100, 110005200]),
        ('observe', None, None),
        ('update', {'longitude': 11.0004}, None),
        ('observe', None, [490000000, 120000000]),
    )),
    ('loaded_fields', json.dumps({'launch_id': 'hprc-t1', 'site': 'pad-a', 'latitude': 45.5, 'longitude': -73.5,
                                  'altitude': 120}), 200.0, (
        ('observe', None, None),
        ('update', {'altitude': 120.0, 'launch_id': 'hprc-t1'}, None),
        ('update', {'zone': [[45.5006, -73.5007], [45.5003, -73.4992]]}, None),
    )),
    ('loaded_out_of_range', json.dumps({'latitude': 200, 'longitude': -73.5,
                                        'zone': [[45.5, -73.5], [45.4, -73.4]]}), 200.0, (
        ('observe', None, None),
        ('observe', None, [454500000, -734500000]),
    )),
    ('loaded_wrong_types', json.dumps({'latitude': True, 'longitude': '11.0', 'zone': [[48.0, 11.0], [48.1]],
                                       'sites': 'none'}), 200.0, (
        ('observe', None, None),
    )),
    ('corrupt_text', '{ not json', 200.0, (('observe', None, None),)),
    ('not_an_object', '[1, 2]', 200.0, (('observe', None, None),)),
    ('json_null', 'null', 200.0, (('observe', None, None),)),
    ('empty_text', '', 200.0, (('observe', None, None),)),
    ('southern_and_western', json.dumps({'site': 'south-west', 'latitude': -34.6037, 'longitude': -58.3816,
                                         'altitude': 25.5, 'zone': [[-34.6030, -58.3830], [-34.6040, -58.3810]]}),
     200.0, (
        ('observe', None, None),
        ('update', {'site': 'south-east', 'latitude': -33.8688, 'longitude': 151.2093,
                    'zone': [[-33.8680, 151.2085], [-33.8695, 151.2090]]}, None),
    )),
    ('sites_survive_set_config', json.dumps({'site': 'field', 'latitude': 48.0, 'longitude': 11.0,
                                             'zone': [[48.0006, 10.9995], [48.0003, 11.0012]],
                                             'sites': _SITES}), 200.0, (
        ('observe', None, None),
        ('set_config', {'launch_id': 'f1'}, None),
        ('reboot', None, None),
        ('set_config', {'sites': [_EAST_SITE, {'name': 'bad', 'pad': [48.0]}]}, None),
        ('reboot', None, None),
        ('set_config', {'sites': 'none'}, None),
        ('set_config', {'sites': []}, None),
    )),
    ('set_config_round_trip', None, 200.0, (
        ('set_config', {'launch_id': 'save-me', 'latitude': 25.51461, 'longitude': -80.3921, 'altitude': 5,
                        'zone': [[25.5153, -80.3926], [25.5150, -80.3911]]}, None),
        ('reboot', None, None),
        ('set_config', {'launch_id': 'with-clock', 'epoch': 1781000000}, None),
        ('reboot', None, None),
    )),
    ('numbers_sent_as_integers', None, 200.0, (
        ('update', {'latitude': 48, 'longitude': 11, 'altitude': 500, 'zone': [[48, 11], [47.9995, 11.001]]}, None),
        ('set_config', {'sites': [{'name': 'whole-degree pad', 'pad': [48, 11],
                                   'zone': [[48, 11], [47.9995, 11.001]]}]}, None),
        ('reboot', None, None),
    )),
)


def _scenario(channel: object, name: str, stored: str | None, max_range_m: float, steps: tuple) -> dict:
    """
    Run one scenario on plan B.

    The Rust view of `sites` is tracked beside plan B's: it starts as plan B's (the same load rule), set-config
    replaces it with plan B's own validation of a `sites` list, and a reboot keeps it. A step whose Rust answer
    differs gets `divergence`: the Rust `sites` and `changed` where they differ, and `floats` where plan B
    answers an integer metre or degree (see the module notes).

    Args:
        channel - the GNSS 'position' channel.
        name - the scenario name.
        stored - the launch.config text it starts from, or None for no file.
        max_range_m - the board config's range limit.
        steps - (action, properties, fix) per step.

    Returns:
        {name, init: {stored, max_range_m}, steps: [{in, out[, divergence]}]}.
    """
    steps_out = []
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, mission.LAUNCH_PATH)
        if stored is not None:
            with open(path, 'w') as handle:
                handle.write(stored)
        launch = mission.Mission(path, max_range_m)
        rust_sites = _sites_json(launch.sites)
        for index, (action, properties, fix) in enumerate(steps):
            _set_fix(channel, fix)
            changed = None
            rust_changed = None
            if action in ('update', 'set_config'):
                changed = launch.update(properties)
                rust_changed = changed
            if action == 'set_config':
                launch.save()
                if isinstance(properties.get('sites'), list):
                    replacement = _sites_json(mission._sites(properties['sites']))
                    if replacement != rust_sites:
                        rust_sites = replacement
                        rust_changed = [key for key in changed if key != 'epoch'] + ['sites'] + (
                            ['epoch'] if 'epoch' in changed else [])
            if action == 'reboot':
                launch = mission.Mission(path, max_range_m)
            out = _state(launch, '%s step %d' % (name, index))
            if changed is not None:
                out['changed'] = changed
            step = {'in': {'action': action, 'properties': properties, 'fix': fix}, 'out': out}
            divergence = {}
            notes = []
            if rust_sites != out['sites']:
                divergence['sites'] = rust_sites
            if rust_changed != changed:
                divergence['changed'] = rust_changed
            if divergence:
                notes.append(_SITES_NOTE)
            if any(_holds_integer(answer) for answer in (out['persisted'], out['inspect'], out['sites'], rust_sites)):
                divergence['floats'] = True
                notes.append(_FLOATS_NOTE)
            if divergence:
                divergence['note'] = '; '.join(notes)
                step['divergence'] = divergence
            steps_out.append(step)
    return {'name': name, 'init': {'stored': stored, 'max_range_m': max_range_m}, 'steps': steps_out}


"""The time-set cases: (name, update properties), each on a fresh HPRC mission. test_mission's included."""
_TIME_SETS: tuple = (
    ('full', {'epoch': 1781000000, 'utc_offset': -240, 'cc_position': [25.5, -80.25], 'source': 'cc-auto'}),
    ('epoch_and_source', {'epoch': 1781000060, 'source': 'dashboard'}),
    ('bad_extras_text', {'epoch': 1781000120, 'utc_offset': '-240', 'cc_position': [200.0, 0.0], 'source': 'a;b'}),
    ('bad_extras_types', {'epoch': 1781000180, 'utc_offset': True, 'cc_position': [25.5], 'source': 7}),
    ('offset_west_edge', {'epoch': 1781000200, 'utc_offset': -720}),
    ('offset_east_edge', {'epoch': 1781000200, 'utc_offset': 840}),
    ('offset_past_west', {'epoch': 1781000200, 'utc_offset': -721}),
    ('offset_past_east', {'epoch': 1781000200, 'utc_offset': 841}),
    ('offset_huge', {'epoch': 1781000200, 'utc_offset': 100000}),
    ('offset_float', {'epoch': 1781000200, 'utc_offset': 60.0}),
    ('cc_position_southern_eastern', {'epoch': 1781000200, 'cc_position': [-33.8688, 151.2093]}),
    ('cc_position_bool', {'epoch': 1781000200, 'cc_position': [25.5, True]}),
    ('cc_position_object', {'epoch': 1781000200, 'cc_position': {'latitude': 25.5, 'longitude': -80.25}}),
    ('epoch_at_2000', {'epoch': 946684800, 'utc_offset': -240, 'source': 'dashboard'}),
    ('epoch_before_2000', {'epoch': 946684799, 'source': 'dashboard'}),
    ('epoch_negative', {'epoch': -1}),
    ('epoch_text', {'epoch': 'now', 'source': 'cc-auto'}),
    ('epoch_bool', {'epoch': True}),
    ('epoch_float', {'epoch': 1781000000.0}),
    ('epoch_null', {'epoch': None}),
    ('extras_without_epoch', {'utc_offset': -240, 'cc_position': [25.5, -80.25], 'source': 'cc-auto'}),
    ('epoch_with_a_field', {'epoch': 1781000000, 'launch_id': 'synced'}),
)


def _time_set(name: str, properties: dict) -> dict:
    """
    One time-set case on a fresh HPRC mission.

    Args:
        name - the case name.
        properties - the update mission properties.

    Returns:
        {name, input, output: {changed, epoch, time_set}}: the epoch set_time accepted (None when refused), and
        the session row's inputs (None when no row was asked for). A non-text source is null: the empty cell plan
        B's Recorder makes of it (recorder._cell).
    """
    with tempfile.TemporaryDirectory() as directory:
        launch = mission.Mission(os.path.join(directory, mission.LAUNCH_PATH))
        _index_calls.clear()
        changed = launch.update(properties)
    time_set = None
    if _index_calls:
        utc_offset, source, cc_position = _index_calls[0]
        time_set = {'utc_offset': utc_offset, 'source': source if isinstance(source, str) else None,
                    'cc_position': list(cc_position) if cc_position is not None else None}
    return {'name': name, 'input': properties,
            'output': {'changed': changed, 'epoch': properties['epoch'] if 'epoch' in changed else None,
                       'time_set': time_set}}


def _clock(name: str, epoch: int) -> dict:
    """
    One clock case: set_time(epoch), then clock() -- both on the shims' virtual RTC.

    Args:
        name - the case name.
        epoch - Unix seconds, from 2000 on (set_time refuses earlier).

    Returns:
        {name, input: epoch, output: the clock text}.

    Raises:
        ValueError - set_time refused the epoch.
    """
    with tempfile.TemporaryDirectory() as directory:
        launch = mission.Mission(os.path.join(directory, mission.LAUNCH_PATH))
        if not launch.set_time(epoch):
            raise ValueError('%s: set_time refused %d' % (name, epoch))
        return {'name': name, 'input': epoch, 'output': launch.clock()}


_CLOCKS: tuple = (
    ('epoch_2000', 946684800),
    ('leap_day_2000', 951782400),
    ('after_the_leap_day_2000', 951868800),
    ('last_second_of_2000', 978307199),
    ('first_second_of_2001', 978307200),
    ('leap_day_2024', 1709164800),
    ('moment_in_2026', 1781000000),
    ('last_second_of_2099', 4102444799),
    ('day_before_march_2100', 4107542399),
    ('march_2100_not_leap', 4107542400),
)


def build() -> dict:
    """
    The mission fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, tolerance, scenarios, time_sets, clocks}.
    """
    channel = _install()
    generator = random.Random(_SEED)
    sweep = sorted(generator.randrange(mission._EPOCH_OFFSET, _CLOCK_SWEEP_END) for _ in range(_CLOCK_SWEEP))
    clocks = list(_CLOCKS) + [('sweep_%02d' % index, epoch) for index, epoch in enumerate(sweep)]
    return {
        'meta': {'source': 'src/glider/mission.py, src/glider/navigation.py'},
        'compare': {'kind': 'abs', 'tol': _TOLERANCE},
        'tolerance': ('5.01e-8: degrees to half the Rust 1e-7 degree unit plus float slack (a zone centre is the '
                      'integer mean, at most half a unit off; a one-unit slip fails); metres exactly, since every '
                      'distance is reported rounded to 0.1 m and no case lies closer to a rounding boundary than '
                      'the Rust f32 answer can differ; an altitude goes out as its shortest f32 text'),
        'scenarios': [_scenario(channel, *scenario) for scenario in _SCENARIOS],
        'time_sets': [_time_set(*case) for case in _TIME_SETS],
        'clocks': [_clock(*case) for case in clocks],
    }
