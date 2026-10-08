"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture readiness.json: plan B's flight-readiness gate, cc_client._readiness() (src/glider/cc_client.py:47-107),
over the board-config corpus -- the five launches/20261003 board files, config_default, config_hitl and the
flight-ready configs tools/launch_config.py makes -- and over copies of them with each gate tripped once.

Members:
  * bases -- the generated documents the cases name (config_corpus.BASES);
  * cases -- {name, input: {config, zone, sites}, output: _readiness()'s dict}. `config` names a document as
    config_corpus.py describes. `zone` and `sites` say whether the mission holds a landing zone and CC-less sites:
    plan B's own Mission is built over a launch.config that carries HPRC's zone and a site with HPRC's pad, or not,
    so _readiness reads the real object.

Two of _readiness's inputs stay out of the fixture:
  * the `fins` gate reads the flight task's mixer (the control surfaces with no driver), and no builder runs a
    flight task: nothing named 'flight' is registered, so that gate never trips here. The port has no mixer to ask
    yet either.
  * plan B's _readiness reads a missing `enabled` as disabled (`task.get('enabled')`), where its Controller reads it
    as enabled; the port takes the Controller's meaning everywhere (doc/specs/avionics-port.md, "`_readiness` and
    `enabled`"). So the builder refuses a case whose `watchdog` or `flight` component lacks the key: its answer
    would be plan B's inconsistency, not the port's contract. The Rust-only tests pin the missing key.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import json
import os
import tempfile

import cc_client
import config_corpus
import mission

"""The generated documents this fixture's cases may name as a base (config_corpus.BASES)."""
_BASE_NAMES: list = ['default', 'hitl', 'hitl_e16_noisy', 'flight_ready_tms7f', 'flight_ready_new']

"""The components _readiness looks up by name, whose `enabled` plan B and the port read differently when absent."""
_LOOKED_UP: tuple = ('watchdog', 'flight')

"""HPRC's landing zone, as plan B's default mission holds it (mission._HPRC_DEFAULT)."""
_ZONE: list = mission._HPRC_DEFAULT['zone']

"""One CC-less site: HPRC's pad and zone (doc/specs/coludo.md "Field operation without CC")."""
_SITE: dict = {'name': 'hprc', 'pad': [mission._HPRC_DEFAULT['latitude'], mission._HPRC_DEFAULT['longitude']],
               'zone': _ZONE}

"""The flight loop's PID axes, as _readiness walks them."""
_AXES: tuple = ('roll', 'pitch', 'yaw')


def _set(path: list, value: object) -> dict:
    """An edit setting the member at `path`."""
    return {'path': path, 'value': value}


def _delete(path: list) -> dict:
    """An edit deleting the member at `path`."""
    return {'path': path, 'delete': True}


def _case(name: str, config: dict, zone: bool = True, sites: bool = False) -> tuple:
    """
    One case before plan B answers it.

    Args:
        name - the case name.
        config - the document, named as config_corpus.py describes.
        zone - whether the mission holds a landing zone (the corpus default: a zone is set).
        sites - whether the mission holds a CC-less site.

    Returns:
        (name, input).
    """
    return name, {'config': config, 'zone': zone, 'sites': sites}


def _component(document: dict, name: str) -> list:
    """The edit path of the component `name` in `document`."""
    return ['components', config_corpus.index_of(document, 'components', name)]


def _corpus() -> list:
    """Every corpus document as it is, with a landing zone set: (name, input)."""
    cases = [_case(path.rsplit('/', 1)[1][:-len('.config')], {'file': path}) for path in config_corpus.LAUNCH_FILES]
    return cases + [_case(name, {'base': name}) for name in _BASE_NAMES]


def _watchdog(default: dict) -> list:
    """The watchdog gate: off, absent, enabled by Python's truthiness, and looked up by name in `components` only."""
    watchdog = _component(default, 'watchdog')
    entry = default['components'][watchdog[1]]
    enabled = watchdog + ['enabled']
    return [
        _case('watchdog_enabled', {'base': 'default', 'edits': [_set(enabled, True)]}),
        _case('watchdog_disabled', {'base': 'default', 'edits': [_set(enabled, False)]}),
        _case('watchdog_absent', {'base': 'default', 'edits': [_delete(watchdog)]}),
        _case('watchdog_enabled_zero', {'base': 'default', 'edits': [_set(enabled, 0)]}),
        _case('watchdog_enabled_null', {'base': 'default', 'edits': [_set(enabled, None)]}),
        _case('watchdog_enabled_text', {'base': 'default', 'edits': [_set(enabled, 'yes')]}),
        _case('watchdog_enabled_empty_text', {'base': 'default', 'edits': [_set(enabled, '')]}),
        _case('watchdog_renamed', {'base': 'default', 'edits': [
            _set(enabled, True), _set(watchdog + ['name'], 'dog')]}),
        _case('watchdog_as_a_sensor', {'base': 'default', 'edits': [
            _delete(watchdog), _set(['sensors', len(default['sensors'])], dict(entry, enabled=True))]}),
    ]


def _flight(ready: dict) -> list:
    """The flight gate: off, absent, renamed, and its gains zero or unset per axis, on the flight-ready new airframe."""
    flight = _component(ready, 'flight')
    gains = flight + ['gains']
    terms = ready['components'][flight[1]]['gains']
    zeroed = {axis: {term: 0 for term in terms[axis]} for axis in _AXES}

    def ready_with(*edits: dict) -> dict:
        """The flight-ready new airframe with these edits."""
        return {'base': 'flight_ready_new', 'edits': list(edits)}

    cases = [
        _case('flight_disabled', ready_with(_set(flight + ['enabled'], False))),
        _case('flight_disabled_with_zero_gains', ready_with(_set(flight + ['enabled'], False), _set(gains, zeroed))),
        _case('flight_absent', ready_with(_delete(flight))),
        _case('flight_renamed', ready_with(_set(flight + ['name'], 'autopilot'))),
        _case('flight_enabled_one', ready_with(_set(flight + ['enabled'], 1))),
        _case('gains_absent', ready_with(_delete(gains))),
        _case('gains_empty', ready_with(_set(gains, {}))),
        _case('gains_all_zero', ready_with(_set(gains, zeroed))),
        _case('gains_all_float_zero', ready_with(_set(gains, {axis: {'kp': 0.0, 'kd': 0.0} for axis in _AXES}))),
        _case('gains_one_term_acts', ready_with(_set(gains, {axis: {'kp': 0, 'ki': 0, 'kd': 0.1} for axis in _AXES}))),
        _case('gains_negative_acts', ready_with(_set(gains, {axis: {'kp': -1.5} for axis in _AXES}))),
        _case('gains_an_extra_axis_is_not_read', ready_with(_set(gains + ['throttle'], {'kp': 0}))),
        _case('gains_terms_that_are_not_numbers', ready_with(_set(gains, {
            'roll': {'kp': True, 'kd': ''}, 'pitch': {'kp': '', 'kd': None, 'ki': []},
            'yaw': {'kp': 'x', 'ki': [1]}}))),
    ]
    for axis in _AXES:
        cases += [
            _case('gains_%s_zero' % axis, ready_with(_set(gains + [axis], zeroed[axis]))),
            _case('gains_%s_empty' % axis, ready_with(_set(gains + [axis], {}))),
            _case('gains_%s_absent' % axis, ready_with(_delete(gains + [axis]))),
        ]
    others = [axis for axis in _AXES if axis != 'pitch']
    cases.append(_case('gains_two_axes_zero', ready_with(*[_set(gains + [axis], {}) for axis in others])))
    return cases


def _multiplier() -> list:
    """The bench-derating gate: `%g` over the numbers it prints (a bool is Python's int), and the ones equal to 1.0."""
    path = ['fins', 'limit_multiplier']
    values = [
        ('half', 0.5), ('tiny', 1e-05), ('large_int', 123456789), ('one_int', 1), ('one_float', 1.0), ('two_int', 2),
        ('negative', -0.5), ('zero_int', 0), ('zero_float', 0.0), ('negative_zero', -0.0), ('ten_thousandth', 0.0001),
        ('six_digits', 100000), ('seven_digits', 1000000), ('fraction_past_six_digits', 0.00001234),
        ('rounds_up_to_a_million', 999999.5), ('rounds_to_one', 1.0000001), ('rounds_up_to_one', 0.99999999),
        ('tie_to_even_down', 1234565), ('tie_to_even_up', 1234575), ('huge', 1e300),
        ('beyond_i64', 12345678901234567890), ('six_significant', 1.23456789), ('fixed_fraction', 123.456789),
        ('false', False), ('true', True),
    ]
    cases = [_case('multiplier_%s' % name, {'base': 'flight_ready_new', 'edits': [_set(path, value)]})
             for name, value in values]
    return cases + [
        _case('multiplier_absent', {'base': 'flight_ready_new', 'edits': [_delete(path)]}),
        _case('fins_absent', {'base': 'flight_ready_new', 'edits': [_delete(['fins'])]}),
    ]


def _zone() -> list:
    """The landing-zone gate: a zone, CC-less sites, both, and neither."""
    ready = {'base': 'flight_ready_new'}
    return [
        _case('zone_set_no_sites', ready, zone=True, sites=False),
        _case('zone_set_with_sites', ready, zone=True, sites=True),
        _case('no_zone_with_sites', ready, zone=False, sites=True),
        _case('no_zone_no_sites', ready, zone=False, sites=False),
    ]


def _all_at_once(default: dict) -> list:
    """Every gate tripped in one document, so the whole verdict is pinned: (name, input)."""
    watchdog = _component(default, 'watchdog')
    flight = _component(default, 'flight')
    return [
        _case('every_gate_at_once', {'base': 'default', 'edits': [
            _set(watchdog + ['enabled'], False), _set(flight + ['enabled'], True),
            _set(flight + ['gains'], {'roll': {'kp': 0}, 'pitch': {}, 'yaw': {'kp': 1.5}}),
            _set(['fins', 'limit_multiplier'], 0.25)]},
            zone=False, sites=False),
    ]


def _refuse_missing_enabled(name: str, document: dict) -> None:
    """
    Refuse a case whose looked-up component carries no `enabled` (see the module notes).

    Args:
        name - the case name, for the message.
        document - the case's document as plan B sees it.

    Returns:
        None when every component named in _LOOKED_UP carries `enabled`.

    Raises:
        ValueError - one does not.
    """
    for component in document.get('components', []):
        if component.get('name') in _LOOKED_UP and 'enabled' not in component:
            raise ValueError('case %s: component %s has no enabled, which plan B and the port read differently'
                             % (name, component.get('name')))


def _readiness(name: str, specification: dict, known: dict) -> dict:
    """
    Plan B's answer to one case.

    Args:
        name - the case name.
        specification - the case input {config, zone, sites}.
        known - the fixture's bases.

    Returns:
        _readiness()'s dict for the document, with plan B's Mission holding the zone and the sites asked for.

    Raises:
        ValueError - the case's document drops `enabled` from a looked-up component, or the Mission did not read
            the zone or the sites as asked.
    """
    document = config_corpus.resolve(specification['config'], known)
    _refuse_missing_enabled(name, document)
    stored = {}
    if specification['zone']:
        stored['zone'] = _ZONE
    if specification['sites']:
        stored['sites'] = [_SITE]
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, mission.LAUNCH_PATH)
        with open(path, 'w') as handle:
            json.dump(stored, handle)
        launch = mission.Mission(path)  # registers itself: _readiness reads it through the Inspector
    if (launch.zone is not None, bool(launch.sites)) != (specification['zone'], specification['sites']):
        raise ValueError('case %s: the mission read zone %r and sites %r' % (name, launch.zone, launch.sites))
    return cc_client._readiness(document)


def build() -> dict:
    """
    The readiness fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, bases, cases}.

    Raises:
        ValueError - a case name is used twice, or a case is refused (see _readiness).
    """
    known = config_corpus.bases(_BASE_NAMES)
    named = (_corpus() + _watchdog(known['default']) + _flight(known['flight_ready_new']) + _multiplier() + _zone()
             + _all_at_once(known['default']))
    names = [name for name, _ in named]
    if len(set(names)) != len(names):
        raise ValueError('a case name is used twice')
    cases = [{'name': name, 'input': specification, 'output': _readiness(name, specification, known)}
             for name, specification in named]
    return {
        'meta': {'source': 'src/glider/cc_client.py (_readiness), src/glider/mission.py; ' + config_corpus.SOURCES},
        'compare': {'kind': 'exact'},
        'bases': known,
        'cases': cases,
    }
