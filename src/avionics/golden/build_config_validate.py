"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture config_validate.json: plan B's config.validate() -- the verdict and every message, in order --
over the board-config corpus, the negative cases of src/glider/test/test_config.py, and copies of the default
(and of a small document) with each rule broken once.

Members:
  * bases -- the generated documents the cases name, plus `small` (a compact valid document of this builder's,
    where a case wants few devices around the rule it breaks);
  * cases -- {name, input: a document named as config_corpus.py describes, output: validate()'s list}. A name
    test_config_<n> marks an input lifted from test_config.py.

Every input is shown to plan B with its keys sorted (config_corpus.resolve): plan B walks a dict in its
insertion order and the Rust port in sorted order, so the message order and the first claimant of a GPIO are
only comparable on a sorted document. A sorted document is what every writer here produces.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import config as config_module
import config_corpus

"""The generated documents this fixture's cases may name as a base (config_corpus.BASES)."""
_BASE_NAMES: list = ['default', 'hitl', 'hitl_e16_noisy', 'flight_ready_tms7f', 'flight_ready_new']

"""A compact valid document: a board, one bus of each kind, two pins, one sensor and an enabled watchdog."""
_SMALL: dict = {
    'version': '20261005',
    'board': {'id': 'bench', 'mcu': 'esp32p4'},
    'wifi': {'ssid': 'lab'},
    'buses': {'i2c': {'0': {'sda': 7, 'scl': 8}}, 'spi': {'1': {'sck': 48, 'mosi': 47, 'miso': 46, 'mode': 3}},
              'uart': {'1': {'tx': 20}}},
    'pins': {'servo_yaw': 26, 'laser_int': None},
    'recorder': {'telemetry_ms': 0},
    'fins': {'limit_multiplier': 1.0},
    'sensors': [{'name': 'baro', 'driver': 'icp10111', 'bus': 'i2c', 'id': 0, 'addr': 99,
                 'provides': {'altitude': {'priority': 0, 'timeout_ms': 200}}}],
    'components': [{'name': 'watchdog', 'activity': 'watchdog', 'enabled': True, 'wdt_timeout_ms': 5000}],
}


def _set(path: list, value: object) -> dict:
    """An edit setting the member at `path`."""
    return {'path': path, 'value': value}


def _delete(path: list) -> dict:
    """An edit deleting the member at `path`."""
    return {'path': path, 'delete': True}


def _default(*edits: dict) -> dict:
    """The default document with these edits."""
    return {'base': 'default', 'edits': list(edits)}


def _small(*edits: dict) -> dict:
    """The small document with these edits."""
    return {'base': 'small', 'edits': list(edits)}


def _device(document: dict, section: str, name: str) -> list:
    """The path of the device `name` in `section` ('sensors' / 'components') of `document`, for an edit."""
    return [section, config_corpus.index_of(document, section, name)]


def _corpus() -> list:
    """The documents that must validate as they are (and the non-objects that are one message): (name, input)."""
    cases = [(path.rsplit('/', 1)[1][:-len('.config')], {'file': path}) for path in config_corpus.LAUNCH_FILES]
    cases += [(name, {'base': name}) for name in _BASE_NAMES + ['small']]
    cases += [
        ('taster_copy_of_tms7f', {'file': config_corpus.TMS7F_FILE, 'edits': [_set(['board', 'id'], 'taster')]}),
        ('empty', {'document': {}}),
        ('null', {'document': None}),
        ('list', {'document': []}),
        ('text', {'document': 'board'}),
        ('integer', {'document': 5}),
        ('float', {'document': 2.5}),
        ('true', {'document': True}),
    ]
    return cases


def _test_config(default: dict) -> list:
    """The inputs of test_config.py's checks, positive and negative, in its order: (name, input)."""
    watchdog, flight = _device(default, 'components', 'watchdog'), _device(default, 'components', 'flight')
    servo, baro = _device(default, 'components', 'servo_yaw'), _device(default, 'sensors', 'baro_icp10111')
    accel = _device(default, 'sensors', 'accel_adxl375')
    return [
        ('test_config_i2c_without_pins', _default(_set(['buses', 'i2c', '9'], {'freq': 400000, 'baud': 9600}))),
        ('test_config_spi_without_pins', _default(_set(['buses', 'spi', '9'], {'freq': 400000, 'baud': 9600}))),
        ('test_config_uart_without_pins', _default(_set(['buses', 'uart', '9'], {'freq': 400000, 'baud': 9600}))),
        ('test_config_uart_tx_only', _default(_set(['buses', 'uart', '9'], {'tx': 44, 'baud': 9600}))),
        ('test_config_bank_limit_text', _default(_set(flight + ['bank_limit'], '45'))),
        ('test_config_trim_text', _default(_set(servo + ['trim'], '2.5'))),
        ('test_config_gain_text', _default(_set(flight + ['gains', 'roll', 'kp'], '0.8'))),
        ('test_config_fins_multiplier_text', _default(_set(['fins', 'limit_multiplier'], '1.0'))),
        ('test_config_period_text', _default(_set(baro + ['period_ms'], '100'))),
        ('test_config_telemetry_text', _default(_set(accel + ['telemetry_ms'], '20'))),
        ('test_config_recorder_telemetry_text', _default(_set(['recorder', 'telemetry_ms'], '500'))),
        ('test_config_recorder_telemetry_negative', _default(_set(['recorder', 'telemetry_ms'], -1))),
        ('test_config_recorder_telemetry_true', _default(_set(['recorder', 'telemetry_ms'], True))),
        ('test_config_recorder_telemetry_zero', _default(_set(['recorder', 'telemetry_ms'], 0))),
        ('test_config_fins_multiplier_true', _default(_set(['fins', 'limit_multiplier'], True))),
        ('test_config_fins_multiplier_half', _default(_set(['fins', 'limit_multiplier'], 0.5))),
        ('test_config_pin_collision', _default(_set(['pins', 'servo_yaw'], 7))),
        ('test_config_reserved_pin', _default(_set(['pins', 'servo_yaw'], 18))),
        ('test_config_disabled_pins', _default(_set(['pins', 'laser_xshut'], None), _set(['pins', 'laser_int'], -1),
                                               _set(['pins', 'accel_int1'], -7))),
        ('test_config_disabled_pins_and_collision', _default(
            _set(['pins', 'laser_xshut'], None), _set(['pins', 'laser_int'], -1), _set(['pins', 'accel_int1'], -7),
            _set(['pins', 'servo_yaw'], 7))),
        ('test_config_undefined_bus', _default(_set(['sensors', 0, 'id'], 9))),
        ('test_config_text_bus_id', _default(_set(['sensors', 0, 'id'], 'x'))),
        ('test_config_bad_bus_type', _default(_set(['buses', 'oops'], {'0': {'tx': 99, 'rx': 98}}))),
        ('test_config_spi_mode_4', _default(_set(['buses', 'spi', '1', 'mode'], 4))),
        ('test_config_board_id_space', _default(_set(['board', 'id'], 'glider 1'))),
        ('test_config_no_implementation', _default(_delete(['components', 0, 'activity']))),
        ('test_config_save_refused', _default(_set(['pins', 'servo_yaw'], 8))),
        ('test_config_fins_list', _default(_set(['fins'], [1, 2]))),
        ('test_config_watchdog_flight_value', _default(_set(watchdog + ['enabled'], True),
                                                       _set(watchdog + ['wdt_timeout_ms'], 5000))),
        ('test_config_watchdog_below_floor', _default(_set(watchdog + ['enabled'], True),
                                                      _set(watchdog + ['wdt_timeout_ms'], 1000))),
        ('test_config_watchdog_disabled_below_floor', _default(_set(watchdog + ['enabled'], False),
                                                               _set(watchdog + ['wdt_timeout_ms'], 1000))),
    ]


def _board_and_wifi() -> list:
    """The board and wifi rules, each broken once: (name, input)."""
    entries = ['coludo', '', {'ssid': ''}, {'ssid': 5}, 5, None, {'ssid': 'x', 'mode': 'ap'},
               {'ssid': 'x', 'mode': 'sta'}, {'mode': 'sta'}, [], {'ssid': 'y', 'mode': None}]
    return [
        ('board_missing', _default(_delete(['board']))),
        ('board_list', _default(_set(['board'], []))),
        ('board_id_missing', _default(_delete(['board', 'id']))),
        ('board_id_empty', _default(_set(['board', 'id'], ''))),
        ('board_id_number', _default(_set(['board', 'id'], 5))),
        ('board_id_tab', _default(_set(['board', 'id'], 'a\tb'))),
        ('board_id_newline', _default(_set(['board', 'id'], 'a\nb'))),
        ('board_id_return', _default(_set(['board', 'id'], 'a\rb'))),
        ('board_id_other_whitespace_passes', _default(_set(['board', 'id'], 'a' + chr(0xA0) + 'b' + chr(0xB) + 'c'))),
        ('board_mcu_missing', _default(_delete(['board', 'mcu']))),
        ('board_mcu_missing_skips_reserved', _default(_delete(['board', 'mcu']), _set(['pins', 'servo_yaw'], 18))),
        ('board_mcu_number', _default(_set(['board', 'mcu'], 5))),
        ('board_mcu_unknown_skips_reserved', _default(_set(['board', 'mcu'], 'esp32s3'),
                                                      _set(['pins', 'servo_yaw'], 18))),
        ('board_mcu_c6_reserves_nothing', _default(_set(['board', 'mcu'], 'esp32c6'), _set(['pins', 'servo_yaw'], 18))),
        ('board_mcu_firebeetle', _default(_set(['board', 'mcu'], 'firebeetle2p4'))),
        ('wifi_null', _default(_set(['wifi'], None))),
        ('wifi_missing', _default(_delete(['wifi']))),
        ('wifi_list', _default(_set(['wifi'], []))),
        ('wifi_mode_ap', _default(_set(['wifi', 'mode'], 'ap'))),
        ('wifi_mode_sta', _default(_set(['wifi', 'mode'], 'sta'))),
        ('wifi_mode_null', _default(_set(['wifi', 'mode'], None))),
        ('wifi_ssid_number', _default(_set(['wifi', 'ssid'], 5))),
        ('wifi_ssid_null', _default(_set(['wifi', 'ssid'], None))),
        ('wifi_networks_text', _default(_set(['wifi', 'networks'], 'coludo'))),
        ('wifi_networks_null', _default(_set(['wifi', 'networks'], None))),
        ('wifi_networks_empty', _default(_set(['wifi', 'networks'], []))),
        ('wifi_network_entries', _default(_set(['wifi', 'networks'], entries))),
    ]


def _buses_and_pins() -> list:
    """The bus, pin and reserved-pin rules, each broken once: (name, input)."""
    sda = ['buses', 'i2c', '0', 'sda']
    mode = ['buses', 'spi', '1', 'mode']
    cases = [
        ('buses_missing', _small(_delete(['buses']))),
        ('buses_list', _small(_set(['buses'], []))),
        ('buses_group_list', _small(_set(['buses', 'i2c'], []))),
        ('bus_not_an_object', _small(_set(['buses', 'i2c', '0'], 5))),
        ('bus_kind_unknown_and_addressed', _default(_set(['buses', 'can'], {'0': {'tx': 98}}),
                                                    _set(['sensors', 0, 'bus'], 'can'),
                                                    _set(['sensors', 0, 'id'], 0))),
        ('bus_pin_collision', _default(_set(['buses', 'i2c', '1', 'sda'], 7))),
        ('bus_pin_reserved', _default(_set(['buses', 'uart', '1', 'tx'], 37))),
        ('bus_uart_without_tx', _default(_delete(['buses', 'uart', '2', 'tx']))),
        ('bus_spi_without_anything', _default(_set(['buses', 'spi', '1'], {}))),
        ('bus_mode_on_i2c_is_not_checked', _default(_set(['buses', 'i2c', '0', 'mode'], 9))),
        ('pins_missing', _small(_delete(['pins']))),
        ('pins_list', _small(_set(['pins'], []))),
        ('pins_reserved_in_gpio_order', _default(_set(['pins', 'servo_yaw'], 54),
                                                 _set(['pins', 'servo_eleron_left'], 6),
                                                 _set(['pins', 'laser_int'], 18))),
        ('pins_collision_on_a_reserved_gpio', _default(_set(['pins', 'servo_yaw'], 18),
                                                       _set(['pins', 'servo_eleron_left'], 18))),
        ('pins_zero_is_a_gpio', _small(_set(['pins', 'boot'], 0))),
    ]
    for label, value in (('text', '7'), ('float', 7.0), ('negative', -1), ('true', True), ('null', None),
                         ('list', [7]), ('object', {'gpio': 7}), ('huge', 2 ** 64 - 1)):
        cases.append(('bus_pin_%s' % label, _default(_set(sda, value))))
    for label, value in (('text', '26'), ('float', 26.0), ('true', True), ('false', False), ('list', [26]),
                         ('negative_float', -1.0)):
        cases.append(('pin_%s' % label, _default(_set(['pins', 'servo_yaw'], value))))
    for label, value in (('float_two', 2.0), ('true', True), ('false', False), ('negative_zero', -0.0),
                         ('null', None), ('text', '3'), ('three_and_a_half', 3.5), ('negative', -1), ('list', [3])):
        cases.append(('spi_mode_%s' % label, _default(_set(mode, value))))
    return cases


def _recorder_and_fins() -> list:
    """The recorder and top-level fins rules, each broken once: (name, input)."""
    return [
        ('recorder_null', _default(_set(['recorder'], None))),
        ('recorder_missing', _default(_delete(['recorder']))),
        ('recorder_list', _default(_set(['recorder'], []))),
        ('recorder_capacity_zero', _default(_set(['recorder', 'tlm_capacity'], 0))),
        ('recorder_sizes_all_wrong', _default(_set(['recorder', 'tlm_capacity'], True),
                                              _set(['recorder', 'log_capacity'], -1),
                                              _set(['recorder', 'cell_size'], '256'),
                                              _set(['recorder', 'stats_ms'], 1000.0),
                                              _set(['recorder', 'telemetry_ms'], 0.0))),
        ('fins_null', _default(_set(['fins'], None))),
        ('fins_missing', _default(_delete(['fins']))),
        ('fins_text', _default(_set(['fins'], 'x'))),
        ('fins_empty', _default(_set(['fins'], {}))),
        ('fins_numeric_fields', _default(_set(['fins', 'trim'], 'x'), _set(['fins', 'period_ms'], None),
                                         _set(['fins', 'fins'], {'limit_multiplier': 'x'}),
                                         _set(['fins', 'gains'], {'roll': 5, 'pitch': {'kp': '1', 'kd': 0.5}}))),
    ]


def _devices(default: dict) -> list:
    """The sensors/components and watchdog rules, each broken once: (name, input)."""
    gnss, laser = _device(default, 'sensors', 'gnss'), _device(default, 'sensors', 'laser_agl')
    laser_l1x = _device(default, 'sensors', 'laser_agl_l1x')
    flight, watchdog = _device(default, 'components', 'flight'), _device(default, 'components', 'watchdog')
    every_numeric = [('limit_multiplier', '1'), ('trim', True), ('still_g', None), ('stall_speed_1g', []),
                     ('stall_margin', {}), ('nav_bank_gain', '1.5'), ('land_bank_gain', False),
                     ('loiter_gain', 'x'), ('final_cross_gain', [1]), ('pitot_gain', {'a': 1}),
                     ('glide_ratio', '5.5'), ('bank_limit', '45'), ('land_bank_limit', None), ('period_ms', '20'),
                     ('telemetry_ms', True)]
    enabled_watchdog = (_set(watchdog + ['enabled'], True),)
    cases = [
        ('sensors_object', _default(_set(['sensors'], {}))),
        ('sensors_null', _default(_set(['sensors'], None))),
        ('components_text', _default(_set(['components'], 'x'))),
        ('sensor_not_an_object', _default(_set(['sensors', 0], 5))),
        ('device_name_missing', _default(_delete(gnss + ['name']), _set(gnss + ['id'], 9))),
        ('device_name_empty', _default(_set(gnss + ['name'], ''))),
        ('device_name_number', _default(_set(gnss + ['name'], 5))),
        ('device_name_duplicate_across_lists', _default(_set(watchdog + ['name'], 'gnss'))),
        ('device_name_duplicate_within_a_list', _default(_set(laser_l1x + ['name'], 'laser_agl'))),
        ('implementation_empty_driver', _default(_set(gnss + ['driver'], ''))),
        ('implementation_zero_driver', _default(_set(gnss + ['driver'], 0))),
        ('implementation_number_driver', _default(_set(gnss + ['driver'], 5))),
        ('implementation_empty_driver_then_activity', _default(_set(gnss + ['driver'], ''),
                                                               _set(gnss + ['activity'], 'gnss'))),
        ('implementation_null_driver_text_activity_number', _default(_set(gnss + ['driver'], None),
                                                                     _set(gnss + ['activity'], 7))),
        ('enabled_text', _default(_set(gnss + ['enabled'], 'yes'))),
        ('enabled_one', _default(_set(gnss + ['enabled'], 1))),
        ('enabled_null', _default(_set(gnss + ['enabled'], None))),
        ('rate_band_floor_zero', _default(_set(flight + ['airspeed_floor_hz'], 0))),
        ('rate_band_floor_text', _default(_set(flight + ['airspeed_floor_hz'], '5'))),
        ('rate_band_floor_float', _default(_set(flight + ['airspeed_floor_hz'], 5.0))),
        ('rate_band_floor_true', _default(_set(flight + ['airspeed_floor_hz'], True))),
        ('rate_band_floor_null', _default(_set(flight + ['airspeed_floor_hz'], None))),
        ('rate_band_inverted', _default(_set(flight + ['airspeed_floor_hz'], 60))),
        ('rate_band_both_wrong', _default(_set(flight + ['airspeed_floor_hz'], -1),
                                          _set(flight + ['airspeed_ceiling_hz'], 'x'))),
        ('every_numeric_field', _default(*[_set(flight + [key], value) for key, value in every_numeric])),
        ('component_fins_multiplier', _default(_set(flight + ['fins'], {'limit_multiplier': 'x'}))),
        ('component_fins_not_an_object', _default(_set(flight + ['fins'], 'x'))),
        ('gains_in_key_order', _default(_set(flight + ['gains'], {'roll': [], 'pitch': {'kp': True, 'ki': None},
                                                                  'yaw': {'kd': 0.1}}))),
        ('gains_not_an_object', _default(_set(flight + ['gains'], 'x'))),
        ('bus_id_undefined', _default(_set(gnss + ['id'], 9))),
        ('bus_id_text', _default(_set(gnss + ['id'], 'x'))),
        ('bus_id_float', _default(_set(gnss + ['id'], 2.0))),
        ('bus_id_true', _default(_set(gnss + ['id'], True))),
        ('bus_id_missing', _default(_delete(laser + ['id']))),
        ('bus_kind_number', _default(_set(gnss + ['bus'], 5))),
        ('bus_null', _default(_set(gnss + ['bus'], None))),
        ('component_bus_undefined', _default(_set(_device(default, 'components', 'recorder') + ['id'], 3))),
        ('addr_text', _default(_set(laser + ['addr'], '0x29'))),
        ('addr_float', _default(_set(laser + ['addr'], 41.0))),
        ('addr_true', _default(_set(laser + ['addr'], True))),
        ('provides_list', _default(_set(laser + ['provides'], []))),
        ('provides_quantity_number', _default(_set(laser + ['provides'], {'agl': 5}))),
        ('provides_quantity_empty', _default(_set(laser + ['provides'], {'agl': {}}))),
        ('provides_wrong_types', _default(_set(laser + ['provides'], {'agl': {'priority': True,
                                                                              'timeout_ms': 100.0}}))),
        ('provides_null', _default(_set(laser + ['provides'], None))),
        ('watchdog_enabled_by_absence', _default(_delete(watchdog + ['enabled']),
                                                 _set(watchdog + ['wdt_timeout_ms'], 1000))),
        ('watchdog_enabled_zero', _default(_set(watchdog + ['enabled'], 0), _set(watchdog + ['wdt_timeout_ms'], 1000))),
        ('watchdog_3999', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], 3999))),
        ('watchdog_4000', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], 4000))),
        ('watchdog_true', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], True))),
        ('watchdog_float', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], 1000.0))),
        ('watchdog_text', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], '1000'))),
        ('watchdog_negative', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], -5))),
        ('watchdog_timeout_missing', _default(*enabled_watchdog, _delete(watchdog + ['wdt_timeout_ms']))),
        ('watchdog_unnamed', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], 1000),
                                      _delete(watchdog + ['name']))),
        ('watchdog_named_by_a_number', _default(*enabled_watchdog, _set(watchdog + ['wdt_timeout_ms'], 1000),
                                                _set(watchdog + ['name'], 5))),
        ('watchdog_twice', _small(_set(['components', 0, 'wdt_timeout_ms'], 100),
                                  _set(['components', 1], {'name': 'second', 'activity': 'watchdog',
                                                           'wdt_timeout_ms': 200}))),
        ('watchdog_as_a_sensor_is_not_checked', _small(_set(['sensors', 1], {'name': 'dog', 'activity': 'watchdog',
                                                                             'wdt_timeout_ms': 100}))),
    ]
    return cases


def _many_at_once() -> list:
    """Several sections broken in one small document, so the section order is pinned too: (name, input)."""
    return [
        ('every_section_at_once', _small(
            _set(['board', 'id'], 'two words'), _set(['board', 'mcu'], 'esp32s3'), _set(['wifi', 'mode'], 'ap'),
            _set(['buses', 'i2c', '0', 'scl'], 7), _set(['buses', 'spi', '1', 'mode'], 7),
            _set(['pins', 'servo_yaw'], 20), _set(['recorder', 'cell_size'], 0),
            _set(['fins', 'limit_multiplier'], '1'), _set(['sensors', 0, 'id'], 4),
            _set(['sensors', 1], {'name': 'baro', 'driver': 'bmp280', 'enabled': 'no'}),
            _set(['components', 0, 'wdt_timeout_ms'], 10))),
    ]


def build() -> dict:
    """
    The config_validate fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, bases, cases}.
    """
    known = config_corpus.bases(_BASE_NAMES)
    known['small'] = config_corpus.sorted_keys(_SMALL)
    default = known['default']
    named = (_corpus() + _test_config(default) + _board_and_wifi() + _buses_and_pins() + _recorder_and_fins()
             + _devices(default) + _many_at_once())
    names = [name for name, _ in named]
    if len(set(names)) != len(names):
        raise ValueError('a case name is used twice')
    cases = [{'name': name, 'input': specification,
              'output': config_module.validate(config_corpus.resolve(specification, known))}
             for name, specification in named]
    return {
        'meta': {'source': 'src/glider/config.py (validate), src/glider/test/test_config.py; ' + config_corpus.SOURCES},
        'compare': {'kind': 'exact'},
        'bases': known,
        'cases': cases,
    }
