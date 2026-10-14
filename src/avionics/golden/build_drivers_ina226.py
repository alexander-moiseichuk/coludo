"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_ina226.json: plan B's INA226 power monitor (src/glider/drivers/ina226.py) as the Rust port
(crates/drivers, module ina226) must answer it. Sections, each a fixture of its own, all compared exactly (the
driver is integer milli-units end to end, in plan B and in the port):
  * calibration -- the CALIBRATION register setup() writes for a full-scale current and a shunt: 0.00512 x 2^15 x
                   1e6 // (mA x mOhm), 5000 mA over 10 mOhm -> 3355 (test_ina226.py), or no value where plan B's
                   setup fails (a zero product, a quotient past 16 bits);
  * alert       -- the ALERT LIMIT setup() writes with an alert pin wired (mA x mOhm x 2 // 5), or none where
                   plan B falls back to polling only;
  * read        -- _read() over a scripted part: THREE separate 2-byte reads (bus voltage, current, power) and the
                   mV / mA / mW they convert to, current signed;
  * setup       -- setup()'s I2C transcript: die id, CONFIGURATION, CALIBRATION, then ALERT LIMIT and MASK/ENABLE
                   with a pin wired, the failures included -- a scaling number that is not an integer among them
                   (plan B's float CALIBRATION fails struct.pack: setup fails; a fractional trip polls only);
  * rearm       -- rearm()'s transcript: CONFIGURATION and the CALIBRATION setup wrote, each refused once. With the
                   alert armed the port also restores ALERT LIMIT and MASK/ENABLE (avionics-port.md §6), so those
                   cases carry a `divergence`: the transcript the port owes instead, plan B's plus the two writes
                   setup made, as the refusing part answers them (a refusal ends the re-arm and fails it);
  * probe       -- probe()'s transcript and verdict: the die id, then one read;
  * diagnose    -- diagnose(): the die id's high byte through i2cbus._Device and commons.id_classify.

Every value comes from running plan B's own code over driver_bus's scripted part: the builder computes nothing the
port is checked on. The calibration and alert cases are setup() runs whose CALIBRATION / ALERT LIMIT write is
read back out of the transcript.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import config_default
import databoard
import driver_bus
import i2cbus
import recorder
from drivers import ina226

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x40
_ALERT_PIN: str = 'ina_alert'  # the alert pin's name in the stand-in board's pin map
_ALERT_GPIO: int = 9


class _Controller:
    """The board the driver reads its pins from: plan B's default config, with an alert pin mapped."""

    def __init__(self) -> None:
        self.config = config_default.default()
        self.config['pins'][_ALERT_PIN] = _ALERT_GPIO


class _Channel:
    """A databoard channel stand-in."""

    def push(self, value) -> None:
        pass

    def value(self):
        return None


def _part(die: int = ina226._DIE_ID, registers: dict = None, refuse: tuple = ()):
    """
    A scripted INA226: a register read answers the register's word (the die id at 0xFF), a write acks; a
    transaction on a register listed in `refuse` (as ('read' | 'write', register)) is refused.
    """
    words = {0xFF: die}
    words.update(registers or {})

    def part(op: str, address: int, register, count: int, data: bytes):
        kind = 'read' if op == 'write_read' else 'write'
        if (kind, register) in refuse:
            raise driver_bus.nak()
        if kind == 'read':
            return words.get(register, 0).to_bytes(2, 'big')[:count]
        return None

    return part


def _setup(config: dict, alert_wired: bool, part) -> tuple:
    """
    Run plan B's setup() over a scripted part.

    Returns:
        (the driver after it, setup's answer, the bus with its transcript).
    """
    bus = driver_bus.Bus(part)
    block = dict(config)
    if alert_wired:
        block['alert_pin'] = _ALERT_PIN
    driver = ina226.Ina226('power_ina226', block, _Controller())
    originals = i2cbus.bind, databoard.Databoard.provide, ina226.Pin
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: tuple(_Channel() for _ in want))
    ina226.Pin = bus.pin_class()
    try:
        ok = driver_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, databoard.Databoard.provide, ina226.Pin = originals
    return driver, ok, bus


def _written(bus: driver_bus.Bus, register: int):
    """The 16-bit value the transcript wrote to `register` (the last write), None when it wrote none."""
    value = None
    for step in bus.transcript:
        if step['op'] == 'write' and not step.get('fails') and step['bytes'][:2] == '%02x' % register:
            value = int(step['bytes'][2:], 16)
    return value


def _calibrations() -> dict:
    """The calibration section."""
    pairs = [(5000, 10), (3000, 10), (20000, 2), (1000, 1), (2560, 1), (2561, 1), (0, 10), (5000, 0), (-5000, 10),
             (-5000, -10), (167772160, 1), (167772161, 1), (100000000000, 100000000000), (1, 1)]
    generator = random.Random(_SEED)
    pairs += [(generator.randint(1, 40000), generator.randint(1, 100)) for _ in range(40)]
    cases = []
    for max_current, shunt in pairs:
        driver, ok, bus = _setup({'max_current_ma': max_current, 'shunt_mohms': shunt}, False, _part())
        cases.append({'name': 'max_%d_ma_shunt_%d_mohm' % (max_current, shunt),
                      'input': {'max_current_ma': max_current, 'shunt_mohms': shunt},
                      'output': _written(bus, ina226._REG_CALIB) if ok else None})
    return {'compare': {'kind': 'exact'},
            'note': 'the CALIBRATION value setup() writes; null where plan B setup fails before writing it',
            'cases': cases}


def _alerts() -> dict:
    """The alert section."""
    pairs = [(3000, 10), (5000, 10), (1, 1), (2, 1), (3, 1), (0, 10), (-1, 10), (163837, 1), (163838, 1),
             (16383, 10), (16384, 10)]
    generator = random.Random(_SEED + 1)
    pairs += [(generator.randint(1, 20000), generator.randint(1, 20)) for _ in range(30)]
    cases = []
    for alert, shunt in pairs:
        driver, ok, bus = _setup({'alert_ma': alert, 'shunt_mohms': shunt, 'max_current_ma': 5000}, True, _part())
        if not ok:
            raise ValueError('drivers_ina226: alert case %r failed setup' % ((alert, shunt),))
        armed = _written(bus, ina226._REG_MASK) is not None  # the SOL mask is the last write of an armed alert
        cases.append({'name': 'alert_%d_ma_shunt_%d_mohm' % (alert, shunt),
                      'input': {'alert_ma': alert, 'shunt_mohms': shunt},
                      'output': _written(bus, ina226._REG_ALERT_LIM) if armed else None})
    return {'compare': {'kind': 'exact'},
            'note': ('the ALERT LIMIT setup() arms with a pin wired (max_current_ma 5000), read from the transcript; '
                     'null for poll only. A trip of 0 mA arms a limit of 0, as plan B writes it'),
            'cases': cases}


def _read_case(name: str, max_current: int, bus_voltage: int, current: int, power: int) -> dict:
    """One read case: plan B's _read() over a part holding these raw words."""
    part = _part(registers={0x02: bus_voltage, 0x03: power, 0x04: current & 0xFFFF})
    bus = driver_bus.Bus(part)
    driver = ina226.Ina226('power_ina226', {}, _Controller())
    driver._bus, driver._addr, driver._max_current_ma = bus, _ADDRESS, max_current
    voltage, current_ma, power_mw = driver_bus.run(bus, driver._read())
    return {'name': name, 'input': {'max_current_ma': max_current, 'script': bus.transcript},
            'output': {'voltage_mv': voltage, 'current_ma': current_ma, 'power_mw': power_mw}}


def _reads() -> dict:
    """The read section."""
    cases = [_read_case('bench_bus_0x0f95', 5000, 0x0F95, 1200, 300),
             _read_case('a_charging_current', 5000, 3300, -1200, 300),
             _read_case('the_most_negative_current', 5000, 0, -32768, 0),
             _read_case('full_scale', 5000, 65535, 32767, 65535),
             _read_case('zero', 5000, 0, 0, 0),
             _read_case('one_lsb_negative_floors', 5000, 1, -1, 1),
             _read_case('a_1_ma_full_scale', 1, 3, -3, 3)]
    generator = random.Random(_SEED + 2)
    for index in range(40):
        custom = generator.randint(1, 60000)  # drawn first: the order the sweep has always drawn in
        full_scale = generator.choice((5000, 3000, 20000, custom))
        cases.append(_read_case('sweep_%02d' % index, full_scale, generator.randint(0, 65535),
                                generator.randint(-32768, 32767), generator.randint(0, 65535)))
    return {'compare': {'kind': 'exact'},
            'note': ('_read() with _max_current_ma: the transcript (three 2-byte reads: 0x02, 0x04, 0x03) and the '
                     'mV / mA / mW'),
            'cases': cases}


def _setup_case(name: str, config: dict, alert_wired: bool, part) -> dict:
    """One setup case: the transcript and what setup() left armed."""
    driver, ok, bus = _setup(config, alert_wired, part)
    return {'name': name, 'input': {'config': config, 'alert_wired': alert_wired, 'script': bus.transcript},
            'output': {'ok': ok, 'calibration': getattr(driver, '_cal', None) if ok else None,
                       'alert_ma': driver._alert_ma if ok else 0}}


def _setups() -> dict:
    """The setup section."""
    base = {'shunt_mohms': 10, 'max_current_ma': 5000}
    cases = [
        _setup_case('healthy', base, False, _part()),
        _setup_case('defaults', {}, False, _part()),
        _setup_case('another_address', {'addr': 0x41}, False, _part()),
        _setup_case('not_an_ina226', base, False, _part(die=0x2270)),
        _setup_case('nothing_acks', base, False, _part(refuse=(('read', 0xFF),))),
        _setup_case('configuration_naks', base, False, _part(refuse=(('write', 0x00),))),
        _setup_case('calibration_naks', base, False, _part(refuse=(('write', 0x05),))),
        _setup_case('a_calibration_past_16_bits', {'shunt_mohms': 1, 'max_current_ma': 1000}, False, _part()),
        _setup_case('a_zero_shunt', {'shunt_mohms': 0}, False, _part()),
        # a scaling number that is not an integer: plan B's float CALIBRATION fails struct.pack after the
        # CONFIGURATION write, so setup fails -- 10.5 is what backing the shunt out of a known current produces
        _setup_case('a_fractional_shunt', {'shunt_mohms': 10.5, 'max_current_ma': 5000}, False, _part()),
        _setup_case('a_whole_float_shunt', {'shunt_mohms': 10.0, 'max_current_ma': 5000}, False, _part()),
        _setup_case('a_fractional_full_scale', {'shunt_mohms': 10, 'max_current_ma': 5000.5}, False, _part()),
        _setup_case('the_alert_armed', dict(base, alert_ma=3000), True, _part()),
        _setup_case('the_alert_at_its_default', base, True, _part()),
        _setup_case('an_alert_limit_past_16_bits', dict(base, alert_ma=20000), True, _part()),
        _setup_case('the_alert_limit_naks', dict(base, alert_ma=3000), True, _part(refuse=(('write', 0x07),))),
        _setup_case('the_mask_naks', dict(base, alert_ma=3000), True, _part(refuse=(('write', 0x06),))),
        _setup_case('no_pin_no_alert_writes', dict(base, alert_ma=3000), False, _part()),
        _setup_case('a_fractional_alert_trip_polls_only', dict(base, alert_ma=2500.5), True, _part()),
    ]
    return {'compare': {'kind': 'exact'},
            'note': ('setup() over a scripted part: input the component block, whether an alert pin is wired and '
                     'the transcript; output ok, the CALIBRATION written (null on failure) and the alert trip armed '
                     '(0: none, or poll only)'),
            'cases': cases}


def _rearm_case(name: str, config: dict, alert_wired: bool, refuse: tuple = ()) -> dict:
    """One rearm case: setup over a healthy part, then rearm() over one that refuses `refuse`."""
    driver, ok, setup_bus = _setup(config, alert_wired, _part())
    if not ok:
        raise ValueError('drivers_ina226: rearm case %s failed setup' % name)
    bus = driver_bus.Bus(_part(refuse=refuse))
    driver._bus = bus
    logged = []
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: logged.append(message))
    try:
        driver_bus.run(bus, driver.rearm())
    finally:
        recorder.Recorder.log = original
    case = {'name': name, 'input': {'config': config, 'alert_wired': alert_wired, 'setup': setup_bus.transcript,
                                    'script': bus.transcript},
            'output': {'ok': not logged}}
    if driver._alert_ma:
        # the port's two extra writes, as the part answers them: the same two setup made, ALERT LIMIT then
        # MASK/ENABLE, stopping at the first one `refuse` refuses (marked failed, as driver_bus records a refusal)
        alert_writes, refused = [], False
        for step in setup_bus.transcript:
            if step['op'] != 'write' or step['bytes'][:2] not in ('07', '06'):
                continue
            step = dict(step)
            if ('write', int(step['bytes'][:2], 16)) in refuse:
                step['fails'], refused = True, True
            alert_writes.append(step)
            if refused:
                break
        case['divergence'] = {
            'note': ('the port also restores ALERT LIMIT and MASK/ENABLE, which the general call cleared and plan B '
                     'left so: the same two writes setup made, after plan B two; a refused one ends the re-arm '
                     'and fails it'),
            'script': bus.transcript + alert_writes,
            'ok': not logged and not refused,
        }
    return case


def _rearms() -> dict:
    """The rearm section."""
    base = {'shunt_mohms': 10, 'max_current_ma': 5000}
    cases = [_rearm_case('restores_configuration_and_calibration', base, False),
             _rearm_case('restores_a_custom_calibration', {'shunt_mohms': 2, 'max_current_ma': 20000}, False),
             _rearm_case('a_dead_bus_is_reported', base, False, refuse=(('write', 0x00),)),
             _rearm_case('the_calibration_naks', base, False, refuse=(('write', 0x05),)),
             _rearm_case('with_the_alert_armed', dict(base, alert_ma=3000), True),
             _rearm_case('the_alert_limit_naks', dict(base, alert_ma=3000), True, refuse=(('write', 0x07),)),
             _rearm_case('the_mask_naks', dict(base, alert_ma=3000), True, refuse=(('write', 0x06),))]
    return {'compare': {'kind': 'exact'},
            'note': ('rearm() after a healthy setup (its transcript in `setup`): the re-arm transcript and ok (plan B '
                     'logs a failure); a case with a divergence owes its script and ok instead -- an armed alert, '
                     'whose two extra writes plan B never made, so a refusal of either fails only the port'),
            'cases': cases}


def _probe_case(name: str, part) -> dict:
    """One probe case over a scripted part (the driver as setup left it, default calibration)."""
    bus = driver_bus.Bus(part)
    driver = ina226.Ina226('power_ina226', {}, _Controller())
    driver._bus, driver._addr, driver._max_current_ma = bus, _ADDRESS, 5000
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    try:
        message = driver_bus.run(bus, driver.probe())
    finally:
        recorder.Recorder.log = original
    failed = None if message is None else message.split(':')[0]
    return {'name': name, 'input': {'script': bus.transcript}, 'output': {'failed': failed}}


def _probes() -> dict:
    """The probe section."""
    words = {0x02: 2900, 0x03: 120, 0x04: 300}
    return {'compare': {'kind': 'exact'},
            'note': "probe(): the transcript and the step that failed ('die id', 'read'), null when it passed",
            'cases': [_probe_case('passes', _part(registers=words)),
                      _probe_case('another_die', _part(die=0x5449, registers=words)),
                      _probe_case('the_die_id_naks', _part(refuse=(('read', 0xFF),))),
                      _probe_case('the_current_naks', _part(registers=words, refuse=(('read', 0x04),)))]}


def _diagnose_case(name: str, answer) -> dict:
    """One diagnose case: the die id's high byte, or a refusal."""
    bus = driver_bus.Bus(driver_bus.sequence([answer]))
    driver = ina226.Ina226('power_ina226', {}, _Controller())
    driver._bus, driver._addr = bus, _ADDRESS
    verdict = driver_bus.run(bus, driver.diagnose())
    return {'name': name, 'input': {'script': bus.transcript}, 'output': verdict}


def _diagnoses() -> dict:
    """The diagnose section."""
    answers = (('present', b'\x22'), ('unpowered', b'\x00'), ('idle_high', b'\xff'), ('crosswired', b'\x40'),
               ('absent', driver_bus.NAK))
    return {'compare': {'kind': 'exact'}, 'note': 'diagnose(): one byte of the die id; the verdict text',
            'cases': [_diagnose_case(name, answer) for name, answer in answers]}


def build() -> dict:
    """
    The drivers_ina226 fixture document.

    Returns:
        {meta, calibration, alert, read, setup, rearm, probe, diagnose}.
    """
    return {
        'meta': {'source': 'src/glider/drivers/ina226.py, src/glider/i2cbus.py, src/glider/test/test_ina226.py'},
        'calibration': _calibrations(),
        'alert': _alerts(),
        'read': _reads(),
        'setup': _setups(),
        'rearm': _rearms(),
        'probe': _probes(),
        'diagnose': _diagnoses(),
    }
