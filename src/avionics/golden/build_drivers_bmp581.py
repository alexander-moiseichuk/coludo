"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_bmp581.json: plan B's BMP581 barometer (src/glider/drivers/bmp581.py) as the Rust port
(crates/drivers, module bmp581) must answer it. Sections, each a fixture of its own:
  * read     -- _read() over a scripted part: the 6-byte burst (temperature 24-bit two's complement, pressure 24-bit
                unsigned, little-endian) and the altitude (m), temperature (degrees C) and pressure (Pa) it converts
                to. Each member to its own margin (the section's `fields`): temperature and pressure EXACT, as the
                counts scale by powers of two exactly in both precisions (the builder proves it per case), and the
                altitude abs 0.005 m: commons.altitude_m, whose f32 port is within 3.4 mm of plan B's f64 (2.9 mm
                over this section). One abs 0.01 for all three let a decode that drops the temperature's XLSB byte
                (under 1/256 degrees C) pass. test_bmp581.py's frames (25 degrees C and the sub-zero sign
                extension), a dead read, frames with non-zero XLSB bytes, and seeded sweeps over 30-110 kPa and
                -40..85 degrees C;
  * setup    -- setup()'s transcript: the identity, OSR_CONFIG 0x50 and ODR_CONFIG 0xBD, the first conversion's wait,
                and the eight-reading ground zero; the failures included. abs 0.01 on the ground (an f32 average of
                f32 altitudes);
  * rearm    -- rearm()'s transcript: the mode written again, out of the STANDBY a reset leaves the part in; and
                whether plan B logged a failure. Exact;
  * update   -- update(): a direct ground (a number, or what plan B's float() also takes: a numeral string, a bool),
                a re-zero from a fresh altitude (the channel's recent() within three periods) or its refusal, and the
                property list. Exact (the grounds are exact f32 values). One `divergence`: a ground that is not a
                number is refused in the port's words, where plan B answered float()'s own text, which is the
                interpreter's (CPython's here, MicroPython's on the board);
  * probe    -- probe()'s transcript and the step that failed. Exact;
  * diagnose -- diagnose(): one identity byte through i2cbus._Device and commons.id_classify. Exact.

Every value comes from running plan B's own code over driver_bus's scripted part: the builder computes nothing the
port is checked on.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random
import struct

import config_default
import databoard
import driver_bus
import i2cbus
import recorder
import register_bus
from drivers import bmp581

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x47
_IDENTITY: int = 0x50
_TOLERANCE_ALTITUDE_M: float = 0.005  # the read section's altitude: 2.9 mm measured at worst


class _Controller:
    """The board the driver reads its config from: plan B's default config."""

    def __init__(self) -> None:
        self.config = config_default.default()


class _Sink:
    """A databoard channel / telemetry stream stand-in."""

    def push(self, *values) -> None:
        pass

    def value(self):
        return None


def _burst(temperature_raw: int, pressure_raw: int) -> bytes:
    """A 6-byte burst: the temperature count (24-bit two's complement) then the pressure count, little-endian."""
    return (temperature_raw & 0xFFFFFF).to_bytes(3, 'little') + (pressure_raw & 0xFFFFFF).to_bytes(3, 'little')


_PAD: bytes = _burst(1638400, 6485504)  # test_bmp581.py: 25.0 degrees C, 101336 Pa


def _part(identity: int = _IDENTITY, bursts: list = None, refuse: tuple = ()):
    """
    A scripted BMP581: the identity read answers `identity`; a burst read answers the next of `bursts` (the last
    repeats; default the pad reading); a write acks. A transaction on a register listed in `refuse` (as ('read' |
    'write', register)) NAKs.
    """
    queue = list(bursts or [_PAD])

    def part(op: str, address: int, register, count: int, data: bytes):
        kind = 'read' if op == 'write_read' else 'write'
        if (kind, register) in refuse:
            raise driver_bus.nak()
        if kind == 'write':
            return None
        if register == bmp581._REG_CHIP_ID:
            return bytes([identity])
        return queue.pop(0) if len(queue) > 1 else queue[0]

    return part


def _driver() -> bmp581.Bmp581:
    """A driver at the default address."""
    return bmp581.Bmp581('baro_bmp581', {'bus': 'i2c', 'id': 0, 'addr': _ADDRESS}, _Controller())


def _single(name: str, value: float) -> float:
    """
    A value plan B computed, proven exact in single precision before the fixture asks the port for it exactly: a
    24-bit count over a power of two needs at most 24 significant bits, so f32 and f64 agree to the bit.

    Args:
        name - the case, for the refusal.
        value - plan B's value.

    Returns:
        The value, unchanged.
    """
    if struct.unpack('<f', struct.pack('<f', value))[0] != value:
        raise ValueError('drivers_bmp581: %s: %r is not exact in f32' % (name, value))
    return value


def _read_case(name: str, temperature_raw: int, pressure_raw: int) -> dict:
    """One read case: plan B's _read() over a part answering this burst."""
    bus = register_bus.Bus(driver_bus.sequence([_burst(temperature_raw, pressure_raw)]))
    driver = _driver()
    driver._bus, driver._addr, driver._buf = bus, _ADDRESS, bytearray(6)
    altitude, temperature, pressure = register_bus.run(bus, driver._read())
    return {'name': name, 'input': {'script': bus.transcript},
            'output': {'altitude': altitude, 'temperature': _single(name, temperature),
                       'pressure': _single(name, pressure)}}


def _reads() -> dict:
    """The read section."""
    cases = [_read_case('test_bmp581_25_c', 1638400, 6485504),
             _read_case('test_bmp581_minus_10_c', -655360, 6485504),
             _read_case('a_dead_read_is_no_altitude', 0, 0),
             _read_case('one_count_each_way', -1, 1),
             _read_case('the_rails', -(1 << 23), (1 << 24) - 1),
             _read_case('sea_level', 0, 101325 * 64),
             _read_case('a_mountain_top', -30 * 65536, 30000 * 64),
             _read_case('the_xlsb_counts', 25 * 65536 + 1, 101336 * 64 + 1),
             _read_case('the_xlsb_counts_below_zero', -10 * 65536 - 1, 101336 * 64 + 63)]
    generator = random.Random(_SEED)
    for index in range(40):
        cases.append(_read_case('sweep_%02d' % index, generator.randint(-40 * 65536, 85 * 65536),
                                generator.randint(30000 * 64, 110000 * 64)))
    return {'compare': {'kind': 'exact'},
            'fields': {'altitude': {'kind': 'abs', 'tol': _TOLERANCE_ALTITUDE_M}},
            'note': ("_read(): altitude m, temperature degrees C, pressure Pa; temperature and pressure exact "
                     "(power-of-two scalings), altitude abs 0.005 (commons.altitude_m's f32)"),
            'cases': cases}


def _setup(part) -> tuple:
    """
    Run plan B's setup() over a scripted part.

    Returns:
        (the driver after it, setup's answer, the bus with its transcript).
    """
    bus = register_bus.Bus(part)
    driver = _driver()
    originals = i2cbus.bind, databoard.Databoard.provide
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: tuple(_Sink() for _ in want))
    try:
        ok = register_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, databoard.Databoard.provide = originals
    return driver, ok, bus


def _setup_case(name: str, part) -> dict:
    """One setup case: the transcript, setup's answer and the ground zero it captured."""
    driver, ok, bus = _setup(part)
    return {'name': name, 'input': {'script': bus.transcript},
            'output': {'ok': ok, 'ground': driver._ground if ok else None}}


def _setups() -> dict:
    """The setup section."""
    generator = random.Random(_SEED + 1)
    drifting = [_burst(1638400 + generator.randint(-3000, 3000), 6485504 + generator.randint(-200, 200))
                for _ in range(8)]
    cases = [_setup_case('healthy', _part()),
             _setup_case('a_drifting_pad', _part(bursts=drifting)),
             _setup_case('another_part', _part(identity=0x58)),
             _setup_case('the_identity_naks', _part(refuse=(('read', bmp581._REG_CHIP_ID),))),
             _setup_case('the_rate_naks', _part(refuse=(('write', bmp581._REG_ODR_CONFIG),))),
             _setup_case('a_read_naks_while_averaging', _part(refuse=(('read', bmp581._REG_TEMP_DATA),)))]
    return {'compare': {'kind': 'abs', 'tol': 0.01},
            'note': 'setup() over a scripted part: the transcript; output ok and the ground zero (m) it captured',
            'cases': cases}


def _rearm_case(name: str, refuse: tuple = ()) -> dict:
    """One rearm case: rearm() over a part that refuses `refuse`, and whether plan B logged a failure."""
    bus = register_bus.Bus(_part(refuse=refuse))
    driver = _driver()
    driver._bus, driver._addr = bus, _ADDRESS
    logged = []
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: logged.append(message))
    try:
        register_bus.run(bus, driver.rearm())
    finally:
        recorder.Recorder.log = original
    return {'name': name, 'input': {'script': bus.transcript}, 'output': {'ok': not logged}}


def _rearms() -> dict:
    """The rearm section."""
    return {'compare': {'kind': 'exact'},
            'note': 'rearm(): the transcript (the mode written again) and ok (plan B logs a failure)',
            'cases': [_rearm_case('writes_the_mode_again'),
                      _rearm_case('a_dead_part_is_logged', refuse=(('write', bmp581._REG_OSR_CONFIG),))]}


class _Altitude:
    """The driver's own altitude channel stand-in: recent() answers the scripted fresh altitude, or None."""

    def __init__(self, fresh) -> None:
        self._fresh = fresh

    def recent(self, window_us: int):
        return self._fresh


def _update_case(name: str, properties: dict, fresh, owed_refusal: str = None) -> dict:
    """
    One update case: plan B's update() from a ground of 7.0 m, with this fresh altitude (None: stale).

    Args:
        name - the case name.
        properties - the update's properties.
        fresh - the fresh altitude the channel answers, or None.
        owed_refusal - the port's own refusal text where plan B's was the interpreter's (a `divergence`), or None.

    Returns:
        The case.
    """
    driver = _driver()
    driver._period_ms, driver._ground, driver._altitude = 100, 7.0, _Altitude(fresh)
    try:
        answer = {'changed': driver.update(properties), 'ground': driver._ground}
    except ValueError as error:
        answer = {'refused': str(error), 'ground': driver._ground}
    case = {'name': name, 'input': {'properties': properties, 'fresh_altitude': fresh}, 'output': answer}
    if owed_refusal is not None:
        if 'refused' not in answer:
            raise ValueError('drivers_bmp581: update case %s is no refusal' % name)
        case['divergence'] = {
            'note': ("float()'s own error text is the interpreter's (%r here); the port refuses in its own words"
                     % answer['refused']),
            'output': dict(answer, refused=owed_refusal),
        }
    return case


def _updates() -> dict:
    """The update section."""
    return {'compare': {'kind': 'exact'},
            'note': ('update() from a ground of 7.0 m: the changed properties and the ground, or the refusal text '
                     'and the ground left as it was; a case with a divergence owes its output instead'),
            'cases': [_update_case('a_direct_ground', {'ground': 12.5}, None),
                      _update_case('a_re_zero_from_a_fresh_altitude', {'rezero': True}, 123.5),
                      _update_case('a_re_zero_with_no_fresh_altitude', {'rezero': True}, None),
                      _update_case('a_falsy_re_zero', {'rezero': False}, 123.5),
                      _update_case('the_ground_wins_over_a_re_zero', {'ground': 3.0, 'rezero': True}, 9.0),
                      _update_case('nothing_known', {'other': 1}, 1.0),
                      _update_case('a_numeral_string_ground', {'ground': '12.5'}, None),
                      _update_case('a_bool_ground', {'ground': True}, None),
                      _update_case('a_word_ground_is_refused', {'ground': 'high'}, None,
                                   owed_refusal='ground must be a number')]}


def _probe_case(name: str, part) -> dict:
    """One probe case over a scripted part."""
    bus = register_bus.Bus(part)
    driver = _driver()
    driver._bus, driver._addr, driver._buf = bus, _ADDRESS, bytearray(6)
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    try:
        message = register_bus.run(bus, driver.probe())
    finally:
        recorder.Recorder.log = original
    return {'name': name, 'input': {'script': bus.transcript},
            'output': {'failed': None if message is None else message.split(':')[0]}}


def _probes() -> dict:
    """The probe section."""
    return {'compare': {'kind': 'exact'},
            'note': "probe(): the transcript and the step that failed ('chip id', 'read'), null when it passed",
            'cases': [_probe_case('passes', _part()),
                      _probe_case('another_part', _part(identity=0x58)),
                      _probe_case('the_identity_naks', _part(refuse=(('read', bmp581._REG_CHIP_ID),))),
                      _probe_case('the_read_naks', _part(refuse=(('read', bmp581._REG_TEMP_DATA),)))]}


def _diagnose_case(name: str, answer) -> dict:
    """One diagnose case: one identity byte, or a refusal."""
    bus = register_bus.Bus(driver_bus.sequence([answer]))
    driver = _driver()
    driver._bus, driver._addr = bus, _ADDRESS
    verdict = register_bus.run(bus, driver.diagnose())
    return {'name': name, 'input': {'script': bus.transcript}, 'output': verdict}


def _diagnoses() -> dict:
    """The diagnose section."""
    answers = (('present', b'\x50'), ('unpowered', b'\x00'), ('idle_high', b'\xff'), ('crosswired', b'\x58'),
               ('absent', driver_bus.NAK))
    return {'compare': {'kind': 'exact'}, 'note': 'diagnose(): one identity byte; the verdict text',
            'cases': [_diagnose_case(name, answer) for name, answer in answers]}


def build() -> dict:
    """
    The drivers_bmp581 fixture document.

    Returns:
        {meta, read, setup, rearm, update, probe, diagnose}.
    """
    return {
        'meta': {'source': ('src/glider/drivers/bmp581.py, src/glider/commons.py, src/glider/i2cbus.py, '
                            'src/glider/test/test_bmp581.py')},
        'read': _reads(),
        'setup': _setups(),
        'rearm': _rearms(),
        'update': _updates(),
        'probe': _probes(),
        'diagnose': _diagnoses(),
    }
