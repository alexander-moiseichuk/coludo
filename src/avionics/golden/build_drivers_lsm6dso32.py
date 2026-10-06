"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_lsm6dso32.json: plan B's LSM6DSO32 IMU (src/glider/drivers/lsm6dso32.py) as the Rust port
(crates/drivers, module lsm6dso32) must answer it. Sections, each a fixture of its own:
  * decode   -- sample() over a scripted part: the 12-byte burst at OUTX_L_G (gyro x..z, then accel x..z, signed
                little-endian) and its conversion: accel in g, the gyro's centidegree-per-second integer (the
                telemetry column, exact) and that integer in degrees per second (fixed.to_float, plan B's operator
                view). Compared rel 1e-6: plan B scales in CPython's f64, the port in f32 (0.976 mg and 70 mdps a
                count, each rounded once: under 2e-7 relative), and every integer compares exactly;
  * setup    -- setup()'s transcript over a scripted part: the WRITE-THEN-VERIFY rounds (CTRL3_C, then WHO_AM_I, up
                to 64 of them; a part that reads 0x00 thirty times and recovers is TMS-7E's measurement), the
                configuration (CTRL1_XL 0x44, CTRL2_G 0x4C), and with INT1 wired the route and the clearing read;
                the failures included. Exact;
  * probe    -- probe()'s transcript and the step that failed. Exact;
  * diagnose -- diagnose(): one WHO_AM_I byte through i2cbus._Device and commons.id_classify. Exact;
  * wake     -- the run loop's INT1 verdict: plan B's run() driven by scripted wake counts (irq_runs), recording
                after each wake whether the line is called silent and whether the note fired. With INT1 wired and
                WITHOUT: plan B's run() never reads the wire on this part (unlike the BMI323's), so an unwired
                LSM6DSO32 is counted and called silent too, a quirk the port keeps and these cases pin. Exact.

The transcripts are I2C: plan B's config `bus: 'i2c'` path (an i2cbus._Device window at 0x6A), the same register
window as the SPI one with another framing. The SPI framing (0x80 | register, no multi-byte bit) is the port's unit
test. Plan B gives this part no rearm(); the port's is its bring-up again (the unit tests).

Every value comes from running plan B's own code over driver_bus's scripted part: the builder computes nothing the
port is checked on. Inputs: the frames and counts test_lsm6dso32.py names (1 g at rest, the INT-silent limit), the
rails, and seeded sweeps.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import config_default
import databoard
import driver_bus
import fixed
import i2cbus
import recorder
import register_bus
from drivers import lsm6dso32

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x6A
_INT_PIN: str = 'lsm6dso32_int1'  # the INT1 pin's name in plan B's default pin map (GPIO28)
_IDENTITY: int = 0x6C
_ROUNDS: int = 64  # lsm6dso32._ID_READS


class _Controller:
    """The board the driver reads its pins from: plan B's default config."""

    def __init__(self) -> None:
        self.config = config_default.default()


class _Sink:
    """A databoard channel / telemetry stream stand-in."""

    def push(self, *values) -> None:
        pass

    def value(self):
        return None


def _part(identities: list, refuse: tuple = ()):
    """
    A scripted LSM6DSO32: WHO_AM_I answers the next of `identities` (the last one repeats), a 12-byte burst answers
    zeros, a write acks; a transaction on a register listed in `refuse` (as ('read' | 'write', register)) NAKs.
    """
    answers = list(identities)

    def part(op: str, address: int, register, count: int, data: bytes):
        kind = 'read' if op == 'write_read' else 'write'
        if (kind, register) in refuse:
            raise driver_bus.nak()
        if kind == 'write':
            return None
        if register == lsm6dso32._WHO_AM_I:
            return bytes([answers.pop(0) if len(answers) > 1 else answers[0]])
        return bytes(count)

    return part


def _driver(interrupt_wired: bool) -> lsm6dso32.Lsm6dso32:
    """A driver on plan B's I2C path, INT1 wired or not."""
    block = {'bus': 'i2c', 'id': 0, 'addr': _ADDRESS}
    if interrupt_wired:
        block['int_pin'] = _INT_PIN
    return lsm6dso32.Lsm6dso32('imu_lsm6dso32', block, _Controller())


def _setup_case(name: str, interrupt_wired: bool, part) -> dict:
    """One setup case: plan B's setup() over a scripted part, its transcript and its answer."""
    bus = register_bus.Bus(part)
    driver = _driver(interrupt_wired)
    originals = i2cbus.bind, databoard.Databoard.provide, lsm6dso32.Pin
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: tuple(_Sink() for _ in want))
    lsm6dso32.Pin = bus.pin_class()
    try:
        ok = register_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, databoard.Databoard.provide, lsm6dso32.Pin = originals
    return {'name': name, 'input': {'interrupt_wired': interrupt_wired, 'script': bus.transcript},
            'output': {'ok': ok}}


def _setups() -> dict:
    """The setup section."""
    configuration = lsm6dso32._CTRL2_G
    cases = [
        _setup_case('healthy', False, _part([_IDENTITY])),
        _setup_case('out_of_step_30_reads_then_pinned', False, _part([0x00] * 30 + [_IDENTITY])),
        _setup_case('pinned_on_the_last_round', False, _part([0x00] * (_ROUNDS - 1) + [_IDENTITY])),
        _setup_case('absent_after_every_round', False, _part([0x00])),
        _setup_case('another_part', False, _part([0x6B])),
        _setup_case('the_pin_write_naks', False, _part([_IDENTITY], refuse=(('write', lsm6dso32._CTRL3_C),))),
        _setup_case('who_am_i_naks', False, _part([_IDENTITY], refuse=(('read', lsm6dso32._WHO_AM_I),))),
        _setup_case('the_gyro_configuration_naks', False, _part([_IDENTITY], refuse=(('write', configuration),))),
        _setup_case('int1_wired', True, _part([0x00, 0x00, _IDENTITY])),
        _setup_case('int1_route_naks', True, _part([_IDENTITY], refuse=(('write', lsm6dso32._INT1_CTRL),))),
        _setup_case('int1_clearing_read_naks', True, _part([_IDENTITY], refuse=(('read', lsm6dso32._OUTX_L_G),))),
    ]
    return {'compare': {'kind': 'exact'},
            'note': ('setup() on the i2c path over a scripted part: input whether INT1 is wired and the transcript; '
                     'output ok'),
            'cases': cases}


def _burst(gyro: tuple, accel: tuple) -> bytes:
    """The 12-byte burst a part holding these counts answers: gyro x, y, z then accel x, y, z, little-endian."""
    return b''.join((value & 0xFFFF).to_bytes(2, 'little') for value in gyro + accel)


def _decode_case(name: str, gyro: tuple, accel: tuple) -> dict:
    """One decode case: plan B's sample() over a part answering this burst."""
    bus = register_bus.Bus(driver_bus.sequence([_burst(gyro, accel)]))
    driver = _driver(False)
    driver._dev, driver._buf = bus.device(_ADDRESS), bytearray(12)
    ax, ay, az, gx, gy, gz = register_bus.run(bus, driver.sample())
    return {'name': name, 'input': {'script': bus.transcript},
            'output': {'accel_g': [ax, ay, az], 'rate_cd_s': [gx, gy, gz],
                       'rate_dps': [fixed.to_float(gx), fixed.to_float(gy), fixed.to_float(gz)]}}


def _decodes() -> dict:
    """The decode section."""
    cases = [_decode_case('zero', (0, 0, 0), (0, 0, 0)),
             _decode_case('one_g_at_rest_on_z', (3, -2, 1), (12, -7, 1024)),
             _decode_case('one_count_each_way', (1, -1, 0), (1, -1, 0)),
             _decode_case('the_rails', (32767, -32768, 0), (32767, -32768, 0)),
             _decode_case('a_boost_at_10_g', (-140, 2000, -35), (300, -500, 10246))]
    generator = random.Random(_SEED)
    for index in range(40):
        gyro = tuple(generator.randint(-32768, 32767) for _ in range(3))
        accel = tuple(generator.randint(-32768, 32767) for _ in range(3))
        cases.append(_decode_case('sweep_%02d' % index, gyro, accel))
    return {'compare': {'kind': 'rel', 'tol': 1e-06},
            'note': ('sample(): accel in g (raw x 0.000976), the gyro in centidegrees per second (raw x 70 x 100 // '
                     '1000, exact) and in degrees per second (fixed.to_float of it); rel 1e-6 for the f32 scaling'),
            'cases': cases}


def _probe_case(name: str, part) -> dict:
    """One probe case over a scripted part."""
    bus = register_bus.Bus(part)
    driver = _driver(False)
    driver._dev, driver._buf = bus.device(_ADDRESS), bytearray(12)
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
            'note': "probe(): the transcript and the step that failed ('who_am_i', 'sample'), null when it passed",
            'cases': [_probe_case('passes', _part([_IDENTITY])),
                      _probe_case('another_part', _part([0x6B])),
                      _probe_case('who_am_i_naks', _part([_IDENTITY], refuse=(('read', lsm6dso32._WHO_AM_I),))),
                      _probe_case('the_sample_naks', _part([_IDENTITY], refuse=(('read', lsm6dso32._OUTX_L_G),)))]}


def _diagnose_case(name: str, answer) -> dict:
    """One diagnose case: one WHO_AM_I byte, or a refusal."""
    bus = register_bus.Bus(driver_bus.sequence([answer]))
    driver = _driver(False)
    driver._dev = bus.device(_ADDRESS)
    verdict = register_bus.run(bus, driver.diagnose())
    return {'name': name, 'input': {'script': bus.transcript}, 'output': verdict}


def _diagnoses() -> dict:
    """The diagnose section."""
    answers = (('present', b'\x6c'), ('chip_select_dead', b'\x00'), ('miso_floating', b'\xff'),
               ('crosswired', b'\x6b'), ('absent', driver_bus.NAK))
    return {'compare': {'kind': 'exact'}, 'note': 'diagnose(): one WHO_AM_I byte; the verdict text',
            'cases': [_diagnose_case(name, answer) for name, answer in answers]}


class _Waiter:
    """commons.Waiter stand-in: wait() answers the scripted wake counts in order, then ends the run."""

    def __init__(self, counts: list) -> None:
        self._counts = list(counts)

    async def wait(self, timeout_ms: int) -> int:
        if not self._counts:
            raise register_bus.Stop()
        return self._counts.pop(0)


def _wake_case(name: str, counts: list, interrupt_wired: bool = True) -> dict:
    """One wake scenario: plan B's run() over scripted wake counts, INT1 wired or not, the verdict after each wake."""
    driver = _driver(interrupt_wired)
    driver._ready, driver._period_ms, driver._int_silent = _Waiter(counts), 10, False
    driver._accel = driver._rate = driver._telemetry = _Sink()
    verdicts = []
    noted = []

    def note(template: str = None, arg=None) -> None:
        if template is not None:
            noted.append(True)

    async def sample() -> tuple:
        verdicts.append({'silent': driver._int_silent, 'noted': bool(noted)})
        noted.clear()
        return (0.0, 0.0, 1.0, 0, 0, 0)

    driver.note, driver.sample = note, sample
    bus = register_bus.Bus(driver_bus.sequence([]))
    register_bus.run_for(bus, driver.run(), end_ms=1 << 30)
    if len(verdicts) != len(counts):
        raise ValueError('drivers_lsm6dso32: wake case %s ran %d of %d wakes' % (name, len(verdicts), len(counts)))
    return {'name': name, 'input': {'interrupt_wired': interrupt_wired, 'irq_runs': counts}, 'output': verdicts}


def _wakes() -> dict:
    """The wake section."""
    cases = [_wake_case('healthy', [1] * 6),
             _wake_case('dead_from_the_start', [0] * 8),
             _wake_case('an_edge_resumes_and_a_later_spell_is_reported_again', [0, 0, 0, 0, 1, 0, 0, 0, 0, 2]),
             _wake_case('two_misses_are_not_silence', [0, 0, 1, 0, 0, 1, 0, 0]),
             _wake_case('an_overrun_is_not_silence', [3, 0, 0, 2, 1]),
             _wake_case('no_wire_is_counted_too', [0] * 8, interrupt_wired=False),
             _wake_case('no_wire_an_edge_resumes', [0, 0, 0, 0, 1, 0, 0, 0, 2], interrupt_wired=False)]
    generator = random.Random(_SEED + 1)
    for index in range(8):
        cases.append(_wake_case('sweep_%02d' % index, [generator.choice((0, 0, 0, 1, 1, 2)) for _ in range(40)]))
    return {'compare': {'kind': 'exact'},
            'note': ('run() with wait() answering the scripted irq_runs, INT1 wired or not (plan B counts both): '
                     'after each wake, whether the INT line is called silent and whether the note fired (once, at '
                     'the limit of 3 timed-out wakes in a row)'),
            'cases': cases}


def build() -> dict:
    """
    The drivers_lsm6dso32 fixture document.

    Returns:
        {meta, decode, setup, probe, diagnose, wake}.
    """
    return {
        'meta': {'source': ('src/glider/drivers/lsm6dso32.py, src/glider/i2cbus.py, src/glider/task.py, '
                            'src/glider/test/test_lsm6dso32.py')},
        'decode': _decodes(),
        'setup': _setups(),
        'probe': _probes(),
        'diagnose': _diagnoses(),
        'wake': _wakes(),
    }
