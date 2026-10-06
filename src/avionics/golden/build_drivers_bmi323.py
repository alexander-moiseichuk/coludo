"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_bmi323.json: plan B's BMI323 IMU (src/glider/drivers/bmi323.py) as the Rust port (crates/drivers,
module bmi323) must answer it. Sections, each a fixture of its own:
  * decode   -- _read() over a scripted part: the 14-byte burst (TWO DUMMY BYTES, then accel x..z and gyro x..z,
                signed little-endian), the accel in g and the gyro's centidegree-per-second integer (raw x 2000 x 100
                // 32768, floored: the telemetry column). Exact: the accel's 1/2048 g is a power of two, exact in f32
                as in f64. test_bmi323.py's frames (non-zero dummy bytes, the rails both ways) and a seeded sweep;
  * rate     -- the same bursts, the gyro in degrees per second: plan B's fixed.to_float of its floored integer,
                which the port's exact f32 (2000/32768 dps a count) is within 0.01 of -- the floor's quantum;
  * setup    -- setup()'s transcript: the identity PAST the dummy bytes, the soft reset, ACC_CONF 0x7038 and GYR_CONF
                0x7048, the start-up wait (the gyro reports 0x8000 until it has a sample; a part stuck there fails),
                and with INT1 wired the route and the clearing read; the failures included, each bus step's NAK
                among them (the start-up look and the clearing read too: plan B's try turns either into False).
                Exact;
  * rearm    -- rearm()'s transcript after a healthy setup, and whether plan B logged a failure (a dead part, and a
                start-up look that NAKs, are both logged). Two cases carry a
                `divergence`, the answer the port owes on purpose (avionics-port.md: a re-arm restores everything
                setup wrote): with INT1 wired the port routes it again (plan B's re-arm left it unrouted), and a gyro
                that never leaves start-up is reported (plan B dropped the verdict). Exact;
  * wake     -- the run loop's INT1 verdict: plan B's run() over scripted wake counts, with a wire declared and with
                none (counted only with one), after each wake whether the line is called silent (null: no wire) and
                whether the note fired. Exact;
  * probe    -- probe()'s transcript and the step that failed. Exact;
  * diagnose -- diagnose()'s verdict over one identity read. The error's own repr ("OSError(19, 'ENODEV')") is the
                runtime's, so a no-ack verdict is kept up to its ' -- '. Exact.

Every value comes from running plan B's own code over driver_bus's scripted part: the builder computes nothing the
port is checked on.

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
from drivers import bmi323

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x69
_INT_PIN: str = 'accel_int1'  # the pin layout routes to the BMI323 on the v1.x map (GPIO4)
_IDENTITY: int = 0x43
_MARKER: int = -32768  # 0x8000: no gyro sample yet


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


def _burst(accel: tuple, gyro: tuple) -> bytes:
    """A 14-byte burst: two dummy bytes (deliberately not zero), then accel x, y, z and gyro x, y, z, little-endian."""
    return b'\xa5\x5a' + b''.join((value & 0xFFFF).to_bytes(2, 'little') for value in accel + gyro)


_STARTED: bytes = _burst((0, 0, 2048), (3, -2, 1))  # a part at rest with its gyro running
_STARTING: bytes = _burst((0, 0, 2048), (_MARKER, _MARKER, _MARKER))  # the gyro still in its start-up state


def _part(identity: int = _IDENTITY, bursts: list = None, refuse: tuple = (), dummy: int = 0x00,
          refuse_at: tuple = ()):
    """
    A scripted BMI323: the identity read answers two `dummy` bytes then `identity`; a burst read answers the next of
    `bursts` (the last one repeats; default a running part); a write acks. A transaction on a register listed in
    `refuse` (as ('read' | 'write', register)) NAKs, and so does the transaction numbered in `refuse_at` (counting
    every transaction from 0), for a register that is read more than once and refused only the once.
    """
    queue = list(bursts or [_STARTED])
    state = {'count': -1}

    def part(op: str, address: int, register, count: int, data: bytes):
        kind = 'read' if op == 'write_read' else 'write'
        state['count'] += 1
        if (kind, register) in refuse or state['count'] in refuse_at:
            raise driver_bus.nak()
        if kind == 'write':
            return None
        if register == bmi323._REG_CHIP_ID:
            return bytes([dummy, dummy, identity])
        return queue.pop(0) if len(queue) > 1 else queue[0]

    return part


def _driver(interrupt_wired: bool) -> bmi323.Bmi323:
    """A driver at the default address, INT1 declared or not."""
    block = {'bus': 'i2c', 'id': 0, 'addr': _ADDRESS, 'int_pin': _INT_PIN if interrupt_wired else None}
    return bmi323.Bmi323('imu_bmi323', block, _Controller())


def _setup(interrupt_wired: bool, part) -> tuple:
    """
    Run plan B's setup() over a scripted part.

    Returns:
        (the driver after it, setup's answer, the bus with its transcript).
    """
    bus = register_bus.Bus(part)
    driver = _driver(interrupt_wired)
    originals = i2cbus.bind, databoard.Databoard.provide, bmi323.Pin
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: tuple(_Sink() for _ in want))
    bmi323.Pin = bus.pin_class()
    try:
        ok = register_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, databoard.Databoard.provide, bmi323.Pin = originals
    return driver, ok, bus


def _setup_case(name: str, interrupt_wired: bool, part, fails_on: str = None) -> dict:
    """
    One setup case: the transcript and setup's answer.

    Args:
        name - the case name.
        interrupt_wired - whether INT1 is declared.
        part - the scripted part.
        fails_on - the hex register address the transcript's LAST step must be a refused read of, or None: proves a
            case refused by transaction number refused the read it names.

    Returns:
        The case.
    """
    _driver_after, ok, bus = _setup(interrupt_wired, part)
    if fails_on is not None:
        last = bus.transcript[-1]
        if not (last.get('fails') and last['op'] == 'write_read' and last['write'] == fails_on):
            raise ValueError('drivers_bmi323: setup case %s did not end on a refused read of 0x%s' % (name, fails_on))
    return {'name': name, 'input': {'interrupt_wired': interrupt_wired, 'script': bus.transcript},
            'output': {'ok': ok}}


def _setups() -> dict:
    """The setup section."""
    cases = [
        _setup_case('healthy', False, _part()),
        _setup_case('the_gyro_starts_after_two_looks', False, _part(bursts=[_STARTING, _STARTING, _STARTED])),
        _setup_case('one_axis_still_starting', False, _part(bursts=[_burst((0, 0, 2048), (5, _MARKER, 5)),
                                                                     _STARTED])),
        _setup_case('the_accel_rail_is_not_a_marker', False, _part(bursts=[_burst((_MARKER, 0, 0), (1, 2, 3))])),
        _setup_case('the_gyro_never_starts', False, _part(bursts=[_STARTING])),
        _setup_case('the_dummy_byte_is_not_the_identity', False, _part(identity=0x00, dummy=_IDENTITY)),
        _setup_case('the_identity_read_naks', False, _part(refuse=(('read', bmi323._REG_CHIP_ID),))),
        _setup_case('the_soft_reset_naks', False, _part(refuse=(('write', bmi323._REG_CMD),))),
        _setup_case('the_gyro_configuration_naks', False, _part(refuse=(('write', bmi323._REG_GYR_CONF),))),
        _setup_case('the_start_up_read_naks', False, _part(refuse=(('read', bmi323._REG_ACC_DATA),)),
                    fails_on='%02x' % bmi323._REG_ACC_DATA),
        _setup_case('a_later_start_up_read_naks', False, _part(bursts=[_STARTING, _STARTED], refuse_at=(5,)),
                    fails_on='%02x' % bmi323._REG_ACC_DATA),
        _setup_case('int1_wired', True, _part(bursts=[_STARTING, _STARTED])),
        _setup_case('int1_route_naks', True, _part(refuse=(('write', bmi323._REG_INT_MAP2),))),
        # the identity, the reset, the two configurations, one start-up look, the two route writes: transaction 7
        _setup_case('int1_clearing_read_naks', True, _part(refuse_at=(7,)), fails_on='%02x' % bmi323._REG_ACC_DATA),
    ]
    return {'compare': {'kind': 'exact'},
            'note': 'setup() over a scripted part: input whether INT1 is wired and the transcript; output ok',
            'cases': cases}


def _read(accel: tuple, gyro: tuple) -> tuple:
    """plan B's _read() over a part answering this burst: (transcript, the 6-tuple)."""
    bus = register_bus.Bus(driver_bus.sequence([_burst(accel, gyro)]))
    driver = _driver(False)
    driver._bus, driver._addr, driver._buf = bus, _ADDRESS, bytearray(14)
    return bus.transcript, register_bus.run(bus, driver._read())


def _bursts() -> list:
    """The decode and rate inputs: test_bmi323.py's frames, the rails, one-count steps and a seeded sweep."""
    named = [('test_bmi323_gravity_and_a_rate_each_way', (0, 0, 2048), (1000, -1000, 0)),
             ('test_bmi323_the_rails', (32767, -32768, 0), (32767, -32768, 0)),
             ('one_count_each_way', (1, -1, 0), (1, -1, 0)),
             ('zero', (0, 0, 0), (0, 0, 0)),
             ('a_boost_at_10_g', (300, -500, 20480), (-140, 2000, -35))]
    generator = random.Random(_SEED)
    for index in range(40):
        named.append(('sweep_%02d' % index, tuple(generator.randint(-32768, 32767) for _ in range(3)),
                      tuple(generator.randint(-32768, 32767) for _ in range(3))))
    return named


def _decodes() -> dict:
    """The decode section."""
    cases = []
    for name, accel, gyro in _bursts():
        transcript, (ax, ay, az, gx, gy, gz) = _read(accel, gyro)
        cases.append({'name': name, 'input': {'script': transcript},
                      'output': {'accel_g': [ax, ay, az], 'rate_cd_s': [gx, gy, gz]}})
    return {'compare': {'kind': 'exact'},
            'note': '_read(): the accel in g (raw / 2048, exact) and the gyro in centidegrees per second (floored)',
            'cases': cases}


def _rates() -> dict:
    """The rate section."""
    cases = []
    for name, accel, gyro in _bursts():
        transcript, sample = _read(accel, gyro)
        cases.append({'name': name, 'input': {'script': transcript},
                      'output': [fixed.to_float(rate) for rate in sample[3:]]})
    return {'compare': {'kind': 'abs', 'tol': 0.01},
            'note': ("the gyro in degrees per second: plan B's to_float of its floored centidegree integer; the "
                     "port's exact 2000/32768 dps a count lies within the floor's 0.01"),
            'cases': cases}


def _rearm_case(name: str, interrupt_wired: bool, part, divergence: str = None) -> dict:
    """One rearm case: setup over a healthy part, then rearm() over `part`, and whether plan B logged a failure."""
    driver, ok, setup_bus = _setup(interrupt_wired, _part())
    if not ok:
        raise ValueError('drivers_bmi323: rearm case %s failed setup' % name)
    bus = register_bus.Bus(part)
    driver._bus = bus
    logged = []
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: logged.append(message))
    try:
        register_bus.run(bus, driver.rearm())
    finally:
        recorder.Recorder.log = original
    case = {'name': name, 'input': {'interrupt_wired': interrupt_wired, 'setup': setup_bus.transcript,
                                    'script': bus.transcript},
            'output': {'ok': not logged}}
    if divergence == 'route':
        route = [step for step in setup_bus.transcript if step['op'] == 'write' and step['bytes'][:2] in ('38', '3b')]
        clearing = setup_bus.transcript[-1]
        case['divergence'] = {
            'note': ('the port routes data-ready to INT1 again and clears it, as setup did: plan B re-armed the '
                     'sensors only, so INT1 stayed unrouted and the driver fell back to polling'),
            'script': bus.transcript + route + [clearing],
            'ok': not logged,
        }
    elif divergence == 'start_up':
        case['divergence'] = {
            'note': 'the port reports a gyro that never leaves start-up after the re-arm; plan B dropped the verdict',
            'script': bus.transcript,
            'ok': False,
        }
    return case


def _rearms() -> dict:
    """The rearm section."""
    return {'compare': {'kind': 'exact'},
            'note': ('rearm() after a healthy setup (its transcript in `setup`): the re-arm transcript and ok (plan B '
                     'logs a failure); a case with a divergence owes its script and ok instead'),
            'cases': [_rearm_case('restores_both_sensors', False, _part(bursts=[_STARTING, _STARTED])),
                      _rearm_case('a_dead_part_is_logged', False, _part(refuse=(('write', bmi323._REG_CMD),))),
                      _rearm_case('the_start_up_read_naks', False, _part(refuse=(('read', bmi323._REG_ACC_DATA),))),
                      _rearm_case('with_int1_wired', True, _part(), divergence='route'),
                      _rearm_case('the_gyro_never_starts_again', False, _part(bursts=[_STARTING]),
                                  divergence='start_up')]}


class _Waiter:
    """commons.Waiter stand-in: wait() answers the scripted wake counts in order, then ends the run."""

    def __init__(self, counts: list) -> None:
        self._counts = list(counts)

    async def wait(self, timeout_ms: int) -> int:
        if not self._counts:
            raise register_bus.Stop()
        return self._counts.pop(0)


def _wake_case(name: str, interrupt_wired: bool, counts: list) -> dict:
    """One wake scenario: plan B's run() over scripted wake counts, the verdict after each wake."""
    driver = _driver(interrupt_wired)
    driver._ready, driver._period_ms, driver._int_silent = _Waiter(counts), 10, False
    driver._int = object() if interrupt_wired else None  # what setup's Pin left: only `is not None` is read
    driver._accel = driver._rate = driver._telemetry = _Sink()
    verdicts = []
    noted = []

    def note(template: str = None, arg=None) -> None:
        if template is not None:
            noted.append(True)

    async def sample() -> tuple:
        silent = driver._int_silent if driver._int is not None else None  # inspect's interrupt_silent
        verdicts.append({'silent': silent, 'noted': bool(noted)})
        noted.clear()
        return (0.0, 0.0, 1.0, 0, 0, 0)

    driver.note, driver._read = note, sample
    register_bus.run_for(register_bus.Bus(driver_bus.sequence([])), driver.run(), end_ms=1 << 30)
    if len(verdicts) != len(counts):
        raise ValueError('drivers_bmi323: wake case %s ran %d of %d wakes' % (name, len(verdicts), len(counts)))
    return {'name': name, 'input': {'interrupt_wired': interrupt_wired, 'irq_runs': counts}, 'output': verdicts}


def _wakes() -> dict:
    """The wake section."""
    cases = [_wake_case('healthy', True, [1] * 25),
             _wake_case('dead_from_the_start', True, [0] * 25),
             _wake_case('an_edge_resumes', True, [0] * 21 + [1] + [0] * 21),
             _wake_case('no_wire_is_polled_by_design', False, [0] * 25),
             _wake_case('no_wire_and_stray_counts', False, [0, 1, 0, 2] * 6)]
    generator = random.Random(_SEED + 1)
    for index in range(4):
        cases.append(_wake_case('sweep_%02d' % index, True,
                                [0 if generator.random() < 0.9 else generator.choice((1, 2)) for _ in range(80)]))
    return {'compare': {'kind': 'exact'},
            'note': ('run() with wait() answering the scripted irq_runs: after each wake, whether the INT line is '
                     'called silent (null with no wire) and whether the note fired (once, at 20 timed-out wakes)'),
            'cases': cases}


def _probe_case(name: str, part) -> dict:
    """One probe case over a scripted part."""
    bus = register_bus.Bus(part)
    driver = _driver(False)
    driver._bus, driver._addr, driver._buf = bus, _ADDRESS, bytearray(14)
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
                      _probe_case('another_part', _part(identity=0x50)),
                      _probe_case('the_identity_naks', _part(refuse=(('read', bmi323._REG_CHIP_ID),))),
                      _probe_case('the_read_naks', _part(refuse=(('read', bmi323._REG_ACC_DATA),)))]}


def _diagnose_case(name: str, answer) -> dict:
    """One diagnose case: one identity read (three bytes), or a refusal."""
    bus = register_bus.Bus(driver_bus.sequence([answer]))
    driver = _driver(False)
    driver._bus, driver._addr = bus, _ADDRESS
    verdict = register_bus.run(bus, driver.diagnose())
    return {'name': name, 'input': {'script': bus.transcript}, 'output': verdict.split(' -- ')[0]}


def _diagnoses() -> dict:
    """The diagnose section."""
    answers = (('present', b'\x00\x00\x43'), ('another_part', b'\x00\x00\x50'),
               ('the_dummy_byte_is_not_the_identity', b'\x43\x43\x00'), ('absent', driver_bus.NAK))
    return {'compare': {'kind': 'exact'},
            'note': "diagnose() on i2c:0 at 0x69: the verdict, up to ' -- ' (the error's repr is the runtime's own)",
            'cases': [_diagnose_case(name, answer) for name, answer in answers]}


def build() -> dict:
    """
    The drivers_bmi323 fixture document.

    Returns:
        {meta, decode, rate, setup, rearm, wake, probe, diagnose}.
    """
    return {
        'meta': {'source': ('src/glider/drivers/bmi323.py, src/glider/i2cbus.py, src/glider/task.py, '
                            'src/glider/test/test_bmi323.py')},
        'decode': _decodes(),
        'rate': _rates(),
        'setup': _setups(),
        'rearm': _rearms(),
        'wake': _wakes(),
        'probe': _probes(),
        'diagnose': _diagnoses(),
    }
