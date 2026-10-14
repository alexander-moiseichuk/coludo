"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_bmm350.json: plan B's BMM350 magnetometer (src/glider/drivers/bmm350.py) as the Rust port
(crates/drivers, module bmm350) must answer it. Every section compares EXACTLY: the part's unit is the count, and
plan B's calibration is integer arithmetic on counts (floored divisions included), as the port's is. Sections:
  * decode      -- _read() over a scripted part, uncalibrated: the 14-byte burst (TWO DUMMY BYTES, then x, y, z as
                   24-bit two's complement, little-endian) and the raw counts it publishes;
  * calibration -- the OPERATOR CALIBRATION, scenario by scenario as test_bmm350.py runs them: the raw samples fed
                   through _read() (the evidence: extrema, octant coverage around a moving centre), then calibrate()
                   (a partial turn, a still board and a dead part REFUSED, an ellipse and a full circle ACCEPTED, a
                   refused re-calibration keeping the old one), then more samples through the calibration in force.
                   Per sample: what the channel publishes and how many octants are covered; at the end the samples
                   seen, the extrema, the instruction before and after, calibrate()'s answer and the calibration in
                   force. Plus seeded random walks;
  * nvs         -- the calibration blob: calibrate() persisting through an NVS stand-in (four i32, little-endian,
                   key bmm350_cal, shared with the Rust board) and _restore() reading one back (a blob of another
                   length leaves the axes raw);
  * setup       -- setup()'s transcript: the identity PAST the dummy bytes, then the configuration table -- the soft
                   reset, the MANDATORY OTP power-off, the rate and its apply, normal mode -- with its waits. Every
                   case whose transcript reaches the rate write carries a `divergence`, the transcript the port owes
                   instead: plan B's with that one byte changed, PMU_CMD_AGGR_SET 0x24 (100 Hz, averaging 4) -> 0x37
                   (12.5 Hz, averaging 8; avionics-port.md M9 D2), every other step and the answer plan B's;
  * rearm       -- rearm()'s transcript (the table again) and whether plan B logged a failure; the same one-byte
                   `divergence` where the re-arm reaches the rate write;
  * probe       -- probe()'s transcript, the step that failed, and the not-measuring reason (all axes zero; the raw
                   0x7F7F7F of a part whose OTP is still powered);
  * diagnose    -- diagnose()'s verdict, up to its ' -- ' (the error's repr is the runtime's own).

Every value comes from running plan B's own code: the builder computes nothing the port is checked on. The sample
streams are test_bmm350.py's (its _turn sweeps, its still-board scatter) and seeded walks; the port is fed the very
integers written here.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import math
import random

import config_default
import databoard
import driver_bus
import i2cbus
import recorder
import register_bus
from drivers import bmm350

_SEED: int = 20261006  # the walks' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x15
_IDENTITY: int = 0x33
_HARD_IRON: tuple = (7500, -4500)  # test_bmm350._turn's centre
_RADIUS: tuple = (4500, 4500)  # test_bmm350._turn's radii


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


def _frame(x: int, y: int, z: int) -> bytes:
    """A 14-byte burst: two dummy bytes (not zero), then x, y, z and a temperature, 24-bit little-endian."""
    return b'\xa5\x5a' + b''.join((value & 0xFFFFFF).to_bytes(3, 'little') for value in (x, y, z, 0x123456))


class _FieldBus:
    """test_bmm350._FakeBus: answers each burst read with the field set in `field`."""

    def __init__(self) -> None:
        self.field: tuple = (0, 0, 0)

    async def read_into(self, address: int, register: int, buffer) -> None:
        buffer[:] = _frame(*self.field)


def _driver(calibration=None) -> bmm350.Bmm350:
    """A driver with no evidence, wired to a field bus, NVS suppressed, a calibration restored or not."""
    driver = bmm350.Bmm350('mag_bmm350', {'bus': 'i2c', 'id': 0, 'addr': _ADDRESS}, _Controller())
    driver._buf, driver._addr, driver._bus = bytearray(14), _ADDRESS, _FieldBus()
    driver._span, driver._sectors, driver._centre, driver._seen = None, 0, None, 0
    driver._calibration = calibration
    return driver


def _turn(degrees: int, centre: tuple = _HARD_IRON, radius: tuple = _RADIUS, step: int = 5) -> list:
    """test_bmm350._turn's samples: `degrees` of a circle about a hard-iron centre, z fixed at 700."""
    samples = []
    for angle in range(0, degrees, step):
        radians = math.radians(angle)
        samples.append([int(centre[0] + radius[0] * math.cos(radians)), int(centre[1] + radius[1] * math.sin(radians)),
                        700])
    return samples


def _scenario(name: str, samples: list, then: list = (), restored: tuple = None) -> dict:
    """
    One calibration scenario through plan B: the samples, calibrate(), then more samples.

    Args:
        name - the case name.
        samples - the raw (x, y, z) fed before calibrate().
        then - the raw (x, y, z) fed after it, through whatever calibration is then in force.
        restored - a calibration restored from NVS before the first sample, or None.

    Returns:
        The case.
    """
    driver = _driver(restored)
    original_nvs, original_log = bmm350._nvs, recorder.Recorder.log
    bmm350._nvs = None
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    published, covered = [], []
    try:
        for sample in samples:
            driver._bus.field = tuple(sample)
            published.append(list(register_bus.run(register_bus.Bus(driver_bus.sequence([])), driver._read())))
            covered.append(driver._covered())
        before = driver.calibration()
        answer = register_bus.run(register_bus.Bus(driver_bus.sequence([])), driver.calibrate())
        after = driver.calibration()
        corrected = []
        for sample in then:
            driver._bus.field = tuple(sample)
            corrected.append(list(register_bus.run(register_bus.Bus(driver_bus.sequence([])), driver._read())))
    finally:
        bmm350._nvs, recorder.Recorder.log = original_nvs, original_log
    return {'name': name,
            'input': {'restored': list(restored) if restored else None, 'samples': samples, 'then': list(then)},
            'output': {'published': published, 'covered': covered, 'seen': driver._seen,
                       'span': list(driver._span) if driver._span else None, 'instruction_before': before,
                       'answer': answer, 'calibration': list(driver._calibration) if driver._calibration else None,
                       'instruction_after': after, 'corrected': corrected}}


def _headings() -> list:
    """test_bmm350's heading probes after a calibration: seven angles around the hard-iron circle."""
    samples = []
    for degrees in (0, 37, 90, 154, 180, 271, 330):
        radians = math.radians(degrees)
        samples.append([int(7500 + 4500 * math.cos(radians)), int(-4500 + 4500 * math.sin(radians)), 700])
    return samples


def _calibrations() -> dict:
    """The calibration section."""
    still = [[13700 + (index * 37) % 201 - 100, 16780 + (index * 53) % 201 - 100, 700] for index in range(400)]
    quarter = _turn(90, step=1) * 3
    partial = [[-4000, -4000, 0], [4000, -4000, 0], [4000, 4000, 0], [-4000, 4000, 0], [4000, 3000, 0],
               [-3900, 3990, 0]] + [[4000, 3000, 0]] * 200
    cases = [
        _scenario('nothing_seen', []),
        _scenario('a_quarter_turn_looks_round_and_is_refused', quarter),
        _scenario('a_still_board_covers_nothing', still),
        _scenario('a_dead_part', [[0, 0, 700]] * 200),
        _scenario('an_ellipse_is_corrected_not_refused', _turn(360, radius=(6000, 3000), step=2) * 2,
                  then=_turn(360, radius=(6000, 3000), step=45)),
        _scenario('a_full_circle_calibrates', _turn(360, step=2) * 2, then=_headings()),
        _scenario('a_refused_recalibration_keeps_the_old_one', partial, then=[[1000, -1000, 5]],
                  restored=(0, 0, 100, 100)),
        _scenario('too_few_samples_on_a_restored_calibration', [[1, 1, 1]] * 5, restored=(0, 0, 100, 100)),
        _scenario('a_turn_one_short_of_the_sample_floor', _turn(360, step=3)[:99]),
        _scenario('the_negative_half_plane', [[-x, -y, -z] for x, y, z in _turn(360, step=2)] * 2,
                  then=[[-7500, 4500, -700], [-12000, 4500, 0]]),
    ]
    generator = random.Random(_SEED)
    for index in range(6):
        x, y = generator.randint(-20000, 20000), generator.randint(-20000, 20000)
        walk = []
        for _ in range(generator.choice((150, 300, 600))):
            x = max(-(1 << 23), min((1 << 23) - 1, x + generator.randint(-900, 900)))
            y = max(-(1 << 23), min((1 << 23) - 1, y + generator.randint(-900, 900)))
            walk.append([x, y, generator.randint(-2000, 2000)])
        cases.append(_scenario('walk_%02d' % index, walk, then=walk[:20]))
    for index in range(4):
        centre = (generator.randint(-30000, 30000), generator.randint(-30000, 30000))
        radius = (generator.randint(2500, 12000), generator.randint(2500, 12000))
        laps = _turn(360, centre=centre, radius=radius, step=generator.choice((1, 2, 3))) * 2
        cases.append(_scenario('circle_%02d' % index, laps, then=_turn(360, centre=centre, radius=radius, step=30)))
    return {'compare': {'kind': 'exact'},
            'note': ('raw samples through _read(), calibrate(), more samples: per sample the published (x, y, z) and '
                     'the octants covered; then the samples seen, the extrema, the instructions, the answer (null '
                     'when calibrated), the calibration in force, and the published values after it'),
            'cases': cases}


class _Nvs:
    """An esp32.NVS stand-in for the calibration key: one blob, recorded on set, answered on get."""

    def __init__(self, blob: bytes = None) -> None:
        self.blob = blob
        self.committed = False

    def set_blob(self, key: str, value: bytes) -> None:
        self.blob = bytes(value)

    def commit(self) -> None:
        self.committed = True

    def get_blob(self, key: str, buffer) -> int:
        if self.blob is None:
            raise OSError(-0x1102, 'ESP_ERR_NVS_NOT_FOUND')
        if len(self.blob) > len(buffer):
            raise OSError(-0x1107, 'ESP_ERR_NVS_INVALID_LENGTH')
        buffer[:len(self.blob)] = self.blob
        return len(self.blob)


def _nvs_cases() -> dict:
    """The nvs section: a persist, and restores of blobs of three lengths."""
    cases = []
    for name, samples in (('a_full_circle_persisted', _turn(360, step=2) * 2),
                          ('an_offset_circle_persisted', _turn(360, centre=(-123456, 98765), radius=(9000, 7000),
                                                               step=2) * 2)):
        driver = _driver()
        store = _Nvs()
        original_nvs, original_log = bmm350._nvs, recorder.Recorder.log
        bmm350._nvs = store
        recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
        try:
            for sample in samples:
                driver._bus.field = tuple(sample)
                register_bus.run(register_bus.Bus(driver_bus.sequence([])), driver._read())
            answer = register_bus.run(register_bus.Bus(driver_bus.sequence([])), driver.calibrate())
        finally:
            bmm350._nvs, recorder.Recorder.log = original_nvs, original_log
        if answer is not None or not store.committed:
            raise ValueError('drivers_bmm350: nvs case %s did not persist (%r)' % (name, answer))
        cases.append({'name': name, 'input': {'calibration': list(driver._calibration)},
                      'output': {'blob': store.blob.hex()}})
    for name, blob in (('a_16_byte_blob_restores', bytes.fromhex('4c1d00006ceeffff9411000094110000')),
                       ('a_corrupt_radius_restores_as_stored', bytes.fromhex('4c1d00006ceeffff94110000941100ff')),
                       ('a_15_byte_blob_is_not_a_calibration', bytes(15)),
                       ('a_17_byte_blob_is_not_a_calibration', bytes(17))):
        driver = _driver()
        original_nvs = bmm350._nvs
        bmm350._nvs = _Nvs(blob)
        try:
            driver._restore()
        finally:
            bmm350._nvs = original_nvs
        cases.append({'name': name, 'input': {'blob': blob.hex()},
                      'output': {'calibration': list(driver._calibration) if driver._calibration else None}})
    return {'compare': {'kind': 'exact'},
            'note': ('calibrate() persisting through an NVS stand-in (the 16-byte blob, hex), and _restore() from a '
                     'stored blob (the calibration it puts in force, null for any other length)'),
            'cases': cases}


def _part(identity: int = _IDENTITY, field: tuple = (1200, -300, 700), refuse: tuple = (), dummy: int = 0x00):
    """
    A scripted BMM350: the identity read answers two `dummy` bytes then `identity`; a burst read answers `field`; a
    write acks. A transaction on a register listed in `refuse` (as ('read' | 'write', register)) NAKs.
    """

    def part(op: str, address: int, register, count: int, data: bytes):
        kind = 'read' if op == 'write_read' else 'write'
        if (kind, register) in refuse:
            raise driver_bus.nak()
        if kind == 'write':
            return None
        if register == bmm350._REG_CHIP_ID:
            return bytes([dummy, dummy, identity])
        return _frame(*field)

    return part


def _decodes() -> dict:
    """The decode section."""
    fields = [('test_bmm350_both_signs', (-12345, 6789, 700)), ('the_rails', (-(1 << 23), (1 << 23) - 1, -1)),
              ('zero', (0, 0, 0)), ('the_otp_powered_pattern', (0x7F7F7F, 0x7F7F7F, 0x7F7F7F))]
    generator = random.Random(_SEED + 1)
    fields += [('sweep_%02d' % index, tuple(generator.randint(-(1 << 23), (1 << 23) - 1) for _ in range(3)))
               for index in range(40)]
    cases = []
    for name, field in fields:
        bus = register_bus.Bus(_part(field=field))
        driver = _driver()
        driver._bus = bus
        published = register_bus.run(bus, driver._read())
        cases.append({'name': name, 'input': {'script': bus.transcript}, 'output': list(published)})
    return {'compare': {'kind': 'exact'},
            'note': '_read() uncalibrated over a scripted part: the raw (x, y, z) counts it publishes',
            'cases': cases}


_RATE_PORT: int = 0x37  # PMU_CMD_AGGR_SET as the port writes it: 12.5 Hz (bits 3:0 = 7), averaging 8 (bits 5:4 = 3)


def _rate_divergence(case: dict) -> dict:
    """
    Pin the port's one rate byte on a setup or rearm case: when plan B's transcript writes PMU_CMD_AGGR_SET, the case
    gains a `divergence` owing that transcript with the write's value 0x24 -> 0x37, and plan B's answer.

    Args:
        case - the case, its plan B transcript in input.script and its answer in output.ok.

    Returns:
        The case, with its divergence when the transcript reaches the rate write.
    """
    plan_b = '%02x%02x' % (bmm350._REG_PMU_CMD_AGGR_SET, bmm350._AGGR_100HZ_AVG4)
    script = case['input']['script']
    rate = [step for step in script if step['op'] == 'write' and step['bytes'] == plan_b]
    if not rate:
        return case
    if len(rate) != 1:
        raise ValueError('drivers_bmm350: case %s writes the rate %d times' % (case['name'], len(rate)))
    port = '%02x%02x' % (bmm350._REG_PMU_CMD_AGGR_SET, _RATE_PORT)
    owed = [dict(step, bytes=port) if step is rate[0] else step for step in script]
    case['divergence'] = {'note': ('the port runs the part at 12.5 Hz with averaging 8 (PMU_CMD_AGGR_SET 0x37, '
                                   'avionics-port.md M9 D2) where plan B ran 100 Hz with averaging 4 (0x24): the one '
                                   'byte that differs'),
                          'script': owed, 'ok': case['output']['ok']}
    return case


def _setup_case(name: str, part) -> dict:
    """One setup case: plan B's setup() over a scripted part, with the port's rate byte pinned."""
    bus = register_bus.Bus(part)
    driver = bmm350.Bmm350('mag_bmm350', {'bus': 'i2c', 'id': 0, 'addr': _ADDRESS}, _Controller())
    originals = i2cbus.bind, databoard.Databoard.provide, bmm350._nvs
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: _Sink())
    bmm350._nvs = None
    try:
        ok = register_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, databoard.Databoard.provide, bmm350._nvs = originals
    return _rate_divergence({'name': name, 'input': {'script': bus.transcript}, 'output': {'ok': ok}})


def _setups() -> dict:
    """The setup section."""
    return {'compare': {'kind': 'exact'},
            'note': ('setup() over a scripted part: the transcript (the table with its waits) and ok; a case with a '
                     'divergence owes its script and ok instead -- the rate byte 0x37 for plan B\'s 0x24'),
            'cases': [_setup_case('healthy', _part()),
                      _setup_case('the_dummy_byte_is_not_the_identity', _part(identity=0x43, dummy=_IDENTITY)),
                      _setup_case('the_identity_read_naks', _part(refuse=(('read', bmm350._REG_CHIP_ID),))),
                      _setup_case('the_otp_power_off_naks', _part(refuse=(('write', bmm350._REG_OTP_CMD),))),
                      _setup_case('normal_mode_naks', _part(refuse=(('write', bmm350._REG_PMU_CMD),)))]}


def _rearm_case(name: str, refuse: tuple = ()) -> dict:
    """
    One rearm case: rearm() over a part that refuses `refuse`, and whether plan B logged a failure, with the port's
    rate byte pinned.
    """
    bus = register_bus.Bus(_part(refuse=refuse))
    driver = _driver()
    driver._bus = bus
    logged = []
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: logged.append(message))
    try:
        register_bus.run(bus, driver.rearm())
    finally:
        recorder.Recorder.log = original
    return _rate_divergence({'name': name, 'input': {'script': bus.transcript}, 'output': {'ok': not logged}})


def _rearms() -> dict:
    """The rearm section."""
    return {'compare': {'kind': 'exact'},
            'note': ('rearm(): the transcript (the configuration table again) and ok (plan B logs a failure); a case '
                     'with a divergence owes its script and ok instead -- the rate byte 0x37 for plan B\'s 0x24'),
            'cases': [_rearm_case('the_table_again'),
                      _rearm_case('a_dead_part_is_logged', refuse=(('write', bmm350._REG_CMD),))]}


def _probe_case(name: str, part) -> dict:
    """One probe case over a scripted part: the step that failed and, for a part not measuring, why."""
    bus = register_bus.Bus(part)
    driver = _driver()
    driver._bus = bus
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    try:
        message = register_bus.run(bus, driver.probe())
    finally:
        recorder.Recorder.log = original
    failed, reason = None, None
    if message is not None:
        failed, _, detail = message.partition(': ')
        reason = detail if 'not measuring' in detail else None
    return {'name': name, 'input': {'script': bus.transcript}, 'output': {'failed': failed, 'reason': reason}}


def _probes() -> dict:
    """The probe section."""
    return {'compare': {'kind': 'exact'},
            'note': ("probe(): the step that failed ('chip id', 'read', null when it passed) and the not-measuring "
                     "reason (null for a bus or identity fault, whose text is the runtime's)"),
            'cases': [_probe_case('passes', _part()),
                      _probe_case('the_otp_still_powered', _part(field=(0x7F7F7F, 0x7F7F7F, 0x7F7F7F))),
                      _probe_case('all_axes_zero', _part(field=(0, 0, 0))),
                      _probe_case('only_x_at_the_pattern', _part(field=(0x7F7F7F, 5, 0))),
                      _probe_case('another_part', _part(identity=0x43)),
                      _probe_case('the_read_naks', _part(refuse=(('read', bmm350._REG_MAG_X),)))]}


def _diagnose_case(name: str, answer) -> dict:
    """One diagnose case: one identity read (three bytes), or a refusal."""
    bus = register_bus.Bus(driver_bus.sequence([answer]))
    driver = _driver()
    driver._bus = bus
    verdict = register_bus.run(bus, driver.diagnose())
    return {'name': name, 'input': {'script': bus.transcript}, 'output': verdict.split(' -- ')[0]}


def _diagnoses() -> dict:
    """The diagnose section."""
    answers = (('present', b'\x00\x00\x33'), ('another_part', b'\x00\x00\x43'),
               ('the_dummy_byte_is_not_the_identity', b'\x33\x33\x00'), ('absent', driver_bus.NAK))
    return {'compare': {'kind': 'exact'},
            'note': "diagnose() on i2c:0 at 0x15: the verdict, up to ' -- ' (the error's repr is the runtime's own)",
            'cases': [_diagnose_case(name, answer) for name, answer in answers]}


def build() -> dict:
    """
    The drivers_bmm350 fixture document.

    Returns:
        {meta, decode, calibration, nvs, setup, rearm, probe, diagnose}.
    """
    return {
        'meta': {'source': 'src/glider/drivers/bmm350.py, src/glider/i2cbus.py, src/glider/test/test_bmm350.py'},
        'decode': _decodes(),
        'calibration': _calibrations(),
        'nvs': _nvs_cases(),
        'setup': _setups(),
        'rearm': _rearms(),
        'probe': _probes(),
        'diagnose': _diagnoses(),
    }
