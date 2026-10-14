"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_icp10111.json: plan B's ICP-10111 barometer (src/glider/drivers/icp10111.py) as the Rust port
(crates/drivers, module icp10111) must answer it. Sections, each a fixture of its own:
  * compensate     -- THE POLYNOMIAL against plan B under CPython's f64: _compensate(p_raw, t_raw) with the OTP set,
                      the triple computed fresh for each case (plan B's per-37-counts cache of it is a GC workaround,
                      not ported; `measured` says by how much it moves the answer). The plan's own case first:
                      _compensate(11441968, 27034) with OTP [211, 371, 553, 3833], test_icp10111.py's live capture
                      (~101797 Pa). Then seeded sweeps over -40..85 degrees C and 30..110 kPa, for the live OTP and
                      two neighbours. abs 0.05 Pa: a thirtieth of the part's 1.6 Pa RMS noise in this mode, ~4 mm of
                      altitude, the same budget commons.altitude_m's f32 port is held to (3.4 mm);
  * compensate_f32 -- the same cases against plan B under the BOARD's arithmetic: MicroPython on the P4 is single
                      precision, so plan B's expression is re-run op by op with every result rounded to f32
                      (_plan_b_arithmetic). That re-run is first proven to BE plan B's expression: with no rounding
                      it reproduces plan B's own _calibrate/_compensate bit for bit on every case, or the build
                      fails. Compared EXACTLY: the port's f32 must be the board's, to the bit;
  * measure        -- _measure() over a scripted part: the measure command, the 12 ms conversion, the 9-byte frame,
                      the THREE CRCs (a frame failing any is refused), and the altitude, temperature and pressure.
                      test_icp10111.py's live frame, one flipped bit per data byte, seeded frames. Each member is
                      held to its own margin (the section's `fields`): pressure abs 0.05 Pa (the polynomial's
                      budget above), temperature EXACT (-45 + 175 x raw / 2^16 is exact in f32 as in f64, and the
                      builder proves it per case) and altitude abs 0.005 m (commons.altitude_m's f32 of a pressure
                      within 0.024 Pa: 3.2 mm at worst over the fixture). A pressure budget applied to degrees and
                      metres let a temperature that drops its low bit, or an altitude from a pressure quantised to
                      0.25 Pa, pass;
  * setup          -- setup()'s transcript: the hardened first touch (a busy part waited out, a latched one given
                      the general call from the second NAK on), the product id, the OTP, the eight-conversion ground
                      zero; whether plan B re-armed the peers after a general call; the failures included. abs 0.01
                      on the ground. The one `divergence`: plan B re-armed the peers only when its setup then
                      succeeded, so a general call followed by a dead part left them in their power-on state; the
                      port reports every general call that went out;
  * run            -- THE RUN LOOP'S TIMELINE: plan B's run() over a scripted part on the shims' virtual clock, each
                      transaction and wait stamped with the millisecond it happened at, the readings it produced and
                      the moments it re-armed the peers. Healthy cycles, a wedge escalating ONCE to the general call
                      and recovering, a CRC run, a collect that NAKs, intermittent failures that never escalate, a
                      later wedge escalating again, and a part that stays dead (one general call, never a second).
                      The port's two-phase poll, driven millisecond by millisecond, must make the same transactions
                      at the same milliseconds. Times exact; readings member by member, as measure;
  * update         -- update(): a direct ground (a number, or what plan B's float() also takes: a numeral string, a
                      bool), a re-zero from a fresh altitude or its refusal. Exact. One `divergence`: a ground that
                      is not a number is refused in the port's words, where plan B answered float()'s own text,
                      which is the interpreter's (CPython's here, MicroPython's on the board);
  * diagnose       -- diagnose(): the product-id command, the masked word through commons.id_classify. Exact.

Every expected value comes from running plan B's own code over driver_bus's scripted part, except compensate_f32,
which is plan B's expression evaluated in single precision as the board does (and proven to be that expression).

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random
import struct

import commons
import config_default
import databoard
import driver_bus
import i2cbus
import recorder
import register_bus
import shims
from drivers import icp10111

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x63
_OTP: tuple = (211, 371, 553, 3833)  # test_icp10111.py's OTP, captured from the wired sensor
_LIVE: bytes = b'\xae\x54\xa9\x3d\x00\x73\x6e\x7a\x9a'  # test_icp10111.py's live frame
_PERIOD_MS: int = 100  # plan B's default period
_TOLERANCE_PA: float = 0.05  # the pressure, in compensate, measure and run: see the module notes
_TOLERANCE_ALTITUDE_M: float = 0.005  # the altitude, in measure and run: 3.2 mm measured at worst
_READING_FIELDS: dict = {'temperature': {'kind': 'exact'},  # measure and run: each member to its own margin
                         'altitude': {'kind': 'abs', 'tol': _TOLERANCE_ALTITUDE_M}}
_COLD_RAW: int = 1872  # t_raw at -40 degrees C: (T + 45) x 65536 / 175
_HOT_RAW: int = 48684  # t_raw at 85 degrees C


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


def _f32(value: float) -> float:
    """`value` rounded to the nearest IEEE single, widened back exactly: one MicroPython float result on the P4."""
    return struct.unpack('<f', struct.pack('<f', value))[0]


def _exact(value: float) -> float:
    """No rounding: the re-run in CPython's own f64, to prove it is plan B's expression."""
    return value


def _single(name: str, temperature: float) -> float:
    """
    Plan B's temperature, proven exact in single precision before the fixture asks the port for it exactly:
    175 x raw / 2^16 needs at most 24 significant bits, so f32 and f64 agree to the bit.

    Args:
        name - the case, for the refusal.
        temperature - plan B's temperature, degrees C.

    Returns:
        The temperature, unchanged.
    """
    if _f32(temperature) != temperature:
        raise ValueError('drivers_icp10111: %s: temperature %r is not exact in f32' % (name, temperature))
    return temperature


def _plan_b_arithmetic(otp: tuple, p_raw: int, t_raw: int, rounding) -> float:
    """
    icp10111._calibrate and the last line of _compensate, operation by operation in plan B's order, every result
    passed through `rounding`.

    With _exact this is CPython's evaluation of plan B's expression (the builder checks it equals plan B's own call
    bit for bit); with _f32 it is the board's, where each operation rounds to single precision. An int meeting a
    float converts first (float(t * t), float(c0), float(p_raw)), as both interpreters do; plan B's constants are
    exact in single precision (powers of two, small integers).

    Args:
        otp - the four OTP constants.
        p_raw - the raw pressure.
        t_raw - the raw temperature.
        rounding - _exact or _f32.

    Returns:
        The pressure in Pa.
    """
    r = rounding
    c0, c1, c2, c3 = (r(float(constant)) for constant in otp)
    t = t_raw - 32768
    quadratic = r(r(float(t * t)) * r(icp10111._QUADR))
    s1 = r(r(icp10111._LUT_LOWER) + r(c0 * quadratic))
    s2 = r(r(r(icp10111._OFFSET) * c3) + r(c1 * quadratic))
    s3 = r(r(icp10111._LUT_UPPER) + r(c2 * quadratic))
    p0, p1, p2 = (r(pressure) for pressure in icp10111._PCAL)
    numerator = r(r(r(r(s1 * s2) * r(p0 - p1)) + r(r(s2 * s3) * r(p1 - p2))) + r(r(s3 * s1) * r(p2 - p0)))
    denominator = r(r(r(s3 * r(p0 - p1)) + r(s1 * r(p1 - p2))) + r(s2 * r(p2 - p0)))
    c = r(numerator / denominator)
    a = r(r(r(r(p0 * s1) - r(p1 * s2)) - r(r(p1 - p0) * c)) / r(s1 - s2))
    b = r(r(p0 - a) * r(s1 + c))
    return r(a + r(b / r(c + r(float(p_raw)))))


def _plan_b(otp: tuple, p_raw: int, t_raw: int) -> float:
    """plan B's own _compensate, the triple computed fresh (its cache cleared first)."""
    driver = icp10111.Icp10111('baro_icp10111', {}, _Controller())
    driver._otp = list(otp)
    driver._calib = None
    return driver._compensate(p_raw, t_raw)


def _cases() -> list:
    """The polynomial's inputs: the plan's case, the range's corners, seeded sweeps for three OTP sets."""
    cases = [('the_plan_s_case_live_otp', _OTP, 11441968, 27034)]
    for name, t_raw in (('cold', _COLD_RAW), ('hot', _HOT_RAW), ('centre', 32768)):
        for p_raw in (7000000, 11441968, 13500000):
            cases.append(('%s_%d' % (name, p_raw), _OTP, p_raw, t_raw))
    generator = random.Random(_SEED)
    otps = [_OTP, tuple(constant + generator.randint(-40, 40) for constant in _OTP),
            tuple(constant + generator.randint(-40, 40) for constant in _OTP)]
    for set_index, otp in enumerate(otps):
        drawn = 0
        while drawn < 40:
            t_raw, p_raw = generator.randint(_COLD_RAW, _HOT_RAW), generator.randint(6000000, 14000000)
            if not 30000.0 <= _plan_b(otp, p_raw, t_raw) <= 110000.0:
                continue
            cases.append(('otp_%d_sweep_%02d' % (set_index, drawn), otp, p_raw, t_raw))
            drawn += 1
    return cases


def _compensations() -> tuple:
    """The compensate and compensate_f32 sections, and what was measured over them."""
    f64_cases, f32_cases = [], []
    worst = 0.0
    for name, otp, p_raw, t_raw in _cases():
        plan_b = _plan_b(otp, p_raw, t_raw)
        if _plan_b_arithmetic(otp, p_raw, t_raw, _exact) != plan_b:
            raise ValueError('drivers_icp10111: the re-run is not plan B\'s expression at %s' % name)
        board = _plan_b_arithmetic(otp, p_raw, t_raw, _f32)
        worst = max(worst, abs(board - plan_b))
        case_input = {'otp': list(otp), 'p_raw': p_raw, 't_raw': t_raw}
        f64_cases.append({'name': name, 'input': case_input, 'output': plan_b})
        f32_cases.append({'name': name, 'input': case_input, 'output': board})
    if worst > _TOLERANCE_PA:
        raise ValueError('drivers_icp10111: single precision strays %.4f Pa from f64 (budget %.2f)'
                         % (worst, _TOLERANCE_PA))
    return ({'compare': {'kind': 'abs', 'tol': _TOLERANCE_PA},
             'note': ("plan B's _compensate under CPython's f64, the triple fresh per case; Pa, abs 0.05 (a thirtieth "
                      "of the 1.6 Pa RMS noise, ~4 mm)"),
             'cases': f64_cases},
            {'compare': {'kind': 'exact'},
             'note': ("plan B's expression evaluated in single precision op by op, as MicroPython on the P4 does "
                      "(proven to be plan B's expression: unrounded it equals plan B's own call bit for bit); Pa, "
                      "exact"),
             'cases': f32_cases},
            worst)


def _cache_effect() -> float:
    """
    How far plan B's cached triple moves the answer: the pressure from a triple computed 37 raw counts (its recompute
    threshold, ~0.1 degrees C) away, against a fresh one, worst over the range. What the port gives up by not
    porting the cache.
    """
    worst = 0.0
    for t_raw in range(_COLD_RAW, _HOT_RAW - 37, 211):
        driver = icp10111.Icp10111('baro_icp10111', {}, _Controller())
        driver._otp = list(_OTP)
        driver._calib = None
        driver._compensate(7000000, t_raw)  # warm the cache at t_raw
        for p_raw in range(6000000, 14000001, 400000):
            cached = driver._compensate(p_raw, t_raw + 37)  # within the threshold: the cache is reused
            worst = max(worst, abs(cached - _plan_b(_OTP, p_raw, t_raw + 37)))
    return worst


def _word(value: int) -> bytes:
    """A 16-bit word answer and its CRC."""
    high, low = (value >> 8) & 0xFF, value & 0xFF
    return bytes((high, low, commons.sensirion_crc8(high, low)))


def _frame(p_raw: int, t_raw: int) -> bytes:
    """A measurement frame with good CRCs: pressure [0, 1] + CRC, pressure [3] + a pad + CRC, temperature + CRC."""
    high, middle, low = (p_raw >> 16) & 0xFF, (p_raw >> 8) & 0xFF, p_raw & 0xFF
    return _word((high << 8) | middle) + _word(low << 8) + _word(t_raw)


def _part(frames: list = None, identity: int = 0x0848, refuse: list = ()):
    """
    A scripted ICP-10111, command-based: the id command then a 3-byte read answers `identity`; each OTP read the next
    live OTP word; each 9-byte read the next of `frames` (the last repeats; default the live conversion). `refuse` is
    a list of transaction numbers (counting every transaction from 0) the part NAKs.
    """
    queue = list(frames or [_frame(11441968, 27034)])
    otp_words = []
    state = {'count': -1, 'last': None}

    def part(op: str, address: int, register, count: int, data: bytes):
        state['count'] += 1
        if state['count'] in refuse:
            raise driver_bus.nak()
        if op == 'write':
            state['last'] = bytes(data)
            if bytes(data) == icp10111._CMD_OTP_UNLOCK:
                otp_words[:] = list(_OTP)
            return None
        if count == 3 and state['last'] == icp10111._CMD_OTP_READ:
            return _word(otp_words.pop(0) & 0xFFFF)
        if count == 3:
            return _word(identity)
        return queue.pop(0) if len(queue) > 1 else queue[0]

    return part


def _setup(part) -> tuple:
    """
    Run plan B's setup() over a scripted part.

    Returns:
        (the driver after it, setup's answer, the bus with its transcript, whether plan B re-armed the peers).
    """
    bus = register_bus.Bus(part)
    driver = icp10111.Icp10111('baro_icp10111', {'bus': 'i2c', 'id': 1, 'addr': _ADDRESS}, _Controller())
    rearmed = []

    async def rearm_peers() -> None:
        rearmed.append(bus.clock_ms)

    driver._rearm_peers = rearm_peers
    originals = i2cbus.bind, databoard.Databoard.provide
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: tuple(_Sink() for _ in want))
    try:
        ok = register_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, databoard.Databoard.provide = originals
    return driver, ok, bus, bool(rearmed)


def _setup_case(name: str, part, divergence: bool = False) -> dict:
    """One setup case: the transcript, setup's answer, the ground, and whether the peers were re-armed."""
    driver, ok, bus, rearmed = _setup(part)
    case = {'name': name, 'input': {'script': bus.transcript},
            'output': {'ok': ok, 'ground': driver._ground if ok else None, 'peers_rearmed': rearmed}}
    if divergence:
        general_calls = [step for step in bus.transcript if step['op'] == 'write' and step['address'] == 0]
        if not general_calls or rearmed:
            raise ValueError('drivers_icp10111: setup case %s is no divergence' % name)
        case['divergence'] = {
            'note': ('a general call went out and plan B re-armed no peer, because its setup then failed; the port '
                     'reports the general call (take_general_call) whatever setup answers'),
            'output': dict(case['output'], peers_rearmed=True),
        }
    return case


def _setups() -> dict:
    """The setup section; transaction numbers count every transaction from 0."""
    cases = [
        _setup_case('a_clean_boot', _part()),
        _setup_case('a_busy_part_is_waited_out', _part(refuse=[0])),
        _setup_case('a_latched_part_gets_the_general_call', _part(refuse=[0, 1, 2])),
        _setup_case('a_latched_part_whose_read_naks', _part(refuse=[0, 2])),
        _setup_case('a_dead_part', _part(refuse=list(range(9))), divergence=True),
        _setup_case('another_part', _part(identity=0x0849)),
        _setup_case('the_otp_unlock_naks', _part(refuse=[4])),
        _setup_case('half_recovered_the_first_measure_naks', _part(refuse=[13])),
        _setup_case('a_ground_conversion_fails_its_crc', _part(frames=[_frame(11441968, 27034)] * 3 + [
            bytes((0xAF,)) + _frame(11441968, 27034)[1:]])),
        _setup_case('a_drifting_pad', _part(frames=[_frame(11441968 + 37 * index, 27034 + 5 * index)
                                                    for index in range(8)])),
    ]
    return {'compare': {'kind': 'abs', 'tol': 0.01},
            'note': ('setup() over a scripted part: the transcript; output ok, the ground zero (m) and whether plan B '
                     're-armed the peers; a case with a divergence owes its output instead'),
            'cases': cases}


def _measure_case(name: str, frame: bytes) -> dict:
    """One measure case: plan B's _measure() over a part answering this frame."""
    bus = register_bus.Bus(_part(frames=[frame]))
    driver = icp10111.Icp10111('baro_icp10111', {}, _Controller())
    driver._bus, driver._addr, driver._otp = bus, _ADDRESS, list(_OTP)
    try:
        altitude, temperature, pressure = register_bus.run(bus, driver._measure())
        output = {'altitude': altitude, 'temperature': _single(name, temperature), 'pressure': pressure}
    except ValueError as error:
        output = {'refused': str(error)}
    return {'name': name, 'input': {'script': bus.transcript}, 'output': output}


def _measures() -> dict:
    """The measure section."""
    cases = [_measure_case('the_live_frame', _LIVE), _measure_case('the_plan_s_raw_words', _frame(11441968, 27034))]
    for byte in (0, 1, 2, 3, 4, 5, 6, 7, 8):
        corrupt = bytearray(_LIVE)
        corrupt[byte] ^= 0x01
        cases.append(_measure_case('one_flipped_bit_in_byte_%d' % byte, bytes(corrupt)))
    generator = random.Random(_SEED + 1)
    for index in range(30):
        cases.append(_measure_case('sweep_%02d' % index, _frame(generator.randint(7000000, 13500000),
                                                                generator.randint(_COLD_RAW, _HOT_RAW))))
    return {'compare': {'kind': 'abs', 'tol': _TOLERANCE_PA},
            'fields': _READING_FIELDS,
            'note': ('_measure(): the command, the 12 ms conversion, the frame; altitude m, temperature degrees C, '
                     'pressure Pa (the triple fresh), or the refusal of a frame failing a CRC; pressure abs 0.05, '
                     'temperature exact, altitude abs 0.005'),
            'cases': cases}


def _run_case(name: str, verdicts: list, end_ms: int) -> dict:
    """
    One run-loop scenario: plan B's run() after a clean setup, over a part whose conversions go by `verdicts`.

    Args:
        name - the case name.
        verdicts - one per measurement cycle: 'ok', 'nak' (the measure command NAKs), 'collect' (the frame read
            NAKs) or 'crc' (the frame fails a CRC); the last repeats.
        end_ms - how long to run, ms of virtual time.

    Returns:
        The case.
    """
    driver, ok, setup_bus, _rearmed = _setup(_part())
    if not ok:
        raise ValueError('drivers_icp10111: run case %s failed setup' % name)
    """
    A period between setup and the run loop. Without it plan B's first pass is served from _read()'s sample cache
    (setup's last ground-zero conversion is younger than a period), published with no bus traffic: the conversation
    guard and cache are a MicroPython workaround the port does not carry, so the timeline starts with a measurement.
    """
    shims.advance(_PERIOD_MS)
    queue = list(verdicts)
    state = {'verdict': None}
    good, crc = _frame(11441968, 27034), bytes((0xAF,)) + _frame(11441968, 27034)[1:]

    def part(op: str, address: int, register, count: int, data: bytes):
        if op == 'write' and address == 0:
            return None  # the general call: something on the bus honours it
        if op == 'write':
            state['verdict'] = queue.pop(0) if len(queue) > 1 else queue[0]
            if state['verdict'] == 'nak':
                raise driver_bus.nak()
            return None
        if state['verdict'] == 'collect':
            raise driver_bus.nak()
        return crc if state['verdict'] == 'crc' else good

    bus = register_bus.Bus(part)
    driver._bus = bus
    readings, rearms = [], []
    driver._altitude = driver._temperature = driver._pressure = driver._elevation = _Sink()

    class _Telemetry:
        def push(self, row: tuple) -> None:
            altitude, temperature, pressure, _elevation = row
            readings.append({'at_ms': bus.clock_ms, 'altitude': altitude, 'temperature': _single(name, temperature),
                             'pressure': pressure})

    async def rearm_peers() -> None:
        rearms.append(bus.clock_ms)

    driver._telemetry, driver._rearm_peers = _Telemetry(), rearm_peers
    driver.note = lambda template=None, arg=None: None
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    try:
        register_bus.run_for(bus, driver.run(), end_ms)
    finally:
        recorder.Recorder.log = original
    return {'name': name, 'input': {'setup': setup_bus.transcript, 'script': bus.transcript, 'end_ms': end_ms},
            'output': {'readings': readings, 'rearms': rearms}}


def _runs() -> dict:
    """The run section."""
    return {'compare': {'kind': 'abs', 'tol': _TOLERANCE_PA},
            'fields': _READING_FIELDS,
            'note': ("run() after a clean setup on the virtual clock: input the stamped transcript (at_ms) and the "
                     "end; output the readings (at_ms, altitude, temperature, pressure) and the re-arm moments. The "
                     "port's poll, ticked every millisecond, must make each transaction at plan B's millisecond. A "
                     "period passes between setup and the run, so _read()'s sample cache (not ported) serves "
                     "nothing. Pressure abs 0.05, temperature exact, altitude abs 0.005, times exact"),
            'cases': [_run_case('healthy_cycles', ['ok'], 500),
                      _run_case('a_wedge_escalates_once_and_recovers', ['nak', 'nak', 'nak', 'ok'], 900),
                      _run_case('a_crc_run_escalates', ['crc', 'crc', 'crc', 'ok'], 900),
                      _run_case('a_collect_that_naks_escalates', ['ok', 'collect', 'collect', 'collect', 'ok'], 1000),
                      _run_case('intermittent_failures_never_escalate', ['nak', 'ok', 'nak', 'nak', 'ok', 'crc', 'ok',
                                                                         'collect', 'nak', 'ok'], 1300),
                      _run_case('a_later_wedge_escalates_again', ['nak', 'nak', 'nak', 'ok', 'ok', 'nak', 'nak', 'nak',
                                                                  'ok'], 1700),
                      _run_case('a_part_that_stays_dead_is_called_once', ['nak'], 1500)]}


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
    driver = icp10111.Icp10111('baro_icp10111', {}, _Controller())
    driver._period_ms, driver._ground, driver._altitude = _PERIOD_MS, 7.0, _Altitude(fresh)
    try:
        answer = {'changed': driver.update(properties), 'ground': driver._ground}
    except ValueError as error:
        answer = {'refused': str(error), 'ground': driver._ground}
    case = {'name': name, 'input': {'properties': properties, 'fresh_altitude': fresh}, 'output': answer}
    if owed_refusal is not None:
        if 'refused' not in answer:
            raise ValueError('drivers_icp10111: update case %s is no refusal' % name)
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
                      _update_case('a_falsy_re_zero', {'rezero': 0}, 123.5),
                      _update_case('the_ground_wins_over_a_re_zero', {'ground': 3.0, 'rezero': True}, 9.0),
                      _update_case('a_numeral_string_ground', {'ground': '12.5'}, None),
                      _update_case('a_bool_ground', {'ground': True}, None),
                      _update_case('a_word_ground_is_refused', {'ground': 'high'}, None,
                                   owed_refusal='ground must be a number')]}


def _diagnose_case(name: str, answers: list) -> dict:
    """One diagnose case: the id command and its read answered (or refused) as listed."""
    bus = register_bus.Bus(driver_bus.sequence(answers))
    driver = icp10111.Icp10111('baro_icp10111', {}, _Controller())
    driver._bus, driver._addr = bus, _ADDRESS
    verdict = register_bus.run(bus, driver.diagnose())
    return {'name': name, 'input': {'script': bus.transcript}, 'output': verdict}


def _diagnoses() -> dict:
    """The diagnose section."""
    return {'compare': {'kind': 'exact'}, 'note': 'diagnose(): the id command, the masked word; the verdict text',
            'cases': [_diagnose_case('present', [None, _word(0x0848)]),
                      _diagnose_case('zeros', [None, _word(0x0000)]),
                      _diagnose_case('another_part', [None, _word(0x0849)]),
                      _diagnose_case('the_command_naks', [driver_bus.NAK]),
                      _diagnose_case('the_read_naks', [None, driver_bus.NAK])]}


def build() -> dict:
    """
    The drivers_icp10111 fixture document.

    Returns:
        {meta, measured, compensate, compensate_f32, measure, setup, run, update, diagnose}.
    """
    compensate, compensate_f32, worst = _compensations()
    return {
        'meta': {'source': ('src/glider/drivers/icp10111.py, src/glider/commons.py, src/glider/i2cbus.py, '
                            'src/glider/task.py, src/glider/test/test_icp10111.py')},
        'measured': {'worst_f32_against_f64_pa': round(worst, 5), 'worst_cache_effect_pa': round(_cache_effect(), 5)},
        'compensate': compensate,
        'compensate_f32': compensate_f32,
        'measure': _measures(),
        'setup': _setups(),
        'run': _runs(),
        'update': _updates(),
        'diagnose': _diagnoses(),
    }
