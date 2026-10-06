"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_sdp810.json: plan B's SDP810 pitot driver (src/glider/drivers/sdp810.py) as the Rust port
(crates/drivers, module sdp810) must answer it. Sections, each a fixture of its own with its own comparison:
  * crc        -- commons.sensirion_crc8 over the datasheet example (0xBEEF -> 0x92) and seeded words;
  * frames     -- _frame_ok (all THREE CRCs, the length) and _signed16 over good frames, every single-bit flip of
                  one frame, and short and long ones;
  * pressure   -- _pressure(raw) with a latched scale and a zero: the telemetry's Pa x 100 (8100 at scale 60 ->
                  13500, -600 -> -1000, 200 -> 333), and the dynamic_pressure channel in Pa, signed;
  * airspeed   -- _airspeed(_pressure(raw)) in m/s, and the telemetry's airspeed_cms (the airspeed band), with the
                  dynamic_pressure it came from;
  * tare       -- stateful: test_sdp810.py's tare ladder (refused before the first frame, 200 -> 333 persisted, a
                  direct offset not persisted, a 0 Pa tare is a tare) and calibration()'s prompt;
  * setup      -- setup() over a scripted part: its I2C transcript (stop, 3 ms, start, 20 ms, read, retried once),
                  the latched scale, and the tare NVS restores over the config's;
  * run        -- run()'s read loop over a scripted part: each pass's transcript and its row (and the two channel
                  values it pushed) or fault, the restart ladder's stop-then-start after the fifth consecutive
                  failure;
  * diagnose   -- diagnose()'s wire-level verdict.

UNITS. Plan B scales the pressure to a fixnum (Pa x 100, floored) and derives the airspeed from that; the Rust port
computes the real-number law in f32 (the pressure raw / scale - zero / 100, then sqrt(2q / rho)) and keeps the
integer units only at the telemetry and NVS boundary, where it computes them from the raw count exactly as plan B
does. So every integer here compares exactly, and the channels' Pa and m/s within _TOLERANCE, which the builder
justifies: it measures plan B against the real law (`measured`) and refuses a case past _BUDGET, and the port's f32
(emulated operation by operation: every f32 operation is the correctly rounded f64 one, rounded once more) past the
rest of the tolerance. VETTED, not fitted: plan B's floor sends a pressure under 0.01 Pa to 0 m/s where the law
gives a few cm/s, so a seeded case whose floored and real pressures straddle zero, or sit under 1 Pa off an exact
hundredth (where the root magnifies the floor), is drawn again; under 1 Pa ON an exact hundredth plan B and the law
agree, and those cases stay (raw 3 at scale 60 is 0.29 m/s). airspeed_cms is the f32 law's truncation, which the
builder requires equal to plan B's: a case where they part is drawn again.

The transcripts are I2C-level (driver_bus.py), so the Rust test replays them through its scripted mock bus; the run
loop's own period sleep (period_ms, 17 here so no wait of the driver's shares its length) is the owner's schedule in
Rust, not the driver's, and marks the end of a pass instead of being recorded.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import math
import random
import struct

import commons
import config_default
import databoard
import driver_bus
import fixed
import i2cbus
import recorder
import shims
from drivers import sdp810

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x25
_PERIOD_MS: int = 17  # the run scenarios' read period: no wait of the driver's own (3, 20) is this long
_TOLERANCE: float = 0.01  # m/s: the airspeed section's comparison
_BUDGET: float = 0.009  # the most plan B may differ from the real-number law, leaving the port's f32 its margin
_SWEEP_CASES: int = 120  # seeded airspeed and pressure cases
_DRAWS: int = 100  # a seeded case is drawn at most this many times before the build gives up


class _Controller:
    """The board the driver reads its pins and buses from: plan B's default config."""

    config: dict = config_default.default()


class _Rows:
    """A telemetry stream stand-in: keeps the rows pushed."""

    def __init__(self) -> None:
        self.rows: list = []

    def push(self, row: tuple) -> None:
        self.rows.append(list(row))


class _Channel:
    """A databoard channel stand-in: keeps the latest value, answers value() with it, and keeps every push."""

    def __init__(self, latest=None) -> None:
        self.latest = latest
        self.pushed: list = []

    def push(self, value) -> None:
        self.latest = value
        self.pushed.append(value)

    def value(self):
        return self.latest


class _Nvs:
    """esp32.NVS('coludo') as sdp810 uses it: one stored i32, or none (get_i32 raises, as for an absent key)."""

    def __init__(self, stored) -> None:
        self.stored = stored

    def get_i32(self, key: str) -> int:
        if self.stored is None:
            raise OSError(-4354, 'ESP_ERR_NVS_NOT_FOUND')
        return self.stored


def _as_f32(value: float) -> float:
    """`value` rounded to the nearest f32, widened back exactly: one f32 operation's result."""
    return struct.unpack('<f', struct.pack('<f', value))[0]


def _rust_airspeed(raw: int, scale: int, zero: int, density: float) -> tuple:
    """
    The Rust port's pressure, airspeed and airspeed_cms, emulated in f32 operation by operation (its exact source
    order: raw / scale - zero / 100, then sqrt(2 q / rho), then truncate(v x 100)).

    Returns:
        (pressure Pa, airspeed m/s, airspeed_cms) as the port computes them.
    """
    pressure = _as_f32(_as_f32(raw / scale) - _as_f32(zero / 100.0))
    if pressure > 0.0:
        airspeed = _as_f32(math.sqrt(_as_f32(_as_f32(2.0 * pressure) / _as_f32(density))))
    else:
        airspeed = 0.0
    return pressure, airspeed, int(_as_f32(airspeed * 100.0))


def _frame(dp_raw: int, temp_raw: int = 5000, scale: int = 60) -> bytes:
    """A valid 9-byte frame (test_sdp810._frame): each word with its correct CRC."""
    body = bytearray(9)
    body[0:2] = struct.pack('>h', dp_raw)
    body[3:5] = struct.pack('>h', temp_raw)
    body[6:8] = struct.pack('>H', scale)
    for base in (0, 3, 6):
        body[base + 2] = commons.sensirion_crc8(body[base], body[base + 1])
    return bytes(body)


def _driver(config: dict = None) -> sdp810.Sdp810:
    """
    A plan B driver that has not run setup() (no bus touched), with the state setup() opens with: no reading yet
    (`_raw` None), untared, the default scale. The cases that need other state set it, as test_sdp810.py does.
    """
    driver = sdp810.Sdp810('airspeed_sdp810', dict(config or {}), _Controller())
    driver._raw, driver._temp_raw, driver._tared = None, 0, False
    driver._scale, driver._zero, driver._density = sdp810._DEFAULT_SCALE, 0, sdp810._AIR_DENSITY
    return driver


def _crc() -> dict:
    """The crc section."""
    generator = random.Random(_SEED)
    words = [(0xBE, 0xEF), (0x00, 0x00), (0xFF, 0xFF), (0x00, 0x01), (0x80, 0x00)]
    words += [(generator.randrange(256), generator.randrange(256)) for _ in range(40)]
    return {
        'compare': {'kind': 'exact'},
        'note': 'commons.sensirion_crc8(first, second): poly 0x31, seed 0xFF',
        'cases': [{'name': 'crc_%02x%02x' % word, 'input': {'first': word[0], 'second': word[1]},
                   'output': commons.sensirion_crc8(*word)} for word in words],
    }


def _frame_case(name: str, frame: bytes) -> dict:
    """One frames case: plan B's verdict, and the decoded words of a 9-byte frame."""
    output = {'ok': sdp810._frame_ok(frame)}
    if len(frame) == 9:
        output.update(pressure=sdp810._signed16(frame[0], frame[1]),
                      temperature=sdp810._signed16(frame[3], frame[4]), scale=(frame[6] << 8) | frame[7])
    return {'name': name, 'input': {'frame': frame.hex()}, 'output': output}


def _frames() -> dict:
    """The frames section."""
    cases = [_frame_case('good_%d_%d_%d' % values, _frame(*values))
             for values in ((8100, 5000, 60), (-600, 5000, 60), (0, -1200, 240), (32767, 32767, 65535),
                            (-32768, -32768, 0), (1, 0, 60))]
    good = _frame(8100)
    for index in range(9):
        for bit in range(8):
            flipped = bytearray(good)
            flipped[index] ^= 1 << bit
            cases.append(_frame_case('flip_byte%d_bit%d' % (index, bit), bytes(flipped)))
    cases.append(_frame_case('short', good[:8]))
    cases.append(_frame_case('long', good + b'\x00'))
    cases.append(_frame_case('empty', b''))
    return {'compare': {'kind': 'exact'},
            'note': '_frame_ok(frame) (length and all three CRCs); for 9 bytes the _signed16 words and the scale',
            'cases': cases}


def _pressure(raw: int, scale: int, zero: int) -> int:
    """plan B's _pressure(raw) with this latched scale and zero: Pa x 100."""
    driver = _driver()
    driver._scale, driver._zero = scale, zero
    return driver._pressure(raw)


def _pressure_vetted(raw: int, scale: int, zero: int):
    """
    plan B's dynamic_pressure channel value in Pa (its floored Pa x 100, as a float), vetted for the comparison: it
    is within _BUDGET of the real law, and the port's f32 within the rest of _TOLERANCE of the law.

    Returns:
        (the channel value, how far plan B is from the law), or None when the case is off limits.
    """
    channel = fixed.to_float(_pressure(raw, scale, zero))
    law = raw / scale - zero / 100.0
    port = _rust_airspeed(raw, scale, zero, sdp810._AIR_DENSITY)[0]
    if abs(channel - law) > _BUDGET or abs(port - law) > _TOLERANCE - _BUDGET:
        return None
    return channel, abs(channel - law)


def _pressure_case(raw: int, scale: int, zero: int):
    """One pressure case (the difference kept for the measurement), or None when it is off limits."""
    vetted = _pressure_vetted(raw, scale, zero)
    if vetted is None:
        return None
    return {'name': 'raw_%d_scale_%d_zero_%d' % (raw, scale, zero),
            'input': {'raw': raw, 'scale': scale, 'zero': zero},
            'output': {'hundredths': _pressure(raw, scale, zero), 'dynamic_pressure': vetted[0]},
            'difference': vetted[1]}


def _pressures() -> dict:
    """The pressure section."""
    cases = []
    for explicit in ((8100, 60, 0), (-600, 60, 0), (200, 60, 0), (200, 60, 333), (0, 60, 0), (-1, 60, 0), (1, 60, 0),
                     (8100, 240, 0), (-32768, 60, -125), (32767, 60, 20), (7, 3, 0), (-7, 3, 0)):
        case = _pressure_case(*explicit)
        if case is None:
            raise ValueError('drivers_sdp810: explicit pressure case %r is off limits' % (explicit,))
        cases.append(case)
    generator = random.Random(_SEED + 1)
    redrawn = 0
    for _ in range(_SWEEP_CASES):
        for _draw in range(_DRAWS):
            case = _pressure_case(generator.randint(-32768, 32767),
                                  generator.choice((60, 240, generator.randint(1, 2000))),
                                  generator.randint(-2000, 2000))
            if case is not None:
                break
            redrawn += 1
        else:
            raise ValueError('drivers_sdp810: no pressure case within %d draws' % _DRAWS)
        cases.append(case)
    worst = max(case.pop('difference') for case in cases)
    return {
        'compare': {'kind': 'abs', 'tol': _TOLERANCE},
        'note': ('_pressure(raw) with _scale = scale and _zero = zero: the tared dynamic pressure as the telemetry '
                 'carries it (hundredths, Pa x 100, exact) and as the channel does (dynamic_pressure, Pa, signed: '
                 'plan B floors to the hundredth, the port computes raw / scale - zero / 100 in f32)'),
        'measured': {'worst_plan_b_against_the_law_pa': round(worst, 6), 'sweeps_redrawn': redrawn},
        'cases': cases,
    }


def _airspeed_case(raw: int, scale: int, zero: int, density: float) -> dict:
    """
    One airspeed case: plan B's airspeed and cms, vetted against the real-number law and the port's f32.

    Returns:
        The case, with `difference` (plan B against the law, m/s) for the builder's measurement; None when the case
        is off limits (see the module notes).
    """
    driver = _driver()
    driver._scale, driver._zero, driver._density = scale, zero, density
    airspeed = driver._airspeed(driver._pressure(raw))
    cms = int(airspeed * fixed.SCALE)
    law_pressure = raw / scale - zero / 100.0
    floored = fixed.to_float(driver._pressure(raw))
    if (floored > 0.0) != (law_pressure > 0.0) or (0.0 < floored < 1.0 and floored != law_pressure):
        return None
    pressure = _pressure_vetted(raw, scale, zero)
    if pressure is None:
        return None
    law = math.sqrt(2.0 * law_pressure / density) if law_pressure > 0.0 else 0.0
    _pressure_f32, airspeed_f32, rust_cms = _rust_airspeed(raw, scale, zero, density)
    if rust_cms != cms or abs(airspeed - law) > _BUDGET or abs(airspeed_f32 - law) > _TOLERANCE - _BUDGET:
        return None
    return {'name': 'raw_%d_scale_%d_zero_%d_rho_%r' % (raw, scale, zero, density),
            'input': {'raw': raw, 'scale': scale, 'zero': zero, 'density': density},
            'output': {'airspeed': airspeed, 'airspeed_cms': cms, 'dynamic_pressure': pressure[0]},
            'difference': abs(airspeed - law)}


def _airspeeds() -> dict:
    """The airspeed section."""
    cases = []
    # the last four: under 1 Pa on an exact hundredth (0.05, 0.5, 0.95 and 0.05 Pa), where plan B and the law agree
    for explicit in ((8100, 60, 0, 1.225), (-600, 60, 0, 1.225), (0, 60, 0, 1.225), (8100, 60, 0, 1.18),
                     (30000, 60, 0, 1.18), (600, 60, 0, 1.18), (8100, 60, 13500, 1.18), (8100, 60, 13600, 1.18),
                     (3, 60, 0, 1.18), (30, 60, 0, 1.18), (57, 60, 0, 1.18), (12, 240, 0, 1.18)):
        case = _airspeed_case(*explicit)
        if case is None:
            raise ValueError('drivers_sdp810: explicit airspeed case %r is off limits' % (explicit,))
        cases.append(case)
    generator = random.Random(_SEED + 2)
    redrawn = 0
    for _ in range(_SWEEP_CASES):
        for _draw in range(_DRAWS):
            case = _airspeed_case(generator.randint(-3000, 32767), generator.choice((60, 60, 240)),
                                  generator.randint(-500, 500), generator.choice((1.18, 1.225, 1.1, 1.3)))
            if case is not None:
                break
            redrawn += 1
        else:
            raise ValueError('drivers_sdp810: no airspeed case within %d draws' % _DRAWS)
        cases.append(case)
    worst = max(case.pop('difference') for case in cases)
    return {
        'compare': {'kind': 'abs', 'tol': _TOLERANCE},
        'note': ('_airspeed(_pressure(raw)) m/s with _scale, _zero (Pa x 100) and _density; airspeed_cms = '
                 'int(airspeed x 100), exact; dynamic_pressure, the Pa it came from. Plan B derives it from the '
                 'floored Pa x 100, the port from raw / scale - zero / 100 in f32'),
        'measured': {'worst_plan_b_against_the_law_ms': round(worst, 6), 'sweeps_redrawn': redrawn},
        'cases': cases,
    }


class _TareRun:
    """One tare scenario on plan B's driver: its state stubs and the steps written so far."""

    def __init__(self, name: str, scale: int, zero: int, density: float, tared: bool) -> None:
        self.name = name
        self.driver = _driver()
        self.driver._scale, self.driver._zero, self.driver._density, self.driver._tared = scale, zero, density, tared
        self.driver._pressure_ch = _Channel()
        self.persisted: list = []
        self.driver._persist_zero = lambda: self.persisted.append(self.driver._zero)  # never a real NVS
        self.init = {'scale': scale, 'zero': zero, 'density': density, 'tared': tared}
        self.steps: list = []

    def state(self) -> dict:
        """The driver state every step reports."""
        return {'zero': self.driver._zero, 'tared': self.driver._tared, 'density': self.driver._density}

    def pressure(self, raw: int) -> None:
        """A frame's pressure, which becomes the tare source."""
        self.steps.append({'in': {'action': 'pressure', 'raw': raw},
                           'out': dict(self.state(), hundredths=self.driver._pressure(raw))})

    def update(self, properties: dict) -> None:
        """An operator update: changed names and the tare persisted, or the refusal text."""
        before = len(self.persisted)
        try:
            changed = self.driver.update(dict(properties))
        except ValueError as error:
            out = dict(self.state(), refused=str(error))
        else:
            persisted = self.persisted[before:]
            out = dict(self.state(), changed=changed, persist=persisted[0] if persisted else None)
        self.steps.append({'in': {'action': 'update', 'properties': properties}, 'out': out})

    def calibration(self, channel) -> None:
        """calibration()'s prompt with the channel's latest (Pa x 100, or None), '' once tared."""
        self.driver._pressure_ch.latest = channel
        self.steps.append({'in': {'action': 'calibration', 'channel': channel},
                           'out': dict(self.state(), prompt=self.driver.calibration())})

    def calibrate(self) -> None:
        """calibrate(): plan B's message before the first frame, else the tare (persisted)."""
        before = len(self.persisted)
        message = asyncio.run(self.driver.calibrate())
        persisted = self.persisted[before:]
        self.steps.append({'in': {'action': 'calibrate'},
                           'out': dict(self.state(), message=message, persist=persisted[0] if persisted else None)})

    def document(self) -> dict:
        return {'name': self.name, 'init': self.init, 'steps': self.steps}


def _tare() -> dict:
    """The tare section: test_sdp810.py's ladder, then the corners."""
    scenarios = []
    ladder = _TareRun('test_sdp810_ladder', 60, 0, 1.225, False)
    ladder.update({'zero': True})  # NEGATIVE first: no frame yet, refused out loud, nothing persisted
    ladder.calibrate()  # calibrate's own answer before the first frame
    ladder.pressure(200)
    ladder.update({'zero': True})  # 200 * 100 // 60 = 333, persisted
    ladder.pressure(200)  # the same reading now tares to 0
    ladder.update({'zero_offset_pa': 1.5})  # a direct set: a tare, not persisted
    ladder.update({'air_density': 1.2})
    ladder.update({'zero_offset_pa': -1.25})
    ladder.pressure(-600)
    scenarios.append(ladder)
    zeroed = _TareRun('a_0_pa_tare_is_a_tare', 60, 0, 1.225, False)
    zeroed.calibration(None)
    zeroed.calibration(1234)
    zeroed.pressure(0)
    zeroed.update({'zero': True})
    zeroed.calibration(0)
    scenarios.append(zeroed)
    restored = _TareRun('a_restored_0_offset_is_a_tare', 60, 0, 1.225, False)
    restored.calibration(-125)
    restored.update({'zero_offset_pa': 0.0})
    restored.calibration(-125)
    scenarios.append(restored)
    both = _TareRun('a_tare_and_an_offset_in_one_update', 60, 0, 1.18, False)
    both.pressure(-600)
    both.update({'zero': True, 'zero_offset_pa': 2.0, 'air_density': 1.25})  # the name twice, as plan B replies
    both.update({})
    both.update({'zero': False})
    both.update({'zero': 0})
    both.update({'zero': 1})
    scenarios.append(both)
    scaled = _TareRun('a_tare_at_scale_240', 240, 0, 1.18, False)
    scaled.pressure(-7)
    scaled.calibrate()
    scaled.pressure(-7)
    scaled.update({'zero_offset_pa': 1.18})  # 118: CPython's f64 product truncated; the port snaps its f32 117.99999
    scenarios.append(scaled)
    return {
        'compare': {'kind': 'abs', 'tol': 1e-6},
        'note': ('stateful; init sets _scale, _zero (Pa x 100), _density, _tared; actions pressure(raw) -> '
                 'hundredths, update(properties) -> changed and the tare persisted (or the refusal text), '
                 'calibration(channel Pa x 100 or null) -> prompt, calibrate() -> message (null on success) and the '
                 'tare persisted; every out carries zero, tared and density after the step'),
        'scenarios': [scenario.document() for scenario in scenarios],
    }


def _setup_case(name: str, answers: list, config: dict = None, stored=None, nvs: bool = False) -> dict:
    """
    One setup case: plan B's setup() over a scripted part, its transcript and the driver state after it.

    Args:
        name - the case name.
        answers - the scripted part's answers (driver_bus.sequence).
        config - the component block.
        stored - the tare NVS holds (Pa x 100), None for none.
        nvs - whether the board has an NVS partition at all when nothing is stored (plan B's host has none).
    """
    part = driver_bus.sequence(answers)
    bus = driver_bus.Bus(part)
    driver = _driver(config)
    original_bind, original_nvs, original_provide = i2cbus.bind, sdp810._nvs, databoard.Databoard.provide
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    sdp810._nvs = _Nvs(stored) if stored is not None or nvs else None
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: tuple(_Channel() for _ in want))
    try:
        ok = driver_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, sdp810._nvs, databoard.Databoard.provide = original_bind, original_nvs, original_provide
    if not part.exhausted():
        raise ValueError('drivers_sdp810: setup case %s left scripted answers unused' % name)
    return {'name': name, 'input': {'config': config or {}, 'stored_zero': stored, 'script': bus.transcript},
            'output': {'ok': ok, 'scale': driver._scale, 'zero': driver._zero, 'tared': driver._tared}}


def _setups() -> dict:
    """The setup section."""
    nak = driver_bus.NAK
    good, bad = _frame(12, 5000, 60), bytearray(_frame(12))
    bad[8] ^= 0x01  # the SCALE word's CRC: the field the three-CRC rule exists for
    cases = [
        _setup_case('healthy_scale_60', [None, None, good]),
        _setup_case('the_125pa_part_scale_240', [None, None, _frame(-40, 2500, 240)]),
        _setup_case('scale_0_keeps_the_default', [None, None, _frame(12, 5000, 0)]),
        _setup_case('stop_naks_then_a_retry_starts', [nak, None, None, good]),
        _setup_case('start_naks_then_a_retry_starts', [None, nak, None, None, good]),
        _setup_case('read_naks_then_a_retry_reads', [None, None, nak, None, None, good]),
        _setup_case('a_corrupt_scale_then_a_good_frame', [None, None, bytes(bad), None, None, good]),
        _setup_case('two_corrupt_frames_fail_and_stop_the_part', [None, None, bytes(bad), None, None, bytes(bad),
                                                                  None]),
        _setup_case('two_corrupt_frames_and_a_dead_stop', [None, None, bytes(bad), None, None, bytes(bad), nak]),
        _setup_case('nothing_acks', [nak, nak]),
        _setup_case('another_address', [None, None, good], config={'addr': 0x26}),
        _setup_case('the_config_zero_when_nvs_has_none', [None, None, good],
                    config={'zero_offset_pa': 2.0, 'air_density': 1.25, 'period_ms': 40}),
        _setup_case('nvs_zero_wins_over_the_config', [None, None, good], config={'zero_offset_pa': 2.0},
                    stored=-125),
        _setup_case('nvs_stored_0_is_a_tare', [None, None, good], stored=0),
        _setup_case('nvs_without_the_key', [None, None, good], config={'zero_offset_pa': -1.18}, nvs=True),
    ]
    return {
        'compare': {'kind': 'exact'},
        'note': ('setup() over a scripted part: input config, stored_zero (the NVS i32, null for none) and the I2C '
                 'transcript; output ok, the latched scale, the zero (Pa x 100) and tared'),
        'cases': cases,
    }


class _Stop(Exception):
    """Raised from the run loop's period wait once the scripted passes are done."""


def _run_case(name: str, passes: list) -> dict:
    """
    One run case: plan B's run() over a scripted part, pass by pass.

    Args:
        name - the case name.
        passes - per pass, the answers its transactions get (a frame, NAK, or the restart's acks after a NAK).

    Returns:
        The case: per pass its transcript, and its row or its fault and whether a restart ran (and succeeded).
    """
    part = driver_bus.sequence([answer for answers in passes for answer in answers])
    bus = driver_bus.Bus(part)
    driver = _driver({'period_ms': _PERIOD_MS})
    driver._bus, driver._addr, driver._period_ms = bus, _ADDRESS, _PERIOD_MS
    driver._scale, driver._zero, driver._density = 60, 0, 1.18
    driver._pressure_ch, driver._airspeed_ch, driver._telemetry = _Channel(), _Channel(), _Rows()
    logs: list = []
    restarts: dict = {}  # pass index -> whether plan B logged the restart it ran on that pass
    marks: list = []  # the transcript length at each pass's end
    original_log, original_restart, original_sleep = recorder.Recorder.log, driver._restart, asyncio.sleep_ms
    recorder.Recorder.log = staticmethod(lambda descriptor, message: logs.append(message))

    async def restart() -> None:
        before = len(logs)
        await original_restart()
        restarts[len(marks)] = len(logs) > before

    async def sleep_ms(milliseconds: int) -> None:
        if milliseconds == _PERIOD_MS:
            marks.append(len(bus.transcript))
            if len(marks) == len(passes):
                raise _Stop()
        else:
            bus.transcript.append({'op': 'delay_ms', 'ms': milliseconds})
        shims.advance(milliseconds)
        await asyncio.sleep(0)

    async def passes_then_stop() -> None:
        try:
            await driver.run()
        except _Stop:
            pass

    driver._restart = restart
    asyncio.sleep_ms = sleep_ms
    try:
        asyncio.run(passes_then_stop())
    finally:
        asyncio.sleep_ms, recorder.Recorder.log = original_sleep, original_log
    if not part.exhausted():
        raise ValueError('drivers_sdp810: run case %s left scripted answers unused' % name)
    steps, start = [], 0
    rows = iter(driver._telemetry.rows)
    pressures, airspeeds = iter(driver._pressure_ch.pushed), iter(driver._airspeed_ch.pushed)
    for index, end in enumerate(marks):
        transcript = bus.transcript[start:end]
        start = end
        read = transcript[0]
        if not read.get('fails') and sdp810._frame_ok(bytes.fromhex(read['bytes'])):
            out = {'row': next(rows), 'dynamic_pressure': fixed.to_float(next(pressures)), 'airspeed': next(airspeeds)}
        else:
            out = {'fault': 'bus' if read.get('fails') else 'checksum', 'restart': restarts.get(index)}
        steps.append({'in': {'script': transcript}, 'out': out})
    if next(rows, None) is not None or next(pressures, None) is not None or next(airspeeds, None) is not None:
        raise ValueError('drivers_sdp810: run case %s pushed a row or a value no pass accounts for' % name)
    return {'name': name, 'steps': steps}


def _clean(raw: int) -> bytes:
    """
    A frame whose airspeed_cms the port's f32 truncates as plan B does, and whose dynamic pressure and airspeed it
    computes within _TOLERANCE of plan B's (checked, not assumed).
    """
    pressure, airspeed, cms = _rust_airspeed(raw, 60, 0, 1.18)
    planned_pressure, planned_airspeed = fixed.to_float(_pressure(raw, 60, 0)), _driver_airspeed(raw)
    if (cms != int(planned_airspeed * fixed.SCALE) or _pressure_vetted(raw, 60, 0) is None
            or abs(pressure - planned_pressure) > _TOLERANCE or abs(airspeed - planned_airspeed) > _TOLERANCE):
        raise ValueError('drivers_sdp810: run frame %d is not clean' % raw)
    return _frame(raw, 5003, 60)


def _driver_airspeed(raw: int) -> float:
    """plan B's airspeed for a raw count at scale 60, zero 0, density 1.18."""
    driver = _driver()
    driver._scale, driver._zero, driver._density = 60, 0, 1.18
    return driver._airspeed(driver._pressure(raw))


def _runs() -> dict:
    """The run section: the restart ladder."""
    nak = driver_bus.NAK
    ok = [_clean(8100)]
    corrupt = bytearray(_clean(8100))
    corrupt[2] ^= 0x10
    reject = [bytes(corrupt)]
    restart = [None, None]  # the restart's stop and start, acked
    cases = [
        _run_case('healthy_reads', [ok, [_clean(-600)], [_clean(3000)], [_clean(0)]]),
        _run_case('five_failures_restart_once',
                  [[nak]] * 4 + [[nak] + restart] + [[nak]] * 3 + [ok]),
        _run_case('a_good_read_rearms_the_ladder',
                  [[nak]] * 4 + [ok] + [[nak]] * 4 + [[nak] + restart] + [ok]),
        _run_case('crc_rejects_neither_count_nor_rearm',
                  [[nak]] * 2 + [reject] * 3 + [[nak]] * 2 + [[nak] + restart] + [ok]),
        _run_case('a_dead_restart_is_not_logged', [[nak]] * 4 + [[nak, nak]] + [[nak]] * 2 + [ok]),
        _run_case('a_restart_whose_start_naks', [[nak]] * 4 + [[nak, None, nak]] + [ok]),
        _run_case('a_second_run_restarts_again',
                  [[nak]] * 4 + [[nak] + restart] + [ok] + [[nak]] * 4 + [[nak] + restart] + [ok]),
    ]
    return {
        'compare': {'kind': 'abs', 'tol': _TOLERANCE},
        'note': ('run(): per pass the I2C transcript (the period wait left out: the owner schedules it) and its out: '
                 'the telemetry row [dynamic_pressure Pa x 100, airspeed_cms, temperature x 100] (integers, exact) '
                 'with the channel values pushed, dynamic_pressure Pa and airspeed m/s; or the fault '
                 '(bus | checksum) and restart: null when no restart ran on that pass, true when it ran and plan B '
                 'logged it, false when it ran and failed. _scale 60, _zero 0, _density 1.18'),
        'scenarios': cases,
    }


def _diagnose_case(name: str, answer) -> dict:
    """One diagnose case over a scripted part."""
    part = driver_bus.sequence([answer])
    bus = driver_bus.Bus(part)
    driver = _driver()
    driver._bus, driver._addr = bus, _ADDRESS
    verdict = driver_bus.run(bus, driver.diagnose())
    return {'name': name, 'input': {'script': bus.transcript}, 'output': verdict}


def _diagnoses() -> dict:
    """The diagnose section."""
    garbage = bytearray(_frame(8100))
    garbage[5] ^= 0xFF
    return {
        'compare': {'kind': 'exact'},
        'note': 'diagnose(): one frame read; the verdict text',
        'cases': [_diagnose_case('absent', driver_bus.NAK), _diagnose_case('garbage', bytes(garbage)),
                  _diagnose_case('present', _frame(8100, 5000, 240))],
    }


def build() -> dict:
    """
    The drivers_sdp810 fixture document.

    Returns:
        {meta, crc, frames, pressure, airspeed, tare, setup, run, diagnose}.
    """
    return {
        'meta': {'source': ('src/glider/drivers/sdp810.py, src/glider/commons.py, src/glider/fixed.py, '
                            'src/glider/test/test_sdp810.py')},
        'crc': _crc(),
        'frames': _frames(),
        'pressure': _pressures(),
        'airspeed': _airspeeds(),
        'tare': _tare(),
        'setup': _setups(),
        'run': _runs(),
        'diagnose': _diagnoses(),
    }
