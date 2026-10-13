"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture warmstart.json: plan B's warm start (src/glider/warmstart.py) as the Rust port (crates/warmstart)
must answer it. Six sections, each a golden fixture of its own ({compare, note, cases}):
  * gate -- should_restore() over every crumb stage (the five flight stages and ids that are none), both
    separation readings, every ESP-IDF reset reason and the 0..600 s age window's edges, plus torn crumbs (a
    stage or stamp missing) and a seeded sweep: {restore, reason}.
  * load -- load() over NVS contents: no stage, the 0 sentinel, a missing, torn, oversized or non-object blob,
    and blobs written by plan B's own checkpoint: the crumb the port reads back.
  * save -- save(): the NVS writes in order (the blob, then the stage i32 last, then the commit).
  * apply -- _apply_restore() on stubs (as test_warmstart drives it): the apply plan as data.
  * restore -- restore(), the boot decision end to end: whether it warm-starts and the NVS writes it makes (a
    rejected gate clears the stage).
  * checkpoint -- the Checkpoint task's own run() loop, poll by poll, on stand-ins (the controller's stage, arm and
    tasks, the mission it freezes from, the altitude and speed channels): whether the poll checkpointed, the
    telemetry row, the crumb it saved (armed only), and the recovery identity it holds.

All six compare exactly. A crumb's metres are f32 in the Rust port and its coordinates 1e-7 degree units; written
back as text (the shortest f32 text, the units divided out) they give the decimal plan B wrote, because every float
in a case has at most 7 significant digits and every coordinate at most 7 decimals -- what a board's f32 json.dumps
writes. The builder refuses a case where that does not hold (_check_exact).

THE RESET CAUSE. Plan B reads machine.reset_cause(), MicroPython's code, which the esp32 port maps from
esp_reset_reason() (ports/esp32/modmachine.c mp_machine_reset_cause, v1.29.0, the boards' firmware): POWERON and
BROWNOUT to PWRON_RESET; INT_WDT, TASK_WDT and WDT to WDT_RESET; DEEPSLEEP to DEEPSLEEP_RESET; SW, PANIC and EXT
to HARD_RESET; SDIO, UNKNOWN and every other reason (USB, JTAG, EFUSE, PWR_GLITCH, CPU_LOCKUP) to 0. The shims'
`machine` has no reset_cause, so this builder stands in that table (_MICROPYTHON_CAUSE) and MicroPython's constants
on it; restore() then runs its own `cause in (WDT_RESET, SOFT_RESET, HARD_RESET)`. A case names the ESP-IDF reason,
which the Rust port reads directly. MicroPython's SOFT_RESET (a soft reboot of the interpreter, no chip reset) has
no ESP-IDF reason and no Rust counterpart, so no case carries it.

DELIBERATE DIVERGENCES (marked: a case where the Rust answer differs carries `divergence`, the Rust answer in full
plus a note, and the Rust test expects that answer instead of `output`):
  * the operator's boot rules of 2026-10-06 (doc/specs/avionics-port.md section 4.6, M8): a crumb reading DONE is a
    new session in SETTING, never a warm start (plan B restores DONE after a reset within 600 s); only a crumb in
    BOOSTING..LANDING warm-starts, so a SETTING crumb is a cold boot (plan B restores SETTING, re-armed); no crumb
    is SETTING, as in plan B. The stage rules come before the reset and age checks, so a DONE or SETTING crumb is
    refused for its stage whatever the cause or age.
  * a crumb stage that is no flight stage (0, 6, -1) is refused; plan B passes the gate ('recover ?') and then
    raises in Controller.set_stage(), crashing the boot after half the restore was applied.
  * Q19's step 2 (the operator, 2026-10-13): the Rust gate makes no time or cause check. With no battery-backed RTC
    a power cycle loses the clock, so a crumb's age says nothing, and a battery bounce in flight is a power-on too;
    only the separation latch refuses a glide stage. Plan B refuses a power-on (and USB, JTAG, CPU_LOCKUP, every cause
    MicroPython maps to cold) and a crumb outside 0..600 s; the Rust port recovers them.
  * a blob that is JSON but not an object, or one holding NaN: plan B's load() raises TypeError on the first (the
    stage assignment), crashing the boot, and CPython reads NaN where a board's MicroPython json does not; the Rust
    port reads both as no crumb (a cold boot that writes nothing, as for a torn blob).
Values of another JSON type than plan B's checkpoint writes (a text stamp, a numeric `armed`) are left to the Rust
unit tests: the Rust port reads a wrong type as absent, where plan B computes on whatever arrived.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import gc
import json
import math
import random
import struct
import time

import controller
import databoard
import inspector
import machine
import shims
import warmstart

_SEED: int = 20261006  # the gate sweep's random.Random seed
_SWEEP: int = 120  # seeded gate cases on top of the explicit ones
_MAX_AGE_S: int = 600  # warmstart._MAX_AGE_S (a const with an underscore: not a module attribute on a board)
_STAMP_COLD_CLOCK: int = 1000  # a stamp before any clock sync: 1000 s into 2000-01-01 (test_warmstart's)
_STAMP_SYNCED: int = 844560000  # a stamp after the hub's sync: 2026-10-06T00:00:00Z in seconds since 2000
_NVS_NOT_FOUND: int = -0x1102  # ESP_ERR_NVS_NOT_FOUND, as esp32.NVS raises it
_NVS_INVALID_LENGTH: int = -0x110C  # ESP_ERR_NVS_INVALID_LENGTH: the blob is longer than the read buffer
_BLOB_MAX: int = 512  # warmstart._BLOB_MAX, the read buffer

"""MicroPython's machine.reset_cause() codes (ports/esp32/modmachine.c, v1.29.0)."""
_PWRON_RESET: int = 1
_HARD_RESET: int = 2
_WDT_RESET: int = 3
_DEEPSLEEP_RESET: int = 4
_SOFT_RESET: int = 5

"""
The ESP-IDF reset reasons (esp_system.h esp_reset_reason_t, IDF 5.5), in code order, and the code MicroPython's
esp32 port gives machine.reset_cause() for each (see the module notes).
"""
_REASONS: tuple = ('UNKNOWN', 'POWERON', 'EXT', 'SW', 'PANIC', 'INT_WDT', 'TASK_WDT', 'WDT', 'DEEPSLEEP', 'BROWNOUT',
                   'SDIO', 'USB', 'JTAG', 'EFUSE', 'PWR_GLITCH', 'CPU_LOCKUP')
_MICROPYTHON_CAUSE: dict = {
    'POWERON': _PWRON_RESET, 'BROWNOUT': _PWRON_RESET,
    'INT_WDT': _WDT_RESET, 'TASK_WDT': _WDT_RESET, 'WDT': _WDT_RESET,
    'DEEPSLEEP': _DEEPSLEEP_RESET,
    'SW': _HARD_RESET, 'PANIC': _HARD_RESET, 'EXT': _HARD_RESET,
}
"""The reasons the Rust port counts as a reset (doc/specs/avionics-port.md section 4.6): plan B's, plus CPU_LOCKUP."""
_STAGE_IDS: tuple = (1, 2, 3, 4, 5)  # SETTING..DONE
_NOT_STAGES: tuple = (0, 6, -1)  # ids a corrupted NVS i32 could hold: the sentinel, past DONE, negative

"""The Rust port's reasons for refusing a crumb plan B would not refuse for the same thing (its Reason Display)."""
_UNKNOWN_STAGE_REASON: str = 'crumb stage %d is no flight stage -> cold boot'
_AFTER_DONE_REASON: str = 'crumb reads done -> a new session in setting'
_ON_THE_PAD_REASON: str = 'crumb reads setting -> a cold boot (only boosting..landing warm-start)'

_BOOT_RULES_NOTE: str = ('the operator boot rules (2026-10-06): a DONE crumb is a new session in SETTING, only '
                         'BOOSTING..LANDING warm-starts; plan B restores SETTING and DONE too')
_UNKNOWN_STAGE_NOTE: str = ('a crumb stage that is no flight stage is refused; plan B passes it and then raises in '
                            'Controller.set_stage()')
_STEP_TWO_NOTE: str = ("Q19's step 2 (the operator, 2026-10-13): no time or cause check, only the separation latch "
                       'refuses a glide stage; plan B refuses a power-on and a crumb outside 0..600 s')
_NOT_AN_OBJECT_NOTE: str = ('a blob that is JSON but no object reads as no crumb; plan B raises TypeError and crashes '
                            'the boot')
_NAN_NOTE: str = ('a blob holding NaN reads as no crumb (strict JSON, as a board\'s MicroPython json reads it); '
                  'CPython reads NaN')

"""
A crumb as plan B's checkpoint writes it for an armed glide: the recovery identity frozen at BOOSTING (launch, zone,
pad altitude, pitot tare), then the live state. test_warmstart's launch and zone, metres with f32-exact text.
"""
_IDENTITY: dict = {'launch': [25.5, -80.4], 'zone': [[25.514, -80.393], [25.514, -80.391]], 'pad_altitude': 100.0,
                   'pitot_zero': -1.75}
_LIVE: dict = {'armed': True, 'altitude': 412.5, 'speed': 21.25, 'airspeed': 16.5, 'ticks_ms': 73215}


def _crumb(stage: int | None, stamp: int | None, **fields: object) -> dict:
    """A full crumb (identity + live state) with this stage and stamp, plus any field overrides."""
    crumb = dict(_IDENTITY)
    crumb.update(_LIVE)
    if stage is not None:
        crumb['stage'] = stage
    if stamp is not None:
        crumb['stamp'] = stamp
    crumb.update(fields)
    return crumb


def _shortest_f32(value: float) -> float:
    """The decimal an f32 of `value` prints as at its shortest (what the Rust writer writes), read back as a float."""
    single = struct.unpack('<f', struct.pack('<f', value))[0]
    for digits in range(1, 10):
        text = '%.*g' % (digits, single)
        if struct.unpack('<f', struct.pack('<f', float(text)))[0] == single:
            return float(text)
    return single


def _check_exact(value: object, path: str) -> None:
    """
    Refuse a crumb value the exact comparison cannot hold.

    The Rust port keeps metres as f32 and coordinates as 1e-7 degree units. A float survives that when its shortest
    f32 text is its own decimal, and a coordinate (launch, zone) when it has at most 7 decimals.

    Args:
        value - a crumb, or a part of one.
        path - where `value` sits, for the message (a coordinate path ends in launch or zone).

    Returns:
        None.

    Raises:
        ValueError - a float that would not come back as itself.
    """
    if isinstance(value, dict):
        for key, item in value.items():
            _check_exact(item, '%s.%s' % (path, key))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _check_exact(item, '%s[%d]' % (path, index))
    elif isinstance(value, float):
        coordinate = '.launch' in path or '.zone' in path
        back = round(value * 1e7) / 1e7 if coordinate else _shortest_f32(value)
        if not math.isfinite(value) or back != value:
            raise ValueError('%s: %r would come back as %r' % (path, value, back))


def _cause_is_reset(reason: str) -> bool:
    """Plan B's own test of the reset cause (warmstart.restore), on the code MicroPython gives this reason."""
    return _MICROPYTHON_CAUSE.get(reason, 0) in (_WDT_RESET, _SOFT_RESET, _HARD_RESET)


def _rust_gate(crumb: dict | None, separated: bool, reason: str, now_s: int) -> tuple:
    """
    The Rust port's answer to a gate case (see the module notes): the operator's stage rules, then Q19's step 2 --
    plan B's texts, but no time or cause check, only the separation latch.

    Args:
        crumb - the crumb, or None.
        separated - the separation latch.
        reason - the ESP-IDF reset reason.
        now_s - the clock, seconds since 2000.

    Returns:
        (restore, reason text).
    """
    if crumb is None:
        return False, 'no crumb'
    stage, stamp = crumb.get('stage'), crumb.get('stamp')
    if stage is None or stamp is None:
        return False, 'crumb missing stage/stamp -> cold boot'
    if stage not in controller.Stage.STAGES:
        return False, _UNKNOWN_STAGE_REASON % stage
    if stage == controller.Stage.DONE:
        return False, _AFTER_DONE_REASON
    if stage == controller.Stage.SETTING:
        return False, _ON_THE_PAD_REASON
    if controller.Stage.passive(stage) and not separated:
        return False, 'separation switch reads nested'
    return True, 'recover %s, %ds after checkpoint' % (controller.Stage.STAGES[stage], now_s - stamp)


def _divergence_note(crumb: dict | None, reason: str, now_s: int) -> str:
    """Why the Rust gate answers this case differently (the first rule that applies)."""
    stage = (crumb or {}).get('stage')
    if stage is not None and stage not in controller.Stage.STAGES:
        return _UNKNOWN_STAGE_NOTE
    if stage in (controller.Stage.SETTING, controller.Stage.DONE):
        return _BOOT_RULES_NOTE
    stamp = (crumb or {}).get('stamp')
    if stamp is not None and (not _cause_is_reset(reason) or not 0 <= now_s - stamp <= 600):
        return _STEP_TWO_NOTE
    raise ValueError('a gate divergence no rule explains: %r %s' % (crumb, reason))


def _gate_case(name: str, crumb: dict | None, separated: bool, reason: str, now_s: int) -> dict:
    """
    One gate case through plan B's should_restore(), with the Rust answer where it differs.

    Args:
        name - the case name.
        crumb - the crumb, or None.
        separated - the separation latch.
        reason - the ESP-IDF reset reason (plan B sees the MicroPython code it maps to).
        now_s - the clock, seconds since 2000.

    Returns:
        {name, input: {crumb, separated, cause, now_s}, output: {restore, reason}[, divergence]}.
    """
    if crumb is not None:
        _check_exact(crumb, name)
    restore, text = warmstart.should_restore(crumb, separated, _cause_is_reset(reason), now_s)
    case = {'name': name, 'input': {'crumb': crumb, 'separated': separated, 'cause': reason, 'now_s': now_s},
            'output': {'restore': restore, 'reason': text}}
    rust = _rust_gate(crumb, separated, reason, now_s)
    if rust != (restore, text):
        case['divergence'] = {'restore': rust[0], 'reason': rust[1], 'note': _divergence_note(crumb, reason, now_s)}
    return case


"""Ages around the 0..600 s window's edges (and far outside), in seconds."""
_AGES: tuple = (-601, -1, 0, 1, 30, 599, 600, 601, 3600)


def _gate_cases() -> list:
    """
    The gate section's cases: plan B's own test, every stage x separation x age, every reason x stage, torn crumbs,
    and a seeded sweep.

    Returns:
        The case list.
    """
    stages = _STAGE_IDS + _NOT_STAGES
    cases = []
    """test_warmstart.test_should_restore, literally: the stamp 1000, a GLIDING crumb armed, now 1030 unless stated."""
    test_crumb = {'launch': [25.5, -80.4], 'zone': [[25.514, -80.393], [25.514, -80.391]], 'pad_altitude': 100.0,
                  'stamp': 1000, 'stage': controller.Stage.GLIDING, 'armed': True}
    for name, crumb, separated, reason, now_s in (
            ('test_happy_path', test_crumb, True, 'TASK_WDT', 1030),
            ('test_no_crumb', None, True, 'TASK_WDT', 1030),
            ('test_torn_without_stage', {'stamp': 1000}, True, 'TASK_WDT', 1030),
            ('test_torn_without_stamp', {'stage': controller.Stage.GLIDING}, True, 'TASK_WDT', 1030),
            ('test_gliding_nested', test_crumb, False, 'TASK_WDT', 1030),
            ('test_power_on', test_crumb, True, 'POWERON', 1030),
            ('test_negative_age', test_crumb, True, 'TASK_WDT', 990),
            ('test_age_601', test_crumb, True, 'TASK_WDT', 1601),
            ('test_age_600', test_crumb, True, 'TASK_WDT', 1600),
            ('test_boosting_nested', {'stage': controller.Stage.BOOSTING, 'stamp': 1000}, False, 'TASK_WDT', 1030),
            ('test_landing_separated', {'stage': controller.Stage.LANDING, 'stamp': 1000}, True, 'TASK_WDT', 1030),
            ('test_landing_nested', {'stage': controller.Stage.LANDING, 'stamp': 1000}, False, 'TASK_WDT', 1030),
            ('test_setting_reset', {'stage': controller.Stage.SETTING, 'stamp': 1000}, False, 'TASK_WDT', 1030),
            ('test_done_reset', {'stage': controller.Stage.DONE, 'stamp': 1000}, False, 'TASK_WDT', 1030),
            ('test_done_power_on', {'stage': controller.Stage.DONE, 'stamp': 1000}, False, 'POWERON', 1030)):
        cases.append(_gate_case(name, crumb, separated, reason, now_s))
    """The grids carry the gate's own fields only (the rest of a crumb is no input to it); a few carry it all."""
    for stage in stages:
        for separated in (True, False):
            for age in _AGES:
                name = 'stage_%d_%s_age_%d' % (stage, 'separated' if separated else 'nested', age)
                crumb = {'stage': stage, 'stamp': _STAMP_SYNCED, 'armed': True}
                cases.append(_gate_case(name, crumb, separated, 'SW', _STAMP_SYNCED + age))
    for reason in _REASONS:
        for stage in stages:
            for separated in (True, False):
                name = 'reason_%s_stage_%d_%s' % (reason, stage, 'separated' if separated else 'nested')
                crumb = {'stage': stage, 'stamp': _STAMP_SYNCED, 'armed': True}
                cases.append(_gate_case(name, crumb, separated, reason, _STAMP_SYNCED + 30))
    for stage in _STAGE_IDS:
        cases.append(_gate_case('full_crumb_stage_%d' % stage, _crumb(stage, _STAMP_SYNCED), True, 'PANIC',
                                _STAMP_SYNCED + 1))
    torn = (('empty', {}), ('stamp_only', {'stamp': _STAMP_SYNCED}), ('stage_only', {'stage': 3}),
            ('stage_null', dict(_crumb(None, _STAMP_SYNCED), stage=None)),
            ('stamp_null', dict(_crumb(3, None), stamp=None)),
            ('done_without_stamp', {'stage': 5}), ('unknown_stage_without_stamp', {'stage': 9}), ('no_crumb', None))
    for label, crumb in torn:
        for reason in ('SW', 'POWERON'):
            cases.append(_gate_case('torn_%s_%s' % (label, reason), crumb, True, reason, _STAMP_SYNCED + 30))
    generator = random.Random(_SEED)
    for index in range(_SWEEP):
        stage = generator.choice(stages)
        stamp = generator.randrange(0, 2 * _STAMP_SYNCED)
        age = generator.choice((generator.randint(-5, 5), generator.randint(_MAX_AGE_S - 5, _MAX_AGE_S + 5),
                                generator.randint(-10 * _MAX_AGE_S, 10 * _MAX_AGE_S)))
        reason = generator.choice(_REASONS)
        separated = generator.random() < 0.5
        crumb = {'stage': stage, 'stamp': stamp, 'armed': generator.random() < 0.8}
        cases.append(_gate_case('sweep_%03d' % index, crumb, separated, reason, max(0, stamp + age)))
    return cases


class _Nvs:
    """
    esp32.NVS('coludo') as warmstart.py uses it: an i32 and a blob store that records every write.

    A missing key raises OSError (ESP_ERR_NVS_NOT_FOUND), and a blob longer than the read buffer raises OSError
    (ESP_ERR_NVS_INVALID_LENGTH), as the board's NVS does.
    """

    def __init__(self, stage: int | None, blob: bytes | None) -> None:
        self.values: dict = {}
        if stage is not None:
            self.values['stage'] = stage
        if blob is not None:
            self.values['crumb'] = blob
        self.writes: list = []

    def get_i32(self, key: str) -> int:
        """The i32 under `key`."""
        if key not in self.values:
            raise OSError(_NVS_NOT_FOUND)
        return self.values[key]

    def get_blob(self, key: str, buffer: bytearray) -> int:
        """Copy the blob under `key` into `buffer`; its length."""
        if key not in self.values:
            raise OSError(_NVS_NOT_FOUND)
        blob = self.values[key]
        if len(blob) > len(buffer):
            raise OSError(_NVS_INVALID_LENGTH)
        buffer[:len(blob)] = blob
        return len(blob)

    def set_blob(self, key: str, value: object) -> None:
        """Store a blob (plan B hands it the JSON text)."""
        blob = value.encode() if isinstance(value, str) else bytes(value)
        self.values[key] = blob
        self.writes.append(['set_blob', key, json.loads(blob)])

    def set_i32(self, key: str, value: int) -> None:
        """Store an i32."""
        self.values[key] = value
        self.writes.append(['set_i32', key, value])

    def commit(self) -> None:
        """Commit the writes."""
        self.writes.append(['commit'])


_CRUMB_KEYS: tuple = ('stage', 'stamp', 'armed', 'altitude', 'speed', 'airspeed', 'ticks_ms', 'launch', 'zone',
                      'pad_altitude', 'pitot_zero')


def _readable(crumb: dict) -> dict:
    """
    A loaded crumb as the Rust port reads it back: the keys it knows, a null value as no value, `armed` a bool.

    Plan B keeps an unknown key (nothing reads it) and a null value (every reader takes it as absent: .get() is
    None either way), so neither is behaviour; the Rust crumb holds the known fields, each present or not.

    Args:
        crumb - the dict plan B's load() returned.

    Returns:
        The crumb as the comparison holds it.
    """
    readable = {key: crumb[key] for key in _CRUMB_KEYS if crumb.get(key) is not None}
    readable['armed'] = bool(crumb.get('armed'))
    return readable


def _load_case(name: str, stage: int | None, blob: bytes | None) -> dict:
    """
    One load case through plan B's load(), with the Rust answer where it differs.

    Args:
        name - the case name.
        stage - the NVS stage i32, or None when never written.
        blob - the NVS crumb blob, or None when never written.

    Returns:
        {name, input: {stage, blob | blob_hex}, output: {crumb} or {raises}[, divergence]}.
    """
    warmstart._nvs = _Nvs(stage, blob)
    case_input = {'stage': stage}
    try:
        case_input['blob'] = None if blob is None else blob.decode()
    except UnicodeError:
        case_input['blob_hex'] = blob.hex()
    case = {'name': name, 'input': case_input}
    try:
        crumb = warmstart.load()
    except TypeError:
        case['output'] = {'raises': 'TypeError'}
        case['divergence'] = {'crumb': None, 'note': _NOT_AN_OBJECT_NOTE}
        return case
    nan_keys = sorted(key for key, value in (crumb or {}).items() if isinstance(value, float) and math.isnan(value))
    if nan_keys:
        readable = _readable({key: value for key, value in crumb.items() if key not in nan_keys})
        case['output'] = {'crumb': readable, 'nan': nan_keys}
        case['divergence'] = {'crumb': None, 'note': _NAN_NOTE}
        return case
    if crumb is not None:
        _check_exact(crumb, name)
    case['output'] = {'crumb': None if crumb is None else _readable(crumb)}
    return case


def _load_cases() -> list:
    """The load section's cases: the stage gate, torn and oversized blobs, and plan B checkpoint blobs."""
    written = json.dumps(_crumb(controller.Stage.GLIDING, _STAMP_SYNCED)).encode()
    """A board's MicroPython json.dumps: its own key order (no insertion order), f32 text, the same separators."""
    board = (b'{"zone": [[25.514, -80.393], [25.514, -80.391]], "stamp": 844560030, "armed": true, '
             b'"pitot_zero": -1.75, "launch": [25.5, -80.4], "speed": 21.25, "ticks_ms": 73215, "altitude": 412.5, '
             b'"airspeed": 16.5, "pad_altitude": 100.0, "stage": 3}')
    no_identity = json.dumps({'stage': 2, 'armed': True, 'altitude': None, 'speed': None, 'airspeed': None,
                              'ticks_ms': 512, 'stamp': 1000}).encode()
    padded = written + b' ' * (_BLOB_MAX - len(written))
    cases = [
        ('no_stage_written', None, written),
        ('stage_sentinel_zero', 0, written),
        ('stage_without_blob', 3, None),
        ('checkpoint_blob', 3, written),
        ('stage_i32_wins_over_the_blob', 4, written),
        ('negative_stage_reads_back', -1, written),
        ('board_key_order', 3, board),
        ('nulls_and_no_identity', 2, no_identity),
        ('blob_of_exactly_512_bytes', 3, padded),
        ('blob_of_513_bytes', 3, padded + b' '),
        ('torn_blob_cut_short', 3, written[:len(written) // 2]),
        ('empty_blob', 3, b''),
        ('not_utf8', 3, b'\xff' + written),
        ('unknown_keys_ignored', 3, json.dumps(dict(_crumb(3, 1000), version=2, note='x')).encode()),
        ('duplicate_key_last_wins', 3, b'{"stamp": 1, "stamp": 1000, "armed": true}'),
        ('object_without_stamp', 3, b'{"armed": true}'),
        ('empty_object', 5, b'{}'),
        ('json_list', 3, b'[1, 2]'),
        ('json_null', 3, b'null'),
        ('json_number', 3, b'7'),
        ('json_text', 3, b'"crumb"'),
        ('not_json', 3, b'stage=3'),
        ('nan_altitude', 3, b'{"stage": 3, "stamp": 1000, "armed": true, "altitude": NaN}'),
    ]
    return [_load_case(*case) for case in cases]


def _save_cases() -> list:
    """The save section's cases: plan B's checkpoint crumbs through save(), the NVS writes in order."""
    cases = []
    for name, crumb in (('armed_glide', _crumb(controller.Stage.GLIDING, _STAMP_SYNCED)),
                        ('boosting_without_identity', {'stage': 2, 'armed': True, 'altitude': None, 'speed': None,
                                                       'airspeed': None, 'ticks_ms': 512, 'stamp': 1000}),
                        ('landing_partial_identity', {'zone': [[48.0005, 11.0], [48.0, 11.001]], 'pad_altitude': 545.4,
                                                      'stage': 4, 'armed': True, 'altitude': 560.0, 'speed': 3.5,
                                                      'airspeed': 7.2, 'ticks_ms': 1000000, 'stamp': 844560000})):
        _check_exact(crumb, name)
        nvs = _Nvs(None, None)
        warmstart._nvs = nvs
        if not warmstart.save(crumb):
            raise ValueError('%s: plan B refused to save' % name)
        cases.append({'name': name, 'input': crumb, 'output': {'writes': nvs.writes}})
    return cases


class _Recorder:
    """A task stand-in that records each update() it is given, under its name."""

    def __init__(self, name: str, calls: list) -> None:
        self.name: str = name
        self._calls: list = calls

    def update(self, patch: dict) -> list:
        """Record the patch."""
        self._calls.append([self.name, patch])
        return list(patch)

    def seed_airspeed(self, airspeed: float) -> None:
        """The flight task's airspeed seed."""
        self._calls.append([self.name, {'seed_airspeed': airspeed}])


class _Flight:
    """The Controller as _apply_restore and restore see it: find() hands out recorders, the stage and arm are kept."""

    def __init__(self, separated: bool = False) -> None:
        self.calls: list = []
        self.checkpoint: warmstart.Checkpoint = warmstart.Checkpoint('checkpoint', {'name': 'checkpoint'}, None)
        self.separated_reading: bool = separated
        self.stage: int | None = None
        self.armed: bool = False
        self.warm_started: bool = False

    def find(self, names: list) -> list:
        """A recorder per name; the checkpoint is plan B's own Checkpoint, so seed() is plan B's."""
        found = []
        for name in names:
            if name == 'checkpoint':
                found.append(self.checkpoint)
            elif name == 'separation':
                found.append(self)
            else:
                found.append(_Recorder(name, self.calls))
        return found

    def separated(self) -> bool:
        """The separation driver's latch reading."""
        return self.separated_reading

    def set_stage(self, stage: int) -> None:
        """Controller.set_stage, which refuses an id that is no stage."""
        if stage not in controller.Stage.STAGES:
            raise ValueError('unknown stage: %s' % stage)
        self.stage = stage

    def arm(self) -> None:
        """Controller.arm."""
        self.armed = True


class _Mission:
    """The mission as _apply_restore sees it: update() records the patch."""

    name: str = 'mission'

    def __init__(self) -> None:
        self.updates: list = []

    def update(self, patch: dict) -> list:
        """Record the patch."""
        self.updates.append(patch)
        return list(patch)


def _plan(flight: _Flight, mission: _Mission) -> dict:
    """What _apply_restore did, as the apply plan: the mission patch, the updates in order, the stage and arm."""
    return {'mission': mission.updates[0] if mission.updates else None, 'updates': flight.calls,
            'seed': {key: value for key, value in flight.checkpoint._static.items() if value is not None},
            'stage': flight.stage, 'armed': flight.armed, 'warm_started': flight.warm_started}


"""The board config's sensors, as the apply cases give them: every baro driver, a pitot, and a non-baro."""
_SENSORS: list = [{'name': 'imu_lsm6dso32', 'driver': 'lsm6dso32'}, {'name': 'baro_icp10111', 'driver': 'icp10111'},
                  {'name': 'baro_bmp581', 'driver': 'bmp581'}, {'name': 'baro_bmp280', 'driver': 'bmp280'},
                  {'name': 'airspeed_sdp810', 'driver': 'sdp810'}]


def _apply_cases() -> list:
    """The apply section's cases: _apply_restore on passed crumbs (BOOSTING..LANDING), identity in every part."""
    full = _crumb(controller.Stage.GLIDING, _STAMP_SYNCED)
    cases = []
    for name, crumb, sensors in (
            ('armed_glide_every_baro', full, _SENSORS),
            ('boosting', _crumb(controller.Stage.BOOSTING, _STAMP_SYNCED), _SENSORS),
            ('landing_disarmed', _crumb(controller.Stage.LANDING, _STAMP_SYNCED, armed=False), _SENSORS),
            ('test_crumb_one_baro', {'launch': [25.5, -80.4], 'zone': [[25.514, -80.393], [25.514, -80.391]],
                                     'pad_altitude': 100.0, 'stamp': 1000, 'stage': 3, 'armed': True},
             [{'name': 'baro_icp10111', 'driver': 'icp10111'}]),
            ('torn_no_launch_no_zone', {'pad_altitude': 100.0, 'stamp': 1000, 'stage': 3},
             [{'name': 'b', 'driver': 'bmp280'}]),
            ('launch_without_zone', _crumb(3, 1000, zone=None), _SENSORS),
            ('zone_without_launch', {key: value for key, value in full.items() if key != 'launch'}, _SENSORS),
            ('pitot_zero_is_a_tare', _crumb(3, 1000, pitot_zero=0.0), _SENSORS),
            ('no_pitot_no_airspeed',
             {key: value for key, value in full.items() if key not in ('pitot_zero', 'airspeed')}, _SENSORS),
            ('no_pad_altitude', {key: value for key, value in full.items() if key != 'pad_altitude'}, _SENSORS),
            ('no_sensors', full, []),
            ('null_identity', _crumb(4, 1000, pad_altitude=None, pitot_zero=None, airspeed=None), _SENSORS),
            ('southern_western', _crumb(2, 1000, launch=[-34.6037, -58.3816],
                                        zone=[[-34.603, -58.383], [-34.604, -58.381]], pad_altitude=-12.5), _SENSORS)):
        _check_exact(crumb, name)
        mission = _Mission()
        inspector.Inspector.register(mission)
        try:
            flight = _Flight()
            warmstart._apply_restore(flight, crumb, {'sensors': sensors})
        finally:
            inspector.Inspector.unregister('mission')
        cases.append({'name': name, 'input': {'crumb': crumb, 'sensors': sensors}, 'output': _plan(flight, mission)})
    return cases


def _set_clock(now_s: int) -> None:
    """Set the virtual RTC so time.time() reads `now_s` (seconds since 2000), as a synced board's clock would."""
    broken = time.gmtime(now_s)
    machine.RTC().datetime((broken[0], broken[1], broken[2], broken[6], broken[3], broken[4], broken[5], 0))


def _restore_case(name: str, stage: int | None, blob: bytes | None, warm_start: bool | None, separated: bool,
                  reason: str, now_s: int) -> dict:
    """
    One boot through plan B's restore(), with the Rust answer where it differs.

    Args:
        name - the case name.
        stage, blob - the NVS contents (None: never written).
        warm_start - the checkpoint component's warm_start, or None for no such key.
        separated - the separation latch.
        reason - the ESP-IDF reset reason.
        now_s - the clock, seconds since 2000.

    Returns:
        {name, input, output: {warm, writes} or {raises}[, divergence]}.
    """
    nvs = _Nvs(stage, blob)
    warmstart._nvs = nvs
    machine.reset_cause = lambda: _MICROPYTHON_CAUSE.get(reason, 0)
    _set_clock(now_s)
    component = {'name': 'checkpoint', 'activity': 'checkpoint'}
    if warm_start is not None:
        component['warm_start'] = warm_start
    flight = _Flight(separated)
    case = {'name': name, 'input': {'stage': stage, 'blob': None if blob is None else blob.decode(),
                                    'warm_start': warm_start, 'separated': separated, 'cause': reason, 'now_s': now_s}}
    try:
        warm = asyncio.run(warmstart.restore(flight, {'sensors': _SENSORS, 'components': [component]}, log=_quiet))
        case['output'] = {'warm': warm, 'writes': nvs.writes}
    except (TypeError, ValueError) as error:
        case['output'] = {'raises': type(error).__name__}
    finally:
        gc.enable()  # an airborne warm start turned the collector off, as on the board
    crumb = None
    try:
        warmstart._nvs = _Nvs(stage, blob)
        crumb = warmstart.load()
    except TypeError:
        pass
    clear = [['set_i32', 'stage', 0], ['commit']]
    if crumb is None:
        rust = {'warm': False, 'writes': []}
    elif warm_start is False:
        rust = {'warm': False, 'writes': clear}
    else:
        allowed = _rust_gate(crumb, separated, reason, now_s)[0]
        rust = {'warm': allowed, 'writes': [] if allowed else clear}
    if rust != case['output']:
        if 'raises' in case['output'] and crumb is None:
            note = _NOT_AN_OBJECT_NOTE
        else:
            note = _divergence_note(crumb, reason, now_s)
        case['divergence'] = dict(rust, note=note)
    return case


def _quiet(message: str) -> None:
    """restore()'s log sink: the boot lines carry MicroPython's cause code, which is no contract."""


def _restore_cases() -> list:
    """The restore section's cases: the boot decision over the NVS contents, the config toggle and the gate."""
    glide = json.dumps(_crumb(3, _STAMP_SYNCED)).encode()
    cases = [
        ('cold_boot_no_stage', None, glide, True, True, 'POWERON', _STAMP_SYNCED + 30),
        ('cold_boot_sentinel', 0, glide, True, True, 'SW', _STAMP_SYNCED + 30),
        ('torn_blob', 3, glide[:40], True, True, 'SW', _STAMP_SYNCED + 30),
        ('not_an_object', 3, b'[3]', True, True, 'SW', _STAMP_SYNCED + 30),
        ('disabled_by_config', 3, glide, False, True, 'SW', _STAMP_SYNCED + 30),
        ('enabled_by_default', 3, glide, None, True, 'SW', _STAMP_SYNCED + 30),
        ('rejected_power_on', 3, glide, True, True, 'POWERON', _STAMP_SYNCED + 30),
        ('rejected_stale', 3, glide, True, True, 'PANIC', _STAMP_SYNCED + 601),
        ('rejected_nested', 3, glide, True, False, 'TASK_WDT', _STAMP_SYNCED + 30),
        ('done_after_landing', 5, glide, True, True, 'SW', _STAMP_SYNCED + 30),
        ('setting_on_the_pad', 1, glide, True, False, 'TASK_WDT', _STAMP_SYNCED + 30),
        ('stage_that_is_none', 9, glide, True, True, 'SW', _STAMP_SYNCED + 30),
    ]
    for reason in _REASONS:
        for stage in _STAGE_IDS:
            cases.append(('reason_%s_stage_%d' % (reason, stage), stage, glide, True, True, reason,
                          _STAMP_SYNCED + 30))
    return [_restore_case(*case) for case in cases]


class _Stop(Exception):
    """Ends the Checkpoint task's loop once a scenario's last poll has run."""


class _Airspeed:
    """The flight task as the checkpoint reads it: the fused airspeed, set per poll."""

    def __init__(self) -> None:
        self.value: float = 0.0

    def airspeed(self) -> float:
        """The fused airspeed in m/s."""
        return self.value


class _Pitot:
    """The SDP810 as the checkpoint freezes it: its tare."""

    def __init__(self, zero: float) -> None:
        self.zero: float = zero

    def inspect(self) -> dict:
        """The driver's status, the tare among it."""
        return {'zero_offset_pa': self.zero}


class _FreezeMission:
    """The mission as the checkpoint freezes it at BOOSTING: the zone, freeze_launch() and the launch point."""

    name: str = 'mission'

    def __init__(self, zone: list, launch: list | None) -> None:
        self.zone: list = zone
        self.launch: list | None = launch

    def freeze_launch(self) -> None:
        """Pin the live fix as the launch point: the stand-in's launch point is already what it is."""

    def launch_point(self) -> tuple | None:
        """The launch point, or None without a fix or a CC point."""
        return None if self.launch is None else tuple(self.launch)


class _Schedule:
    """The Controller as the Checkpoint task sees it: the stage, the arm, and find() for the flight task and pitot."""

    def __init__(self, airspeed: _Airspeed | None, pitot: _Pitot | None) -> None:
        self.stage: int = controller.Stage.SETTING
        self.armed: bool = False
        self.tasks: dict = {'flight': airspeed, 'airspeed_sdp810': pitot}

    def find(self, names: list) -> list:
        """The tasks of these names, None for one that does not run."""
        return [self.tasks.get(name) for name in names]


class _RowSink:
    """The checkpoint.csv stream: keeps the rows pushed."""

    def __init__(self, rows: list) -> None:
        self.rows: list = rows

    def push(self, values: tuple) -> None:
        """Keep the row."""
        self.rows.append(list(values))


_FRESH_WINDOW_MS: int = 1000  # the stand-in altitude and speed sources' freshness window
_channels: dict = {}  # those sources, made once per process (the databoard keeps them)


def _channel(name: str) -> object:
    """The stand-in source of the `name` parameter (altitude or speed), made on first use."""
    if name not in _channels:
        _channels[name] = databoard.Databoard.provide('stand_in_' + name, {name: {'priority': 0,
                                                                                 'timeout_ms': _FRESH_WINDOW_MS}},
                                                      name)
    return _channels[name]


def _reading(name: str, value: float | None) -> None:
    """A fresh reading of `name` now, or (None) its source gone stale: its last push dated back past the window."""
    channel = _channel(name)
    if value is None:
        channel.t1 = time.ticks_add(time.ticks_us(), -2 * _FRESH_WINDOW_MS * 1000)
    else:
        channel.push(value)


def _check_rounding(value: float, name: str) -> None:
    """
    Refuse an airspeed whose f32 tenth (the Rust crumb's rounding) is not Python's round(value, 1).

    Raises:
        ValueError - the two would round apart (a tie in one width that is not in the other).
    """
    single = struct.unpack('<f', struct.pack('<f', value))[0]
    tenths = struct.unpack('<f', struct.pack('<f', single * 10.0))[0]
    rust = _shortest_f32(round(tenths) / 10.0)
    if rust != round(value, 1):
        raise ValueError('%s: airspeed %r rounds to %r in f32, %r in plan B' % (name, value, rust, round(value, 1)))


def _checkpoint_scenario(name: str, init: dict, steps: list) -> dict:
    """
    One run of plan B's Checkpoint task, poll by poll.

    The task's run() loop runs for real; its asyncio.sleep_ms is the driver: each call first records what the poll
    before it did, then moves the clock by the next poll's advance and sets that poll's stage, arm and readings, and
    after the last poll ends the loop.

    Args:
        name - the scenario name.
        init - {checkpoint_s, zone, launch, pitot_zero, flight, seed}: the component's cadence, the mission's zone and
            launch point (zone None: no mission), the pitot's tare (None: no pitot), whether a flight task runs, and the
            crumb a warm start seeds the identity from (None: none).
        steps - per poll {advance_ms, stage, armed, altitude, speed, airspeed}: a reading None is stale.

    Returns:
        {name, init (with start_us), steps: [{in (with now_us, ticks_ms, stamp_s), out}]}.
    """
    airspeed = _Airspeed() if init['flight'] else None
    schedule = _Schedule(airspeed, None if init['pitot_zero'] is None else _Pitot(init['pitot_zero']))
    checkpoint = warmstart.Checkpoint('checkpoint', {'name': 'checkpoint', 'checkpoint_s': init['checkpoint_s']},
                                      schedule)
    if not asyncio.run(checkpoint.setup()):
        raise ValueError('%s: the checkpoint did not set up' % name)
    rows = []
    saved = []
    checkpoint._telemetry = _RowSink(rows)
    if init['seed'] is not None:
        _check_exact(init['seed'], name)
        checkpoint.seed(init['seed'])
    mission = None if init['zone'] is None else _FreezeMission(init['zone'], init['launch'])
    if mission is not None:
        inspector.Inspector.register(mission)
    recorded = []
    position = [0]

    async def poll(milliseconds: int) -> None:
        """The task's sleep: close the poll before, open the next one (see above)."""
        if position[0] > 0:
            crumb = saved[0] if saved else None
            if crumb is not None:
                _check_exact(crumb, name)
            recorded[-1]['out'] = {'checkpoint': bool(rows), 'row': rows[0] if rows else None, 'crumb': crumb,
                                   'identity': {key: value for key, value in checkpoint._static.items()
                                                if value is not None}}
        if position[0] == len(steps):
            raise _Stop()
        step = steps[position[0]]
        position[0] += 1
        rows.clear()
        saved.clear()
        shims.advance(step['advance_ms'])
        schedule.stage, schedule.armed = step['stage'], step['armed']
        _reading('altitude', step['altitude'])
        _reading('speed', step['speed'])
        if airspeed is not None:
            _check_rounding(step['airspeed'], name)
            airspeed.value = step['airspeed']
        recorded.append({'in': dict(step, now_us=time.ticks_us(), ticks_ms=time.ticks_ms(),
                                    stamp_s=int(time.time()))})

    real_save, real_sleep = warmstart.save, asyncio.sleep_ms
    warmstart.save = lambda crumb: saved.append(dict(crumb)) or True
    asyncio.sleep_ms = poll
    start_us = time.ticks_us()  # run() takes its last checkpoint time before its first sleep
    try:
        asyncio.run(checkpoint.run())
    except _Stop:
        pass
    finally:
        warmstart.save, asyncio.sleep_ms = real_save, real_sleep
        if mission is not None:
            inspector.Inspector.unregister('mission')
    return {'name': name, 'init': dict(init, start_us=start_us), 'steps': recorded}


def _polls(stage: int, armed: bool, count: int, altitude: float | None = 100.0, speed: float | None = 0.0,
           airspeed: float = 0.0, advance_ms: int = 500) -> list:
    """`count` polls at `stage`, one every advance_ms, with these readings."""
    return [{'advance_ms': advance_ms, 'stage': stage, 'armed': armed, 'altitude': altitude, 'speed': speed,
             'airspeed': airspeed} for _index in range(count)]


_STAGES = controller.Stage
_FREEZE: dict = {'checkpoint_s': 1, 'zone': [[25.514, -80.393], [25.514, -80.391]], 'launch': [25.5, -80.4],
                 'pitot_zero': -1.75, 'flight': True, 'seed': None}


def _flight(armed: bool) -> list:
    """A whole flight's polls: the pad (a stale reading in it), the boost, the glide, the landing, DONE."""
    return (_polls(_STAGES.SETTING, False, 2, altitude=101.5) + _polls(_STAGES.SETTING, armed, 1, altitude=None) +
            _polls(_STAGES.SETTING, armed, 2, altitude=102.0) +
            _polls(_STAGES.BOOSTING, armed, 3, altitude=180.0, speed=40.25, airspeed=35.5) +
            _polls(_STAGES.GLIDING, armed, 5, altitude=420.0, speed=None, airspeed=16.25) +
            _polls(_STAGES.LANDING, armed, 3, altitude=None, speed=3.5, airspeed=7.75) +
            _polls(_STAGES.DONE, armed, 4, altitude=95.0, speed=0.0, airspeed=0.0))


def _empty_freeze() -> dict:
    """
    A freeze that finds nothing (no mission, a stale altitude, no pitot), then fresh readings later in the boost.

    Plan B freezes only on the poll that ENTERS BOOSTING (`changed and ...`), so the identity stays empty for the
    rest of the boost; re-freezing on a later poll would take the mid-boost 250 m as the pad.

    Returns:
        The scenario.

    Raises:
        ValueError - plan B held an identity on some poll, so the scenario no longer pins the entry-only freeze.
    """
    scenario = _checkpoint_scenario('empty_freeze_not_retried', dict(_FREEZE, zone=None, pitot_zero=None),
                                    _polls(_STAGES.SETTING, True, 1, altitude=None) +
                                    _polls(_STAGES.BOOSTING, True, 1, altitude=None) +
                                    _polls(_STAGES.BOOSTING, True, 3, altitude=250.0))
    if any(step['out']['identity'] for step in scenario['steps']):
        raise ValueError('empty_freeze_not_retried: plan B froze an identity')
    return scenario


def _checkpoint_scenarios() -> list:
    """
    The checkpoint section: whole flights armed and not, freezes without a fix or a mission, a freeze that found
    nothing, a seeded warm start, the cadence.
    """
    seed = {'launch': [48.0, 11.0], 'zone': [[48.0005, 11.0], [48.0, 11.001]], 'pad_altitude': 545.5,
            'pitot_zero': 0.0, 'stage': 3, 'stamp': 1000}
    return [
        _checkpoint_scenario('armed_flight', _FREEZE, _flight(True)),
        _checkpoint_scenario('disarmed_flight', _FREEZE, _flight(False)),
        _checkpoint_scenario('no_fix_no_pitot_no_flight_task', dict(_FREEZE, launch=None, pitot_zero=None,
                                                                    zone=[[25.514, -80.393], [25.5144, -80.3912]],
                                                                    flight=False), _flight(True)),
        _checkpoint_scenario('no_mission_no_pad_latch', dict(_FREEZE, zone=None),
                             _polls(_STAGES.BOOSTING, True, 4, altitude=230.0) + _polls(_STAGES.GLIDING, True, 2)),
        _empty_freeze(),
        _checkpoint_scenario('warm_start_seeded', dict(_FREEZE, seed=seed),
                             _polls(_STAGES.BOOSTING, True, 3, altitude=700.0, airspeed=30.0) +
                             _polls(_STAGES.SETTING, False, 2, altitude=96.5) +
                             _polls(_STAGES.BOOSTING, True, 2, altitude=150.0)),
        _checkpoint_scenario('slow_cadence', dict(_FREEZE, checkpoint_s=2.5),
                             _polls(_STAGES.BOOSTING, True, 12) + _polls(_STAGES.GLIDING, True, 1, advance_ms=4000)),
        _checkpoint_scenario('cadence_floored', dict(_FREEZE, checkpoint_s=0.25),
                             _polls(_STAGES.SETTING, True, 1) + _polls(_STAGES.GLIDING, True, 6, airspeed=12.34)),
    ]


def build() -> dict:
    """
    The warmstart fixture document.

    Args:
        (none)

    Returns:
        {meta, gate, load, save, apply, restore, checkpoint}: each a golden fixture of its own.
    """
    machine.WDT_RESET, machine.SOFT_RESET, machine.HARD_RESET = _WDT_RESET, _SOFT_RESET, _HARD_RESET
    return {
        'meta': {'source': 'src/glider/warmstart.py, src/glider/controller.py'},
        'gate': {
            'compare': {'kind': 'exact'},
            'note': ('should_restore(crumb, separated, cause, now_s): restore and the reason text, exactly; cause is '
                     'the ESP-IDF reset reason, now_s seconds since 2000 like the stamp'),
            'cases': _gate_cases(),
        },
        'load': {
            'compare': {'kind': 'exact'},
            'note': ('load() over the NVS stage i32 and crumb blob: the crumb read back (its known keys, a null as no '
                     'value, armed a bool), or null'),
            'cases': _load_cases(),
        },
        'save': {
            'compare': {'kind': 'exact'},
            'note': 'save(crumb): the NVS writes in order, the blob as the JSON it parses to',
            'cases': _save_cases(),
        },
        'apply': {
            'compare': {'kind': 'exact'},
            'note': ('_apply_restore on a passed crumb: the mission patch, the task updates in order (each baro of '
                     'the config, the pitot tare, the airspeed seed), the identity seeded (a null one as none), the '
                     'stage and arm'),
            'cases': _apply_cases(),
        },
        'restore': {
            'compare': {'kind': 'exact'},
            'note': 'restore(): whether the boot warm-starts, and the NVS writes it makes (a refusal clears the stage)',
            'cases': _restore_cases(),
        },
        'checkpoint': {
            'compare': {'kind': 'exact'},
            'note': ('the Checkpoint task, poll by poll: checkpointed, the checkpoint.csv row (stage, altitude, speed, '
                     'airspeed to 0.1, ticks_ms), the crumb saved (armed only), the identity held (a null as none)'),
            'scenarios': _checkpoint_scenarios(),
        },
    }
