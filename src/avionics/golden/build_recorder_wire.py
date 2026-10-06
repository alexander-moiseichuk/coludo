"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture recorder_wire.json: the recorder's line wire as plan B writes it (src/glider/recorder.py, through its
real path: the rings, Telemetry.push, Recorder.log, index_session and the drain's viper _wrap) and as the host
reference reads it (tools/recorder_wire.py), for the Rust port (crates/recorder_wire).

Members, each a list of {name, input, output} cases:
  * crc32 -- binascii.crc32(bytes, previous), the CRC under every wrapper.
  * wraps -- (routing, payload) to the wire line: test_recorder.py's literal vectors, read from its source (the
    module runs its tests on import), then uptimes past 2**30, 2**32 and 2**64. Every line is plan B's drain output
    AND the host reference's wrap(); the builder refuses a case where they, or a literal, disagree.
  * damage -- a damaged wire line to the host reference's split() verdict (classify() in Rust): status, routing,
    payload. A BAD line carries `salvage` too: what recover() gets back from it offline against the boot's known
    routings. The board does not salvage (it keeps a line only when it verifies under its own routing), so the Rust
    test checks only that its verify() agrees with every record the host salvaged.
  * telemetry -- a stream's fields and rows of typed cells to its wire lines, header first, from Telemetry.push.
  * logs -- Recorder.log(descriptor, message) under a ring cell's limit to the wire line.
  * session_rows -- index_session(), the boot row and the anchor to session.csv's header and row lines.
  * statuses -- Recorder.report() to its str(), the status message; one case also carries members plan B never
    had, after 'log' (the flash writer's 'ring', M4's 'mover'), each as its repr() text, and its oracle is str() of
    the report with them added. Their keys are placeholders: the case pins how members join, not their figures.
  * cells, labels, prefixes -- _cell(), _is_label(), and the session prefix a recorder.session value gives.

A byte string is its UTF-8 text, or {"hex": ...} when it is not UTF-8 (a cut inside a character, a damaged byte).
A typed cell is {"kind", "value"}: the kinds of the Rust ring record (none, i32, i64, f32, bool, e7, text). Plan B
is handed what its own producers push for each: an int for i32, i64 and a flag (`1 if x else 0`), a float for f32,
float degrees for e7 (gnss.py's lat/lon), a str for text, None for none. A float JSON cannot hold is a string.

DELIBERATE DIVERGENCES. A case whose Rust answer differs from its oracle's (plan B, or the host reference for
`damage`) carries `divergence`, the reason, and `oracle`, the oracle's own answer; `output` is then the Rust answer.
The builder derives it here, independently of the Rust code: the f32 text as the shortest digits that read back as
the same single-precision value, and degrees from the 1e-7 unit integer. It refuses a case whose answers differ
without a declared divergence, and one that declares a divergence its answers do not show. The reasons:
  * an F32 cell is Rust's Display: no '.0', no exponent, 'NaN', '-0'; and CPython's f64 str() shows digits no f32
    holds (0.1 + 0.2), where the board's single precision never does;
  * an E7 cell and session.csv's cc_lat/cc_lon print the 1e-7 degree integer with 7 decimals;
  * a Text cell has ';' and control bytes made spaces, a log line its control bytes;
  * a wrapper tag that unwrap() and verify() read is exactly 8 ASCII hex digits, where the host's int(tag, 16) also
    takes a space, a sign or an underscore, so one flipped bit of a leading '0' (to a space) verifies there. Only
    that GOOD verdict differs: wrapped() reads a tag as leniently as the host's _hex8 (ASCII), so a line left with
    only a lenient tag is BAD on both sides, never LEGACY in Rust (the *_and_no_tail / *_and_no_head cases);
  * a recorder.session value that is not text names no session (plan B took str(value), so true named 'True').

The clock is the shims' virtual one, so every uptime here is a ticks_us below 2**30 that the case records as its
input. asyncio.ThreadSafeFlag, which CPython lacks and Recorder.setup constructs, is stood in for below (_Flag): the
shims docstring asks the first builder that runs Recorder.setup to add it there, and shims.py is shared golden
infrastructure this builder's stage does not edit, so moving _Flag into shims.install() is left to its owner.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import ast
import asyncio
import binascii
import importlib.util
import math
import os
import random
import struct
import time

import config as config_module
import config_default
import recorder
import shims

_REPOSITORY: str = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
_HOST_REFERENCE_PATH: str = os.path.join(_REPOSITORY, 'tools', 'recorder_wire.py')
_BOARD_TEST_PATH: str = os.path.join(_REPOSITORY, 'src', 'glider', 'test', 'test_recorder.py')
_BOARD_VECTORS: tuple = ('_ROUTED_VECTORS', '_LOG_VECTORS')  # test_recorder.py's literal (raw, line) tuples
_BOOT_ID: int = 123
_EPOCH_2000: int = 946684800  # Unix seconds at 2000-01-01, the MicroPython RTC's epoch
_CLOCK_SET: int = 844352467  # 2026-10-03T14:21:07Z on the RTC's 2000 epoch (test_recorder's _CLOCK_SET)
_FIRST_SET: int = 31622400  # 2001-01-01T00:00:00Z on the RTC's epoch: its first dated second
_UNITS_PER_DEGREE: int = 10000000  # a Rust E7 cell counts 1e-7 degrees
_ROW_GAP_MS: int = 10  # the virtual time between a stream's rows, so no two share an uptime
_SEED: int = 20261006  # the legacy session prefix's random.seed
_KNOWN_ROUTINGS: list = ['000123_imu_lsm6dso32.csv', '000123_health.csv', 'session.csv']  # the boot's routings
_PREFIXES: tuple = ('000123_',)  # the session prefix an unlisted `<name>@` may still prove a stream under
_IMU: str = '000123_imu_lsm6dso32.csv'
_HEALTH: str = '000123_health.csv'
_FLOAT_NOTE: str = ("an F32 cell is Rust's Display of the f32 (shortest, never an exponent, no '.0', 'NaN', '-0'); "
                    'plan B printed str(float), and on CPython that float is an f64')
_E7_NOTE: str = 'an E7 cell prints the 1e-7 degree integer with 7 decimals; plan B printed its float degrees'
_TEXT_NOTE: str = "a Text cell has ';' and control bytes made spaces; plan B wrote them as they came"
_LOG_NOTE: str = 'a log line has its control bytes made spaces; plan B kept them, so a newline split its wire line'
_CC_NOTE: str = 'cc_lat/cc_lon print the 1e-7 degree integers with 7 decimals; plan B printed its float degrees'
_TAG_NOTE: str = ('a wrapper tag is exactly 8 ASCII hex digits; the host reads it with int(tag, 16), which also '
                  'takes %s, so this damaged line verifies there')
_LABEL_NOTE: str = 'a recorder.session value that is not text names no session; plan B took str(value)'


class _Wire:
    """Stands in for the recorder UART's StreamWriter, as test_recorder's FakeWriter does: drain() reads out_buf."""

    def __init__(self) -> None:
        self.lines: list = []
        self.out_buf: bytes = b''

    def write(self, data: bytes) -> None:
        """Take one wrapped line, as the UART would."""
        self.lines.append(bytes(data))
        self.out_buf += bytes(data)

    async def drain(self) -> None:
        """The UART caught up."""
        self.out_buf = b''


class _Flag:
    """asyncio.ThreadSafeFlag, which CPython lacks: Recorder.setup builds one, producers set it, nothing waits."""

    def set(self) -> None:
        """A producer queued a record."""


def _host_reference() -> object:
    """
    tools/recorder_wire.py, loaded from its file under a name of its own, so no tools/ module can shadow plan B's.

    Returns:
        The module.
    """
    spec = importlib.util.spec_from_file_location('host_recorder_wire', _HOST_REFERENCE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_REFERENCE: object = _host_reference()


def _board_vectors() -> dict:
    """
    test_recorder.py's literal wire vectors, read from its source: importing it would run the board tests.

    Returns:
        {'_ROUTED_VECTORS': ((raw, line), ...), '_LOG_VECTORS': (...)}.

    Raises:
        ValueError - the file no longer assigns one of them.
    """
    with open(_BOARD_TEST_PATH) as handle:
        tree = ast.parse(handle.read())
    vectors = {node.targets[0].id: ast.literal_eval(node.value) for node in tree.body
               if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
               and node.targets[0].id in _BOARD_VECTORS}
    if set(vectors) != set(_BOARD_VECTORS):
        raise ValueError('test_recorder.py lost %s' % ', '.join(sorted(set(_BOARD_VECTORS) - set(vectors))))
    return vectors


def _bytes(data: bytes) -> object:
    """A byte string as the fixture holds it: its text when it is UTF-8, else {'hex': ...}."""
    try:
        return data.decode('utf-8')
    except UnicodeDecodeError:
        return {'hex': data.hex()}


def _config(**recorder_section: object) -> dict:
    """The default board config, with these recorder keys set."""
    config = config_default.default()
    config['recorder'].update(recorder_section)
    return config


def _setup(config: dict | None = None, boot_id: int | None = _BOOT_ID) -> _Wire:
    """
    A fresh Recorder on a stand-in UART, as test_recorder sets one up.

    Args:
        config - the board config; the default one when None.
        boot_id - this boot's NVS id; None for a boot nothing counted.

    Returns:
        The stand-in UART the drain writes to. The global decimation is off, so every push emits a row.
    """
    wire = _Wire()
    recorder.Recorder.boot_id = boot_id
    recorder.Recorder.setup(config if config is not None else config_default.default(), uart=wire)
    recorder.Recorder.telemetry_decimate_us = 0
    return wire


def _drained(wire: _Wire) -> list:
    """The lines a drain writes now, wrapped as on the wire; the stand-in UART is emptied."""
    asyncio.run(recorder.Recorder.drain())
    lines, wire.lines = wire.lines, []
    return lines


def _host_wrap(routing: str | None, payload: bytes) -> bytes:
    """The host reference's wire line for `payload` under `routing`, as bytes."""
    return _REFERENCE.wrap(payload.decode('utf-8', 'surrogateescape'), routing).encode('utf-8', 'surrogateescape')


def _payload(line: bytes) -> bytes:
    """The payload of a wire line that must verify."""
    status, _routing, payload = _REFERENCE.split(line.decode('utf-8', 'surrogateescape'))
    if status != _REFERENCE.GOOD:
        raise ValueError('plan B wrote a line the host reference calls %s: %r' % (status, line))
    return payload.encode('utf-8', 'surrogateescape')


def _f32(value: float) -> float:
    """`value` rounded to the nearest single-precision float, as the board and the Rust port hold it."""
    return struct.unpack('<f', struct.pack('<f', value))[0]


def _f32_text(value: float) -> str:
    """
    Rust's Display of the f32 nearest `value`: the shortest digits that read back as that f32, positional.

    Args:
        value - the float.

    Returns:
        Its text: 'NaN', 'inf' and '-inf' as Rust spells them, '-0' for negative zero, no '.0' on an integral value
        and never an exponent.
    """
    single = _f32(value)
    if math.isnan(single):
        return 'NaN'
    if math.isinf(single):
        return 'inf' if single > 0 else '-inf'
    if single == 0:
        return '-0' if math.copysign(1.0, single) < 0 else '0'
    for digits in range(1, 10):  # nine significant digits always identify an f32
        text = '%.*e' % (digits - 1, single)
        if _f32(float(text)) == single:
            break
    mantissa, exponent = text.split('e')
    sign = '-' if mantissa.startswith('-') else ''
    figures = mantissa.lstrip('-').replace('.', '')
    point = int(exponent) + 1  # the figures read as 0.<figures> * 10**point
    if point <= 0:
        return sign + '0.' + '0' * -point + figures
    if point >= len(figures):
        return sign + figures + '0' * (point - len(figures))
    return sign + figures[:point] + '.' + figures[point:]


def _e7_text(units: int) -> str:
    """1e-7 degree `units` as signed degrees with 7 decimals, as the Rust E7 cell prints them."""
    magnitude = abs(units)
    return '%s%d.%07d' % ('-' if units < 0 else '', magnitude // _UNITS_PER_DEGREE, magnitude % _UNITS_PER_DEGREE)


def _cleaned(text: str, replaced: str = '') -> str:
    """`text` with every control character (below 0x20, and DEL) and every one of `replaced` made a space."""
    return ''.join(' ' if ord(character) < 0x20 or character == '\x7f' or character in replaced else character
                   for character in text)


def _value(kind: str, value: object) -> object:
    """The JSON value a cell's input carries: a float JSON cannot hold goes as its Rust spelling."""
    if kind == 'f32' and (math.isnan(value) or math.isinf(value)):
        return _f32_text(value)
    return value


def _plan_b_value(kind: str, value: object) -> object:
    """What plan B's own producers push for a cell of this kind (see the module notes)."""
    if kind == 'bool':
        return 1 if value else 0
    if kind == 'e7':
        return value / _UNITS_PER_DEGREE
    return value


def _rust_text(kind: str, value: object) -> str:
    """The text the Rust row writes for a cell (see the module notes)."""
    if kind == 'none':
        return 'None'
    if kind in ('i32', 'i64'):
        return str(value)
    if kind == 'f32':
        return _f32_text(value)
    if kind == 'bool':
        return '1' if value else '0'
    if kind == 'e7':
        return _e7_text(value)
    return _cleaned(value, ';')


def _settle(case: dict, oracle: object, rust: object, note: str | None) -> dict:
    """
    Fill a case's answer: the oracle's, or the Rust one with the declared divergence beside the oracle's.

    Args:
        case - the case so far, {name, input}.
        oracle - the oracle's answer (plan B's, or the host reference's).
        rust - the answer the Rust port owes.
        note - the declared divergence; None when the two must agree.

    Returns:
        The case, with `output` (and `divergence`, `oracle` when they differ).

    Raises:
        ValueError - the answers differ with no divergence declared, or agree while one is.
    """
    if (oracle != rust) != (note is not None):
        raise ValueError('%s: %s\n  oracle %r\n  rust   %r' % (
            case['name'], 'a divergence nobody declared' if note is None else 'the declared divergence is not there',
            oracle, rust))
    case['output'] = rust
    if note is not None:
        case['divergence'] = note
        case['oracle'] = oracle
    return case


_CRC_CASES: tuple = (
    ('empty', 0, b''),
    ('check_value', 0, b'123456789'),
    ('one_byte', 0, b'a'),
    ('every_byte', 0, bytes(range(256))),
    ('continued', binascii.crc32(b'123'), b'456789'),
    ('continued_from_all_ones', 0xFFFFFFFF, b'x'),
    ('a_routed_cover', 0, b'000123_imu_lsm6dso32.csv@884029;0.98;0.05;0.01'),
)


def _crc32() -> list:
    """The crc32 cases: binascii.crc32(bytes, previous)."""
    return [{'name': name, 'input': {'previous': previous, 'bytes': _bytes(data)},
             'output': binascii.crc32(data, previous)} for name, previous, data in _CRC_CASES]


"""Payloads past the ticks_us period and past 2**32 and 2**64: (name, routing, payload)."""
_LARGE_UPTIMES: tuple = (
    ('below_2_30', _HEALTH, b'1073741823;41.5;None'),
    ('at_2_30', _HEALTH, b'1073741824;41.5;None'),
    ('at_2_31', _HEALTH, b'2147483648;41.5;None'),
    ('below_2_32', 'session.csv', b'4294967295;123;000123;;;taster;dev;95e1af1d7b1d;boot;;'),
    ('past_2_32', _IMU, b'4294967297;0.98;0.05;0.01'),
    ('ninety_minutes', _IMU, b'5400000000;0.98;0.05;0.01'),
    ('u64_max', _HEALTH, b'18446744073709551615;41.5'),
    ('past_u64', _HEALTH, b'18446744073709551616;41.5'),
    ('leading_zeros', _HEALTH, b'0000000000004294967296;1'),
    ('zero', _HEALTH, b'0;0'),
    ('a_header', _HEALTH, b'uptime;temp;mem_free'),
    ('an_empty_routed_payload', 'a.csv', b''),
    ('log_past_2_30', None, b'1073741824 sequencer :: stage -> boosting'),
    ('log_u64_max', None, b"18446744073709551615 Recorder :: {'session': '000123'}"),
    ('log_twenty_five_digits', None, b'1234567890123456789012345 x :: y'),
    ('an_empty_log_payload', None, b''),
)


def _record(routing: str | None, payload: bytes) -> bytes:
    """The raw ring record for a payload, as plan B's producers queue it: `[@<routing>@]<payload>\\n`."""
    return (b'' if routing is None else b'@%s@' % routing.encode()) + payload + b'\n'


def _plan_b_line(raw: bytes) -> bytes:
    """
    Plan B's wire line for one raw record: queued on its ring and drained, the path a flight takes.

    Args:
        raw - the record, `\\n` included; a routed one goes on the telemetry ring, any other on the log ring.

    Returns:
        The one line the drain wrote.
    """
    wire = _setup()
    ring = recorder.Recorder._tlm if raw.startswith(b'@') else recorder.Recorder._log
    if not ring.write(raw):
        raise ValueError('the ring refused %r' % raw)
    lines = _drained(wire)
    if len(lines) != 1:
        raise ValueError('%r drained as %r' % (raw, lines))
    return lines[0]


def _wrap_case(name: str, routing: str | None, payload: bytes, literal: bytes | None = None) -> dict:
    """
    One wrap case, its line agreed by plan B's drain, the host reference and the literal when there is one.

    Raises:
        ValueError - they disagree.
    """
    line = _plan_b_line(_record(routing, payload))
    if line != _host_wrap(routing, payload) or (literal is not None and line != literal):
        raise ValueError('%s: plan B %r, host %r, literal %r' % (name, line, _host_wrap(routing, payload), literal))
    return {'name': name, 'input': {'routing': routing, 'payload': _bytes(payload)}, 'output': _bytes(line)}


def _parts(raw: bytes) -> tuple:
    """(routing, payload) of a raw record, `[@<routing>@]<payload>\\n`, by the drain's own rule."""
    text = raw[:-1] if raw.endswith(b'\n') else raw
    if text.startswith(b'@') and text.find(b'@', 1) > 0:
        end = text.find(b'@', 1)
        return text[1:end].decode(), text[end + 1:]
    return None, text


def _wraps() -> list:
    """The wrap cases: test_recorder.py's literal vectors, then the large uptimes."""
    vectors = _board_vectors()
    cases = []
    for kind, member in (('routed', '_ROUTED_VECTORS'), ('log', '_LOG_VECTORS')):
        for index, (raw, literal) in enumerate(vectors[member]):
            cases.append(_wrap_case('%s_vector_%d' % (kind, index), *_parts(raw), literal))
    return cases + [_wrap_case(*case) for case in _LARGE_UPTIMES]


def _leading_zero_line(routing: str, tag: str, zeros: int) -> bytes:
    """
    A good wire line whose OPEN or CLOSE tag starts with `zeros` zeros: the shape the host's lenient int() reads
    past. The payload's second cell counts up until one does.

    Args:
        routing - the line's routing.
        tag - 'open' or 'close'.
        zeros - how many leading zeros.

    Returns:
        The line.
    """
    count = 0
    while True:
        line = _host_wrap(routing, b'884029;%d' % count)
        start = line.index(b'{') + 1 if tag == 'open' else len(line) - 10
        if line[start:start + zeros] == b'0' * zeros:
            return line
        count += 1


def _replaced(line: bytes, old: bytes, new: bytes, occurrence: int = 0) -> bytes:
    """`line` with its `occurrence`th (0-based) `old` replaced by `new`."""
    start = -1
    for _ in range(occurrence + 1):
        start = line.index(old, start + 1)
    return line[:start] + new + line[start + len(old):]


def _damage_lines() -> tuple:
    """(name, line, divergence note) per damage case: good lines first, then each kind of damage."""
    row = _host_wrap(_IMU, b'884029;0.98;0.05;0.01')
    log = _host_wrap(None, b'884030 sequencer :: stage -> boosting')
    header = _host_wrap(_IMU, b'uptime;ax;ay;az')
    health = _host_wrap(_HEALTH, b'884031;41.5;8388608;3;None;None;0;0;0')
    open_start = row.index(b'{') + 1
    close_start = len(row) - 10
    spaced = _leading_zero_line(_IMU, 'open', 1)
    signed = _leading_zero_line(_IMU, 'close', 1)
    underscored = _leading_zero_line(_HEALTH, 'open', 2)
    spaced_at = spaced.index(b'{') + 1
    signed_at = len(signed) - 10
    underscored_at = underscored.index(b'{') + 2
    no_tail = row[:-2] + b'\n'  # the tail's '>' lost: only the head is left as wrapper evidence
    no_head = row[:open_start - 1] + row[open_start:]  # the head's '{' lost: only the tail is left
    return (
        ('a_good_row', row, None),
        ('a_good_log_line', log, None),
        ('a_good_header', header, None),
        ('a_good_row_with_crlf', row[:-1] + b'\r\n', None),
        ('uppercase_tags', row[:open_start] + row[open_start:open_start + 8].upper() + row[open_start + 8:close_start]
         + row[close_start:close_start + 8].upper() + row[close_start + 8:], None),
        ('a_payload_digit_flipped', _replaced(row, b'0.98', b'0.99'), None),
        ('the_uptime_flipped', _replaced(row, b'884029', b'884028'), None),
        ('the_open_tag_flipped', row[:open_start] + b'0' + row[open_start + 1:] if row[open_start] != 0x30
         else row[:open_start] + b'1' + row[open_start + 1:], None),
        ('the_close_tag_flipped', row[:close_start + 7] + (b'0' if row[close_start + 7] != 0x30 else b'1')
         + row[close_start + 8:], None),
        ('a_log_line_flipped', _replaced(log, b'boosting', b'boostinG'), None),
        ('a_high_byte_in_the_payload', _replaced(row, b'0.05', b'0.\xff5'), None),
        ('a_corrupted_routing', _replaced(row, b'lsm6dso32', b'lsm6dso3Z'), None),
        ('the_tail_cut', row[:-7] + b'\n', None),
        ('cut_in_the_payload', row[:open_start + 20] + b'\n', None),
        ('the_head_only', row[:open_start + 10] + b'\n', None),
        ('two_rows_merged', row[:-1] + health, None),
        ('a_row_merged_with_a_log_line', row[:-1] + log, None),
        ('a_spliced_row', row[:-20] + health, None),
        ('the_leading_at_lost', row[1:], None),
        ('the_second_at_lost', _replaced(row, b'@', b'', 1), None),
        ('another_sessions_name_left', b'000999_imu_lsm6dso32.csv@' + row[row.index(b'{'):], None),
        ('an_empty_routing', _host_wrap('', b'7;x'), None),
        ('a_legacy_row', b'@000123_imu_lsm6dso32.csv@884029;0.98;0.05;0.01\n', None),
        ('a_legacy_log_line', b'884030 sequencer :: stage -> boosting\n', None),
        ('an_empty_line', b'\n', None),
        ('a_routing_alone', b'@000123_health.csv@\n', None),
        ('a_leading_zero_made_a_space', spaced[:spaced_at] + b' ' + spaced[spaced_at + 1:], _TAG_NOTE % 'a space'),
        ('a_leading_zero_made_a_sign', signed[:signed_at] + b'+' + signed[signed_at + 1:], _TAG_NOTE % 'a sign'),
        ('a_zero_made_an_underscore', underscored[:underscored_at] + b'_' + underscored[underscored_at + 1:],
         _TAG_NOTE % 'an underscore between digits'),
        ('a_spaced_head_and_no_tail', no_tail[:open_start + 7] + b' ' + no_tail[open_start + 8:], None),
        ('a_head_with_a_carriage_return_and_no_tail', no_tail[:open_start + 7] + b'\r' + no_tail[open_start + 8:],
         None),
        ('a_0x_head_and_no_tail', no_tail[:open_start] + b'0x' + no_tail[open_start + 2:], None),
        ('an_underscored_head_and_no_tail', no_tail[:open_start + 3] + b'_' + no_tail[open_start + 4:], None),
        ('a_signed_tail_and_no_head', no_head[:close_start - 1] + b'+' + no_head[close_start:], None),
        ('a_head_int_refuses_and_no_tail', no_tail[:open_start + 3] + b' ' + no_tail[open_start + 4:], None),
        ('a_tail_int_refuses_and_no_head', no_head[:close_start - 1] + b'__' + no_head[close_start + 1:], None),
    )


def _verdict(status: str, routing: str | None, payload: str) -> dict:
    """A split() verdict as the fixture holds it, its text back to the line's bytes."""
    def raw(text: str) -> object:
        return _bytes(text.encode('utf-8', 'surrogateescape'))
    return {'status': status, 'routing': None if routing is None else raw(routing), 'payload': raw(payload)}


def _strict_verdict(line: str) -> dict:
    """The Rust verdict on a line whose tag the host's int() reads past: BAD, with the body as it came."""
    status, routing, _payload_text = _REFERENCE.split(line)
    stripped = line.rstrip('\r\n')
    body = stripped if routing is None else stripped[len(routing) + 2:]
    if status != _REFERENCE.GOOD:
        raise ValueError('the host was meant to verify %r' % line)
    return _verdict(_REFERENCE.BAD, routing, body)


def _damage() -> list:
    """The damage cases: the host reference's split() verdict, and recover()'s for a BAD line."""
    cases = []
    for name, line, note in _damage_lines():
        text = line.decode('utf-8', 'surrogateescape')
        status, routing, payload = _REFERENCE.split(text)
        oracle = _verdict(status, routing, payload)
        case = _settle({'name': name, 'input': _bytes(line)}, oracle,
                       _strict_verdict(text) if note is not None else oracle, note)
        if status == _REFERENCE.BAD:
            records, lost = _REFERENCE.recover(routing, payload, _KNOWN_ROUTINGS, _PREFIXES)
            case['salvage'] = {'lost': lost, 'records': [
                {'routing': found, 'payload': _bytes(proven.encode('utf-8', 'surrogateescape')),
                 'record': _bytes(record.encode('utf-8', 'surrogateescape'))} for found, proven, record in records]}
        cases.append(case)
    return cases


"""
Streams as plan B's producers push them: (name, session label or None for the boot id, file, fields, rows, note).
Each row is a list of (kind, value) cells.
"""
_STREAMS: tuple = (
    ('an_imu', None, 'imu_lsm6dso32.csv', ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'irq_runs'),
     [[('f32', 0.984784), ('f32', 0.065392), ('f32', -0.005856), ('i32', -21), ('i32', 14), ('i32', -35), ('i32', 1)],
      [('f32', 0.982832), ('f32', 0.062464), ('f32', -0.002928), ('i32', -28), ('i32', 14), ('i32', -21), ('i32', 2)]],
     None),
    ('health', None, 'health.csv', ('temp', 'mem_free', 'load', 'oom_s', 'land_s', 'leak_kbps', 'rescues', 'rescue_ms'),
     [[('f32', 41.5), ('i64', 32549200), ('i32', 58), ('none', None), ('none', None), ('i32', 0), ('i32', 0),
       ('i32', 0)],
      [('f32', 42.3), ('i64', 8388608), ('i32', 15), ('none', None), ('i32', 187), ('i32', 0), ('i32', 0), ('i32', 0)]],
     None),
    ('a_barometer', None, 'baro_bmp280.csv', ('altitude', 'temp_c', 'pressure', 'elevation'),
     [[('f32', -36.80688), ('f32', 38.29), ('f32', 101767.91), ('f32', 4.244812)]], None),
    ('the_sequencer', None, 'sequencer.csv', ('stage', 'reason'),
     [[('text', 'boosting'), ('text', 'launch: accel>3g for 300ms')], [('text', 'gliding'), ('text', '')]], None),
    ('a_servo', None, 'servo_eleron_left.csv', ('angle', 'pulse_us', 'done'),
     [[('i32', -15), ('i32', 1333), ('bool', False)], [('i32', -15), ('i32', 1333), ('bool', True)]], None),
    ('wide_integers', None, 'counters.csv', ('a', 'b', 'c', 'd'),
     [[('i64', 1 << 40), ('i64', -(1 << 63)), ('i32', -(1 << 31)), ('i32', (1 << 31) - 1)]], None),
    ('a_labelled_session', 'catapult-run3', 'fins.csv', ('eleron_left', 'eleron_right', 'yaw'),
     [[('i32', 1200), ('i32', -1200), ('i32', 0)]], None),
    ('no_fields', None, 'empty.csv', (), [[]], None),
    ('gnss_degrees', None, 'gnss.csv', ('lat', 'lon', 'speed_kn', 'course'),
     [[('e7', 255143790), ('e7', -803917950), ('f32', 12.5), ('f32', 271.25)],
      [('e7', -1), ('e7', 0), ('f32', 0.5), ('f32', 0.25)]], _E7_NOTE),
    ('integral_floats', None, 'attitude.csv', ('yaw', 'pitch', 'roll'),
     [[('f32', 1.0), ('f32', 2.0), ('f32', 3.0)]], _FLOAT_NOTE),
    ('floats_plan_b_writes_with_an_exponent', None, 'tiny.csv', ('small', 'large', 'smallest'),
     [[('f32', 1e-05), ('f32', 1.5e+16), ('f32', 2.5e-07)]], _FLOAT_NOTE),
    ('floats_that_are_no_numbers', None, 'odd.csv', ('nan', 'up', 'down', 'zero'),
     [[('f32', math.nan), ('f32', math.inf), ('f32', -math.inf), ('f32', -0.0)]], _FLOAT_NOTE),
    ('f64_digits_no_f32_holds', None, 'sum.csv', ('sum',), [[('f32', 0.1 + 0.2)]], _FLOAT_NOTE),
    ('text_with_separators', None, 'board_events.csv', ('event',),
     [[('text', 'stage a;b')], [('text', 'two\nlines')], [('text', 'tab\there')]], _TEXT_NOTE),
)


def _telemetry_case(name: str, label: str | None, stream: str, fields: tuple, rows: list, note: str | None) -> dict:
    """
    One stream through plan B's Telemetry: a header, then a row per push, each at its own uptime.

    Args:
        name - the case name.
        label - the recorder.session label; None for the boot id.
        stream - the stream file.
        fields - its field names.
        rows - its rows of (kind, value) cells.
        note - the declared divergence, or None.

    Returns:
        The case: input {session, stream, fields, rows: [{uptime_us, cells}]}, output the wire lines.
    """
    _setup(_config(session=label) if label is not None else None)
    wire = recorder.Recorder._uart
    telemetry = recorder.Telemetry(stream, fields)
    session = recorder.Recorder.session()
    routing = '%s_%s' % (session, stream)
    pushed = []
    for cells in rows:
        shims.advance(_ROW_GAP_MS)
        uptime_us = time.ticks_us()
        telemetry.push(tuple(_plan_b_value(kind, value) for kind, value in cells))
        pushed.append({'uptime_us': uptime_us,
                       'cells': [{'kind': kind, 'value': _value(kind, value)} for kind, value in cells]})
    oracle = _drained(wire)
    if len(oracle) != len(rows) + 1:
        raise ValueError('%s: %d lines for %d rows' % (name, len(oracle), len(rows)))
    rust = [_host_wrap(routing, ('uptime;' + ';'.join(fields)).encode())]
    for row, cells in zip(pushed, rows):
        text = '%d;%s' % (row['uptime_us'], ';'.join(_rust_text(kind, value) for kind, value in cells))
        rust.append(_host_wrap(routing, text.encode()))
    for line in oracle:
        if not line.startswith(b'@%s@' % routing.encode()):
            raise ValueError('%s: %r is not routed to %s' % (name, line, routing))
    case = {'name': name, 'input': {'session': session, 'stream': stream, 'fields': list(fields), 'rows': pushed}}
    return _settle(case, [_bytes(line) for line in oracle], [_bytes(line) for line in rust], note)


"""Log lines: (name, cell_size, descriptor, message, note); an int message is a length that fits the cell exactly."""
_LOGS: tuple = (
    ('a_stage_line', 256, 'sequencer', 'stage -> boosting', None),
    ('the_status_line', 256, 'Recorder',
     "{'session': '000123', 'tlm': {'count': 1, 'max': 11, 'dropped': 0}, 'log': {'count': 0, 'max': 1, "
     "'dropped': 0}}", None),
    ('semicolons_and_unicode_stay', 256, 'Mission', 'session.csv row lost; café', None),
    ('cut_to_the_default_cell', 256, 'X', 'y' * 300, None),
    ('cut_to_a_small_cell', 64, 'X', 'y' * 300, None),
    ('exactly_fills_a_small_cell', 64, 'X', 0, None),
    ('one_byte_past_a_small_cell', 64, 'X', 1, None),
    ('cut_inside_a_character', 64, 'X', 'é' * 40, None),
    ('an_inner_newline', 256, 'gnss', 'one\ntwo', _LOG_NOTE),
    ('a_trailing_newline', 256, 'gnss', 'done\n', _LOG_NOTE),
    ('tabs', 256, 'i2c\t0', 'x\ty', _LOG_NOTE),
)


def _log_case(name: str, cell_size: int, descriptor: str, message: str | int, note: str | None) -> dict:
    """
    One Recorder.log() line, under a ring of `cell_size` cells.

    Args:
        name - the case name.
        cell_size - the recorder.cell_size: the line, its '\\n' included, is cut to cell_size - 2 bytes.
        descriptor - the source tag.
        message - the text; an int asks for a message that long past the length that exactly fills the cell.
        note - the declared divergence, or None.

    Returns:
        The case: input {uptime_us, descriptor, message, limit}, output the wire line.
    """
    wire = _setup(_config(cell_size=cell_size))
    limit = recorder.Recorder._log.max_payload
    uptime_us = time.ticks_us()
    if isinstance(message, int):
        message = 'y' * (limit - len(('%u %s :: \n' % (uptime_us, descriptor)).encode()) + message)
    if not recorder.Recorder.log(descriptor, message):
        raise ValueError('%s: the log ring refused it' % name)
    oracle = _drained(wire)
    text = ('%u %s :: %s' % (uptime_us, _cleaned(descriptor), _cleaned(message))).encode()
    rust = _host_wrap(None, text[:limit - 1])
    case = {'name': name, 'input': {'uptime_us': uptime_us, 'descriptor': descriptor, 'message': message,
                                    'limit': limit}}
    return _settle(case, _bytes(oracle[0]), _bytes(rust), note)


"""
session.csv rows: (name, boot_id, board config edits, how, seconds, utc_offset, source, cc_position, note). `how` is
'index' for an index_session() call (a time set: Mission has already vetted utc_offset and cc_position), 'boot' for
the row run() writes as it starts, 'anchor' for the minute's anchor. `seconds` is the RTC on its 2000 epoch, None for
the virtual clock's own reading.
"""
_SESSION_ROWS: tuple = (
    ('a_time_set', _BOOT_ID, {}, 'index', _CLOCK_SET, -240, 'cc-auto', None, None),
    ('a_set_with_the_cc_position', _BOOT_ID, {}, 'index', _CLOCK_SET + 233, None, 'dashboard', (25.5, -80.25),
     _CC_NOTE),
    ('a_southern_eastern_cc_position', _BOOT_ID, {}, 'index', _CLOCK_SET, 600, 'cc-auto', (-33.8688, 151.2093),
     _CC_NOTE),
    ('utc_offset_zero', _BOOT_ID, {}, 'index', _CLOCK_SET, 0, 'cc-auto', None, None),
    ('utc_offset_west_edge', _BOOT_ID, {}, 'index', _CLOCK_SET, -720, 'cc-auto', None, None),
    ('utc_offset_east_edge', _BOOT_ID, {}, 'index', _CLOCK_SET, 840, 'cc-auto', None, None),
    ('a_power_on_clock', _BOOT_ID, {}, 'index', 0, -240, 'dashboard', None, None),
    ('the_last_unset_second', _BOOT_ID, {}, 'index', _FIRST_SET - 1, -240, 'dashboard', None, None),
    ('the_first_set_second', _BOOT_ID, {}, 'index', _FIRST_SET, -240, 'dashboard', None, None),
    ('a_source_with_a_semicolon', _BOOT_ID, {}, 'index', _CLOCK_SET, None, 'a;b', None, None),
    ('a_source_with_a_tab', _BOOT_ID, {}, 'index', _CLOCK_SET, None, 'cc\tauto', None, None),
    ('a_source_that_is_no_text', _BOOT_ID, {}, 'index', _CLOCK_SET, None, 7, None, None),
    ('a_source_of_33_bytes', _BOOT_ID, {}, 'index', _CLOCK_SET, None, 'x' * 33, None, None),
    ('a_source_of_32_bytes', _BOOT_ID, {}, 'index', _CLOCK_SET, None, 'x' * 32, None, None),
    ('an_unnamed_board_without_a_firmware', _BOOT_ID, {'id': 'tms;7', 'firmware_version': None}, 'index', _CLOCK_SET,
     None, 'cc-auto', None, None),
    ('a_non_ascii_firmware', _BOOT_ID, {'firmware_version': 'café'}, 'index', _CLOCK_SET, None, 'cc-auto', None,
     None),
    ('a_real_firmware_stamp', _BOOT_ID, {'firmware_version': '2026.09.23.55c494a0c659'}, 'index', _CLOCK_SET, -240,
     'cc-auto', None, None),
    ('a_boot_nothing_counted', None, {}, 'index', _CLOCK_SET, None, 'dashboard', None, None),
    ('a_wide_boot_id', 1234567, {}, 'index', _CLOCK_SET, None, 'dashboard', None, None),
    ('the_boot_row', _BOOT_ID, {}, 'boot', None, None, 'boot', None, None),
    ('a_dated_anchor', _BOOT_ID, {}, 'anchor', _CLOCK_SET, None, 'anchor', None, None),
    ('an_undated_anchor', _BOOT_ID, {}, 'anchor', 0, None, 'anchor', None, None),
)


def _session_row_case(name: str, boot_id: int | None, board: dict, how: str, seconds: int | None,
                      utc_offset: int | None, source: object, cc_position: tuple | None, note: str | None) -> dict:
    """
    One session.csv row through plan B's Recorder, header first.

    Args:
        name - the case name.
        boot_id - the boot's NVS id, or None.
        board - edits to the config's board section; a None value removes the key.
        how - 'index', 'boot' or 'anchor' (see _SESSION_ROWS).
        seconds - the RTC on its 2000 epoch, or None for the virtual clock's.
        utc_offset, source, cc_position - the row's inputs, as Mission hands them to index_session().
        note - the declared divergence, or None.

    Returns:
        The case: input the SessionRow fields (epoch in Unix seconds, cc_position in 1e-7 degree units), output the
        header line and the row line.
    """
    config = config_default.default()
    for key, value in board.items():
        if value is None:
            config['board'].pop(key, None)
        else:
            config['board'][key] = value
    random.seed(_SEED)  # a boot nothing counted draws the legacy prefix's random suffix
    wire = _setup(config, boot_id)
    shims.advance(_ROW_GAP_MS)
    uptime_us = time.ticks_us()
    clock = time.time() if seconds is None else seconds
    if how == 'index':
        recorder.Recorder.index_session(clock, utc_offset, source, cc_position)
    elif how == 'boot':
        recorder.Recorder._index_row('boot', clock)
    elif not recorder.Recorder._anchor(time.ticks_add(recorder.Recorder._setup_ms, 60000), clock):
        raise ValueError('%s: the anchor was not due' % name)
    oracle = _drained(wire)
    if len(oracle) != 2:
        raise ValueError('%s: %r' % (name, oracle))
    units = None if cc_position is None else [round(degrees * _UNITS_PER_DEGREE) for degrees in cc_position]
    cells = _payload(oracle[1]).decode().split(';')
    if units is not None:
        cells[9:11] = [_e7_text(unit) for unit in units]
    rust = [oracle[0], _host_wrap('session.csv', ';'.join(cells).encode())]
    case = {'name': name, 'input': {
        'uptime_us': uptime_us, 'boot_id': boot_id, 'session': recorder.Recorder.session(),
        'epoch': clock + _EPOCH_2000, 'utc_offset': utc_offset, 'board': config['board']['id'],
        'firmware': config['board'].get('firmware_version', 'dev'), 'config_id': config_module.config_id(config),
        'source': source, 'cc_position': units}}
    return _settle(case, [_bytes(line) for line in oracle], [_bytes(line) for line in rust], note)


"""
Members after 'log' that plan B's report never had: (key, value). Placeholders for the flash writer's 'ring' and M4's
'mover' (doc/specs/avionics-port.md), whose figures those stages define; a bool shows Python's own spelling.
"""
_STATUS_MEMBERS: tuple = (
    ('ring', {'erased_ahead': 380, 'unconfirmed': 4, 'spill_frames': 0, 'formatting': False}),
    ('mover', {'lag_ms': 120, 'batches': 3}),
)


def _status_case(name: str, config: dict | None, boot_id: int | None, telemetry: int, logs: int, drain: bool,
                 members: tuple = ()) -> dict:
    """
    One status message: Recorder.report() after `telemetry` rows and `logs` log lines, drained or not.

    Args:
        members - (key, value) members to add after 'log'; () for plan B's own message.

    Returns:
        The case: input {session, tlm, log, members: [[key, repr(value)]]}, output str() of the report with the
        members added.
    """
    random.seed(_SEED)
    wire = _setup(config, boot_id)
    session = recorder.Recorder.session()
    for index in range(telemetry):
        try:
            recorder.Recorder.tlm('t.csv', '%d' % index)
        except ValueError:  # a full ring raises and counts the drop: the report shows it
            pass
    for index in range(logs):
        recorder.Recorder.log('X', 'line %d' % index)
    if drain:
        _drained(wire)
    report = recorder.Recorder.report()
    if report['session'] != session:
        raise ValueError('%s: the report names %r' % (name, report['session']))
    if list(report) != ['session', 'tlm', 'log']:
        raise ValueError('%s: the report now holds %r' % (name, list(report)))
    message = dict(report, **dict(members))
    return {'name': name, 'input': dict(report, members=[[key, repr(value)] for key, value in members]),
            'output': str(message)}


def _statuses() -> list:
    """The status cases: fresh, queued, drained, dropped, labelled and legacy sessions, and members after 'log'."""
    return [
        _status_case('fresh', None, _BOOT_ID, 0, 0, False),
        _status_case('queued', None, _BOOT_ID, 3, 2, False),
        _status_case('drained', None, _BOOT_ID, 3, 2, True),
        _status_case('dropped', _config(tlm_capacity=2, log_capacity=2), _BOOT_ID, 3, 4, False),
        _status_case('labelled', _config(session='hitl-f15'), _BOOT_ID, 1, 1, True),
        _status_case('a_boot_nothing_counted', None, None, 0, 1, True),
        _status_case('members_after_the_log', None, _BOOT_ID, 3, 2, True, _STATUS_MEMBERS),
    ]


"""_cell() inputs: (name, value); a value that is not text stands for a config or JSON value of another type."""
_CELLS: tuple = (
    ('a_source', 'cc-auto'),
    ('spaces_and_a_tilde', ' cc-auto ~'),
    ('empty', ''),
    ('32_bytes', 'x' * 32),
    ('33_bytes', 'x' * 33),
    ('a_firmware_stamp', '2026.09.23.55c494a0c659'),
    ('a_semicolon', 'a;b'),
    ('a_tab', 'a\tb'),
    ('a_newline', 'a\nb'),
    ('a_delete', 'a\x7fb'),
    ('non_ascii', 'café'),
    ('the_last_printable', '~'),
    ('a_number', 7),
    ('nothing', None),
)

"""_is_label() inputs: (name, text)."""
_LABELS: tuple = (
    ('letters_digits_dash', 'A-1'),
    ('one_letter', 'x'),
    ('a_run', 'catapult-run3'),
    ('32_bytes', 'run-' + 'x' * 28),
    ('33_bytes', 'run-' + 'x' * 29),
    ('empty', ''),
    ('a_dash', '-'),
    ('digits_only', '20261003'),
    ('digits_and_dash', '123-456'),
    ('an_underscore', 'catapult_run3'),
    ('a_slash', 'run/3'),
    ('non_ascii', 'réunion'),
    ('a_semicolon', 'run;3'),
    ('a_space', 'a b'),
)

"""recorder.session values: (name, value, boot_id, note); the key is absent for a value of `...`."""
_PREFIXES_CASES: tuple = (
    ('no_label', ..., _BOOT_ID, None),
    ('a_label', 'catapult-run3', _BOOT_ID, None),
    ('spaces_made_dashes', 'catapult run3', _BOOT_ID, None),
    ('stripped', '  padded\t', _BOOT_ID, None),
    ('python_strips_separators', '\x1crun\x1f', _BOOT_ID, None),
    ('an_ideographic_space', '　run', _BOOT_ID, None),
    ('a_no_break_space_stays', 'a b', _BOOT_ID, None),
    ('blank', '   ', _BOOT_ID, None),
    ('empty', '', _BOOT_ID, None),
    ('an_underscore', 'catapult_run3', _BOOT_ID, None),
    ('digits_only', '20261003', _BOOT_ID, None),
    ('32_bytes', 'run-' + 'x' * 28, _BOOT_ID, None),
    ('33_bytes', 'run-' + 'x' * 29, _BOOT_ID, None),
    ('a_wide_boot_id', ..., 1234567, None),
    ('the_legacy_date_form', ..., None, None),
    ('a_label_without_a_boot_id', 'bench', None, None),
    ('null', None, _BOOT_ID, None),
    ('a_number', 5, _BOOT_ID, None),
    ('true', True, _BOOT_ID, _LABEL_NOTE),
)


def _prefix_case(name: str, value: object, boot_id: int | None, note: str | None) -> dict:
    """
    The session prefix plan B settles on for a recorder.session value and boot id.

    Returns:
        The case: input {configured, boot_id, epoch, random}, output the prefix. `epoch` (Unix seconds) and
        `random` (mapped by the Rust port onto 100000 + random % 900000) reproduce the legacy form.
    """
    config = _config() if value is ... else _config(session=value)
    random.seed(_SEED)
    drawn = random.randint(100000, 999999)
    random.seed(_SEED)
    _setup(config, boot_id)
    epoch = time.time() + _EPOCH_2000
    session = recorder.Recorder.session()
    if boot_id is None and value is ... and not session.endswith('_%d' % drawn):
        raise ValueError('%s: %r does not end with the drawn %d' % (name, session, drawn))
    rust = session
    if note is not None:  # a value that is not text names no session in Rust
        rust = '%06u' % boot_id
    case = {'name': name, 'input': {'configured': None if value is ... else value, 'boot_id': boot_id,
                                    'epoch': epoch, 'random': drawn - 100000}}
    return _settle(case, session, rust, note)


def build() -> dict:
    """
    The recorder_wire fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, crc32, wraps, damage, telemetry, logs, session_rows, statuses, cells, labels, prefixes}.
    """
    asyncio.ThreadSafeFlag = _Flag
    return {
        'meta': {'source': 'src/glider/recorder.py, src/glider/test/test_recorder.py, tools/recorder_wire.py'},
        'compare': {'kind': 'exact'},
        'crc32': _crc32(),
        'wraps': _wraps(),
        'damage': _damage(),
        'telemetry': [_telemetry_case(*case) for case in _STREAMS],
        'logs': [_log_case(*case) for case in _LOGS],
        'session_rows': [_session_row_case(*case) for case in _SESSION_ROWS],
        'statuses': _statuses(),
        'cells': [{'name': name, 'input': value, 'output': recorder._cell(value)} for name, value in _CELLS],
        'labels': [{'name': name, 'input': text, 'output': recorder._is_label(text)} for name, text in _LABELS],
        'prefixes': [_prefix_case(*case) for case in _PREFIXES_CASES],
    }
