"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture gnss.json: plan B's GNSS (src/glider/gnss.py, with the command logic of drivers/atgm336h.py and
drivers/neo6mv2.py) as the Rust port (crates/gnss) must answer it. Six sections:
  * checksum -- gnss.checksum_ok(line): exact.
  * degrees -- gnss.degrees(ddmm, hemisphere): {degrees} (null for an empty field) or {malformed} where plan B
    raises ValueError (the line is dropped). Abs 1e-6 degree, the plan's figure (about 11 cm): the Rust port
    works in integer 1e-7 degree units, at most half a unit (5e-8 degree) from plan B's float, so the tolerance
    has a twentyfold margin and still catches a minute digit lost (1.7e-6 degree at the 4th decimal).
  * nmea -- gnss.nmea(body), the framed command sentence: bytes, exact.
  * ubx -- neo6mv2._ubx(class, id, payload), the UBX frame: bytes, exact.
  * commands -- each driver's init for an `hz`: the frames _configure() writes, in order, and the sky switch pair it
    returns (off: the flight init's own selection; on: the diagnostics'): bytes, exact.
  * trace -- scenarios on one receiver each (Atgm336h or Neo6mv2 set up on stand-ins), step by step: lines through
    run()'s loop body, sentences through _parse() (a few at a microsecond offset, so an outage's milliseconds
    are floored from a time between two of them), the 1 s tick through _sweeping()'s loop body, and _settle(),
    _sweep() and _sky_window() at a time. Each step's answer comes in four parts, each with its own comparison:
      degrees -- the position pushed and the one in the fix row, abs 1e-6 degree (as above);
      speed   -- the ground speed pushed (m/s) and the fix row's knots, rel 3e-7: three f32 roundings at most
                 (the field's parse, the knots constant, their product; each within 2**-24, 6e-8, relative), so the
                 port's own error stays under 1.8e-7, while a constant other than plan B's truncated 0.514444
                 (the exact 1852/3600 is 8.6e-7 away) fails. An absolute tolerance could not tell them apart;
      values  -- metres, degrees of course, HDOP and the C/N0 mean, abs 1e-3: NMEA carries 0.1 m and 0.01 degree
                 at best, and an f32 below 4096 holds a value to 2.4e-4 (the cases stay below 3000 m, so an
                 elevation, a difference of two, stays within 5e-4);
      exact   -- what was pushed and in which order, the GGA row's integers, the sky row, the events, the bytes
                 written and the receiver's state (fix, quality, satellites, the sky, the antenna, lines read).
    The scenarios sit on every edge plan B draws: each sentence's field-count gate from both sides, the 0.5 s
    burst quiet and the 3 s outage gap exactly, and the GSA system key (the NMEA 4.1 id, else the talker).
The bursts are test_gnss.py's, captured on the 2026-10-04 bench (verbatim but one synthetic RMC position); seeded
sweeps add RMC and GGA sentences. Nothing here reads doc/benches/gnss-20261004/raw.

The receiver runs plan B's own code. The hardware is stood in: machine.UART (constructed, never read),
asyncio.StreamReader (hands run() the step's line, then ends the loop), asyncio.StreamWriter (records the frames),
and asyncio.sleep_ms (returns at once, and ends _sweeping() after one pass). The databoard channels and the three
telemetry streams are replaced after setup with recorders, as test_gnss replaces the sky stream, and the events
with a list. The clock is the shims' virtual one; every step says the uptime it ran at (now_us), which the Rust
receiver takes as its argument.

DELIBERATE DIVERGENCES (marked; the Rust test expects the Rust answer):
  * hdop_cd, the GGA row's HDOP in hundredths. Plan B writes int(hdop * 100) of the float, which truncates a
    binary fraction: on CPython '0.29' gives 28, '1.15' 114 and '17.9' 1789 (about 6 % of two-decimal values lose
    a unit), and the board's f32 loses a unit too, in other cases. The Rust port reads the decimal text as
    integer hundredths, 29, 115, 1790. A step where that differs carries `divergence` overriding exact.gga_row.
  * degrees(): plan B slices the field around its dot and hands the parts to int() and float(), so a field with
    no dot ('4807' reads 17.45), a sign ('-4807.038'), an exponent ('4807.038e1') or more degrees than an i32 of
    1e-7 units holds (999 degrees) still yields a number. The Rust port reads only digits, at least one degree
    digit and two minute digits before the dot, and refuses the rest (the line is dropped, as plan B drops one
    whose field raises). A degrees case where that differs carries `divergence`: the Rust answer in full, plus a
    note.
Text plan B's int()/float() accept but the Rust port does not (an underscore between digits, or Unicode
whitespace) and a non-ASCII sentence (plan B checksums it by character index over the UTF-8 bytes) are left to the
Rust unit tests.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import random
import struct
import time

import config_default
import controller
import gnss
import shims
from drivers import atgm336h, neo6mv2

_SEED: int = 20261006  # every sweep's random.Random seed
_CHECKSUM_SWEEP: int = 120
_DEGREES_SWEEP: int = 160
_BODY_SWEEP: int = 60
_UBX_SWEEP: int = 40
_RMC_SWEEP: int = 60
_GGA_SWEEP: int = 80
_DEGREES_TOLERANCE: float = 1e-06  # the plan's lat/lon tolerance, degrees (see the module notes)
_SPEED_TOLERANCE: float = 3e-07  # relative, the m/s pushed and the knots in the fix row (see the module notes)
_VALUES_TOLERANCE: float = 1e-03  # metres, degrees of course, HDOP, C/N0 (see the module notes)
_HDOP_NOTE: str = ('hdop_cd: plan B truncates the float int(hdop * 100), which loses a unit on a binary fraction; the '
                   'Rust port reads the decimal text as integer hundredths')
_DEGREES_NOTE: str = ('the Rust port reads ddmm.mmmm as digits only (at least one degree and two minute digits before '
                      'the dot, a 1e-7 degree unit that fits an i32); plan B slices it into int() and float()')

"""test_gnss.py's captured bursts (see there for where each comes from)."""
_ATGM_BURST: tuple = (
    '$GNGSA,A,3,01,02,04,07,08,09,14,17,22,30,,,1.6,0.9,1.3,1*3C',
    '$GNGSA,A,3,11,19,21,22,29,36,39,,,,,,1.6,0.9,1.3,4*37',
    '$GPGSV,4,1,13,01,37,138,26,02,48,105,20,04,16,182,19,07,73,337,27,0*60',
    '$GPGSV,4,2,13,08,48,033,20,09,32,214,40,14,23,294,38,17,21,228,40,0*65',
    '$GPGSV,4,3,13,22,09,281,32,27,11,043,,30,36,320,40,194,,,25,0*69',
    '$GPGSV,4,4,13,197,,,26,0*5C',
    '$BDGSV,2,1,07,11,66,159,26,19,07,202,30,21,05,299,30,22,15,252,44,0*73',
    '$BDGSV,2,2,07,29,48,192,40,36,59,003,26,39,23,312,37,0*4B',
    '$GNRMC,001847.900,A,4807.03800,N,01131.00000,E,0.00,0.00,051026,,,A,V*0B',
    '$GPTXT,01,01,01,ANTENNA OK*35',
)
_ATGM_COLD: tuple = (
    '$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,1*01',
    '$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,4*04',
    '$GPGSV,1,1,01,01,,,39,0*6F',
    '$BDGSV,1,1,00,0*74',
)
_NEO_BURST: tuple = (
    '$GPGSA,A,3,07,02,08,01,46,30,09,14,17,04,27,22,1.99,0.98,1.74*06',
    '$GPGSV,3,1,12,01,43,130,27,02,50,092,20,04,09,178,32,07,81,335,25*79',
    '$GPGSV,3,2,12,08,41,035,22,09,26,208,39,14,27,300,42,17,27,233,38*72',
    '$GPGSV,3,3,12,22,13,286,40,27,05,045,12,30,43,319,40,46,29,248,43*79',
)


def _line(body: str) -> str:
    """A valid NMEA sentence around `body` (test_gnss's _line)."""
    return gnss.nmea(body).decode().strip()


class _Stop(Exception):
    """Ends one pass of a plan B loop (run() or _sweeping()) once the step it was driven for is done."""


class _Uart:
    """machine.UART on the host: setup() opens it and hands it to the stream stand-ins; nothing reads it."""

    def __init__(self, *arguments: object, **keywords: object) -> None:
        pass


class _Reader:
    """asyncio.StreamReader: run() reads the queued raw lines, then the loop ends (_Stop)."""

    def __init__(self, *arguments: object) -> None:
        self.queue: list = []

    async def readline(self) -> bytes:
        """The next queued line."""
        if not self.queue:
            raise _Stop()
        return self.queue.pop(0)


class _Writer:
    """asyncio.StreamWriter: records every frame written, in order, into the step log."""

    def __init__(self, *arguments: object) -> None:
        self.frames: list = []

    def write(self, data: bytes) -> None:
        """Record the frame."""
        self.frames.append(bytes(data))
        _log.append(('write', bytes(data)))

    async def drain(self) -> None:
        """Nothing to flush."""


class _Channel:
    """A databoard channel: push() records (name, value) into the step log; value() is the last pushed."""

    def __init__(self, name: str) -> None:
        self.name: str = name
        self.last: object = None

    def push(self, value: object) -> None:
        """Record the push."""
        self.last = value
        _log.append(('push', self.name, value))

    def value(self) -> object:
        """The last pushed value (inspect() reads it)."""
        return self.last


class _Rows:
    """A Telemetry stream: push() records the row into the step log."""

    def __init__(self, name: str) -> None:
        self.name: str = name

    def push(self, values: tuple) -> None:
        """Record the row."""
        _log.append(('row', self.name, tuple(values)))


_log: list = []  # what the current step did, in order
_passes: list = [None]  # how many asyncio.sleep_ms may still return before the loop ends: None is unlimited


async def _sleep_ms(milliseconds: int) -> None:
    """
    asyncio.sleep_ms: returns at once (the trace moves the clock itself), and ends the loop past its pass budget.

    Args:
        milliseconds - ignored: the trace sets every step's time.

    Returns:
        None.

    Raises:
        _Stop - the budget is spent.
    """
    if _passes[0] is not None:
        if _passes[0] == 0:
            raise _Stop()
        _passes[0] -= 1


async def _idle() -> None:
    """What run() starts as its sweeper here: nothing (a tick step drives _sweeping() itself)."""


def _install() -> None:
    """Put the hardware stand-ins in place (see the module notes)."""
    gnss.UART = _Uart
    asyncio.StreamReader = _Reader
    asyncio.StreamWriter = _Writer
    asyncio.sleep_ms = _sleep_ms


class _Controller:
    """The controller as a receiver sees it: the config (its uart:2 bus) and the flight stage a tick reads."""

    config: dict = config_default.default()
    stage: int = controller.Stage.SETTING


_DRIVERS: dict = {'atgm336h': atgm336h.Atgm336h, 'neo6mv2': neo6mv2.Neo6mv2}


def _receiver(driver: str, hz: int) -> object:
    """
    A receiver set up by plan B on the stand-ins, its outputs then recorded.

    Args:
        driver - 'atgm336h' or 'neo6mv2'.
        hz - its fix rate.

    Returns:
        The set-up receiver: channels, streams, events and writer recording into the step log.
    """
    spec = dict(next(sensor for sensor in config_default.default()['sensors'] if sensor['name'] == 'gnss'))
    spec.update(driver=driver, hz=hz)
    unit = _DRIVERS[driver]('gnss', spec, _Controller())
    _passes[0] = None
    if not asyncio.run(unit.setup()):
        raise ValueError('plan B refused to set up %s' % driver)
    for name in ('position', 'altitude', 'elevation', 'speed', 'course'):
        setattr(unit, '_' + name, _Channel(name))
    unit._telemetry = _Rows('fix')
    unit._gga_telemetry = _Rows('gga')
    unit._sky_telemetry = _Rows('sky')
    unit._writer = _Writer()
    unit.event = lambda message: _log.append(('event', message))
    unit._sweeping = _idle  # run()'s create_task: a tick step drives the class's _sweeping() instead
    return unit


def _decimal_centi(text: str) -> int:
    """
    A decimal text as integer hundredths, truncated toward zero: how the Rust port reads hdop_cd.

    Args:
        text - a plain decimal ('0.9', '1.15', '-0.5'); '' is 0.

    Returns:
        The hundredths.
    """
    if not text:
        return 0
    negative = text.startswith('-')
    whole, _dot, fraction = text.lstrip('+-').partition('.')
    centi = int(whole or '0') * 100 + int((fraction + '00')[:2])
    return -centi if negative else centi


def _step_out(unit: object) -> dict:
    """
    The step's answer from the step log and the receiver's state, in the three comparison parts.

    Args:
        unit - the receiver after the step.

    Returns:
        {degrees, speed, values, exact}.
    """
    pushes = {entry[1]: entry[2] for entry in _log if entry[0] == 'push'}
    rows = {entry[1]: entry[2] for entry in _log if entry[0] == 'row'}
    writes = [entry[1] for entry in _log if entry[0] == 'write']
    if len(writes) > 1 or len([entry for entry in _log if entry[0] == 'row']) > len(rows):
        raise ValueError('one step wrote twice or pushed a stream twice: split it')
    status = unit.inspect()
    fix_row = rows.get('fix')
    gga_row = rows.get('gga')
    position = pushes.get('position')
    return {
        'degrees': {'position': None if position is None else list(position),
                    'fix_row': None if fix_row is None else list(fix_row[:2])},
        'speed': {'speed_ms': pushes.get('speed'), 'fix_row_knots': None if fix_row is None else fix_row[2]},
        'values': {'course_deg': pushes.get('course'),
                   'altitude_m': pushes.get('altitude'), 'elevation_m': pushes.get('elevation'),
                   'fix_row_course': None if fix_row is None else fix_row[3],
                   'gga_row': None if gga_row is None else list(gga_row[:2]),
                   'hdop': status['hdop'], 'cn0_mean': status['cn0_mean']},
        'exact': {'pushed': [entry[1] for entry in _log if entry[0] == 'push'],
                  'gga_row': None if gga_row is None else list(gga_row[2:]),
                  'sky_row': None if rows.get('sky') is None else list(rows['sky']),
                  'events': [entry[1] for entry in _log if entry[0] == 'event'],
                  'write': writes[0].hex() if writes else None,
                  'fix': status['fix'], 'fix_quality': status['fix_quality'], 'satellites': status['satellites'],
                  'in_view': status['in_view'], 'used': status['used'], 'mode': status['mode'],
                  'cn0': list(status['cn0']), 'antenna': status['antenna'], 'lines': unit._lines,
                  'diagnosing': unit._diagnosing},
    }


def _run_step(unit: object, operation: str, argument: object) -> dict:
    """
    Run one step on plan B's receiver at the current virtual time.

    Args:
        unit - the receiver.
        operation - 'line' (text through run()), 'raw' (bytes through run()), 'parse' (text through _parse()),
            'parse_at' (text through _parse() at now + an offset in us, written as a 'parse' at that time),
            'settle' / 'sweep' (at now + the argument in us), 'window' (_sky_window(stage)), 'tick' (one pass of
            _sweeping() with the controller in the argument's stage).
        argument - the text, the bytes, (the text, the offset in us), the offset in us, or the stage id.

    Returns:
        The step's input as the fixture writes it.
    """
    now_us = time.ticks_us()
    if operation == 'parse_at':
        """
        The shims' clock moves in whole milliseconds; a sentence arriving between two of them is read at its own
        microsecond, so the interval an outage event floors (`// 1000`) has a remainder. The clock stays where it
        is: the next step advances past this time.
        """
        text, offset_us = argument
        at_us = time.ticks_add(now_us, offset_us)
        clock = time.ticks_us
        time.ticks_us = lambda: at_us
        try:
            unit._parse(text)
        except (UnicodeError, ValueError, IndexError):
            pass  # what run() does with a line that raises
        finally:
            time.ticks_us = clock
        return {'op': 'parse', 'text': text, 'now_us': at_us}
    if operation in ('line', 'raw'):
        raw = argument.encode() + b'\r\n' if operation == 'line' else argument
        unit._reader.queue = [raw]
        try:
            asyncio.run(unit.run())
        except _Stop:
            pass
        if operation == 'line':
            return {'op': 'line', 'text': argument, 'now_us': now_us}
        return {'op': 'raw', 'hex': argument.hex(), 'now_us': now_us}
    if operation == 'parse':
        try:
            unit._parse(argument)
        except (UnicodeError, ValueError, IndexError):
            pass  # what run() does with a line that raises
        return {'op': 'parse', 'text': argument, 'now_us': now_us}
    if operation in ('settle', 'sweep'):
        at_us = time.ticks_add(now_us, argument)
        getattr(unit, '_' + operation)(at_us)
        return {'op': operation, 'at_us': at_us}
    if operation == 'window':
        if unit._sky_window(argument):
            asyncio.run(unit._writer.drain())
        return {'op': 'window', 'stage': argument}
    if operation == 'tick':
        unit.controller.stage = argument
        _passes[0] = 1
        try:
            asyncio.run(type(unit)._sweeping(unit))
        except _Stop:
            pass
        _passes[0] = None
        return {'op': 'tick', 'stage': argument, 'now_us': now_us}
    raise ValueError('unknown step operation %s' % operation)


def _scenario(name: str, driver: str, hz: int, steps: list) -> dict:
    """
    One scenario on a fresh receiver.

    Args:
        name - the scenario name.
        driver, hz - the receiver.
        steps - (advance_ms, operation, argument) per step: the clock moves by advance_ms first.

    Returns:
        {name, init: {driver, hz}, steps: [{in, out[, divergence]}]}.
    """
    unit = _receiver(driver, hz)
    unit.controller.stage = controller.Stage.SETTING
    out_steps = []
    for advance_ms, operation, argument in steps:
        shims.advance(advance_ms)
        _log.clear()
        step = {'in': _run_step(unit, operation, argument)}
        step['out'] = _step_out(unit)
        gga_row = step['out']['exact']['gga_row']
        if gga_row is not None:
            recorded = step['in']
            text = recorded['text'] if 'text' in recorded else bytes.fromhex(recorded['hex']).decode()
            hdop_text = text.split('*')[0].split(',')[8]
            rust = gga_row[:2] + [_decimal_centi(hdop_text)]
            if rust != gga_row:
                step['divergence'] = {'exact': {'gga_row': rust}, 'note': _HDOP_NOTE}
        out_steps.append(step)
    return {'name': name, 'init': {'driver': driver, 'hz': hz}, 'steps': out_steps}


def _parses(lines: tuple, advance_ms: int = 0) -> list:
    """Parse steps for `lines`, the first after advance_ms and the rest at the same time."""
    return [(advance_ms if index == 0 else 0, 'parse', line) for index, line in enumerate(lines)]


_RMC: str = _line('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W')
_GGA: str = _line('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,')
_STAGES = controller.Stage


def _canned() -> list:
    """
    test_gnss's canned RMC/GGA and the outage events, plus fields left empty.

    The outage gap is swept exactly: GGA (last heard 100 ms before RMC) is quiet exactly 3 s at the first sweep
    (not past the gap), 3 s and 500 us at the next (lost, 'quiet 3000 ms': the milliseconds floored, never
    rounded up); then RMC the same at 3 s and at 3 s plus 1 us. GGA comes back 999 us past a whole millisecond,
    so its 'back after' is floored too.
    """
    return [
        (0, 'parse', _RMC),
        (0, 'parse', _GGA),
        (100, 'parse', _line('GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,550.4,M,46.9,M,,')),
        (100, 'parse', _line('GPRMC,123520,V,,,,,,,230394,,')),
        (0, 'parse', '$GPRMC,123521,A,1234.000,N,01234.000,E,0,0,230394,,*00'),
        (0, 'sweep', 1000000),
        (0, 'sweep', 2900000),
        (0, 'sweep', 2900500),
        (0, 'sweep', 3000000),
        (0, 'sweep', 3000001),
        (0, 'sweep', 4000000),
        (0, 'sweep', 5000000),
        (5000, 'parse', _line('GPRMC,123522,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W')),
        (0, 'sweep', 0),
        (200, 'parse_at', (_line('GPGGA,123530,,,,,0,00,99.99,,M,,M,,'), 999)),
        (200, 'parse', _line('GNRMC,123531,A,3352.12345,S,15112.54321,W,,,051026,,,A')),
        (200, 'parse', _line('GNRMC,123532,A,3352.12345,S,15112.54321,W,,123.45,051026,,,A')),
        (200, 'parse', _line('GPGGA,123533,3352.123,S,15112.543,W,2,12,1.15,-12.5,M,20.1,M,,')),
        (200, 'parse', _line('GNRMC,123534,V,4807.038,N,01131.000,E,1.0,2.0,051026,,,N')),
        (200, 'parse', _line('GPRMC,123535,A,,N,01131.000,E,1.0,2.0,051026')),
    ]


def _atgm_sky() -> list:
    """test_gnss's ATGM sky sequence: the cold and the fixed burst, antenna texts, GSV group order, a lost system."""
    steps = [(0, 'settle', 5000000)]
    steps += _parses(_ATGM_COLD, 100)
    steps += [(0, 'settle', 100000), (0, 'settle', 1000000), (0, 'settle', 2000000)]
    steps += _parses(_ATGM_BURST, 3000)
    steps += [(0, 'settle', 1000000)]
    steps += _parses(('$GPTXT,01,01,01,ANTENNA OK*35', '$GPTXT,01,01,01,ANTENNA OPEN*25',
                      _line('GPTXT,01,01,01,ANTENNA SHORT'), '$GPTXT,01,01,02,ANTSTATUS=INIT*25',
                      '$GPTXT,01,01,01,ANTENNA OK*00'), 2000)
    steps += [(0, 'settle', 1000000)]
    steps += _parses(('$GPTXT,01,01,02,ANTSTATUS=INIT*25',), 2000)
    steps += [(0, 'settle', 1000000)]
    steps += _parses(('$GPGSV,4,2,13,08,48,033,20,09,32,214,40,14,23,294,38,17,21,228,40,0*65',
                      _line('GPGSV,3,1,11,01,43,131,50,02,50,092,50,04,09,179,50,07,81,335,50,0'),
                      _line('GPGSV,3,3,11,22,13,287,50,27,05,045,50,30,43,319,50,0'),
                      _line('GPGSV,2,1,05,01,43,131,51,02,50,092,51,04,09,179,51,07,81,335,51,0'),
                      '$GPGSV,2,2,05,08,41,035,51,0*00',
                      _line('BDGSV,2,2,07,29,48,192,52,36,59,003,52,39,23,312,52,0')), 2000)
    steps += [(0, 'settle', 1000000)]
    steps += _parses((_line('GPGSV,1,1,02,01,43,131,,02,50,092,'), _line('GNGSA,A,1,,,,,,,,,,,,,9.9,9.9,9.9,4')), 2000)
    steps += [(0, 'settle', 1000000)]
    """Groups that are odd but whole: part 0 of 0 continues the last group's C/N0 list, a GSA with no PRN."""
    steps += _parses((_line('GLGSV,1,1,03,65,10,100,33,66,20,200,31,67,30,300,'), _line('GLGSV,0,0,01,68,40,40,45'),
                      _line('GNGSA,A,2,,,,,,,,,,,,,5.0,4.0,3.0,2'), _line('GPGSV,2,3,05,01,02,03,04')), 2000)
    steps += [(0, 'settle', 1000000)]
    """
    The field-count gates from both sides. A NEO-6M with nothing in view sends a 4-field GSV, which plan B takes
    (GSV needs > 3): GP's view folds to none in view and the burst clock restarts. A 3-field GSV and a 14-field
    GSA are not the sky: no view changes and no burst opens (the settle after them pushes nothing). A 15-field
    GSA is the shortest taken (> 14), and with no system id its talker keys it.
    """
    steps += _parses(('$GPGSV,1,1,00*79',), 2000)
    steps += [(0, 'settle', 1000000)]
    steps += _parses((_line('GPGSV,1,1'), _line('GPGSA,A,3,05,06,07,,,,,,,,')), 2000)
    steps += [(0, 'settle', 1000000)]
    steps += _parses(('$GPGSA,A,1,,,,,,,,,,,,*32', _line('GPGSA,A,3,05,06,07,,,,,,,,,')), 2000)
    steps += [(0, 'settle', 1000000)]
    return steps


def _gsa_systems() -> list:
    """
    The GSA system key: the NMEA 4.1 system id (field 18) when the sentence has one, else its talker.

    The NEO-6M's NMEA 2.3 GSA (18 fields) has no id, so two from different talkers are two systems and their
    counts SUM, while a second from the same talker replaces the first. An id-carrying GSA of system 1 (GPS) is
    keyed by '1', apart from the talker-keyed 'GP' even though both are GPS: plan B's own keying, which only a
    receiver mixing the two NMEA versions would meet.
    """
    steps = _parses((_line('GPGSA,A,3,01,02,03,04,,,,,,,,,1.5,0.9,1.2'),), 100)
    steps += _parses((_line('BDGSA,A,2,11,12,13,,,,,,,,,,1.6,1.0,1.3'),), 100)
    steps += [(0, 'settle', 1000000)]
    steps += _parses((_line('GPGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2'),), 100)
    steps += _parses((_line('GNGSA,A,3,05,06,07,08,09,,,,,,,,1.5,0.9,1.2,1'),), 100)
    steps += [(0, 'settle', 1000000)]
    return steps


def _quiet_restart() -> list:
    """
    test_gnss's quiet-window case: each sky sentence restarts the burst's quiet clock, another text does not.

    Then the quiet itself, exactly: 1 us short of half a second the burst may still be arriving (no row), at
    exactly half a second it is over (plan B ends it unless the quiet is SHORTER than _BURST_QUIET_US), and the
    burst is spent after its row.
    """
    steps = []
    for line in (_NEO_BURST[0], _NEO_BURST[1], '$GPTXT,01,01,01,ANTENNA OK*35', '$GPTXT,01,01,02,ANTSTATUS=INIT*25'):
        steps += [(3000, 'parse', _NEO_BURST[0]), (1000, 'parse', line), (0, 'settle', 200000),
                  (0, 'settle', 1000000)]
    steps += [(3000, 'parse', _NEO_BURST[0]), (0, 'settle', 499999), (0, 'settle', 500000), (0, 'settle', 500001)]
    return steps


def _window() -> list:
    """The sky window per stage, reaching every stage from both states with no state poked."""
    stages = (_STAGES.SETTING, _STAGES.SETTING, _STAGES.BOOSTING, _STAGES.BOOSTING, _STAGES.GLIDING, _STAGES.GLIDING,
              _STAGES.LANDING, _STAGES.LANDING, _STAGES.DONE, _STAGES.DONE, _STAGES.GLIDING, _STAGES.SETTING,
              _STAGES.LANDING, _STAGES.DONE, _STAGES.BOOSTING, _STAGES.SETTING, _STAGES.DONE)
    return [(100, 'window', stage) for stage in stages]


def _clocked() -> list:
    """test_gnss's real-clock flow, on explicit times: bursts arriving in parts around the 1 s ticks, then a flight."""
    steps = [(0, 'tick', _STAGES.SETTING)]
    steps += _parses(_ATGM_BURST[:4], 900)
    steps += [(100, 'tick', _STAGES.SETTING)]
    steps += _parses(_ATGM_BURST[4:], 300)
    steps += [(0, 'settle', 200000), (0, 'settle', 400000), (700, 'tick', _STAGES.SETTING)]
    steps += _parses(_ATGM_COLD, 300)
    steps += [(700, 'tick', _STAGES.SETTING)]
    steps += _parses(_NEO_BURST[:2], 900)
    steps += [(100, 'tick', _STAGES.SETTING)]
    steps += _parses(_NEO_BURST[2:], 300)
    steps += [(0, 'settle', 200000), (700, 'tick', _STAGES.SETTING), (1000, 'tick', _STAGES.BOOSTING),
              (1000, 'tick', _STAGES.GLIDING), (1000, 'tick', _STAGES.DONE), (1000, 'tick', _STAGES.SETTING),
              (4000, 'tick', _STAGES.SETTING)]
    steps += _parses((_RMC, _GGA), 100)
    steps += [(900, 'tick', _STAGES.SETTING), (2500, 'tick', _STAGES.SETTING), (1000, 'tick', _STAGES.SETTING)]
    return steps


def _warm_start() -> list:
    """A reset into GLIDING (a warm start): the init left the diagnostics on, the first tick switches them off."""
    return [(0, 'tick', _STAGES.GLIDING), (1000, 'tick', _STAGES.GLIDING), (1000, 'tick', _STAGES.LANDING),
            (1000, 'tick', _STAGES.DONE), (1000, 'tick', _STAGES.DONE)]


def _run_lines() -> list:
    """
    Lines through run()'s loop body: line endings, noise, malformed fields, the counter and the per-line sweep.

    It ends on RMC's and GGA's field-count gates (both need > 9) from both sides, after an outage so the arrival
    shows: a 9-field RMC and GGA are refused (nothing pushed, no row), yet each still marks its arrival (the 'back
    after' event); the 10-field ones are the shortest taken.
    """
    return [
        (0, 'line', _RMC),
        (0, 'raw', b''),
        (100, 'raw', b'\xff\xfe' + _RMC.encode() + b'\r\n'),
        (100, 'line', '  ' + _RMC + ' \t'),
        (100, 'line', _RMC[1:]),
        (100, 'line', _line('GPRMC,123519,A,48x7.038,N,01131.000,E,022.4,084.4,230394,003.1,W')),
        (100, 'line', _line('GPRMC,123519,A,4807.038,N,01131.000,E,fast,084.4,230394,003.1,W')),
        (100, 'line', _line('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,east,230394,003.1,W')),
        (100, 'line', _line('GPRMC,1,A,4807.038,N')),
        (100, 'line', _line('GPGGA,123519,4807.038,N,01131.000,E,x,08,0.9,545.4,M,46.9,M,,')),
        (100, 'line', _line('GPGGA,123519,4807.038,N,01131.000,E,2,y,0.9,545.4,M,46.9,M,,')),
        (100, 'line', _line('GPGGA,123519,4807.038,N,01131.000,E,1,07,1.3,high,M,46.9,M,,')),
        (100, 'line', _line('GPGGA,123519,4807.038,N,01131.000,E,1,07,1.3,,M,46.9,M,,')),
        (100, 'line', _line('GPGGA,1,2,3')),
        (100, 'line', _line('GPGSV,1,,05,01,43,131,51')),
        (100, 'line', _line('GPGSV,1,1,05,01,43,131,51,02,50,092,zz,04,09,179,49')),
        (100, 'line', _line('GPGSV,1,1,05,01,43,131,51,02,50,092,48,04,09,179,49')),
        (100, 'line', _line('GNGSA,A,m,01,02,,,,,,,,,,,1.0,1.0,1.0,1')),
        (100, 'line', _line('GPGSV')),
        (100, 'line', _line('GPTXT,01,01,01')),
        (100, 'line', _line('PUBX,00,123519.00,4807.03800,N')),
        (100, 'line', '$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W'),
        (4000, 'line', _NEO_BURST[0]),
        (0, 'line', _RMC),
        (100, 'line', _line('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.29,545.4,M,46.9,M,,')),
        (100, 'line', _line('GPRMC,123519,A,4807.038,N,01131.000,E,1.5,2.5,230394,003.1,W').lower()),
        (100, 'line', _line('GPRMC,123519,A,4807.038,N,01131.000,E,1.5,2.5,230394,003.1,W') + '*FF'),
        (4000, 'line', '$GPTXT,01,01,02,ANTSTATUS=INIT*25'),
        (100, 'line', _line('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4')),
        (100, 'line', _line('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9')),
        (100, 'line', _line('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394')),
        (100, 'line', '$GPGGA,1,4807.038,N,01131.000,E,1,08,0.9,545.4*42'),
    ]


def _minutes_field(generator: random.Random, degree_digits: int, limit: int) -> tuple:
    """A random ddmm.mmmmm field below `limit` degrees, and its value in degrees (for the hemisphere draw only)."""
    degrees = generator.randrange(0, limit)
    decimals = generator.choice((4, 5))
    minutes = generator.randrange(0, 60 * 10 ** decimals)
    text = '%0*d%02d.%0*d' % (degree_digits, degrees, minutes // 10 ** decimals, decimals, minutes % 10 ** decimals)
    return text, degrees


def _rmc_sweep(generator: random.Random) -> list:
    """Seeded RMC sentences: positions on the whole globe at 4 or 5 minute decimals, speeds, courses, void fixes."""
    steps = []
    for _index in range(_RMC_SWEEP):
        latitude, _degrees = _minutes_field(generator, 2, 90)
        longitude, _degrees = _minutes_field(generator, 3, 180)
        speed = generator.choice(('', '0.00', '%.1f' % generator.uniform(0, 300), '%.3f' % generator.uniform(0, 30)))
        course = generator.choice(('', '%.2f' % generator.uniform(0, 360), '%.1f' % generator.uniform(0, 360)))
        status = 'A' if generator.random() < 0.9 else 'V'
        body = 'GNRMC,%06d.00,%s,%s,%s,%s,%s,%s,%s,051026,,,A' % (
            generator.randrange(240000), status, latitude, generator.choice('NS'), longitude, generator.choice('EW'),
            speed, course)
        steps.append((100, 'parse', _line(body)))
    return steps


def _gga_sweep(generator: random.Random) -> list:
    """Seeded GGA sentences: quality, satellites, HDOP at one or two decimals, altitudes up to 3000 m or none."""
    steps = []
    for _index in range(_GGA_SWEEP):
        hdop = generator.choice(('%.1f' % generator.uniform(0.5, 30), '%.2f' % generator.uniform(0.5, 10),
                                 '%.2f' % generator.uniform(0.5, 2), '99.99', ''))
        altitude = generator.choice(('', '%.1f' % generator.uniform(-50, 3000), '%.1f' % generator.uniform(0, 600)))
        body = 'GPGGA,%06d.00,4807.03800,N,01131.00000,E,%d,%02d,%s,%s,M,46.9,M,,' % (
            generator.randrange(240000), generator.randrange(3), generator.randrange(25), hdop, altitude)
        steps.append((100, 'parse', _line(body)))
    return steps


def _degrees_case(name: str, value: str, hemisphere: str) -> dict:
    """
    One degrees() case, with the Rust answer where it differs.

    Args:
        name - the case name.
        value - the ddmm.mmmm field.
        hemisphere - its hemisphere field.

    Returns:
        {name, input: {value, hemisphere}, output: {degrees} or {malformed}[, divergence]}.
    """
    try:
        output = {'degrees': gnss.degrees(value, hemisphere)}
    except ValueError:
        output = {'malformed': True}
    case = {'name': name, 'input': {'value': value, 'hemisphere': hemisphere}, 'output': output}
    rust = _rust_degrees(value, hemisphere)
    same = (rust.keys() == output.keys() and
            (rust.get('degrees') is None) == (output.get('degrees') is None) and
            ('degrees' not in rust or rust['degrees'] is None or
             abs(rust['degrees'] - output['degrees']) <= _DEGREES_TOLERANCE))
    if not same:
        case['divergence'] = dict(rust, note=_DEGREES_NOTE)
    return case


_I32_MAX: int = 2 ** 31 - 1


def _rust_degrees(value: str, hemisphere: str) -> dict:
    """
    The Rust port's answer to a degrees case (see the module notes): integer 1e-7 degree units from the digits.

    Args:
        value - the ddmm.mmmm field.
        hemisphere - its hemisphere field.

    Returns:
        {degrees} (None for an empty field) or {malformed}.
    """
    if not value:
        return {'degrees': None}
    whole, dot, fraction = value.partition('.')
    digits = '0123456789'
    if (not dot or len(whole) < 3 or any(character not in digits for character in whole) or
            any(character not in digits for character in fraction)):
        return {'malformed': True}
    fraction = fraction[:9]  # the Rust port keeps nine minute decimals (1e-9 minute; the rest cannot move a unit)
    scaled = int(whole[-2:] + fraction) if fraction else int(whole[-2:])
    denominator = 60 * 10 ** len(fraction)
    units = int(whole[:-2]) * 10 ** 7 + (scaled * 10 ** 7 * 2 + denominator) // (2 * denominator)
    if units > _I32_MAX:
        return {'malformed': True}
    return {'degrees': (-units if hemisphere in ('S', 'W') else units) / 1e7}


def _degrees_cases() -> list:
    """The degrees section: the fields of test_gnss's sentences, edges, plan B's slicing quirks, and a sweep."""
    cases = [
        ('test_latitude', '4807.038', 'N'), ('test_longitude_west', '01131.000', 'W'), ('test_empty', '', 'N'),
        ('burst_latitude', '4807.03800', 'N'), ('burst_longitude', '01131.00000', 'E'),
        ('equator', '0000.0000', 'N'), ('south_pole', '9000.0000', 'S'), ('date_line_west', '18000.0000', 'W'),
        ('last_minute_east', '17959.99999', 'E'), ('dot_without_decimals', '4807.', 'N'),
        ('lowercase_hemisphere_reads_north', '4807.038', 's'), ('unknown_hemisphere', '4807.038', 'X'),
        ('no_hemisphere', '4807.038', ''), ('ten_decimals', '4807.0380000001', 'N'),
        ('more_decimals_than_kept', '4807.038000000000012', 'N'), ('one_degree_digit', '123.5', 'N'),
        ('minutes_past_sixty', '4875.0', 'N'), ('no_dot', '4807', 'N'), ('two_digits_before_the_dot', '48.5', 'N'),
        ('one_digit_before_the_dot', '7.5', 'N'), ('dot_first', '.5', 'N'), ('negative', '-4807.038', 'N'),
        ('plus_sign', '+4807.038', 'N'), ('exponent', '4807.038e1', 'N'), ('letter_in_degrees', '48a7.038', 'N'),
        ('letter_in_minutes', '4807.0a8', 'N'), ('three_digit_degrees_999', '99959.99999', 'E'),
        ('past_an_i32_of_units', '21500.0', 'E'), ('just_inside_an_i32', '21400.0', 'W'),
        ('exactly_half_a_unit_rounds_up', '0000.000003', 'N'), ('a_third_of_a_unit_rounds_down', '0000.000002', 'S'),
    ]
    out = [_degrees_case(*case) for case in cases]
    generator = random.Random(_SEED)
    for index in range(_DEGREES_SWEEP):
        if index % 2:
            value, _degrees = _minutes_field(generator, 3, 180)
            hemisphere = generator.choice('EW')
        else:
            value, _degrees = _minutes_field(generator, 2, 90)
            hemisphere = generator.choice('NS')
        out.append(_degrees_case('sweep_%03d' % index, value, hemisphere))
    return out


def _checksum_cases() -> list:
    """The checksum section: every sentence above, the forms Python's int(.., 16) takes, and a seeded sweep."""
    lines = list(_ATGM_BURST + _ATGM_COLD + _NEO_BURST) + [
        _RMC, _GGA, '$GPRMC,123521,A,1234.000,N,01234.000,E,0,0,230394,,*00', '$GPTXT,01,01,01,ANTENNA OPEN*25',
        '$GPTXT,01,01,02,ANTSTATUS=INIT*25', '$GPTXT,01,01,01,ANTENNA OK*00']
    cases = [('sentence_%02d' % index, line) for index, line in enumerate(lines)]
    """A body whose XOR is under 16 (one hex digit: 'A' ^ 'B' is 3), and the forms Python's int(.., 16) reads."""
    small = 'AB'
    xor = ord('A') ^ ord('B')
    cases += [
        ('lowercase_hex', '$GNRMC,001847.900,A,4807.03800,N,01131.00000,E,0.00,0.00,051026,,,A,V*0b'),
        ('no_star', '$GPRMC,123519,A'), ('star_last', '$GPRMC,123519,A*'), ('star_only', '*'), ('empty', ''),
        ('nothing_between_dollar_and_star', '$*00'), ('one_hex_digit_wrong', '$GPRMC*5'),
        ('one_hex_digit', '$%s*%X' % (small, xor)), ('one_hex_digit_and_a_space', '$%s*%X ' % (small, xor)),
        ('space_then_digit', '$%s* %X' % (small, xor)), ('plus_sign', '$%s*+%X' % (small, xor)),
        ('minus_sign', '$%s*-%X' % (small, xor)), ('minus_zero', '$*-0'), ('trailing_text', _RMC + 'XYZ'),
        ('three_hex_digits', '$*000'), ('not_hex', '$GPRMC*G1'), ('two_stars', _line('GP*RMC,1')),
        ('no_dollar', 'X' + _RMC[1:]),
    ]
    generator = random.Random(_SEED)
    alphabet = ''.join(chr(code) for code in range(32, 127) if chr(code) not in '*$')
    for index in range(_CHECKSUM_SWEEP):
        body = ''.join(generator.choice(alphabet) for _ in range(generator.randrange(0, 70)))
        line = _line(body)
        kind = index % 3
        if kind == 1 and len(line) > 4:
            position = generator.randrange(1, len(line) - 3)
            line = line[:position] + generator.choice(alphabet) + line[position + 1:]
        elif kind == 2:
            line = line[:-2] + '%02X' % generator.randrange(256)
        cases.append(('sweep_%03d' % index, line))
    return [{'name': name, 'input': line, 'output': gnss.checksum_ok(line)} for name, line in cases]


def _bodies() -> list:
    """Every command body the drivers frame, and seeded printable bodies."""
    bodies = ['PCAS03,10,0,0,0,1,0,0,0,0,0,,,0,0', 'PCAS02,100', 'PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0',
              'PMTK220,100', 'PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0', 'PUBX,40,RMC,0,1,0,0,0,0',
              'PUBX,40,GSA,0,50,0,0,0,0', 'PUBX,40,GSV,0,50,0,0,0,0', 'GPRMC,123519,A', '']
    generator = random.Random(_SEED + 1)
    alphabet = ''.join(chr(code) for code in range(32, 127))
    for _index in range(_BODY_SWEEP):
        bodies.append(''.join(generator.choice(alphabet) for _ in range(generator.randrange(1, 80))))
    return bodies


def _ubx_cases() -> list:
    """The ubx section: the frames the NEO-6M init sends, an empty payload, and seeded class/id/payloads."""
    frames = [('cfg_rate_1000', 0x06, 0x08, struct.pack('<HHH', 1000, 1, 1)),
              ('cfg_nav5_airborne', 0x06, 0x24, struct.pack('<HB', 1, 8) + bytes(33)),
              ('empty_payload', 0x0A, 0x04, b''), ('all_ones', 0xFF, 0xFF, b'\xff' * 255)]
    generator = random.Random(_SEED + 2)
    for index in range(_UBX_SWEEP):
        payload = bytes(generator.randrange(256) for _ in range(generator.randrange(0, 120)))
        frames.append(('sweep_%03d' % index, generator.randrange(256), generator.randrange(256), payload))
    return [{'name': name, 'input': {'class': class_id, 'id': message_id, 'payload': payload.hex()},
             'output': neo6mv2._ubx(class_id, message_id, payload).hex()} for name, class_id, message_id, payload
            in frames]


"""The rates the commands are built for: the plan's 0/1/5/10, every edge of the dividers, and nonsense."""
_RATES: tuple = (-5, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 15, 20, 24, 25, 26, 30, 33, 50, 99, 100, 101, 999,
                 1000, 1001, 5000, 100000)


def _command_cases() -> list:
    """
    The commands section: each driver's _configure(hz) on a recording writer: the frames, then the sky pair.

    Returns:
        The cases: {name, input: {driver, hz}, output: {frames, sky_off, sky_on}} (bytes as hex).
    """
    cases = []
    for driver in ('atgm336h', 'neo6mv2'):
        unit = _receiver(driver, 10)
        for hz in _RATES:
            unit._writer = _Writer()
            off, on = asyncio.run(unit._configure(hz))
            frames = [frame.hex() for frame in unit._writer.frames]
            module = atgm336h if driver == 'atgm336h' else neo6mv2
            if frames != [frame.hex() for frame in module._commands(hz)]:
                raise ValueError('%s %d: _configure() did not send _commands()' % (driver, hz))
            cases.append({'name': '%s_hz_%d' % (driver, hz), 'input': {'driver': driver, 'hz': hz},
                          'output': {'frames': frames, 'sky_off': off.hex(), 'sky_on': on.hex()}})
    return cases


def build() -> dict:
    """
    The gnss fixture document.

    Args:
        (none)

    Returns:
        {meta, checksum, degrees, nmea, ubx, commands, trace}: each a golden fixture of its own.
    """
    _install()
    generator = random.Random(_SEED + 3)
    scenarios = [
        _scenario('canned', 'atgm336h', 10, _canned()),
        _scenario('atgm_sky', 'atgm336h', 10, _atgm_sky()),
        _scenario('neo_sky', 'neo6mv2', 5, _parses(_NEO_BURST, 100) + [(0, 'settle', 1000000)]),
        _scenario('gsa_systems', 'neo6mv2', 5, _gsa_systems()),
        _scenario('quiet_restart', 'atgm336h', 10, _quiet_restart()),
        _scenario('window', 'atgm336h', 10, _window()),
        _scenario('window_neo', 'neo6mv2', 5, _window()),
        _scenario('clocked', 'atgm336h', 10, _clocked()),
        _scenario('warm_start', 'atgm336h', 10, _warm_start()),
        _scenario('run_lines', 'atgm336h', 10, _run_lines()),
        _scenario('rmc_sweep', 'atgm336h', 10, _rmc_sweep(generator)),
        _scenario('gga_sweep', 'neo6mv2', 1, _gga_sweep(generator)),
    ]
    return {
        'meta': {'source': ('src/glider/gnss.py, src/glider/drivers/atgm336h.py, src/glider/drivers/neo6mv2.py, '
                            'src/glider/test/test_gnss.py')},
        'checksum': {'compare': {'kind': 'exact'}, 'note': 'checksum_ok(line)', 'cases': _checksum_cases()},
        'degrees': {
            'compare': {'kind': 'abs', 'tol': _DEGREES_TOLERANCE},
            'note': ('degrees(ddmm, hemisphere): the Rust 1e-7 degree units are at most half a unit (5e-8 degree) '
                     'from plan B'),
            'cases': _degrees_cases(),
        },
        'nmea': {'compare': {'kind': 'bytes_hex'}, 'note': 'nmea(body): the framed sentence',
                 'cases': [{'name': 'body_%03d' % index, 'input': body, 'output': gnss.nmea(body).hex()}
                           for index, body in enumerate(_bodies())]},
        'ubx': {'compare': {'kind': 'bytes_hex'}, 'note': '_ubx(class, id, payload): the UBX frame',
                'cases': _ubx_cases()},
        'commands': {'compare': {'kind': 'bytes_hex'},
                     'note': 'the frames _configure(hz) sends, in order, and the (off, on) sky switch it returns',
                     'cases': _command_cases()},
        'trace': {
            'compare': {'degrees': {'kind': 'abs', 'tol': _DEGREES_TOLERANCE},
                        'speed': {'kind': 'rel', 'tol': _SPEED_TOLERANCE},
                        'values': {'kind': 'abs', 'tol': _VALUES_TOLERANCE},
                        'exact': {'kind': 'exact'}},
            'note': ('per step: the input (with the uptime it ran at) and the answer in four parts; a divergence '
                     'overrides members of a part. speed is relative: at most three f32 roundings (the parse, the '
                     'knots constant, the product) stay under 1.8e-7, and the exact 1852/3600 in place of plan B\'s '
                     '0.514444 is 8.6e-7 away'),
            'scenarios': scenarios,
        },
    }
