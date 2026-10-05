"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the shared GNSS base (gnss.py): the NMEA helpers and the Gnss base Task -- graceful
setup on an undefined bus, and that a parsed RMC/GGA lands on the databoard (fed canned sentences, so
it is deterministic without a satellite fix), including the GGA-derived elevation. A trivial
_StubGnss subclass exercises the base independently of any module driver. The sky diagnostics (GSV,
GSA, the antenna text -> one `<name>_sky.csv` row per burst, events, inspect) are fed real bursts
captured on the 2026-10-04 bench (verbatim but one synthetic RMC position), then once more on the real
clock: an ATGM and a NEO burst each arriving over time through the run loop's 1 s sweep, and a full
telemetry ring. The diagnostics run on the pad only: the window per stage, one write per edge, the
events, a warm start into GLIDING switching them off at its first tick, and no allocation in a switch.
The three streams' declarations (file name, fields, decimation) are checked as built. Run by `make test`.
"""

import asyncio
import gc
import time

import config_default
import controller
import databoard
import gnss
import recorder


class _FakeWriter:
    """The Recorder's UART: takes everything, keeps nothing."""

    def write(self, data: bytes) -> None:
        """Drop `data`."""
        pass

    async def drain(self) -> None:
        """Nothing to flush."""
        pass


class _StubController:
    """The controller as a task sees it: the config, and the flight stage the sky switch reads."""

    config: dict = config_default.default()
    stage: int = controller.Stage.SETTING


class _StubGnss(gnss.Gnss):
    """A concrete Gnss that sends nothing -- exercises the base; its sky switch is two marker frames."""

    async def _configure(self, hz: int) -> tuple:
        """No module commands; (off, on) are b'off' and b'on', so a test can tell which one was written."""
        return b'off', b'on'


class _Rows:
    """Stands in for a Telemetry stream: keeps every pushed row."""

    def __init__(self) -> None:
        self.rows: list = []

    def push(self, values: tuple) -> None:
        """Keep the row."""
        self.rows.append(values)


class _FullRows:
    """Stands in for a Telemetry stream whose ring is full: every push raises, as the Recorder's does."""

    def __init__(self) -> None:
        self.attempts: int = 0

    def push(self, values: tuple) -> None:
        """Count the attempt and refuse it."""
        self.attempts += 1
        raise recorder._RecorderError('telemetry ring full')


class _Writes:
    """Stands in for the GNSS UART's writer: counts the writes and keeps the last, allocating nothing."""

    def __init__(self) -> None:
        self.count: int = 0
        self.last: bytes = None

    def write(self, data: bytes) -> None:
        """Count the write and keep `data` (a reference: no copy)."""
        self.count += 1
        self.last = data

    async def drain(self) -> None:
        """Nothing to flush."""
        pass


def _ignore(message: str) -> None:
    """An event sink that keeps nothing, so an allocation measurement sees the sky switch alone."""
    pass


def _allocated(unit: gnss.Gnss, stage: int) -> tuple:
    """
    Run one _sky_window(stage) with the GC off, as in flight, and measure the heap.

    Args:
        unit - the Gnss under test (its writer and event sink must allocate nothing).
        stage - the flight stage to switch for.

    Returns:
        (whether it wrote, the bytes the heap grew by).
    """
    gc.collect()
    gc.disable()
    try:
        before = gc.mem_alloc()
        switched = unit._sky_window(stage)
        spent = gc.mem_alloc() - before
    finally:
        gc.enable()
    return switched, spent


"""
One ATGM burst from the bench (session 5: the small active antenna, fixed): GSA per system (1 GPS, 4 BDS
-- PRN 22 is in BOTH, two satellites), a 4-part GPS group whose last part holds one satellite, PRN 27 in
view with an EMPTY C/N0, a 2-part BDS group, then the epoch's RMC and, after it, the antenna text. All
verbatim but the RMC's position: the synthetic 4807.038 N 01131.000 E (checksum recomputed), since no
tracked file holds the bench site's. The checker's own summary of that moment: in_view=20, sats=17,
cn0=44,40,40,40, OK.
"""
_ATGM_BURST = (
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
"""The same ATGM cold, before its first fix (session 5, first second): mode 1, nothing used, one GPS satellite."""
_ATGM_COLD = (
    '$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,1*01',
    '$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,4*04',
    '$GPGSV,1,1,01,01,,,39,0*6F',
    '$BDGSV,1,1,00,0*74',
)
"""A NEO-6M burst, verbatim (session 6): NMEA 2.3 -- one GSA with no system id, GSV without a signal id."""
_NEO_BURST = (
    '$GPGSA,A,3,07,02,08,01,46,30,09,14,17,04,27,22,1.99,0.98,1.74*06',
    '$GPGSV,3,1,12,01,43,130,27,02,50,092,20,04,09,178,32,07,81,335,25*79',
    '$GPGSV,3,2,12,08,41,035,22,09,26,208,39,14,27,300,42,17,27,233,38*72',
    '$GPGSV,3,3,12,22,13,286,40,27,05,045,12,30,43,319,40,46,29,248,43*79',
)


def _after(microseconds: int) -> int:
    """The time (ticks_us) `microseconds` from now: a clock for _settle() that needs no sleep."""
    return time.ticks_add(time.ticks_us(), microseconds)


def _spec(name: str) -> dict:
    """The default config's sensor called `name`; {} when there is none."""
    for sensor in config_default.default()['sensors']:
        if sensor['name'] == name:
            return sensor
    return {}


def _line(body: str) -> str:
    """A valid NMEA sentence (correct checksum) around `body`, for feeding _parse."""
    return gnss.nmea(body).decode().strip()


async def amain():
    # NMEA helpers: checksum + ddmm.mmmm -> decimal degrees with hemisphere sign
    assert gnss.checksum_ok(_line('GPRMC,123519,A'))
    assert not gnss.checksum_ok('$GPRMC,123519,A*00')
    assert abs(gnss.degrees('4807.038', 'N') - 48.1173) < 1e-3
    assert gnss.degrees('01131.000', 'W') < 0 and gnss.degrees('', 'N') is None

    # an undefined bus -> graceful False, no UART touched
    no_bus = _StubGnss('gnss', {'bus': 'uart', 'id': 9}, _StubController())
    assert await no_bus.setup() is False

    # real uart:2: build the base (configures nothing), then feed canned NMEA
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    unit = _StubGnss('gnss', _spec('gnss'), _StubController())
    assert await unit.setup() is True

    # a valid RMC -> position on the databoard, fix True
    unit._parse(_line('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W'))
    latitude, longitude = databoard.Databoard.value('position')
    assert abs(latitude - 48.1173) < 1e-3 and abs(longitude - 11.5167) < 1e-3 and unit._fix
    # RMC field-7 speed (knots) -> 'speed' channel in m/s for the airspeed governor (022.4 kn ~= 11.52 m/s)
    assert abs(databoard.Databoard.value('speed') - 22.4 * 0.514444) < 1e-2
    # RMC field-8 course (deg) -> 'course' channel: the attitude backup's absolute yaw reference
    assert abs(databoard.Databoard.value('course') - 84.4) < 1e-2

    # GGA -> altitude + elevation: the first valid GGA fixes the ground (elevation 0), next is the delta
    unit._parse(_line('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'))
    assert abs(databoard.Databoard.value('altitude') - 545.4) < 1e-3
    assert abs(databoard.Databoard.value('elevation')) < 1e-3 and abs(unit._ground - 545.4) < 1e-3
    # GGA also carries the signal quality (for antenna/sky checks): fix quality 1, 8 satellites, HDOP 0.9
    assert unit._fix_quality == 1 and unit._satellites == 8 and abs(unit._hdop - 0.9) < 1e-3
    unit._parse(_line('GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,550.4,M,46.9,M,,'))
    assert abs(databoard.Databoard.value('elevation') - 5.0) < 1e-3  # 550.4 - 545.4 ground

    # a void fix (status V) clears `fix` and does NOT move position; a bad checksum is ignored
    previous = databoard.Databoard.value('position')
    unit._parse(_line('GPRMC,123520,V,,,,,,,230394,,'))
    assert unit._fix is False and databoard.Databoard.value('position') == previous
    unit._parse('$GPRMC,123521,A,1234.000,N,01234.000,E,0,0,230394,,*00')  # bad checksum
    assert databoard.Databoard.value('position') == previous

    """
    RMC and GGA outages are EVENTED separately, because they fail separately -- a receiver holding
    almanac but no fix keeps emitting GGA (quality 0) while RMC goes void. Guidance dead-reckons
    through a GNSS loss by design, so the trajectory looks normal and these events are the only record
    that the receiver was dead; they go to the durable per-component stream, not to best-effort logs.
    """
    events = []
    unit.event = lambda message: events.append(message)
    unit._seen_us = {'RMC': time.ticks_us(), 'GGA': time.ticks_us()}
    unit._absent = {'RMC': False, 'GGA': False}
    unit._sweep(time.ticks_add(time.ticks_us(), 1000000))  # 1 s quiet -> jitter, not an outage
    assert events == []
    unit._sweep(time.ticks_add(time.ticks_us(), 4000000))  # 4 s -> BOTH sentences are out
    assert len(events) == 2 and any('RMC lost' in e for e in events) and any('GGA lost' in e for e in events)
    unit._sweep(time.ticks_add(time.ticks_us(), 5000000))  # still out -> each edge fires ONCE, no spam
    assert len(events) == 2
    # only RMC comes back -> exactly one recovery event, and GGA stays flagged out
    unit._parse(_line('GPRMC,123522,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W'))
    assert len(events) == 3 and 'RMC back' in events[2]
    assert unit._absent['GGA'] is True and unit._absent['RMC'] is False

    """
    The three streams as declared: file name, field order and decimation. Below, the sky stream is swapped
    for a stub, so this is where a renamed file, reordered fields or a dropped decimate_us shows -- and the
    host's recorder_flight reads a capture by exactly these (src/control's test_tools checks its copy).
    A per-component telemetry_ms makes the decimation visible: the default config leaves it at 0.
    """
    spec = dict(_spec('gnss'))
    spec['telemetry_ms'] = 250
    declared = _StubGnss('gnss', spec, _StubController())
    assert await declared.setup() is True
    streams = (declared._telemetry, declared._gga_telemetry, declared._sky_telemetry)
    assert [(stream.filename, stream.fields, stream.decimate_us) for stream in streams] == [
        ('gnss.csv', ('lat', 'lon', 'speed_kn', 'course'), 250000),
        ('gnss_gga.csv', ('altitude_m', 'elevation_m', 'quality', 'satellites', 'hdop_cd'), 250000),
        ('gnss_sky.csv', ('in_view', 'used', 'mode', 'cn0_1', 'cn0_2', 'cn0_3', 'cn0_4', 'antenna'), 250000),
    ], [(stream.filename, stream.fields, stream.decimate_us) for stream in streams]

    """
    The sky: a burst of GSA/GSV/antenna text every ~10 s, recorded as ONE row once a 1 s clock tick finds
    it quiet for 0.5 s. Nothing but RMC/GGA so far -- the HITL case, the sim publishes no GSV: no row
    ever, and inspect() reads an empty sky.
    """
    sky = _Rows()
    unit._sky_telemetry = sky
    events.clear()
    unit._settle(_after(5000000))
    assert sky.rows == [] and events == []
    status = unit.inspect()
    assert (status['in_view'], status['used'], status['mode'], status['antenna']) == (0, 0, 0, 'unknown')
    assert tuple(status['cn0']) == (0, 0, 0, 0) and status['cn0_mean'] == 0

    # the cold burst: GSA mode 1 per system, one GPS satellite (NMEA 4.1 signal id after it), BDS group EMPTY
    for line in _ATGM_COLD:
        unit._parse(line)
    unit._settle(_after(100000))  # 0.1 s on: the burst may still be arriving -> no row yet
    assert sky.rows == []
    unit._settle(_after(1000000))  # 1 s of quiet: the burst is over -> ONE row
    assert sky.rows == [(1, 0, 1, 39, 0, 0, 0, 0)], sky.rows
    assert unit._views['GP'] == (1, [39]) and unit._views['BD'] == (0, [])  # the signal id '0' is no C/N0
    assert len(events) == 1 and events[0].startswith('first sky view: 1 in view'), events
    unit._settle(_after(2000000))  # nothing new -> no second row
    assert len(sky.rows) == 1

    # the fixed burst: used is the SUM over systems (PRN 22 twice), the best mode, the top four of all
    unit._fix = False
    for line in _ATGM_BURST:
        unit._parse(line)
    assert unit._fix  # its RMC parsed: the synthetic position's recomputed checksum holds
    unit._settle(_after(1000000))
    assert sky.rows[1] == (20, 17, 3, 44, 40, 40, 40, 1), sky.rows
    assert len(events) == 2 and events[1] == 'antenna OK (was unknown)', events  # no second 'first sky view'
    status = unit.inspect()
    assert (status['in_view'], status['used'], status['mode'], status['antenna']) == (20, 17, 3, 'OK')
    assert tuple(status['cn0']) == (44, 40, 40, 40) and status['cn0_mean'] == 41.0

    # the antenna text: an event on a CHANGE only; another text, or a bad checksum, changes nothing
    unit._parse('$GPTXT,01,01,01,ANTENNA OK*35')  # the same status again (every burst repeats it)
    assert len(events) == 2
    unit._parse('$GPTXT,01,01,01,ANTENNA OPEN*25')  # the passive 12x12 patch (bench session 6)
    unit._parse(_line('GPTXT,01,01,01,ANTENNA SHORT'))
    assert events[2:] == ['antenna OPEN (was OK)', 'antenna SHORT (was OPEN)'], events
    unit._parse('$GPTXT,01,01,02,ANTSTATUS=INIT*25')  # the NEO-6M's boot text: no antenna status
    unit._parse('$GPTXT,01,01,01,ANTENNA OK*00')  # bad checksum
    assert len(events) == 4 and unit.inspect()['antenna'] == 'SHORT'
    unit._settle(_after(1000000))  # the antenna texts were a burst too: one row
    assert len(sky.rows) == 3 and sky.rows[2] == (20, 17, 3, 44, 40, 40, 40, 3), sky.rows
    unit._parse('$GPTXT,01,01,02,ANTSTATUS=INIT*25')  # a text that is not the sky opens no burst
    unit._settle(_after(1000000))
    assert len(sky.rows) == 3

    # a GSV group replaces its talker's view only WHOLE and IN ORDER: none of these touch the GPS view
    unit._parse('$GPGSV,4,2,13,08,48,033,20,09,32,214,40,14,23,294,38,17,21,228,40,0*65')  # part 2, no part 1
    unit._parse(_line('GPGSV,3,1,11,01,43,131,50,02,50,092,50,04,09,179,50,07,81,335,50,0'))  # part 1 ...
    unit._parse(_line('GPGSV,3,3,11,22,13,287,50,27,05,045,50,30,43,319,50,0'))  # ... then part 3: out of order
    unit._parse(_line('GPGSV,2,1,05,01,43,131,51,02,50,092,51,04,09,179,51,07,81,335,51,0'))  # part 1 ...
    unit._parse('$GPGSV,2,2,05,08,41,035,51,0*00')  # ... and its last part with a bad checksum: never whole
    unit._parse(_line('BDGSV,2,2,07,29,48,192,52,36,59,003,52,39,23,312,52,0'))  # BDS part 2 after GPS part 1
    unit._settle(_after(1000000))
    assert sky.rows[-1] == (20, 17, 3, 44, 40, 40, 40, 3), sky.rows

    # satellites in view but none tracked: an empty C/N0 is no signal, not a 0 dB-Hz one
    unit._parse(_line('GPGSV,1,1,02,01,43,131,,02,50,092,'))
    assert unit._views['GP'] == (2, [])
    # a system losing its fix: the mode is the BEST system's, the used count drops by that system's
    unit._parse(_line('GNGSA,A,1,,,,,,,,,,,,,9.9,9.9,9.9,4'))
    unit._settle(_after(1000000))
    assert sky.rows[-1] == (9, 10, 3, 44, 40, 37, 30, 3), sky.rows

    # the NEO-6M's NMEA 2.3 burst on a fresh sky: a GSA with no system id, no signal id, no antenna text
    unit._views = {}
    unit._systems = {}
    unit._antenna = 0
    for line in _NEO_BURST:
        unit._parse(line)
    unit._settle(_after(1000000))
    assert sky.rows[-1] == (12, 12, 3, 43, 42, 40, 40, 0), sky.rows
    assert unit.inspect()['antenna'] == 'unknown'

    """
    Every GSV, GSA and antenna text restarts the quiet clock; any other text does not. A burst left pending
    1 s ago (no tick has run since), then one more sentence: 0.2 s on, a restarting one holds the row back
    and the other lets it go; a second on, the burst is over either way.
    """
    for line, restarts in ((_NEO_BURST[0], True), (_NEO_BURST[1], True), ('$GPTXT,01,01,01,ANTENNA OK*35', True),
                           ('$GPTXT,01,01,02,ANTSTATUS=INIT*25', False)):
        rows = len(sky.rows)
        unit._pending = True
        unit._pending_us = time.ticks_add(time.ticks_us(), -1000000)
        unit._parse(line)
        unit._settle(_after(200000))
        assert len(sky.rows) == rows + (0 if restarts else 1), (line, sky.rows)
        unit._settle(_after(1000000))
        assert len(sky.rows) == rows + 1, (line, sky.rows)

    """
    The diagnostics run on the pad only: on before BOOSTING and again from DONE, off from BOOSTING to DONE
    (_sky_window(), on the sweep tick). Every stage from both states: a write -- the frame the driver made
    at setup -- and an event only when the window changes, nothing while it holds.
    """
    assert unit._diagnosing is True and (unit._sky_off, unit._sky_on) == (b'off', b'on')  # as setup left it
    writes = _Writes()
    unit._writer = writes
    for stage, diagnosing in ((controller.Stage.SETTING, True), (controller.Stage.BOOSTING, False),
                              (controller.Stage.GLIDING, False), (controller.Stage.LANDING, False),
                              (controller.Stage.DONE, True)):
        for before in (True, False):
            unit._diagnosing = before
            count = writes.count
            events.clear()
            switched = unit._sky_window(stage)
            assert unit._diagnosing is diagnosing, (stage, before)
            if before == diagnosing:
                assert not switched and writes.count == count and events == [], (stage, before, events)
            elif diagnosing:
                assert switched and writes.count == count + 1 and writes.last == b'on', (stage, writes.last)
                assert events == ['sky diagnostics on'], (stage, events)
            else:
                assert switched and writes.count == count + 1 and writes.last == b'off', (stage, writes.last)
                assert events == ['sky diagnostics off for flight'], (stage, events)

    # a flight in order, every tick repeating its stage: one write at launch, one at DONE, none between
    unit._diagnosing = True
    count = writes.count
    events.clear()
    stages = controller.Stage
    for stage in (stages.SETTING, stages.SETTING, stages.BOOSTING, stages.BOOSTING, stages.GLIDING,
                  stages.GLIDING, stages.LANDING, stages.LANDING, stages.DONE, stages.DONE):
        unit._sky_window(stage)
    assert writes.count == count + 2 and writes.last == b'on', (writes.count - count, writes.last)
    assert events == ['sky diagnostics off for flight', 'sky diagnostics on'], events
    # a launch called off: SETTING straight to DONE stays on the pad, no write
    count = writes.count
    for stage in (stages.SETTING, stages.DONE):
        unit._sky_window(stage)
    assert writes.count == count and unit._diagnosing is True

    """
    A switch allocates nothing -- the GC is off from BOOSTING: the frames are the driver's, made at setup,
    and the write passes them on. The event allocates its row by design (task.event()), so a sink that keeps
    nothing stands in for it here; neither the hold nor either edge may grow the heap.
    """
    unit.event = _ignore
    unit._diagnosing = True
    for stage, switched in ((controller.Stage.BOOSTING, True), (controller.Stage.GLIDING, False),
                            (controller.Stage.DONE, True)):
        measured = _allocated(unit, stage)
        assert measured == (switched, 0), (stage, measured)
    unit.event = lambda message: events.append(message)

    """
    On the real clock, the path that actually pushes rows: _sweeping() -- the 1 s tick run() starts --
    calling _settle(). A real burst is spread over ~0.5 s, so each one here arrives in two parts 0.4 s apart
    with a tick between them, 0.1 s after the first part. The quiet window runs from the burst's LAST
    sentence: that tick and the settles 0.2 s (and for the ATGM 0.4 s) after the last sentence all wait,
    and the next tick, 0.7 s after it, pushes the burst as ONE row. Times below are from the sweepers'
    start, ticks at ~1, 2, 3, 4 and 5 s. A tick sees 0.1 s or 0.7 s of quiet: >= 0.2 s from the 0.5 s
    window and from a 0.3 s one, and every assert clears a tick by >= 0.3 s, so a late tick or a GC pause
    cannot flip one. The settles take no clock: only the parse before them eats into their margin.

    A second unit, reset into GLIDING (a warm start), sweeps alongside: its init left the diagnostics on,
    as at any boot, and its first tick switches them off. The first unit stays in SETTING: no write.
    """
    unit._views = {}
    unit._systems = {}
    unit._antenna = 0
    sky = _Rows()
    unit._sky_telemetry = sky
    writes = _Writes()
    unit._writer = writes
    airborne = _StubController()
    airborne.stage = controller.Stage.GLIDING  # restored by the warm start before the tasks run
    warm = _StubGnss('gnss', _spec('gnss'), airborne)
    assert await warm.setup() is True and warm._diagnosing is True
    warm_writes = _Writes()
    warm._writer = warm_writes
    warm_events = []
    warm.event = lambda message: warm_events.append(message)
    sweeper = asyncio.create_task(unit._sweeping())
    warm_sweeper = asyncio.create_task(warm._sweeping())
    await asyncio.sleep_ms(900)
    for line in _ATGM_BURST[:4]:  # ~0.9 s: the GSAs and the GPS group's first two parts ...
        unit._parse(line)
    await asyncio.sleep_ms(400)  # ... the ~1 s tick finds them 0.1 s quiet: mid-burst, no row ...
    assert sky.rows == [], sky.rows
    assert (warm_writes.count, warm_writes.last, warm._diagnosing) == (1, b'off', False), warm_writes.count
    assert warm_events == ['sky diagnostics off for flight'], warm_events  # the warm start's first tick
    for line in _ATGM_BURST[4:]:  # ... ~1.3 s: the rest, the RMC and the antenna text last
        unit._parse(line)
    unit._settle(_after(200000))  # 0.6 s after the burst's first sentence, 0.2 s after its last: no row
    unit._settle(_after(400000))  # 0.4 s after its last: still inside the 0.5 s window
    assert sky.rows == [], sky.rows
    await asyncio.sleep_ms(1000)  # ~2.3 s: the ~2 s tick found it 0.7 s quiet -> ONE row, the whole burst
    assert sky.rows == [(20, 17, 3, 44, 40, 40, 40, 1)], sky.rows
    assert warm_writes.count == 1 and len(warm_events) == 1  # its second tick: the window holds

    # a full telemetry ring: the tick's push raises, that burst is spent, and the clock -- the outage
    # sweep's too -- lives on, so the next burst still makes its row
    full = _FullRows()
    unit._sky_telemetry = full
    for line in _ATGM_COLD:  # ~2.3 s
        unit._parse(line)
    await asyncio.sleep_ms(1000)  # ~3.3 s: the ~3 s tick tried
    assert full.attempts == 1 and not unit._pending and not sweeper.done(), (full.attempts, unit._pending)

    # the NEO's burst ends on a GSV, with no antenna text after it: its GSVs must restart the quiet window
    unit._sky_telemetry = sky
    unit._views = {}
    unit._systems = {}
    unit._antenna = 0
    await asyncio.sleep_ms(600)
    for line in _NEO_BURST[:2]:  # ~3.9 s: the GSA and the first GSV ...
        unit._parse(line)
    await asyncio.sleep_ms(400)  # ... the ~4 s tick finds them 0.1 s quiet: no row ...
    assert len(sky.rows) == 1, sky.rows
    for line in _NEO_BURST[2:]:  # ... ~4.3 s: the last two GSVs
        unit._parse(line)
    unit._settle(_after(200000))  # 0.6 s after the GSA, 0.2 s after the last GSV: no row
    assert len(sky.rows) == 1, sky.rows
    await asyncio.sleep_ms(1000)  # ~5.3 s: the ~5 s tick found it 0.7 s quiet -> ONE row, the whole burst
    assert sky.rows == [(20, 17, 3, 44, 40, 40, 40, 1), (12, 12, 3, 43, 42, 40, 40, 0)], sky.rows
    assert writes.count == 0 and unit._diagnosing is True  # five ticks in SETTING: the window held
    assert warm_writes.count == 1 and not warm_sweeper.done()
    # the stage moves UNDER the running tick: read on every tick, not once (a launch, then the landing)
    unit.controller.stage = controller.Stage.BOOSTING
    await asyncio.sleep_ms(1000)
    assert (writes.count, writes.last) == (1, b'off'), (writes.count, writes.last)
    unit.controller.stage = controller.Stage.DONE
    await asyncio.sleep_ms(1000)
    assert (writes.count, writes.last) == (2, b'on'), (writes.count, writes.last)
    sweeper.cancel()
    warm_sweeper.cancel()

    print('ok: gnss base -- NMEA helpers; RMC->position+speed, GGA->altitude+elevation(ground)+quality; '
          'per-sentence outage events; void/bad ignored; the three streams as declared; sky row once per '
          'burst (GSV groups whole + in order, GSA summed per system, antenna events on change, GSV/GSA/antenna '
          'restart the quiet clock), empty without diagnostics; pad-only diagnostics (window per stage, one '
          'write per edge, events, no allocation); on the clock: an ATGM and a NEO burst arriving over time are '
          'one row each, a full ring keeps the clock, a warm start into GLIDING switches off at its first tick')


asyncio.run(amain())
