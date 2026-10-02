"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the ANALYSIS TOOLS (tools/flight_kpi, flight_svg, airspeed_calibrate, and the
board-shape handling in flight_telemetry). Stdlib only -- plotly-dependent rendering is not exercised.

Why this file exists (findings §27.8): ~4 K lines of analysis tooling had almost no tests, and it is the
layer that produces the CONCLUSIONS we draw from a flight -- a silent bug here is worse than a firmware
bug, because it corrupts the answer rather than announcing itself. The §26 scan found 17 defects in these
tools; every one of them is pinned below so it cannot come back. Run by `make test` / `make check`.
"""

import json
import os
import re
import sys
import tempfile

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import airspeed_calibrate  # noqa: E402
import cc  # noqa: E402
import flight_kpi  # noqa: E402
import flight_report  # noqa: E402
import flight_svg  # noqa: E402
import flight_synth_capture  # noqa: E402
import flight_telemetry  # noqa: E402

_ZONE = ((25.514944, -80.392972), (25.514583, -80.391111))  # the HPRC strip (TL, BR)


def _board_capture() -> str:
    """
    A capture in the BOARD's shape: per-servo streams instead of the sim's fused fins.csv.

    Event-based rows at staggered stamps, exactly as sg90's compare-and-set writes them.
    """
    lines = []

    def tlm(name, row):
        lines.append('@20260725_120000_%s@%s' % (name, row))

    for fin in ('servo_eleron_left', 'servo_eleron_right', 'servo_yaw'):
        tlm('%s.csv' % fin, 'uptime;angle;pulse_us;done')
    tlm('servo_eleron_left.csv', '1000000;80;1000;1')
    tlm('servo_eleron_right.csv', '1000500;100;2000;1')   # 0.5 ms later -- no shared timeline
    tlm('servo_yaw.csv', '1002000;90;1500;1')
    tlm('servo_eleron_left.csv', '2000000;70;900;1')      # only the left fin moves again
    return '\n'.join(lines)


def test_board_shape_is_readable():
    """
    A BOARD capture renders like a sim one: the fused `fins` shape is rebuilt from per-servo streams.

    findings §27.1 -- the tools were written against sim captures and would have come back SILENTLY
    EMPTY on the first real flight. Un-moved fins must FORWARD-FILL (a servo holds its last command).
    """
    streams, _logs = flight_telemetry.parse(_board_capture())
    fins = flight_telemetry.find_stream(streams, 'eleron_left', 'eleron_right', 'yaw')
    assert fins is not None, 'per-servo streams must rebuild the fused fins shape'
    assert fins.column('eleron_left')[1][-1] == 70.0   # moved again
    assert fins.column('eleron_right')[1][-1] == 100.0  # held -- forward-filled, not dropped
    assert fins.column('yaw')[1][-1] == 90.0
    # a capture that ALREADY has a fused stream is left alone (the sim case)
    sim = '\n'.join(['@20260725_120000_fins.csv@uptime;eleron_left;eleron_right;yaw',
                     '@20260725_120000_fins.csv@1000000;11;22;33'])
    assert flight_telemetry.parse(sim)[0]['fins.csv'].column('eleron_left')[1] == [11.0]


def test_kpi_on_the_synthetic_flight():
    """The KPI numbers on a deterministic capture -- the golden values a refactor must preserve."""
    streams, _logs = flight_telemetry.parse(flight_synth_capture.generate())
    rows, span = flight_kpi._fin_activity(flight_telemetry.find_stream(streams, *flight_kpi._FINS))
    assert span > 0.0, 'a zero span would divide by zero in the moves/s report (§26.23)'
    assert len(rows) == len(flight_kpi._FINS)
    """
    §26.30: `span` used to be reassigned inside the per-fin loop, so every fin's moves/s was divided by
    the LAST fin's window. One span now covers all fins, and it spans the whole fin timeline.
    """
    times = flight_telemetry.find_stream(streams, *flight_kpi._FINS).column('eleron_left')[0]
    assert span >= (times[-1] - times[0]) - 1e-6, (span, times[-1] - times[0])
    miss, inside = flight_kpi._touchdown(flight_telemetry.find_stream(streams, 'lat', 'lon'), _ZONE)
    assert miss >= 0.0 and isinstance(inside, bool)


def test_kpi_survives_a_partial_capture():
    """
    Empty / short streams must not crash the analysis (§26.18, §26.19, §26.24).

    A partial capture is the NORMAL case for an aborted or degraded flight -- exactly when you most want
    the numbers -- so every one of these used to IndexError instead of reporting what it had.
    """
    empty, _logs = flight_telemetry.parse('@20260725_120000_power_ina226.csv@uptime;power_mw')
    assert flight_kpi._servo_energy(flight_telemetry.find_stream(empty, 'power_mw')) == (0.0, 0.0)
    gnss_only_header, _l = flight_telemetry.parse('@20260725_120000_gnss.csv@uptime;lat;lon')
    # no fix is None, never a 0.0 m miss -- that would print as a bullseye
    assert flight_kpi._touchdown(flight_telemetry.find_stream(gnss_only_header, 'lat', 'lon'), _ZONE) is None
    assert flight_kpi._servo_energy(None) == (0.0, 0.0)  # stream absent entirely
    assert flight_kpi._touchdown(None, _ZONE) is None
    # a single power sample has no window to average over -> must not divide by zero
    one, _l = flight_telemetry.parse('@20260725_120000_power_ina226.csv@uptime;power_mw\n'
                                     '@20260725_120000_power_ina226.csv@1000000;2500')
    joules, duration = flight_kpi._servo_energy(flight_telemetry.find_stream(one, 'power_mw'))
    assert duration == 0.0 and joules == 0.0


def _kpi_report(text: str) -> str:
    """Run flight_kpi.report() over a capture given as text; return what it printed."""
    import contextlib
    import io
    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as handle:
        handle.write(text)
    printed = io.StringIO()
    try:
        with contextlib.redirect_stdout(printed):
            flight_kpi.report('capture', handle.name, _ZONE)
    finally:
        os.remove(handle.name)
    return printed.getvalue()


def test_touchdown_is_the_last_fix_before_done():
    """
    The touchdown is where the glider was at DONE, not where the GNSS was last carried.

    The GNSS records on after the landing, so the last row of a real flight is the recovery walk. With
    no fins stream (TMS-7C) the touchdown used to be dropped altogether, and with no fix it printed
    0.0 m -- a bullseye. Negative cases: no DONE falls back to the last fix, and no fix says so.
    """
    inside, walked = _ZONE[0][0] - 0.0001, _ZONE[0][0] + 0.01   # in the zone; ~1 km north of it
    rows = ['@20260725_120000_gnss.csv@uptime;lat;lon;speed_kn;course']
    for second, latitude in ((0, walked), (1, inside), (2, inside), (4, walked), (5, walked)):
        rows.append('@20260725_120000_gnss.csv@%d;%.6f;%.6f;0.0;0.0'
                    % (second * 1000000, latitude, _ZONE[0][1] + 0.001))
    landed = '\n'.join(rows + ['3000000 controller :: stage -> done'])
    streams, logs = flight_telemetry.parse(landed)
    done_s = flight_kpi._done_time(logs)
    assert done_s == 3.0, done_s
    miss, in_zone = flight_kpi._touchdown(flight_telemetry.find_stream(streams, 'lat', 'lon'), _ZONE, done_s)
    assert in_zone and miss < 100.0, (miss, in_zone)
    printed = _kpi_report(landed)
    assert '(no fins stream)' in printed and 'inside zone: True' in printed, printed
    # no DONE logged: the last fix is all there is, and the report says which one it took
    carried = _kpi_report('\n'.join(rows))
    assert 'inside zone: False' in carried and 'no DONE logged' in carried, carried
    # a GNSS stream with no fix at all
    nothing = _kpi_report('\n'.join(rows[:1] + ['3000000 controller :: stage -> done']))
    assert 'NO GNSS FIX before DONE' in nothing and ' 0.0 m' not in nothing, nothing
    # gnss_gga.csv also has 'gnss' in its name and no position: it must never be taken for the track
    gga_first = '@20260725_120000_gnss_gga.csv@uptime;altitude_m;elevation_m;quality;satellites;hdop_cd\n'
    assert 'inside zone: True' in _kpi_report(gga_first + landed)


def test_irq_summary_reads_polled_sensors():
    """
    A sensor with no INT wire records irq_runs 0 on every row BY DESIGN; that is not a missed interrupt.

    The summary counted TMS-7F's BMI323 as N missed of N, and with nothing interrupt-driven at all it
    still certified the capture 'IRQ clean'. Negative: a genuinely interrupt-driven stream that misses
    edges is still reported as missed.
    """
    def streams(*rows_by_name):
        lines = []
        for name, values in rows_by_name:
            lines.append('@20260725_120000_%s@uptime;ax;ay;az;irq_runs' % name)
            lines += ['@20260725_120000_%s@%d;0;0;1;%d' % (name, index * 1000, value)
                      for index, value in enumerate(values)]
        return flight_telemetry.parse('\n'.join(lines))[0]

    polled = flight_report.irq_health(streams(('imu_bmi323.csv', [0, 0, 0, 0])))
    assert 'missed' not in polled and 'never saw an edge' in polled and 'clean' not in polled, polled
    assert flight_report.irq_health({}) == 'no IRQ data'
    clean = flight_report.irq_health(streams(('imu_lsm6dso32.csv', [1, 1, 1])))
    assert clean.startswith('IRQ clean'), clean
    lossy = flight_report.irq_health(streams(('imu_lsm6dso32.csv', [1, 0, 1, 2]),
                                             ('imu_bmi323.csv', [0, 0, 0])))
    assert 'lsm6dso32 1 missed / 1 overrun of 4' in lossy and 'bmi323 never saw an edge' in lossy, lossy


def test_backstop_is_only_ever_the_adxl():
    """
    The +/-200 g backstop verdict is about the ADXL375; with none fitted there is no backstop to judge.

    `prefer` only breaks ties, so without an ADXL it handed back the first accel it found and a TMS-7F
    BMI323 was reported, and voted on, as the backstop. Negative: a real ADXL stream is still used.
    """
    import contextlib
    import io

    def envelope(text):
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            flight_kpi._accel_envelope(flight_telemetry.parse(text)[0])
        return printed.getvalue()

    imu = ['@20260725_120000_imu_bmi323.csv@uptime;ax;ay;az;gx;gy;gz',
           '@20260725_120000_imu_bmi323.csv@0;0;0;1;0;0;0', '@20260725_120000_imu_bmi323.csv@1000;0;0;2;0;0;0']
    accel = ['@20260725_120000_attitude.csv@uptime;ax;ay;az', '@20260725_120000_attitude.csv@0;0;0;1']
    assert 'adxl' not in envelope('\n'.join(accel + imu)), envelope('\n'.join(accel + imu))
    adxl = ['@20260725_120000_accel_adxl375.csv@uptime;ax;ay;az', '@20260725_120000_accel_adxl375.csv@0;0;0;3',
            '@20260725_120000_accel_adxl375.csv@1000;0;0;3', '@20260725_120000_accel_adxl375.csv@2000;0;0;3']
    assert 'peak |a| adxl' in envelope('\n'.join(imu + adxl))


def test_cc_exit_code_is_the_verdict():
    """
    tools/cc.py's exit code is the board's verdict, including the one INSIDE an `ok` reply.

    `verify` and `probe` always answer `ok` and carry the result in their JSON, so a failing pre-flight
    exited 0. Negative cases: a clean verify/probe, and the ordinary commands, still exit 0.
    """
    def ok(result):
        return {'status': 'ok', 'args': [json.dumps(result)]}

    assert cc._verdict('verify', 200, ok({'pass': True, 'ready': True})) == 0
    assert cc._verdict('verify', 200, ok({'pass': False, 'ready': True})) == 1
    assert cc._verdict('verify', 200, ok({'pass': True, 'ready': False})) == cc._NOT_READY
    assert cc._verdict('verify', 200, {'status': 'ok', 'args': ['{garbled']}) == 1
    assert cc._verdict('probe', 200, ok({'imu': None, 'baro': None})) == 0
    assert cc._verdict('probe', 200, ok({'imu': None, 'baro': 'not connected: ENODEV'})) == 1
    assert cc._verdict('inspect', 200, ok({'anything': 'at all'})) == 0
    assert cc._verdict('ping', 200, {'status': 'pong', 'args': []}) == 0
    assert cc._verdict('verify', 504, {'error': 'board did not answer in time'}) == 1
    assert cc._verdict('ping', 200, {'status': 'err', 'args': ['badcmd']}) == 1


def test_logger_join_on_a_still_to_moving_transition_is_lossless():
    """
    The nose logger's continuity check accepts ANY whole number of periods up to one decimation step.

    Still, the logger keeps one frame in _DECIMATE; moving, every frame. A save on the transition joins
    after any number of periods in between, and only the two ends were accepted, so a clean flight could
    report SAMPLES LOST. Negative: a join off the period grid, or past one decimation step, is still loss.
    """
    import contextlib
    import importlib.util
    import io
    path = os.path.join(_ROOT, 'src', 'logger', 'decode.py')
    spec = importlib.util.spec_from_file_location('logger_decode', path)
    decode = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(decode)

    def verdict(*gaps_ms):
        summaries, last = [], 0
        for segment, gap in enumerate((0,) + gaps_ms):
            first = last + gap
            summaries.append({'boot': 7, 'segment': segment, 'period': 20, 'first': first, 'last': first + 5000})
            last = first + 5000
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            decode._continuity(summaries)
        return printed.getvalue()

    assert 'LOSSLESS' in verdict(20, 1000, 340, 980), verdict(20, 1000, 340, 980)
    assert 'SAMPLES LOST' in verdict(20, 30), 'a join off the 20 ms grid is lost samples'
    assert 'SAMPLES LOST' in verdict(1020), 'more than one decimation step is lost samples'


def test_svg_renders_a_track():
    """flight_svg builds a plan from a capture, and tolerates a lat/lon length mismatch (§26.27)."""
    streams, _logs = flight_telemetry.parse(flight_synth_capture.generate())
    track = flight_svg._track(streams)
    assert len(track) > 10 and len(track[0]) == 2
    body = flight_svg._plan([streams], ['synthetic'], None, None, (0, 0, 400, 300))
    assert '<rect' in body and '<path' in body, body[:160]  # a framed plan with the track drawn
    # _plan takes a FLAT zone (lat,lon,lat,lon) -- unlike flight_kpi's nested pair; both shapes work
    zoned = flight_svg._plan([streams], ['synthetic'], (_ZONE[0][0], _ZONE[0][1]),
                             (_ZONE[0][0], _ZONE[0][1], _ZONE[1][0], _ZONE[1][1]), (0, 0, 400, 300))
    assert '<rect' in zoned
    document = flight_svg._svg(400, 300, 'test', body)
    assert document.startswith('<svg') and document.rstrip().endswith('</svg>')


def test_airspeed_calibration_recovers_a_known_density():
    """
    The calm-pass trim must recover the density that generated the data (tools/airspeed_calibrate).

    Synthesised from q = 0.5*rho*v^2 at a known rho, so the fit has a right answer to hit.
    """
    true_rho = 1.15
    knots = 1.0 / 0.514444
    pitot, gnss = [], []
    for i in range(400):
        speed = 15.0 + 1.0 * ((i % 40) - 20) / 20.0     # a gentle 14-16 m/s glide
        stamp = i * 20000                                 # 50 Hz, microseconds
        pressure = 0.5 * true_rho * speed * speed
        pitot.append({'uptime': str(stamp), 'dynamic_pressure': str(int(pressure * 100))})
        gnss.append({'uptime': str(stamp), 'speed_kn': '%.4f' % (speed * knots)})
    result = airspeed_calibrate.calibrate(pitot, gnss, min_speed=8.0, current=1.225)
    assert result['samples'] == len(gnss)
    assert abs(result['air_density'] - true_rho) < 0.01, result['air_density']
    assert result['error_after'] < result['error_before']  # the trim must IMPROVE the match
    # too little data -> reported as such, never a bogus fit
    assert airspeed_calibrate.calibrate(pitot[:2], gnss[:2], 8.0, 1.225)['samples'] < 5


def test_airspeed_calibration_from_an_assembled_capture():
    """
    The calibrator must eat the SAME artifact every other tool does -- an assembled capture (§27.17).

    It used to read only loose per-stream CSVs, so trimming air_density from a field recording meant
    hand-splitting the capture first. The capture path reuses the shared flight_telemetry parser, which
    yields FLOAT cells where the CSV path yields integer strings -- the reason calibrate() floats both.
    Also exercises plot(), which had no coverage at all.
    """
    true_rho = 1.15
    knots = 1.0 / 0.514444
    pitot_lines = ['@s_airspeed_sdp810.csv@uptime;dynamic_pressure;airspeed_cms;temperature']
    gnss_lines = ['@s_gnss.csv@uptime;lat;lon;speed_kn;course']
    for i in range(400):
        speed = 15.0 + 1.0 * ((i % 40) - 20) / 20.0
        stamp = i * 20000
        pressure = 0.5 * true_rho * speed * speed
        pitot_lines.append('@s_airspeed_sdp810.csv@%d;%d;%d;21.0'
                           % (stamp, int(pressure * 100), int(speed * 100)))
        gnss_lines.append('@s_gnss.csv@%d;25.5;-80.4;%.4f;90.0' % (stamp, speed * knots))
    capture = os.path.join(tempfile.mkdtemp(), 'calm_pass.txt')
    with open(capture, 'w') as handle:
        handle.write('\n'.join(pitot_lines + gnss_lines) + '\n')

    """
    A SPLICED line -- two records run together because the first lost its newline -- must be split, not
    silently mis-parsed.

    Measured on the real recorder path: 20 lines in 1,004,804 across 48 flights, roughly one flight in
    three. The damage is not the lost tail, it is the MISATTRIBUTION: the second record's fields land in
    the first record's columns, so the row keeps parsing and produces plausible-looking nonsense. That
    is where an airspeed of 1.4e12 cm/s and a heading error of 15330 deg came from -- both were simply
    the next stream's numbers read in the wrong place, and a tool downstream reported an L/D of 64 from
    exactly this class.
    """
    spliced_text = (
        '@20260101_010101_x_flight.csv@uptime;stage;fin_cap;heading_err\n'
        '@20260101_010101_x_airspeed_sdp810.csv@uptime;dynamic_pressure;airspeed_cms\n'
        '@20260101_010101_x_flight.csv@1000;3;45;120\n'
        '@20260101_010101_x_flight.csv@2000;3;45;1@20260101_010101_x_airspeed_sdp810.csv@2001;13000;1500\n'
    )
    spliced_streams, _unused_logs = flight_telemetry.parse(spliced_text)
    assert flight_telemetry.spliced_rows() == 1, flight_telemetry.spliced_rows()
    flight_stream = flight_telemetry.find_stream(spliced_streams, 'heading_err')
    _stamps, headings = flight_stream.column('heading_err')
    live = [v for v in headings if v is not None]
    assert max(live) <= 180, 'the airspeed record leaked into heading_err: %r' % live
    air = flight_telemetry.find_stream(spliced_streams, 'dynamic_pressure')
    _stamps, speeds = air.column('airspeed_cms')
    assert 1500 in [v for v in speeds if v is not None], 'the second record was lost, not re-queued'

    pitot_rows, gnss_rows = airspeed_calibrate._read_capture(capture)
    assert len(pitot_rows) == 400 and len(gnss_rows) == 400

    """
    A TRUNCATED telemetry line must be skipped, not crash the calibration. Rows were rebuilt with a
    bare zip(), which stops at the shorter sequence -- so a short row produced a dict missing its
    trailing keys and the caller's float(row['dynamic_pressure']) raised KeyError, losing the whole
    calibration to one bad line. Captures really do contain them: q55/e16_full.txt carries a 3-cell
    row where 4 are expected.
    """
    with open(capture) as handle:
        body = handle.read().rstrip('\n').split('\n')
    body.insert(3, '@s_airspeed_sdp810.csv@240000;13000')   # 2 cells where 4 are expected
    truncated = os.path.join(tempfile.mkdtemp(), 'truncated.txt')
    with open(truncated, 'w') as handle:
        handle.write('\n'.join(body) + '\n')
    short_pitot, short_gnss = airspeed_calibrate._read_capture(truncated)
    assert len(short_pitot) == 400, len(short_pitot)   # the bad row skipped, the good ones kept
    assert airspeed_calibrate.calibrate(short_pitot, short_gnss, 8.0, 1.225)['samples'] > 0
    result = airspeed_calibrate.calibrate(pitot_rows, gnss_rows, min_speed=8.0, current=1.225)
    assert abs(result['air_density'] - true_rho) < 0.01, result['air_density']

    # plot() writes a well-formed, self-contained SVG (no plotly at the field)
    svg = capture.replace('.txt', '.svg')
    airspeed_calibrate.plot(result, svg, current=1.225)
    with open(svg) as handle:
        document = handle.read()
    assert document.startswith('<svg') and document.rstrip().endswith('</svg>')
    # self-contained: the w3.org xmlns is the XML namespace (not a fetch), but nothing may be LOADED
    assert 'href="http' not in document and 'src="http' not in document, 'the field SVG fetches a remote asset'

    # NEGATIVE: a capture with no pitot stream reports empty rather than crashing
    bare = os.path.join(os.path.dirname(capture), 'bare.txt')
    with open(bare, 'w') as handle:
        handle.write('\n'.join(gnss_lines) + '\n')
    empty_pitot, some_gnss = airspeed_calibrate._read_capture(bare)
    assert empty_pitot == [] and len(some_gnss) == 400


def test_ticks_us_wraparound_is_unwrapped():
    """
    MicroPython's ticks_us wraps at 2**30 us (~17.9 min of uptime) and the recorder stamps rows raw.

    A board powered up, set up, waiting on a GNSS fix and then flown crosses that boundary mid-capture,
    after which stamps jump BACKWARDS by a full period and every downstream duration goes negative --
    observed twice on the bench as a flight reported -1015.6 s long at 7.3e12 deg/s of fin travel.
    The shared parser unwraps it, so captures already on disk are repaired too.
    """
    period = flight_telemetry._TICKS_PERIOD
    lines = ['@20260726_090000_1_flight.csv@uptime;fin_cap']
    stamps = [period - 30000, period - 20000, period - 10000, 10000, 20000, 30000]  # wraps mid-stream
    for stamp in stamps:
        lines.append('@20260726_090000_1_flight.csv@%d;20' % stamp)
    streams, _logs = flight_telemetry.parse('\n'.join(lines) + '\n')
    times = [row[0] for row in streams['flight.csv'].rows]
    assert times == sorted(times), times                 # monotonic after unwrapping
    assert times[-1] - times[0] == 60000, times          # 60 ms total, not a negative period-sized jump

    # a capture that never wraps is untouched
    plain = ['@20260726_090000_1_flight.csv@uptime;fin_cap'] + \
            ['@20260726_090000_1_flight.csv@%d;20' % (i * 1000) for i in range(5)]
    streams, _logs = flight_telemetry.parse('\n'.join(plain) + '\n')
    assert [row[0] for row in streams['flight.csv'].rows] == [0, 1000, 2000, 3000, 4000]


def test_parser_edge_cases():
    """Malformed rows degrade instead of crashing (§26.28, §26.32) -- captures do get truncated."""
    streams, _logs = flight_telemetry.parse(
        '@20260725_120000_x.csv@uptime;v\n'
        '@20260725_120000_x.csv@1000000;5\n'
        '@20260725_120000_x.csv@badtime;6\n'      # bad uptime -> row dropped, not a crash
        '@20260725_120000_x.csv@2000000;junk\n')  # bad value -> nan, so arithmetic downstream survives
    values = streams['x.csv'].column('v')[1]
    assert values[0] == 5.0
    assert values[1] != values[1]  # nan is the only value not equal to itself
    assert flight_telemetry.parse('')[0] == {}  # an empty capture is empty, not an exception



def test_session_tail_variants_all_key_the_same():
    """
    Every session-tag ERA parses to the same stream names.

    The tag after the date/time has changed shape three times -- absent (oldest captures), a random
    disambiguator, and now an operator label CC sets via `recorder.session` -- and all three are the
    same SHAPE as a stream whose own name starts with a word ('imu_bno055.csv'). Getting this wrong is
    silent: the streams still parse, they are just keyed under names no tool looks for, so every panel
    goes empty on a capture that is perfectly good. Each era below must land on identical keys.
    """
    def keys(tag):
        return set(flight_telemetry.parse(
            '@20260726_090000_%shealth.csv@uptime;mem_free\n' % tag +
            '@20260726_090000_%shealth.csv@1000000;90000\n' % tag +
            '@20260726_090000_%simu_bno055.csv@uptime;roll\n' % tag +
            '@20260726_090000_%simu_bno055.csv@1000000;3\n' % tag)[0])

    expected = {'health.csv', 'imu_bno055.csv'}
    assert keys('') == expected, 'legacy tag-less capture'
    assert keys('989510_') == expected, 'the board random disambiguator'
    assert keys('catapult-run3_') == expected, 'an operator label from recorder.session'

    # a lone stream still gets its numeric tag stripped -- digits can only ever be a tag
    assert set(flight_telemetry.parse('@20260726_090000_1_flight.csv@uptime;v\n')[0]) == {'flight.csv'}

    """
    The trap that makes this subtle: per-servo streams SHARE a leading word by construction, and a
    servo-only capture is exactly what a fin bench run produces. Stripping 'servo_' as if it were a
    tag would break the fins.csv synthesis every fin tool depends on.
    """
    streams, _logs = flight_telemetry.parse(
        '@20260726_090000_servo_yaw.csv@uptime;angle\n'
        '@20260726_090000_servo_yaw.csv@1000000;95\n'
        '@20260726_090000_servo_eleron_left.csv@uptime;angle\n'
        '@20260726_090000_servo_eleron_left.csv@1000000;85\n')
    assert 'servo_yaw.csv' in streams, sorted(streams)
    assert streams['fins.csv'].fields == ['eleron_left', 'yaw'], streams['fins.csv'].fields


def test_a_spliced_capture_is_reported_not_swallowed():
    """
    Two boots appended into one file must be VISIBLE, not silently half-eaten.

    When two recorder sessions land on the same prefix the Luckfox appends, so the file carries a
    second `uptime;...` header partway down. The parser used to drop that row on the floor (it fails
    the uptime parse), leaving a capture that looks like one long flight whose clock restarts midway --
    every duration and rate then spans two flights. Nothing here can repair it (the rows carry no boot
    identity), so the requirement is simply that it is detected and the rows survive.
    """
    streams, _logs = flight_telemetry.parse(
        '@20260726_090000_1_x.csv@uptime;v\n'
        '@20260726_090000_1_x.csv@1000000;5\n'
        '@20260726_090000_1_x.csv@uptime;v\n'      # <- second boot appended into the same file
        '@20260726_090000_1_x.csv@1000;7\n')
    assert flight_telemetry.spliced(streams) == ['x.csv']
    assert len(streams['x.csv'].rows) == 2, 'the data rows must survive the detection'

    clean, _logs = flight_telemetry.parse('@20260726_090000_1_x.csv@uptime;v\n'
                                          '@20260726_090000_1_x.csv@1000000;5\n')
    assert flight_telemetry.spliced(clean) == [], 'a single session must not be flagged'


def test_calibration_refuses_a_simulated_capture():
    """
    air_density must never be fitted from a HITL capture, and the tool must SAY so.

    In the sim the pitot pressure and the GNSS ground speed are both derived from one body state, so
    fitting one against the other measures the model's constants rather than the atmosphere. Without
    this guard the calibrator still printed a plausible density and an `apply:` line offering to write
    it onto real hardware -- a confident recommendation from data that cannot support one, which is
    the same failure mode as reporting a glide ratio from a nan.
    """
    lines = ['@s_airspeed_sdp810.csv@uptime;dynamic_pressure;airspeed_cms;temperature',
             '@s_gnss.csv@uptime;lat;lon;speed_kn;course',
             '@s_hitl_clock.csv@uptime;drift_ms']      # <- the sim's own clock stream: the marker
    for i in range(20):
        stamp = i * 20000
        lines.append('@s_airspeed_sdp810.csv@%d;13000;1500;21.0' % stamp)
        lines.append('@s_gnss.csv@%d;25.5;-80.4;29.2;90.0' % stamp)
        lines.append('@s_hitl_clock.csv@%d;0' % stamp)
    capture = os.path.join(tempfile.mkdtemp(), 'hitl_run.txt')
    with open(capture, 'w') as handle:
        handle.write('\n'.join(lines) + '\n')

    streams, _logs = flight_telemetry.parse(open(capture).read())
    assert flight_telemetry.simulated(streams) is True
    try:
        airspeed_calibrate._read_capture(capture)
        raise AssertionError('a HITL capture must be refused, not calibrated')
    except SystemExit as exit_code:
        assert exit_code.code == 2, exit_code.code

    # NEGATIVE: the same data WITHOUT the sim clock is a real capture and must still calibrate
    real = os.path.join(tempfile.mkdtemp(), 'real_pass.txt')
    with open(real, 'w') as handle:
        handle.write('\n'.join(row for row in lines if 'hitl_clock' not in row) + '\n')
    assert flight_telemetry.simulated(flight_telemetry.parse(open(real).read())[0]) is False
    assert len(airspeed_calibrate._read_capture(real)[0]) == 20


def test_recorder_space_gate_reads_a_wrapped_df():
    """
    The preflight recorder gate must read busybox's WRAPPED df, and must fail a full disk.

    A full recorder does not fail loudly: it creates every stream of a session and writes no rows, so a
    flight runs perfectly and the capture is 0 bytes. The first version of this check took a fixed field
    index and read '1%' as the free space, so it passed a disk with nothing left on it.
    """
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'preflight', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..',
                                  'tools', 'preflight.py'))
    preflight = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preflight)

    wrapped = ('Filesystem           1K-blocks      Used Available Use% Mounted on\n'
               '/dev/block/by-name/userdata\r\n'
               '                      56451596    362608  54090836   1% /userdata\n')
    assert preflight.free_kb(wrapped) == 54090836, 'the wrapped device-name line must still parse'

    full = ('Filesystem           1K-blocks      Used Available Use% Mounted on\n'
            '/dev/block/by-name/userdata\n'
            '                      56451596  56451596         0 100% /userdata\n')
    assert preflight.free_kb(full) == 0, 'a full disk reads as zero free, not as the percentage'
    assert preflight.free_kb(full) < preflight._RECORDER_FREE_KB, 'a full disk must trip the gate'

    single = ('Filesystem     1K-blocks    Used Available Use% Mounted on\n'
              '/dev/sda1       56451596  362608  54090836   1% /userdata\n')
    assert preflight.free_kb(single) == 54090836, 'an UNwrapped line must parse too'

    # NEGATIVE: output with no percentage column at all yields None (treated as "no recorder"), never 0,
    # because a 0 would fail the gate on every host that has no recorder attached
    assert preflight.free_kb('adb: no devices/emulators found\n') is None
    assert preflight.free_kb('') is None


def test_every_provided_quantity_has_a_consumer():
    """
    A quantity a device PROVIDES but nobody READS is a provider left behind by a refactor -- fused,
    recorded and delivered to no one.

    This check used to live in the board suite against a HARDCODED set of consumed names, which meant
    adding a provider failed the test until someone edited the set, even when a real consumer existed.
    Worse, the board now runs .mpy, so it has no source to scan and the set could never become
    derived there. Here on the host the sources are present, so the consumed set is DERIVED from the
    actual `Databoard.parameter('x')` call sites and cannot drift.
    """
    sys.path.insert(0, os.path.join(_ROOT, 'src', 'glider'))
    import config_default
    import sources

    consumed = set()
    for _relative, _name, path in sources.modules(sources.GLIDER_DIRS):
        with open(path) as handle:
            for match in re.finditer(r"parameter\(\s*'([a-z_0-9]+)'", handle.read()):
                consumed.add(match.group(1))
    assert consumed, 'found no Databoard.parameter() call sites -- the scan itself is broken'

    provided = set()
    for device in config_default.default()['sensors'] + config_default.default()['components']:
        provided.update((device.get('provides') or {}).keys())

    """
    Not every provided quantity is READ by control, and that is legitimate -- some exist to be
    recorded and shown. Naming them explicitly is the point: the old hardcoded `consumed` set listed
    these alongside the real control inputs, which quietly asserted that the INA226's power figures
    were steering something. They are not. This list states intent, and it is far smaller and more
    meaningful to maintain than a mirror of every control input.
    """
    operator_only = {
        'voltage', 'current', 'power',     # INA226 -> energy budget, dashboard, telemetry
        'pressure', 'temperature',         # baro raw -> telemetry and the operator panel
        'dynamic_pressure',                # pitot raw Pa -> flight_report + airspeed_calibrate (host)
    }
    orphans = provided - consumed - operator_only
    assert not orphans, 'quantities provided but consumed by nobody: %s' % sorted(orphans)
    # and the reverse: an operator_only entry that HAS become a control input is stale bookkeeping
    stale = operator_only & consumed
    assert not stale, 'listed operator-only but now read by control: %s' % sorted(stale)


test_board_shape_is_readable()
test_kpi_on_the_synthetic_flight()
test_kpi_survives_a_partial_capture()
test_touchdown_is_the_last_fix_before_done()
test_irq_summary_reads_polled_sensors()
test_backstop_is_only_ever_the_adxl()
test_cc_exit_code_is_the_verdict()
test_logger_join_on_a_still_to_moving_transition_is_lossless()
test_svg_renders_a_track()
test_airspeed_calibration_recovers_a_known_density()
test_airspeed_calibration_from_an_assembled_capture()
test_ticks_us_wraparound_is_unwrapped()
test_parser_edge_cases()
test_session_tail_variants_all_key_the_same()
test_a_spliced_capture_is_reported_not_swallowed()
test_calibration_refuses_a_simulated_capture()
test_recorder_space_gate_reads_a_wrapped_df()
test_every_provided_quantity_has_a_consumer()
print('ok: tools -- board-shape fins rebuild, kpi golden + partial captures, touchdown at DONE, '
      'polled-IRQ summary, adxl-only backstop, cc.py verdict exit codes, logger join continuity, svg render, '
      'airspeed calibration fit, parser edge cases, session-tag eras, '
      'spliced-capture detection, sim-capture refusal, provider/consumer closure')
