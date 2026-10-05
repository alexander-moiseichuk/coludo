"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the ANALYSIS TOOLS (tools/flight_kpi, flight_svg, airspeed_calibrate, the
board-shape handling in flight_telemetry, and the recorder-dump readers recorder_flight,
assemble_capture and flight_pull.sh on wrapped and unwrapped dumps -- every kind of link damage, across a
ticks_us wrap, and under a label). Stdlib only -- plotly rendering runs only where plotly is importable
(flight_report on a damaged byte); everywhere, the report runs with plotly and the figure stubbed, and
what it would draw is checked. Every folder a test writes lives in one temporary directory, gone at exit.

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
import assemble_capture  # noqa: E402
import cc  # noqa: E402
import flight_campaign  # noqa: E402
import flight_kpi  # noqa: E402
import flight_metrics  # noqa: E402
import flight_report  # noqa: E402
import flight_svg  # noqa: E402
import flight_synth_capture  # noqa: E402
import flight_telemetry  # noqa: E402
import gen_schema  # noqa: E402
import glide_polar  # noqa: E402
import hitl_compare  # noqa: E402
import recorder_flight  # noqa: E402
import recorder_wire  # noqa: E402

_ZONE = ((25.514944, -80.392972), (25.514583, -80.391111))  # the HPRC strip (TL, BR)
_SCRATCH: tempfile.TemporaryDirectory = tempfile.TemporaryDirectory()  # every folder a test writes; gone at exit


def _folder() -> str:
    """A fresh folder inside _SCRATCH: a run leaves nothing behind in /tmp, whatever fails."""
    return tempfile.mkdtemp(dir=_SCRATCH.name)


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
    # an unfitted alternative rides in the SECOND map: `probe laser_agl` on an L1X board is no failure
    unfitted = json.dumps({'laser_agl': 'vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:0 0x29'})
    assert cc._verdict('probe', 200, {'status': 'ok', 'args': [json.dumps({'laser_agl': None}), unfitted]}) == 0
    down = json.dumps({'laser_agl': 'vl53l4cx -- not connected: setup failed (absent / miswired?)'})
    assert cc._verdict('probe', 200, {'status': 'ok', 'args': [down]}) == 1  # nothing answers the socket
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
    capture = os.path.join(_folder(), 'calm_pass.txt')
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
    truncated = os.path.join(_folder(), 'truncated.txt')
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

    """
    The current era: the board names its session by its NVS boot id ('000123'), with no date at all, or
    by the `recorder.session` label verbatim. Both must key the same, wrapped or not -- and the shared
    session.csv, which carries no session prefix, must not stop a label from being recognised as one.
    """
    def boot_keys(session, wrap):
        rows = [('%s_health.csv' % session, 'uptime;mem_free'), ('%s_health.csv' % session, '1000000;90000'),
                ('%s_imu_bno055.csv' % session, 'uptime;roll'), ('%s_imu_bno055.csv' % session, '1000000;3'),
                ('session.csv', '900000;123;%s;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;' % session)]
        lines = [recorder_wire.wrap(row, routing).rstrip('\n') if wrap else '@%s@%s' % (routing, row)
                 for routing, row in rows]
        return set(flight_telemetry.parse('\n'.join(lines))[0]) - {'session.csv'}

    assert boot_keys('000123', True) == expected, 'a boot-id session'
    assert boot_keys('000123', False) == expected, 'a boot-id session, unwrapped'
    assert boot_keys('1234567', True) == expected, 'a boot id past 999999 grows a digit'
    assert boot_keys('taster', True) == expected, 'a recorder.session label next to session.csv'
    assert boot_keys('taster', False) == expected, '... unwrapped, as the CC tee keeps it'
    # a spliced line under a boot-id prefix is still split in an unwrapped capture
    flight_telemetry.parse('@000123_flight.csv@uptime;stage;fin_cap\n'
                           '@000123_flight.csv@2000;3;4@000123_airspeed_sdp810.csv@2001;13000;1500\n')
    assert flight_telemetry.spliced_rows() == 1


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
    capture = os.path.join(_folder(), 'hitl_run.txt')
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
    real = os.path.join(_folder(), 'real_pass.txt')
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


def _gnss_mismatches(path: str, name: str) -> list:
    """
    Where recorder_flight's GNSS field lists differ from the Telemetry(...) declarations in `path`.

    Read with gen_schema's ast scan -- the one doc/telemetry.md is generated by -- so the firmware is
    parsed, never imported.

    Args:
        path - a gnss.py to scan.
        name - the receiver's device name, which a per-device stream's file is named after.

    Returns:
        [(stream, recorder_flight's fields, the declared fields or None when not declared)], empty when
        the GGA and the sky streams both agree.
    """
    declared = {pattern: tuple(fields or ()) for _stream, fields, pattern in gen_schema._streams_in(path) if pattern}
    mismatches = []
    for suffix in ('gga', 'sky'):
        stream = '%s_%s' % (name, suffix)
        fields = declared.get('<name>_%s.csv' % suffix)
        if recorder_flight._FIELDS.get(stream) != fields:
            mismatches.append((stream, recorder_flight._FIELDS.get(stream), fields))
    return mismatches


def test_recorder_flight_knows_the_gnss_streams_as_declared():
    """
    recorder_flight._FIELDS is a hand-kept copy of the firmware's declarations, read when a stream lost its
    header row (it goes out once, so on a damaged link it usually is lost). Nothing tied the GNSS pair to
    gnss.py: a field reordered on either side would label a real flight's columns wrongly, silently.

    Checked under the device name the default config gives the receiver; a doctored copy of gnss.py --
    two fields swapped, or the sky stream renamed -- must be caught, so the check cannot pass vacuously.
    """
    sys.path.insert(0, os.path.join(_ROOT, 'src', 'glider'))
    import config_default

    names = [sensor['name'] for sensor in config_default.default()['sensors']
             if sensor.get('driver') in ('atgm336h', 'neo6mv2')]
    assert names == ['gnss'], names
    source = os.path.join(_ROOT, 'src', 'glider', 'gnss.py')
    assert _gnss_mismatches(source, names[0]) == [], _gnss_mismatches(source, names[0])

    with open(source) as handle:
        text = handle.read()
    for original, doctored, stream in (("'cn0_1', 'cn0_2'", "'cn0_2', 'cn0_1'", 'gnss_sky'),
                                       ("'quality', 'satellites'", "'satellites', 'quality'", 'gnss_gga'),
                                       ("'%s_sky.csv'", "'%s_sats.csv'", 'gnss_sky')):
        assert text.count(original) == 1, original  # the doctoring below must hit the declaration itself
        path = os.path.join(_folder(), 'gnss.py')
        with open(path, 'w') as handle:
            handle.write(text.replace(original, doctored))
        assert [mismatch[0] for mismatch in _gnss_mismatches(path, names[0])] == [stream], doctored


def _body(payload: str, routing: str) -> str:
    """A wrapped row as the Luckfox file named `routing` holds it: the wire line without its routing."""
    return recorder_wire.wrap(payload, routing).rstrip('\n')[len(routing) + 2:]


def _write_dump(directory: str, files: dict) -> None:
    """Write a recorder dump: {file name: [line, ...]}."""
    for name, lines in files.items():
        with open(os.path.join(directory, name), 'w') as handle:
            handle.write(''.join(line + '\n' for line in lines))


def _imu_rows() -> list:
    """An LSM6DSO32 boot at 100 Hz, 1..3 s of uptime, boosting at 5 g from 1.5 s to 2.5 s."""
    return ['%d;0.0;0.0;%.1f;0;0;0;1' % (uptime, 5.0 if 1500000 <= uptime < 2500000 else 1.0)
            for uptime in range(1000000, 3000001, 10000)]


def _baro_rows() -> list:
    """A BMP280 at 10 Hz over the same 1..3 s (21 rows); its header is lost, as on the real dumps."""
    return ['%d;%.2f;25.0;101300;%.2f' % (uptime, uptime / 1e6, uptime / 1e6)
            for uptime in range(1000000, 3000001, 100000)]


def _cut(recordings: str, session: str, before: str = '1') -> tuple:
    """
    Run recorder_flight over a dump into a fresh folder.

    Args:
        recordings - the dump directory.
        session - the session to cut.
        before - the seconds kept ahead of ignition (--before).

    Returns:
        (what it printed, the folder).
    """
    import contextlib
    import io
    out = _folder()
    printed = io.StringIO()
    saved, sys.argv = sys.argv, ['recorder_flight.py', recordings, '--session', session, '-o', out, '--before', before]
    try:
        with contextlib.redirect_stdout(printed):
            recorder_flight.main()
    finally:
        sys.argv = saved
    return printed.getvalue(), out


def _read(path: str) -> list:
    """A file's lines, newlines stripped."""
    with open(path) as handle:
        return handle.read().splitlines()


def test_recorder_flight_salvages_a_wrapped_dump():
    """
    A wrapped dump is read by its checks: every row the link damaged is salvaged or rejected, never guessed.

    The 2026-10-03 dumps lost ~4 % of rows: corrupted values, file names mangled into thousands of junk
    files, rows that lost their leading '@' and fell into recorder.log, and splices -- a row that lost its
    tail and newline, with the next record running on. Each shape is planted here, and each must land
    where its CRC says, counted per stream.
    """
    session = '000123'
    imu, baro = _imu_rows(), _baro_rows()
    imu_routing, baro_routing = '000123_imu_lsm6dso32.csv', '000123_baro_bmp280.csv'
    stranded = {row.split(';')[0]: _body(row, baro_routing) for row in baro if row.split(';')[0] in
                ('1500000', '1600000', '1700000', '1800000', '1900000')}
    imu_file = [_body('uptime;ax;ay;az;gx;gy;gz;irq_runs', imu_routing)]
    for row in imu:
        body = _body(row, imu_routing)
        if row.startswith('1800000;'):  # a splice: this row lost its tail, the baro row ran on
            body = body[:30] + '@' + baro_routing + '@' + stranded['1800000']
        imu_file.append(body)
    imu_file.append('2990000;0.0;0.0;1.0;0;0;0;1')  # NEGATIVE: an unwrapped row in a wrapped file
    baro_file = []
    for row in baro:
        if row.split(';')[0] in stranded:
            continue
        body = _body(row, baro_routing)
        baro_file.append(body.replace(';1.20;', ';1.29;') if row.startswith('1200000;') else body)  # corrupted
    status = recorder_wire.wrap("900000 recorder :: {'session': '000123', 'lines': 9}").rstrip('\n')
    other = '000122_imu_lsm6dso32.csv'
    clock = ['uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon',
             '884031;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;',
             '5000;122;000122;2026-10-02T10:00:00Z;-240;taster;dev;3f2a91;cc-auto;;']
    recordings = _folder()
    _write_dump(recordings, {
        imu_routing: imu_file,
        baro_routing: baro_file,
        'xx7_baro.cs': [stranded['1500000']],                    # a junk name: the routing was mangled
        '000123_baro_bmXX280.csv': [stranded['1600000']],        # ... one that keeps the session prefix
        'recorder.log': [status,
                         baro_routing + '@' + stranded['1700000'],   # a lost leading '@'
                         '3_baro_bmp280.csv@' + stranded['1900000'],  # ... and part of the name with it
                         recorder_wire.wrap('1500000 sequencer :: stage -> boosting').rstrip('\n')],
        '000123_attitude.csv': [_body('uptime;heading_cd;roll_cd', '000123_attitude.csv'),
                                _body('1600000;100;5', '000123_attitude.csv')],  # not a stream it knows of
        other: [_body(row, other) for row in imu[:5]],             # another session: left alone
        'session.csv': [_body(row, 'session.csv') for row in clock],
    })
    printed, out = _cut(recordings, session)
    assert 'ignition at uptime 1.500000 s' in printed, printed
    assert re.search(r'imu_lsm6dso32\s+201 good,\s+0 salvaged in,\s+2 rejected', printed), printed
    assert re.search(r'baro_bmp280\s+15 good,\s+5 salvaged in,\s+1 rejected', printed), printed
    assert 'dated by cc-auto at uptime 884031 us (raw ticks): 2026-10-03T14:21:07Z' in printed, printed
    assert '2026-10-02' not in printed, 'another boot\'s clock set was printed'

    recorded = _read(os.path.join(out, 'recorder', 'baro_bmp280.csv'))
    assert len(recorded) == 20, len(recorded)  # 21 rows, less the corrupted one
    assert all(recorder_wire.verify(baro_routing, line) for line in recorded), 'recorder/ must keep proven rows'
    flown = _read(os.path.join(out, 'flight', 'baro_bmp280.csv'))
    assert flown[0] == 't_s;altitude;temperature;pressure;elevation;uptime_us', flown[0]  # the _FIELDS fallback
    assert [line.split(';')[-1] for line in flown[1:]] == [str(row.split(';')[0]) for row in baro
                                                           if not row.startswith('1200000;')]
    assert len(_read(os.path.join(out, 'flight', 'imu_lsm6dso32.csv'))) == 1 + 200  # less the spliced row
    assert _read(os.path.join(out, 'flight', 'attitude.csv'))[0] == 't_s;heading_cd;roll_cd;uptime_us'
    assert not any('000122' in name for name in os.listdir(os.path.join(out, 'recorder')))
    assert status in _read(os.path.join(out, 'recorder', 'board.log'))


def test_recorder_flight_reads_an_unwrapped_dump_as_before():
    """
    A dump from before the wrapper is cut exactly as it always was: by its time sequence, known streams only.

    The 2026-10-03 cuts in launches/ must regenerate byte for byte, so this path must not change at all --
    board.log included, which reads recorder.log as those cuts did: a byte past ASCII is U+FFFD, never
    the byte itself (a wrapped session's board.log keeps the bytes).
    """
    session = '20000101_000006_898573'
    imu, baro = _imu_rows(), _baro_rows()
    baro_file = baro[:5] + ['999999999;9.99;25.0;101300;9.99'] + baro[5:]  # a corrupted uptime: out of sequence
    recordings = _folder()
    _write_dump(recordings, {
        '%s_imu_lsm6dso32.csv' % session: ['uptime;ax;ay;az;gx;gy;gz;irq_runs'] + imu,
        '%s_baro_bmp280.csv' % session: baro_file,
        '%s_attitude.csv' % session: ['uptime;heading_cd;roll_cd', '1600000;100;5'],  # not a known stream
        '%s_baro_bmXX280.csv' % session: [_body(baro[1], 'x')],  # a wrapper-shaped junk row decides nothing
    })
    status = ("900000 recorder :: {'session': '%s'}\n" % session).encode()
    with open(os.path.join(recordings, 'recorder.log'), 'wb') as handle:
        handle.write(status + '1500000 sequencer :: zündung\n'.encode('utf-8'))
    printed, out = _cut(recordings, session)
    assert 'ignition at uptime 1.500000 s' in printed and 'rows checked' not in printed, printed
    with open(os.path.join(out, 'recorder', 'board.log'), 'rb') as handle:
        assert handle.read() == status + '1500000 sequencer :: z\ufffd\ufffdndung\n'.encode('utf-8')
    assert _read(os.path.join(out, 'recorder', 'baro_bmp280.csv')) == baro  # the corrupted uptime dropped
    assert len(_read(os.path.join(out, 'flight', 'imu_lsm6dso32.csv'))) == 1 + len(imu)
    assert not os.path.exists(os.path.join(out, 'flight', 'attitude.csv')), 'only known streams on an old dump'


def test_assemble_capture_keeps_a_wrapped_capture_strict():
    """
    An assembled capture of a wrapped dump stays checkable, and the lines the tools ADD are wrapped too.

    assemble_capture re-tags each row with its file name, which IS the row's routing, so a wrapped row
    becomes its original wire line again. Its own stage marks and hitl_collect's build note are host-built
    log lines: unwrapped, flight_telemetry would reject them as damage in a wrapped capture. On an old
    dump everything stays unwrapped, as before.
    """
    note = '0 capture :: build 2026.10.04 3f2a91 flash'
    rows = {'accel.csv': ['uptime;ax;ay;az', '1000000;0.0;0.0;1.0', '1010000;0.1;0.0;1.0'],
            'sequencer.csv': ['uptime;stage;reason', '1500000;boosting;launch', '1600000;gliding;apogee']}
    for wrap in (True, False):
        session = '000123' if wrap else '20000101_000006_898573'
        directory = _folder()
        files = {}
        for name, lines in rows.items():
            routing = '%s_%s' % (session, name)
            files[routing] = [_body(line, routing) if wrap else line for line in lines]
        if wrap:
            files['%s_sequencer.csv' % session][2] = files['%s_sequencer.csv' % session][2].replace('apogee', 'apogeX')
            files['%s_accel.csv' % session].pop(2)
            files['%s_acXel.csv' % session] = [_body(rows['accel.csv'][2], '%s_accel.csv' % session)]  # junk name
        _write_dump(directory, files)
        capture = os.path.join(directory, 'capture.txt')
        assemble_capture.assemble(session, directory, capture, note)
        lines = _read(capture)
        assert lines[-1] == (recorder_wire.wrap(note).rstrip('\n') if wrap else note), lines[-1]
        assert airspeed_calibrate._is_capture(capture)
        streams, logs = flight_telemetry.parse('\n'.join(lines))
        counts = flight_telemetry.line_counts()
        texts = [text for _stamp, text in logs]
        assert len(streams['accel.csv'].rows) == 2 and note in texts, (sorted(streams), texts)
        assert '1500000 controller :: stage -> boosting' in texts, texts
        if wrap:
            # the damaged sequencer row is rejected and makes no stage mark; the junk-named row is salvaged
            assert counts['salvaged'] == 1 and counts['rejected'] == 1 and counts['legacy'] == 0, counts
            assert not any('gliding' in text for text in texts), texts
        else:
            assert counts['legacy'] == len(lines) and '1600000 controller :: stage -> gliding' in texts

    # a capture whose first line is a wrapped LOG line is still recognised as a capture
    first_log = os.path.join(_folder(), 'capture.txt')
    with open(first_log, 'w') as handle:
        handle.write(recorder_wire.wrap('100 main :: boot') + recorder_wire.wrap('1;2', '000123_x.csv'))
    assert airspeed_calibrate._is_capture(first_log)
    # NEGATIVE: an unwrapped log line first is still not a capture (unchanged)
    with open(first_log, 'w') as handle:
        handle.write('100 main :: boot\n@000123_x.csv@1;2\n')
    assert not airspeed_calibrate._is_capture(first_log)



def _log(payload: str) -> str:
    """A board log line as recorder.log holds it: wrapped, with no routing."""
    return recorder_wire.wrap(payload).rstrip('\n')


def test_recorder_flight_error_matrix():
    """
    Every kind of link damage through recorder_flight on a Luckfox dump: what is proven is kept and placed
    by time, what is not is rejected and counted -- never guessed, and never another session's.

    Damage per row: a corrupted value, uptime or CLOSE, a lost tail or head, an unwrapped row (rejected);
    a corrupted routing (a junk file, with or without the session prefix), a lost leading '@' and a lost
    second '@' (both land in recorder.log), two lines merged into one, and a splice (salvaged). Plus a
    header (U = 0), several session.csv rows for one boot, a leftover name from another session, a
    non-ASCII payload, and a byte the link damaged in recorder.log -- which must come out as it went in.
    Every line that proves nothing is counted: under '(no stream)' when it is this session's (a junk file
    with its prefix, its part of recorder.log), apart when it may be another session's. A stream whose
    every row was stranded still comes out, and a 'boot' row with no utc dates nothing.
    """
    session, other = '000123', '000122'
    imu_routing, baro_routing = '000123_imu_lsm6dso32.csv', '000123_baro_bmp280.csv'
    imu, baro = _imu_rows(), _baro_rows()
    bodies = {row.split(';')[0]: _body(row, baro_routing) for row in baro}
    stranded = ('1500000', '1600000', '1700000', '1800000', '1900000', '2000000')
    imu_file = [_body('uptime;ax;ay;az;gx;gy;gz;irq_runs', imu_routing)]
    for row in imu:
        body = _body(row, imu_routing)
        damage = {
            '1100000': body.replace(';0.0;0.0;1.0;', ';0.0;0.0;1.1;'),          # a corrupted value
            '1110000': body.replace('1110000', '1110001'),                        # a corrupted uptime
            '1120000': body[:-2] + ('0' if body[-2] != '0' else '1') + '>',     # a corrupted CLOSE
            '1130000': body[:-6],                                                 # a lost tail
            '1140000': body[11:],                                                 # a lost head
            '1150000': body + '@' + baro_routing + '@' + bodies['1500000'],     # two lines merged
            '1800000': body[:30] + '@' + baro_routing + '@' + bodies['1800000'],  # a splice
        }
        imu_file.append(damage.get(row.split(';')[0], body))
    imu_file.append('2990000;0.0;0.0;1.0;0;0;0;1')  # NEGATIVE: an unwrapped row in a wrapped file
    baro_file = [_body('uptime;altitude;temperature;pressure;elevation', baro_routing)]
    baro_file += [body for uptime, body in bodies.items() if uptime not in stranded]
    sequencer_routing = '000123_sequencer.csv'
    header = 'uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon'
    clock = [header, '100000;123;000123;;;tästér;dev;3f2a91;boot;;',
             '884031;123;000123;2026-10-03T14:21:07Z;-240;tästér;dev;3f2a91;cc-auto;;',
             '1400000;123;000123;2026-10-03T14:21:08Z;-240;tästér;dev;3f2a91;dashboard;;', header,
             '5000;122;000122;2026-10-02T10:00:00Z;-240;taster;dev;3f2a91;cc-auto;;']
    anchor = '2950000;123;000123;2026-10-03T14:21:10Z;;tästér;dev;3f2a91;anchor;;'
    foreign = _body('1950000;9.99;25.0;101300;9.99', '000122_baro_bmp280.csv')
    log = [_log("900000 recorder :: {'session': '000123', 'lines': 9}"),
           _log('1000000 health :: ok'),
           baro_routing + '@' + bodies['1700000'],                   # a lost leading '@'
           _log('1750000 health :: ok'),
           '@' + baro_routing + bodies['1900000'],                   # a lost second '@'
           '%s_baro_bmp280.csv@%s' % (other, foreign),              # NEGATIVE: another session's leftover name
           'session.csv@' + _body(anchor, 'session.csv'),           # the anchor row lost its '@' too
           '000123_separation.csv@' + _body('1600000;deployed;gliding', '000123_separation.csv'),  # its only row
           sequencer_routing + '@' + _body('1600000;gliding;apogee', sequencer_routing).replace('apogee', 'apogeX'),
           'a line that lost both ends of its wrapper',               # NEGATIVE: proves nothing
           _log('2500000 sequencer :: stage -> boosting'),
           _log("2900000 recorder :: {'session': '000123', 'lines': 99}")]
    recordings = _folder()
    _write_dump(recordings, {
        imu_routing: imu_file,
        baro_routing: baro_file,
        'xx7_baro.cs': [bodies['1600000']],                    # a corrupted routing: a junk file
        '000123_baro_bmXX280.csv': [bodies['2000000']],        # ... one that keeps the session prefix
        '000123_junk.csv': [bodies['1300000'].replace(';25.0;', ';25.5;'), 'plain junk'],  # proves nothing
        'yy.csv': [bodies['1300000'].replace(';25.0;', ';25.7;')],  # ... nor this, perhaps another session's
        sequencer_routing: [_body('1500000;boosting;zündung', sequencer_routing)],
        'session.csv': [_body(row, 'session.csv') for row in clock],
    })
    damaged = b'\xff\xfe a byte the link damaged\n'
    with open(os.path.join(recordings, 'recorder.log'), 'wb') as handle:
        handle.write(('\n'.join(log[:2]) + '\n').encode('utf-8') + damaged + ('\n'.join(log[2:]) + '\n').encode())
    printed, out = _cut(recordings, session)
    assert 'ignition at uptime 1.500000 s' in printed, printed
    assert re.search(r'imu_lsm6dso32\s+195 good,\s+1 salvaged in,\s+7 rejected', printed), printed
    assert re.search(r'baro_bmp280\s+16 good,\s+6 salvaged in,\s+0 rejected', printed), printed
    # the damaged byte, another session's leftover name, the damaged sequencer row and the line with no
    # wrapper in recorder.log; the junk file's two lines
    assert re.search(r'\(no stream\)\s+0 good,\s+0 salvaged in,\s+6 rejected', printed), printed
    assert '  1 more damaged line(s) in junk files without this prefix' in printed, printed
    assert re.search(r'separation\s+0 good,\s+1 salvaged in,\s+0 rejected', printed), printed
    assert len(re.findall(r'dated by \S+ at uptime \d+ us \(raw ticks\): 2026-10-03', printed)) == 3, printed
    assert 'dated by anchor' in printed and 'dated by boot' not in printed, printed
    assert '2026-10-02' not in printed and other not in printed, printed
    assert 'ignition at 2026-10-03T14:21:08.100Z UTC, from the dashboard row at uptime 1.400000 s' in printed

    recorded = _read(os.path.join(out, 'recorder', 'baro_bmp280.csv'))
    assert recorded == [bodies[row.split(';')[0]] for row in baro], 'every baro row, proven, in time order'
    flown = _read(os.path.join(out, 'flight', 'baro_bmp280.csv'))
    assert flown[0] == 't_s;altitude;temperature;pressure;elevation;uptime_us', flown[0]
    assert [line.split(';')[-1] for line in flown[1:]] == [row.split(';')[0] for row in baro]
    imu_kept = [line.split(';')[-1] for line in _read(os.path.join(out, 'flight', 'imu_lsm6dso32.csv'))[1:]]
    lost = {'1100000', '1110000', '1120000', '1130000', '1140000', '1800000'}
    assert imu_kept == [row.split(';')[0] for row in imu if row.split(';')[0] not in lost], len(imu_kept)
    assert _read(os.path.join(out, 'flight', 'separation.csv')) == [
        't_s;event;stage;uptime_us', '0.100000;deployed;gliding;1600000']  # a stream no good row named
    # the raw bytes survive: a non-ASCII payload in recorder/ and flight/, a damaged byte in board.log
    with open(os.path.join(out, 'recorder', 'sequencer.csv'), 'rb') as handle:
        assert handle.read() == (_body('1500000;boosting;zündung', sequencer_routing) + '\n').encode('utf-8')
    assert _read(os.path.join(out, 'flight', 'sequencer.csv'))[1].startswith('0.000000;boosting;zündung;')
    with open(os.path.join(out, 'recorder', 'board.log'), 'rb') as handle:
        assert damaged in handle.read()


def test_recorder_flight_places_salvage_across_a_wrap():
    """
    A session longer than the ticks_us wrap (2**30 us, 17.9 min): recovered rows are placed by the good
    lines around them where they were found, at their unwrapped time, in order -- and a row nothing times
    (a junk file) is rejected, since its time has two candidates a wrap apart.

    TMS-7D sat 23.5 minutes on the pad. The first version placed rows by the stream's span alone, which in
    a session that long holds every recorded time twice, so it rejected every salvaged row of the flight.
    A sparse stream (the sequencer: one row at boot, the next ones in flight) never shows its own wraps, so
    its good rows keep their recorded time while a recovered one is placed at the unwrapped time; the cut
    takes both modulo the wrap into the flight window, where they must agree.
    """
    period = recorder_wire.TICKS_PERIOD
    session, imu_routing, baro_routing = '000123', '000123_imu_lsm6dso32.csv', '000123_baro_bmp280.csv'
    ignition_us = 1_440_000_000
    stamps = list(range(30_000_000, 1_400_000_000, 1_000_000)) + list(range(1_400_000_000, 1_460_000_001, 10_000))
    imu = ['%d;0.0;0.0;%.1f;0;0;0;1' % (stamp % period, 5.0 if ignition_us <= stamp < ignition_us + 1_500_000 else 1.0)
           for stamp in stamps]
    baro_times = range(30_500_000, 1_460_000_000, 1_000_000)
    baro = {stamp: _body('%d;1.0;25.0;101300;1.0' % (stamp % period), baro_routing) for stamp in baro_times}
    lost_at = (600_500_000, 1_445_500_000, 1_446_500_000)  # lost leading '@': stranded in recorder.log
    imu_file = [_body(row, imu_routing) for row in imu]
    imu_file[stamps.index(1_430_000_000)] += '@' + baro_routing + '@' + baro[1_430_500_000]  # merged, past the wrap
    sequencer_routing = '000123_sequencer.csv'
    sequencer = {stamp: _body('%d;%s' % (stamp % period, event), sequencer_routing)
                 for stamp, event in ((5_000_000, 'setting;boot'), (1_440_000_000, 'boosting;launch'),
                                      (1_443_000_000, 'gliding;burnout'))}
    log = [_log("1000000 recorder :: {'session': '000123', 'lines': 1}")]
    for tick in range(5_000_000, 1_460_000_001, 5_000_000):
        log.append(_log('%d health :: ok' % (tick % period)))
        log += [baro_routing + '@' + baro[stamp] for stamp in lost_at if tick < stamp < tick + 5_000_000]
        if tick == 1_435_000_000:
            log.append(sequencer_routing + '@' + sequencer[1_440_000_000])  # the ignition event lost its '@'
    recordings = _folder()
    _write_dump(recordings, {
        sequencer_routing: [sequencer[5_000_000], sequencer[1_443_000_000]],
        imu_routing: imu_file,
        baro_routing: [body for stamp, body in baro.items() if stamp not in lost_at + (1_430_500_000, 1_450_500_000)],
        '000123_baro_bmXX280.csv': [baro[1_450_500_000]],  # a junk file: nothing to time it by
        'recorder.log': log,
    })
    printed, out = _cut(recordings, session)
    assert 'ignition at uptime 1440.000000 s' in printed, printed
    assert re.search(r'baro_bmp280\s+%d good,\s+4 salvaged in,\s+1 rejected' % (len(baro) - 5), printed), printed
    assert re.search(r'imu_lsm6dso32\s+%d good,\s+1 salvaged in,\s+0 rejected' % (len(imu) - 1), printed), printed
    flown = [int(line.split(';')[-1]) for line in _read(os.path.join(out, 'flight', 'baro_bmp280.csv'))[1:]]
    window = [stamp for stamp in baro_times if stamp >= ignition_us - 1_000_000 and stamp != 1_450_500_000]
    assert flown == window, (flown[:3], window[:3])  # unwrapped, in order, the stranded ones in their place
    recorded = _read(os.path.join(out, 'recorder', 'baro_bmp280.csv'))
    assert recorded == [baro[stamp] for stamp in window], 'recorder/ keeps the proven rows in time order'
    assert re.search(r'sequencer\s+2 good,\s+1 salvaged in,\s+0 rejected', printed), printed
    assert _read(os.path.join(out, 'flight', 'sequencer.csv')) == [
        't_s;stage;reason;uptime_us', '0.000000;boosting;launch;1440000000', '3.000000;gliding;burnout;1443000000']


def test_a_label_never_takes_another_sessions_files():
    """
    Under a label another session's files can share the prefix: an older label could hold '_', so
    `hitl_f15_*` sits beside `hitl_*`. Neither recorder_flight nor assemble_capture may take them -- the
    cut and the capture would carry a foreign `f15_imu_lsm6dso32` stream. And the reverse must never
    happen: junk names that look like a session's (`tms-7d_bno055.csv`, the tail of `tms-7d_imu_bno055.csv`
    after a lost `imu_`) must not drop a real stream.
    """
    session, foreign = 'hitl', 'hitl_f15'
    files = {}
    for prefix in (session, foreign):
        imu_routing, baro_routing = prefix + '_imu_lsm6dso32.csv', prefix + '_baro_bmp280.csv'
        files[imu_routing] = [_body(row, imu_routing) for row in _imu_rows()]
        files[baro_routing] = [_body(row, baro_routing) for row in _baro_rows()]
        files[prefix + '_health.csv'] = [_body('1000000;35;31480912;10;0;0;0;0;0', prefix + '_health.csv')]
        files[prefix + '_sequencer.csv'] = [_body('1500000;boosting;launch', prefix + '_sequencer.csv')]
    files['hitl_attitude.csv'] = [_body('1600000;100;5', 'hitl_attitude.csv')]  # this session's, but undeclared
    files['recorder.log'] = [_log("900000 recorder :: {'session': 'hitl'}")]
    recordings = _folder()
    _write_dump(recordings, files)
    printed, out = _cut(recordings, session)
    assert 'f15' not in printed and 'ignition at uptime 1.500000 s' in printed, printed
    assert sorted(os.listdir(os.path.join(out, 'flight'))) == [
        'baro_bmp280.csv', 'health.csv', 'imu_lsm6dso32.csv', 'sequencer.csv']
    capture = os.path.join(_folder(), 'capture.txt')
    assemble_capture.assemble(session, recordings, capture)
    streams, _logs = flight_telemetry.parse('\n'.join(_read(capture)))
    assert sorted(streams) == ['attitude.csv', 'baro_bmp280.csv', 'health.csv', 'imu_lsm6dso32.csv',
                               'sequencer.csv'], sorted(streams)
    # NEGATIVE: the other label is still its own session, whole, and only that
    assemble_capture.assemble(foreign, recordings, capture)
    assert sorted({line[1:].partition('@')[0] for line in _read(capture) if line.startswith('@')}) == [
        'hitl_f15_baro_bmp280.csv', 'hitl_f15_health.csv', 'hitl_f15_imu_lsm6dso32.csv', 'hitl_f15_sequencer.csv']

    # junk tails under a label: both IMUs' tails, each holding a row its CRC files back where it belongs
    session = 'tms-7d'
    bno055, lsm6dso32 = session + '_imu_bno055.csv', session + '_imu_lsm6dso32.csv'
    bno055_rows = ['%d;90.0;0.0;0.0;0.0;0.0;9.8' % uptime for uptime in range(1000000, 3000001, 100000)]
    imu = _imu_rows()
    files = {bno055: [_body(row, bno055) for row in bno055_rows if not row.startswith('2000000;')],
             lsm6dso32: [_body(row, lsm6dso32) for row in imu if not row.startswith('2000000;')],
             session + '_bno055.csv': [_body(bno055_rows[10], bno055)],    # a lost 'imu_'
             session + '_lsm6dso32.csv': [_body(imu[100], lsm6dso32)],     # ... twice
             session + '_health.csv': [_body('1000000;35;31480912;10;0;0;0;0;0', session + '_health.csv')],
             'recorder.log': [_log("900000 recorder :: {'session': 'tms-7d'}")]}
    recordings = _folder()
    _write_dump(recordings, files)
    assemble_capture.assemble(session, recordings, capture)
    streams, _logs = flight_telemetry.parse('\n'.join(_read(capture)))
    assert flight_telemetry.line_counts()['salvaged'] == 2, flight_telemetry.line_counts()
    assert len(streams['imu_bno055.csv'].rows) == len(bno055_rows), sorted(streams)
    assert len(streams['imu_lsm6dso32.csv'].rows) == len(imu), sorted(streams)
    printed, out = _cut(recordings, session)
    assert re.search(r'imu_bno055\s+20 good,\s+1 salvaged in', printed), printed
    assert len(_read(os.path.join(out, 'flight', 'imu_lsm6dso32.csv'))) == 1 + len(imu), printed


_INDEX_HEADER = 'uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon'


def _boot_dump(session: str, index: list) -> str:
    """A wrapped dump of one short boot (_imu_rows(), _baro_rows()) beside the session.csv rows `index`."""
    imu_routing, baro_routing = session + '_imu_lsm6dso32.csv', session + '_baro_bmp280.csv'
    recordings = _folder()
    _write_dump(recordings, {
        imu_routing: [_body(row, imu_routing) for row in _imu_rows()],
        baro_routing: [_body(row, baro_routing) for row in _baro_rows()],
        'recorder.log': [_log("900000 recorder :: {'session': '%s'}" % session)],
        'session.csv': [_body(row, 'session.csv') for row in index],
    })
    return recordings


def test_recorder_flight_dates_every_boot():
    """
    session.csv lists every boot -- a 'boot' row, a row per time set, an 'anchor' row -- and only a row with
    a utc dates one. A boot nothing dated is named by its boot id, board, firmware and config, and placed
    between its board's dated neighbours, since boot ids only grow. A label may be 'session', which the
    header's own session cell reads: the header must never print as a row of it.
    """
    index = [_INDEX_HEADER, '2000000;121;000121;2026-10-03T10:00:00Z;-240;taster;dev;3f2a91;cc-auto;;',
             _INDEX_HEADER, '100000;122;000122;;;taster;dev;3f2a91;boot;;',
             '1500000;122;000122;;;taster;dev;3f2a91;cc-auto;25.5;-80.3',  # a set that left the clock in 2000
             _INDEX_HEADER, '61000000;122;000122;;;taster;dev;3f2a91;anchor;;',
             _INDEX_HEADER, '100000;123;000123;;;taster;dev;3f2a91;boot;;',
             '300000;124;000124;2026-10-03T11:00:00Z;;other;dev;3f2a91;anchor;;',  # another board: no neighbour
             '61000000;125;000125;2026-10-03T12:00:00Z;;taster;dev;3f2a91;anchor;;']
    printed, _out = _cut(_boot_dump('000122', index), '000122')
    assert 'boot 122 (board taster, firmware dev, config 3f2a91): clock never set; it ran after boot 121 ' \
           '(2026-10-03T10:00:00Z) and before boot 125 (2026-10-03T12:00:00Z)' in printed, printed
    assert 'dated by' not in printed and 'ignition at 20' not in printed, printed  # undated rows date nothing
    assert 'boot 123' not in printed and 'boot 121 (board' not in printed, 'another session\'s boot was listed'

    # NEGATIVE: no row names the session at all
    printed, _out = _cut(_boot_dump('000126', index), '000126')
    assert 'session.csv names no boot of this session: nothing dates it' in printed, printed

    # the label 'session': the header's third cell reads 'session' too; two boots share the label
    index = [_INDEX_HEADER, '100000;126;session;;;taster;dev;3f2a91;boot;;',
             '1200000;126;session;2026-10-03T13:00:00Z;-240;taster;dev;3f2a91;cc-auto;;',
             _INDEX_HEADER, '61000000;126;session;2026-10-03T13:01:00Z;;taster;dev;3f2a91;anchor;;',
             _INDEX_HEADER, '100000;127;session;;;taster;dev;3f2a91;boot;;']
    printed, _out = _cut(_boot_dump('session', index), 'session')
    assert re.findall(r'boot (\S+) \(board', printed) == ['126', '126', '127'], printed
    assert 'dated by cc-auto at uptime 1200000 us (raw ticks): 2026-10-03T13:00:00Z, local offset -240 min' in printed
    assert 'dated by anchor' in printed and 'boot 127 (board taster, firmware dev, config 3f2a91): clock never ' \
           'set; it ran after boot 126 (2026-10-03T13:01:00Z)' in printed, printed
    assert 'ignition at 20' not in printed, 'two boots share the label: which one flew is not the index\'s to say'
    # ... and with the one boot, the ignition gets its UTC from the latest dated row before it
    printed, _out = _cut(_boot_dump('session', index[:4]), 'session')
    assert 'ignition at 2026-10-03T13:00:00.300Z UTC, from the cc-auto row at uptime 1.200000 s' in printed, printed


def _long_dump(index: list) -> str:
    """
    A wrapped 24-minute boot past the ticks_us wrap, ignition at 1440 s, beside the session.csv rows `index`.

    Args:
        index - session.csv's rows, header included.

    Returns:
        The dump directory.
    """
    period = recorder_wire.TICKS_PERIOD
    imu_routing = '000123_imu_lsm6dso32.csv'
    stamps = list(range(30_000_000, 1_435_000_000, 5_000_000)) + list(range(1_435_000_000, 1_460_000_001, 10_000))
    imu = ['%d;0.0;0.0;%.1f;0;0;0;1' % (stamp % period, 5.0 if 1_440_000_000 <= stamp < 1_441_500_000 else 1.0)
           for stamp in stamps]
    recordings = _folder()
    _write_dump(recordings, {
        imu_routing: [_body(row, imu_routing) for row in imu],
        'recorder.log': [_log("1000000 recorder :: {'session': '000123'}")],
        'session.csv': [_body(row, 'session.csv') for row in index],
    })
    return recordings


def test_recorder_flight_dates_the_flight_across_a_wrap():
    """
    A row's uptime is raw ticks_us, so in a boot longer than a wrap a time set can sit at two places a
    wrap apart. The anchor row has one (a minute after the boot row), and a later set takes the place its
    utc puts at the right distance from it; the ignition then takes its UTC from the latest dated row
    before it. With the anchor lost, two sets still place each other when only one choice of wraps lets
    their utcs agree. A dated row nothing places is not guessed: the ignition's UTC is then unknown. And a
    count of wraps is never negative, so a boot whose streams start within the slack keeps a set just short
    of the first wrap at its one place.
    """
    period = recorder_wire.TICKS_PERIOD
    index = [_INDEX_HEADER, '30000000;123;000123;;;taster;dev;3f2a91;boot;;', _INDEX_HEADER,
             '90000000;123;000123;2026-10-03T14:00:01Z;;taster;dev;3f2a91;anchor;;',
             '%d;123;000123;2026-10-03T14:20:16Z;-240;taster;dev;3f2a91;dashboard;;' % (1_300_000_000 % period)]
    printed, _out = _cut(_long_dump(index), '000123')
    assert 'ignition at uptime 1440.000000 s' in printed, printed
    # the dashboard re-sync corrected the anchor's clock by 5 s: 14:00:01 + 1210 s + 5 s, then + 140 s
    assert 'ignition at 2026-10-03T14:22:36.000Z UTC, from the dashboard row at uptime 1300.000000 s' in printed
    # the anchor alone dates the boot: it went out a minute after the boot row, so it has one place
    printed, _out = _cut(_long_dump(index[:4]), '000123')
    assert 'ignition at 2026-10-03T14:22:31.000Z UTC, from the anchor row at uptime 90.000000 s' in printed, printed
    # the anchor lost: a set at 20 s (or 20 s + a wrap) and the re-sync agree only at 20 s and 1300 s
    index = [_INDEX_HEADER, '20000000;123;000123;2026-10-03T13:59:51Z;-240;taster;dev;3f2a91;cc-auto;;',
             index[-1]]
    printed, _out = _cut(_long_dump(index), '000123')
    assert 'ignition at 2026-10-03T14:22:36.000Z UTC, from the dashboard row at uptime 1300.000000 s' in printed
    # NEGATIVE: the only dated row is a set at 200 s -- or 200 s + a wrap -- and nothing else is dated
    index = [_INDEX_HEADER, '30000000;123;000123;;;taster;dev;3f2a91;boot;;', _INDEX_HEADER,
             '90000000;123;000123;;;taster;dev;3f2a91;anchor;;',
             '200000000;123;000123;2026-10-03T14:03:21Z;-240;taster;dev;3f2a91;cc-auto;;']
    printed, _out = _cut(_long_dump(index), '000123')
    assert 'dated by cc-auto at uptime 200000000 us' in printed, printed
    assert 'ignition UTC unknown: no dated row has a single place on the timeline' in printed, printed
    # ... and two sets that agree at both choices, 100 s apart: still two places each
    index.append('300000000;123;000123;2026-10-03T14:05:01Z;-240;taster;dev;3f2a91;dashboard;;')
    printed, _out = _cut(_long_dump(index), '000123')
    assert 'ignition UTC unknown: no dated row has a single place on the timeline' in printed, printed
    # the streams start 30 s in, inside the slack: a set in the minute before the first wrap has one place,
    # never a second at a negative uptime (1050 s less a wrap)
    index = [_INDEX_HEADER, '1050000000;123;000123;2026-10-03T14:20:00Z;-240;taster;dev;3f2a91;dashboard;;']
    printed, _out = _cut(_long_dump(index), '000123')
    assert 'ignition at 2026-10-03T14:26:30.000Z UTC, from the dashboard row at uptime 1050.000000 s' in printed


def test_recorder_flight_times_the_log_from_this_boot_only():
    """
    recorder.log's part for a session starts after the previous session's last status line, so it can open
    with that boot's last few lines. Their ticks are that boot's: read as this boot's, the fall to this
    boot's small ticks looked like a wrap, and every row stranded in the log was filed 17.9 minutes late
    -- out of the cut. A fall is a wrap only when it is one, and never before this session's first status
    line: a previous boot ending 30 s short of a wrap must not pass for one either.
    """
    period = recorder_wire.TICKS_PERIOD
    imu_routing, baro_routing = '000123_imu_lsm6dso32.csv', '000123_baro_bmp280.csv'
    stamps = list(range(10_000_000, 275_000_000, 1_000_000)) + list(range(275_000_000, 300_000_001, 10_000))
    imu = ['%d;0.0;0.0;%.1f;0;0;0;1' % (stamp, 5.0 if 280_000_000 <= stamp < 281_500_000 else 1.0) for stamp in stamps]
    baro = {stamp: _body('%d;1.0;25.0;101300;%d' % (stamp, stamp // 1000), baro_routing)
            for stamp in range(10_500_000, 300_000_000, 1_000_000)}
    previous = '000122_baro_bmp280.csv'
    for tail in (699_000_000, period - 30_000_000):
        log = [_log("%d recorder :: {'session': '000122'}" % tail), _log('%d health :: bye' % (tail + 500_000)),
               previous + '@' + _body('%d;1.0;25.0;101300;0' % (tail + 600_000), previous)]  # it lost its '@' too
        for tick in range(9_000_000, 300_000_000, 1_000_000):
            log.append(_log("%d recorder :: {'session': '000123'}" % tick))
            if tick == 200_000_000:
                log.append(baro_routing + '@' + baro[200_500_000])  # a lost leading '@'
        recordings = _folder()
        _write_dump(recordings, {imu_routing: [_body(row, imu_routing) for row in imu],
                                 baro_routing: [body for stamp, body in baro.items() if stamp != 200_500_000],
                                 'recorder.log': log})
        saved, sys.argv = sys.argv, ['recorder_flight.py', recordings, '--session', '000123', '-o', recordings,
                                     '--before', '100']
        try:
            import contextlib
            import io
            printed = io.StringIO()
            with contextlib.redirect_stdout(printed):
                recorder_flight.main()
        finally:
            sys.argv = saved
        assert re.search(r'baro_bmp280\s+289 good,\s+1 salvaged in,\s+0 rejected', printed.getvalue()), tail
        # the previous boot's stranded row is not this session's to count as rejected
        assert re.search(r'\(no stream\)\s+0 good,\s+0 salvaged in,\s+0 rejected', printed.getvalue()), tail
        assert '  1 more damaged line(s) in junk files' in printed.getvalue(), printed.getvalue()
        flown = [line.split(';') for line in _read(os.path.join(recordings, 'flight', 'baro_bmp280.csv'))[1:]]
        assert ['200500000', '200500'] in [cells[-1:] + cells[-2:-1] for cells in flown], (tail, flown[80:82])
        assert all(int(cells[-1]) // 1000 == int(cells[-2]) for cells in flown), 'a row filed at another time'


def test_recorder_flight_files_a_log_line_found_in_a_stream():
    """
    A board log line that ran on into a stream's file -- merged after a row that lost its newline -- is
    proven like a row and belongs to board.log, at its time among the session's log lines, counted under
    '(board.log)'. It used to be dropped without a word (flight_telemetry already merged it back). A log
    line proves no session, so one in a junk file without the prefix is not taken; one whose time fits no
    place, or in a session holding two boots (one label, the ticks restarted), is rejected. A damaged line
    of recorder.log is already in board.log, verbatim, and is never added a second time.
    """
    session, imu_routing = '000123', '000123_imu_lsm6dso32.csv'
    imu = [_body(row, imu_routing) for row in _imu_rows()]
    boosting, far = _log('1495000 sequencer :: stage -> boosting'), _log('900000000 health :: far')
    junk, elsewhere = _log('2505000 health :: junk file'), _log('2605000 health :: elsewhere')
    imu_file = list(imu)
    imu_file[50] += boosting       # 1.50 s: merged at a '>{' seam
    imu_file[120] += far           # 2.20 s: proven, but its time is nowhere near the rows around it
    for moved in (160, 150):       # rows whose routing the link mangled, each with a log line run on
        imu_file.pop(moved)
    log = [_log("900000 recorder :: {'session': '000123'}"), _log('1400000 health :: before'),
           _log('1600000 health :: after'), _log('2550000 health :: late'),
           _log('2560000 health :: one') + _log('2570000 health :: two'),  # merged in recorder.log: kept as is
           _log('2700000 health :: last')]
    recordings = _folder()
    _write_dump(recordings, {imu_routing: imu_file, 'recorder.log': log,
                             '000123_imu_lsmXX.csv': [imu[150] + junk],       # a junk file with the prefix
                             'zz.csv': [imu[160] + elsewhere]})               # NEGATIVE: one without it
    printed, out = _cut(recordings, session)
    assert re.search(r'\(board\.log\)\s+5 good,\s+2 salvaged in,\s+1 rejected', printed), printed
    assert re.search(r'imu_lsm6dso32\s+197 good,\s+4 salvaged in,\s+0 rejected', printed), printed
    assert 'more damaged line' not in printed, printed  # the junk file's row is this session's: claimed
    assert _read(os.path.join(out, 'recorder', 'board.log')) == log[:2] + [boosting] + log[2:3] + [junk] + log[3:]
    assert len(_read(os.path.join(out, 'flight', 'imu_lsm6dso32.csv'))) == 1 + len(imu)

    # NEGATIVE: two boots under one label -- board.log has no one timeline to put the line on
    session, imu_routing = 'tms-7d', 'tms-7d_imu_lsm6dso32.csv'
    rows = ['%d;0.0;0.0;%.1f;0;0;0;1' % (uptime, 5.0 if boot == 2 and 20_000_000 <= uptime < 21_500_000 else 1.0)
            for boot in (1, 2) for uptime in range(1_000_000, 40_000_001, 10_000)]  # the ticks restart: 1..40 s twice
    imu_file = [_body(row, imu_routing) for row in rows]
    imu_file[-500] += _log('35005000 sequencer :: stage -> gliding')  # boot 2, 35 s
    recordings = _folder()
    _write_dump(recordings, {imu_routing: imu_file, 'recorder.log': [
        _log("%d recorder :: {'session': 'tms-7d'}" % tick) for tick in (900_000, 30_000_000, 900_000, 30_000_000)]})
    printed, out = _cut(recordings, session)
    assert 'session tms-7d: 2 boot(s)' in printed and 'the flight is boot 1' in printed, printed
    assert re.search(r'\(board\.log\)\s+\d+ good,\s+0 salvaged in,\s+1 rejected', printed), printed
    assert 'gliding' not in ''.join(_read(os.path.join(out, 'recorder', 'board.log')))


def test_recorder_flight_never_times_by_a_wrap_its_file_missed():
    """
    A sparse stream is silent across the ticks_us wrap -- the sequencer here, a servo still through the pad
    dwell -- so its file never shows the wrap, and its good lines past it keep a time one wrap short. A
    record found among them, run on after a row that lost its newline, was timed by them and filed a wrap
    (1073.7 s) early, counted as salvaged in: a log line into board.log among the lines of 366 s, a 5 g
    boost row into flight/ at t_s -1073.7. In a session longer than a wrap such a record has two places,
    so it is rejected and counted. The same kind of log line found in the dense IMU file IS placed, by
    the good rows around it -- the session's span alone could not tell its two places apart -- early in
    the session too, where the rows that vouch for those around it come much later. recorder.log is such
    a file too, when it falls silent across the wrap.

    board.log is this session's part of recorder.log only: the previous boot's last line at its head,
    never the previous session's status nor the next session's lines. Its good and salvaged in count
    exactly its lines that check out.
    """
    period = recorder_wire.TICKS_PERIOD
    imu_routing, sequencer_routing = '000123_imu_lsm6dso32.csv', '000123_sequencer.csv'
    stamps = list(range(30_000_000, 1_435_000_000, 5_000_000)) + list(range(1_435_000_000, 1_460_000_001, 10_000))
    moved = 1_440_010_000  # this boost row runs on into the sequencer file: marked by irq_runs 7
    imu = {stamp: _body('%d;0.0;0.0;%.1f;0;0;0;%d' % (stamp % period, 5.0 if 1_440_000_000 <= stamp < 1_441_500_000
                                                      else 1.0, 7 if stamp == moved else 1), imu_routing)
           for stamp in stamps}
    sparse = _log('%d sequencer :: stage -> boosting' % (1_440_000_500 % period))
    dense = {stamp: _log('%d health :: ran on' % ((stamp + 700) % period)) for stamp in (100_000_000, 1_450_000_000)}
    sequencer = [_body('%d;%s' % (stamp % period, stage), sequencer_routing) for stamp, stage in (
        (400_000_000, 'setting;boot'), (1_200_000_000, 'armed;pad'), (1_440_000_000, 'boosting;launch'),
        (1_442_000_000, 'coasting;burnout'), (1_443_000_000, 'gliding;burnout'), (1_445_000_000, 'gliding;apogee'))]
    sequencer[2] += sparse                                       # merged at a '>{' seam
    sequencer[3] += '@' + imu_routing + '@' + imu[moved]          # ... and at a '>@' one
    imu_file = [body + dense.get(stamp, '') for stamp, body in imu.items() if stamp != moved]
    tail = _log('699500000 health :: bye')                      # the previous boot's last line
    part = [tail, _log("1000000 recorder :: {'session': '000123'}")] + [
        _log('%d health :: at %d s' % (second * 1_000_000 % period, second)) for second in range(2, 1461)]
    log = ([_log("699000000 recorder :: {'session': '000122'}")] + part +
           [_log("900000 recorder :: {'session': '000124'}"), _log('1900000 health :: next boot')])
    recordings = _folder()
    _write_dump(recordings, {imu_routing: imu_file, sequencer_routing: sequencer, 'recorder.log': log})
    printed, out = _cut(recordings, '000123', '1200')
    assert 'ignition at uptime 1440.000000 s' in printed, printed
    # the IMU rows the dense run-ons followed are salvaged in; the one in the sequencer file has two places.
    # The sequencer's own two rows on the merged lines are its, placed modulo the wrap by the window.
    assert re.search(r'imu_lsm6dso32\s+%d good,\s+2 salvaged in,\s+1 rejected' % (len(stamps) - 3), printed), printed
    assert re.search(r'\(board\.log\)\s+%d good,\s+2 salvaged in,\s+1 rejected' % len(part), printed), printed
    assert re.search(r'sequencer\s+4 good,\s+2 salvaged in,\s+0 rejected', printed), printed
    flown = [line.split(';') for line in _read(os.path.join(out, 'flight', 'imu_lsm6dso32.csv'))[1:]]
    assert flown and all(float(cells[0]) >= -1200.0 and cells[-2] == '1' for cells in flown), 'a row filed a wrap early'
    board = _read(os.path.join(out, 'recorder', 'board.log'))
    expected = list(part)
    for stamp, line in sorted(dense.items(), reverse=True):  # each after the log line of its second
        expected.insert(expected.index(_log('%d health :: at %d s' % (stamp % period, stamp // 1_000_000))) + 1, line)
    assert board == expected, 'board.log: this session\'s part, each dense run-on in its place'
    assert len(board) == len(part) + 2 == sum(1 for line in board if recorder_wire.verify(None, line) is not None)

    # recorder.log is such a file too: silent for 200 s across the wrap, its ticks after it keep a time one
    # wrap short, so a row stranded among them (a lost leading '@') has two places as well
    quiet = [_log("1000000 recorder :: {'session': '000123'}")]
    for second in list(range(5, 1001, 5)) + list(range(1200, 1461, 5)):
        quiet.append(_log('%d health :: at %d s' % (second * 1_000_000 % period, second)))
        if second == 1440:
            quiet.append(imu_routing + '@' + imu[moved])
    recordings = _folder()
    imu_file = [body for stamp, body in imu.items() if stamp != moved]
    _write_dump(recordings, {imu_routing: imu_file, 'recorder.log': quiet})
    printed, out = _cut(recordings, '000123', '1200')
    assert re.search(r'imu_lsm6dso32\s+%d good,\s+0 salvaged in,\s+1 rejected' % (len(stamps) - 1), printed), printed
    flown = [line.split(';')[-2] for line in _read(os.path.join(out, 'flight', 'imu_lsm6dso32.csv'))[1:]]
    assert flown and '7' not in flown, 'a row filed a wrap early'


def test_every_reader_takes_a_damaged_byte():
    """
    A byte the link damaged is not UTF-8. Every tool reads a capture through flight_telemetry.load()
    (surrogateescape), so it costs its own line and never the read: the six parse() callers that used a
    plain open() raised UnicodeDecodeError on the whole capture, and assemble_capture writes such a byte
    through unchanged, as the Luckfox holds it.
    """
    import contextlib
    import io
    import subprocess
    directory = _folder()
    capture = os.path.join(directory, 'damaged.txt')
    with open(capture, 'wb') as handle:
        handle.write(flight_synth_capture.generate().encode() + b'1 health :: \xff\xfe damaged\n')
    _streams, logs = flight_telemetry.load(capture)
    assert logs[-1][1] == '1 health :: \udcff\udcfe damaged', logs[-1]
    with contextlib.redirect_stdout(io.StringIO()) as printed:
        flight_kpi.report('damaged', capture, _ZONE)
        glide_polar.report('damaged', capture, 0.0, 0.0)
    assert 'damaged' in printed.getvalue()
    assert flight_metrics.metrics(capture) is not None
    assert flight_campaign.measure(capture)['apogee'] is not None
    assert hitl_compare._metrics(capture)
    svg = subprocess.run([sys.executable, os.path.join(_ROOT, 'tools', 'flight_svg.py'), capture,
                          '-o', os.path.join(directory, 'damaged.svg')], capture_output=True, text=True)
    assert svg.returncode == 0, svg.stderr
    """
    flight_report: the byte in a STAGE line reaches the figure, and orjson (plotly's serialiser) refuses
    its surrogate -- the report exited 1. It renders as U+FFFD; the helper runs directly without plotly.
    """
    staged, report = os.path.join(directory, 'staged.txt'), os.path.join(directory, 'damaged.html')
    with open(staged, 'wb') as handle:
        handle.write(flight_synth_capture.generate().encode() + b'5000000 controller :: stage -> glid\xffing\n')
    saved, sys.argv = sys.argv, ['flight_report.py', staged, '-o', report, '--cdn']
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            flight_report.main()  # reads the capture first: without plotly it then stops, with plotly it renders
        with open(report, encoding='utf-8') as handle:
            rendered = handle.read()
        assert 'glid\ufffding' in rendered or 'glid\\ufffding' in rendered, 'the stage line was not rendered'
    except SystemExit as stop:
        assert 'needs plotly' in str(stop), stop
    finally:
        sys.argv = saved
    assert flight_report._printable('5000000 controller :: stage -> glid\udcffing') == \
        '5000000 controller :: stage -> glid\ufffding'
    assert flight_report._printable('\udcff\udcfe') == '\ufffd\ufffd'  # one U+FFFD per damaged byte
    assert flight_report._printable('1500000 sequencer :: zündung') == '1500000 sequencer :: zündung'  # NEGATIVE
    """
    The report on any host: plotly and the figure stubbed, main() hands build() what it would draw. Two
    junk stream names that differ only in a damaged byte stay two streams -- as U+FFFD they were one key,
    and one stream dropped out -- and neither passes for a name that holds a backslash. Nothing build()
    is handed keeps a lone surrogate, which orjson would refuse.
    """
    tag = flight_synth_capture._SESSION.encode()
    with open(staged, 'ab') as handle:
        for name, runs in ((b'\xff', 1), (b'\xfe', 0), (b'\\xff', 2)):  # two damaged bytes, and a backslash
            handle.write(b'@%s_irq%s.csv@uptime;irq_runs\n@%s_irq%s.csv@1000000;%d\n' % (tag, name, tag, name, runs))
    drawn = {}

    def build(streams: dict, logs: list, *_figure) -> tuple:
        """build() without plotly: keeps what main() hands it."""
        drawn.update(streams=streams, logs=logs)
        return None, None

    saved = sys.argv, flight_report._require_plotly, flight_report.build, flight_report.write_html
    sys.argv = ['flight_report.py', staged, '-o', report]
    flight_report._require_plotly = lambda: (None, None, None)
    flight_report.build, flight_report.write_html = build, lambda *_arguments: None
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            flight_report.main()
    finally:
        sys.argv, flight_report._require_plotly, flight_report.build, flight_report.write_html = saved
    names = sorted(name for name in drawn['streams'] if name.startswith('irq'))
    assert names == ['irq\\\\xff.csv', 'irq\\xfe.csv', 'irq\\xff.csv'], names
    for text in list(drawn['streams']) + [line for _stamp, line in drawn['logs']]:
        text.encode('utf-8')  # strict: a lone surrogate raises here as it does in the figure
    assert '5000000 controller :: stage -> glid�ing' in [line for _stamp, line in drawn['logs']]
    # assemble_capture keeps the byte as the Luckfox file holds it
    routing = '000123_imu_lsm6dso32.csv'
    with open(os.path.join(directory, routing), 'wb') as handle:
        handle.write(_body(_imu_rows()[0], routing).encode() + b'\n' + b'{0a};1010000;\xff;<0b>\n')
    with contextlib.redirect_stdout(io.StringIO()):
        assemble_capture.assemble('000123', directory, capture)
    with open(capture, 'rb') as handle:
        assert ('@%s@{0a};1010000;\xff;<0b>\n' % routing).encode('latin-1') in handle.read()


_FAKE_ADB = '\n'.join([
    '#!/usr/bin/env python3',
    '"""A fake adb over a local directory, in bytes like the real one: `shell "ls [-t] ..."`, `pull <path> <dir>`."""',
    'import os',
    'import shutil',
    'import sys',
    '',
    "recordings = os.environ['FAKE_RECORDINGS'].encode()",
    'names = sorted(os.listdir(recordings))',
    "if sys.argv[1] == 'shell' and sys.argv[2].startswith('ls -t'):",
    '    names.sort(key=lambda name: -os.path.getmtime(os.path.join(recordings, name)))',
    "    listed = [b'/userdata/recordings/' + name for name in names if b'_' in name and name.endswith(b'.csv')]",
    "    sys.stdout.buffer.write(b'\\n'.join(listed) + b'\\n')",
    "elif sys.argv[1] == 'shell' and sys.argv[2].startswith('ls '):",
    "    sys.stdout.buffer.write(b'\\n'.join(names) + b'\\n')",
    "elif sys.argv[1] == 'pull':",
    '    shutil.copy(os.path.join(recordings, os.fsencode(os.path.basename(sys.argv[2]))), os.fsencode(sys.argv[3]))',
    ''])


def test_flight_pull_takes_a_session_not_a_junk_name():
    """
    flight_pull.sh with no session pulls the newest one: the session whose second-newest file is the newest.
    The newest FILE is often a junk name the link made (`000124_xx.csv`, or the older shape's
    `20000101_000005_1927_gl.csv`), and a lone junk name must not become "the session" -- nor lift an OLD
    session whose id it happens to repeat (`000122_xx.csv`), which counting a prefix over the whole listing
    did. Junk names whose bytes are not UTF-8 (the TMS-7D card held two) or that hold a space must not stop
    the pull. The default never picks a label session, even a newer one, and the usage says to name it.
    Given a label, it pulls that session's files and not an older label's sharing the prefix. Run against
    a fake adb.
    """
    import subprocess
    import time
    recordings, bin_dir, out, labelled = (_folder() for _ in range(4))
    files = {'000122_imu_lsm6dso32.csv': ['1;0;0;1;0;0;0;1'], '000122_health.csv': ['1;35;1;1;0;0;0;0;0'],
             '000123_imu_lsm6dso32.csv': [_body(row, '000123_imu_lsm6dso32.csv') for row in _imu_rows()],
             '000123_baro_bmp280.csv': [_body(row, '000123_baro_bmp280.csv') for row in _baro_rows()],
             '000123_imu l.csv': [_body(_imu_rows()[0], '000123_imu_lsm6dso32.csv')],  # a junk name with a space
             'hitl_health.csv': ['1;35;1;1;0;0;0;0;0'], 'hitl_imu.csv': ['1;1'], 'hitl_baro.csv': ['1;1'],
             'hitl_flight.csv': ['1;1'], 'hitl_f15_health.csv': ['1;35;1;1;0;0;0;0;0'], 'hitl_f15_imu.csv': ['1;1'],
             'hitl_f15_baro.csv': ['1;1'], 'hitl_f15_flight.csv': ['1;1'],
             'tms-7d_imu_lsm6dso32.csv': ['1;1'], 'tms-7d_health.csv': ['1;35;1;1;0;0;0;0;0'],  # newer, a label
             '000122_xx.csv': ['junk'], '000124_xx.csv': ['junk'], '20000101_000005_1927_gl.csv': ['junk'],
             'recorder.log': ['900000 boot']}
    _write_dump(recordings, files)
    now = time.time()
    for name in (b'20000101_000\xff', b'20000101_000256_705050\xdf', b'000123_imu\xff.csv'):  # old junk
        with open(os.path.join(os.fsencode(recordings), name), 'w') as handle:
            handle.write(_body(_imu_rows()[1], '000123_imu_lsm6dso32.csv') + '\n')
        os.utime(os.path.join(os.fsencode(recordings), name), (now - 200, now - 200))
    order = ['000122_imu_lsm6dso32.csv', '000122_health.csv', 'hitl_health.csv', 'hitl_imu.csv', 'hitl_baro.csv',
             'hitl_flight.csv', 'hitl_f15_health.csv', 'hitl_f15_imu.csv', 'hitl_f15_baro.csv',
             'hitl_f15_flight.csv', 'recorder.log', '000123_imu_lsm6dso32.csv', '000123_baro_bmp280.csv',
             '000123_imu l.csv', 'tms-7d_imu_lsm6dso32.csv', 'tms-7d_health.csv', '000122_xx.csv', '000124_xx.csv',
             '20000101_000005_1927_gl.csv']
    for age, name in enumerate(order):  # oldest first: the junk names are the newest files
        os.utime(os.path.join(recordings, name), (now - 100 + age, now - 100 + age))
    adb = os.path.join(bin_dir, 'adb')
    with open(adb, 'w') as handle:
        handle.write(_FAKE_ADB)
    os.chmod(adb, 0o755)
    environment = dict(os.environ, FAKE_RECORDINGS=recordings, PATH=bin_dir + os.pathsep + os.environ['PATH'],
                       PLY=os.path.join(bin_dir, 'no-plotly'))
    script = os.path.join(_ROOT, 'tools', 'flight_pull.sh')
    pulled = subprocess.run(['bash', script, '', out], env=environment, capture_output=True)
    printed = pulled.stdout.decode('utf-8', 'backslashreplace') + pulled.stderr.decode('utf-8', 'backslashreplace')
    assert 'latest session: 000123' in printed, printed  # not the newer label session
    assert 'pulled 4 streams' in printed, printed
    with open(script) as handle:
        usage = ' '.join(line.strip('#\n ') for line in handle.read().split('set -u')[0].splitlines())
    assert 'the default never picks a label session, so a labelled run needs this argument' in usage, usage
    assert sorted(name for name in os.listdir(os.fsencode(out)) if name.endswith(b'.csv')) == [
        b'000123_baro_bmp280.csv', b'000123_imu l.csv', b'000123_imu_lsm6dso32.csv', b'000123_imu\xff.csv']
    assert 'assembled %s' % os.path.join(out, '000123.txt') in printed, printed  # the junk names print, escaped
    pulled = subprocess.run(['bash', script, 'hitl', labelled], env=environment, capture_output=True, text=True)
    assert 'pulled 4 streams' in pulled.stdout, pulled.stdout + pulled.stderr
    assert sorted(name for name in os.listdir(labelled) if name.endswith('.csv')) == [
        'hitl_baro.csv', 'hitl_flight.csv', 'hitl_health.csv', 'hitl_imu.csv']


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
test_recorder_flight_knows_the_gnss_streams_as_declared()
test_recorder_flight_salvages_a_wrapped_dump()
test_recorder_flight_reads_an_unwrapped_dump_as_before()
test_assemble_capture_keeps_a_wrapped_capture_strict()
test_recorder_flight_error_matrix()
test_recorder_flight_places_salvage_across_a_wrap()
test_a_label_never_takes_another_sessions_files()
test_flight_pull_takes_a_session_not_a_junk_name()
test_recorder_flight_dates_every_boot()
test_recorder_flight_dates_the_flight_across_a_wrap()
test_recorder_flight_times_the_log_from_this_boot_only()
test_recorder_flight_files_a_log_line_found_in_a_stream()
test_recorder_flight_never_times_by_a_wrap_its_file_missed()
test_every_reader_takes_a_damaged_byte()
print('ok: tools -- board-shape fins rebuild, kpi golden + partial captures, touchdown at DONE, '
      'polled-IRQ summary, adxl-only backstop, cc.py verdict exit codes, logger join continuity, svg render, '
      'airspeed calibration fit, parser edge cases, session-tag eras (boot id included), '
      'spliced-capture detection, sim-capture refusal, provider/consumer closure, '
      'the GNSS field lists as gnss.py declares them, '
      'recorder_flight on wrapped + unwrapped dumps, assemble_capture strictness, the recorder_flight error '
      'matrix (every unproven line counted), salvage across a ticks wrap (a sparse stream too), label-prefix '
      'collisions and junk tails, flight_pull session pick (byte-safe, never a label), every boot of the session '
      'index, the flight\'s UTC across a wrap, the log timeline of this boot only, a log line found in a stream, '
      'nothing timed by a wrap its file missed, a damaged byte through every reader (and the report, names too)')
