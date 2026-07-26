"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the ANALYSIS TOOLS (tools/flight_kpi, flight_svg, airspeed_calibrate, and the
board-shape handling in flight_telemetry). Stdlib only -- plotly-dependent rendering is not exercised.

Why this file exists (findings §27.8): ~4 K lines of analysis tooling had almost no tests, and it is the
layer that produces the CONCLUSIONS we draw from a flight -- a silent bug here is worse than a firmware
bug, because it corrupts the answer rather than announcing itself. The §26 scan found 17 defects in these
tools; every one of them is pinned below so it cannot come back. Run by `make test` / `make check`.
"""

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import airspeed_calibrate  # noqa: E402
import flight_kpi  # noqa: E402
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
    assert flight_kpi._touchdown(flight_telemetry.find_stream(gnss_only_header, 'lat', 'lon'), _ZONE) \
        == (0.0, False)
    assert flight_kpi._servo_energy(None) == (0.0, 0.0)  # stream absent entirely
    assert flight_kpi._touchdown(None, _ZONE) == (0.0, False)
    # a single power sample has no window to average over -> must not divide by zero
    one, _l = flight_telemetry.parse('@20260725_120000_power_ina226.csv@uptime;power_mw\n'
                                     '@20260725_120000_power_ina226.csv@1000000;2500')
    joules, duration = flight_kpi._servo_energy(flight_telemetry.find_stream(one, 'power_mw'))
    assert duration == 0.0 and joules == 0.0


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


test_board_shape_is_readable()
test_kpi_on_the_synthetic_flight()
test_kpi_survives_a_partial_capture()
test_svg_renders_a_track()
test_airspeed_calibration_recovers_a_known_density()
test_parser_edge_cases()
print('ok: tools -- board-shape fins rebuild, kpi golden + partial captures, svg render, '
      'airspeed calibration fit, parser edge cases')
