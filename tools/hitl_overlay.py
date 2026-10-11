#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Overlay a Rust board's HITL flight on a plan B flight of the same scenario, both aligned on their own BOOSTING entry
(the stage machine's launch detection), so the two stage machines and their timelines compare directly:

  * height above the pad (the ICP-10111 stream's elevation), each flight's stage changes marked;
  * the boost's |a| (the LSM6DSO32 stream), the first seconds;
  * airspeed (the SDP810 stream);
  * the flight law's roll setpoint and roll command (flight.csv), and the left eleron's angle (the fin stream);
  * the ground track in metres from the pad, with the landing zone (flight_kpi's HPRC rectangle).

The table adds each touchdown's miss from the zone centre and whether it is inside, measured as flight_kpi does: the
last fix at or before DONE (the last fix when the flight logged none, as a virtual flight stops at touchdown).

The Rust side is tools/recorder_flight.py's cut (<launch>/flight/<stream>.csv, t_s from ignition, sequencer.csv's
stage changes); the plan B side is any capture tools/flight_telemetry.py parses (tools/virtual_flight.py's, or a
board HITL capture), its stage from flight.csv. A table of the milestones goes to stdout and into the figure.

  ~/.local/share/pipx/venvs/plotly/bin/python tools/hitl_overlay.py <rust_launch> <planb_capture> -o <out_prefix>

Writes <out_prefix>.html (interactive, plotly.js from the CDN) and <out_prefix>.svg (needs kaleido).
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_kpi  # noqa: E402
import flight_telemetry  # noqa: E402

_STAGES: dict = {1: 'setting', 2: 'boosting', 3: 'gliding', 4: 'landing', 5: 'done'}
_METRES_PER_DEGREE: float = 111_320.0
_BOOST_WINDOW_S: tuple = (-2.0, 10.0)


class Flight:
    """One flight's series, in seconds from its BOOSTING entry."""

    def __init__(self, label: str) -> None:
        self.label: str = label
        self.height: tuple = ([], [])  # (t, m)
        self.accel: tuple = ([], [])  # (t, |a| g)
        self.airspeed: tuple = ([], [])  # (t, m/s)
        self.track: tuple = ([], [])  # (east m, north m)
        self.stages: list = []  # (t, stage name)
        self.roll_setpoint: tuple = ([], [])  # (t, deg)
        self.roll_command: tuple = ([], [])  # (t, deg)
        self.fin: tuple = ([], [])  # (t, the left eleron's angle deg)
        self.origin: tuple | None = None  # the first fix (lat, lon): the track's and the zone's origin
        self.touchdown: tuple | None = None  # (miss m, inside) or None without a fix

    def milestones(self) -> dict:
        """When each stage began, the apogee height and the peak |a|."""
        found = {name: t for t, name in reversed(self.stages)}
        heights = self.height[1]
        return {'gliding_s': found.get('gliding'), 'landing_s': found.get('landing'), 'done_s': found.get('done'),
                'apogee_m': max(heights) if heights else None,
                'peak_g': max(self.accel[1]) if self.accel[1] else None,
                'miss_m': self.touchdown[0] if self.touchdown else None,
                'inside': self.touchdown[1] if self.touchdown else None}


def _rust_column(folder: str, stream: str, *fields: str) -> list:
    """The rows of `<folder>/flight/<stream>.csv` as (t_s, field values...), the text None read as None."""
    path = os.path.join(folder, 'flight', stream + '.csv')
    if not os.path.exists(path):
        return []
    rows = []
    with open(path, newline='') as handle:
        for row in csv.DictReader(handle, delimiter=';'):
            try:
                values = [None if row[field] in ('', 'None') else float(row[field]) for field in fields]
                rows.append((float(row['t_s']), *values))
            except (KeyError, ValueError):
                continue
    return rows


def _metres(origin: tuple, points: list) -> tuple:
    """(east, north) metres of (lat, lon) `points` from `origin`, equirectangular at its latitude."""
    latitude0, longitude0 = origin
    scale = math.cos(math.radians(latitude0))
    return ([(longitude - longitude0) * _METRES_PER_DEGREE * scale for _, longitude in points],
            [(latitude - latitude0) * _METRES_PER_DEGREE for latitude, _ in points])


def _track(points: list) -> tuple:
    """(east, north) metres from the first fix."""
    return _metres(points[0], points) if points else ([], [])


def _zone() -> tuple:
    """The landing rectangle ((lat, lon) top-left, (lat, lon) bottom-right): flight_kpi's HPRC default."""
    latitude_top, longitude_left, latitude_bottom, longitude_right = (
        float(value) for value in flight_kpi._ZONE_DEFAULT.split(','))
    return (latitude_top, longitude_left), (latitude_bottom, longitude_right)


def _touchdown(fixes: list, done_s: float | None) -> tuple | None:
    """
    The touchdown against the zone, as flight_kpi._touchdown measures it.

    Args:
        fixes - (t, lat, lon) in flight seconds.
        done_s - DONE in the same seconds, or None to take the last fix.

    Returns:
        (metres from the zone centre, inside the rectangle), or None without a fix at or before DONE.
    """
    kept = [(latitude, longitude) for t, latitude, longitude in fixes if done_s is None or t <= done_s]
    if not kept:
        return None
    latitude, longitude = kept[-1]
    (latitude_top, longitude_left), (latitude_bottom, longitude_right) = _zone()
    centre = ((latitude_top + latitude_bottom) / 2, (longitude_left + longitude_right) / 2)
    east, north = _metres(centre, [(latitude, longitude)])
    inside = (min(latitude_top, latitude_bottom) <= latitude <= max(latitude_top, latitude_bottom)
              and min(longitude_left, longitude_right) <= longitude <= max(longitude_left, longitude_right))
    return math.hypot(east[0], north[0]), inside


def rust_flight(folder: str, label: str) -> Flight:
    """The Rust board's flight from recorder_flight.py's cut, aligned on sequencer.csv's boosting row."""
    flight = Flight(label)
    with open(os.path.join(folder, 'flight', 'sequencer.csv'), newline='') as handle:
        changes = [(float(row['t_s']), row['stage']) for row in csv.DictReader(handle, delimiter=';')]
    zero = next(t for t, stage in changes if stage == 'boosting')
    flight.stages = [(t - zero, stage) for t, stage in changes]
    rows = _rust_column(folder, 'baro_icp10111', 'elevation')
    flight.height = ([t - zero for t, _ in rows], [value for _, value in rows])
    rows = _rust_column(folder, 'imu_lsm6dso32', 'ax', 'ay', 'az')
    window = [(t - zero, math.sqrt(x * x + y * y + z * z)) for t, x, y, z in rows
              if _BOOST_WINDOW_S[0] <= t - zero <= _BOOST_WINDOW_S[1]]
    flight.accel = ([t for t, _ in window], [g for _, g in window])
    rows = _rust_column(folder, 'airspeed_sdp810', 'airspeed_cms')
    flight.airspeed = ([t - zero for t, _ in rows], [value / 100.0 for _, value in rows])
    fixes = [(t - zero, latitude, longitude) for t, latitude, longitude in _rust_column(folder, 'gnss', 'lat', 'lon')
             if latitude is not None and longitude is not None]
    flight.track = _track([(latitude, longitude) for _, latitude, longitude in fixes])
    flight.origin = (fixes[0][1], fixes[0][2]) if fixes else None
    flight.touchdown = _touchdown(fixes, flight.milestones()['done_s'])
    rows = _rust_column(folder, 'flight', 'roll_sp', 'roll_cmd')
    flight.roll_setpoint = ([t - zero for t, _, _ in rows], [setpoint / 100.0 for _, setpoint, _ in rows])
    flight.roll_command = ([t - zero for t, _, _ in rows], [command for _, _, command in rows])
    rows = _rust_column(folder, 'servo_eleron_left', 'angle')
    flight.fin = ([t - zero for t, _ in rows], [angle for _, angle in rows])
    return flight


def planb_flight(capture: str, label: str) -> Flight:
    """A plan B capture's flight, aligned on flight.csv's first BOOSTING row."""
    with open(capture, errors='replace') as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    flight = Flight(label)
    times, stages = streams['flight.csv'].column('stage')
    changes, last = [], None
    for t, stage in zip(times, stages):
        if stage != last:
            changes.append((t, _STAGES.get(int(stage), str(stage))))
            last = stage
    zero = next(t for t, stage in changes if stage == 'boosting')
    flight.stages = [(t - zero, stage) for t, stage in changes]
    baro = streams.get('baro_icp10111.csv')
    if baro is not None:
        times, values = baro.column('elevation')
        flight.height = ([t - zero for t in times], values)
    imu = streams.get('imu_lsm6dso32.csv')
    if imu is not None:
        axes = [imu.column(axis) for axis in ('ax', 'ay', 'az')]
        window = [(t - zero, math.sqrt(x * x + y * y + z * z))
                  for t, x, y, z in zip(axes[0][0], axes[0][1], axes[1][1], axes[2][1])
                  if _BOOST_WINDOW_S[0] <= t - zero <= _BOOST_WINDOW_S[1]]
        flight.accel = ([t for t, _ in window], [g for _, g in window])
    pitot = streams.get('airspeed_sdp810.csv')
    if pitot is not None:
        times, values = pitot.column('airspeed_cms')
        flight.airspeed = ([t - zero for t in times], [value / 100.0 for value in values])
    gnss = streams.get('gnss.csv')
    if gnss is not None:
        longitude_at = dict(zip(*gnss.column('lon')))
        fixes = [(t - zero, latitude, longitude_at[t]) for t, latitude in zip(*gnss.column('lat')) if t in longitude_at]
        flight.track = _track([(latitude, longitude) for _, latitude, longitude in fixes])
        flight.origin = (fixes[0][1], fixes[0][2]) if fixes else None
        flight.touchdown = _touchdown(fixes, flight.milestones()['done_s'])
    times, setpoints = streams['flight.csv'].column('roll_sp')
    flight.roll_setpoint = ([t - zero for t in times], [setpoint / 100.0 for setpoint in setpoints])
    times, commands = streams['flight.csv'].column('roll_cmd')
    flight.roll_command = ([t - zero for t in times], commands)
    fins = streams.get('fins.csv')
    if fins is not None:
        times, angles = fins.column('eleron_left')
        flight.fin = ([t - zero for t in times], angles)
    return flight


def _table(flights: list) -> str:
    """The milestones, one line per flight."""
    def cell(value: float | None, unit: str) -> str:
        return '-' if value is None else '%.1f %s' % (value, unit)
    lines = ['%-34s %10s %10s %10s %10s %8s %9s %7s' % ('flight', 'gliding', 'landing', 'done', 'apogee', 'peak',
                                                        'miss', 'in zone')]
    for flight in flights:
        found = flight.milestones()
        lines.append('%-34s %10s %10s %10s %10s %8s %9s %7s' % (
            flight.label[:34], cell(found['gliding_s'], 's'), cell(found['landing_s'], 's'), cell(found['done_s'], 's'),
            cell(found['apogee_m'], 'm'), cell(found['peak_g'], 'g'), cell(found['miss_m'], 'm'),
            '-' if found['inside'] is None else ('yes' if found['inside'] else 'no')))
    return '\n'.join(lines)


def figure(flights: list, title: str) -> object:
    """The seven-panel overlay, the ground track across the bottom row."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    colours = ('#c0392b', '#2471a3')
    panels = make_subplots(rows=4, cols=2, specs=[[{}, {}], [{}, {}], [{}, {}], [{'colspan': 2}, None]],
                           row_heights=[0.2, 0.2, 0.2, 0.4],
                           subplot_titles=('height above the pad (m)', 'boost |a| (g)', 'airspeed (m/s)',
                                           'roll setpoint (deg)', 'roll command (deg)', 'left eleron (deg)',
                                           'ground track (m from the pad) and the landing zone'))
    origin = next((flight.origin for flight in flights if flight.origin), None)
    if origin is not None:
        (latitude_top, longitude_left), (latitude_bottom, longitude_right) = _zone()
        east, north = _metres(origin, [(latitude_top, longitude_left), (latitude_top, longitude_right),
                                       (latitude_bottom, longitude_right), (latitude_bottom, longitude_left),
                                       (latitude_top, longitude_left)])
        panels.add_trace(go.Scatter(x=east, y=north, name='landing zone', fill='toself', mode='lines',
                                    line={'color': '#27ae60', 'width': 1}, fillcolor='rgba(39,174,96,0.15)'), 4, 1)
    for flight, colour in zip(flights, colours):
        common = {'line': {'color': colour, 'width': 1.4}, 'legendgroup': flight.label}
        series = ((flight.height, 1, 1), (flight.accel, 1, 2), (flight.airspeed, 2, 1), (flight.roll_setpoint, 2, 2),
                  (flight.roll_command, 3, 1), (flight.fin, 3, 2))
        for (times, values), row, column in series:
            panels.add_trace(go.Scatter(x=times, y=values, name=flight.label, showlegend=(row, column) == (1, 1),
                                        **common), row, column)
        if flight.origin is not None and origin is not None and flight.track[0]:
            east, north = _metres(origin, [flight.origin])
            panels.add_trace(go.Scatter(x=[x + east[0] for x in flight.track[0]],
                                        y=[y + north[0] for y in flight.track[1]], name=flight.label,
                                        showlegend=False, **common), 4, 1)
        for t, stage in flight.stages:
            if stage in ('setting',):
                continue
            panels.add_vline(x=t, line={'color': colour, 'width': 0.8, 'dash': 'dot'}, row=1, col=1,
                             annotation_text=stage, annotation_font={'size': 9, 'color': colour},
                             annotation_position='top right')
    for row, column in ((1, 1), (1, 2), (2, 1), (2, 2), (3, 1), (3, 2)):
        panels.update_xaxes(title_text='s from BOOSTING entry', row=row, col=column)
    panels.update_xaxes(title_text='east (m)', row=4, col=1)
    panels.update_yaxes(title_text='north (m)', scaleanchor='x7', scaleratio=1, row=4, col=1)
    panels.update_layout(title=title, height=1500, width=1300, template='plotly_white',
                         legend={'orientation': 'h', 'y': -0.04})
    return panels


def main() -> int:
    """Build the overlay; 0 on success."""
    parser = argparse.ArgumentParser(description='Overlay a Rust HITL flight on a plan B flight')
    parser.add_argument('rust', help="the Rust flight's launch folder (recorder_flight.py -o)")
    parser.add_argument('planb', help='a plan B capture (virtual_flight.py -o, or a board HITL capture)')
    parser.add_argument('-o', '--out', required=True, help='output prefix: <out>.html and <out>.svg')
    parser.add_argument('--label-rust', default='Rust board HITL')
    parser.add_argument('--label-planb', default='plan B')
    parser.add_argument('--title', default='Rust HITL vs plan B')
    args = parser.parse_args()
    flights = [rust_flight(args.rust, args.label_rust), planb_flight(args.planb, args.label_planb)]
    table = _table(flights)
    print(table)
    panels = figure(flights, args.title)
    panels.write_html(args.out + '.html', include_plotlyjs='cdn')
    try:
        panels.write_image(args.out + '.svg')
    except Exception as error:  # kaleido missing or its browser absent: the HTML is the figure
        print('no SVG: %s' % error)
    print('wrote %s.html' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
