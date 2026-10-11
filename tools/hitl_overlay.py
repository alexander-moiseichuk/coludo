#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Overlay a Rust board's HITL flight on a plan B flight of the same scenario, both aligned on their own BOOSTING entry
(the stage machine's launch detection), so the two stage machines and their timelines compare directly:

  * height above the pad (the ICP-10111 stream's elevation), each flight's stage changes marked;
  * the boost's |a| (the LSM6DSO32 stream), the first seconds;
  * airspeed (the SDP810 stream);
  * the ground track in metres from the pad.

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

    def milestones(self) -> dict:
        """When each stage began, the apogee height and the peak |a|."""
        found = {name: t for t, name in reversed(self.stages)}
        heights = self.height[1]
        return {'gliding_s': found.get('gliding'), 'landing_s': found.get('landing'), 'done_s': found.get('done'),
                'apogee_m': max(heights) if heights else None,
                'peak_g': max(self.accel[1]) if self.accel[1] else None}


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


def _track(points: list) -> tuple:
    """(east, north) metres from the first fix, equirectangular at its latitude."""
    if not points:
        return [], []
    latitude0, longitude0 = points[0]
    scale = math.cos(math.radians(latitude0))
    return ([(longitude - longitude0) * _METRES_PER_DEGREE * scale for _, longitude in points],
            [(latitude - latitude0) * _METRES_PER_DEGREE for latitude, _ in points])


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
    fixes = _rust_column(folder, 'gnss', 'lat', 'lon')
    flight.track = _track([(latitude, longitude) for _, latitude, longitude in fixes if latitude is not None])
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
        flight.track = _track(list(zip(gnss.column('lat')[1], gnss.column('lon')[1])))
    return flight


def _table(flights: list) -> str:
    """The milestones, one line per flight."""
    def cell(value: float | None, unit: str) -> str:
        return '-' if value is None else '%.1f %s' % (value, unit)
    lines = ['%-34s %10s %10s %10s %10s %8s' % ('flight', 'gliding', 'landing', 'done', 'apogee', 'peak')]
    for flight in flights:
        found = flight.milestones()
        lines.append('%-34s %10s %10s %10s %10s %8s' % (
            flight.label[:34], cell(found['gliding_s'], 's'), cell(found['landing_s'], 's'), cell(found['done_s'], 's'),
            cell(found['apogee_m'], 'm'), cell(found['peak_g'], 'g')))
    return '\n'.join(lines)


def figure(flights: list, title: str) -> object:
    """The four-panel overlay."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    colours = ('#c0392b', '#2471a3')
    panels = make_subplots(rows=2, cols=2, subplot_titles=('height above the pad (m)', 'boost |a| (g)',
                                                           'airspeed (m/s)', 'ground track (m from the pad)'))
    for flight, colour in zip(flights, colours):
        common = {'line': {'color': colour, 'width': 1.4}, 'legendgroup': flight.label}
        panels.add_trace(go.Scatter(x=flight.height[0], y=flight.height[1], name=flight.label, **common), 1, 1)
        panels.add_trace(go.Scatter(x=flight.accel[0], y=flight.accel[1], name=flight.label, showlegend=False,
                                    **common), 1, 2)
        panels.add_trace(go.Scatter(x=flight.airspeed[0], y=flight.airspeed[1], name=flight.label,
                                    showlegend=False, **common), 2, 1)
        panels.add_trace(go.Scatter(x=flight.track[0], y=flight.track[1], name=flight.label, showlegend=False,
                                    **common), 2, 2)
        for t, stage in flight.stages:
            if stage in ('setting',):
                continue
            panels.add_vline(x=t, line={'color': colour, 'width': 0.8, 'dash': 'dot'}, row=1, col=1,
                             annotation_text=stage, annotation_font={'size': 9, 'color': colour},
                             annotation_position='top right')
    panels.update_xaxes(title_text='s from BOOSTING entry', row=1, col=1)
    panels.update_xaxes(title_text='s from BOOSTING entry', row=1, col=2)
    panels.update_xaxes(title_text='s from BOOSTING entry', row=2, col=1)
    panels.update_xaxes(title_text='east (m)', row=2, col=2)
    panels.update_yaxes(title_text='north (m)', scaleanchor='x4', scaleratio=1, row=2, col=2)
    panels.update_layout(title=title, height=820, width=1300, template='plotly_white',
                         legend={'orientation': 'h', 'y': -0.08})
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
