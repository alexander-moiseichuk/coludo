#!/usr/bin/env python3
# Coludo project, copyright under MIT license, Alexander Moiseichuk
"""
Draw one recorded flight as a README "flight card": an SVG with the ground track seen from above (metres from the
pad, coloured by stage, over the landing zone) beside the altitude over time. No coordinates appear: the track is
drawn relative to the pad.

The flight is a Rust cut (`<dir>/flight/`: gnss.csv `t_s;lat;lon`, baro_icp10111.csv `t_s;elevation`, sequencer.csv
`t_s;stage`), as `tools/hitl_host.py --sitl ... --record <dir>` writes it; the pad and the zone are the HITL scenario's
(src/glider sim_model.HPRC).

  tools/render_flight_card.py <dir> --title 'A simulated TMS-8 flight' --wind 3 --wind-dir 90 --out doc/photos/x.svg
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))
import sim_model  # noqa: E402

_METRES_PER_DEGREE: float = 111_320.0
_COLOURS: dict = {'boosting': '#fb923c', 'gliding': '#38bdf8', 'landing': '#a3e635', 'done': '#a3e635'}
_LABELS: dict = {'boosting': 'boost', 'gliding': 'glide', 'landing': 'land'}
_SMOOTH_S: float = 0.5  # the altitude's moving average, seconds: the recorded baro carries the sim's noise


def rows(path: str) -> list:
    """A ';' CSV as a list of dicts."""
    with open(path, newline='') as handle:
        return list(csv.DictReader(handle, delimiter=';'))


def stage_at(stages: list, time_s: float) -> str:
    """The stage the sequencer was in at `time_s`."""
    current = stages[0][1]
    for start, stage in stages:
        if start <= time_s:
            current = stage
    return current


def segments(points: list, stages: list) -> list:
    """The points split into runs of one stage each, [(stage, [(x, y), ...])], each run joined to the next."""
    runs: list = []
    for time_s, x, y in points:
        stage = stage_at(stages, time_s)
        if runs and runs[-1][0] == stage:
            runs[-1][1].append((x, y))
        else:
            if runs:
                runs[-1][1].append((x, y))
            runs.append((stage, [(x, y)]))
    return runs


def polyline(points: list, colour: str, width: float) -> str:
    """An SVG polyline."""
    text = ' '.join('%.1f,%.1f' % point for point in points)
    return '<polyline points="%s" fill="none" stroke="%s" stroke-width="%.1f" stroke-linejoin="round" ' \
           'stroke-linecap="round"/>' % (text, colour, width)


def card(cut: str, title: str, subtitle: str, wind_ms: float, wind_toward_deg: float) -> str:
    """The SVG text."""
    gnss, baro = rows(os.path.join(cut, 'gnss.csv')), rows(os.path.join(cut, 'baro_icp10111.csv'))
    stages = [(float(row['t_s']), row['stage']) for row in rows(os.path.join(cut, 'sequencer.csv'))]
    start = next(time_s for time_s, stage in stages if stage == 'boosting')
    end = next((time_s for time_s, stage in stages if stage == 'done'), float(gnss[-1]['t_s']))
    pad_latitude, pad_longitude = sim_model.HPRC['launch']
    east_per_degree = _METRES_PER_DEGREE * math.cos(math.radians(pad_latitude))

    def metres(latitude: float, longitude: float) -> tuple:
        return (longitude - pad_longitude) * east_per_degree, (latitude - pad_latitude) * _METRES_PER_DEGREE
    flown = [(float(row['t_s']), *metres(float(row['lat']), float(row['lon']))) for row in gnss
             if start <= float(row['t_s']) <= end]
    flown = flown[::4] + flown[-1:]  # 12.5 Hz is plenty at this scale
    (top, left), (bottom, right) = sim_model.HPRC['zone']
    zone = [metres(top, left), metres(bottom, right)]
    # the altitude, smoothed over _SMOOTH_S
    raw = [(float(row['t_s']), float(row['elevation'])) for row in baro if start <= float(row['t_s']) <= end]
    window = max(1, int(_SMOOTH_S / max(1e-3, raw[1][0] - raw[0][0])))
    altitude = [(raw[index][0], sum(value for _, value in raw[max(0, index - window):index + 1])
                 / len(raw[max(0, index - window):index + 1])) for index in range(0, len(raw), 4)]
    apogee_time, apogee = max(altitude, key=lambda sample: sample[1])

    width, height = 920, 400
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="%d" height="%d" '
             'font-family="system-ui,-apple-system,Segoe UI,Helvetica,Arial,sans-serif">' % (width, height, width,
                                                                                             height),
             '<rect width="%d" height="%d" rx="18" fill="#0b1220"/>' % (width, height),
             '<text x="28" y="40" fill="#f8fafc" font-size="21" font-weight="700">%s</text>' % title,
             '<text x="28" y="64" fill="#94a3b8" font-size="13.5">%s</text>' % subtitle]

    # the map: metres east/north, fitted into its panel with the zone and the pad in view
    panel_x, panel_y, panel_w, panel_h = 28, 86, 380, 290
    xs = [x for _, x, _ in flown] + [zone[0][0], zone[1][0], 0.0]
    ys = [y for _, _, y in flown] + [zone[0][1], zone[1][1], 0.0]
    span = max(max(xs) - min(xs), (max(ys) - min(ys)) * panel_w / panel_h) * 1.12
    centre_x, centre_y = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    scale = panel_w / span

    def to_map(x: float, y: float) -> tuple:
        return panel_x + panel_w / 2.0 + (x - centre_x) * scale, panel_y + panel_h / 2.0 - (y - centre_y) * scale
    parts.append('<rect x="%d" y="%d" width="%d" height="%d" rx="12" fill="#111a2e" stroke="#1e293b"/>'
                 % (panel_x, panel_y, panel_w, panel_h))
    (zone_left, zone_top), (zone_right, zone_bottom) = to_map(*zone[0]), to_map(*zone[1])
    parts.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="4" fill="#123524" stroke="#22c55e" '
                 'stroke-dasharray="5 4"/>' % (zone_left, zone_top, zone_right - zone_left, zone_bottom - zone_top))
    parts.append('<text x="%.1f" y="%.1f" fill="#4ade80" font-size="12">landing zone</text>'
                 % (zone_left + 6, zone_top - 6))
    for stage, run in segments([(time_s, *to_map(x, y)) for time_s, x, y in flown], stages):
        parts.append(polyline(run, _COLOURS.get(stage, '#e2e8f0'), 2.6))
    pad_x, pad_y = to_map(0.0, 0.0)
    parts.append('<circle cx="%.1f" cy="%.1f" r="5.5" fill="#f8fafc"/>' % (pad_x, pad_y))
    parts.append('<text x="%.1f" y="%.1f" fill="#e2e8f0" font-size="12">pad</text>' % (pad_x + 9, pad_y + 4))
    land_x, land_y = to_map(*flown[-1][1:])
    parts.append('<circle cx="%.1f" cy="%.1f" r="6" fill="none" stroke="#a3e635" stroke-width="2.5"/>'
                 % (land_x, land_y))
    # a 50 m scale bar, north, and the wind
    bar = 50.0 * scale
    base_x, base_y = panel_x + 18, panel_y + panel_h - 18
    parts.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#cbd5e1" stroke-width="2"/>'
                 % (base_x, base_y, base_x + bar, base_y))
    parts.append('<text x="%.1f" y="%.1f" fill="#cbd5e1" font-size="11.5">50 m</text>' % (base_x, base_y - 7))
    north_x, north_y = panel_x + panel_w - 26, panel_y + 34
    parts.append('<path d="M %.1f %.1f l -6 14 l 6 -4 l 6 4 z" fill="#cbd5e1"/>' % (north_x, north_y - 16))
    parts.append('<text x="%.1f" y="%.1f" fill="#cbd5e1" font-size="11.5" text-anchor="middle">N</text>'
                 % (north_x, north_y + 12))
    if wind_ms > 0:
        # an arrow the way the air moves, in the panel's bottom-right corner; plain shapes only (no markers, no
        # opacity, no gradients: not every SVG renderer GitHub or a viewer uses draws them)
        east, north = math.sin(math.radians(wind_toward_deg)), math.cos(math.radians(wind_toward_deg))
        middle_x, middle_y = panel_x + panel_w - 52, panel_y + panel_h - 34
        tip_x, tip_y = middle_x + 14 * east, middle_y - 14 * north
        tail_x, tail_y = middle_x - 14 * east, middle_y + 14 * north
        parts.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e2e8f0" stroke-width="2.2"/>'
                     % (tail_x, tail_y, tip_x, tip_y))
        head = [(tip_x + 7 * east, tip_y - 7 * north), (tip_x - 5 * north, tip_y - 5 * east),
                (tip_x + 5 * north, tip_y + 5 * east)]
        parts.append('<polygon points="%s" fill="#e2e8f0"/>' % ' '.join('%.1f,%.1f' % point for point in head))
        parts.append('<text x="%.1f" y="%.1f" fill="#cbd5e1" font-size="11.5" text-anchor="middle">wind %g m/s'
                     '</text>' % (middle_x, middle_y + 30, wind_ms))

    # the altitude over the flight's time, from ignition
    plot_x, plot_y, plot_w, plot_h = 440, 86, 452, 290
    parts.append('<rect x="%d" y="%d" width="%d" height="%d" rx="12" fill="#111a2e" stroke="#1e293b"/>'
                 % (plot_x, plot_y, plot_w, plot_h))
    inner_x, inner_y, inner_w, inner_h = plot_x + 46, plot_y + 26, plot_w - 66, plot_h - 64
    duration = end - start
    top_m = math.ceil(apogee / 50.0) * 50.0

    def to_plot(time_s: float, metres_up: float) -> tuple:
        return (inner_x + (time_s - start) / duration * inner_w,
                inner_y + inner_h - max(0.0, metres_up) / top_m * inner_h)
    for level in range(0, int(top_m) + 1, 100):
        _, line_y = to_plot(start, level)
        parts.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#1e293b"/>'
                     % (inner_x, line_y, inner_x + inner_w, line_y))
        parts.append('<text x="%.1f" y="%.1f" fill="#64748b" font-size="11" text-anchor="end">%d m</text>'
                     % (inner_x - 6, line_y + 4, level))
    for second in range(0, int(duration) + 1, 20):
        tick_x, _ = to_plot(start + second, 0.0)
        parts.append('<text x="%.1f" y="%.1f" fill="#64748b" font-size="11" text-anchor="middle">%d s</text>'
                     % (tick_x, inner_y + inner_h + 18, second))
    area = [to_plot(start, 0.0)] + [to_plot(time_s, value) for time_s, value in altitude] + [to_plot(end, 0.0)]
    parts.append('<polygon points="%s" fill="#13304a"/>' % ' '.join('%.1f,%.1f' % point for point in area))
    for stage, run in segments([(time_s, *to_plot(time_s, value)) for time_s, value in altitude], stages):
        parts.append(polyline(run, _COLOURS.get(stage, '#e2e8f0'), 3.0))
    apogee_x, apogee_y = to_plot(apogee_time, apogee)
    parts.append('<circle cx="%.1f" cy="%.1f" r="4.5" fill="#f8fafc"/>' % (apogee_x, apogee_y))
    parts.append('<text x="%.1f" y="%.1f" fill="#f8fafc" font-size="12.5">apogee %d m</text>'
                 % (apogee_x + 9, apogee_y - 6, round(apogee)))
    # the legend: each stage's colour
    legend_x = inner_x + inner_w - 190
    for index, stage in enumerate(('boosting', 'gliding', 'landing')):
        parts.append('<rect x="%.1f" y="%.1f" width="14" height="4" rx="2" fill="%s"/>'
                     % (legend_x + index * 66, inner_y - 2, _COLOURS[stage]))
        parts.append('<text x="%.1f" y="%.1f" fill="#cbd5e1" font-size="11.5">%s</text>'
                     % (legend_x + index * 66 + 19, inner_y + 4, _LABELS[stage]))
    parts.append('</svg>')
    return '\n'.join(parts) + '\n'


def main() -> None:
    """Parse the arguments, draw, save."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('cut', help="a --record directory (its flight/ is read)")
    parser.add_argument('--out', required=True)
    parser.add_argument('--title', default='A simulated flight')
    parser.add_argument('--subtitle', default='')
    parser.add_argument('--wind', type=float, default=0.0, help='m/s, drawn as an arrow')
    parser.add_argument('--wind-dir', type=float, default=0.0, help='degrees the wind blows TOWARD')
    args = parser.parse_args()
    with open(args.out, 'w') as handle:
        handle.write(card(os.path.join(args.cut, 'flight'), args.title, args.subtitle, args.wind, args.wind_dir))


if __name__ == '__main__':
    main()
