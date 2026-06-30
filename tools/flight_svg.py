# flight_svg.py — render a Coludo flight capture as a standalone SVG (no dependencies), for when plotly
# (flight_report.py) is not available. Two modes:
#   single  : python3 flight_svg.py capture.txt -o report.svg [--title T] [--pad LAT,LON] [--zone ...]
#             a two-panel report -- plan-view ground track (left) + altitude & roll vs time (right).
#   overlay : python3 flight_svg.py a.txt b.txt c.txt --overlay -o cmp.svg --labels "5%,10%,25%"
#             every track on one plan view, to compare runs.
# Reuses flight_telemetry.parse(), so it keys on the same field names the recorder writes. Pad/zone are
# not in a capture; pass --pad/--zone to draw them (defaults: origin = first GNSS fix, no zone box).

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_telemetry  # noqa: E402

_PALETTE = ('#1f77b4', '#17becf', '#2ca02c', '#bcbd22', '#ff7f0e', '#d62728', '#9467bd', '#8c564b')


def _find(streams, *fields):
    """The first stream carrying all the given fields (e.g. 'lat','lon' -> the GNSS stream)."""
    for stream in streams.values():
        if all(field in stream.fields for field in fields):
            return stream
    return None


def _track(streams):
    """(lat, lon) fixes from the GNSS stream."""
    gnss = _find(streams, 'lat', 'lon')
    if gnss is None:
        return []
    _t, lat = gnss.column('lat')
    _t, lon = gnss.column('lon')
    return list(zip(lat, lon))


def _enu(lat, lon, lat0, lon0):
    return ((lon - lon0) * 111320.0 * math.cos(math.radians(lat0)), (lat - lat0) * 111320.0)


def _polyline(points, fx, fy, colour, width=1.7):
    d = ' '.join('%s%.1f,%.1f' % ('M' if i == 0 else 'L', fx(x), fy(y)) for i, (x, y) in enumerate(points))
    return '<path d="%s" fill="none" stroke="%s" stroke-width="%.1f" opacity="0.85"/>' % (d, colour, width)


def _plan(streams_list, labels, pad, zone, box):
    """Plan-view (ground-track) SVG fragment for one or more flights, fitted into `box` (x0,y0,w,h)."""
    x0, y0, w, h = box
    tracks = [_track(s) for s in streams_list]
    lat0, lon0 = pad if pad else (tracks[0][0] if tracks and tracks[0] else (0.0, 0.0))
    enu = [[_enu(la, lo, lat0, lon0) for la, lo in t] for t in tracks]
    pts = [p for t in enu for p in t] + [(0.0, 0.0)]
    if zone:
        pts += [_enu(zone[0], zone[1], lat0, lon0), _enu(zone[2], zone[3], lat0, lon0)]
    xs = [p[0] for p in pts] or [0.0]
    ys = [p[1] for p in pts] or [0.0]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    span = max(maxx - minx, maxy - miny, 1.0)
    pad_px = 30
    scale = (min(w, h) - 2 * pad_px) / span
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    fx = lambda x: x0 + w / 2 + (x - cx) * scale            # noqa: E731  (east right)
    fy = lambda y: y0 + h / 2 - (y - cy) * scale            # noqa: E731  (north up)
    out = ['<rect x="%d" y="%d" width="%d" height="%d" fill="#fbfbfb" stroke="#ddd"/>' % (x0, y0, w, h)]
    if zone:
        zx0, zy0 = _enu(zone[0], zone[1], lat0, lon0)
        zx1, zy1 = _enu(zone[2], zone[3], lat0, lon0)
        out.append('<rect x="%.0f" y="%.0f" width="%.0f" height="%.0f" fill="#cde" stroke="#369" '
                   'opacity="0.7"/>' % (min(fx(zx0), fx(zx1)), min(fy(zy0), fy(zy1)),
                                        abs(fx(zx1) - fx(zx0)) or 3, abs(fy(zy1) - fy(zy0)) or 3))
        out.append('<text x="%.0f" y="%.0f" fill="#369" font-size="12">landing zone</text>'
                   % (min(fx(zx0), fx(zx1)), min(fy(zy0), fy(zy1)) - 5))
    out.append('<circle cx="%.0f" cy="%.0f" r="5" fill="black"/>'
               '<text x="%.0f" y="%.0f" font-size="12">pad</text>' % (fx(0), fy(0), fx(0) + 8, fy(0) + 4))
    for i, track in enumerate(enu):
        if not track:
            continue
        colour = _PALETTE[i % len(_PALETTE)]
        out.append(_polyline(track, fx, fy, colour))
        ex, ey = track[-1]
        out.append('<circle cx="%.0f" cy="%.0f" r="4.5" fill="%s"/>' % (fx(ex), fy(ey), colour))
        if labels:
            out.append('<rect x="%d" y="%d" width="13" height="13" fill="%s"/>'
                       '<text x="%d" y="%d" font-size="12">%s</text>'
                       % (x0 + 8, y0 + 8 + i * 20, colour, x0 + 26, y0 + 19 + i * 20, labels[i]))
    out.append('<text x="%d" y="%d" font-size="11" fill="#888">ground track — m from pad (N up)</text>'
               % (x0 + 8, y0 + h - 8))
    return ''.join(out)


def _timeseries(streams, box):
    """Altitude (blue, left axis) + roll (orange, right axis) vs time, fitted into `box`."""
    x0, y0, w, h = box
    baro = _find(streams, 'altitude')
    imu = _find(streams, 'roll')
    at, av = baro.column('altitude') if baro else ([], [])
    rt, rv = imu.column('roll') if imu else ([], [])
    out = ['<rect x="%d" y="%d" width="%d" height="%d" fill="#fbfbfb" stroke="#ddd"/>' % (x0, y0, w, h)]
    if not at and not rt:
        return ''.join(out)
    tmax = max((at[-1] if at else 0), (rt[-1] if rt else 0), 1.0)
    pad_px = 36
    fx = lambda t: x0 + pad_px + t / tmax * (w - 2 * pad_px)   # noqa: E731

    def axis(values, colour, label, right):
        if not values:
            return ''
        lo, hi = min(values), max(values)
        span = (hi - lo) or 1.0
        fy = lambda v: y0 + h - pad_px - (v - lo) / span * (h - 2 * pad_px)   # noqa: E731
        frag = ['<text x="%d" y="%d" fill="%s" font-size="11">%s %.0f</text>'
                % (x0 + (w - 80 if right else 6), y0 + 14, colour, label, hi),
                '<text x="%d" y="%d" fill="%s" font-size="11">%.0f</text>'
                % (x0 + (w - 30 if right else 6), y0 + h - 22, colour, lo)]
        return ''.join(frag), fy

    a_axis = axis(av, '#1f77b4', 'alt', False)
    r_axis = axis(rv, '#ff7f0e', 'roll', True)
    if a_axis:
        out.append(a_axis[0])
        out.append(_polyline(list(zip(at, av)), fx, a_axis[1], '#1f77b4', 1.4))
    if r_axis:
        out.append(r_axis[0])
        out.append(_polyline(list(zip(rt, rv)), fx, r_axis[1], '#ff7f0e', 1.1))
    out.append('<text x="%d" y="%d" font-size="11" fill="#888">time 0..%.0f s — altitude (blue) &amp; roll '
               '(orange)</text>' % (x0 + pad_px, y0 + h - 8, tmax))
    return ''.join(out)


def _svg(width, height, title, body):
    return ('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" font-family="sans-serif" '
            'font-size="13"><rect width="%d" height="%d" fill="white"/>'
            '<text x="20" y="26" font-size="15" fill="#333">%s</text>%s</svg>'
            % (width, height, width, height, title, body))


def main():
    parser = argparse.ArgumentParser(description='Render a Coludo capture as a dependency-free SVG.')
    parser.add_argument('captures', nargs='+', help='capture file(s)')
    parser.add_argument('-o', '--out', required=True, help='output SVG')
    parser.add_argument('--overlay', action='store_true', help='overlay all captures on one plan view')
    parser.add_argument('--title', default='Coludo flight', help='figure title')
    parser.add_argument('--labels', help='comma-separated legend labels (overlay mode)')
    parser.add_argument('--pad', help='pad LAT,LON (draws the pad origin)')
    parser.add_argument('--zone', help='zone TL_LAT,TL_LON,BR_LAT,BR_LON (draws the landing box)')
    args = parser.parse_args()

    pad = tuple(float(v) for v in args.pad.split(',')) if args.pad else None
    zone = tuple(float(v) for v in args.zone.split(',')) if args.zone else None
    streams_list = []
    for c in args.captures:
        with open(c) as handle:
            streams_list.append(flight_telemetry.parse(handle.read())[0])

    if args.overlay or len(streams_list) > 1:
        labels = args.labels.split(',') if args.labels else [os.path.basename(c) for c in args.captures]
        body = _plan(streams_list, labels, pad, zone, (20, 40, 800, 800))
        svg = _svg(840, 860, args.title, body)
    else:
        body = (_plan(streams_list, None, pad, zone, (20, 40, 540, 540))
                + _timeseries(streams_list[0], (580, 40, 540, 540)))
        svg = _svg(1140, 600, args.title, body)
    with open(args.out, 'w') as handle:
        handle.write(svg)
    sys.stderr.write('wrote %s\n' % args.out)


if __name__ == '__main__':
    main()
