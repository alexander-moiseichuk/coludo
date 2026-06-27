# tools/flight_video.py -- render a narrated top-down animation of one or more flight captures to an mp4:
# a green field with four trees at the landing-zone corners, the glider tracking its real trajectory, a
# live telemetry panel (t / speed / accel / height / stage), an altitude bar, and prompter-style captions
# for the key events (on-the-rod -> ignition -> climb -> apogee/eject -> glide -> touchdown). Pure PIL
# frames piped to ffmpeg, FHD 1920x1080 @ 50 fps.
# Usage: flight_video.py <out.mp4> <LABEL> <capture.txt> [<LABEL> <capture.txt>] ...

import math
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_telemetry  # noqa: E402

_M = 111320.0
_PAD = (25.514379, -80.391795)
_TL, _BR = (25.514944, -80.392972), (25.514583, -80.391111)
_COSLAT = math.cos(math.radians(_PAD[0]))
_W, _H, _FPS = 1920, 1080, 50
_GREEN, _ZONE, _SKY = (74, 124, 60), (96, 150, 78), (26, 30, 38)
_FONTS = '/usr/share/fonts/truetype/dejavu/'
# layout (px)
_MAP = (60, 110, 1180, 980)          # map pane x0,y0,x1,y1
_BAR = (1200, 130, 1250, 960)        # altitude bar
_PANEL_X = 1300
_CAP_Y = 992


def _font(name, size):
    try:
        return ImageFont.truetype(_FONTS + name, size)
    except OSError:
        return ImageFont.load_default()


F_HEAD = _font('DejaVuSans-Bold.ttf', 40)
F_TITLE = _font('DejaVuSans-Bold.ttf', 34)
F_BIG = _font('DejaVuSans-Bold.ttf', 46)
F_NUM = _font('DejaVuSansMono-Bold.ttf', 50)
F_LBL = _font('DejaVuSans.ttf', 26)


def _to_m(lat, lon):
    return ((lon - _PAD[1]) * _M * _COSLAT, (lat - _PAD[0]) * _M)  # east, north metres from pad


def _at(times, values, t):
    """Interpolate `values` (scalars or (a, b) tuples) at time `t`, clamped to the ends; None if empty."""
    if not times:
        return None
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    lo, hi = 0, len(times) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= t:
            lo = mid
        else:
            hi = mid
    f = (t - times[lo]) / ((times[hi] - times[lo]) or 1)
    a, b = values[lo], values[hi]
    if isinstance(a, tuple):
        return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
    return a + (b - a) * f


def load(label, path):
    """Parse a capture into the series + key metrics the renderer needs, re-based to launch t=0."""
    streams, logs = flight_telemetry.parse(open(path).read())
    stages = [(us / 1e6, line.split('stage -> ')[1].split()[0]) for us, line in logs
              if us and 'stage -> ' in line]
    launch = next((t for t, s in stages if s == 'boosting'), 0.0)
    gnss, baro, accel = streams.get('gnss.csv'), streams.get('baro_icp10111.csv'), streams.get('accel_adxl375.csv')

    def rel(stream, field):
        if stream is None or field not in stream.fields:
            return [], []
        ts, vs = stream.column(field)
        return [t - launch for t in ts], vs

    lat_t, lat = rel(gnss, 'lat')
    _, lon = rel(gnss, 'lon')
    spd_t, knots = rel(gnss, 'speed_kn')
    speed = (spd_t, [k / 1.94384 for k in knots])
    hgt = rel(baro, 'elevation') if (baro and 'elevation' in baro.fields) else rel(baro, 'altitude')
    ax_t, ax = rel(accel, 'ax')
    _, ay = rel(accel, 'ay')
    _, az = rel(accel, 'az')
    amag = (ax_t, [(ax[i] ** 2 + ay[i] ** 2 + az[i] ** 2) ** 0.5 for i in range(len(ax))])
    track = [_to_m(lat[i], lon[i]) for i in range(len(lat))]
    pos = (lat_t, track)
    apogee = max(zip(hgt[0], hgt[1]), key=lambda p: p[1]) if hgt[0] else (0.0, 0.0)
    land_t = next((t for t, s in stages if s == 'landing'), apogee[0])
    done_t = next((t for t, s in stages if s == 'done'), hgt[0][-1] if hgt[0] else land_t)
    centre = ((_TL[0] + _BR[0]) / 2, (_TL[1] + _BR[1]) / 2)
    td, cm = (track[-1] if track else (0.0, 0.0)), _to_m(*centre)
    miss = math.hypot(td[0] - cm[0], td[1] - cm[1])
    in_zone = bool(track) and (_BR[0] <= lat[-1] <= _TL[0]) and (_TL[1] <= lon[-1] <= _BR[1])
    rod = next((speed[1][i] for i in range(len(speed[0])) if speed[1][i] > 13), 15.0)
    return {'label': label, 'motor': label.split()[0], 'pos': pos, 'speed': speed, 'height': hgt,
            'amag': amag, 'stages': stages, 'launch': launch, 'end': max(done_t, land_t) + 2.0,
            'apogee': apogee, 'land_t': land_t, 'miss': miss, 'in_zone': in_zone, 'rod_speed': rod,
            'rod_g': _at(amag[0], amag[1], 0.4) or 3.5, 'track': track}


def captions(fl):
    """The prompter timeline: list of (t_start, t_end, big_text)."""
    motor, ap, lt = fl['motor'], fl['apogee'], fl['land_t']
    return [
        (-3.0, 0.0, 'On the rod: TMS-7 glider standing vertical, loaded with %s.' % motor),
        (0.0, 5.0, 'IGNITION!  off the rod at ~%.0f m/s, ~%.1f g.' % (fl['rod_speed'], fl['rod_g'])),
        (5.0, ap[0], 'Powered climb -- speed and altitude building (live readout, right).'),
        (ap[0], ap[0] + 5, 'Apogee ~%.0f m at t=%.0f s -- booster ejects, glider deploys.' % (ap[1], ap[0])),
        (ap[0] + 5, lt, 'Gliding home -- banking to turn back toward the landing zone.'),
        (lt, fl['end'], 'Touchdown t=%.0f s -- %.0f m from centre -- %s' % (
            lt, fl['miss'], 'IN THE ZONE' if fl['in_zone'] else 'just outside the zone')),
    ]


def bounds(flights):
    es, ns = [0.0], [0.0]
    for la, lo in (_TL, _BR):
        e, n = _to_m(la, lo)
        es.append(e)
        ns.append(n)
    for fl in flights:
        for e, n in fl['track']:
            es.append(e)
            ns.append(n)
    me, mn = (max(es) - min(es)) * 0.12 + 15, (max(ns) - min(ns)) * 0.12 + 15
    return min(es) - me, max(es) + me, min(ns) - mn, max(ns) + mn


class Scene:
    """Maps metres -> the left map pane; draws zone, trees, pad, glider, trail."""

    def __init__(self, b):
        emin, emax, nmin, nmax = b
        self.s = min((_MAP[2] - _MAP[0]) / (emax - emin), (_MAP[3] - _MAP[1]) / (nmax - nmin))
        self.ec, self.nc = (emin + emax) / 2, (nmin + nmax) / 2

    def px(self, e, n):
        return ((_MAP[0] + _MAP[2]) / 2 + (e - self.ec) * self.s,
                (_MAP[1] + _MAP[3]) / 2 - (n - self.nc) * self.s)

    def tree(self, d, e, n):
        x, y = self.px(e, n)
        d.rectangle([x - 4, y, x + 4, y + 20], fill=(92, 62, 36))
        d.ellipse([x - 22, y - 32, x + 22, y - 2], fill=(34, 78, 36))
        d.ellipse([x - 15, y - 40, x + 15, y - 12], fill=(46, 104, 50))


def render(flights, out):
    sc = Scene(bounds(flights))
    corners = [(_TL[0], _TL[1]), (_TL[0], _BR[1]), (_BR[0], _BR[1]), (_BR[0], _TL[1])]
    zc = [sc.px(*_to_m(la, lo)) for la, lo in corners]
    trees = [_to_m(la, lo) for la, lo in corners]
    pad = sc.px(0, 0)
    cm = sc.px(*_to_m((_TL[0] + _BR[0]) / 2, (_TL[1] + _BR[1]) / 2))
    cap = max(fl['apogee'][1] for fl in flights) * 1.1 + 1
    proc = subprocess.Popen(
        ['ffmpeg', '-y', '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '%dx%d' % (_W, _H),
         '-framerate', str(_FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', out],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for fl in flights:
        cards = captions(fl)
        for f in range(int((fl['end'] + 3.0) * _FPS)):
            t = -3.0 + f / _FPS
            img = Image.new('RGB', (_W, _H), _GREEN)
            d = ImageDraw.Draw(img)
            d.rectangle([0, 0, _W, 84], fill=_SKY)
            d.rectangle([0, _CAP_Y - 8, _W, _H], fill=_SKY)
            d.rectangle([_MAP[0], _MAP[1], _MAP[2], _MAP[3]], outline=(52, 92, 46))
            d.polygon(zc, fill=_ZONE, outline=(225, 225, 130))
            d.line([cm[0] - 10, cm[1], cm[0] + 10, cm[1]], fill=(235, 235, 150), width=2)
            d.line([cm[0], cm[1] - 10, cm[0], cm[1] + 10], fill=(235, 235, 150), width=2)
            for e, n in trees:
                sc.tree(d, e, n)
            d.ellipse([pad[0] - 7, pad[1] - 7, pad[0] + 7, pad[1] + 7], fill=(205, 60, 50))
            d.text((pad[0] + 12, pad[1] - 8), 'pad', font=F_LBL, fill=(235, 205, 185))
            trail = [sc.px(*p) for tt, p in zip(fl['pos'][0], fl['pos'][1]) if tt <= t]
            if len(trail) > 1:
                d.line(trail, fill=(238, 238, 248), width=3)
            if t >= 0:
                cur = _at(fl['pos'][0], fl['pos'][1], t)
                if cur is not None:
                    gx, gy = sc.px(*cur)
                    d.ellipse([gx - 9, gy - 9, gx + 9, gy + 9], fill=(255, 222, 40), outline=(20, 20, 20))
            height = max(0.0, _at(fl['height'][0], fl['height'][1], t) or 0.0) if t >= 0 else 0.0
            spd = max(0.0, _at(fl['speed'][0], fl['speed'][1], t) or 0.0) if t >= 0 else 0.0
            acc = (_at(fl['amag'][0], fl['amag'][1], t) or 1.0) if t >= 0 else 1.0
            stage = 'on the rod'
            for st, s in fl['stages']:
                if st - fl['launch'] <= t:
                    stage = s
            d.rectangle([_BAR[0], _BAR[1], _BAR[2], _BAR[3]], outline=(120, 150, 120))
            fillh = _BAR[3] - (_BAR[3] - _BAR[1]) * min(1.0, height / cap)
            if fillh < _BAR[3] - 2:
                d.rectangle([_BAR[0] + 2, fillh, _BAR[2] - 2, _BAR[3] - 2], fill=(90, 165, 225))
            d.text((_BAR[0] - 6, _BAR[1] - 34), 'alt', font=F_LBL, fill=(170, 200, 170))
            d.text((_PANEL_X, 120), '%s   -   5%% noise, calm' % fl['label'], font=F_TITLE, fill=(245, 245, 185))
            for i, (lab, val) in enumerate([('time', '%5.1f s' % max(0.0, t)), ('speed', '%5.1f m/s' % spd),
                                            ('accel', '%5.1f g' % acc), ('height', '%5.0f m' % height)]):
                y = 200 + i * 110
                d.text((_PANEL_X, y), lab, font=F_LBL, fill=(170, 205, 170))
                d.text((_PANEL_X, y + 28), val, font=F_NUM, fill=(255, 255, 255))
            d.text((_PANEL_X, 680), 'stage', font=F_LBL, fill=(170, 205, 170))
            d.text((_PANEL_X, 712), stage.upper(), font=F_BIG, fill=(255, 212, 120))
            d.text((30, 18), 'Coludo TMS-7  --  guarded-fins HITL simulation', font=F_HEAD, fill=(232, 232, 242))
            d.text((40, _CAP_Y + 24), next((c[2] for c in cards if c[0] <= t < c[1]), ''),
                   font=F_BIG, fill=(255, 255, 255))
            proc.stdin.write(img.tobytes())
    proc.stdin.close()
    proc.wait()


if __name__ == '__main__':
    out = sys.argv[1]
    flights = [load(sys.argv[i], sys.argv[i + 1]) for i in range(2, len(sys.argv), 2)]
    render(flights, out)
    print('wrote', out, '(%d flights)' % len(flights))
