# tools/flight_video.py -- render a narrated 3D-ish animation of one or more flight captures to an mp4.
# A fixed oblique camera frames the whole flight over a green field with the landing zone + four corner
# trees, so ALTITUDE shows: the glider climbs up the screen during boost (a flat top-down view made it
# look stuck on the rod). The glider is a small shaded 3D plane rotated by its real attitude
# (heading/pitch/roll) -- vertical with folded wings on the rod, then at separation the booster drops away
# under a parachute and the wings sweep out to 15deg (F-111 style) for the glide. The ground track is a
# smoothed dashed trail; the panel shows live telemetry + the event schedule with timings. Pure PIL frames
# piped to ffmpeg, FHD 1920x1080 @ 50 fps.
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
_GREEN, _ZONE, _SKY = (78, 128, 64), (98, 152, 80), (24, 28, 36)
_KH, _KN = 1.0, 0.55                  # oblique gains: altitude (vertical), north (depth)
_MAP = (44, 100, 1190, 968)           # map pane x0,y0,x1,y1
_PANEL_X = 1320
_CAP_Y = 990
_PS = 19.0                            # plane pixel scale (per body unit)
_BURN = {'F15': 3.45, 'E16': 1.77}
_LIGHT = (0.36, -0.45, 0.82)
_FONTS = '/usr/share/fonts/truetype/dejavu/'


def _font(name, size):
    try:
        return ImageFont.truetype(_FONTS + name, size)
    except OSError:
        return ImageFont.load_default()


F_HEAD = _font('DejaVuSans-Bold.ttf', 38)
F_TITLE = _font('DejaVuSans-Bold.ttf', 32)
F_BIG = _font('DejaVuSans-Bold.ttf', 42)
F_NUM = _font('DejaVuSansMono-Bold.ttf', 46)
F_LBL = _font('DejaVuSans.ttf', 25)
F_SCH = _font('DejaVuSansMono.ttf', 26)


def _to_m(lat, lon):
    return ((lon - _PAD[1]) * _M * _COSLAT, (lat - _PAD[0]) * _M)


def _at(times, values, t):
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


def _wxy(e, n, h):
    return (e, -(h * _KH + n * _KN))


def _smooth(xs, win=9):
    if len(xs) < 3:
        return xs
    half = win // 2
    return [sum(xs[max(0, i - half):min(len(xs), i + half + 1)]) /
            len(xs[max(0, i - half):min(len(xs), i + half + 1)]) for i in range(len(xs))]


# -- tiny 3D: body axes from attitude, vectors --
def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    m = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) ** 0.5 or 1.0
    return (a[0] / m, a[1] / m, a[2] / m)


def _axes(heading, pitch, roll):
    """Body axes (forward, right, up) in world ENU from heading(0=N,CW), pitch(90=nose up), roll(right+)."""
    th, ph, ro = math.radians(heading), math.radians(pitch), math.radians(roll)
    f = (math.cos(ph) * math.sin(th), math.cos(ph) * math.cos(th), math.sin(ph))
    r0 = _norm(_cross(f, (0.0, 0.0, 1.0))) if abs(ph) < math.radians(89.5) else (math.cos(th), -math.sin(th), 0.0)
    u0 = _cross(r0, f)
    cr, sr = math.cos(ro), math.sin(ro)
    r = (r0[0] * cr - u0[0] * sr, r0[1] * cr - u0[1] * sr, r0[2] * cr - u0[2] * sr)
    u = _cross(r, f)
    return f, r, u


# plane model: vertices (forward, right, up) and faces (i, j, k, base_colour). tipx set per wing sweep.
def _plane_faces(sweep_frac, wings):
    tx = 0.4 - sweep_frac * 1.5
    v = [(1.9, 0, 0), (-1.3, 0, 0), (0.4, 0, 0.05), (-0.5, 0, 0.05), (tx, -1.9, -0.05),
         (tx, 1.9, -0.05), (-1.25, 0, 0.7), (-1.05, -0.7, 0), (-1.05, 0.7, 0), (0.3, 0, -0.30)]
    body, wing, tail = (210, 205, 70), (200, 196, 64), (180, 176, 58)
    faces = [(3, 7, 1, tail), (3, 1, 8, tail), (3, 1, 6, tail),     # tailplane + fin (always)
             (0, 2, 3, body), (0, 3, 1, body), (0, 1, 9, body)]     # fuselage (a slim dart on the rod)
    if wings:                                                       # main wings only once deployed
        faces = [(2, 4, 3, wing), (2, 3, 5, wing)] + faces
    return v, faces


def draw_plane(d, cx, cy, heading, pitch, roll, sweep_frac, wings):
    f, r, u = _axes(heading, pitch, roll)
    verts, faces = _plane_faces(sweep_frac, wings)
    world = [(vx * f[0] + vy * r[0] + vz * u[0], vx * f[1] + vy * r[1] + vz * u[1],
              vx * f[2] + vy * r[2] + vz * u[2]) for vx, vy, vz in verts]
    scr = [(cx + _PS * w[0], cy - _PS * (w[2] * _KH + w[1] * _KN)) for w in world]
    drawn = []
    for i, j, k, col in faces:
        a, b, c = world[i], world[j], world[k]
        nrm = _norm(_cross((b[0] - a[0], b[1] - a[1], b[2] - a[2]),
                           (c[0] - a[0], c[1] - a[1], c[2] - a[2])))
        depth = (a[1] + b[1] + c[1]) / 3.0                  # north = into the screen
        sh = 0.42 + 0.58 * max(0.0, abs(nrm[0] * _LIGHT[0] + nrm[1] * _LIGHT[1] + nrm[2] * _LIGHT[2]))
        drawn.append((depth, [scr[i], scr[j], scr[k]], tuple(int(c2 * sh) for c2 in col)))
    for _, poly, col in sorted(drawn, key=lambda z: -z[0]):
        d.polygon(poly, fill=col, outline=(40, 38, 18))


def _dashed(d, pts, fill, width=3, dash=16, gap=12):
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        ux, uy = (x1 - x0) / seg, (y1 - y0) / seg
        s = -carry
        while s < seg:
            a, b = max(0.0, s), min(seg, s + dash)
            if b > 0:
                d.line([(x0 + ux * a, y0 + uy * a), (x0 + ux * b, y0 + uy * b)], fill=fill, width=width)
            s += dash + gap
        carry = (carry + seg) % (dash + gap)


def load(label, path):
    streams, logs = flight_telemetry.parse(open(path).read())
    stages = [(us / 1e6, line.split('stage -> ')[1].split()[0]) for us, line in logs
              if us and 'stage -> ' in line]
    launch = next((t for t, s in stages if s == 'boosting'), 0.0)
    gnss, baro, imu = streams.get('gnss.csv'), streams.get('baro_icp10111.csv'), streams.get('imu_bno055.csv')

    def rel(stream, field):
        if stream is None or field not in stream.fields:
            return [], []
        ts, vs = stream.column(field)
        return [t - launch for t in ts], vs

    lat_t, lat = rel(gnss, 'lat')
    _, lon = rel(gnss, 'lon')
    spd_t, knots = rel(gnss, 'speed_kn')
    hgt = rel(baro, 'elevation') if (baro and 'elevation' in baro.fields) else rel(baro, 'altitude')
    hgt = (hgt[0], _smooth(_smooth(hgt[1], 11), 11))           # smooth the glider's vertical motion
    pit = rel(imu, 'pitch')
    rol = rel(imu, 'roll')
    pit = (pit[0], _smooth(_smooth(pit[1], 7), 7))             # smooth attitude -> jet-like, no jitter
    rol = (rol[0], _smooth(_smooth(rol[1], 7), 7))
    ex = _smooth(_smooth([_to_m(lat[i], lon[i])[0] for i in range(len(lat))], 21), 21)
    ny = _smooth(_smooth([_to_m(lat[i], lon[i])[1] for i in range(len(lat))], 21), 21)
    track = list(zip(ex, ny))
    apogee = max(zip(hgt[0], hgt[1]), key=lambda p: p[1]) if hgt[0] else (0.0, 0.0)
    land_t = next((t for t, s in stages if s == 'landing'), apogee[0])
    done_t = next((t for t, s in stages if s == 'done'), hgt[0][-1] if hgt[0] else land_t)
    glide_t = next((t for t, s in stages if s == 'gliding'), apogee[0])
    cmid = ((_TL[0] + _BR[0]) / 2, (_TL[1] + _BR[1]) / 2)
    td, cm = (track[-1] if track else (0.0, 0.0)), _to_m(*cmid)
    return {'label': label, 'motor': label.split()[0], 'pos': (lat_t, track), 'height': hgt,
            'speed': (spd_t, [k / 1.94384 for k in knots]), 'heading': rel(imu, 'heading'),
            'roll': rol, 'pitch': pit, 'stages': stages, 'launch': launch,
            'glide_t': glide_t, 'burn': _BURN.get(label.split()[0], 3.0), 'end': max(done_t, land_t) + 2.5,
            'apogee': apogee, 'land_t': land_t, 'miss': math.hypot(td[0] - cm[0], td[1] - cm[1]),
            'in_zone': bool(track) and (_BR[0] <= lat[-1] <= _TL[0]) and (_TL[1] <= lon[-1] <= _BR[1]),
            'apogee_pos': _at(lat_t, track, apogee[0]) or (0.0, 0.0)}


def captions(fl):
    ap, lt, sep = fl['apogee'], fl['land_t'], fl['glide_t']
    return [
        (-3.0, 0.0, 'On the rod: TMS-7 glider vertical, %s loaded, wings folded.' % fl['motor']),
        (0.0, fl['burn'], 't=0.0 s  IGNITION -- off the rod, climbing.'),
        (fl['burn'], sep, 't=%.1f s  burnout, motor spent -- coasting up to apogee.' % fl['burn']),
        (sep, sep + 6, 't=%.1f s  SEPARATION -- booster ejects under chute, wings sweep to 15deg.' % sep),
        (sep + 6, lt, 'Gliding home (~%.0f m apogee) -- banking to turn toward the zone.' % ap[1]),
        (lt, fl['end'], 'Touchdown t=%.0f s -- %.0f m from centre -- %s' % (
            lt, fl['miss'], 'IN THE ZONE' if fl['in_zone'] else 'just outside the zone')),
    ]


def fixed_cam(fl):
    pts = [_wxy(p[0], p[1], max(0.0, _at(fl['height'][0], fl['height'][1], tt) or 0.0))
           for tt, p in zip(fl['pos'][0], fl['pos'][1])]
    for la, lo in ((_TL[0], _TL[1]), (_TL[0], _BR[1]), (_BR[0], _BR[1]), (_BR[0], _TL[1])):
        pts.append(_wxy(*_to_m(la, lo), 0.0))
    pts.append(_wxy(0.0, 0.0, 0.0))
    wx, wy = [p[0] for p in pts], [p[1] for p in pts]
    zoom = min((_MAP[2] - _MAP[0]) / ((max(wx) - min(wx)) * 1.15 + 1),
               (_MAP[3] - _MAP[1]) / ((max(wy) - min(wy)) * 1.18 + 1))
    cwx, cwy = (min(wx) + max(wx)) / 2, (min(wy) + max(wy)) / 2
    ax, ay = (_MAP[0] + _MAP[2]) / 2, (_MAP[1] + _MAP[3]) / 2
    return zoom, cwx, cwy, ax, ay


def render(flights, out):
    proc = subprocess.Popen(
        ['ffmpeg', '-y', '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '%dx%d' % (_W, _H),
         '-framerate', str(_FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', out],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    corners = [(_TL[0], _TL[1]), (_TL[0], _BR[1]), (_BR[0], _BR[1]), (_BR[0], _TL[1])]
    cmid = ((_TL[0] + _BR[0]) / 2, (_TL[1] + _BR[1]) / 2)
    for fl in flights:
        cards = captions(fl)
        zoom, cwx, cwy, cax, cay = fixed_cam(fl)
        gp, hh = fl['pos'], fl['height']

        def proj(e, n, h, zoom=zoom, cwx=cwx, cwy=cwy, cax=cax, cay=cay):
            wx, wy = _wxy(e, n, h)
            return (cax + zoom * (wx - cwx), cay + zoom * (wy - cwy))

        sched = sorted([('ignite', 0.0), ('burnout', fl['burn']), ('apogee', fl['apogee'][0]),
                        ('separate', fl['glide_t']), ('land', fl['land_t'])], key=lambda x: x[1])
        for f in range(int((fl['end'] + 3.0) * _FPS)):
            t = -3.0 + f / _FPS
            tc = max(t, 0.0)
            gpos = _at(gp[0], gp[1], tc) or (0.0, 0.0)
            height = max(0.0, _at(hh[0], hh[1], tc) or 0.0)
            img = Image.new('RGB', (_W, _H), _GREEN)
            d = ImageDraw.Draw(img)
            zc = [proj(*_to_m(la, lo), 0.0) for la, lo in corners]
            d.polygon(zc, fill=_ZONE, outline=(228, 228, 135))
            ccx, ccy = proj(*_to_m(*cmid), 0.0)
            d.line([ccx - 9, ccy, ccx + 9, ccy], fill=(235, 235, 150), width=2)
            d.line([ccx, ccy - 9, ccx, ccy + 9], fill=(235, 235, 150), width=2)
            for la, lo in corners:
                e, n = _to_m(la, lo)
                bx, by = proj(e, n, 0.0)
                tx, ty = proj(e, n, 6.0)
                d.line([bx, by, tx, ty], fill=(92, 62, 36), width=4)
                r = 16
                d.ellipse([tx - r, ty - r, tx + r, ty + r // 2], fill=(36, 82, 38))
                d.ellipse([tx - r * 3 // 4, ty - r - 5, tx + r * 3 // 4, ty], fill=(48, 108, 52))
            px, py = proj(0.0, 0.0, 0.0)
            d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(208, 62, 52))
            d.text((px + 10, py - 8), 'pad', font=F_LBL, fill=(235, 205, 185))
            trail = [proj(p[0], p[1], max(0.0, _at(hh[0], hh[1], tt) or 0.0))
                     for tt, p in zip(gp[0], gp[1]) if 0 <= tt <= tc]
            if len(trail) > 1:
                _dashed(d, trail, (242, 242, 252))
            if t > fl['glide_t']:
                bh = max(0.0, fl['apogee'][1] - 7.0 * (t - fl['glide_t']))
                bx, by = proj(fl['apogee_pos'][0], fl['apogee_pos'][1], bh)
                d.line([bx, by - 14, bx, by + 12], fill=(70, 70, 78), width=4)       # booster body
                rr, cyb = 24, by - 8                                                 # canopy radius + rim y
                d.pieslice([bx - rr, cyb - rr, bx + rr, cyb + rr], 180, 360, fill=(238, 238, 238))
                for cgx in range(-rr, rr, 9):                                        # Estes red/white checker
                    for cgy in range(-rr, 1, 9):
                        if cgx * cgx + cgy * cgy <= rr * rr and (cgx // 9 + cgy // 9) % 2 == 0:
                            d.rectangle([bx + cgx, cyb + cgy, bx + cgx + 9, cyb + cgy + 9], fill=(206, 56, 52))
                d.arc([bx - rr, cyb - rr, bx + rr, cyb + rr], 180, 360, fill=(150, 150, 150))
                d.line([bx - rr + 3, cyb - 2, bx, by - 14], fill=(210, 210, 210))    # shroud lines
                d.line([bx + rr - 3, cyb - 2, bx, by - 14], fill=(210, 210, 210))
            pitch = _at(fl['pitch'][0], fl['pitch'][1], tc) or 90.0
            roll = _at(fl['roll'][0], fl['roll'][1], tc) or 0.0
            nxt = _at(gp[0], gp[1], tc + 0.7) or gpos
            vel = (nxt[0] - gpos[0], nxt[1] - gpos[1])
            if (vel[0] ** 2 + vel[1] ** 2) ** 0.5 > 1.0:                      # smooth: point along the path
                head = math.degrees(math.atan2(vel[0], vel[1]))
            else:
                head = _at(fl['heading'][0], fl['heading'][1], tc) or 0.0
            wings = t >= fl['glide_t']                                        # no wings until separation
            sweep = max(0.0, 1.0 - (t - fl['glide_t']) / 1.5) if wings else 1.0  # then unfold folded->15deg
            gx, gy = proj(gpos[0], gpos[1], height)
            if t >= -0.3:
                draw_plane(d, gx, gy, head, max(pitch, -90), roll, sweep, wings)
            # HUD
            d.rectangle([0, 0, _W, 80], fill=_SKY)
            d.rectangle([0, _CAP_Y - 8, _W, _H], fill=_SKY)
            d.rectangle([_PANEL_X - 30, 92, _W, _CAP_Y - 16], fill=(18, 40, 28))
            spd = max(0.0, _at(fl['speed'][0], fl['speed'][1], tc) or 0.0)
            d.text((_PANEL_X, 112), '%s   -   5%% noise, calm' % fl['label'], font=F_TITLE, fill=(245, 245, 185))
            for i, (lab, val) in enumerate([('time', '%5.1f s' % tc), ('speed', '%5.1f m/s' % spd),
                                            ('height', '%5.0f m' % height), ('roll', '%+5.0f deg' % roll),
                                            ('pitch', '%+5.0f deg' % pitch)]):
                yy = 172 + i * 84
                d.text((_PANEL_X, yy), lab, font=F_LBL, fill=(170, 205, 170))
                d.text((_PANEL_X, yy + 22), val, font=F_NUM, fill=(255, 255, 255))
            d.text((_PANEL_X, 612), 'schedule', font=F_LBL, fill=(170, 205, 170))
            for i, (lab, et) in enumerate(sched):
                done = tc >= et
                col = (255, 212, 120) if (done and tc - et < 1.2) else ((150, 230, 150) if done else (120, 140, 120))
                d.text((_PANEL_X, 644 + i * 34), '%4.1fs  %s' % (et, lab), font=F_SCH, fill=col)
            d.text((30, 16), 'Coludo TMS-7  --  guarded-fins HITL simulation', font=F_HEAD, fill=(232, 232, 242))
            d.text((40, _CAP_Y + 22), next((c[2] for c in cards if c[0] <= t < c[1]), ''),
                   font=F_BIG, fill=(255, 255, 255))
            proc.stdin.write(img.tobytes())
    proc.stdin.close()
    proc.wait()


if __name__ == '__main__':
    out = sys.argv[1]
    flights = [load(sys.argv[i], sys.argv[i + 1]) for i in range(2, len(sys.argv), 2)]
    render(flights, out)
    print('wrote', out, '(%d flights)' % len(flights))
