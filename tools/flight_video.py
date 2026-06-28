# tools/flight_video.py — render a narrated 3D-ish animation of one or more flight captures to an mp4.
# Uses follow-cam (glider stays at 35 % left, 40 % from bottom of the map pane) so the plane is large
# enough to see its attitude, fin deflections, and wing deployment throughout. The ground track, zone,
# trees, and trail all shift each frame. A pure-PIL frame pipeline piped to ffmpeg, FHD 1920x1080 @ 50 fps.
# Usage: flight_video.py <out.mp4> <LABEL> <capture.txt> [<LABEL> <capture.txt>] ...

import math
import os
import subprocess
import sys
from collections import deque

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_telemetry  # noqa: E402

_M = 111320.0
_PAD = (25.514379, -80.391795)
_TL, _BR = (25.514944, -80.392972), (25.514583, -80.391111)
_COSLAT = math.cos(math.radians(_PAD[0]))
_W, _H, _FPS = 1920, 1080, 50
_GREEN, _ZONE, _SKY = (78, 128, 64), (98, 152, 80), (24, 28, 36)
_KH, _KN = 1.0, 0.55
_MAP = (44, 100, 1390, 968)
_PANEL_X = 1520
_CAP_Y = 990
_PS = 19.0
_BURN = {'F15': 3.45, 'E16': 1.77}
_LIGHT = (0.36, -0.45, 0.82)
_FONTS = '/usr/share/fonts/truetype/dejavu/'
_TRAIL_SEC = 2.0
_TRAIL_POINTS_MAX = 200
_TARGET_GX = 0.35
_TARGET_GY = 0.40


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
        return tuple(av + (bv - av) * f for av, bv in zip(a, b))
    return a + (b - a) * f


def _wxy(e, n, h):
    return (e, -(h * _KH + n * _KN))


def _smooth(xs, win=9):
    if len(xs) < 3:
        return xs
    half = win // 2
    return [sum(xs[max(0, i - half):min(len(xs), i + half + 1)]) /
            len(xs[max(0, i - half):min(len(xs), i + half + 1)]) for i in range(len(xs))]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    m = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) ** 0.5 or 1.0
    return (a[0] / m, a[1] / m, a[2] / m)


def _axes(heading, pitch, roll):
    th, ph, ro = math.radians(heading), math.radians(pitch), math.radians(roll)
    f = (math.cos(ph) * math.sin(th), math.cos(ph) * math.cos(th), math.sin(ph))
    r0 = _norm(_cross(f, (0.0, 0.0, 1.0))) if abs(ph) < math.radians(89.5) else (math.cos(th), -math.sin(th), 0.0)
    u0 = _cross(r0, f)
    cr, sr = math.cos(ro), math.sin(ro)
    r = (r0[0] * cr - u0[0] * sr, r0[1] * cr - u0[1] * sr, r0[2] * cr - u0[2] * sr)
    u = _cross(r, f)
    return f, r, u


# -- plane model --
# Vertices (x fwd, y right, z up). Base model at neutral fins, wings deployed.
# Wing indices are grouped so _fold_wing_verts can rotate them around the fuselage
# (butterfly-knife fold). Fin indices group trailing-edge vertices that deflect.
_V = [
    (2.4, 0.0, 0.0),       # 0  nose tip
    (1.6, 0.0, 0.20),      # 1  nose base top
    (0.7, 0.0, 0.40),      # 2  canopy
    (-0.4, 0.0, 0.35),     # 3  aft canopy
    (-1.0, 0.0, 0.20),     # 4  tail top
    (-2.0, 0.0, 0.0),      # 5  tail tip
    (0.5, 0.0, -0.30),     # 6  belly
    (-1.0, 0.0, -0.18),    # 7  tail bottom
    (0.8, -0.35, 0.05),    # 8  left wing root LE
    (0.0, -0.35, 0.05),    # 9  left wing root TE
    (0.2, -3.0, 0.05),     # 10 left wing tip LE
    (-0.5, -2.8, 0.05),    # 11 left wing tip TE
    (0.8, 0.35, 0.05),     # 12 right wing root LE
    (0.0, 0.35, 0.05),     # 13 right wing root TE
    (0.2, 3.0, 0.05),      # 14 right wing tip LE
    (-0.5, 2.8, 0.05),     # 15 right wing tip TE
    (-0.6, 0.0, 0.20),     # 16 vert fin root front
    (-1.6, 0.0, 0.0),      # 17 vert fin root rear
    (-0.9, 0.0, 1.40),     # 18 vert fin tip (tall tail fin)
    (-1.4, -0.50, 0.05),   # 19 left tailplane tip
    (-1.4, 0.50, 0.05),    # 20 right tailplane tip
    (-1.7, -0.30, 0.05),   # 21 left tailplane root
    (-1.7, 0.30, 0.05),    # 22 right tailplane root
]

# (i, j, k, colour) — order controls backface culling (CW = visible from outside)
_FACES = [
    # Fuselage
    (0, 1, 6, (180, 175, 70)),      # nose left
    (0, 6, 1, (180, 175, 70)),      # nose right (double-sided)
    (1, 2, 6, (190, 185, 75)),      # canopy -> belly
    (2, 3, 6, (185, 180, 72)),      # aft canopy -> belly
    (3, 7, 6, (175, 170, 65)),      # tail -> belly
    (3, 4, 7, (170, 165, 60)),      # tail top -> bottom
    (4, 5, 7, (165, 160, 58)),      # tail tip
    (1, 0, 2, (200, 195, 80)),      # nose top
    (2, 4, 3, (195, 190, 78)),      # spine
    (2, 0, 8, (190, 185, 75)),      # nose -> wing root

    # Left wing (top surface)
    (8, 10, 11, (200, 196, 64)),
    (8, 11, 9, (200, 196, 64)),
    # Left wing (bottom surface, darker)
    (8, 11, 10, (165, 162, 50)),
    (8, 9, 11, (165, 162, 50)),

    # Right wing
    (12, 15, 14, (200, 196, 64)),
    (12, 13, 15, (200, 196, 64)),
    (12, 14, 15, (165, 162, 50)),
    (12, 15, 13, (165, 162, 50)),

    # Vertical fin
    (16, 17, 18, (190, 185, 70)),
    (16, 18, 17, (160, 155, 58)),

    # Left tailplane
    (19, 4, 21, (200, 196, 64)),
    (19, 21, 4, (165, 162, 50)),

    # Right tailplane
    (20, 22, 4, (200, 196, 64)),
    (20, 4, 22, (165, 162, 50)),
]

# Indices of wing vertices (including tip LE/TE and root LE/TE for both wings)
_WING_LEFT = (8, 9, 10, 11)
_WING_RIGHT = (12, 13, 14, 15)
_ROOT_Y = 0.35
_HINGE_X = 0.4         # wing pivot x (between root LE 0.8 and TE 0.0) for the sweep


def _fold_wing_verts(v, fold_frac):
    """Stow/deploy the wings the way Coludo really does it: during boost the wings are swept AFT and
    tucked UNDER the fuselage (folded back along the body, not raised up like a bird); when the booster
    drops and the hook releases them they SWEEP OUT to the flying position. fold_frac 1 = stowed, 0 =
    deployed. The sweep is a rotation about the root's vertical axis so each tip swings from lateral
    (+/-y) toward the tail (-x), plus a small downward tuck under the body line."""
    phi = fold_frac * math.radians(88)     # 0 deployed (full span) -> 88 stowed (swept back along body)
    c, s = math.cos(phi), math.sin(phi)
    tuck = -0.30 * fold_frac               # drop under the body centreline when stowed
    for side, idx in ((1, _WING_RIGHT), (-1, _WING_LEFT)):
        for i in idx:
            vx, vy, vz = v[i]
            ox = vx - _HINGE_X
            v[i] = (_HINGE_X + ox * c - side * vy * s,   # tip swings aft (-x) as it folds
                    side * ox * s + vy * c,              # ... and inboard toward the centreline
                    vz + tuck)                           # ... tucked just under the body
    return v


def _deflect_fins(v, left, right, rudder):
    """Deflect control surfaces: elevons bend trailing edge, rudder swings tail."""
    if left is not None:
        dz = math.radians(left) * 0.4
        v[11] = (v[11][0], v[11][1], v[11][2] + dz)
    if right is not None:
        dz = math.radians(right) * 0.4
        v[15] = (v[15][0], v[15][1], v[15][2] + dz)
    if rudder is not None:
        dx = math.radians(rudder) * 0.25
        v[17] = (v[17][0] - dx, v[17][1], v[17][2])
    return v


def draw_plane(d, cx, cy, heading, pitch, roll, fold_frac, left_ele, right_ele, yaw):
    f, r, u = _axes(heading, pitch, roll)
    verts = list(_V)
    _deflect_fins(verts, left_ele, right_ele, yaw)
    _fold_wing_verts(verts, fold_frac)
    world = [(vx * f[0] + vy * r[0] + vz * u[0], vx * f[1] + vy * r[1] + vz * u[1],
              vx * f[2] + vy * r[2] + vz * u[2]) for vx, vy, vz in verts]
    scr = [(cx + _PS * w[0], cy - _PS * (w[2] * _KH + w[1] * _KN)) for w in world]
    drawn = []
    for i, j, k, col in _FACES:
        a, b, c = world[i], world[j], world[k]
        nrm = _norm(_cross((b[0] - a[0], b[1] - a[1], b[2] - a[2]),
                           (c[0] - a[0], c[1] - a[1], c[2] - a[2])))
        depth = (a[1] + b[1] + c[1]) / 3.0
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


def _draw_sky_gradient(d, horizon_y, height):
    alt_factor = min(1.0, height / 150.0)
    bands = 30
    bh = horizon_y / bands
    for i in range(bands):
        t = i / bands
        hz_r = 110 - int(70 * alt_factor)
        hz_g = 160 - int(50 * alt_factor)
        hz_b = 210 - int(70 * alt_factor)
        r = int(hz_r + (_SKY[0] - hz_r) * t)
        g = int(hz_g + (_SKY[1] - hz_g) * t)
        b = int(hz_b + (_SKY[2] - hz_b) * t)
        d.rectangle([0, int(i * bh), _W, int((i + 1) * bh)], fill=(r, g, b))


def _draw_smoke_trail(d, trail):
    for sx, sy, age in trail:
        t = age / _TRAIL_SEC
        radius = 2 + t * 8
        gray = 100 + int(t * 100)
        d.ellipse([sx - radius, sy - radius, sx + radius, sy + radius],
                  fill=(gray, gray, gray))


def _draw_exhaust(d, bx, by, dx, dy, intensity):
    if intensity <= 0:
        return
    length = 30 * intensity
    flicker = 0.85 + 0.15 * math.sin(bx * 0.1 + by * 0.07) * math.sin(bx * 0.07 + by * 0.05)
    length *= flicker
    tip_x = bx + length * dx
    tip_y = by + length * dy
    layers = [(length * 0.35, (255, 60, 10)),
              (length * 0.20, (255, 160, 30)),
              (length * 0.08, (255, 240, 180))]
    for width, color in layers:
        px = -dy * width
        py = dx * width
        d.polygon([(bx + px, by + py), (bx - px, by - py), (tip_x, tip_y)], fill=color)


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
    hgt = (hgt[0], _smooth(_smooth(hgt[1], 11), 11))
    pit = rel(imu, 'pitch')
    rol = rel(imu, 'roll')
    pit = (pit[0], _smooth(_smooth(pit[1], 7), 7))
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

    # Load fins telemetry (eleron_left, eleron_right, yaw) — optional
    fins = None
    fins_str = streams.get('fins.csv')
    if fins_str and 'eleron_left' in fins_str.fields:
        ft, fl = rel(fins_str, 'eleron_left')
        _, fr = rel(fins_str, 'eleron_right')
        _, fy = rel(fins_str, 'yaw')
        fins = (ft, list(zip(fl, fr, fy)))

    # Load acceleration (ax, ay, az) — optional
    accel = None
    accel_str = streams.get('accel_adxl375.csv')
    if accel_str and 'ax' in accel_str.fields:
        at, ax = rel(accel_str, 'ax')
        _, ay = rel(accel_str, 'ay')
        _, az = rel(accel_str, 'az')
        accel = (at, list(zip(ax, ay, az)))

    return {'label': label, 'motor': label.split()[0], 'pos': (lat_t, track), 'height': hgt,
            'speed': (spd_t, [k / 1.94384 for k in knots]), 'heading': rel(imu, 'heading'),
            'roll': rol, 'pitch': pit, 'stages': stages, 'launch': launch,
            'glide_t': glide_t, 'burn': _BURN.get(label.split()[0], 3.0), 'end': max(done_t, land_t) + 2.5,
            'apogee': apogee, 'land_t': land_t, 'miss': math.hypot(td[0] - cm[0], td[1] - cm[1]),
            'in_zone': bool(track) and (_BR[0] <= lat[-1] <= _TL[0]) and (_TL[1] <= lon[-1] <= _BR[1]),
            'apogee_pos': _at(lat_t, track, apogee[0]) or (0.0, 0.0),
            'fins': fins, 'accel': accel}


def captions(fl):
    ap, lt, sep = fl['apogee'], fl['land_t'], fl['glide_t']
    return [
        (-3.0, 0.0, 'On the rod: TMS-7 glider vertical, %s loaded, wings folded.' % fl['motor']),
        (0.0, fl['burn'], 't=0.0 s  IGNITION -- off the rod, climbing.'),
        (fl['burn'], sep, 't=%.1f s  burnout, motor spent -- coasting up to apogee.' % fl['burn']),
        (sep, sep + 6, 't=%.1f s  SEPARATION -- booster ejects under chute, wings deploy.' % sep),
        (sep + 6, lt, 'Gliding home (~%.0f m apogee) -- banking to turn toward the zone.' % ap[1]),
        (lt, fl['end'], 'Touchdown t=%.0f s -- %.0f m from centre -- %s' % (
            lt, fl['miss'], 'IN THE ZONE' if fl['in_zone'] else 'just outside the zone')),
    ]


def _calc_zoom():
    corners = [(_TL[0], _TL[1]), (_TL[0], _BR[1]), (_BR[0], _BR[1]), (_BR[0], _TL[1])]
    pts = [_wxy(*_to_m(la, lo), 0.0) for la, lo in corners]
    pts.append(_wxy(0.0, 0.0, 0.0))
    wx = [p[0] for p in pts]
    wy = [p[1] for p in pts]
    return min((_MAP[2] - _MAP[0]) / ((max(wx) - min(wx)) * 1.15 + 1),
               (_MAP[3] - _MAP[1]) / ((max(wy) - min(wy)) * 1.18 + 1))


def render(flights, out):
    proc = subprocess.Popen(
        ['ffmpeg', '-y', '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '%dx%d' % (_W, _H),
         '-framerate', str(_FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', out],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    corners = [(_TL[0], _TL[1]), (_TL[0], _BR[1]), (_BR[0], _BR[1]), (_BR[0], _TL[1])]
    cmid = ((_TL[0] + _BR[0]) / 2, (_TL[1] + _BR[1]) / 2)
    map_cx = (_MAP[0] + _MAP[2]) / 2
    map_cy = (_MAP[1] + _MAP[3]) / 2
    target_sx = _MAP[0] + _TARGET_GX * (_MAP[2] - _MAP[0])
    target_sy = _MAP[1] + _TARGET_GY * (_MAP[3] - _MAP[1])
    zoom = _calc_zoom()

    for fl in flights:
        cards = captions(fl)
        gp, hh = fl['pos'], fl['height']
        fins_data = fl.get('fins')
        accel_data = fl.get('accel')
        burn_time = fl['burn']

        _cam = [0.0, 0.0]

        def proj(e, n, h, cam=_cam):
            wx, wy = _wxy(e, n, h)
            return (map_cx + zoom * (wx - cam[0]), map_cy + zoom * (wy - cam[1]))

        sched = sorted([('ignite', 0.0), ('burnout', fl['burn']), ('apogee', fl['apogee'][0]),
                        ('separate', fl['glide_t']), ('land', fl['land_t'])], key=lambda x: x[1])

        smoke_trail = deque(maxlen=_TRAIL_POINTS_MAX)
        prev_t = None

        for f in range(int((fl['end'] + 3.0) * _FPS)):
            t = -3.0 + f / _FPS
            tc = max(t, 0.0)
            gpos = _at(gp[0], gp[1], tc) or (0.0, 0.0)
            height = max(0.0, _at(hh[0], hh[1], tc) or 0.0)

            # Follow-cam: compute world center that places the glider at target screen position
            gwx, gwy = _wxy(gpos[0], gpos[1], height)
            _cam[0] = gwx - (target_sx - map_cx) / zoom
            _cam[1] = gwy - (target_sy - map_cy) / zoom

            img = Image.new('RGB', (_W, _H), _GREEN)
            d = ImageDraw.Draw(img)

            # Sky gradient
            horizon_y = int(proj(gpos[0], gpos[1], height)[1] + _MAP[1] * 0.08)
            horizon_y = max(20, min(_H - 200, horizon_y))
            _draw_sky_gradient(d, horizon_y, height)
            d.rectangle([0, horizon_y, _W, _H], fill=_GREEN)

            # Zone
            zc = [proj(*_to_m(la, lo), 0.0) for la, lo in corners]
            d.polygon(zc, fill=_ZONE, outline=(228, 228, 135))
            ccx, ccy = proj(*_to_m(*cmid), 0.0)
            d.line([ccx - 9, ccy, ccx + 9, ccy], fill=(235, 235, 150), width=2)
            d.line([ccx, ccy - 9, ccx, ccy + 9], fill=(235, 235, 150), width=2)

            # Trees
            for la, lo in corners:
                e, n = _to_m(la, lo)
                bx, by = proj(e, n, 0.0)
                tx, ty = proj(e, n, 6.0)
                d.line([bx, by, tx, ty], fill=(92, 62, 36), width=4)
                r = 16
                d.ellipse([tx - r, ty - r, tx + r, ty + r // 2], fill=(36, 82, 38))
                d.ellipse([tx - r * 3 // 4, ty - r - 5, tx + r * 3 // 4, ty], fill=(48, 108, 52))

            # Pad
            px, py = proj(0.0, 0.0, 0.0)
            d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(208, 62, 52))
            d.text((px + 10, py - 8), 'pad', font=F_LBL, fill=(235, 205, 185))

            # Ground track trail (dashed line)
            trail = [proj(p[0], p[1], max(0.0, _at(hh[0], hh[1], tt) or 0.0))
                     for tt, p in zip(gp[0], gp[1]) if 0 <= tt <= tc]
            if len(trail) > 1:
                _dashed(d, trail, (242, 242, 252))

            # Smoke trail
            gx, gy = proj(gpos[0], gpos[1], height)
            if t >= 0:
                smoke_trail.append((gx, gy, 0.0))
                dt_frame = (t - prev_t) if prev_t is not None else (1.0 / _FPS)
                aged = []
                for sx, sy, age in smoke_trail:
                    age += dt_frame
                    if age < _TRAIL_SEC:
                        aged.append((sx, sy, age))
                smoke_trail.clear()
                smoke_trail.extend(aged)
            prev_t = t if t >= 0 else None
            _draw_smoke_trail(d, smoke_trail)

            # Exhaust flame during boost
            if 0 <= t <= fl['burn']:
                intensity = 1.0 - (t / burn_time) ** 0.5
                gx_fwd, gy_fwd = proj(gpos[0], gpos[1], height + 0.5)
                svx = gx_fwd - gx
                svy = gy_fwd - gy
                if svx * svx + svy * svy > 1.0:
                    sv_len = math.hypot(svx, svy)
                    c, s = -svx / sv_len, -svy / sv_len
                else:
                    c, s = 0.0, 1.0

                bx_flame, by_flame = proj(gpos[0], gpos[1], height - 0.5)
                _draw_exhaust(d, bx_flame, by_flame, c, s, intensity)

            # Booster under chute (after separation)
            if t > fl['glide_t']:
                bh = max(0.0, fl['apogee'][1] - 7.0 * (t - fl['glide_t']))
                bx_stage, by_stage = proj(fl['apogee_pos'][0], fl['apogee_pos'][1], bh)
                d.line([bx_stage, by_stage - 14, bx_stage, by_stage + 12], fill=(70, 70, 78), width=4)
                rr, cyb = 24, by_stage - 8
                d.pieslice([bx_stage - rr, cyb - rr, bx_stage + rr, cyb + rr], 180, 360, fill=(238, 238, 238))
                for cgx in range(-rr, rr, 9):
                    for cgy in range(-rr, 1, 9):
                        if cgx * cgx + cgy * cgy <= rr * rr and (cgx // 9 + cgy // 9) % 2 == 0:
                            d.rectangle([bx_stage + cgx, cyb + cgy, bx_stage + cgx + 9, cyb + cgy + 9],
                                        fill=(206, 56, 52))
                d.arc([bx_stage - rr, cyb - rr, bx_stage + rr, cyb + rr], 180, 360, fill=(150, 150, 150))
                d.line([bx_stage - rr + 3, cyb - 2, bx_stage, by_stage - 14], fill=(210, 210, 210))
                d.line([bx_stage + rr - 3, cyb - 2, bx_stage, by_stage - 14], fill=(210, 210, 210))

            # Glider attitude
            pitch = _at(fl['pitch'][0], fl['pitch'][1], tc) or 90.0
            roll = _at(fl['roll'][0], fl['roll'][1], tc) or 0.0
            nxt = _at(gp[0], gp[1], tc + 0.7) or gpos
            vel = (nxt[0] - gpos[0], nxt[1] - gpos[1])
            if (vel[0] ** 2 + vel[1] ** 2) ** 0.5 > 1.0:
                head = math.degrees(math.atan2(vel[0], vel[1]))
            else:
                head = _at(fl['heading'][0], fl['heading'][1], tc) or 0.0

            # Wing deployment animation
            wings = t >= fl['glide_t']
            fold_frac = max(0.0, 1.0 - (t - fl['glide_t']) / 1.5) if wings else 1.0

            # Fin angles
            left_ele = right_ele = yaw = None
            if fins_data is not None:
                fin_val = _at(fins_data[0], fins_data[1], tc)
                if fin_val is not None:
                    left_ele, right_ele, yaw = fin_val

            if t >= -0.3:
                draw_plane(d, gx, gy, head, max(pitch, -90), roll, fold_frac, left_ele, right_ele, yaw)

            # Vertical speed from smoothed height profile (immune to per-frame noise)
            vspeed = 0.0
            if tc > 0 and tc < hh[0][-1]:
                h1 = _at(hh[0], hh[1], tc + 0.15) or height
                h2 = _at(hh[0], hh[1], max(0, tc - 0.15)) or height
                vspeed = (h1 - h2) / 0.3

            # G-load from accel
            g_load = None
            if accel_data is not None:
                acc = _at(accel_data[0], accel_data[1], tc)
                if acc is not None:
                    ax, ay, az = acc
                    g_load = math.sqrt(ax * ax + ay * ay + az * az) / 9.80665

            # HUD
            d.rectangle([0, 0, _W, 80], fill=_SKY)
            d.rectangle([0, _CAP_Y - 8, _W, _H], fill=_SKY)
            d.rectangle([_PANEL_X - 30, 92, _W, _CAP_Y - 16], fill=(18, 40, 28))
            spd = max(0.0, _at(fl['speed'][0], fl['speed'][1], tc) or 0.0)
            d.text((_PANEL_X, 112), '%s   -   5%% noise, calm' % fl['label'], font=F_TITLE, fill=(245, 245, 185))

            metrics = [
                ('time', '%5.1f s' % tc),
                ('speed', '%5.1f m/s' % spd),
                ('height', '%5.0f m' % height),
                ('roll', '%+5.0f deg' % roll),
                ('pitch', '%+5.0f deg' % pitch),
                ('v/speed', '%+5.1f m/s' % vspeed),
            ]
            if g_load is not None:
                metrics.append(('g-load', '%5.2f g' % g_load))

            for i, (lab, val) in enumerate(metrics):
                yy = 172 + i * 72
                d.text((_PANEL_X, yy), lab, font=F_LBL, fill=(170, 205, 170))
                d.text((_PANEL_X, yy + 22), val, font=F_NUM, fill=(255, 255, 255))

            schedule_y = 172 + len(metrics) * 72 + 20
            d.text((_PANEL_X, schedule_y), 'schedule', font=F_LBL, fill=(170, 205, 170))
            for i, (lab, et) in enumerate(sched):
                done = tc >= et
                col = (255, 212, 120) if (done and tc - et < 1.2) else ((150, 230, 150) if done else (120, 140, 120))
                d.text((_PANEL_X, schedule_y + 32 + i * 34), '%4.1fs  %s' % (et, lab), font=F_SCH, fill=col)

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
