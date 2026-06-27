# tools/flight_video.py -- render a narrated 3D-ish animation of one or more flight captures to an mp4.
# An oblique camera shows ALTITUDE (the glider climbs up the screen during boost, unlike a flat top-down
# view) over a green field with the landing zone + four corner trees. The glider is a little swing-wing
# plane: vertical on the rod, wings folded during boost, then -- at separation -- the booster drops away
# under a parachute and the glider's wings sweep out to 15deg (F-111 style) for the glide. Attitude
# (pitch/roll/heading) and the live telemetry are reflected; the camera follows the plane and pulls back
# for landing. Pure PIL frames piped to ffmpeg, FHD 1920x1080 @ 50 fps.
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
_GREEN, _ZONE, _SKY, _GROUND = (78, 128, 64), (98, 152, 80), (24, 28, 36), (60, 104, 52)
_KH, _KN = 1.0, 0.55          # oblique gains: altitude (vertical) and north (depth)
_ANCHOR = (760, 470)          # where the followed glider sits on screen
_PANEL_X = 1320
_CAP_Y = 992
_FONTS = '/usr/share/fonts/truetype/dejavu/'


def _font(name, size):
    try:
        return ImageFont.truetype(_FONTS + name, size)
    except OSError:
        return ImageFont.load_default()


F_HEAD = _font('DejaVuSans-Bold.ttf', 40)
F_TITLE = _font('DejaVuSans-Bold.ttf', 34)
F_BIG = _font('DejaVuSans-Bold.ttf', 44)
F_NUM = _font('DejaVuSansMono-Bold.ttf', 50)
F_LBL = _font('DejaVuSans.ttf', 26)


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
    """World point -> oblique plane coords (x right, y down); altitude + north both lift it up."""
    return (e, -(h * _KH + n * _KN))


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
    track = [_to_m(lat[i], lon[i]) for i in range(len(lat))]
    apogee = max(zip(hgt[0], hgt[1]), key=lambda p: p[1]) if hgt[0] else (0.0, 0.0)
    land_t = next((t for t, s in stages if s == 'landing'), apogee[0])
    done_t = next((t for t, s in stages if s == 'done'), hgt[0][-1] if hgt[0] else land_t)
    glide_t = next((t for t, s in stages if s == 'gliding'), apogee[0])
    centre = ((_TL[0] + _BR[0]) / 2, (_TL[1] + _BR[1]) / 2)
    td, cm = (track[-1] if track else (0.0, 0.0)), _to_m(*centre)
    speed = (spd_t, [k / 1.94384 for k in knots])
    rod = next((speed[1][i] for i in range(len(speed[0])) if speed[1][i] > 13), 15.0)
    return {'label': label, 'motor': label.split()[0], 'pos': (lat_t, track), 'speed': speed,
            'height': hgt, 'heading': rel(imu, 'heading'), 'roll': rel(imu, 'roll'),
            'pitch': rel(imu, 'pitch'), 'stages': stages, 'launch': launch, 'glide_t': glide_t,
            'end': max(done_t, land_t) + 2.5, 'apogee': apogee, 'land_t': land_t,
            'miss': math.hypot(td[0] - cm[0], td[1] - cm[1]),
            'in_zone': bool(track) and (_BR[0] <= lat[-1] <= _TL[0]) and (_TL[1] <= lon[-1] <= _BR[1]),
            'rod_speed': rod, 'apogee_pos': _at(lat_t, track, apogee[0]) or (0.0, 0.0)}


def captions(fl):
    motor, ap, lt = fl['motor'], fl['apogee'], fl['land_t']
    return [
        (-3.0, 0.0, 'On the rod: TMS-7 glider vertical, loaded with %s, wings folded.' % motor),
        (0.0, 5.0, 'IGNITION!  off the rod, climbing fast (~%.0f m/s).' % fl['rod_speed']),
        (5.0, ap[0], 'Powered climb -- watch the altitude build (live readout, right).'),
        (ap[0], ap[0] + 6, 'Apogee ~%.0f m -- booster ejects under chute, wings sweep out to 15deg.' % ap[1]),
        (ap[0] + 6, lt, 'Gliding -- banking to turn back toward the landing zone.'),
        (lt, fl['end'], 'Touchdown t=%.0f s -- %.0f m from centre -- %s' % (
            lt, fl['miss'], 'IN THE ZONE' if fl['in_zone'] else 'just outside the zone')),
    ]


def plane_sprite(scale, sweep_deg, bank_deg, body):
    """A little plane drawn nose-UP on an RGBA tile: swept wings (sweep_deg back from straight-out) and a
    bank (one wingtip raised). Caller rotates it by the screen heading."""
    s = int(96 * scale)
    im = Image.new('RGBA', (s, s), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    cx, cy = s / 2, s / 2
    nose, tail = cy - 40 * scale, cy + 30 * scale
    d.polygon([(cx, nose), (cx - 5 * scale, tail), (cx + 5 * scale, tail)], fill=body)        # fuselage
    d.polygon([(cx - 3 * scale, tail - 4), (cx - 16 * scale, tail + 6), (cx + 16 * scale, tail + 6),
               (cx + 3 * scale, tail - 4)], fill=body)                                         # tailplane
    span, root = 40 * scale, cy - 2 * scale
    sw, bk = math.radians(sweep_deg), math.sin(math.radians(bank_deg))
    tipx, tipy = span * math.cos(sw), span * math.sin(sw)
    d.polygon([(cx, root - 4 * scale), (cx - tipx, root + tipy - 7 * scale * bk),
               (cx - tipx, root + tipy + 3 * scale - 7 * scale * bk), (cx, root + 5 * scale)], fill=body)
    d.polygon([(cx, root - 4 * scale), (cx + tipx, root + tipy + 7 * scale * bk),
               (cx + tipx, root + tipy + 3 * scale + 7 * scale * bk), (cx, root + 5 * scale)], fill=body)
    d.ellipse([cx - 4 * scale, nose - 2, cx + 4 * scale, nose + 8 * scale], fill=(245, 245, 250))  # canopy
    return im


def draw_plane(img, x, y, heading_deg, bank_deg, sweep_deg, scale, body):
    spr = plane_sprite(scale, sweep_deg, bank_deg, body).rotate(-heading_deg, expand=True,
                                                                resample=Image.BICUBIC)
    img.paste(spr, (int(x - spr.width / 2), int(y - spr.height / 2)), spr)


def render(flights, out):
    proc = subprocess.Popen(
        ['ffmpeg', '-y', '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '%dx%d' % (_W, _H),
         '-framerate', str(_FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', out],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    corners = [(_TL[0], _TL[1]), (_TL[0], _BR[1]), (_BR[0], _BR[1]), (_BR[0], _TL[1])]
    cmid = ((_TL[0] + _BR[0]) / 2, (_TL[1] + _BR[1]) / 2)
    for fl in flights:
        cards = captions(fl)
        gp, hh = fl['pos'], fl['height']
        for f in range(int((fl['end'] + 3.0) * _FPS)):
            t = -3.0 + f / _FPS
            tc = max(t, 0.0)
            gpos = _at(gp[0], gp[1], tc) or (0.0, 0.0)
            height = max(0.0, _at(hh[0], hh[1], tc) or 0.0)
            # camera: follow the glider, pull back (lower zoom, ease toward zone) for the landing
            land_blend = min(1.0, max(0.0, (t - (fl['land_t'] - 3)) / 5.0))
            zoom = 4.6 * (1 - land_blend) + 2.0 * land_blend
            gw = _wxy(gpos[0], gpos[1], height)
            zw = _wxy(*_to_m(*cmid), 0.0)
            cw = (gw[0] + (zw[0] - gw[0]) * land_blend, gw[1] + (zw[1] - gw[1]) * land_blend)
            ay = _ANCHOR[1] + int(120 * land_blend)

            def proj(e, n, h, zoom=zoom, cw=cw, ay=ay):
                wx, wy = _wxy(e, n, h)
                return (_ANCHOR[0] + zoom * (wx - cw[0]), ay + zoom * (wy - cw[1]))

            img = Image.new('RGB', (_W, _H), _GREEN)
            d = ImageDraw.Draw(img)
            # ground field band + zone + trees + pad (all at h=0)
            zc = [proj(*_to_m(la, lo), 0.0) for la, lo in corners]
            d.polygon(zc, fill=_ZONE, outline=(228, 228, 135))
            ccx, ccy = proj(*_to_m(*cmid), 0.0)
            d.line([ccx - 9, ccy, ccx + 9, ccy], fill=(235, 235, 150), width=2)
            d.line([ccx, ccy - 9, ccx, ccy + 9], fill=(235, 235, 150), width=2)
            for la, lo in corners:                                    # trees: trunk h=0..5 m + canopy
                e, n = _to_m(la, lo)
                bx, by = proj(e, n, 0.0)
                tx, ty = proj(e, n, 6.0)
                d.line([bx, by, tx, ty], fill=(92, 62, 36), width=max(2, int(zoom)))
                r = int(11 * zoom / 4 + 6)
                d.ellipse([tx - r, ty - r, tx + r, ty + r // 2], fill=(36, 82, 38))
                d.ellipse([tx - r * 3 // 4, ty - r - 4, tx + r * 3 // 4, ty], fill=(48, 108, 52))
            px, py = proj(0.0, 0.0, 0.0)
            d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(208, 62, 52))
            d.text((px + 10, py - 8), 'pad', font=F_LBL, fill=(235, 205, 185))
            # trail (the part flown so far), drawn at altitude
            trail = [proj(p[0], p[1], max(0.0, _at(hh[0], hh[1], tt) or 0.0))
                     for tt, p in zip(gp[0], gp[1]) if 0 <= tt <= tc]
            if len(trail) > 1:
                d.line(trail, fill=(240, 240, 250), width=3)
            # booster under parachute after separation
            if t > fl['glide_t']:
                bh = max(0.0, fl['apogee'][1] - 7.0 * (t - fl['glide_t']))
                ax_, an = fl['apogee_pos']
                bx, by = proj(ax_, an, bh)
                d.line([bx, by - 16, bx, by + 14], fill=(70, 70, 78), width=4)               # booster
                d.pieslice([bx - 26, by - 50, bx + 26, by - 6], 180, 360, fill=(210, 90, 80))  # canopy
                d.line([bx - 24, by - 28, bx, by - 14], fill=(220, 220, 220))
                d.line([bx + 24, by - 28, bx, by - 14], fill=(220, 220, 220))
            # the glider plane sprite (followed -> near anchor), with attitude + swing wings
            pitch = _at(fl['pitch'][0], fl['pitch'][1], tc) or 90.0
            roll = _at(fl['roll'][0], fl['roll'][1], tc) or 0.0
            head = _at(fl['heading'][0], fl['heading'][1], tc) or 0.0
            vert = max(0.0, min(1.0, (pitch - 20) / 60.0))            # 1 = vertical (boost), 0 = glide
            sweep = 70 * vert + 15 * (1 - vert)                       # folded on the rod -> 15deg gliding
            screen_head = 0.0 * vert + head * (1 - vert)              # nose up on boost, along course gliding
            gx, gy = proj(gpos[0], gpos[1], height)
            if t >= -0.2:
                draw_plane(img, gx, gy, screen_head, roll, sweep, 1.1, (235, 225, 60))
            # HUD: title strip, panel, caption
            d.rectangle([0, 0, _W, 84], fill=_SKY)
            d.rectangle([0, _CAP_Y - 8, _W, _H], fill=_SKY)
            d.rectangle([_PANEL_X - 30, 96, _W, _CAP_Y - 16], fill=(18, 40, 28))
            spd = max(0.0, _at(fl['speed'][0], fl['speed'][1], tc) or 0.0)
            stage = 'on the rod'
            for st, s in fl['stages']:
                if st - fl['launch'] <= t:
                    stage = s
            d.text((_PANEL_X, 120), '%s   -   5%% noise, calm' % fl['label'], font=F_TITLE, fill=(245, 245, 185))
            for i, (lab, val) in enumerate([('time', '%5.1f s' % tc), ('speed', '%5.1f m/s' % spd),
                                            ('height', '%5.0f m' % height), ('roll', '%+5.0f deg' % roll),
                                            ('pitch', '%+5.0f deg' % (_at(fl['pitch'][0], fl['pitch'][1], tc) or 0))]):
                yy = 184 + i * 96
                d.text((_PANEL_X, yy), lab, font=F_LBL, fill=(170, 205, 170))
                d.text((_PANEL_X, yy + 24), val, font=F_NUM, fill=(255, 255, 255))
            d.text((_PANEL_X, 672), 'stage', font=F_LBL, fill=(170, 205, 170))
            d.text((_PANEL_X, 700), stage.upper(), font=F_BIG, fill=(255, 212, 120))
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
