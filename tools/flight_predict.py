"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Compare a REAL flight against what `sim_model` predicted for it — the artifact `plan.md` asks for and
findings §27.15 flags as missing.

Every landing-accuracy claim in `doc/sims/` rests on the sim being a fair model of the airframe, and
until the two are drawn on the same axes that trust is untested. This flies the SAME `sim_model.Body`
the studies use, from the capture's own motor + liftoff mass, and lines the prediction up against the
measured accelerometer and barometer traces.

    python3 tools/flight_predict.py capture.txt --motor F15 --mass 471
    python3 tools/flight_predict.py capture.txt --motor F15 --mass 471 --svg predict.svg

Reads what the capture already carries (accel magnitude, baro elevation) so it works on a passive
telemetry flight — no control data required.
"""

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'glider'))
import flight_telemetry  # noqa: E402
import preflight  # noqa: E402
import sim_model  # noqa: E402

_STEP = 0.02  # s -- prediction integration step


def predict(motor: str, liftoff_g: float, glider_g: float, seconds: float) -> dict:
    """
    Fly the model open-loop (fins neutral) and return the predicted traces.

    Open-loop on purpose: the point is to test the PHYSICS against a real boost/coast, not the control
    law, and a passive telemetry flight has no active control to reproduce anyway.

    Args:
        motor - the motor key ('E16' / 'F15').
        liftoff_g - whole-stack mass at liftoff (grams).
        glider_g - glider-only mass after separation (grams).
        seconds - how long to integrate.

    Returns:
        {'t': [...], 'accel_g': [...], 'elevation_m': [...], 'apogee_m': float, 'peak_g': float}.
    """
    thrust, burn = sim_model.MOTORS[motor]
    body = sim_model.Body(liftoff_g / 1000.0, sim_model.HPRC['launch'], 0.0,
                          sim_model.HPRC['heading_deg'], glide_mass=glider_g / 1000.0)
    times, accel, elevation = [], [], []
    t = 0.0
    apogee = 0.0
    gliding = False
    while t < seconds:
        if not gliding:
            body.boost_step(_STEP, thrust if t < burn else 0.0, 0.0, 0.0)
            if body.vu <= 0.0 and t > burn:  # apogee -> the booster ejects and the glide begins
                body.begin_glide()
                gliding = True
        else:
            body.glide_step(_STEP, 0.0, 0.0, 0.0)
        times.append(t)
        accel.append(body.accel_g)
        elevation.append(body.alt)
        apogee = max(apogee, body.alt)
        if gliding and body.alt <= 0.0:
            break
        t += _STEP
    return {'t': times, 'accel_g': accel, 'elevation_m': elevation,
            'apogee_m': apogee, 'peak_g': max(accel) if accel else 0.0}


def measured(streams) -> dict:
    """
    The comparable traces from the capture: |accel| (g) and baro elevation (m).

    Returns:
        {'t_accel', 'accel_g', 't_elev', 'elevation_m', 'apogee_m', 'peak_g'}; empty lists when absent.
    """
    accel_stream = flight_telemetry.find_stream(streams, 'ax', 'ay', 'az', prefer='adxl')
    baro = (flight_telemetry.find_stream(streams, 'elevation', prefer='icp')
            or flight_telemetry.find_stream(streams, 'altitude'))
    out = {'t_accel': [], 'accel_g': [], 't_elev': [], 'elevation_m': [], 'apogee_m': 0.0, 'peak_g': 0.0}
    if accel_stream is not None:
        times, ax = accel_stream.column('ax')
        _, ay = accel_stream.column('ay')
        _, az = accel_stream.column('az')
        out['t_accel'] = times
        out['accel_g'] = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(ax, ay, az)]
        out['peak_g'] = max(out['accel_g']) if out['accel_g'] else 0.0
    if baro is not None:
        field = 'elevation' if 'elevation' in baro.fields else 'altitude'
        times, values = baro.column(field)
        if field == 'altitude' and values:  # altitude is AMSL -> re-base to the pad so both are AGL
            ground = min(values)
            values = [v - ground for v in values]
        out['t_elev'], out['elevation_m'] = times, values
        out['apogee_m'] = max(values) if values else 0.0
    return out


def _svg(prediction: dict, real: dict, path: str, title: str) -> None:
    """Write a two-panel SVG (accel, elevation) with predicted vs measured overlaid. Stdlib only."""
    width, height, pad = 980, 620, 60
    panel = (height - 3 * pad) / 2

    def scale(values, lo, hi, y0):
        span = (hi - lo) or 1.0
        return lambda v: y0 + panel - (v - lo) / span * panel

    def line(times, values, fx, fy, colour, dash=''):
        if not times:
            return ''
        points = ' '.join('%.1f,%.1f' % (fx(t), fy(v)) for t, v in zip(times, values))
        return ('<polyline points="%s" fill="none" stroke="%s" stroke-width="1.8"%s/>'
                % (points, colour, ' stroke-dasharray="6 4"' if dash else ''))

    span_t = max(max(prediction['t'] or [1]), max(real['t_accel'] or [1]), max(real['t_elev'] or [1]), 1.0)
    fx = lambda t: pad + t / span_t * (width - 2 * pad)  # noqa: E731
    body = ['<rect width="%d" height="%d" fill="white"/>' % (width, height),
            '<text x="%d" y="28" font-size="17" font-family="sans-serif">%s</text>' % (pad, title)]
    for index, (key_pred, key_t, key_real, label, colour) in enumerate(
            (('accel_g', 't_accel', 'accel_g', '|accel| (g)', '#d62728'),
             ('elevation_m', 't_elev', 'elevation_m', 'elevation (m)', '#1f77b4'))):
        y0 = pad + index * (panel + pad)
        values = list(prediction[key_pred]) + list(real[key_real])
        lo, hi = (min(values), max(values)) if values else (0.0, 1.0)
        fy = scale(values, lo, hi, y0)
        body.append('<rect x="%d" y="%.0f" width="%d" height="%.0f" fill="#fbfbfb" stroke="#ddd"/>'
                    % (pad, y0, width - 2 * pad, panel))
        body.append('<text x="%d" y="%.0f" font-size="13" font-family="sans-serif">%s</text>'
                    % (pad, y0 - 6, label))
        body.append(line(prediction['t'], prediction[key_pred], fx, fy, colour, dash=True))
        body.append(line(real[key_t], real[key_real], fx, fy, colour))
        body.append('<text x="%d" y="%.0f" font-size="11" fill="%s" font-family="sans-serif">'
                    '- - predicted     —— measured</text>' % (width - 240, y0 + 14, colour))
    with open(path, 'w') as handle:
        handle.write('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
                     'viewBox="0 0 %d %d">%s</svg>\n' % (width, height, width, height, ''.join(body)))


def _delta(predicted: float, actual: float) -> str:
    """A signed percentage difference, or 'n/a' when there is nothing measured to compare against."""
    if not actual:
        return '   n/a'
    return '%+5.0f%%' % (100.0 * (predicted - actual) / actual)


def main() -> int:
    parser = argparse.ArgumentParser(description='Compare a flight capture against the sim prediction.')
    parser.add_argument('capture', help='the recorder capture to measure')
    parser.add_argument('--motor', default='F15', choices=sorted(sim_model.MOTORS), help='motor (default F15)')
    parser.add_argument('--mass', type=float, default=471.0, help='liftoff mass, grams (default 471)')
    parser.add_argument('--glider', type=float, default=270.0, help='glider mass after separation, g')
    parser.add_argument('--seconds', type=float, default=240.0, help='prediction horizon (default 240)')
    parser.add_argument('--svg', help='also write a predicted-vs-measured overlay here')
    args = parser.parse_args()
    preflight.gate('prediction')

    with open(args.capture) as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    real = measured(streams)
    prediction = predict(args.motor, args.mass, args.glider, args.seconds)

    print('capture : %s' % args.capture)
    print('model   : %s, liftoff %.0f g -> glider %.0f g' % (args.motor, args.mass, args.glider))
    print()
    print('  metric            predicted    measured     delta')
    print('  ---------------------------------------------------')
    print('  peak |accel|      %7.1f g   %7.1f g   %s'
          % (prediction['peak_g'], real['peak_g'], _delta(prediction['peak_g'], real['peak_g'])))
    print('  apogee            %7.1f m   %7.1f m   %s'
          % (prediction['apogee_m'], real['apogee_m'], _delta(prediction['apogee_m'], real['apogee_m'])))
    if not real['accel_g'] and not real['elevation_m']:
        print('\n  NOTE: the capture carries neither accel nor baro -- nothing to compare against.')
    if args.svg:
        _svg(prediction, real, args.svg, 'predicted vs measured — %s, %.0f g' % (args.motor, args.mass))
        print('\nwrote %s' % args.svg)
    return 0


if __name__ == '__main__':
    sys.exit(main())
