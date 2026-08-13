"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Overlay the SIMULATOR'S PREDICTION on a real flight capture (findings §27.15) -- the artifact that
either earns or destroys trust in `sim_model`.

Every landing-accuracy number in `doc/sims/` rests on the sim being right, and the sim has never been
drawn on the same axes as a real flight. This runs `sim_model` from the SAME initial conditions as a
capture and reports where the two diverge: apogee, duration, and the altitude/speed traces.

Two launch modes, because both matter now:

  * CATAPULT (the current glide ladder) -- give it the launch speed/angle/height and it predicts the
    ballistic arc plus the glide. Pair it with tools/glide_polar.py: measure L/D from the capture, feed
    it back as --quality, and the question becomes "does the sim reproduce the whole trajectory once
    its ONE free parameter is measured rather than guessed?"
  * ROCKET -- give it the motor and mass and it predicts boost -> coast -> glide.

A prediction that matches is evidence the sim's endgame/landing conclusions transfer. A prediction that
does NOT match is more valuable still: it says which term is wrong while there is still time to fix it.

  python3 tools/flight_predict.py capture.txt --launch-speed 8.2 --launch-angle 45 --quality 5.1
  python3 tools/flight_predict.py capture.txt --motor F15 --mass 0.45 -o predict.html
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))
import flight_telemetry  # noqa: E402
import preflight  # noqa: E402
import sim_model  # noqa: E402

_STEP_S: float = 0.01     # sim integration step; well under the sensor rates being compared against
_MAX_S: float = 600.0     # runaway guard for a prediction that never lands


def measured(streams) -> dict:
    """
    Pull the comparable traces out of a real capture.

    Args:
        streams - parsed telemetry streams.

    Returns:
        {'time': [...], 'elevation': [...], 'speed': ([t], [v]), 'apogee': m, 'duration': s}.
    """
    find = flight_telemetry.find_stream
    baro = find(streams, 'elevation', prefer='icp') or find(streams, 'elevation') or find(streams, 'altitude')
    if baro is None:
        return {}
    field = 'elevation' if 'elevation' in baro.fields else 'altitude'
    times, elevation = baro.column(field)
    if not times:
        return {}
    pitot = find(streams, 'dynamic_pressure')
    speed_t, speed_v = [], []
    if pitot is not None and 'airspeed_cms' in pitot.fields:
        speed_t, raw = pitot.column('airspeed_cms')
        speed_v = [v / 100.0 for v in raw]
    elif pitot is not None and 'airspeed' in pitot.fields:
        speed_t, speed_v = pitot.column('airspeed')
    return {'time': times, 'elevation': elevation, 'speed': (speed_t, speed_v),
            'apogee': max(elevation), 'duration': times[-1] - times[0]}


def predict(mode: str, speed: float, angle: float, height: float, motor: str,
            mass: float, quality: float) -> dict:
    """
    Run sim_model from the given launch condition and return the predicted traces.

    Args:
        mode - 'catapult' or 'rocket'.
        speed - catapult release speed (m/s).
        angle - catapult release angle (deg above horizontal).
        height - catapult release height (m).
        motor - rocket motor key (sim_model.MOTORS).
        mass - airframe mass (kg).
        quality - glide L/D; sim_model.trim_sink = 14 / quality.

    Returns:
        {'time': [...], 'elevation': [...], 'speed': ([t],[v]), 'apogee': m, 'duration': s}.
    """
    body = sim_model.Body(mass, (25.514379, -80.391795), 0.0, 0.0)
    body.trim_sink = 14.0 / quality
    times, elevation, speeds = [], [], []
    moment = 0.0
    if mode == 'rocket':
        thrust, burn = sim_model.MOTORS[motor]
        while moment < burn * 3 and body.alt >= 0.0 and moment < _MAX_S:
            body.boost_step(_STEP_S, thrust if moment < burn else 0.0)
            times.append(moment)
            elevation.append(body.alt)
            speeds.append(math.hypot(body.speed, body.vu))
            moment += _STEP_S
            if moment > burn and body.vu <= 0.0:  # apogee -> the glide begins
                break
        body.begin_glide()
    else:
        """
        CATAPULT: seed the body straight into the glide at the release condition. There is no boost to
        model -- the launch is over in 0.2 s, far shorter than anything the baro resolves, so it is an
        initial condition rather than a phase.
        """
        body.alt = height
        body.speed = speed * math.cos(math.radians(angle))
        body.vu = speed * math.sin(math.radians(angle))
        body.pitch = angle
        body.begin_glide()
    while body.alt > 0.0 and moment < _MAX_S:
        body.glide_step(_STEP_S, 0.0, 0.0, 0.0)  # no control input: the AIRFRAME's own glide
        times.append(moment)
        elevation.append(body.alt)
        speeds.append(math.hypot(body.speed, body.vu))
        moment += _STEP_S
    return {'time': times, 'elevation': elevation, 'speed': (times, speeds),
            'apogee': max(elevation) if elevation else 0.0,
            'duration': times[-1] - times[0] if times else 0.0}


def _delta(name: str, actual: float, expected: float, unit: str) -> str:
    """One comparison row: measured, predicted, and the error as both absolute and percent."""
    error = expected - actual
    percent = (100.0 * error / actual) if actual else float('nan')
    return '  %-12s measured %8.2f %-3s | predicted %8.2f %-3s | error %+7.2f (%+.1f%%)' % (
        name, actual, unit, expected, unit, error, percent)


def render(real: dict, sim: dict, path: str) -> None:
    """Write the overlay HTML (plotly), or explain why it was skipped."""
    try:
        import plotly.graph_objects as go
        import plotly.io as pio
        from plotly.subplots import make_subplots
    except ImportError:
        print('  (no plotly -- numbers only; pip install plotly for the overlay)')
        return
    figure = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.06,
                           subplot_titles=('altitude (m) — measured vs predicted',
                                           'speed (m/s) — measured vs predicted'))
    figure.add_trace(go.Scatter(x=real['time'], y=real['elevation'], name='measured'), row=1, col=1)
    figure.add_trace(go.Scatter(x=sim['time'], y=sim['elevation'], name='predicted',
                                line=dict(dash='dash')), row=1, col=1)
    if real['speed'][0]:
        figure.add_trace(go.Scatter(x=real['speed'][0], y=real['speed'][1], name='measured speed'),
                         row=2, col=1)
    figure.add_trace(go.Scatter(x=sim['speed'][0], y=sim['speed'][1], name='predicted speed',
                                line=dict(dash='dash')), row=2, col=1)
    figure.update_layout(height=820, hovermode='x unified',
                         title='predicted vs measured — does sim_model reproduce the real flight?')
    pio.write_html(figure, file=path, include_plotlyjs='cdn', auto_open=False)
    print('  wrote %s' % path)


def main() -> int:
    parser = argparse.ArgumentParser(description='Overlay the sim prediction on a real capture.')
    parser.add_argument('capture', help='the recorder capture to compare against')
    parser.add_argument('--launch-speed', type=float, default=None, help='catapult release speed (m/s)')
    parser.add_argument('--launch-angle', type=float, default=0.0, help='release angle above horizontal (deg)')
    parser.add_argument('--launch-height', type=float, default=1.0, help='release height (m)')
    parser.add_argument('--motor', default=None, choices=sorted(sim_model.MOTORS), help='rocket motor')
    parser.add_argument('--mass', type=float, default=0.176, help='airframe mass (kg, default 0.176)')
    parser.add_argument('--quality', type=float, default=sim_model.AIR_QUALITY,
                        help='glide L/D (default %.1f = sim_model.AIR_QUALITY, the measured airframe)'
                             % sim_model.AIR_QUALITY)
    parser.add_argument('-o', '--out', default=None, help='overlay HTML path')
    args = parser.parse_args()
    preflight.gate('prediction')
    if args.motor is None and args.launch_speed is None:
        return parser.error('give either --motor (rocket) or --launch-speed (catapult)')

    with open(args.capture) as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    real = measured(streams)
    if not real:
        print('cannot compare: the capture has no baro trace', file=sys.stderr)
        return 1
    mode = 'rocket' if args.motor else 'catapult'
    sim = predict(mode, args.launch_speed or 0.0, args.launch_angle, args.launch_height,
                  args.motor, args.mass, args.quality)

    print('%s  (%s launch, mass %.3f kg, L/D %.1f)' % (os.path.basename(args.capture), mode,
                                                       args.mass, args.quality))
    print(_delta('apogee', real['apogee'], sim['apogee'], 'm'))
    print(_delta('duration', real['duration'], sim['duration'], 's'))
    """
    A CONTROLLED flight is not an apples-to-apples comparison and must say so. The prediction flies the
    airframe's own straight glide with NO control input, while a guided capture turns, loiters and runs
    an endgame -- and every turn costs energy (induced drag scales n^1.5). So against a guided flight the
    prediction will over-predict duration, and that gap is the CONTROL cost, not sim error. The clean
    comparison is an unguided glide: exactly what the catapult ladder produces.
    """
    control = flight_telemetry.find_stream(streams, 'fin_cap')
    if control is not None:
        active = [v for v in control.column('active')[1] if v] if 'active' in control.fields else []
        if active:
            print('  NOTE: this capture was GUIDED for %d samples -- the prediction flies a straight,'
                  % len(active))
            print('        uncontrolled glide, so it will over-predict duration. The gap is the cost of')
            print('        turning, not sim error. Compare against an UNGUIDED glide for a clean check.')
    if args.quality == sim_model.AIR_QUALITY:
        print('  NOTE: L/D is the shipped sim_model.AIR_QUALITY (%.1f), measured by hand toss below trim'
              % sim_model.AIR_QUALITY)
        print('        speed -- a FLOOR. Measure this airframe with tools/glide_polar.py once it flies')
        print('        long enough for a real polar, and pass --quality.')
    if args.out:
        render(real, sim, args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
