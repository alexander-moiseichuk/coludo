"""
Does this BMM350 need calibrating? Rotate the airframe and find out.

A magnetometer in clean air traces a CIRCLE as you turn it: constant field strength, only the direction
changing. What the airframe does to that circle is the whole question --

  * the centre displaced from the origin is HARD IRON: a fixed field from the airframe itself (magnets in
    the servos, current in a wire, steel). It biases heading by a direction-DEPENDENT amount, which is why
    it cannot be removed by a single offset in the heading;
  * an ELLIPSE rather than a circle is soft iron plus per-axis sensitivity. The learned track offset in
    attitude.py cannot absorb either, because both change with which way the nose points.

attitude.py's magnetic yaw learns ONE offset from the GNSS track, which handles declination, mounting and
the average hard-iron bias -- so a modest circle offset costs nothing. This tells you whether you are in
that case or the other one, before a flight depends on it.

    mpremote connect $PORT run src/glider/test/diag_mag_calibration.py

WARNING: turn the airframe SLOWLY through at least one full revolution, keeping it LEVEL -- the heading
path only trusts the mag near level, so that is the attitude worth characterising.
"""

import asyncio
import math

import config
import layout
import main

_SECONDS: int = 30  # long enough for an unhurried full turn
_PERIOD_MS: int = 100


async def run():
    board, source, _errors = config.load()
    revision = layout.resolve(board, log=lambda line: None)
    flight = await main.bringup(board, log=lambda line: None)
    unit = flight.active('mag_bmm350')
    if unit is None:
        print('mag_bmm350 is not fitted on this board (layout %s) -- nothing to characterise' % revision)
        return

    print('TURN THE BOARD SLOWLY THROUGH A FULL CIRCLE, KEEPING IT LEVEL -- %d s' % _SECONDS)
    samples = []
    for step in range(_SECONDS * 1000 // _PERIOD_MS):
        try:
            samples.append(await unit._read())
        except Exception as error:
            print('read failed: %r' % error)
            return
        if step % (5000 // _PERIOD_MS) == 0:
            print('  %2d s ...' % (step * _PERIOD_MS // 1000))
        await asyncio.sleep_ms(_PERIOD_MS)

    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    x_min, x_max, y_min, y_max = min(xs), max(xs), min(ys), max(ys)
    x_centre, y_centre = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
    x_radius, y_radius = (x_max - x_min) / 2.0, (y_max - y_min) / 2.0
    radius = (x_radius + y_radius) / 2.0

    print('')
    print('%d samples' % len(samples))
    print('x: %8d .. %8d   y: %8d .. %8d' % (x_min, x_max, y_min, y_max))
    if radius <= 0:
        print('NO ROTATION SEEN -- the field never moved. Turn the board while this runs.')
        return
    print('centre offset (hard iron): x %+.0f  y %+.0f  = %.0f%% of the radius' % (
        x_centre, y_centre, 100.0 * math.sqrt(x_centre ** 2 + y_centre ** 2) / radius))
    ellipticity = abs(x_radius - y_radius) / radius
    print('axis ratio (soft iron / scale): %.2f  -> %.0f%% out of round' % (
        x_radius / y_radius if y_radius else 0.0, 100.0 * ellipticity))

    """
    The verdict is about HEADING error, not about tidiness. A hard-iron offset of h radii produces a
    heading error of about asin(h) at worst, and an axis ratio of (1+e) about e/2 radians at 45 deg --
    both direction-dependent, so neither is removed by attitude.py's single learned offset.
    """
    hard = math.sqrt(x_centre ** 2 + y_centre ** 2) / radius
    worst = math.degrees(math.asin(min(1.0, hard))) + math.degrees(ellipticity / 2.0)
    print('')
    print('estimated worst-case heading error: ~%.0f deg' % worst)
    if worst < 5:
        print('VERDICT: usable uncalibrated -- inside the ~5 deg that keeps dead reckoning in the zone.')
    elif worst < 15:
        print('VERDICT: marginal. Usable to BOUND gyro drift, not to steer on. Calibration would pay.')
    else:
        print('VERDICT: calibrate before trusting it. The airframe is distorting the field more than the'
              ' heading can absorb.')
    print('DONE')


asyncio.run(run())
