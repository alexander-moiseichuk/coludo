"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the HITL simulator (tasks/hitl.py + config_hitl.py): the pure flight Body physics
(boost -> apogee -> fin-controlled glide, a roll command turns the heading) and that the Hitl task is
registered and config_hitl produces a valid, correctly-wired config. Run by `make test`.
"""

import math

import config
import config_hitl
import fixed
import layout
import task
from tasks import hitl


def test_body():
    body = hitl.Body(0.43, (25.514379, -80.391795), 2.0, 30.0)
    dt = 0.02
    # boost: thrust climbs the body and the accelerometer sees > launch_g (3 g) -> launch is detectable
    peak_g = 0.0
    for _ in range(int(1.77 / dt)):  # ~F15 burn
        body.boost_step(dt, 14.4)
        peak_g = max(peak_g, body.accel_g)
    assert peak_g > 3.0, peak_g          # boost reads above the launch threshold
    assert body.alt > 5.0 and body.vu > 0.0    # climbing

    # coast to apogee
    while body.vu > 0.0:
        body.boost_step(dt, 0.0)
    apogee = body.alt
    assert apogee > 20.0, apogee         # reached real altitude

    # glide: wings level -> descends roughly straight; a right-roll command turns the heading right
    body.begin_glide()
    assert body.gliding
    h0, alt0 = body.heading, body.alt
    for _ in range(200):
        body.glide_step(dt, 0.0, 0.0, 0.0)
    assert body.alt < alt0                                      # losing altitude
    assert abs(((body.heading - h0 + 180) % 360) - 180) < 30   # ~straight (only small drift)
    h1 = body.heading
    for _ in range(100):
        body.glide_step(dt, 20.0, 0.0, 0.0)                     # sustained right roll
    assert ((body.heading - h1) % 360) > 5.0                    # turned right

    # the position tracks away from the pad as it flies (lat/lon move)
    assert body.position() != (body.lat0, body.lon0)


def test_wiring():
    assert task.ACTIVITIES.get('hitl') is hitl.Hitl         # registered driver
    cfg = config_hitl.default(motor='E16', noise=0.1)
    assert config.validate(cfg) == [], config.validate(cfg)  # the HITL config validates clean

    sensors = {s['name']: s['enabled'] for s in cfg['sensors']}
    assert sensors['imu_bno055'] is False and sensors['laser_agl'] is False  # real sensors off
    """
    EVERY sensor the sim publishes over must be masked, the v1.1 attitude module included. A real part
    left enabled does not fail loudly -- it publishes plausible, perfectly FRESH bench data into a
    simulated flight, which is how a still-air pitot once became the only airspeed source in the whole
    matrix. The magnetometer is the worst of them: stationary, it reads a constant heading forever.
    """
    for name in ('imu_bmi323', 'mag_bmm350', 'baro_bmp581', 'airspeed_sdp810'):
        assert sensors.get(name) is False, '%s must be masked in HITL' % name
    """
    ...and it must stay masked AFTER the layout, on every revision. layout.apply() sets `enabled` from what
    the revision fits, so hitl_run's resolve re-enabled the bench parts (a v1.0 board flew on its real
    BNO055 + BMP280, a v1.1 on its BMI323 + BMM350 + BMP581) and an L1X laser was never masked at all.
    Checked by CHANNEL, not by name: no enabled driver may publish anything the sim publishes.
    """
    simulated = ('accel', 'attitude', 'rate', 'agl', 'altitude', 'elevation', 'position', 'speed',
                 'course', 'airspeed', 'dynamic_pressure', 'mag')
    for revision in layout._REVISIONS:
        resolved = config_hitl.default(motor='E16')
        layout.apply(resolved, revision)
        config_hitl.mask(resolved)
        for sensor in resolved['sensors']:
            if sensor.get('driver') and sensor.get('enabled', True):
                clash = [name for name in (sensor.get('provides') or {}) if name in simulated]
                assert not clash, '%s: real %s publishes simulated %s' % (revision, sensor['name'], clash)
    comp = {c['name']: c for c in cfg['components']}
    assert comp['hitl']['enabled'] and comp['hitl']['noise'] == 0.1 and comp['hitl']['motor'] == 'E16'
    assert comp['flight']['enabled'] and comp['watchdog']['enabled'] is False
    assert comp['servo_yaw']['enabled']                      # servos stay on (the sim reads the fins)


def test_simulated_mag_round_trips():
    """
    The sim's magnetometer must decode back to the heading it was built from, THROUGH the consumer's own
    formula (heading = atan2(-my, mx)) rather than a copy of it -- a sim that invents its own frame grades
    the harness instead of the flight code, which has happened here before.
    """
    for heading in (0.0, 45.0, 137.0, 180.0, 271.0, 359.0):
        magnetic = math.radians(heading + hitl._MAG_DECLINATION_DEG)
        mag = (int(hitl._MAG_UNIT * math.cos(magnetic)), int(-hitl._MAG_UNIT * math.sin(magnetic)))
        recovered = fixed.to_float(fixed.atan2_cd(-mag[1], mag[0]) % 36000)
        expected = (heading + hitl._MAG_DECLINATION_DEG) % 360.0
        error = abs(((recovered - expected + 180.0) % 360.0) - 180.0)
        assert error < 0.5, 'heading %.0f -> %.2f, wanted %.2f' % (heading, recovered, expected)

    # and the declination must actually BE an offset -- a sim publishing true heading would let the
    # learned-offset path pass while never exercising it
    assert hitl._MAG_DECLINATION_DEG != 0.0


def test_noise():
    # clean (frac 0) passes through; noisy stays within the clamp bounds
    assert hitl._noisy(50.0, 0.0, 0.0, 360.0) == 50.0
    for _ in range(200):
        assert 0.0 <= hitl._noisy(2.0, 0.5, 0.0, 360.0) <= 360.0


test_body()
test_wiring()
test_simulated_mag_round_trips()
test_noise()
print('ok: hitl -- 6-DoF body (boost/apogee/glide/turn), config_hitl wiring + validation, '
      'simulated mag round-trip, noise bounds')
