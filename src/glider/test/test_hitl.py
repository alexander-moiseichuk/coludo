# On-board test for the HITL simulator (tasks/hitl.py + config_hitl.py, 6/23 g15): the pure flight
# Body physics (boost -> apogee -> fin-controlled glide, a roll command turns the heading) and that the
# Hitl task is registered and config_hitl produces a valid, correctly-wired config. Run by `make test`.

import config
import config_hitl
import task
from tasks import hitl


def test_body():
    b = hitl.Body(0.43, (25.514379, -80.391795), 2.0, 30.0)
    dt = 0.02
    # boost: thrust climbs the body and the accelerometer sees > launch_g (3 g) -> launch is detectable
    peak_g = 0.0
    for _ in range(int(1.77 / dt)):  # ~F15 burn
        b.boost_step(dt, 14.4)
        peak_g = max(peak_g, b.accel_g)
    assert peak_g > 3.0, peak_g          # boost reads above the launch threshold
    assert b.alt > 5.0 and b.vu > 0.0    # climbing

    # coast to apogee
    while b.vu > 0.0:
        b.boost_step(dt, 0.0)
    apogee = b.alt
    assert apogee > 20.0, apogee         # reached real altitude

    # glide: wings level -> descends roughly straight; a right-roll command turns the heading right
    b.begin_glide()
    assert b.gliding
    h0, alt0 = b.heading, b.alt
    for _ in range(200):
        b.glide_step(dt, 0.0, 0.0, 0.0)
    assert b.alt < alt0                                      # losing altitude
    assert abs(((b.heading - h0 + 180) % 360) - 180) < 30   # ~straight (only small drift)
    h1 = b.heading
    for _ in range(100):
        b.glide_step(dt, 20.0, 0.0, 0.0)                     # sustained right roll
    assert ((b.heading - h1) % 360) > 5.0                    # turned right

    # the position tracks away from the pad as it flies (lat/lon move)
    assert b.position() != (b.lat0, b.lon0)


def test_wiring():
    assert task.ACTIVITIES.get('hitl') is hitl.Hitl         # registered driver
    cfg = config_hitl.default(motor='E16', noise=0.1)
    assert config.validate(cfg) == [], config.validate(cfg)  # the HITL config validates clean

    sensors = {s['name']: s['enabled'] for s in cfg['sensors']}
    assert sensors['imu_bno055'] is False and sensors['laser_agl'] is False  # real sensors off
    comp = {c['name']: c for c in cfg['components']}
    assert comp['hitl']['enabled'] and comp['hitl']['noise'] == 0.1 and comp['hitl']['motor'] == 'E16'
    assert comp['flight']['enabled'] and comp['watchdog']['enabled'] is False
    assert comp['servo_yaw']['enabled']                      # servos stay on (the sim reads the fins)


def test_noise():
    # clean (frac 0) passes through; noisy stays within the clamp bounds
    assert hitl._noisy(50.0, 0.0, 0.0, 360.0) == 50.0
    for _ in range(200):
        assert 0.0 <= hitl._noisy(2.0, 0.5, 0.0, 360.0) <= 360.0


test_body()
test_wiring()
test_noise()
print('ok: hitl -- 6-DoF body (boost/apogee/glide/turn), config_hitl wiring + validation, noise bounds')
