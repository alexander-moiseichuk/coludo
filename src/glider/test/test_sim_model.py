"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for sim_model.Body boost-attitude dynamics: a crosswind weathercocks the stack off
vertical, control fins restore it, calm air stays vertical, and the 'speed' sensor reports the GNSS 2D
GROUND speed (with wind). Pure math (deterministic -- no noise()), runs identically host + board.
"""

import sim_model


def _boost(mass, wind_e, gain):
    """
    Run an F15 burn with an east crosswind and a capped P-controller on pitch (gain 0 = no control).

    Returns:
        (body, worst lean off vertical in deg).
    """
    body = sim_model.Body(mass, (25.5, -80.4), 2.0, 30.0)
    body.wind_e = wind_e
    thrust, burn = sim_model.MOTORS['F15']
    dt = 0.01
    worst_lean = 0.0
    for step in range(int(burn / dt)):
        pitch_cmd = max(-14.0, min(14.0, gain * (90.0 - body.pitch)))  # hold pitch -> 90 (vertical)
        body.boost_step(dt, thrust if step * dt < burn else 0.0, pitch_cmd)
        worst_lean = max(worst_lean, 90.0 - body.pitch)
    return body, worst_lean


def test_weathercock_and_control():
    _open, lean_open = _boost(0.43, 5.0, 0.0)      # uncontrolled: the crosswind leans it well off vertical
    assert lean_open > 8.0, lean_open
    _closed, lean_closed = _boost(0.43, 5.0, 0.6)  # control fins fight it -> a much smaller lean
    assert lean_closed < lean_open, (lean_closed, lean_open)
    _calm, lean_calm = _boost(0.43, 0.0, 0.0)      # no wind -> stays vertical
    assert lean_calm < 0.5, lean_calm


def test_speed_sensor():
    # the GNSS 'speed' is 2D GROUND speed (with wind), NOT 3D airspeed: a vertical boost has ~no
    # horizontal travel -> ground speed ~0 even climbing fast (what a real receiver reports on the rod).
    body = sim_model.Body(0.43, (25.5, -80.4), 2.0, 30.0)
    thrust, _burn = sim_model.MOTORS['F15']
    for _ in range(100):
        body.boost_step(0.01, thrust)
    assert body.vu > 5.0                          # climbing fast (vertical speed)
    assert body.sensors()['speed'] < 0.5          # but the GROUND speed is ~0 -- no horizontal motion
    # a glide heading east at 14 m/s in a 6 m/s east (along-track) wind -> ground speed = 14 + 6 = 20
    glide = sim_model.Body(0.43, (25.5, -80.4), 2.0, 90.0)
    glide.begin_glide()
    glide.speed, glide.heading, glide.wind_e = 14.0, 90.0, 6.0
    assert abs(glide.sensors()['speed'] - 20.0) < 0.1  # ground speed = airspeed + along-track wind


def test_gyro_rates():
    # glide angular rates (feed the PID D term): a roll fin command drives a roll rate of the sign of the
    # command, and a bank turns the heading -> a yaw rate. Right roll (+ cmd) -> + roll_rate -> + yaw_rate.
    body = sim_model.Body(0.43, (25.5, -80.4), 2.0, 30.0)
    body.begin_glide()
    body.speed = 14.0
    for _ in range(50):
        body.glide_step(0.02, 20.0, 0.0, 0.0)  # steady right-roll command
    sensors = body.sensors()
    assert body.roll > 5.0, body.roll                       # banked right
    assert sensors['yaw_rate'] > 0.0, sensors['yaw_rate']   # bank -> coordinated turn (+ heading rate)
    assert 'roll_rate' in sensors and 'pitch_rate' in sensors
    # rate ~ the modelled first-order response, finite (no NaN/huge); leveled bank -> small residual rate
    assert abs(sensors['roll_rate']) < 200.0, sensors['roll_rate']


def test_gnss_drift():
    # a STATIONARY receiver (not gliding) reports ONLY its GNSS drift as ground velocity -- a fixed
    # antenna is not carried by the wind, so the wind must NOT appear (this is what the pad calib reads)
    body = sim_model.Body(0.43, (25.5, -80.4), 2.0, 30.0)
    body.gnss_drift_e, body.gnss_drift_n = 0.3, 0.4
    body.wind_e = 5.0
    assert not body.gliding
    assert abs(body.ground_speed() - 0.5) < 1e-6  # |(0.3, 0.4)| = 0.5, wind excluded
    # airborne (gliding): the drift ADDS to the air + wind ground velocity
    body.begin_glide()
    body.speed, body.heading, body.wind_e, body.wind_n = 14.0, 90.0, 0.0, 0.0
    assert abs(body.ground_speed() - (14.3 ** 2 + 0.4 ** 2) ** 0.5) < 1e-6  # 14 + 0.3 E, 0.4 N


def test_stall():
    # the stall floor: a coordinated turn needs airspeed >= V_stall_1g*sqrt(load); below it the
    # wing stalls -> a hard sink break on top of induced drag.
    def _sink_after(bank, speed):  # vu after ONE step at a fixed bank/speed (a clean isolation)
        body = sim_model.Body(0.285, (48.0, 11.0), 100.0, 90.0)
        body.begin_glide()
        body.alt = 100.0  # above ground -> no ground-contact reset of vu
        body.roll, body.speed, body.vu = bank, speed, -7.0
        body.glide_step(0.02, 0.0, 0.0, 0.0)
        return body.vu

    # NEGATIVE: a trimmed straight glide (14 m/s, level) is ~1.5x above the 1-g stall -> normal trim sink.
    assert _sink_after(0.0, 14.0) > -7.5  # not stalled, no break
    # POSITIVE: a 45deg bank (stall speed ~10.7 m/s) flown at 8 m/s is STALLED and sinks HARDER than the
    # same bank flown at 14 m/s (above stall) -- the extra drop is the stall break, not just the load.
    assert _sink_after(45.0, 8.0) < _sink_after(45.0, 14.0) - 0.02, (_sink_after(45.0, 8.0), _sink_after(45.0, 14.0))


def test_drag_polar():
    """
    Sink follows the airspeed POLAR, not just the bank (findings §27.22).

    Normalised to 1.0 at trim; minimum sink sits BELOW trim (~0.76x) because minimum-sink speed is below
    best-glide speed on a real glider -- so flying well off trim costs energy in BOTH directions, which is
    what makes an airspeed-gated bank a genuine trade-off.
    """
    assert abs(sim_model._polar(sim_model._V_TRIM) - 1.0) < 1e-9  # exactly 1.0 at trim, by construction
    assert sim_model._polar(30.0) > 3.0  # way fast -> profile drag (v^3) dominates, sink climbs hard
    assert sim_model._polar(4.0) > 1.5   # way slow -> induced drag (1/v) dominates
    minimum = min(sim_model._polar(v / 10.0) for v in range(40, 260))  # scan 4..26 m/s
    assert minimum < 1.0, minimum  # min sink is cheaper than trim...
    assert sim_model._polar(0.76 * sim_model._V_TRIM) < sim_model._polar(sim_model._V_TRIM)  # ...and sits below it
    assert sim_model._polar(0.0) == sim_model._polar(sim_model._V_POLAR_FLOOR)  # floored, no 1/0


def test_gusts():
    """
    Gusts are OFF by default (every existing study reproduces), and an OU process when enabled.

    Correlated rather than white: a glider integrates gusts, so what matters is the slow wander.
    """
    body = sim_model.Body(0.43, (25.5, -80.4), 2.0, 30.0)
    assert body.gust == 0.0 and body._advance_gust(0.02) == (0.0, 0.0)  # default calm -> exactly zero
    body.gust = 2.0
    samples = [body._advance_gust(0.02)[0] for _ in range(3000)]
    spread = (sum(s * s for s in samples) / len(samples)) ** 0.5
    assert 0.5 * body.gust < spread < 2.0 * body.gust, spread  # right order, not white noise or runaway
    assert max(abs(s) for s in samples) < 10.0 * body.gust  # bounded: it decays toward zero, never walks off


def test_faults():
    """Fault injection: dead channels go None, but a saturated pitot RAILS (the dangerous under-read)."""
    assert not sim_model.Faults('')  # empty spec -> no faults, falsy
    faults = sim_model.Faults('gnss@30,baro')
    assert faults and not faults.active('gnss', 29.9) and faults.active('gnss', 30.0)
    assert faults.active('baro', 0.0)  # no '@' -> from t=0
    assert faults.apply('gnss', 12.0, 10.0) == 12.0 and faults.apply('gnss', 12.0, 31.0) is None
    assert faults.apply('laser', 3.0, 99.0) == 3.0  # a channel not in the spec is untouched
    # a pitot fault RAILS instead of going silent: a saturated cell under-reads, which LOOSENS the fin cap
    railed = sim_model.Faults('pitot')
    assert railed.apply('pitot', 14.0, 0.0) == sim_model.Faults.RAIL_PITOT_MS
    assert sim_model.Faults('junk@notanumber').active('junk', 0.0)  # a bad time degrades to t=0, never raises


test_weathercock_and_control()
test_speed_sensor()
test_gyro_rates()
test_gnss_drift()
test_stall()
test_drag_polar()
test_gusts()
test_faults()
print('ok: sim_model -- boost weathercock vs fin restore, calm stays vertical, speed sensor, gyro rates, '
      'gnss drift, stall, drag polar, gusts, fault injection')
