# On-board test for sim_model.Body boost-attitude dynamics: a crosswind weathercocks the stack off
# vertical, control fins restore it, calm air stays vertical, and the new 'speed' sensor reports true
# airspeed. Pure math (deterministic -- no noise()), runs identically on host + board. Run by `make test`.

import sim_model


def _boost(mass, wind_e, gain):
    """Run an F15 burn with an east crosswind and a capped P-controller on pitch (gain 0 = no control);
    return (body, worst lean off vertical in deg)."""
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
    body = sim_model.Body(0.43, (25.5, -80.4), 2.0, 30.0)
    thrust, _burn = sim_model.MOTORS['F15']
    for _ in range(100):
        body.boost_step(0.01, thrust)
    assert abs(body.sensors()['speed'] - body.vu) < 0.5  # vertical climb -> airspeed ~ vertical speed


test_weathercock_and_control()
test_speed_sensor()
print('ok: sim_model -- boost weathercock vs fin restore, calm stays vertical, speed sensor')
