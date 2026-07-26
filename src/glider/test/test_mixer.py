"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the control-surface mixer (mixer.py): the elevon + rudder mixing matrix, the +/-
limit clamp, and the fused bind()/actuate() servo path the flight loop drives. Run by `make test`.
"""

import mixer


class _Fin:
    """set_angle() stand-in for an sg90 driver (records the last commanded angle)."""

    def __init__(self):
        self.angle = None

    def settle(self):
        pass

    def set_angle(self, angle):
        self.angle = angle
        return angle


def test_default_mixing():
    m = mixer.Mixer()  # default elevon + rudder layout, neutral 90, limit 45

    # neutral -> every fin at 90; neutralise() == zero-deflection mix
    assert m.mix() == {'servo_yaw': 90, 'servo_eleron_left': 90, 'servo_eleron_right': 90}
    assert m.neutralise() == m.mix(0, 0, 0)

    # pitch -> both elevons together, rudder unaffected
    out = m.mix(pitch=10)
    assert out['servo_eleron_left'] == 100 and out['servo_eleron_right'] == 100 and out['servo_yaw'] == 90

    # roll -> elevons differential
    out = m.mix(roll=10)
    assert out['servo_eleron_left'] == 100 and out['servo_eleron_right'] == 80

    # yaw -> rudder only
    out = m.mix(yaw=15)
    assert out['servo_yaw'] == 105 and out['servo_eleron_left'] == 90

    # combined: left = 90 + pitch + roll, right = 90 + pitch - roll
    out = m.mix(roll=5, pitch=10)
    assert out['servo_eleron_left'] == 105 and out['servo_eleron_right'] == 95

    # integer angles out
    assert all(isinstance(v, int) for v in m.mix(roll=3, pitch=7, yaw=2).values())


def test_clamp():
    m = mixer.Mixer()
    # the control deflection is clamped to +/- limit (45) even when commanded past it
    assert m.mix(pitch=100)['servo_eleron_left'] == 135  # 90 + 45
    assert m.mix(yaw=-100)['servo_yaw'] == 45  # 90 - 45
    # a combined deflection over the limit is clamped as a whole (total surface authority bounded)
    assert m.mix(pitch=40, roll=40)['servo_eleron_left'] == 135  # 80 -> clamp 45 -> 90 + 45

    # a custom neutral/limit rides the clamp; mechanical zero is the sg90 driver's job, not the mixer
    tight = mixer.Mixer({'neutral_deg': 90, 'limit_deg': 30, 'surfaces': {'servo_yaw': {'yaw': 1}}})
    assert tight.mix() == {'servo_yaw': 90}  # common neutral, no per-fin offset
    assert tight.mix(yaw=100) == {'servo_yaw': 120}  # 90 + clamp(100, 30)
    assert tight.mix(yaw=-100) == {'servo_yaw': 60}  # 90 - 30


def test_bind_and_actuate():
    m = mixer.Mixer()
    assert m.bound is False
    m.actuate(0, 10, 0)  # not bound yet -> drives nothing (safe no-op, never raises)

    fins = {name: _Fin() for name in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}
    m.bind(fins)
    assert m.bound is True

    # actuate == mix + set_angle in one loop: same numbers as the mix() cases above
    m.actuate(10, 0, 0)  # roll -> elevons differential, rudder neutral
    assert fins['servo_eleron_left'].angle == 100 and fins['servo_eleron_right'].angle == 80
    assert fins['servo_yaw'].angle == 90
    m.actuate(0, 0, 15)  # yaw -> rudder only
    assert fins['servo_yaw'].angle == 105 and fins['servo_eleron_left'].angle == 90
    m.actuate(0, 100, 0)  # clamped to +/- limit (45) like mix()
    assert fins['servo_eleron_left'].angle == 135

    # a surface with no resolved fin is skipped (bind tolerance); the bound ones still drive
    partial = mixer.Mixer()
    left = _Fin()
    partial.bind({'servo_eleron_left': left, 'servo_yaw': None})
    partial.actuate(0, 10, 0)
    assert left.angle == 100


test_default_mixing()
test_clamp()
test_bind_and_actuate()
print('ok: mixer -- elevon + rudder mixing, +/- limit clamp, integer angles, bind/actuate fusion')
