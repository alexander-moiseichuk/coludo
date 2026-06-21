# On-board test for the control-surface mixer (mixer.py): the elevon + rudder mixing matrix, trim, and
# the +/- limit clamp. Pure math, no hardware. Run by `make test`.

import mixer


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


def test_clamp_and_trim():
    m = mixer.Mixer()
    # the control deflection is clamped to +/- limit (45) even when commanded past it
    assert m.mix(pitch=100)['servo_eleron_left'] == 135  # 90 + 45
    assert m.mix(yaw=-100)['servo_yaw'] == 45  # 90 - 45
    # a combined deflection over the limit is clamped as a whole (total surface authority bounded)
    assert m.mix(pitch=40, roll=40)['servo_eleron_left'] == 135  # 80 -> clamp 45 -> 90 + 45

    # trim is the neutral offset, applied on top of the clamped deflection
    trimmed = mixer.Mixer({'neutral_deg': 90, 'limit_deg': 30, 'trim': {'servo_yaw': 4},
                           'surfaces': {'servo_yaw': {'yaw': 1}}})
    assert trimmed.mix() == {'servo_yaw': 94}  # neutral + trim
    assert trimmed.mix(yaw=100) == {'servo_yaw': 124}  # 90 + 4 + clamp(100, 30)
    assert trimmed.mix(yaw=-100) == {'servo_yaw': 64}  # 90 + 4 - 30


test_default_mixing()
test_clamp_and_trim()
print('ok: mixer -- elevon + rudder mixing, trim, +/- limit clamp, integer angles')
