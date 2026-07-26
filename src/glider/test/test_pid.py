"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the PID controller (pid.py): P/I/D terms, integral + output anti-windup clamps, and
reset(). Fixed-point integer math (error/output in fixed.fixnum, integer-ms dt) -- so outputs are EXACT.
Values go through fixed.from_float, so the test is independent of fixed.SCALE (survives a 100->1000
bump). Run by `make test`.
"""

import fixed
import pid


def test_terms():
    # proportional: kp=2, error 3° -> 6°
    assert pid.Pid(kp=2.0).step(fixed.from_float(3), 100) == fixed.from_float(6)

    # integral accumulates error*dt: 2° for 0.5 s -> 1 °·s -> ki*1 = 1°
    integ = pid.Pid(ki=1.0)
    assert integ.step(fixed.from_float(2), 500) == fixed.from_float(1)
    assert integ.step(fixed.from_float(2), 500) == fixed.from_float(2)  # + another 1 °·s

    # derivative on the error change: (1°)/0.1 s = 10 °/s -> kd*10 = 10°
    deriv = pid.Pid(kd=1.0)
    deriv.step(0, 100)  # prime previous = 0 (100 ms slice)
    assert deriv.step(fixed.from_float(1), 100) == fixed.from_float(10)

    # derivative-on-measurement: a supplied gyro rate feeds the D term directly (negated), no
    # attitude differentiation and no first-step guard needed
    rated = pid.Pid(kd=1.0)
    assert rated.step(0, 100, rate=fixed.from_float(10)) == fixed.from_float(-10)   # kd·(-10 °/s) = -10
    assert rated.step(0, 100, rate=fixed.from_float(-4)) == fixed.from_float(4)

    # first step after init/reset takes NO derivative -> no spike from a 0 baseline
    spike = pid.Pid(kd=1.0)
    assert spike.step(fixed.from_float(5), 100) == 0
    spike.reset()
    assert spike.step(fixed.from_float(9), 100) == 0

    # a full ±180° heading swing (worst case) stays integer + correctly signed, no overflow/mpz
    swing = pid.Pid(kp=1.5)
    assert swing.step(fixed.from_float(180), 10) == fixed.from_float(270)   # kp*180 = 270°
    assert swing.step(fixed.from_float(-180), 10) == fixed.from_float(-270)


def test_clamps_and_reset():
    # integral anti-windup: limit 5 °·s, hammered at 10° -> pinned, output ki*5 = 5°
    integ = pid.Pid(ki=1.0, integral_limit=5.0)
    out = 0
    for _ in range(20):
        out = integ.step(fixed.from_float(10), 1000)  # 10° for 1 s each
    assert out == fixed.from_float(5)

    # output clamp: kp=100 on a 10° error is huge -> clamped to ±45°
    clamp_pid = pid.Pid(kp=100.0, output_limit=45.0)
    assert clamp_pid.step(fixed.from_float(10), 100) == fixed.from_float(45)
    assert clamp_pid.step(fixed.from_float(-10), 100) == fixed.from_float(-45)

    # reset clears the integral
    reset_pid = pid.Pid(ki=1.0)
    reset_pid.step(fixed.from_float(5), 1000)
    reset_pid.reset()
    assert reset_pid.step(0, 1000) == 0


def test_set_limit():
    # set_limit retunes BOTH clamps live -> the output tracks the governor's tightened fin cap
    clamp_pid = pid.Pid(kp=100.0, output_limit=45.0)
    assert clamp_pid.step(fixed.from_float(10), 100) == fixed.from_float(45)  # wide cap -> 45
    clamp_pid.set_limit(5)  # governor tightens authority to 5° at high q
    assert clamp_pid.step(fixed.from_float(10), 100) == fixed.from_float(5)   # tracks the live 5° cap
    clamp_pid.set_limit(45)  # cap reopens
    assert clamp_pid.step(fixed.from_float(10), 100) == fixed.from_float(45)

    # the integral clamp tracks the cap too -> no wind-up behind a tight cap that dumps on reopen
    integ = pid.Pid(ki=1.0, integral_limit=45.0)
    integ.set_limit(5)  # tight cap
    out = 0
    for _ in range(20):
        out = integ.step(fixed.from_float(10), 1000)  # hammer 10°/s -> integral pinned at the 5° cap
    assert out == fixed.from_float(5)  # ki*5, NOT ki*45 -> no windup
    integ.set_limit(45)  # cap reopens -- the integral was held at 5, so no saturating dump
    assert integ.step(0, 1000) == fixed.from_float(5)  # still ki*5, not a 45° spike


def test_back_calculation():
    """
    findings §23.5: while the fin is SATURATED the integral must unwind, not sit at its clamp.

    The clamps bound magnitude but never bleed, so a clamp-only loop dumps full deflection the moment
    authority returns. Deep saturation here is the warm-start case: a fin left ~90° to the airflow is an
    airbrake, so the loop restarts against a large sustained disturbance under a tight cap.
    """
    def saturate(shift):
        axis = pid.Pid(kp=1.0, ki=2.0, integral_limit=45, output_limit=5, anti_windup_shift=shift)
        for _ in range(50):
            axis.step(fixed.from_float(30), 20)  # 30° error against a 5° cap -> deeply saturated
        wound = axis._integral
        axis.set_limit(45)                       # authority returns
        return wound, axis.step(0, 20)

    wound, dump = saturate(2)                    # back-calculation on
    assert wound == 0, wound                     # unwound, not pinned
    assert dump == 0, dump                       # nothing to dump

    pinned, spike = saturate(30)                 # shift so large the bleed truncates to 0 -> clamp-only
    assert pinned == fixed.from_float(30)        # the old behaviour: integral sits at the clamp
    assert spike == fixed.from_float(45)         # ...and dumps FULL deflection at zero error

    # the unwinding STOPS AT ZERO -- it must never cross into an opposite-sign command (a reversed fin).
    # Textbook back-calculation settles the integral at -1250 here, because P alone oversaturates 6x.
    reversal = pid.Pid(kp=1.0, ki=2.0, integral_limit=45, output_limit=5)
    for _ in range(200):
        reversal.step(fixed.from_float(30), 20)
        assert reversal._integral >= 0, reversal._integral

    # NEGATIVE: back-calculation must not disable the I term when the output is NOT saturated
    free = pid.Pid(ki=2.0, integral_limit=45, output_limit=45)
    for _ in range(50):
        free.step(fixed.from_float(1), 20)
    assert free._integral > 0 and free.step(fixed.from_float(1), 20) > 0

    # NEGATIVE: ki == 0 -> no integral to unwind, and no division by zero
    assert pid.Pid(kp=100.0, output_limit=5).step(fixed.from_float(30), 20) == fixed.from_float(5)


test_terms()
test_clamps_and_reset()
test_set_limit()
test_back_calculation()
print('ok: pid -- fixed-point P/I/D, integral + output clamps, reset, ±180 swing, live set_limit '
      '(SCALE-agnostic), back-calculation anti-windup')
