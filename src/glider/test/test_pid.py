# On-board test for the PID controller (pid.py): P/I/D terms, integral + output anti-windup clamps, and
# reset(). Fixed-point integer math (error/output in fixed.fixnum, integer-ms dt) -- so outputs are EXACT.
# Values go through fixed.from_float, so the test is independent of fixed.SCALE (survives a 100->1000
# bump). Run by `make test`.

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

    # first step after init/reset takes NO derivative -> no spike from a 0 baseline (finding 1.14.1)
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
    p = pid.Pid(kp=100.0, output_limit=45.0)
    assert p.step(fixed.from_float(10), 100) == fixed.from_float(45)
    assert p.step(fixed.from_float(-10), 100) == fixed.from_float(-45)

    # reset clears the integral
    r = pid.Pid(ki=1.0)
    r.step(fixed.from_float(5), 1000)
    r.reset()
    assert r.step(0, 1000) == 0


test_terms()
test_clamps_and_reset()
print('ok: pid -- fixed-point P/I/D terms, integral + output clamps, reset, ±180 swing (SCALE-agnostic)')
