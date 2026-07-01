# On-board test for the PID controller (pid.py): P/I/D terms, integral + output anti-windup clamps,
# and reset(). Pure integer math (fixed-point millidegrees in/out, integer-ms dt), no hardware -- so
# outputs are EXACT (no float epsilon). Run by `make test`.

import pid


def test_terms():
    # proportional: kp=2, error 3 deg (3000 mdeg) -> 6 deg (6000 mdeg)
    assert pid.Pid(kp=2.0).step(3000, 100) == 6000

    # integral accumulates error*dt (millidegree-seconds): 2 deg for 0.5 s -> 1 deg-s -> ki*1 = 1 deg
    integ = pid.Pid(ki=1.0)
    assert integ.step(2000, 500) == 1000  # 2000 mdeg * 500 ms // 1000 = 1000 mdeg-s -> 1000 mdeg out
    assert integ.step(2000, 500) == 2000  # + another 1000 -> 2 deg

    # derivative on the error change: (1 deg)/0.1 s = 10 deg/s -> kd*10 = 10 deg
    deriv = pid.Pid(kd=1.0)
    deriv.step(0, 100)  # prime previous = 0 (100 ms slice)
    assert deriv.step(1000, 100) == 10000  # (1000-0) mdeg * 1000 // 100 ms = 10000 mdeg/s -> 10 deg

    # first step after init/reset takes NO derivative -> no spike from a 0 baseline (finding 1.14.1)
    spike = pid.Pid(kd=1.0)
    assert spike.step(5000, 100) == 0  # first step: D skipped (else it would be (5-0)/0.1 = 50 deg)
    spike.reset()
    assert spike.step(9000, 100) == 0  # first step after reset: again no derivative kick

    # a full ±180 deg heading swing (worst case) stays integer + correctly signed, no overflow/mpz
    swing = pid.Pid(kp=1.5)
    assert swing.step(180000, 10) == 270000 and swing.step(-180000, 10) == -270000  # kp*180 = 270 deg


def test_clamps_and_reset():
    # integral anti-windup: limit 5 deg-s, hammered at 10 deg -> pinned, output ki*5 = 5 deg
    integ = pid.Pid(ki=1.0, integral_limit=5.0)
    out = 0
    for _ in range(20):
        out = integ.step(10000, 1000)  # 10 deg for 1 s each
    assert out == 5000  # integral pinned at the 5 deg-s limit -> 5 deg

    # output clamp: kp=100 on a 10 deg error is huge -> clamped to ±45 deg
    p = pid.Pid(kp=100.0, output_limit=45.0)
    assert p.step(10000, 100) == 45000 and p.step(-10000, 100) == -45000

    # reset clears the integral
    r = pid.Pid(ki=1.0)
    r.step(5000, 1000)
    r.reset()
    assert r.step(0, 1000) == 0


test_terms()
test_clamps_and_reset()
print('ok: pid -- fixed-point P/I/D terms, integral + output clamps, reset, ±180 swing')
