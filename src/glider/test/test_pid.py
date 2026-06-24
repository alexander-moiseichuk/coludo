# On-board test for the PID controller (pid.py): P/I/D terms, integral + output anti-windup clamps,
# and reset(). Pure math, no hardware. Run by `make test`.

import pid


def test_terms():
    # proportional
    assert pid.Pid(kp=2.0).step(3.0, 0.1) == 6.0

    # integral accumulates error*dt
    integ = pid.Pid(ki=1.0)
    assert abs(integ.step(2.0, 0.5) - 1.0) < 1e-9  # 2*0.5
    assert abs(integ.step(2.0, 0.5) - 2.0) < 1e-9  # +2*0.5

    # derivative on the error change
    deriv = pid.Pid(kd=1.0)
    deriv.step(0.0, 0.1)  # prime previous = 0
    assert abs(deriv.step(1.0, 0.1) - 10.0) < 1e-9  # (1-0)/0.1

    # first step after init/reset takes NO derivative -> no spike from a 0 baseline (finding 1.14.1)
    spike = pid.Pid(kd=1.0)
    assert spike.step(5.0, 0.1) == 0.0  # first step: D skipped (else it would be (5-0)/0.1 = 50)
    spike.reset()
    assert spike.step(9.0, 0.1) == 0.0  # first step after reset: again no derivative kick


def test_clamps_and_reset():
    # integral anti-windup
    integ = pid.Pid(ki=1.0, integral_limit=5.0)
    out = 0.0
    for _ in range(20):
        out = integ.step(10.0, 1.0)
    assert abs(out - 5.0) < 1e-9  # integral pinned at the limit

    # output clamp
    p = pid.Pid(kp=100.0, output_limit=45.0)
    assert p.step(10.0, 0.1) == 45.0 and p.step(-10.0, 0.1) == -45.0

    # reset clears the integral
    r = pid.Pid(ki=1.0)
    r.step(5.0, 1.0)
    r.reset()
    assert r.step(0.0, 1.0) == 0.0


test_terms()
test_clamps_and_reset()
print('ok: pid -- P/I/D terms, integral + output clamps, reset')
