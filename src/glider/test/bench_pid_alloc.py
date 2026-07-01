# F01 allocation benchmark (not a make-test case; run by hand via mpremote).
# Measures GROSS heap allocation per PID step with GC DISABLED -- i.e. the exact in-flight leak
# rate (gc.disable() on BOOSTING, so nothing is freed and every alloc accumulates to OOM). Compares
# the current float Pid against integer fixed-point candidates (millidegree + centidegree), at both
# a realistic error and the worst case (180 deg heading error), and prices the call-site float->int
# conversion. Goal: pick the scale that (a) cuts the leak and (b) never spills into 16-byte mpz.

import gc

import pid

_N = 2000  # samples per measurement


def per_step(fn):
    """Mean gross bytes allocated per call of fn(), GC disabled (== the flight leak per step)."""
    gc.collect()
    gc.disable()
    base = gc.mem_alloc()
    for _ in range(_N):
        fn()
    used = gc.mem_alloc() - base
    gc.enable()
    return used / _N


class _FixedPid:
    """Integer fixed-point PID candidate. error in `unit` sub-degrees (1000 = mdeg, 100 = cdeg), dt in
    ms, gains scaled by _KU. Every intermediate stays a small int if the scale is chosen so products
    stay < 2**30 (the RV32 small-int ceiling); above it MicroPython boxes a 16-byte mpz per op."""

    _KU = 100  # gain scale: 0.01 resolution (kp 1.50 -> 150)

    def __init__(self, kp, ki, kd, integral_limit, output_limit, unit):
        self.kp = int(kp * self._KU)
        self.ki = int(ki * self._KU)
        self.kd = int(kd * self._KU)
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.unit = unit  # sub-degree ticks per degree in error / output
        self._integral = 0
        self._previous = None

    def step(self, error, dt_ms):
        # integral += error*dt in (sub-deg * s): the /1000 converts dt_ms->s. Clamped inline.
        integral = self._integral + error * dt_ms // 1000
        if integral > self.integral_limit:
            integral = self.integral_limit
        elif integral < -self.integral_limit:
            integral = -self.integral_limit
        self._integral = integral
        prev = self._previous
        if prev is None or dt_ms <= 0:
            derivative = 0
        else:
            derivative = (error - prev) * 1000 // dt_ms  # sub-deg per second
        self._previous = error
        out = (self.kp * error + self.ki * integral + self.kd * derivative) // self._KU
        if out > self.output_limit:
            out = self.output_limit
        elif out < -self.output_limit:
            out = -self.output_limit
        return out


def main():
    print('N = %d samples/measurement; GC disabled (flight leak rate)\n' % _N)

    # ---- baseline: the real float Pid, steady (derivative primed) ----
    fp = pid.Pid(kp=1.5, ki=0.2, kd=0.05, integral_limit=100.0, output_limit=45.0)
    fp.step(1.0, 0.01)
    print('float Pid.step   err=5deg   : %6.1f B/step' % per_step(lambda: fp.step(5.0, 0.01)))
    print('float Pid.step   err=180deg : %6.1f B/step' % per_step(lambda: fp.step(180.0, 0.01)))
    # call site today already does one float subtract (setpoint - actual) before step:
    print('float call-site  (s-a)      : %6.1f B/axis' % per_step(lambda: 30.0 - 25.0) + '\n')

    bench_unit(1000, 'mdeg')
    bench_unit(100, 'cdeg')


def bench_unit(unit, tag):
    """Fixed-point candidate at `unit` sub-degree ticks/degree: realistic error, ±180 swing (worst case
    for the derivative's *1000), and the call-site float->int conversion. All measured alloc-free."""
    fx = _FixedPid(1.5, 0.2, 0.05, 100 * unit, 45 * unit, unit)
    fx.step(1 * unit, 10)
    e_small = 5 * unit
    flip = [180 * unit, -180 * unit]
    idx = [0]

    def swing():  # alternate sign so the derivative term sees the full ±180 swing
        idx[0] ^= 1
        return fx.step(flip[idx[0]], 10)

    print('fixed %-4s err=5deg   dt=10ms: %6.1f B/step' % (tag, per_step(lambda: fx.step(e_small, 10))))
    print('fixed %-4s err=+-180 swing  : %6.1f B/step' % (tag, per_step(swing)))
    print('fixed %-4s call-site conv   : %6.1f B/axis\n' % (tag, per_step(lambda: int((30.5 - 25.25) * unit))))


main()
