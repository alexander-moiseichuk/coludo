# g15 Step 0 -- TOOLCHAIN GATE. Before any native/viper work: does THIS firmware's emitter compile and
# run @micropython.native (keeps float semantics) and @micropython.viper (int-only) on the P4 (RISC-V
# rv32)? Defines both next to a bytecode baseline, checks they return identical results, and times them.
# If the emitter is absent the @decorator raises at compile -> this test fails -> g15 falls back to a C
# natmod (mpy_ld.py). Run by run_tests_board.sh; the printed speedups feed doc/benches/.

import time

import micropython


def _between_upy(low, value, high):
    return low if value < low else (high if value > high else value)


@micropython.native
def _between_native(low, value, high):
    return low if value < low else (high if value > high else value)


def _clamp_upy(low, value, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


@micropython.viper
def _clamp_viper(low: int, value: int, high: int) -> int:
    if value < low:
        return low
    if value > high:
        return high
    return value


# correctness: native/viper must match the bytecode baseline exactly
assert _between_native(0.0, 5.0, 10.0) == 5.0
assert _between_native(0.0, -3.0, 10.0) == 0.0
assert _between_native(0.0, 99.0, 10.0) == 10.0
assert _clamp_viper(0, 5, 10) == 5
assert _clamp_viper(0, -3, 10) == 0
assert _clamp_viper(0, 99, 10) == 10

_N = 20000


def _bench(function, low, value, high):
    start = time.ticks_us()
    for _ in range(_N):
        function(low, value, high)
    return time.ticks_diff(time.ticks_us(), start)

# subtract the loop+call overhead is impractical here; report wall time + ratio (relative is the signal)
_us_upy_f = _bench(_between_upy, 0.0, 5.0, 10.0)
_us_nat_f = _bench(_between_native, 0.0, 5.0, 10.0)
_us_upy_i = _bench(_clamp_upy, 0, 5, 10)
_us_vip_i = _bench(_clamp_viper, 0, 5, 10)
print('native float: upy=%dus native=%dus (%.2fx) over %d calls' % (_us_upy_f, _us_nat_f, _us_upy_f / _us_nat_f, _N))
print('viper int:    upy=%dus viper=%dus (%.2fx) over %d calls' % (_us_upy_i, _us_vip_i, _us_upy_i / _us_vip_i, _N))
print('ok: native_gate -- @micropython.native + @micropython.viper compile and run on this firmware')
