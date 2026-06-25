# g15 performance bench for the commons primitives (and native-variant data for the float ones, to
# decide whether they are worth converting later). Times each function over N calls and reports ns/call
# + the speedup of the emitter variant over bytecode. NOT a correctness test (see test_commons); run
# on-board:  boardrun.py PORT runfile test/bench_commons.py 40
import math
import time

import commons
import micropython

_N = 30000


def _bench3(function, a, b, c):
    """Wall-clock ns/call for function(a, b, c) over _N calls (includes loop+call overhead, equal for
    every variant, so the RATIO between variants is the clean signal)."""
    start = time.ticks_us()
    for _ in range(_N):
        function(a, b, c)
    return time.ticks_diff(time.ticks_us(), start) * 1000 // _N


def _bench1(function, a):
    """ns/call for a single-argument function (wrap180)."""
    start = time.ticks_us()
    for _ in range(_N):
        function(a)
    return time.ticks_diff(time.ticks_us(), start) * 1000 // _N


# --- native variants of the FLOAT primitives, defined here only for measurement (commons stays viper-only) ---
@micropython.native
def _between_native(low, value, high):
    return low if value < low else (high if value > high else value)


def _magnitude_upy(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


@micropython.native
def _magnitude_native(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def _line(label, upy_ns, fast_ns):
    print('  %-22s bytecode=%4dns  fast=%4dns  %.2fx' % (label, upy_ns, fast_ns, upy_ns / max(fast_ns, 1)))


print('g15 commons bench -- %d calls each, ns/call (incl. fixed loop+call overhead):' % _N)
_line('clamp_int (viper)', _bench3(commons._clamp_int_upy, 0, 5, 10), _bench3(commons._clamp_int_viper, 0, 5, 10))
_line('wrap180 (viper)', _bench1(commons._wrap180_upy, 200), _bench1(commons._wrap180_viper, 200))
_line('between (native)', _bench3(commons.between, 0.0, 5.0, 10.0), _bench3(_between_native, 0.0, 5.0, 10.0))
_line('magnitude (native)', _bench3(_magnitude_upy, 3.0, 4.0, 12.0), _bench3(_magnitude_native, 3.0, 4.0, 12.0))
print('ok: bench_commons -- viper/native vs bytecode measured')
