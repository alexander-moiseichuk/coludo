# performance bench for the commons primitives: each function's _opt (viper int / native float) vs
# its _upy bytecode reference, ns/call + speedup. NOT a correctness test (see test_commons). Run
# on-board: boardrun.py PORT runfile test/bench_commons.py 40
import time

import commons

_N = 30000


def _bench3(function, a, b, c):
    """Wall-clock ns/call for function(a, b, c) over _N calls (fixed loop+call overhead is equal across
    variants, so the RATIO between _upy and _opt is the clean signal)."""
    start = time.ticks_us()
    for _ in range(_N):
        function(a, b, c)
    return time.ticks_diff(time.ticks_us(), start) * 1000 // _N


def _bench1(function, a):
    start = time.ticks_us()
    for _ in range(_N):
        function(a)
    return time.ticks_diff(time.ticks_us(), start) * 1000 // _N


def _line(label, upy_ns, opt_ns):
    print('  %-22s bytecode=%4dns  opt=%4dns  %.2fx' % (label, upy_ns, opt_ns, upy_ns / max(opt_ns, 1)))


print(' commons bench -- %d calls each, ns/call (incl. fixed loop+call overhead):' % _N)
_line('clamp_int (viper)', _bench3(commons.clamp_int_upy, 0, 5, 10), _bench3(commons.clamp_int_opt, 0, 5, 10))
_line('wrap180 (viper)', _bench1(commons.wrap180_upy, 200), _bench1(commons.wrap180_opt, 200))
_line('between (native)', _bench3(commons.between_upy, 0.0, 5.0, 10.0), _bench3(commons.between_opt, 0.0, 5.0, 10.0))
_line('magnitude_sq (native)',
      _bench3(commons.magnitude_sq_upy, 3.0, 4.0, 12.0), _bench3(commons.magnitude_sq_opt, 3.0, 4.0, 12.0))
_line('bank_demand (native)',
      _bench3(commons.bank_demand_upy, 90.0, 1.5, 30.0), _bench3(commons.bank_demand_opt, 90.0, 1.5, 30.0))
print('ok: bench_commons -- viper(int) + native(float) vs bytecode measured')
