"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

What a FLOAT costs against an INT on this build -- the measurement behind any float->fixnum migration.

MicroPython heap-boxes every float while small ints (< 2^30) are unboxed immediates, so with GC off in
flight every float that crosses the control path is permanent leak until landing. This prices that,
per value and per second, so the decision to migrate a channel is arithmetic rather than taste.

Run: mpremote connect PORT exec "import sys; sys.path.insert(0,'/test'); \
     import diag_float_box; diag_float_box.probe()"
"""

import gc
import time

_ROUNDS: int = 2000  # enough that per-op bytes resolve; small enough not to exhaust a ballasted heap


def _cost(make, rounds: int) -> tuple:
    """
    Bytes and microseconds per call of `make`, with GC off (the flight-slice condition).

    Args:
        make - a zero-argument callable producing one value.
        rounds - how many times to call it.

    Returns:
        (bytes_per_call, us_per_call).
    """
    gc.collect()
    gc.disable()
    before, started = gc.mem_alloc(), time.ticks_us()
    sink = None
    for _ in range(rounds):
        sink = make()
    elapsed = time.ticks_diff(time.ticks_us(), started)
    used = gc.mem_alloc() - before
    gc.enable()
    return used / rounds, elapsed / rounds, sink


def probe() -> None:
    """Report the per-value heap cost of floats vs ints across the operations the control path uses."""
    base_f, base_i, step = 288.125, 28812, 7
    cases = (
        ('float add', lambda: base_f + 0.5),
        ('int add', lambda: base_i + step),
        ('float multiply', lambda: base_f * 1.5),
        ('int multiply', lambda: base_i * 3),
        ('float divide', lambda: base_f / 4.0),
        ('int floordiv', lambda: base_i // 4),
        ('float from int', lambda: float(base_i)),
        ('int from float', lambda: int(base_f * 100)),
        ('float tuple push', lambda: (base_f, base_f + 1.0)),
        ('int tuple push', lambda: (base_i, base_i + 100)),
    )
    print('op                    bytes/call    us/call')
    for name, make in cases:
        used, micros, _sink = _cost(make, _ROUNDS)
        print('%-20s %10.2f %10.2f' % (name, used, micros))
    print()
    print('A boxed float is the difference between the float and int rows. Multiply by the number of')
    print('float values a channel pushes per second to price migrating it.')
