# exploration: push past viper -- measure @micropython.native on the FLOAT hot path and a
# hand-written @micropython.asm_rv32 integer clamp, both vs bytecode/viper, with correctness asserts.
# Run on-board: mpremote connect PORT run test/bench_emitters.py
# ruff: noqa: F821 -- the asm_rv32 body uses bare RV32 mnemonics/registers the inline assembler provides.
import time

import micropython

_N = 30000


def _bench3(function, a, b, c):
    start = time.ticks_us()
    for _ in range(_N):
        function(a, b, c)
    return time.ticks_diff(time.ticks_us(), start) * 1000 // _N


# ---------- integer clamp: bytecode vs viper vs inline RV32 assembly ----------
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


# NOTE: @micropython.asm_rv32 is NOT available on this firmware build ("invalid micropython decorator")
# -- inline RV32 assembly isn't compiled into the board's MicroPython, so asm is off the table without a
# firmware rebuild. Only @native + @viper are usable (both measured here).


# ---------- float multiply-chain (a pid.step-shaped body): bytecode vs native ----------
def _axis_upy(error, integral, derivative):
    return 0.8 * error + 0.05 * integral + 0.1 * derivative


@micropython.native
def _axis_native(error, integral, derivative):
    return 0.8 * error + 0.05 * integral + 0.1 * derivative


# correctness before timing
for low, value, high in ((0, 5, 10), (0, -3, 10), (0, 99, 10), (-45, 60, 45), (-45, -60, 45)):
    assert _clamp_viper(low, value, high) == _clamp_upy(low, value, high), (low, value, high)
assert abs(_axis_native(10.0, 2.0, -1.0) - _axis_upy(10.0, 2.0, -1.0)) < 1e-9

print(' emitter bench -- %d calls each, ns/call:' % _N)
print('  clamp_int  bytecode=%4d  viper=%4d ns (int)'
      % (_bench3(_clamp_upy, 0, 5, 10), _bench3(_clamp_viper, 0, 5, 10)))
print('  pid-axis   bytecode=%4d  native=%4d ns (float mul-chain)'
      % (_bench3(_axis_upy, 10.0, 2.0, -1.0), _bench3(_axis_native, 10.0, 2.0, -1.0)))
print('ok: bench_emitters -- native + viper measured against bytecode (asm_rv32 unavailable on firmware)')
