"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

findings 31.6 follow-up: is the offset() cos(lat) hoist WORTH doing? The proposal removes a
radians()+cos() (and their boxed floats) from the one geographic primitive everything derives from.
Its value is bytes-per-second of heap saved under the GC-off-in-flight policy -- so measure the
allocation per call, then weigh it against the nav rate and the measured real-flight leak.

    mpremote run test/diag_nav_alloc.py
"""

import gc
import math

import navigation

_CALLS = 200
_LAT, _LON = 60.17, 24.94
_TARGET = (60.1745, 24.9455)  # ~500 m away


def _per_call(label, function) -> float:
    """Bytes allocated per call, averaged -- GC disabled so nothing is reclaimed mid-measurement."""
    function()  # warm: first call binds names / builds constants
    gc.collect()
    gc.disable()
    before = gc.mem_alloc()
    for _ in range(_CALLS):
        function()
    used = gc.mem_alloc() - before
    gc.enable()
    print('    %-34s %8.1f B/call' % (label, used / _CALLS))
    return used / _CALLS


print('=== allocation on the geographic path (GC off, as in flight) ===')
offset_b = _per_call('navigation.offset()', lambda: navigation.offset(_LAT, _LON, _TARGET[0], _TARGET[1]))
_per_call('navigation.range_bearing()', lambda: navigation.range_bearing(_LAT, _LON, _TARGET[0], _TARGET[1]))
_per_call('navigation.distance()', lambda: navigation.distance(_LAT, _LON, _TARGET[0], _TARGET[1]))
trig_b = _per_call('bare math.cos(math.radians(x))', lambda: math.cos(math.radians(_LAT)))

print()
print('=== what the cos(lat) hoist would save ===')
print('    the hoist removes the radians()+cos() pair from offset(): %.1f B/call' % trig_b)
for calls_per_update in (2, 4):
    for hz in (10,):  # nav_period_ms default 100 -> the float trig is cached at 10 Hz, not 100
        rate = calls_per_update * hz
        print('    at %d offset() per nav update, %d Hz -> %4d calls/s -> %6.2f KB/s saved'
              % (calls_per_update, hz, rate, trig_b * rate / 1024.0))
print('    measured real-flight leak for scale: ~130 KB/s (OOM ~250 s)')
