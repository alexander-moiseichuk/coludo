"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

WHERE the sensor hot loop allocates -- per call, in isolation.

The per-component profile (test/diag_real_leak.py) put accel_adxl375 at 111 KB/s, which is ~1.1 KB per
sample at its 100 Hz ODR -- far more than its read path explains. This measures each piece of that
loop separately, so the fix targets the actual cost instead of the suspected one.

    mpremote run test/diag_alloc_hotloop.py
"""

import asyncio
import gc
import struct

import databoard
import fixed
import recorder


class _FakeWriter:
    """Swallow the recorder wire so Telemetry.push is measured, not the UART behind it."""

    def write(self, _data) -> None:
        pass

    async def drain(self) -> None:
        pass


def _alloc(label: str, call, reps: int = 200) -> float:
    """Bytes allocated per call, GC off (the airborne policy) so nothing is reclaimed mid-count."""
    call()  # warm: first call binds names and builds any constants
    gc.collect()
    gc.disable()
    before = gc.mem_alloc()
    for _ in range(reps):
        call()
    used = gc.mem_alloc() - before
    gc.enable()
    print('   %-44s %7.1f B/call' % (label, used / reps))
    return used / reps


async def _alloc_async(label: str, call, reps: int = 200) -> float:
    """Same, for a coroutine -- the awaited machinery is what is being measured."""
    await call()
    gc.collect()
    gc.disable()
    before = gc.mem_alloc()
    for _ in range(reps):
        await call()
    used = gc.mem_alloc() - before
    gc.enable()
    print('   %-44s %7.1f B/call' % (label, used / reps))
    return used / reps


async def main() -> None:
    import config_default
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    buf = bytearray(6)
    scale = 0.049
    flag = asyncio.ThreadSafeFlag()

    print('=== the ADXL375 sample path, piece by piece ===')
    _alloc('struct.unpack(\'<hhh\', buf)', lambda: struct.unpack('<hhh', buf))
    unpacked = struct.unpack('<hhh', buf)
    _alloc('the (x*s, y*s, z*s) float tuple', lambda: (unpacked[0] * scale, unpacked[1] * scale,
                                                       unpacked[2] * scale))
    channel = databoard.Databoard.provide('bench_probe', {'accel': {'priority': 9}}, 'accel')
    sample = (0.1, 0.2, 0.98)
    _alloc('channel.push((x, y, z))', lambda: channel.push(sample))
    stream = recorder.Telemetry('bench_probe.csv', ('ax', 'ay', 'az'), decimate_us=-1)
    _alloc('Telemetry.push((x, y, z))', lambda: stream.push(sample))
    _alloc('the rounded telemetry tuple', lambda: (round(sample[0], 3), round(sample[1], 3),
                                                   round(sample[2], 3)))

    print()
    print('=== the LSM6DSO32 sample path (12-byte block, 3 accel floats + 3 gyro fixnums) ===')
    buf12 = bytearray(12)
    _alloc("struct.unpack('<hhhhhh', buf12)", lambda: struct.unpack('<hhhhhh', buf12))
    six = struct.unpack('<hhhhhh', buf12)
    _alloc('the 6-value sample tuple', lambda: (six[3] * 0.061, six[4] * 0.061, six[5] * 0.061,
                                                six[0] * 70 // 10, six[1] * 70 // 10, six[2] * 70 // 10))
    _alloc('to_str(fixnum) x3 -- REMOVED from the driver', lambda: (fixed.to_str(1234),
                                                                    fixed.to_str(5678),
                                                                    fixed.to_str(9012)))
    gyro = recorder.Telemetry('bench_gyro.csv', ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'irq_runs'),
                              decimate_us=-1)
    row = (0.1, 0.2, 0.98, '1.23', '4.56', '7.89', 1)
    _alloc('Telemetry.push(7 fields)', lambda: gyro.push(row))

    print()
    print('=== the WAIT, which is the suspect ===')

    async def _sleep():
        await asyncio.sleep_ms(0)

    async def _waitfor():
        flag.set()
        try:
            await asyncio.wait_for_ms(flag.wait(), 50)
        except asyncio.TimeoutError:
            pass

    async def _flag_only():
        flag.set()
        await flag.wait()

    await _alloc_async('asyncio.sleep_ms(0)                  ', _sleep)
    await _alloc_async('ThreadSafeFlag.wait() alone          ', _flag_only)
    await _alloc_async('asyncio.wait_for_ms(flag.wait(), 50) ', _waitfor)


asyncio.run(main())
