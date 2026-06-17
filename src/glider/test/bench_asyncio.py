# Comprehensive micro-benchmark of MicroPython asyncio / GC / PSRAM / timing on the concrete
# Coludo glider board (FireBeetle 2 ESP32-P4, 32 MB PSRAM, MicroPython v1.28.0). The numbers
# feed the latency and GC assumptions in specs/coludo.md, and characterise behaviour worth
# reporting upstream (see doc/benches/esp32p4-micropython-findings.md).
#
# Headline finding reproduced by bench_buffer(): bytearray / memoryview slice-assignment
# `buf[a:b] = src` costs O(len(buf)) — it memmoves the tail past the slice even when the
# source and destination lengths are equal (no resize). The same 32-byte write is ~6 us into
# a 64-byte buffer but ~20 ms into a 256 KB buffer. struct.pack_into() and indexed writes are
# O(record) and must be used instead for any large preallocated (PSRAM) buffer.
#
# Run transiently (does not persist to the board's filesystem):
#     mpremote connect /dev/ttyACM0 run src/glider/test/bench_asyncio.py
#
# Emits a human-readable log to stdout and a final compact JSON line prefixed "BENCH_JSON ".

import asyncio
import gc
import struct
import sys
import time

us = time.ticks_us
diff = time.ticks_diff
add = time.ticks_add

R = {}  # collected results


def stats(samples):
    s = sorted(samples)
    n = len(s)
    if not n:
        return {'n': 0}
    total = 0
    for v in s:
        total += v
    p95 = s[min(int(n * 0.95), n - 1)]
    return {'n': n, 'min': s[0], 'avg': round(total / n, 1), 'p95': p95, 'max': s[-1]}


def log(*a):
    print(*a)


# ---------------------------------------------------------------- synchronous


def bench_ticks():
    N = 10000
    t0 = us()
    for _ in range(N):
        x = us()
    R['ticks_us_call_us'] = round(diff(us(), t0) / N, 4)
    log('ticks_us() overhead        : %.4f us/call' % R['ticks_us_call_us'])


def bench_alloc():
    N = 2000
    gc.collect()
    t0 = us()
    for _ in range(N):
        d = {}
    R['alloc_dict_us'] = round(diff(us(), t0) / N, 4)
    t0 = us()
    for _ in range(N):
        d = [0] * 8
    R['alloc_list8_us'] = round(diff(us(), t0) / N, 4)
    t0 = us()
    for _ in range(N):
        d = bytes(256)
    R['alloc_bytes256_us'] = round(diff(us(), t0) / N, 4)
    t0 = us()
    for _ in range(N):
        d = bytearray(256)
    R['alloc_bytearray256_us'] = round(diff(us(), t0) / N, 4)
    log(
        'alloc dict/list8/b256/ba256: %.3f / %.3f / %.3f / %.3f us'
        % (R['alloc_dict_us'], R['alloc_list8_us'], R['alloc_bytes256_us'], R['alloc_bytearray256_us'])
    )


def bench_buffer():
    # FINDING: bytearray/memoryview slice-assignment cost scales with the TOTAL buffer
    # length, not the slice length — assigning the same 32-byte slice into ever-larger
    # buffers gets dramatically slower (the implementation memmoves the tail past the
    # slice even when source and destination lengths are equal and no resize occurs).
    # This makes the obvious ring-buffer write `buf[off:off+REC] = rec` O(buffer_size),
    # i.e. unusable for a large preallocated PSRAM queue. See doc/benches/.
    rec = bytearray(32)
    by_len = {}
    for size in (64, 4096, 65536, 262144):
        b = bytearray(size)
        reps = 2000 if size <= 4096 else (200 if size <= 65536 else 50)
        t0 = us()
        for _ in range(reps):
            b[0:32] = rec
        by_len[size] = round(diff(us(), t0) / reps, 3)
    R['slice_assign_us_by_buflen'] = by_len
    log('slice-assign 32B vs buflen: ' + ' '.join('%d=%.1fus' % (k, by_len[k]) for k in sorted(by_len)))

    # Correct in-place primitives whose cost is independent of buffer size — what the
    # logger's PSRAM ring buffer must use instead of slice-assignment.
    SIZE = 262144
    b = bytearray(SIZE)
    N = 20000
    sz = 16
    fmt = '<IIII'
    cap = SIZE // sz
    t0 = us()
    for i in range(N):
        struct.pack_into(fmt, b, (i % cap) * sz, i, i, i, i)
    R['pack_into_16B_us'] = round(diff(us(), t0) / N, 3)
    t0 = us()
    for i in range(N):
        b[i % SIZE] = i & 0xFF
    R['indexed_write_us'] = round(diff(us(), t0) / N, 3)
    log('pack_into 16B / indexed b : %.3f / %.3f us/op' % (R['pack_into_16B_us'], R['indexed_write_us']))


def bench_psram():
    # Raw memcpy bandwidth of the (PSRAM-backed) heap: a full-length slice copy
    # `dst[:] = src` is a legitimate equal-size memcpy with no tail move.
    gc.collect()
    SIZE = 1 << 20  # 1 MB
    src = bytearray(SIZE)
    dst = bytearray(SIZE)
    REPS = 8
    t0 = us()
    for _ in range(REPS):
        dst[:] = src
    dt = diff(us(), t0) / REPS
    R['psram_memcpy_us_1MB'] = round(dt, 1)
    R['psram_memcpy_MBps'] = round(SIZE / (dt / 1e6) / 1e6, 1)
    log('psram 1MB memcpy          : %.0f us  (%.1f MB/s)' % (dt, R['psram_memcpy_MBps']))


def bench_gc():
    gc.collect()
    t0 = us()
    gc.collect()
    R['gc_collect_clean_us'] = diff(us(), t0)
    # Fragment the heap with many small live objects to approximate a busy runtime.
    # (Kept modest: collecting a very large object graph on PSRAM is extremely slow.)
    junk = []
    try:
        for i in range(10000):
            junk.append(bytearray(32))
    except MemoryError:
        pass
    R['gc_frag_objects'] = len(junk)
    t0 = us()
    gc.collect()
    R['gc_collect_fragmented_us'] = diff(us(), t0)
    del junk
    gc.collect()
    log(
        'gc collect clean/frag     : %d / %d us  (%d live objs)'
        % (R['gc_collect_clean_us'], R['gc_collect_fragmented_us'], R['gc_frag_objects'])
    )


# ---------------------------------------------------------------------- async


async def bench_yield():
    N = 5000
    t0 = us()
    for _ in range(N):
        await asyncio.sleep_ms(0)
    R['await_sleep0_us'] = round(diff(us(), t0) / N, 3)
    log('await sleep_ms(0) yield   : %.3f us/iter' % R['await_sleep0_us'])


async def bench_sleep_jitter():
    for req in (1, 5, 10):
        samples = []
        for _ in range(200):
            t0 = us()
            await asyncio.sleep_ms(req)
            samples.append(diff(us(), t0))
        st = stats(samples)
        R['sleep_%dms_us' % req] = st
        log(
            'sleep_ms(%-2d) actual us    : min/avg/p95/max = %d/%.1f/%d/%d'
            % (req, st['min'], st['avg'], st['p95'], st['max'])
        )


async def bench_notify():
    a = asyncio.Event()
    b = asyncio.Event()
    N = 500
    samples = []

    async def ponger():
        for _ in range(N):
            await a.wait()
            a.clear()
            b.set()

    t = asyncio.create_task(ponger())
    for _ in range(N):
        t0 = us()
        a.set()
        await b.wait()
        b.clear()
        samples.append(diff(us(), t0))
    await t
    st = stats(samples)
    R['event_pingpong_us'] = st
    log('event ping-pong rt us     : min/avg/p95/max = %d/%.1f/%d/%d' % (st['min'], st['avg'], st['p95'], st['max']))


async def _bg_worker(stop):
    # Background load: sleeps and allocates a little, creating realistic scheduler + GC pressure.
    while not stop[0]:
        x = [0] * 16
        d = {'a': 1, 'b': 2}
        await asyncio.sleep_ms(2)


async def bench_control_jitter(period_ms, ntasks, cycles, label):
    stop = [False]
    tasks = [asyncio.create_task(_bg_worker(stop)) for _ in range(ntasks)]
    samples = []
    ideal = us()
    for _ in range(cycles):
        ideal = add(ideal, period_ms * 1000)
        await asyncio.sleep_ms(period_ms)
        samples.append(diff(us(), ideal))  # >0 means the wake was late
        s = 0
        for i in range(50):  # token control-loop compute
            s += i * i
    stop[0] = True
    for t in tasks:
        try:
            await t
        except Exception:
            pass
    st = stats(samples)
    R[label] = st
    log(
        'ctl loop %-7s lateness us: min/avg/p95/max = %d/%.1f/%d/%d'
        % (label.split('_', 2)[-1], st['min'], st['avg'], st['p95'], st['max'])
    )
    return st


async def amain():
    await bench_yield()
    await bench_sleep_jitter()
    await bench_notify()
    await bench_control_jitter(5, 6, 300, 'ctl_200hz_6tasks_gc_on')
    gc.collect()
    gc.disable()
    await bench_control_jitter(5, 6, 300, 'ctl_200hz_6tasks_gc_off')
    gc.enable()


def main():
    R['impl'] = sys.version
    R['platform'] = sys.platform
    try:
        import machine

        R['freq_hz'] = machine.freq()
    except Exception:
        R['freq_hz'] = None
    gc.collect()
    R['mem_free_start'] = gc.mem_free()

    log('== coludo glider board benchmark ==')
    log(
        'impl: %s  platform: %s  freq: %s  mem_free: %d' % (R['impl'], R['platform'], R['freq_hz'], R['mem_free_start'])
    )
    log('-- synchronous --')
    log('> ticks')
    bench_ticks()
    log('> alloc')
    bench_alloc()
    log('> buffer')
    bench_buffer()
    log('> psram')
    bench_psram()
    log('> gc')
    bench_gc()
    log('-- asyncio --')
    asyncio.run(amain())

    gc.collect()
    R['mem_free_end'] = gc.mem_free()
    import json

    print('BENCH_JSON ' + json.dumps(R))


main()
