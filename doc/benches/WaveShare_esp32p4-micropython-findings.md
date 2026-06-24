# MicroPython on ESP32-P4 — benchmark findings

Measurements taken for the Coludo flight-controller project to validate latency and memory
assumptions, with several results that look like general MicroPython behaviour worth sharing
upstream. Re-run 2026-06-24 (same board + firmware) — every number reproduced within measurement
noise, so the findings below are stable. Raw data:
[`WaveShare_esp32p4_micropython-1.28.0.log`](WaveShare_esp32p4_micropython-1.28.0.log) and
[`WaveShare_esp32p4_micropython-1.28.0.json`](WaveShare_esp32p4_micropython-1.28.0.json). Benchmark source:
[`../../src/glider/test/bench_asyncio.py`](../../src/glider/test/bench_asyncio.py).

## Environment

| | |
|---|---|
| Board | [WaveShare ESP32-P4-WIFI6, 32 MB PSRAM](https://www.waveshare.com/esp32-p4-wifi6.htm) |
| Firmware | `MicroPython v1.28.0 on 2026-04-06` (impl 3.4.0), `sys.platform == "esp32"` |
| CPU | `machine.freq()` = 360 MHz |
| Heap | `gc.mem_free()` ≈ 33.08 MB at start → the GC heap is backed by (slow) PSRAM |
| Driver | run via `mpremote run`, output streamed over USB-CDC |

Reproduce all of it:

```
mpremote connect /dev/ttyACM0 run src/glider/test/bench_asyncio.py
```

---

## Finding 1 — `bytearray`/`memoryview` slice-assignment is O(len(buffer)), not O(len(slice))

Assigning an **equal-length** slice (no resize) gets dramatically slower as the *destination
buffer* grows, even though the number of bytes written is constant. The same 32-byte store:

| destination buffer | `buf[0:32] = rec` |
|---|---|
| 64 B | **4.1 µs** |
| 4 KiB | 71.3 µs |
| 64 KiB | 1098 µs |
| 256 KiB | **19 664 µs** |

The cost tracks the buffer length (~12 MB/s, i.e. the PSRAM memcpy rate from Finding 4), which
strongly suggests the slice-store path **memmoves the tail of the buffer past the assigned
region even when `new_len == old_len`** and no resize is required. A same-length slice store
should be a `memcpy` of the slice only — O(len(slice)).

Minimal reproducer:

```python
import time
us = time.ticks_us; d = time.ticks_diff
rec = bytearray(32)
for size in (64, 4096, 65536, 262144):
    b = bytearray(size)
    n = 2000 if size <= 4096 else 50
    t = us()
    for _ in range(n):
        b[0:32] = rec          # equal-length store, no resize
    print(size, round(d(us(), t) / n, 1), "us/op")
```

Confirmed identical for `memoryview(buf)[0:32] = rec`.

**Workaround** (cost independent of buffer size): use `struct.pack_into()` or indexed stores.

| in-place primitive into 256 KiB buffer | per op |
|---|---|
| `struct.pack_into("<IIII", buf, off, …)` (16 B) | 8.5 µs |
| `buf[i] = v` (1 B indexed) | 2.7 µs |

---

## Finding 2 — `asyncio.sleep_ms()` floors at ~10 ms (FreeRTOS 100 Hz tick)

Requested vs. actual sleep (200 samples each, no other tasks running):

| requested | min | avg | p95 | max |
|---|---|---|---|---|
| `sleep_ms(1)` | 8279 µs | **9986 µs** | 10004 µs | 10005 µs |
| `sleep_ms(5)` | 8547 µs | **9988 µs** | 10004 µs | 10004 µs |
| `sleep_ms(10)` | 9932 µs | 10039 µs | 10004 µs | 18800 µs |

`sleep_ms(1)` and `sleep_ms(5)` both actually sleep ~10 ms — the delay quantises to the
FreeRTOS tick (`configTICK_RATE_HZ` = 100 → 10 ms). **Cooperative scheduling via
`asyncio.sleep_ms` therefore cannot exceed ~100 Hz**, and any requested period below 10 ms
silently becomes 10 ms. A periodic loop that advances an ideal schedule by 5 ms per cycle
accumulates unbounded drift (we measured cumulative lateness growing into the hundreds of ms
to >1 s over 300 cycles).

Implication for sub-10 ms control loops: don't rely on `asyncio.sleep`; use a hardware timer
+ `ThreadSafeFlag`, or busy-wait on `time.ticks_us()`.

For reference, the bare scheduler round-trip `await asyncio.sleep_ms(0)` is **83.4 µs/iter**,
and an `Event` set→wake ping-pong round-trip is **306 µs avg** (172 min / 487 max).

---

## Finding 3 — GC pause scales with live-object count on the PSRAM heap

| `gc.collect()` | pause |
|---|---|
| clean heap | **318 µs** |
| 10 000 small live objects (`bytearray(32)`) | **67 232 µs (67 ms)** |

A modestly fragmented heap (10k objects, ~0.5 MB of a 33 MB heap) pushes a single collection
to 67 ms — far beyond a 10 ms real-time loop. The collector cost is dominated by walking the
object graph in slow PSRAM. This argues for minimising allocations on hot paths and scheduling
collections explicitly at safe points (or disabling GC during time-critical phases).

---

## Finding 4 — PSRAM heap memcpy bandwidth ≈ 12 MB/s

`dst[:] = src` for two 1 MiB buffers (a legitimate equal-size copy): **88 996 µs/MiB → 11.8
MB/s**. Since the whole GC heap lives in PSRAM on this board, *every* allocation and bulk copy
pays this rate, which is what makes Finding 1's tail-memmove so expensive.

---

## Finding 5 — `machine.I2C(2)` hard-crashes instead of raising `ValueError`

On this build there are two hardware I²C controllers. Requesting a non-existent peripheral id
is handled inconsistently: `SPI(0)` and `SPI(3)` cleanly raise `ValueError: SPI(n) doesn't
exist`, but **`I2C(2)` triggers a fatal error and crash dump** (the board resets) instead of a
`ValueError`. Reproduce from the REPL:

```python
from machine import I2C, SPI
SPI(3)    # -> ValueError: SPI(3) doesn't exist   (correct)
I2C(2)    # -> "A fatal error occurred. The crash dump printed below ..."  (board resets)
```

An out-of-range bus id should raise, not crash.

## Other primitives (for budgeting)

| op | cost |
|---|---|
| `time.ticks_us()` call | 3.1 µs |
| `{}` alloc | 2.6 µs |
| `[0]*8` alloc | 7.0 µs |
| `bytes(256)` / `bytearray(256)` alloc | 44 / 40 µs |

---

## Net effect on Coludo

- The flight-control loop cannot be driven by `asyncio.sleep` at 200 Hz/5 ms (Finding 2); the
  hot path uses a hardware-timer-paced loop reading a preallocated, no-allocation "blackboard"
  (Finding 3) and never slice-assigns into large buffers (Finding 1).
- The logger's PSRAM ring buffer writes records with `struct.pack_into`, never slice-assignment.
- GC is collected at safe points and avoided during time-critical phases.

These feed the latency budget and data-flow model in
[`../../specs/coludo.md`](../../specs/coludo.md).
