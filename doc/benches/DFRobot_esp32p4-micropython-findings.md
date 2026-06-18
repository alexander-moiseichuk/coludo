# MicroPython on DFRobot FireBeetle 2 ESP32-P4 — benchmark findings

Same benchmark as the [WaveShare report](WaveShare_esp32p4-micropython-findings.md), run on the
[DFRobot FireBeetle 2 ESP32-P4](https://www.dfrobot.com/product-2915.html). Raw data:
[`DFRobot_esp32p4_micropython-1.28.0.log`](DFRobot_esp32p4_micropython-1.28.0.log) /
[`.json`](DFRobot_esp32p4_micropython-1.28.0.json). Source:
[`../../src/glider/test/bench_asyncio.py`](../../src/glider/test/bench_asyncio.py).

## Environment

| | |
|---|---|
| Board | [DFRobot FireBeetle 2 ESP32-P4](https://www.dfrobot.com/product-2915.html), 32 MB PSRAM |
| Firmware | `MicroPython v1.28.0 on 2026-04-06`, `sys.platform == "esp32"`, 360 MHz |
| Heap | `gc.mem_free()` ≈ 33.08 MB → GC heap backed by PSRAM |
| USB | native Espressif USB-JTAG/serial (`303a:1001`) — *not* a CH343 like the WaveShare |

## Result: performance is identical to the WaveShare board

Both boards use the same in-package **ESP32-P4NRW32** (32 MB PSRAM) and the same MicroPython
build, so every number matches within run-to-run noise:

| metric | WaveShare | DFRobot FireBeetle 2 |
|---|---|---|
| `ticks_us()` call | 3.12 µs | 3.11 µs |
| `struct.pack_into` 16 B | 8.5 µs | 8.5 µs |
| `buf[i]=v` indexed | 2.7 µs | 2.7 µs |
| slice-assign 32 B into 256 KiB | 19.7 ms | 19.6 ms |
| PSRAM 1 MiB memcpy | 11.8 MB/s | 11.8 MB/s |
| `gc.collect()` clean / fragmented | 0.33 / 67 ms | 0.32 / 67 ms |
| `asyncio.sleep_ms(1)` actual | ~10 ms | ~10 ms |
| `Event` ping-pong round-trip | 305 µs | 307 µs |

So **all four upstream-worthy findings carry over unchanged** — O(len(buffer)) slice-assignment,
the ~10 ms `asyncio.sleep_ms` floor, ~67 ms fragmented-heap GC, and ~12 MB/s PSRAM bandwidth (see
the WaveShare report for the detail + reproducers). The firmware's design choices (pack_into ring,
event-driven drain, hardware-timer-or-not control loop) are driven by these and apply to both.

## What differs is the *board*, not the chip

The FireBeetle 2 and WaveShare ESP32-P4-WIFI6 differ in **USB interface** (native ESP-JTAG vs
CH343), **header pinout**, and onboard peripherals — so the pin map (`doc/waveshare_esp32p4_pins.md`)
is board-specific and a `board.json` per board is the right model. Performance assumptions are
shared; pin assignments are not.
