# Avionics bench 2026-10-05: the Rust firmware on the ESP32-P4

The first numbers for Phase 6 (`src/avionics`, Rust std on ESP-IDF v5.5.5). Everything ran on the **taster**
(WaveShare ESP32-P4-WIFI6: P4 **rev v1.3**, 360 MHz, 32 MB hex PSRAM at 200 MHz, 32 MB GigaDevice flash
`0xC84019`, ESP32-C6 for Wi-Fi). Raw results: [`results.csv`](results.csv) (`;`-separated; `variant` is where
the code executed from). The tables quote the XIP-from-PSRAM build, now the default, unless a column says
otherwise. The code-in-flash runs are in the CSV.

## Reproduce

```sh
cd src/avionics/firmware
cargo build --release                                   # all binaries; nightly + build-std from rust-toolchain.toml
./flash.sh ../target/riscv32imafc-esp-espidf/release/bench   # or isa, flash, avionics
../../../tools/avionics_console.py --reset --wait 90 | grep '^bench'
```

| program | what it measures |
|---|---|
| `src/bin/bench.rs` | B1 CPU, B2 FPU, B3 memory, B5 threads and loop pacing |
| `src/bin/isa.rs` | which ISA extensions the core implements; atomics and locks across cores; esp-dsp SIMD kernels |
| `src/bin/flash.rs` | B4: flash erase/program latency, and the **stall test** (a 1 kHz loop while the other core writes flash) |
| `src/main.rs` | X1: console, panic → reboot, Wi-Fi through the C6, `whoami`/`health` to the CC hub |

## Results

### X1: the stack works

Boot to `main` 0.76 s (the PSRAM self-test, 0.9 s, is off). Wi-Fi joined through the C6 at ~9 s, and the hub
shows `taster online`. Hub round trip: **p50 3.2 ms, max 5.6 ms** (50 pings via the operator port). Before two
fixes it was 154 ms: one write per reply line (two writes waited out the hub's delayed ACK), and Wi-Fi power
save off (modem sleep held each packet until the next DTIM beacon). A deliberate panic prints its backtrace and
the board is back on the hub ~12 s later.

### CPU and FPU (B1, B2)

| measure | result | MicroPython 1.29 |
|---|---|---|
| f32 add / mul / div / sqrt | 22 / 22 / 56 / 56 ns (loop overhead included) | boxed floats → fixed-point |
| f32 sin / atan2 | 210 / 498 ns | |
| f64 add / sin / atan2 | 230 ns / 3.8 µs / 7.6 µs — software, **10–15× f32** | |
| a whole f32 control step (attitude + 2 PIDs + mixer) | **1.5 µs** (0.06% of a 400 Hz period) | |
| CRC32 of 64 KB, table / ROM | 2.0 / 1.7 ms (the ROM one matches zlib) | |
| a 20-field recorder row, formatted + CRC'd | 53 µs (float formatting dominates) | |

### Memory (B3)

| measure | result | MicroPython 1.29 |
|---|---|---|
| PSRAM cache miss (random 64 B lines) | **460 ns**; internal RAM 35 ns | |
| copy 1 MB PSRAM → PSRAM | **57 MB/s** — latency-bound; bulk moves belong to DMA | memcpy 33.5 MB/s |
| copy 4 KB / 64 KB (L2-cache resident) | 575 / 370 MB/s | |
| internal RAM free at main / largest block | 513 KB / 344 KB | |

### Threads and pacing (B5)

| measure | p50 | max | MicroPython 1.29 |
|---|---|---|---|
| raw FreeRTOS task-notify round trip, cross-core | **15 µs** | 29 µs | `Event` round trip 301 µs |
| std `mpsc` round trip, cross-core | 75 µs | 103 µs | |
| 400 Hz, `thread::sleep` (tick-quantised) | 172 µs | **672 µs** | `sleep_ms` floor ~10 ms |
| 400 Hz, **esp_timer → task notify** | **15 µs** | **19 µs** | |

Flight pacing is therefore an esp_timer (or IMU interrupt) → task notification, never `sleep`.

### What the core implements, atomics, locks

* `misa` = rv32 **acfimux**. Executed by raw encoding: **Zcb and Zcmp work, Zba/Zbb/Zbs/Zbc/Zicond trap**. This
  matches Espressif clang's `-mcpu=esp32p4`.
* Rust builds with **`+zcb`**. Zcmp was tried and dropped. The bench crash-looped once threads context-switched,
  and call-heavy code got 9% slower. The git history keeps both commits.
* **Atomics** (`amoadd`, LR/SC) are exact from both cores at once, on internal RAM **and PSRAM**. Cost: 59 ns per
  `fetch_add`, 92 ns per spin-lock round.
* **Locks**: std `Mutex` costs 4.25 µs per lock/unlock even uncontended. The FreeRTOS mutex under it costs 3.98 µs,
  and a port critical section 0.81 µs. The cost is in IDF's port layer, not the silicon. Rule: one owner per
  resource, with atomics and task notify between owners, and no mutex in the hot path.
* **FPU** (from IDF's port):
  * it is enabled lazily per task: the first float traps once and pins the task to its core;
  * **a float in an ISR aborts**.
* **PIE SIMD** is reachable only through esp-dsp (upstream LLVM has no `xesppie`):
  * int16 dot product of 1024: **2.6 µs vs 34 µs** in Rust (13×);
  * f32 kernels: only 1.2–1.3× (hardware loops; there is no float SIMD);
  * a 4×4 matrix is faster in plain Rust.

### Flash: can telemetry go to flash in flight? (B4)

| measure | code in flash | **XIP from PSRAM** (now the default) |
|---|---|---|
| 1 kHz loop while the other core erases + programs 64 KB blocks | p50 7.3 ms, **max 125 ms, 9249/10000 ticks missed** | p50 14 µs, **max 19 µs, 0 missed** (4 MB written in 10 s) |
| same loop, flash idle | 18 µs, max 41 µs | 14 µs, max 18 µs |
| erase 4 KB / 64 KB | 30 / 111 ms (previously written; blank: 13 / 24 ms) | 16 / 95 ms |
| program 256 B; sustained | 550 µs; 0.49 MB/s | 518 µs; 0.50 MB/s |
| pre-erase 8 MB | 13.5 s | 8.6 s |

**XIP from PSRAM is what makes the flash tier possible.** The P4's flash and PSRAM sit on separate buses. With the
code copied to PSRAM at boot, erasing and programming the flash no longer freezes the caches. Flash auto-suspend
is not an alternative here: IDF enables it only for GD `C84016/17/18` and `C84319`, and ours is `C84019`.

The same bench on the XIP build costs nothing that matters, and is slightly faster:
* control step −5%;
* pacing max 41 → 19 µs;
* worst same-core wake-up 190 → 122 µs;
* 64 KB PSRAM copies −2%;
* about 0.75 MB of PSRAM for the copied image.

## Decided by these numbers

* **D5 pacing:** esp_timer / IMU interrupt → task notify (not `sleep`). 400 Hz has headroom.
* **D6 storage tier:** keep the flash ring. Build with `CONFIG_SPIRAM_XIP_FROM_PSRAM=y`.
* **D18 numerics:** f32 in the hot path, no f64.
* **Codegen:** `+zcb`, nothing else. PIE SIMD only via esp-dsp, and only for int16/int8 vector work.
* **Concurrency:** no mutex in the hot path (4 µs each). Atomics and notify between single owners.

## Open

* **B4 soak:** 10 minutes rather than 10 s. Then a **10 min stall test with video DMA on PSRAM**: code fetch now
  shares that bus.
* **B3b:** L2 cache 256/512 KB and 128 B lines, plus GDMA `async_memcpy` for bulk PSRAM moves.
* **The recorder row (53 µs):** format integers (the wire's `_cd`/`_cms` units) instead of floats.
* **The 32 MB part's upper 16 MB:** IDF marks `0x19`-class GD chips 32 MB-capable (4-byte addressing). The ring
  could live there.
* **S1/S2:** SD on SDMMC slot 0 next to the C6 on slot 1. V1: the camera.
* **Pin the nightly** (`rust-toolchain.toml` says `nightly`; it was 1.101.0-nightly 2026-10-04 here).
