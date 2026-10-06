# Avionics bench 2026-10-05: the Rust firmware on the ESP32-P4

The first numbers for Phase 6 (`src/avionics`, Rust std on ESP-IDF v5.5.5). Everything ran on the **taster**
(WaveShare ESP32-P4-WIFI6: P4 **rev v1.3**, 360 MHz, 32 MB hex PSRAM at 200 MHz, 32 MB GigaDevice flash
`0xC84019`, ESP32-C6 for Wi-Fi). Raw results: [`results.csv`](results.csv) (`;`-separated; `variant` is where
the code executed from). The tables quote the XIP-from-PSRAM build, now the default, unless a column says
otherwise. The code-in-flash runs are in the CSV.

## Headline numbers

Per HP core at 360 MHz; "chip" is both cores running at once. Throughput, from `src/bin/peak.rs`:

| measure | per core | chip |
|---|---|---|
| **CoreMark** (EEMBC, Espressif's port, GCC 14.2 `-O3`, code in IRAM) | **1068 = 2.97 CoreMark/MHz** | (single-threaded run) |
| **f32 FMA**, 16 independent chains | **499 MFLOPS** | **996 MFLOPS ≈ 1.0 GFLOPS** |
| f32 FMA / add / mul, 8 chains (latency not hidden) | 382 / 191 / 191 MFLOPS | 764 / 382 / 382 |
| f64 (software emulation) | 4.3 MFLOPS | 8.6 |
| int32 add/shift/xor, 16 chains | 313 MOPS (IPC ~0.9, single-issue) | 625 MOPS |
| int32 multiply-add, 8 chains | 250 MOPS | 498 MOPS |
| **PIE SIMD int16 MAC** (esp-dsp Q15 dot product) | **546 MMAC/s = 1.09 GOPS** | **1.09 GMAC/s = 2.2 GOPS** |
| f32 dot product (esp-dsp, hardware loops) | 142 MFLOPS | 284 MFLOPS |
| stream read, internal SRAM | 436 MB/s | 872 MB/s |
| stream read, PSRAM (CPU, cache-miss bound) | 70 MB/s | 140 MB/s |

## Our flight code: Rust vs MicroPython, and which rustc flags

The cases are our own functions, not synthetic kernels: each is a faithful Rust port of the `src/glider`
function it is named after ([`src/bin/cases.rs`](../../../src/avionics/firmware/src/bin/cases.rs)). The MicroPython
column comes from the twin [`src/glider/test/bench_cases.py`](../../../src/glider/test/bench_cases.py), which calls
the REAL deployed modules with the same inputs: MicroPython 1.29.0 (`PRE_REV3_C6_WIFI`, viper/native where the
module binds them), on the same taster, the same day. Both sides print 12 `check;` lines (PID outputs, fin angles,
the CORDIC angle, the steer bearing, the CRC-framed recorder line byte for byte, ...): **all 12 are identical** on
every flag set, so the ports compute exactly what the MicroPython code does.

Nanoseconds per call, best of 5 runs (MicroPython: best of 3), loop and call included. **Bold** = fastest flag set.
`pid_step_f32` / `control_step_f32` are the PID written the Rust way (plain f32): `fixed.py`'s centidegree
integers exist only to dodge MicroPython's boxed floats with the GC off, and in Rust the f32 version costs the
same.

| case | MicroPython 1.29 | o3 | o3_lto | o2 | os | os_inline | os_lto | oz | best vs MicroPython |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `clamp_int` | 7034 | **36.3** | **36.3** | **36.3** | 55.9 | 55.8 | 55.8 | 55.8 | 194× |
| `wrap180` | 6810 | 64.3 | 64.2 | 64.3 | 61.4 | 64.4 | 61.4 | **42.1** | 162× |
| `between` | 8417 | **61.5** | **61.5** | **61.5** | 64.3 | 64.3 | 64.3 | 64.3 | 137× |
| `magnitude_sq` | 30754 | **55.9** | **55.9** | **55.9** | 64.3 | 64.3 | 64.3 | 64.3 | 550× |
| `bank_demand` | 21589 | 72.7 | 72.7 | **72.6** | 81.0 | 83.2 | 93.0 | 81.0 | 297× |
| `sensirion_crc8` | 8670 | **41.9** | **41.9** | **41.9** | 634 | 576 | 433 | 684 | 207× |
| `atan2_cd` | 16229 | 370 | **370** | 468 | 537 | 455 | 523 | 623 | 44× |
| `to_str` | 78583 | 2361 | 2332 | **2293** | 2563 | 2397 | 2634 | 3265 | 34× |
| `pid_step` | 37409 | **134** | 134 | 226 | 238 | 235 | 221 | 285 | 279× |
| `pid_step_x3` | 102114 | 357 | **357** | 693 | 722 | 716 | 674 | 836 | 286× |
| `mixer_actuate` | 65596 | **439** | 448 | 449 | 559 | 466 | 568 | 587 | 149× |
| `control_step` | 275158 | **1208** | 1218 | 1494 | 1636 | 1537 | 1588 | 1852 | 228× |
| `pid_step_f32` | — | **137** | **137** | 243 | 254 | 157 | 257 | 347 | — |
| `control_step_f32` | — | **1196** | 1207 | 1535 | 1631 | 1270 | 1638 | 1969 | — |
| `steer_f32` | 535903 | 3534 | **2135** | 3593 | 3554 | 3552 | 2399 | 4148 | 251× |
| `approach_f32` | 367852 | 2871 | **890** | 2905 | 2926 | 2916 | 3122 | 3189 | 413× |
| `telemetry_row_format` | 259439 | 23626 | **19797** | 23716 | 26097 | 24756 | 26941 | 34726 | 13× |
| `recorder_wrap` | 51651 | 5437 | **5406** | 5444 | 6324 | 6317 | 5820 | 6339 | 10× |
| `ticks_us` | 2490 | 773 | 770 | 792 | 780 | 798 | **762** | 769 | 3× |
| `alloc_bytes256` | 221790 | 5632 | **5553** | 5625 | 6073 | 6094 | 5832 | 6456 | 40× |
| `pack_into_16B` | 11524 | **115** | **115** | **115** | 498 | 498 | 126 | 780 | 101× |
| `yield` | — | 4766 | 4790 | **4728** | 4802 | 4791 | 4781 | 4742 | — |
| image text (bytes) | — | 783880 | 749366 | 765184 | 736234 | 754202 | 685046 | 729992 | |

Flag sets, each a full build of the same sources (`CARGO_PROFILE_RELEASE_*`; `os_inline` adds
`-C llvm-args=-inline-threshold=225`, the analogue of GCC's `-Os -finline-functions`):

| flags | opt-level | LTO | codegen-units | geomean (ns) | vs MicroPython (geomean) | text of the `cases` binary |
|---|---|---|---|---:|---:|---:|
| **`o3_lto`** (now the release profile) | 3 | fat | 1 | **415** | **98×** | 749 KB |
| `o3` (the previous default) | 3 | — | 16 | 458 | 88× | 784 KB |
| `o2` | 2 | — | 16 | 500 | 81× | 765 KB |
| `os_lto` | s | fat | 1 | 604 | 67× | **685 KB** |
| `os_inline` | s + inline 225 | — | 16 | 659 | 61× | 754 KB |
| `os` | s | — | 16 | 679 | 60× | 736 KB |
| `oz` | z | — | 16 | 739 | 55× | 730 KB |

* **Rust is 55–98× faster on our code** (geometric mean over the 19 shared cases), 10–550× per case: a whole glide
  control step (governor, bank demand, three PIDs, the fused mixer) 275 µs → **1.2 µs**, one PID step 37 µs →
  **134 ns**, `navigation.steer` 536 µs → **2.1 µs**.
* **`opt-level=3` + fat LTO + one codegen unit wins**: fastest overall, and smaller than plain O3. LTO matters most
  where code crosses functions: `approach` 2.9 → 0.9 µs, `steer` 3.5 → 2.1 µs. It is the release profile now.
* **The size levels cost 1.4–1.8×**, up to 15× on bit loops (`sensirion_crc8` 42 ns at O3, 434–684 ns at s/z: O3
  unrolls the 16-step loop). Raising the inline threshold under `s` buys little. The smallest image
  (`os_lto`, 685 KB) would be the pick only if flash space ever got tight -- it does not: the app partition is 4 MB.
* **Weak spot: text formatting.** A 20-integer telemetry row costs 20 µs (only 13×): `core::fmt` is heavy. At
  100 Hz that is 0.2% of a core; a digit-writing formatter would cut it ~10× if it ever matters.
* `ticks_us` (esp_timer) costs 770 ns -- a peripheral-bus read; hot-path timing reads the `cycle` CSR instead.

## Selections and tweaks

| area | chosen | why (measured here unless noted) |
|---|---|---|
| language / runtime | Rust std on ESP-IDF; `riscv32imafc-esp-espidf`, nightly + build-std | no_std esp-hal does not support P4 rev < 3, nor CSI / H.264 |
| ESP-IDF | **v5.5.5 tag**, pinned | the esp-idf crates' version; 6.0.0–6.0.2 break SD + esp_hosted; esp_ipa crashes on rev < 3 under 6.x |
| crates | esp-idf-sys 0.38.1 / hal 0.47.0 / svc 0.53.0 | latest; Wi-Fi bindings work through esp_wifi_remote (X1) |
| silicon | `ESP32P4_SELECTS_REV_LESS_V3` + `REV_MIN_100`; **360 MHz** | the fleet is rev v1.3; 400 MHz needs Espressif-qualified chips |
| C6 Wi-Fi link | esp_hosted 2.7.0 + esp_wifi_remote 0.15.2, plus `components/hosted_no_reboot` | MicroPython v1.29.0's pair; the C6 keeps its factory firmware (reflashing needs its UART pad). The guard parks esp_hosted's "reset host" paths: a C6 brown-out no longer reboots the P4 (tested) |
| code placement | **XIP from PSRAM** | flash writes stall nothing (B4: 0 missed 1 kHz ticks vs 125 ms stalls); control step −5% |
| PSRAM / flash | PSRAM 200 MHz hex, boot memtest off; flash QIO 80 MHz, declared 16 MB | memtest cost 0.9 s per boot; >16 MB addressing not yet tested |
| scheduler | `FREERTOS_HZ=1000`; pacing = esp_timer / IMU interrupt → task notify | `thread::sleep` is tick-quantised (918 µs p99) |
| codegen | **`+zcb`** only | Zcmp crash-looped and was 9% slower (reverted, in git history); Zb*/Zicond trap |
| release profile | **`opt-level=3`, `lto="fat"`, `codegen-units=1`** | fastest of seven flag sets on our own code, and smaller than plain O3 (see above) |
| SIMD | esp-dsp (PIE + hardware loops) via FFI, for int16/int8 vector work | upstream LLVM cannot emit PIE; 13× on int16, 1.2× on f32 |
| concurrency | one owner per resource; atomics + task notify; **no mutex in the hot path** | std Mutex 4.25 µs, FreeRTOS mutex 3.98 µs, atomics 59 ns |
| floats | **f32 by default**; f64 only with a stated reason in a genuinely critical spot (none so far: JSON via typed structs, lat/lon in a local metric frame); never a float in an ISR | no double-precision unit: f64 is 10–15× slower; a float in an ISR aborts; `json!` widens f32 to f64 |
| temperature | on-die sensor (20..100 °C band, ±2 °C) on health and every bench | CPU load alone: +4 °C (see the soak) |
| storage | flash declared 32 MB; **24 MB `log` partition** (0x800000 to the end) for the telemetry ring | 4-byte addressing verified across 16 MB; 0 missed ticks while writing above it; pre-erase 31 s |
| Wi-Fi | power save off; one write per reply line; `TCP_NODELAY` | hub round trip 154 ms → 3 ms |
| console | UART0 (CH343P) speaks the CC protocol; DTR+RTS held asserted on open | releasing them resets the board |
| flashing | esptool `elf2image` + IDF `gen_esp32part.py` (`firmware/flash.sh`) | espflash rejects custom subtypes and cannot package XIP images |

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
| `src/bin/peak.rs` | CoreMark and peak throughput (FLOPS, OPS, MAC/s, stream bandwidth) on one core and both |
| `src/bin/cases.rs` | our own flight functions, ported (twin of `src/glider/test/bench_cases.py`) |
| `src/bin/soak.rs` | thermal soak: 10 min full load on both cores, then 3 min idle |
| `src/bin/dma.rs` | PSRAM/internal bandwidth through AXI-GDMA, by burst size and number of engines |
| `src/bin/sd.rs` | microSD mount, write rate and per-write latency by chunk size and buffer, contiguous file |
| `src/main.rs` console `video start [w h fps kbit/s] [bars] [bayer=N]` / `stop` / `stats` | the camera recorder; `tools/video_sei.py` reads its frame stamps |
| `src/main.rs` console `load [s]`, `pace [s] [gptimer]` | per-core CPU load and top tasks; the 1 kHz loop's lateness (src/diagnostics.rs) |
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

### The whole flash for the ring: 32 MB declared, `log` = 24 MB (0x800000–0x2000000)

With the code in PSRAM the flash only holds the ~1 MB app, so the telemetry ring takes everything from 8 MB to
the end of the 32 MB part; a background GC erases blocks once the SD copy is confirmed. Above 16 MB the flash
needs 4-byte addressing, which `CONFIG_ESPTOOLPY_FLASHSIZE_32MB` turns on in the bootloader and `esp_flash`
(data access only -- the ring is never cache-mapped):

| measure | result |
|---|---|
| boundary check: 4 KB patterns at absolute 9 MB and 25 MB (16 MB apart) | **both intact** -- no wrap onto the low half (the app) |
| stall test with the writer entirely above 16 MB (16–23 MB) | 1 kHz loop: max **17 µs**, **0 missed** (47 blocks of 64 KB written in 10 s) |
| pre-erase all 24 MB | 31 s (a pad-time, one-off cost) |
| capacity | at 80 KB/s (400 Hz × 200 B rows) the ring holds 5 min before the GC must have freed blocks |

### The microSD (S1/S2, `src/bin/sd.rs`)

Taster's card: SDHC, 29.8 GB, 4-bit at 40 MHz (high speed; UHS is excluded -- it drops LDO VO4 to 1.8 V, and VO4
also feeds the IMU's SPI pins). Mounted on SDMMC slot 0 next to the C6 on slot 1 in **67 ms**, never formatted.
Power: Q1 (SI2301CDS, P-channel) switches ESP_3V3 to the card, gate on **GPIO45 with a 10 k pull-down -- low = on**
(the default), so GPIO45 high power-cycles a hung card. IDF 5.5.5's default host deinitialises per slot, so a
failed SD mount cannot take the C6 link down. FatFs needs long file names enabled (8.3 by default).

32 MB per row, sustained rate including the final fsync; every readback verified:

| write chunk | buffer | rate | write p50 | p99 | max |
|---|---|---:|---:|---:|---:|
| 4 KB | internal | 1.8 MB/s | 1.7 ms | 10 ms | 61 ms |
| 4 KB | PSRAM | 1.6 MB/s | 1.8 ms | 49 ms | 65 ms |
| 32 KB | internal | **8.0 MB/s** | 3.8 ms | 20 ms | 24 ms |
| 32 KB | PSRAM | 4.1 MB/s | 4.8 ms | 59 ms | 118 ms |
| 128 KB | internal | 4.7 MB/s | 15 ms | 121 ms | 194 ms |
| 128 KB | PSRAM | **6.5 MB/s** | 19 ms | 38 ms | 39 ms |
| 512 KB | PSRAM | 4.1 MB/s | 94 ms | 425 ms | 473 ms |
| 512 KB, preallocated contiguous file | PSRAM | 4.3 MB/s | 80 ms | 417 ms | 423 ms |

* **The card sets the pace** (4–8 MB/s, with write stalls up to ~0.5 s -- the SD busy window), not FatFs: the
  preallocated contiguous file (no FAT updates) is no faster. A faster card is the lever, if one is ever needed.
* **For video that is comfortable**: 8 Mbit/s is 1 MB/s (4–8× headroom); a ~1–2 MB PSRAM bitstream ring absorbs
  the 0.5 s stalls. Write in 32–128 KB chunks, not 512 KB.
* Files: one directory per session id, names from the start uptime (`/video/<boot>/<uptime_ms>.h264`).

**SD writes vs the C6 link (S5)** -- the avionics firmware mounts the card at boot (46 ms) and the console's
`sdtest <MB> [KB/s]` writes 128 KB PSRAM chunks while the hub pings the board (60 pings, 4/s):

| SD write rate | hub round trip p50 | p99 | timeouts |
|---|---:|---:|---:|
| none | 3.3 ms | 5.3 ms | 0 |
| 1 MB/s (video at 8 Mbit/s) | 4.7 ms | 10 ms | 0 |
| 2.85 MB/s | 8.1 ms | 57 ms | 0 |
| flat out (4.27 MB/s) | 8.1 ms | 54 ms | 0 |

The shared SDMMC controller is no problem: the C6 keeps its link at any SD rate. A first run looked like
starvation -- a ping over 5 s, the hub dropping the link -- but the culprit was health calling `f_getfree`, which
waits for the FatFs volume lock a writer holds (`vfs_fat: Failed to get number of free clusters (15)` =
FR_TIMEOUT). Rule: **nothing on the link or health path touches the filesystem**; free space is measured after
mounting (0 ms -- the card's FSINFO) and after write bursts, and health reads the cached figure.

### PSRAM bandwidth as DMA sees it (B3b, `src/bin/dma.rs`)

The camera, ISP and H.264 encoder move frames by DMA, so CPU copies (latency-bound, 57 MB/s) say little about
video. AXI-GDMA async memcpy, MB/s of copy (each byte read once and written once):

| route | burst 16 (IDF default) | 32 | 64 | 128 |
|---|---:|---:|---:|---:|
| PSRAM → PSRAM | 71 | 103 | 137 | **178** |
| PSRAM → internal | 106 | 132 | 175 | 175 |
| internal → PSRAM | 176 | 176 | 176 | 176 |
| internal → internal | 176 | 176 | 176 | 176 |
| PSRAM → PSRAM, 2 / 3 engines at once (burst 64) | | | 162 / 163 | |

* **The burst size is the big lever** (2.5× from 16 to 128): every video DMA should use the largest burst.
* One engine tops out at ~176 MB/s; with PSRAM on both sides, several engines together still total ~162–178 MB/s,
  so **the PSRAM at 200 MHz delivers ~330–355 MB/s of combined read + write traffic**.
* **1080p30 is bandwidth-bound on v1.x**: CSI RAW in, ISP YUV420 out (93 MB/s), the encoder reading the frame and
  its reference and writing the reconstruction is roughly 370+ MB/s -- above the measured ceiling, which matches
  others' reports of ~27–29 fps. 25 fps is borderline, **20 fps fits** (the operator's accepted fallback). The
  250 MHz PSRAM option (+25%) exists only for rev v3.x silicon (`SPIRAM_SPEED_250M` depends on
  `!ESP32P4_SELECTS_REV_LESS_V3`; the v1.3 datasheet specifies 200 MHz).

### The camera (V1/V3): OV5647 → ISP → H.264 → microSD (`components/coludo_video`)

esp_video 2.5.0 + esp_cam_sensor 2.6.0 + esp_h264 1.4.1 (rev v1.x profile) + esp_sccb_intf 0.0.9, pinned to the
IDF v5.5.5 tag. Capture through V4L2 (`/dev/video0`, 3 MMAP buffers in PSRAM) → the H.264 hardware encoder called
directly (force-IDR, frame type, QP range) → a 2 MB PSRAM bitstream ring → an SD writer task (aligned 64 KB
writes, fsync once a second) → `/sdcard/video/<session>/<start uptime ms>.h264`, raw Annex-B, an IDR every second.
Every frame carries an SEI user-data NAL with boot id, frame number and the uptime at the end of its readout
(`tools/video_sei.py` lists them); files come off the board over Wi-Fi (`GET /sd/...`, 4.6 MB/s).

Board: a 22-pin 0.5 mm camera connector (Pi-5 style, 2 lanes), no camera power or reset pin, the MIPI PHY on LDO
VO3. The camera's SCCB shares I2C port 0 (GPIO7/8) with the flight sensors, so the firmware owns port 0 as one
`i2c_master` bus (`firmware::i2c`) and skips IDF's legacy-driver conflict check (esp-idf-hal links the legacy
driver; it must never touch port 0).

Found and fixed on the way:
* **Stream-on** (esp-video-components #97): the stock driver writes `0x4800 = 0x14` and the receiver never sees
  a frame; the component wraps the driver's stream-on (`0x34`) instead of patching it.
* **Bayer order**: the stock tables run the readout mirrored but declare GBRG; the sensor's colour bars came out
  green↔magenta swapped. Recorded once per order: **BGGR** gives the exact bars (white, yellow, cyan, green,
  magenta, red, blue, black) -- what Linux's ov5647 driver declares. Overridden by wrapping the driver's format
  table (esp_video resolves every format to the table entry, so a corrected copy only works as the table).
* **The taster's module runs on a 24 MHz oscillator**: every frame rate came out exactly 24/25 of the request in
  the SEI stamps (the tables assume 25 MHz). The frame-rate stretch now computes with the real clock.

Exact frame rates from the SEI stamps, 8 Mbit/s target, sensor at night (pure noise: the hardest content):

| mode | measured fps | encode avg / max | frame interval p50 / max | missed / dropped |
|---|---:|---:|---:|---:|
| 1920×1080 @ 20 (before the clock fix) | 19.20 | 34.3 / 36 ms | 52.1 / 53 ms | 0 / 0 |
| 1920×1080 @ 25 (before the clock fix) | 24.00 | 35.9 / 61 ms | 41.7 / 62 ms | 1 / 0 |
| **1920×1080 @ 25, 65 s** | **25.010** | **36.3 / 39 ms** | **40.0 / 45 ms** | **0 / 0** |
| 1920×1080 @ 30 | 26.57 -- encoder-bound | 36.9 / 40 ms | 36.5 / 42 ms | -- |
| 1280×960 binned @ 30 (before the clock fix) | 28.79 | 19.9 / 20 ms | 34.7 / 35 ms | 0 / 0 |
| 1280×960 binned @ 45 (before the clock fix) | 43.20 | 21.1 / 23 ms | 23.1 / 23 ms | 0 / 0 |

* **FHD @ 25 fps holds**: 1626 frames in 65 s, none missed or dropped, every frame decodes; 8 Mbit/s exactly;
  worst SD write 133 ms; the ring peaked at 443 KB of 2 MB; die 37 °C while recording.
* **FHD @ 30 is out on rev v1.3**: the encoder needs ~37 ms per 1080p frame (≈27 fps ceiling) -- before PSRAM
  contention, which the DMA bench put at the edge too.
* The binned 960p mode (the sensor's full field of view; 1080p is a centre crop) encodes in ~20 ms and runs 45 fps.
* Open: a daylight image check (the night frames are sensor noise; without esp_ipa -- no rev<3 build in the
  registry -- there is no auto white balance or colour correction, the sensor's own auto-exposure only),
  orientation, the Arducam module's oscillator, CPU load and the 1 kHz loop with video running (V5).

### What video costs the flight loop (V5)

The avionics console's `load [s]` (FreeRTOS run-time stats) and `pace [s] [gptimer]` (a 1 kHz loop on core 1 at
the flight thread's priority, 20) while 1080p25 records:

| measure | no video | 1080p25 recording |
|---|---:|---:|
| CPU, core 0 | 0.1 % | **13.1 %** (SD writer 10.7 %, encoder 2.3 % -- it waits on the hardware) |
| CPU, core 1 (flight) | 0.0 % | **0.0 %** |
| 1 kHz loop, esp_timer (its task is on core 0): p99.9 / max / missed of 30 000 | 19 / 59 µs / 0 | 243 / **4146 µs / 17** |
| 1 kHz loop, **gptimer interrupt on core 1**: p99 / p99.9 / max / missed | 0 / 1 / 6 µs / 0 | 11 / 18 / **139 µs / 0** |

* **Video leaves the flight core alone** (0 % on core 1) and costs 13 % of core 0, mostly the SD writer (FatFs +
  SDMMC at 1 MB/s) -- the one thing to optimise (larger chunks, raw sectors) if core 0 gets busy.
* **Pacing from esp_timer is not good enough under video**: its task shares core 0 with the camera and SD traffic,
  and ticks went missing. A gptimer interrupt allocated on core 1 holds the loop within 139 µs worst case.
  The flight loop is paced by a core-1 timer interrupt (or the IMU's INT1 routed to core 1), never esp_timer.

### MicroPython 1.29.0 on the same board, the same day

Flashed with `tools/flash_micropython.sh` (NVS kept), the existing benches run unchanged; raw logs in
[`micropython-1.29.0/`](micropython-1.29.0/).

* **`bench_flight` step breakdown**: whole `_tick` (nav cached) **611 µs**; `_run_pid` 517 µs (85% of a step);
  one `pid.step` 35.4 µs, three 95.5 µs; `rate.value` (databoard) 26.6 µs; `mixer.actuate` 67.8 µs;
  `navigation.steer` 330 µs (throttled ~1 in 10 steps). A step is 6.1% of the 100 Hz budget.
  *Retro-finding:* its load sweep reports **0.0 Hz achieved** at every schedule -- the harness no longer drives
  the flight task (an API drift in the bench, not the flight code). Recorded for the MicroPython side.
* **`bench_commons`** (ns/call, bytecode → optimised): `clamp_int` 6137 → 2925 (viper, 2.10×), `wrap180`
  7523 → 2686 (2.80×), `between` 6737 → 4117 (native, 1.64×), `magnitude_sq` 11550 → 9232 (1.25×),
  `bank_demand` 10693 → 10168 (1.05×).
* **`bench_emitters`**: `clamp_int` bytecode 6136 / viper 2953 ns; a PID-axis float chain bytecode 11422 /
  native 9116 ns (`asm_rv32` is not in this firmware).
* **`bench_asyncio`**: PSRAM memcpy 1 MB 33.2 MB/s; `await sleep_ms(0)` 79 µs; `sleep_ms(1)` really ~10 ms
  (the asyncio floor); event ping-pong p50 ~293 µs; gc collect clean / fragmented 0.3 / 22 ms; `ticks_us()`
  2.9 µs; `bytearray(256)` 36 µs.

The Rust counterparts: control step 1.2 µs (vs 611 µs `_tick`), task-notify round trip 15 µs (vs 293 µs), no
sleep floor (esp_timer pacing 15 µs late), PSRAM copy 57 MB/s, no GC.

### Thermal soak (`src/bin/soak.rs`), bare board, no heatsink

10 min of f32 FMA on both cores, then 3 min idle, the die read every 5 s:

| idle start | plateau under full load | time to plateau | idle after |
|---:|---:|---:|---:|
| 33.3 °C | **37.3 °C** | ~2 min | 33.3 °C within a minute |

CPU load alone heats the die by ~4 °C: no throttling risk from compute. The camera/H.264 test (V8) repeats the
soak with the video pipeline running, with and without a heatsink.

### C6 link failure (`c6reset`): no longer a reboot

esp_hosted 2.7.0 restarts the whole P4 when its SDIO link fails. With `components/hosted_no_reboot`, the console's
`c6reset` (C6 EN, P4 GPIO54, low for 100 ms) gives: "sdio_write_task: Failed to send data" → esp_hosted asks for
a host restart → the guard parks the task instead; the board stays up (uptime unbroken), the console answers,
health shows `degraded: ["c6-link-parked"]`, and the link thread stops calling Wi-Fi. Without the guard the same
test logged "Unrecoverable host sdio state, reset host mcu" and rebooted. Wi-Fi returns at the next boot.

## Decided by these numbers

* **D5 pacing:** a timer interrupt on core 1 (gptimer, or the IMU's INT1 routed to core 1) → task notify; never
  `sleep`, and not esp_timer either (its task shares core 0 with video: 17 missed ticks per 30 s at 1 kHz).
* **D6 storage tier:** keep the flash ring. Build with `CONFIG_SPIRAM_XIP_FROM_PSRAM=y`.
* **D18 numerics:** f32 by default; f64 only where a critical computation needs it and the reason is written
  down -- never for incidentals like a temperature on the wire. In Rust the PID is plain f32 -- `fixed.py`'s
  integers were a MicroPython GC workaround and cost the same here.
* **Build:** `opt-level=3` + fat LTO + one codegen unit (fastest and smaller than plain O3).
* **C6 link:** host-side guard instead of a C6 reflash; a link failure parks Wi-Fi, never reboots.
* **Codegen:** `+zcb`, nothing else. PIE SIMD only via esp-dsp, and only for int16/int8 vector work.
* **Concurrency:** no mutex in the hot path (4 µs each). Atomics and notify between single owners.

## Open

* **B4 soak:** 10 minutes rather than 10 s. Then a **10 min stall test with video DMA on PSRAM**: code fetch now
  shares that bus.
* **B3b (rest):** L2 cache 256/512 KB and 128 B lines for the CPU side (DMA is measured).
* **The recorder row:** 53 µs with floats, 20 µs with integers (`core::fmt`); a digit writer would cut it ~10×.
* **esp_hosted 2.7.0, still open:** its streaming-mode RX trusts the length word -- an all-ones read becomes a
  ~1 MB allocation and an assert. Fixing that needs a patched copy of the component. Also: never call
  `esp_hosted_deinit` with the SD card mounted (2.7.0 deinits the whole SDMMC host), and decide whether a parked
  link may reboot the board on the ground (never in flight; the post-landing beacon needs Wi-Fi).
* **Camera, next:** daylight image + orientation; B4's flash stall test with video running; the Arducam module's
  oscillator; recording from the rod-placement event; the SD writer's 10.7 % of core 0.
* **Pin the nightly** (`rust-toolchain.toml` says `nightly`; it was 1.101.0-nightly 2026-10-04 here).
