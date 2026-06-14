# Coludo — Development & Testing Guide

How we build, flash, and test Coludo across its components. This captures the working
conventions; the architecture itself lives in [`specs/`](specs/) (`coludo.md`,
`board-config.md`, `cc-protocol.md`).

## Repository layout

| Path | Contents |
|------|----------|
| `src/glider/` | Main Controller firmware — **MicroPython**, runs on the ESP32-P4 / ESP32-C6 board |
| `src/glider/test/` | Test cases for every glider module (**required** — see [Testing](#testing-requirements)) |
| `src/control/` | Control Center (CC) application — **Python**, runs on the host/PC |
| `src/camera/` | Recorder module (Luckfox Pico) — already implemented |
| `specs/` | Architecture & protocol specs |
| `doc/`, `models/`, `videos/` | Hardware notes, STL models, flight footage |

New controller code goes under `src/glider/`; **all Control Center code goes under
`src/control/`**.

## Working with a connected board

A board is normally connected over USB serial (115200 baud). Use whichever tool fits the
task — **`ampy`, `mpremote`, or `rshell`** are all fine for pushing files, opening the REPL,
and inspecting the on-device filesystem. `mpremote` is the most capable for quick loops
(`mpremote cp`, `run`, `ls`, `mount`).

## Build: compile to `.mpy` with `mpy-cross`

Compile modules ahead of flashing:

```
mpy-cross -O3 module.py -o module.mpy
```

Why this is the default:

- **Catches syntax / compile errors on the host** before anything reaches the board.
- **Much faster upload** — `.mpy` bytecode is smaller than source, which matters a lot over
  the 115200 link.
- **Faster import and lower RAM** on the device (no on-device compile step).
- `-O3` is the highest optimization level (drops assertions, `__debug__` blocks, and source
  line numbers). Trade-off: tracebacks lose line numbers, so develop against `.py` source and
  ship `-O3` `.mpy`.

Keep `.py` as the source of truth and treat `.mpy` as build artifacts (don't hand-edit; they
can be gitignored). Upload `.mpy` to the board.

## Test network

An ad-hoc test Wi-Fi network, **SSID `panda`**, is available for connectivity testing
(board ↔ CC links, telemetry, console). The password is **not a secret** — ask if you need it.

Point a board at it via the `wifi` section of its `board.json` (`ssid=panda`, plus
`cc_host`/`cc_port` for the running CC instance). See `board-config.md`.

## Testing requirements

**Every module under `src/glider/` must have comprehensive test cases under
`src/glider/test/`.** A new module and its tests land in the same change.

This dovetails with the task model in `coludo.md`, where each task exposes `testing()` and
`validate()`. Tests should cover:

- `setup()` / `run()` / `validate()` happy paths and resets,
- edge cases and **degraded / failure paths** (sensor timeout, missing component, bad data),
- **sensor-fusion selection** (priority + timeout ordering, switch to backup),
- **config parsing and validation** (pin uniqueness, bad bus refs, fallback to defaults),
- **protocol parsing** (command tokenizer, `key=value`, quoted values, error codes).

Prefer tests runnable **on-device** (`mpremote run`). Where the logic is host-portable — the
config loader/validator, fusion ordering, the CC protocol parser — also make it **runnable on
the host** for fast iteration without a board.

**Running the tests.** `src/glider/test/run_tests.sh` (or `make test` in that directory)
compiles every `test_*.py` with `mpy-cross -O3`, runs it on the board, and prints a pass/fail
report. Convention: a test is named `test_*.py` and **passes if it compiles and runs to
completion without raising** (mpremote exit 0); it **fails** on a compile error, an uncaught
exception / failed `assert`, a timeout, or `FAIL`/`Traceback` in its output. So write checks
as `assert`. Override the port with `PORT=/dev/ttyACM0` and the per-test limit with
`TIMEOUT=<secs>`. `make bench` runs the benchmark.

## Control Center

All CC code lives in `src/control/` and is plain **Python** for the host: stdlib `asyncio`,
no third-party dependencies (per `cc-protocol.md`).

## Typical dev loop

1. Edit the module in `src/glider/`.
2. `mpy-cross -O3` to compile — catches errors and shrinks the upload.
3. Push the `.mpy` with `mpremote` / `ampy` / `rshell`.
4. Run the tests on-device: `cd src/glider/test && make test` (or `./run_tests.sh`).
5. Observe live behaviour through CC (telnet on 1235, browser on 8080) over the `panda` network.
