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

The glider firmware targets **MicroPython only** — tests run **on the board** (`mpremote`), not
on a host CPython approximation. Even host-portable logic (config loader/validator, fusion
ordering, the protocol parser) is verified on the real runtime.

**Running the tests.** `src/glider/test/run_tests.sh` (or `make test` in that directory) first
deploys the glider modules (`src/glider/*.py`) to the board with `deploy_modules.sh` so tests
can `import` them, then compiles every `test_*.py` with `mpy-cross -O3`, runs it on the board,
and prints a pass/fail report. Convention: a test is named `test_*.py` and **passes if it
compiles and runs to completion without raising** (mpremote exit 0); it **fails** on a compile
error, an uncaught exception / failed `assert`, a timeout, or `FAIL`/`Traceback` in its output.
So write checks as `assert`. Override the port with `PORT=/dev/ttyACM0` and the per-test limit
with `TIMEOUT=<secs>`. `make bench` runs the benchmark; `make test-recorder` runs the
adb-backed recorder UART integration test (which reads its UART pins from the board's config).

## Coding conventions

Follow these so code is right the first time; `ruff` and the `deploy.sh` gate enforce most of
them. `src/glider/` is **MicroPython** (the board); `src/control/` is **CPython 3.12** (the host);
`cc_protocol.py` is **shared** (it lives in `src/glider/` and is symlinked into `src/control/`),
so it must run on both.

- **Strings**: single quotes `'...'`; double quotes only when the literal contains a single quote
  (e.g. `"board.mcu '%s' invalid"`) — don't backslash-escape when switching the outer style avoids it.
- **No abbreviations**, and an argument's name matches the field it sets: `capacity`/`cell_size`
  not `slots`, `max_payload` not `maxpay`, `storage` not `buf`, `servo_eleron_left` not `..._l`.
- **PEP8**; module-internal names and classes start with `_` (e.g. `_Msg`, `_is_simple`).
- **Type annotations** on every non-local: module constants, class variables, function arguments
  and return types (both CPython 3.12 and MicroPython 1.28 accept them).
- **Constants** via `micropython.const`, with a portable shim at the top of shared/board modules:
  ```python
  try:
      from micropython import const
  except ImportError:        # CPython (Control)
      def const(x): return x
  ```
- **Docstring or comment per entry** — every class, method, and non-trivial constant.
- **Slim classes, YAGNI**: no unused parameters or speculative flexibility. If the class already
  holds a value, don't also pass it in.
- **Error policy by criticality**: logs are best-effort — drop silently (or truncate) when the
  buffer is full; **telemetry is important — raise** when a record won't fit or there is no room.
- **Async I/O**: wrap blocking peripherals (UART) in `asyncio.StreamWriter`/`StreamReader` and
  `await writer.drain()`; never do a blocking write inside a task. Long-running loops run forever
  (no `stop` flag) — a wedged board reboots via the watchdog.
- **Inspectable**: any object an operator inspects/tweaks via Control implements the `Inspectable`
  mixin — `inspect() -> dict` (json-able properties), `update(dict) -> list` (names of properties
  actually changed), `stats() -> dict` — plus `type`/`name`. Design now, adopt per-class incrementally.
- **Tests cover positive *and* negative** cases, on the board (`test_*.py`, `make test`).

### Tooling

- **ruff** for lint + format (`ruff check`, `ruff format`; config in `ruff.toml`).
- **`deploy.sh`** maps `src/glider/` → `/pyboard/`: each Python file is ruff-checked and
  `mpy-cross`-compiled then pushed; non-Python files are pushed as-is; `test/` → `/pyboard/test`.
- The Wi-Fi password is **not** committed: a `src/glider/<ssid>.creds` file (e.g. `panda.creds`,
  one line — the plain password) is gitignored (`*.creds`) and pushed by `deploy.sh`; `wifi.py`
  reads it for the password.

## Control Center

All Control code lives in `src/control/` and is plain **CPython 3.12** for the host: stdlib
`asyncio`, no third-party dependencies (per `cc-protocol.md`). The protocol parser is shared:
`src/glider/cc_protocol.py` is symlinked as `src/control/cc_protocol.py`, so edit it once (in
`src/glider/`) and keep it CPython+MicroPython portable.

## Typical dev loop

1. Edit the module in `src/glider/`.
2. `./deploy.sh` — ruff-checks + `mpy-cross`-compiles each Python file and pushes to `/pyboard/`
   (non-Python as-is); fails before touching the board if lint or compile fails.
3. Run the tests on-device: `cd src/glider/test && make test` (or `./run_tests.sh`).
5. Observe live behaviour through CC (telnet on 1235, browser on 8080) over the `panda` network.
