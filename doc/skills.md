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

Point a board at it via the `wifi` section of its `board.config` (`ssid=panda`, plus
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

Every module gets comprehensive `test_*.py` (positive **and** negative cases). The **glider**
firmware targets **MicroPython only** — its tests run **on the board** (`mpremote`), not on a host
CPython approximation; even host-portable logic (config loader/validator, fusion ordering, the
protocol parser) is verified on the real runtime. **Control** (`src/control/`) is **host CPython**,
so its tests run with `python3`: `cd src/control/test && make test`.

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
  Docstrings use `"""` (ruff / PEP 257 — the one exception, applied by `ruff format`).
- **No abbreviations**, and an argument's name matches the field it sets: `capacity`/`cell_size`
  not `slots`, `max_payload` not `maxpay`, `storage` not `buf`, `servo_eleron_left` not `..._l`.
- **PEP8**; module-internal names and classes start with `_` (e.g. `_Msg`, `_is_simple`).
- **Qualify project-module references**: import our own modules whole and call *through* them —
  `import cc_client` then `cc_client.create_dispatcher()`, not `from cc_client import create_dispatcher`.
  The module scope stays visible at the call site (which catches edit errors) and a module's external
  surface is greppable (`cc_client.`); this matters most when pulling several names from one module.
  Standard-library / builtin imports are fine as-is (`import json`, `from micropython import const`,
  `from machine import RTC`), and `cc_protocol` keeps its `as cc` alias. This holds in a module's
  **own** unit test too — `import mission` and reach its internals qualified (`mission._load`),
  renaming any instance variable that would otherwise shadow the module. Give the rename a
  **problem-specific** name that reads (`launch = mission.Mission(...)`; a Controller Task becomes
  `new_task` / `pending_task` / `closing_task` by lifecycle phase) — never a bare `task_` disambiguator.
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
- **API docs**: `python3 tools/gen_docs.py` regenerates [`doc/api.md`](api.md) from module
  headers + docstrings by *parsing* the sources (stdlib `ast`, never imports them — so it works
  for the firmware too). Regenerate it when public signatures or docstrings change.

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
4. Observe live behaviour through CC (telnet on 1235, browser on 8080) over the `panda` network.

## Scripting a board over CC (`tools/cc.py`)

For anything past a one-off poke, drive the board over CC instead of the interactive console — it
is the *designed* control channel (the dispatcher already answers `verify`/`probe`/`inspect`/
`update`/`arm`/`tlm`/`get-config`/`set-config`/...), and it sidesteps the flaky USB-CDC/rshell path
for complex bring-up. Start the hub once (`cd src/control && python3 main.py`); the board joins
`panda` and dials in. Then each command is one scriptable line with a verdict exit code:

```
tools/cc.py taster verify                 # pre-flight pass/fail — exit 0 == clean, 1 == problems
tools/cc.py taster probe imu_lsm6dso32    # one device self-test
tools/cc.py taster inspect mission        # an inspectable's snapshot (pretty-printed JSON)
tools/cc.py taster tlm 2000               # telemetry rows buffered since the last tlm
tools/cc.py taster set-config launch @launch.config   # @path -> file contents (config push)
tools/cc.py taster update mission '{"launch_id":"flight.8"}'
```

It talks to the hub's `POST /api/cmd` (web port 8080; `--host`/`--port` to point elsewhere). Exit
code is the board's verdict (`0` only on `ok`/`pong`/`iam`), so on-device tests chain in bash:
`until tools/cc.py taster ping; do sleep 2; done && tools/cc.py taster verify`. Note: it bypasses
any `http_proxy` (the hub is LAN/localhost — a system proxy would otherwise hijack the request).

## Working effectively with the AI agent

Much of this codebase is built in pair with Claude Code. To get reliable results (not just fast
typing), a few things matter more than clever prompts:

- **Front-load the spec and the acceptance test.** State up front what "done" means ("all objects
  inspectable, verified over a real wifi exchange"). It turns "build it all" into a target the
  agent can sequence toward.
- **Batch related thoughts** into one message instead of a stream of interrupts; each mid-flight
  scope flip costs a re-plan (and tokens).
- **Say the budget/completeness tradeoff** explicitly when they conflict.
- **Keep the loop tight:** conventions in `CLAUDE.md`/this file, a structured findings list,
  on-device tests after each change, small commits.

Reading:

- [Anthropic — Claude Code best practices](https://www.anthropic.com/engineering/claude-code-best-practices)
  — `CLAUDE.md`, permissions, plan mode, custom commands.
- [Anthropic — prompt engineering docs](https://docs.anthropic.com/en/docs/build-with-claude/prompt-engineering)
  — be explicit, give context/examples, let the model reason before acting.
- [Anthropic — Building effective agents](https://www.anthropic.com/engineering/building-effective-agents)
  — orchestration patterns (subagents, fan-out, verify-then-commit) for the bigger pieces.
- [Simon Willison — LLMs](https://simonwillison.net/tags/llms/) — practical, skeptical field notes
  on coding with these tools.
