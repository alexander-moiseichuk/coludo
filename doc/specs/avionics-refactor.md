# Avionics refactoring plan (Phase 6, after M21)

The Rust port (`src/avionics`, [avionics-port.md](avionics-port.md)) reached plan B's behaviour milestone by
milestone, and its shape shows it: each milestone added its arms to the same few firmware functions. This plan
restructures the code without changing what it does. Every stage is proven like a milestone (§4), and is committed
on its own.

The operator's asks (2026-10-12):
1. No huge functions such as `main`: decompose them.
2. Restore plan B's reflection (its `Inspectable` objects and `Task` lifecycle) and a driver abstraction, naming
   the crate each abstraction lives in and any crate it needs.
3. API docs generated as for Python: the legacy one as `doc/glider_api.md`, the new one as `doc/avionics_api.md`
   (done in R0).
4. Investigate the codebase first, so the decomposition is the most effective one (§1).

## 1. What the investigation found (measured 2026-10-12)

**The crates are healthy, the firmware crate is not.** Function lengths, with comments and string literals blanked
(the API generator's lexer, so a `{` inside a `format!` text is not code):

| | functions | over 100 lines | over 60 lines | host-tested |
|---|---|---|---|---|
| `crates/*` (19 crates) | 2169 | 0 | 6 | yes, every crate |
| `firmware/src` (bench binaries excluded) | 591 | 13 | 41 | no: it builds for the ESP32-P4 only |

The longest firmware functions: `main` 369 lines, `control::run` 222, `files::serve` 202, `inspection::device` 199,
`ota::register_commands` 157, `recorder::write` 143, `link::run` 135, `devices::bring_up` 131, `Part::tick` 116,
`owner::serve` 113.

**Much of the firmware crate is no hardware code.** It holds logic that would test on the host, but nothing tests
it there, so it grows where it stands. Calls into ESP-IDF (`sys::`) per file:

| file | lines | `sys::` calls | what it is |
|---|---|---|---|
| `sensors.rs` | 2062 | 1 | every I2C part's run: publish, stall and freshness judgement, calibration, inspection state |
| `commands/*` | 2163 | ~2 | the CC answers: JSON shapes, readiness, degraded flags |
| `gnss.rs` | 480 | 1 | the GNSS thread's line handling around `crates/gnss` |
| `gcl.rs` | 405 | 0 | the command registry and its validation |
| `control.rs` | 1963 | 74 | eight subsystems in one file (below), half of them pure |

**State crosses threads through 116 static atomics in 17 files** (`control.rs` 36, `sensors.rs` 25), plus 28
`Mutex`/`OnceLock` statics. Almost every `inspect` field, `health` flag and readiness item has its own static and
its own accessor function (`absolute_inspection`, `magnetometer_inspection`, `power_inspection`, ...).

**The SITL duplicates the board's orchestration.** `sitl/src/main.rs` has its own `Filter` (the attitude filter's
pacing and publish) and `Flight` (the stage machine's step and the law's step). They mirror `control.rs`'s `Filter`
and `Sequencer` and the flight loop. That is plan B's sim-mirrors-flight-code trap: two copies drift, and a
comparison then grades the copy. Plan B moved its detectors into commons to escape it.

**`control.rs` is eight subsystems:** the thread and its gptimer alarm, the request mailbox, the separation switch,
the SPI IMU, the lapse watches, the stage machine's glue, the attitude filter, and the GPIO interrupts. The INA226
ALERT and the BMI323 INT1 sit there only because this thread installs the ISR service.

**`devices.rs` is plan B's task registry by hand:** 13 `*Member` types and 12 `set_up_*` functions, one per kind.

**Adding a part is scattered.** The BNO055 (M18) touched about 17 places in 7 files:
- `sensors.rs`: the enum, the macro, build, diagnose, probe, tick, first, calibrate, apply_calibration, update, a
  new run and its statics;
- `inspection.rs`, `identity.rs`, `diagnostics.rs`, `owner.rs`, `main.rs` and `controller` `streams.rs`.

**Boilerplate repeated by hand:**
- 42 uptime reads, each `unsafe` with its own SAFETY comment;
- 15 thread spawns, each with the same PSRAM-stack `ThreadSpawnConfiguration` block;
- 24 log-once flags (`failing`, `stale`, `overflowing` ...);
- 38 GCL commands whose bodies sit inline in long `register_commands` functions.

**What already serves:** `crates/drivers` is bus-generic over embedded-hal (`I2cPort` implements its `I2c`). The
host crates also already provide the rest of what the parts glue uses: `crates/databoard` (writers),
`crates/recorder` (`Telemetry`), `crates/controller` (`Topology`, `DeviceEntry`, streams). The parts glue's only
firmware ties:
- the recorder's globals (a `Telemetry` can be handed in);
- the control thread's interrupt counters (a counter can be handed in);
- NVS for the calibrations (stays in the firmware);
- the esp-hal types: `Delay`, `PinDriver`, `I2cPort`, each with an embedded-hal trait.

## 2. The decomposition: extract, then split

Splitting the long functions in place would leave the logic untested and in the firmware. The decomposition with
the most effect moves the hardware-free logic into host-tested crates, where this codebase already stays short and
tested. The firmware is left as the platform: threads, ESP-IDF calls, buses, NVS, radio, card and camera. Most long
functions then dissolve on the way, and the rest are split last.

### 2.1 `crates/parts` (new): the parts, with reflection

Every I2C part's run moves out of `sensors.rs`, generic over embedded-hal (`I2c`, `DelayNs`, `OutputPin`), one file
per part:
- `parts::bno055` is the driver plus its databoard writers, its stream and its snapshot;
- the stall withhold, the calibration latch, the alert count and the freshness judgements become host-tested with
  `drivers`' `MockBus`, positive and negative.

A `Part` enum and a `Run` trait (setup, rearm, diagnose, probe, tick, calibrate, update, first, rezeroes) replace the
`each!` macro and the 87 hand arms. The dispatch is static, so the owner's tick stays as cheap as today's `match`
(no `Box<dyn>`, no heap). `enum_dispatch` (a compile-time proc macro) writes it if it takes the generic trait;
otherwise one small macro inside the crate does.

One table, `parts::KNOWN`, gives each implementation name:
- its kind and its default address;
- its builder;
- its stream's columns, which `crates/controller` `streams.rs` then reads instead of its own `match`.

Adding a part becomes one driver file, one parts file and one table line.

**Reflection, plan B's `Inspectable`, comes back with it:**
- each part publishes a plain `Copy` snapshot into a `Published<T>` slot (a seqlock, in the crate, host-tested);
- the snapshot implements `Inspectable`: `kind`, `inspect` (plan B's keys, as JSON), `stats`, `calibration`;
- the firmware keeps a registry, `name -> &Published<_>`, so `inspect <name>`, `stats`, `calibrate`'s pending list,
  `health`'s `imu_calibration` and `verify`'s readiness all look the part up by name.

`inspection::device`'s 199-line match goes, and so do the parts' statics. The parts never run on the executor's
core: the owner (core 1) writes, and the executor (core 0) reads and renders.

The firmware keeps about 200 lines of glue: building the parts over `I2cPort`, arming their interrupts, and the
calibrations' NVS.

### 2.2 `crates/controller`: one control core for the board and the SITL

The control tick's pure duties move into `crates/controller` as `ControlCore`:
- the attitude filter's pacing, assembly and publish;
- the stage machine's step and its records (`sequencer.csv`, `vetoes.csv`);
- the flight law's step;
- the lapse watches.

It records through a `Telemetry` handed in, and logs only constant texts through a sink (core 1 formats nothing, as
today), with no allocation per tick. The firmware's control thread keeps what is hardware: the gptimer and its
waking, the SPI IMU's reads, the separation pin and its edges, the GPIO interrupts and the fins' LEDC.

The SITL then runs the same `ControlCore` and deletes its own `Filter` and `Flight`. The SITL becomes the board's
code path minus the platform, which closes the sim-mirrors gap.

### 2.3 The firmware: the platform, by subsystem

- **Utilities first:**
  - `clock::uptime_us()` for the 42 timer reads (one `unsafe`, one SAFETY comment);
  - `threads::spawn(Spec)` for the 15 spawns (name, stack, priority, core, PSRAM stack);
  - a `LogOnce` latch for the 24 log-once flags.
- **`control.rs`** becomes `control/`: `thread.rs` (alarm, wakers, start, run), `requests.rs`, `imu.rs` (SPI),
  `separation.rs`, `interrupts.rs` (the ISR service and every GPIO counter: INT1, the second IMU, the ALERT, the
  separation edges). The duties `ControlCore` took leave with it.
- **`main`** becomes a `boot` module. One function per phase, each documented with what it needs and gives:
  1. `storage` (NVS, boot id, config, recorder);
  2. `ports` (the lent I2C ports);
  3. `layout` (scan, resolve, mask, clocks);
  4. `devices` (bring-up);
  5. `recovery` (warm start, calibrations);
  6. `threads` (owners, control, supervisor, sim-link, GNSS);
  7. `operator` (commands, executor, console, entry);
  8. `radio` (beacon, link).

  A `Boot` struct carries what passes between them, and `main` reads as the list of phases, under 40 lines.
- **`devices.rs`** brings up from tables: `parts::KNOWN` for the I2C parts, a component table for the rest
  (sequencer, attitude, GNSS, servos ...), each kind's `set_up_*` kept as its entry.
- **The rest of the long functions:**
  - `files::serve` becomes one handler per route, from a table;
  - the `register_commands` functions (ota, sd, diagnostics, video) list `Command`s whose bodies are named
    functions;
  - `recorder::write`, `link::run` and `owner::serve` split by state or request kind.
- **A guard:** clippy's `too_many_lines` at 80 for the firmware crate (`clippy.toml`), with a short allow-list of
  measured exceptions, each saying why. A length test in `make test` fails the build on a new long function.

### 2.4 API docs (R0, done)

`doc/glider_api.md` (`tools/gen_docs.py`, plan B and the hub) and `doc/avionics_api.md`
(`tools/gen_avionics_docs.py`, the Rust crates from their `//!` and `///` docs, parsed, never built). `make
docs-check` fails on a stale one. A new crate gets a chapter on its own.

## 3. Expected result

| | now | after |
|---|---|---|
| firmware functions over 60 lines | 41 | under 5, each allow-listed |
| `sensors.rs` | 2062 lines in the firmware | about 200 of glue; the rest in `crates/parts`, host-tested |
| `control.rs` | 1963 lines, eight subsystems | `control/` by subsystem; the pure tick in `crates/controller` |
| cross-thread statics | 116 atomics + 28 locks | the ISR counters and the platform's own; the inspection state in `Published<T>` |
| SITL orchestration | its own copy | the board's `ControlCore` |
| adding an I2C part | about 17 places, 7 files | a driver, a parts file, a table line |

## 4. Stages and proof

Each stage keeps the behaviour. Its proof:
- host: every crate's tests (the new ones included), clippy `-D warnings`, the firmware build, `make docs-check`;
- taster: boot, the boot log's lines unchanged (timestamps and the build masked), and `tools/cc_contract.py taster
  --milestone M21 --allow-active` at 0 violations with every `inspect` answer identical to before (captured once
  per board before R2, diffed after each stage);
- one clean HITL flight;
- for the parts and control stages, also the SITL matrix seed for seed identical to before, and a 10-minute soak
  for the owner tick's lateness and the lapses.

| stage | what | risk |
|---|---|---|
| R0 | API docs (`27cc539`) | done |
| R1 | the utilities: `uptime_us`, `threads::spawn`, `LogOnce` | low: mechanical, proven by the boot log |
| R2 | `crates/parts` with `Published<T>` and `Inspectable`, part by part (the BNO055 first), `parts::KNOWN`, the registry; `inspection::device` and the parts' statics removed | medium: every inspect answer must stay identical |
| R3 | `ControlCore` in `crates/controller`; `control/` split by subsystem; the SITL on `ControlCore` | medium: the SITL matrix must stay seed for seed |
| R4 | the `boot` phases; `devices` from tables; the remaining long functions | low to medium |
| R5 | the guard: `too_many_lines` at 80 with the allow-list, and the length test | none |

**Crates this needs:** possibly `enum_dispatch` (R2, compile-time only), if it takes the generic part trait.
Everything else is already in the workspace: `embedded-hal`, `serde`/`serde_json`, `crates/controller`,
`crates/databoard` and `crates/recorder`.

**Not in this plan:** behaviour changes. A finding the refactor turns up gets its own commit, as every finding did
in the milestones.
