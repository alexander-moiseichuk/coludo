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
  for the owner tick's lateness and the lapses;
- from R4 on, every thread's never-used stack (`heap stacks`) after the contract, against the stage before: a
  split moves locals between frames, and the 8 KB control stack has the least to spare.

| stage | what | risk |
|---|---|---|
| R0 | API docs (`27cc539`) | done |
| R1 | the utilities: `uptime_us`, `threads::spawn`, `Rows`/`LogOnce` (`8939dab`) | done |
| R2 | `crates/parts` with `Published<T>`, the parts' published state and its inspect keys, `parts::KNOWN`, the registry; `inspection::device` and the parts' statics removed (as built: below) | done |
| R3 | `ControlCore` in `crates/controller`; `control/` split by subsystem; the SITL on `ControlCore` (as built: below) | done |
| R4 | the `boot` phases; `devices` from tables; the remaining long functions (as built: below) | done |
| R5 | the guard: `too_many_lines` at 80 with the allow-list, and the length test (as built: below) | done |
| R6 | the hardware's own types (operator, 2026-10-12): see below | dropped (operator, 2026-10-12): measured, not worth it |

**R2 as built.** `crates/parts` holds every I2C part's run, generic over embedded-hal and host-tested on the
drivers' mock bus:
- `Part` (the drivers, built per attempt), `Run` (each kind's channels, stream and health), and `Member`, a part that
  came up. `calibrate`, `update`, `apply_calibration` and the warm start's pitot zero moved onto `Member`, because
  they publish.
- Reflection is the `Shared` enum: one `Published<T>` per kind (a seqlock, `parts::published`), and `parts::own`,
  the typed inspect keys in plan B's order. An enum match does it, with no trait object.
- The firmware registers each member's `Shared` by name (`sensors::register`) and looks it up for `inspect`, the
  pending calibrations, `verify`'s `imu_calibration` and the checkpoint's pitot zero.
- The board comes in through `parts::Platform` (the control thread's interrupts and lapse watches, the recorder's
  rows) and `parts::Store` (the calibrations' NVS reads).
- `parts::KNOWN` gives each part its name and default address. The streams' columns stay in
  `crates/controller` (`streams.rs`): `parts` depends on the controller's `Topology`, so the controller cannot read
  `parts`.

Measured: `sensors.rs` went from 1963 lines to 215 and `inspection::device` from 199 to 26; its other arms are
`component()` (100 lines, for R4). The firmware lost about 2100 lines, the new crate holds 3000 with its tests, and
`sensors.rs`'s 27 statics became two (the platform and the registry).

**R3 as built.** `crates/controller` `ControlCore` holds the control tick's pure duties, in the board's order:
- the attitude filter, stopped at DONE;
- the stage machine, with its rows, its vetoes and its transition's lines;
- the flight law, with flight.csv.

The host comes in through `controller::Hooks`: the clock read after the IMU's push, the mission and the drift, the
recorder's flight flag, the console's stage line, the published state, the fins and the vitals. The SITL runs it
with a lockstep host (the tick's time, the launch payload's mission, no drift, no rows) and lost its own `Flight`
and `Filter`. The board's `control.rs` became `control/` (the thread, `imu`, `separation`, `interrupts`, `watch`,
`duties`, `requests`; 605 lines the largest, from 1892), and `flight.rs` kept only the board's side (the surfaces'
servos, the vitals, the mission's view).

The proof changed, as this stage showed it had to. The SITL cannot stay seed for seed, because its copies were not
the board's: it stepped the stage machine on a 10 ms phase no board runs (0, 30, 80 ms where the board steps at 0,
40, 90). Two runs prove the extraction exact:
- the old SITL with only the board's pacing patched in flies the new SITL's 72 flights, every figure identical;
- against plan B's 20 seeds, the SITL on the board's phase fails 3 gates in 240 flights, all in the noisiest
  families (doc/sims/TMS-7-rust_hitl_m14 §3e).

On the boards, as every stage: identical inspect shapes, boot lines equal but for the log targets, the contract at 0
violations, one HITL flight, a soak.

**R4 as built.** `main` is the list of phases (73 lines, from 369), each a `boot` function documented with what it
needs and gives: `storage`, `media`, `ports`, `layout`, `recorder`, `board`, then `devices::bring_up`, `recovery`,
`flight`, `operator` and `radio`. No `Boot` struct: each phase returns what the next ones take (`Storage`, `Media`,
`Recovered`), and `main` hands it on. The rest split by state, request kind or route:
- `devices::bring_up` dispatches each entry on its `parts::classify` kind (`Bringing::entry`), gathering the members
  (`Gathered`); each kind's `set_up_*` stays its entry;
- the control thread is a `Thread` (`new`, then `tick` with `clock`, `rest` and `sweep`);
- `files::serve` is one handler per route, and the `register_commands` of `ota`, `sd` and `diagnostics` list named
  `*_command()` constructors;
- `link::run` is `start` and a `Link` state (`look`, `link_parked`, `beacon`, `join_next`); `owner::serve` is
  `touch`, `read` and `general_call`;
- `recorder::write` is `log_partition`, `scan_ring`, `SaverStart::judge`, `record_session_start` and `run_writer`;
  `recorder::open` gains `device_streams`;
- `inspection::component`, the IMU's `tick`, `record_last_boot`, `step_sweep`, `set_up_spi` and `pace` lost their
  largest arms to named functions.

Measured: functions over 80 lines 18 to 1, over 60 lines 37 to 23 (`tools/function_lengths.py`). The commit said
17 to 0: the tool then missed every function with a `;` in its signature (a `[T; 2]` parameter), `set_up_i2c` at
102 lines among them. R5 fixed the tool and split `set_up_i2c`.

The stage's first build overflowed the control thread's 8 KB stack on `probe imu_lsm6dso32`, five boots in a row, in
the probe line's float formatting (1.2 KB). `Thread::new`, inlined, kept the bring-up's temporaries and the
`ControlCore` (~1.1 KB, once per move) in the thread's frame under every tick, 5.8 KB of it. The fix boxes the core
(one allocation at the bring-up) and never inlines `new`:
- the thread's frame is now 1376 bytes, against 2960 before the stage;
- after the contract, `heap stacks` reads 2312 bytes never used on the control thread, against 1336 before;
- every other thread is within 600 bytes of before.

Never inlining `new` alone was not enough: its 3.5 KB frame on top of the thread's 3.5 KB overflowed at the bring-up
instead, every boot, until the core was boxed.

The proof added the stack check above. On the boards: the contract at 0 violations on both, identical inspect shapes
on 7F, boot lines equal but for the log targets of the lines that moved to `boot`. One HITL flight (miss 91.25 m,
plan B's band 87.2 to 103.6 m, late max 7 µs, 0 overruns) and a soak.

**R5 as built.** Two gates, one limit:
- `make lengths` (in `make check`): `tools/function_lengths.py --check` fails on a firmware function over 80 lines
  that `tools/function_lengths.allow` does not name, and on an allowed one no longer over 80. It needs no build, so
  it runs where the esp toolchain does not. The allow-list is empty, so the file does not exist yet; an entry is
  `<path under src/avionics>::<function>  # why`.
- clippy's `too_many_lines` at 80 on the firmware crate (`[lints.clippy]` in its Cargo.toml, `clippy.toml` beside
  it). Clippy also reads the bench binaries, which the tool skips: their four long measurement lists carry an
  `#[expect]` with the reason.

The firmware's clippy now passes `-D warnings`. The stage cleared the warnings left in the bench binaries
(`is_multiple_of`, a type alias, a collapsed `if`, a `black_box` for a formatted literal) and two `thread_local`
initializers that are already `const`: clippy on the esp target reads them as not, so the expectation is scoped to
that target. The tool's fix found 19 functions it had missed; `set_up_i2c` (102 lines) split into `i2c_binding` and
`i2c_member`. Proof: the taster's contract at 0 violations, the stacks as R4's within run-to-run noise.

**R6, the 32-bit types.** The P4 is RV32 with a single-precision FPU: a u64 or i64 takes two registers, and its
division is a library call. An f64 is all software. The tree has 480 lines with a u64, 235 with an i64 and 166
with an f64 (no 64-bit atomic: RV32 has none). The stage moves them to i32/u32/f32:
- timestamps: u32 with wrapping differences (`wrapping_sub` read as i32), in microseconds for the flight's short
  spans (35 minutes either way) and in milliseconds for the long ages (the beacon's fix age, hours after the
  landing; 49 days). IDF's i64 timer read stays, cut to u32 at the one read (`firmware::time`);
- positions: i32 in 1e-7 degrees (1.1 cm), as the beacon's `launch_e7` already is. Distances and offsets are in
  f32 metres, accumulated in metres and converted once (plan B's single-precision lesson: a 0.2 m step added to
  latitude 48 in f32 is lost);
- the control maths (`crates/control`, `crates/controller`, the firmware): f32;
- a 64-bit type stays only at a boundary, with its reason in a comment: IDF's own APIs, byte counts on cards over
  4 GB, and JSON numbers (serde_json's f64, converted where the config is read).

R6 is the one stage that changes behaviour, as rounding: its proof is the HITL matrix against plan B's bands
(`tools/hitl_matrix_compare.py`), not seed for seed. The SITL's golden runs are regenerated after it.

**Measured before R6 (2026-10-12, the R5 build).** On RV32 a 64-bit add, compare or shift is a few instructions;
only division and modulo are library calls (`__divdi3`, `__moddi3` ..., ~100 cycles), and f64 arithmetic is
software. On the control tick's path the build makes about 20 such calls per 10 ms tick: 9 `__moddi3` in
`ControlCore::tick` (the antimeridian wrap of `navigation::offset`, inlined), the clock's `now_us / 1000`, one per
fin (`sg90` `floor_divide`, `fins::write`'s duty), the BMM350's read and the BMP280's datasheet compensation. That
is 2-4 k cycles, 6-11 µs per tick, about 0.1% of core 1, whose tick works ~100 µs. The f64 left is the JSON boundary
(D18) and set-up code (`FlightLaw::new`, `from_json`); no 64-bit atomic exists (the seqlocks). The timebase
migration's gain is that 0.1%; its cost is ~600 sites that must all wrap correctly at 71.6 minutes of uptime, a
pad wait the boards can reach.

**R6 dropped (operator, 2026-10-12):** "64-bit integers not an issue at all"; a u64 `%` is nothing at 0.1% of a
core. The u64 timebase stays, as does f32 for the maths (D18). Optimizations come later, from profiling, and core
placement stays static: each thread pinned to a core chosen from a measured load profile, no migration by load.

**Crates this needs:** possibly `enum_dispatch` (R2, compile-time only), if it takes the generic part trait.
Everything else is already in the workspace: `embedded-hal`, `serde`/`serde_json`, `crates/controller`,
`crates/databoard` and `crates/recorder`.

**Not in this plan:** behaviour changes, except R6's rounding. A finding the refactor turns up gets its own
commit, as every finding did in the milestones.
