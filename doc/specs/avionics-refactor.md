# Avionics refactoring plan (Phase 6, after M21)

The Rust port (`src/avionics`, [avionics-port.md](avionics-port.md)) reached plan B's behaviour milestone by
milestone, and its shape shows it: each milestone added its arms to the same few functions. This plan restructures
the code without changing what it does. Every stage is proven by the same checks as a milestone (below), and is
committed on its own.

The operator's asks (2026-10-12):
1. No huge functions such as `main`: decompose them.
2. Restore plan B's reflection (its `Inspectable` objects and `Task` lifecycle) and a driver abstraction, naming
   the crate each abstraction lives in and any crate it needs.
3. API docs generated as for Python: the legacy one as `doc/glider_api.md`, the new one as `doc/avionics_api.md`.

## 1. Where it stands (measured 2026-10-12)

**Long functions** (firmware, excluding the bench binaries), by lines:

| function | lines | what it does |
|---|---|---|
| `main.rs` `main` | 369 | the whole boot: storage, ports, layout, recorder, bring-up, warm start, threads, commands, link |
| `control.rs` `run` | 222 | the control thread: set-up, then every tick's duties inline |
| `files.rs` `serve` | 202 | every HTTP route in one function |
| `commands/inspection.rs` `device` | 199 | `inspect <device>`: one arm per implementation |
| `ota.rs` `register_commands` | 157 | six GCL commands, their bodies inline |
| `recorder.rs` `write` | 143 | the ring writer's loop |
| `link.rs` `run` | 135 | the Wi-Fi link's states in one loop |
| `devices.rs` `bring_up` | 131 | every entry kind's bring-up |
| `sensors.rs` `Part::tick` | 116 | one arm per part |
| `owner.rs` `serve` | 113 | one arm per owner request |

55 firmware functions run over 60 lines, and 16 over 100. `sensors.rs` (2062 lines) and `control.rs` (1963) are the
two largest files.

**No reflection.** Plan B answered `inspect`, `update`, `stats`, `probe`, `calibrate` and `calibration` through one
interface (`inspector.Inspectable`, `task.Task`) and one registry (`Inspector`). The port answers them through
`match` statements on the implementation name: `inspection.rs` `device` for inspect, `Part`'s methods for the rest.
Each part also publishes its own ad-hoc statics for those answers (`MAGNETOMETER_*`, `ABSOLUTE_*`, `PITOT_STATE`,
`POWER_STATE`, `BACKUP_TORN`, ...).

**No driver abstraction above `Device`.** `crates/drivers` has `Device { setup, rearm }`. The firmware's `Part` enum
dispatches everything else by hand: an `each!` macro plus 87 `Part::` arms in `sensors.rs`, and a `Run` enum beside
it for the outputs. Adding the BNO055 (M18) touched about 17 places in 7 files:
- `sensors.rs`: the enum, the macro, build, diagnose, probe (step and outcome), tick, first, calibrate,
  apply_calibration, update, and a new run with its statics;
- `inspection.rs`, `identity.rs`, `diagnostics.rs`, `owner.rs`, `main.rs` (the profile restore needed the ports), and
  `controller` `streams.rs`.

## 2. Target design

### 2.1 The part trait (`crates/drivers`, firmware glue)

`crates/drivers` gains the capabilities every part shares, next to `Device`, bus-generic over embedded-hal:

```rust
/// What the operator's `probe` reads: the part identifies itself and one reading comes back.
pub trait Probe<Bus: I2c> {
    /// The step's name in the log (`chip id + read`, `data`, ...).
    const STEP: &'static str;
    /// The reading as the log shows it, or why it failed.
    fn probe(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<String, String>;
}

/// The wire-level classification of a failed setup (`diagnose`).
pub trait Diagnose<Bus: I2c> {
    fn diagnose(&self, bus: &mut Bus) -> String;
}
```

The firmware keeps one `Part` enum, its variants being one glue type per part (`parts::Bno055`: the driver, its
databoard writers, its stream, its published snapshot), and dispatches through a firmware trait with
**`enum_dispatch`** (crate `enum_dispatch`, a proc macro with no runtime part). That replaces the `each!` macro and
the hand-written arms with static dispatch: the owner's tick stays as cheap as today's `match`, with no `Box<dyn>`
and no heap.

```rust
#[enum_dispatch]
pub trait I2cPart {
    fn setup(&mut self, port: &mut I2cPort, delay: &mut Delay) -> Result<(), SetupFault>;
    fn rearm(&mut self, port: &mut I2cPort, delay: &mut Delay) -> Result<(), String>;
    fn diagnose(&self, port: &mut I2cPort) -> String;
    fn probe(&mut self, name: &str, port: &mut I2cPort, delay: &mut Delay) -> Option<String>;
    fn tick(&mut self, name: &str, port: &mut I2cPort, delay: &mut Delay, now_us: u64) -> Option<Event>;
    fn calibrate(&mut self, name: &str, port: &mut I2cPort, delay: &mut Delay) -> Calibrated { .. default .. }
    fn update(&mut self, properties: &Value, now_us: u64) -> Result<Vec<&'static str>, String> { Ok(vec![]) }
    fn first(&self) -> bool { false }       // an IMU: first on each owner wake
    fn rezeroes(&self) -> bool { false }    // a barometer: calibrate and update at DONE too
}
```

Each part lives in its own file, `firmware/src/parts/<part>.rs`. It implements the trait once and folds today's
`Run` into the same type. One registration table, `parts::KNOWN`, gives each implementation name:
- its kind (I2C, SPI, native...);
- its default address;
- its builder;
- its stream's columns, which `crates/controller` `streams.rs` reads instead of its own `match`.

Adding a part becomes one driver file in `crates/drivers`, one glue file in `firmware/src/parts`, and one line in
`KNOWN`.

### 2.2 Reflection (`crates/controller`, firmware registry)

Plan B's `Inspectable` came back as a trait, with plan B's object model: each operator-facing object answers in its
own words.

```rust
/// An operator-facing object (plan B inspector.Inspectable): what `inspect`, `stats` and `calibrate` answer for it.
pub trait Inspectable {
    fn kind(&self) -> &'static str;                     // its implementation: 'bno055', 'sequencer', ...
    fn inspect(&self) -> serde_json::Value;             // the snapshot, plan B's keys
    fn stats(&self) -> serde_json::Value { json!({}) }
    fn calibration(&self) -> Option<String> { None }    // what it still waits for
}
```

The parts run on the bus owners (core 1), and `inspect` runs on the executor (core 0), so the executor never calls a
part. Each run publishes a plain snapshot struct (numbers, `Copy`) into a lock-free slot, `Published<T>`. A seqlock
in `crates/controller` replaces the per-part atomics: the owner writes it on its tick, and the executor reads it and
renders the JSON (the `Inspectable` impl is on the snapshot). One registry, `firmware::inspector`, maps each entry
name to its slot's renderer:
- `inspect <name>`, `stats <name>` and `calibrate` (the pending list) look the name up;
- the `device` match (199 lines) and the ad-hoc statics go;
- `health`'s `imu_calibration` and `verify`'s readiness ask the registry, not a part's own function.

The plan B lifecycle maps onto it as before:
- `Task.setup` is the trait's `setup` with `controller::set_up`'s retries;
- `run` is `tick`;
- `probe`, `diagnose`, `calibrate` and `update` are the trait methods;
- `inspect`, `stats` and `calibration` are the snapshot's.

### 2.3 Long functions

- **`main`**: a `boot` module, one function per phase, each documented with what it needs and gives:
  1. `storage` (NVS, boot id, config, recorder);
  2. `ports` (the lent I2C ports);
  3. `layout` (scan, resolve, mask, clocks);
  4. `devices` (bring-up);
  5. `recovery` (warm start, calibrations);
  6. `threads` (owners, control, supervisor, sim-link, GNSS);
  7. `operator` (commands, executor, console, entry);
  8. `radio` (beacon, link).

  A `Boot` struct carries what passes between them. `main` reads as the list of phases, under 40 lines.
- **`control::run`**: its set-up (`Control::new`) apart from its tick; the tick as named duties (requests, IMU,
  separation, watches, filter, machine, flight, fins), each a method.
- **`files::serve`**: one handler function per route, registered from a table.
- **`register_commands`** (ota, sd, diagnostics, video): each command's body a named function; the registration
  lists the `Command`s only.
- **`recorder::write`**, **`link::run`**, **`owner::serve`**, **`devices::bring_up`**: by state or request kind.
- **A guard:** clippy's `too_many_lines` at 80 for the firmware crate (`clippy.toml`), with a short allow-list of
  measured exceptions, each saying why. Plus a test of the longest function in `make test`, so a new long function
  fails the build.

### 2.4 API docs (`tools/`)

- `doc/api.md` becomes `doc/glider_api.md`: plan B and the control hub, as `tools/gen_docs.py` writes them now.
  Its references move with it (`architecture.md`, `skills.md`, the generator's header).
- `tools/gen_avionics_docs.py` writes `doc/avionics_api.md` from the Rust sources, as `gen_docs.py` does from
  Python. It parses, never builds (no cargo, no toolchain):
  - one chapter per crate (`crates/*`, `firmware`, `recorder`, `sitl`), in dependency order, with the crate's
    `//!` text and the crates it uses;
  - per module, its `//!` text, then each public item (`pub fn`, `struct`, `enum`, `trait`, `const`, `static`,
    `type`) with its signature and its `///` summary.

  The test file that tests a module is named as Python's is.
- Both generators take `--check` (exit 1 when the file on disk is stale). `make test` runs both, so the docs never
  drift.

## 3. Stages

Each stage keeps the behaviour. Its proof:
- host: every crate's tests, clippy `-D warnings`, the firmware build, and the API docs' `--check`;
- taster: boot, the boot log's lines unchanged (timestamps and the build masked), and `tools/cc_contract.py
  taster --milestone M21 --allow-active` at 0 violations with every `inspect` answer identical to before;
- one clean HITL flight;
- for the parts stages, also a probe, calibrate and update round on the taster's SEN0253 and SEN0697.

Each stage is committed before the next.

| stage | what | risk |
|---|---|---|
| R0 | API docs: `glider_api.md`, `gen_avionics_docs.py`, `avionics_api.md`, both `--check` in `make test` | none (tools and docs) |
| R1 | `main` into the `boot` phases | low: a pure move, proven by the boot log |
| R2 | `Published<T>` and the `Inspectable` registry; parts' snapshots moved over one by one; the `device` match removed | medium: every inspect answer must stay identical |
| R3 | `I2cPart` with `enum_dispatch`, one file per part, `parts::KNOWN`; `Run` folded in; streams' columns from `KNOWN` | medium: the owner tick's timing (a soak checks the tick lateness and lapses) |
| R4 | the remaining long functions (`control::run`, `files::serve`, the `register_commands`, `recorder::write`, `link::run`, `owner::serve`) | low to medium |
| R5 | the guard: `too_many_lines` at 80 with the allow-list, and the length test in `make test` | none |

**Crates this needs:** `enum_dispatch` (R3, compile-time only). Everything else is already in the workspace:
`embedded-hal` (the part traits), `serde`/`serde_json` (the snapshots' JSON), and `crates/controller` (the registry
and the seqlock, host-tested there).

**Not in this plan:** behaviour changes. A finding the refactor turns up gets its own commit, as every finding
did in the milestones.
