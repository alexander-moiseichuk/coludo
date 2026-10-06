# Avionics port: src/glider (MicroPython) → src/avionics (Rust)

Status: accepted as the build plan, 2026-10-06; M1 in progress. Branch `lmoiseichuk/phase6_rustification`.
Plan B is frozen at tag `micropython_plan_B`, and 7C still flies it.

This document is the build plan for moving the flight firmware from `src/glider` to the Rust firmware in
`src/avionics`. Four things stay as they are on the wire and on disk, and the Rust board must match them:
- the CC protocol (`doc/specs/cc-protocol.md`);
- the recorder wire format (`doc/specs/recorder-wire.md`);
- the config JSON and board files (`launches/<date>/<BOARD>/*.config`, `doc/specs/board-config.md`);
- the telemetry schema (`doc/telemetry.md`).

`src/control` (hub, dashboard) and `tools/` (`tools/cc.py` among them) keep working against the Rust board
without changes. The new host code listed below (contract runner, pull tool, HITL host) is additive.

Decisions this plan builds on:
- **D5:** the flight loop is paced by a timer interrupt on core 1.
- **D6:** the code runs XIP from PSRAM, and the flash ring is kept.
- **D18:** f32 by default. f64 needs a clear written reason and only goes into absolutely critical spots
  (§7.2, Q8); never for plain values such as the MCU temperature.
- No mutex in hot paths: one owner per resource, plus atomics and task notify.
- Storage tiers: telemetry → PSRAM ring → flash ring (top priority) → SD mover (below video). On the SD, video
  is priority 1 and logging priority 2: the mover yields to the video writer.
- Video target: **FHD@25**. The operator accepts 25 fps, and FHD@30 is encoder-bound at about 26.6 fps on P4
  rev v1.3 (`doc/benches/avionics-20261005/README.md` §camera).

---

## 1. Target crate layout

The workspace rule (`src/avionics/Cargo.toml:3-7`) is unchanged:
- `crates/*` are portable crates. Stable Rust, host `cargo test`, no esp dependencies. Each is named after the
  `src/glider` module it ports. The rule stays as written (the conservative choice over restating it), and the
  exceptions are named after their concern: `golden` and `flash_ring` port no module, `board_config` ports
  `config.py`, `control` and `drivers` span many files, and `sim_link` is new.
- `firmware/` is the ESP-IDF binary (nightly, build-std).

### 1.1 Host-testable crates (`src/avionics/crates/`)

| crate | ports | one line |
|---|---|---|
| `cc_protocol` | `cc_protocol.py` | **exists.** Tokens, base64, build/parse. Add the dispatcher error-path vectors. |
| `golden` | — | **Being built in M1.** Dev-only. Loads a fixture JSON and applies its compare kind (`exact`/`abs`/`rel`/`angle_deg`/`bytes_hex`) with per-case messages. |
| `board_config` | `config.py`, `config_default.py` (as data) | `ConfigDocument(serde_json::Value)`, Python-repr canon, `config_id`, `validate` (the same accept set as Python), `load` → `BootSource`, a canonical pretty writer, and a typed read-only view. |
| `layout` | `layout.py` | `resolve(&mut Value, scan)` → `Resolved{Declared,Detected,Undecided}`, plus `verdict`/`apply`/`fit`/`rank`/`fitted`. The scan is injected as a closure. |
| `mission` | `mission.py` | **Being built in M1** (it came forward from the supervisor: the `cc` executor owns the Mission until a supervisor thread exists). The launch.config document, merge/persist (keeps `sites`), the HPRC default, zone geometry, `select_site`/`fallback_zone`, `probe` checks. Positions are i32 1e-7 deg plus an f32 local metric frame. |
| `recorder_wire` | `recorder.py` (wire parts), `tools/recorder_wire.py` | CRC-32, `wrap`/`verify`/salvage, row/log/session.csv text, label rules, flash frame codec, and recorderd's demux `split`. |
| `flash_ring` | — (new; replaces the UART + recorderd) | Ring geometry, boot-scan logic, GC rules, and the mover with its journal. Written over two traits, `Flash` and `Fs`, so it is host-tested with an in-memory flash and a temp directory. |
| `databoard` | `databoard.py` | `no_std`, core atomics. Seqlock `Channel<T>` and `Parameter<T>` (rank, window, `read() -> Option<Reading>`). There is no extrapolating accessor. Reconcile learning runs on the writer side. |
| `sequencer` | `controller.py` Stage, `tasks/sequencer.py`, `commons.dwell_step`/`apogee_step` | **exists** (`Stage` only, commit `83c02236`): `#[repr(u8)]` ids 1..5 (SETTING..DONE) with names and `ground()`; 0 is the NVS no-checkpoint sentinel, not a variant. The detectors and the pure `StageMachine::step(now_ms, &Readings, manual) -> Option<Transition>` join later (M7). |
| `warmstart` | `warmstart.py` | The crumb serde (the same NVS keys), `should_restore(...) -> (bool, Reason)`, and the apply plan as data. |
| `control` | `pid.py`, `mixer.py`, `governor.py`, `airspeed.py`, `wind.py`, `navigation.py`, `guidance.py`, `tasks/attitude.py` (filter), `commons` fin table and `altitude_m` | **Being built in M1** (the navigation subset first; the rest in M9/M13). f32 flight math with one module per Python file. Every function takes time as an argument; nothing reads a clock. |
| `gnss` | `gnss.py`, `drivers/neo6mv2.py`, `drivers/atgm336h.py` | NMEA checksum and parse with integer ddmm→e7, GSV/sky row, CASIC/PUBX/UBX command bytes, and a thin wrapper over the `ublox` crate (NAV-PVT) for the NEO-M8N. |
| `drivers` | `drivers/*.py` (device logic) | One module per part. Register init tables as data, conversions, the CRC-8, and `Device` trait impls over embedded-hal 1.0 `I2c`/`SpiDevice`. Host-tested with a scripted mock bus. |
| `sim_link` | `tasks/hitl.py` (interface only) | A versioned, allocation-free binary codec for SENSORS/FINS frames, shared by the firmware and `sitl`. |

### 1.2 Firmware modules (`src/avionics/firmware/src/`)

| module | role |
|---|---|
| `main.rs` | Bring-up in a fixed order (§4.5). |
| `board.rs`, `thermal.rs`, `sd.rs`, `video.rs`, `files.rs`, `diagnostics.rs`, `console.rs`, `link.rs` | **Exist.** `board.rs` loses its hard-coded `taster` (M2). `files.rs` gains `GET /ring` and an answer for locked files (M3/M4). |
| `commands/{mod,identity,control,inspection,config,diagnostics,streaming,system}.rs` | Replaces `commands.rs`. One table and one executor thread (§4.4). |
| `config.rs` | Config store on the `cfg` NVS partition. Boot load, then `Box::leak` into a `&'static` resolved document, typed view and cached `config_id`. |
| `controller.rs` | Sequential bring-up: `DeviceEntry` table, setup retries, the alternatives pass, the static databoard topology. |
| `inspector.rs` | The frozen `&'static [&'static dyn Inspectable]`, sorted by name. |
| `recorder/{ring,telemetry,writer,tee}.rs` | Producer SPSC rings, `Telemetry`, the flash-owner writer thread, the CC tees. |
| `recorder/mover.rs` | The SD mover thread. It glues `flash_ring::mover` to FatFs. |
| `buses/{i2c,spi,uart}.rs` | `I2cPort` for ports 0 and 1 (wedge recovery, general call, probe), SPI2 with discard-N, UART2. |
| `sensors/<part>.rs` | Owner-thread glue per part: schedule slot, channel publish, telemetry row, inspect/update/probe/calibrate. |
| `control.rs` | The core-1 control thread: gptimer ISR, tick, IMU read, attitude, sequencer, separation, flight, fins (LEDC). |
| `supervisor.rs` | Watchdog, health, checkpoint, gnss_calib, field, and the Mission owner (until a supervisor thread exists, the `cc` executor owns the Mission). |
| `warm.rs` | Warm-start load and apply, run before any thread spawns. |
| `sim_link.rs` | HITL UDP endpoint (only with an enabled `hitl` component). |

### 1.3 Host tools (additive)

| path | role |
|---|---|
| `src/avionics/golden/{generate.py,shims.py,*.json}` | Frozen oracle fixtures generated from `src/glider` under CPython 3.12. |
| `src/avionics/sitl/` | Host binary: `control` + `sequencer` behind a UDP `sim_link` endpoint. |
| `src/control/contract/{cases,transports,normalize}.py`, `tools/cc_contract.py`, `src/control/test/test_contract.py` | The contract runner (`BOARD=fake|taster|<id>`). |
| `tools/board_pull.sh`, `tools/ring_split.py` | Pull a session over HTTP; demux a raw ring dump. |
| `tools/hitl_host.py`, `doc/specs/sim-link.md` | HITL with the physics on the host (`sim_model.py` stays the one implementation). |
| `tools/provision.sh` | Builds a `cfg` NVS image (board config, launch config, Wi-Fi creds) for `flash.sh`, in a temporary directory; the image is never committed (§7, Q12). |

---

## 2. Port order: milestones

Rules for every milestone:
- Each one fits in one or two days on the taster.
- Each ends with a commit of the passed stage. A failed experiment gets an enable commit plus a revert commit
  carrying the reason.
- `src/glider` is never edited. The generator only reads it.

The taster is v1.1. Its scan in `test_layout.py:27` (`_TASTER`, derived from its 2026-09-16 verdict plus the
ES8311 anchor, not a recorded scan) is i2c:0 {0x15, 0x18, 0x40, 0x47, 0x69} and i2c:1 {0x25, 0x29, 0x63}. Its
VL53L1X (0x29 on i2c:1) has since moved to 7F. It carries a NEO-6M, three SG90s, the camera and a
32 GB SD card. A part the taster lacks is verified on 7F (or 7C) when that milestone reaches it.

### M1. CC surface parity: the hub and dashboard treat the Rust board like a MicroPython one (2 days)

**Scope**
- **`commands/` table.** Add `ErrCode` (badcmd, badargs, invalid, unsafe, refused, unsupported, internal) and
  `Gate` (`Open`, `Ground` = stage in {setting, done}). A parse failure answers `err badargs <repr>`, which
  replaces `badline` (`commands.rs:11`). An unknown command answers `err badcmd <cmd>`. A handler `Err`
  answers `err internal`. The gate refusal answers code `unsafe` with verbatim text: `err unsafe refused in
  flight -- stage is X (ground-only: setting/done)`. (`refused` is only the `update` driver-rejection code,
  even though the gate text starts with the word.)
- **One `cc` executor thread** (core 0, priority 5). Link and console post lines to it through a bounded
  channel (depth 4) with a reply slot. Every handler runs serially and needs no locks.
- **Clock.** At boot, if `gettimeofday` reads before 946684800, set it to 946684800 + uptime. A cold board then
  reports an epoch inside 2000, the same as MicroPython's RTC. That is necessary for the hub to sync, not
  sufficient: it syncs only when the whoami epoch is a JSON integer (not a float or bool) in [946684800,
  978307200), the whoami stage is `setting`, and CC's own clock reads 2020-01-01 or later
  (`server.py:39-42,322-331`).
- **`StageCell`** plus `armed`, `manual` and `warm_started` atomics. The stage stays `setting`. `stage <name>`
  holds the stage (manual=true), `stage setting` returns to the ground (manual=false), and `stage auto`
  resumes. The replies match `cc_client.py:525-545`.
- **`whoami`.** Reports mcu, firmware_version, config_id, config_source, the live stage, uptime, boot_id,
  session and epoch (a JSON integer, so the hub's sync check accepts it). Until M2, `config_id` comes from
  the embedded `golden/config_default.json`, the default config generated from plan B (expected
  `95e1af1d7b1d`), and `config_source` is `default`.
- **`health` with the full key set.** temp, mem_free, uptime, stage, layout and degraded, plus clock, epoch,
  launchpad, launchpad_set, site, `armed: false`, `tasks: []`, `position: null` and `agl: null`.
  - `layout` is `undecided`, which is true until M5.
  - `position` and `agl` are always present, as null while there is no fresh value: plan B emits them the same
    way, because health itself creates the databoard parameters.
  - `calibration` is omitted (nothing is outstanding, which is true).
  - `flight` is omitted until its parts exist. It, `calibration` and `imu_calibration` are the only
    conditional keys in plan B.
  - `degraded` gains `not-flight-capable` until M7, so the dashboard row reads NOT READY for a true reason.
  - Health never touches FatFs; the SD free space comes from the cached figure (`sd.rs:10-12`).
- **Mission owner** (`crates/mission` plus the `cc` executor, which owns the Mission until a supervisor
  thread exists; the Mission came forward from the supervisor):
  - launch.config lives in a new `cfg` NVS partition (§3.10).
  - `update mission {epoch, utc_offset, cc_position, source, launch fields}` replies `ok {changed:[…,'epoch']}`.
    The session.csv row is written from M3 on.
  - Also: `inspect mission`, `get-config launch`, `set-config launch` (ground-only), and the health launchpad
    fields.
- **System and streaming.**
  - `reboot` joins the shared table (ground-only). It replies `ok` and calls `esp_restart` 200 ms later. The
    console's own `reboot` uses the same handler, so its `ok rebooting` goes away.
  - `log [ms]` runs off a `log` crate bridge into a tee ring (temporary until M3's writer tee). `probe mission`
    logs its check through that bridge, as plan B does (`mission.py` `probe`: `probe: launch position ...`,
    then the verdict): the contract's stream tier opens a window across it and requires at least one line, the
    one case a stub `ok {lines:[], dropped:0}` cannot pass.
  - `tlm` replies `ok {samples:[],dropped:0}`.
- **Honest answers where the feature does not exist yet:**

  | command | answer until it is ported |
  |---|---|
  | `get-config board/running/saved/default` | the embedded default document |
  | `set-config board`, `reset-config` | `err unsupported 'config store -- M2'` |
  | `arm` | `err unsafe base64:{"flight":"flight -- not ported yet"}` |
  | `verify` | `ok {pass:false, devices:{}, problems:{flight:…}, alternatives:{}, ready:false, readiness:{flight:…}}` |
  | `probe`, `probe mission` | `ok {mission: null}` (or the mission's problem): plan B's answer over a registry holding only the mission |
  | `probe <other>` | `err badargs 'no probe for <other>'` |
  | `calibrate` | `ok {}` (nothing outstanding) |
  | `calibrate <device>` | `err badargs 'no calibrate for <device>'` |
  | `disarm` | `ok {armed:false}` |
  | `objects`, `inspect`, `stats`, `report` | over a registry holding `mission` |
  | `detect`, `bustune` | `err unsupported` until M5 |
  | `push-*` | `err badcmd` (§6) |

**Proof**
- Contract runner, every tier, with `BOARD=fake` and `BOARD=taster`: `tools/cc_contract.py taster --milestone M1
  --beats 30 --allow-launch-write --allow-stage --allow-reboot --allow-active` exits 0, so every case M1 owes
  passes (core, stream(log), config(launch), inspect(mission), control(stage), diagnostics(probe/verify/
  calibrate), hub, system); a not-yet answer to one of them is a violation. `--allow-active` is harmless on the
  M1 taster, where no fin is driven yet; it is not on plan B. `--diff fake` lists the known differences apart
  and shows only the documented not-yet answers.
  - The fake needs more than a bare dispatcher (§5.3): a `mission.Mission`, a stub Controller
    (stage/armed/manual) and `config.BOOT_SOURCE = 'default'`. The shims add `time.time()` on MicroPython's
    2000 epoch as an int and a `machine.RTC` stub, so `update mission {epoch}` reports `changed`.
  - `normalize` lists `firmware_version` and `config_source` as known differences. The fake resolves the layout
    over an empty host scan, as the golden `cc_dispatch` builder does, so `layout` reads `undecided` like the
    taster's (a bare dispatcher reports null).
- Hub level, by the runner's hub tier: the taster registers, its dashboard row fills, it stays online and fresh
  across 30 heartbeats, and its clock no longer reads 2000. By hand: the hub logs `clock set 2000-… -> 2026-…`
  at a cold registration, the board goes offline after 3 missed beats when it is powered off, and the
  dashboard's 'sync time' and 'sync GNSS' work. `cc.py taster ping` and `cc.py taster whoami` exit 0, and
  `cc.py taster verify` exits 1 (honest).
- Golden: dispatcher error paths (`golden/cc_dispatch.json`).

**Operator gains.** The dashboard row shows version, config id, stage, clock, launch pad, uptime and
temperature. NOT READY carries a true reason. The live log pane streams. Clock sync, pad setting and reboot work
from the dashboard. Every button gets a spec answer and nothing times out.

### M1b. esp_hosted on core 0 (a stage right after M1)

**Scope**
- lwIP's TCP/IP task is already pinned to core 0 (`CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0`, done in M1).
- esp_hosted creates its tasks with plain `xTaskCreate` (`port_esp_hosted_host_os.c` `hosted_thread_create`),
  so no sdkconfig option pins them. Wrap the `_h_thread_create` slot of `g_hosted_osi_funcs` with a
  core-0 `xTaskCreatePinnedToCore`, the same hook `components/hosted_no_reboot` already uses for
  `_h_restart_host`.
- The hook only works if it is installed before esp_hosted creates its tasks.

**Proof.** The console `load` command, which prints each task's core, shows every esp_hosted task and lwIP's
TCP/IP task on core 0.

**Operator gains.** Nothing visible: the radio no longer competes with the flight core (§4.2, §7.1 R5).

### M2. Board config and the config store (2 days)

**Scope**
- **`crates/board_config`:**
  - Python-repr canon.
  - `config_id` = sha256(canon(cfg minus `board.firmware_version`))[:12].
  - A Python float-repr formatter (`1e-05`, `1e+16`, `5.0`).
  - A canonical pretty writer (`indent=1, sort_keys` plus `\n`; byte-identical to the launches files).
  - A line-by-line `validate` port with the same messages.
  - `load` → `BootSource` (`active`, `active(config X, firmware Y -- re-save to adopt)`, `default`,
    `default(fallback: …)`), which restamps the firmware version.
  - The typed view uses config_default values as its serde defaults. Integer knobs accept integral floats.
- **`firmware/config.rs`:**
  - Boot load from `cfg` blob `board`.
  - `set-config board` validates, writes, and replies `{config_id of the payload}`, ground-only.
  - `reset-config` erases the key.
  - `get-config` follows the plan B meanings: `board`/`running` (the running document), `saved` (re-run
    load), `default`, `launch`.
  - `whoami.config_source` reports the BootSource. `degraded` adds `CONFIG FALLBACK -- …`.
- **`board.id` and `wifi`** come from the config: networks, `cc_host` (absent = dial `.1`, `''` = CC off) and
  `cc_port`. Passwords come from `cfg`/`creds` keyed `w<crc32(ssid) hex>`, because NVS keys are at most 15
  characters. The console `wifi` command stays as the fallback.
- **`tools/provision.sh <board-config> [launch.config] [ssid creds]`** builds a `cfg` NVS image with IDF
  `nvs_partition_gen.py`, so `flash.sh` can flash it (§7, Q12). The image is built in a temporary directory
  and never committed: it carries Wi-Fi credentials.

**Proof**
- Golden `config_id.json`. Test vectors: tms7c `e95f341166d4`, tms7d `7f06b2f496a9`, tms7d_control
  `04b9c8644717`, tms7e `2492734e035f`, tms7f `f80fcbb3231e`, default `95e1af1d7b1d`, `{}` `44136fa355b3`.
- Golden `config_validate.json`: the verdict and error list over the same corpus, plus `test_config`'s
  negative cases and `launch_config` outputs.
- Byte round trip: the canonical writer reproduces all 5 `launches/20261003/*/*.config` files.
- On the taster: `cc.py taster set-config board @<copy of launches/20261003/TMS-7F/tms7f.config with
  board.id 'taster'>`, then `reboot`; whoami then shows `38052780281d` (still pre-layout) and `active(config
  20260828, firmware 20261005 -- re-save to adopt)` (the firmware slot is the config schema version
  `CONFIG_VERSION`, not the build). The copy keeps the taster's id: pushing `tms7f.config` itself would make the
  taster register as `TMS-7F` and collide with the real, sealed 7F on the hub. This is the conservative one of
  the two fixes; the other is to keep 7F offline and address the taster as `TMS-7F`. The dashboard config
  editor's Load, Save, reset and reboot all work.

**Operator gains.** Config is edited from the dashboard and cc.py exactly as on MicroPython. The board id and
network come from the board file. A bad file boots the default and says so.

### M3. Recorder core and the flash-ring writer (2 days)

**Scope**
- **`crates/recorder_wire`:** wrap/verify, row/log/session.csv text, the frame codec, `split`.
- **`crates/flash_ring`:** scan, spill and open logic over the `Flash` trait.
- **Firmware recorder:**
  - stream registry, `Telemetry` (`due`/`push`), per-producer PSRAM SPSC rings (§3.2);
  - the writer thread, which formats, tees, wraps, cuts 50 ms frames and does one `esp_partition_write_raw`
    per frame;
  - BOOT frames, boot scan, spill, and a first-boot format of a foreign ring;
  - CC tees (replacing M1's temporary one);
  - session.csv `boot`, clock-set and `anchor` rows;
  - the `'session': '<s>'` status line every `stats_ms`;
  - `health.csv` at 1 Hz;
  - the `log` crate bridge plus IDF `esp_log_set_vprintf` into recorder.log;
  - `GET /ring[?boot=N]` on the file server.
- The Recorder comes up **first** in bring-up, so setup-time log lines are not lost (`recorder.py:21-23`). The
  old global-decimation latch bug is fixed separately, by reading the global decimation at use (§3.2), as plan
  B fixed it.

**Proof**
- Golden `recorder_wire.json`: the 8 literal vectors (`test_recorder.py:23-44`), damage cases, row/log/index
  text.
- Host: a Rust-built ring image passes `ring_split.py` and then `flight_telemetry` strict, with every line
  'good'.
- Bench gate G1: 10 000 frames of 40-200 B appended at 8-byte alignment, read back intact. This is partial-page
  programming on the GD `C84019` (§7, R1).
- On the taster: the dashboard 'telemetry' stream shows health rows. A `/ring` dump parses strict.

**Operator gains.** Live telemetry on the dashboard. From boot, every log line and row is on flash with about
70 ms typical and at most about 115 ms at risk (135 ms for log lines, §3.3). No Luckfox and no SD card are
needed to get a record.

### M4. SD mover, GC, journal, HTTP pull (2 days)

**Scope**
- `flash_ring::mover`: cursor, demux, salvage, journal, idempotent batches.
- The firmware mover thread at priority 4.
- CONFIRM frames.
- The 4 KiB reverse-order GC.
- `ring_erased_s`, `ring_reclaim_s` and the `ring-*` degraded tokens.
- Recorder `stats`/`inspect` grows `ring:{…}` and `mover:{…}`.
- `sd_card.c` `max_files` goes from 8 to 40.
- `CONFIG_FATFS_FS_LOCK` goes from 0 (the IDF default) to at least `max_files` (40). Below that, a full lock
  table answers `FR_TOO_MANY_OPEN_FILES` (ENFILE).
- `files.rs` answers `503` on a locked file. Through `std::fs` and the VFS that arrives as errno EACCES (os
  error 13), not `FR_LOCKED`. Without the lock table, an HTTP read during a mover batch simply succeeds with
  the size recorded before the batch.
- `tools/board_pull.sh <host> [session] [outdir]` verifies file count and byte sizes and flags 0-byte streams.

**Proof**
- Host end to end: Rust ring, then mover into a temp session dir, then `recorder_flight` (0 rejected,
  board.log non-empty, session.csv dates the boot) and `assemble_capture` plus `flight_telemetry.parse` (all
  good). This includes a mover crash at every journal step (no duplicated rows or header) and a session
  longer than 2³⁰ µs whose `anchor` row is dated.
  - The long-session case is limited to a dated anchor (the normal case: the hub syncs on connect, usually
    inside the 60 s before the anchor row). `recorder_flight._clock_times` still enumerates 2³⁰ wraps for
    every session.csv row, so in a boot longer than 2³⁰ µs a clock-set row that is the only dated row gets
    two places and the boot stays undated, as it already does for plan B. Restricting the test is the
    conservative one of the two fixes; the other is a tool change (no wraps seen in the streams means k = 0).
- On the taster: 5 power pulls while writing at the flight rate with FHD@25 recording. Each must show loss of
  at most §3.3's worst case (about 115 ms of rows, 135 ms of log lines), a clean salvage, and no spliced
  streams. Tightening this to 100 ms needs Q11's crumb-frame alternative.

**Operator gains.** `tools/board_pull.sh taster` drops a session into the unchanged tool chain
(`assemble_capture`, `flight_svg`, `flight_report`, `flight_kpi`, `recorder_flight`).

### M5. Buses, layout, databoard, device registry (2 days)

**Scope**
- **`I2cPort`** for ports 0 (SDA7/SCL8) and 1 (SDA31/SCL30). Each implements `embedded_hal::i2c::I2c` over
  `i2c_master` with per-device speed. Wedge = 4 consecutive failures, cured by `i2c_master_bus_reset` and
  re-adding the handles. Also `general_call_reset()` and `probe(addr)`.
- **SPI2** on 48/47/46, mode 3, one handle per CS. Discard-N on first use.
- **`crates/layout`** with `resolve`. `health.layout` becomes real. `detect` and `bustune` work through
  bus-owner mailboxes.
- **`crates/databoard`.**
- **`controller.rs`:** the `DeviceEntry` table (Up/Failed/Unfitted/Crashed), setup retries
  (`board.setup_retries`, 200 ms apart, diagnose on the last attempt), and the alternatives pass with all four
  rules including XSHUT.
- **`config_id`** is now computed **post-layout**. It differs from the pre-layout id `set-config` returns
  (`config.py:643-661`, `save()` hashes the saved snapshot). That split is plan B behaviour, not a documented
  plan B decision: `main.bringup()` runs `layout.resolve(cfg)` in place (`main.py:91-102`) and whoami hashes
  that mutated config (`cc_link.py:48`, `cc_client.py:398`). Keeping it is the port's decision (§7.2, Q4).

**Proof**
- Golden `layout.json`: the `_TMS7C`, `_TASTER` (derived, not recorded) and `_LEGACY` scans give the
  expected verdicts and edits. `tms7c.config` as-is (no `board.layout` key, which resolves as v0.1 and logs
  '(declared)') gives `f6a9c35d6372`; the generator must not add the key (an explicit `'layout': 'v0.1'`
  hashes to `92d537918c7c`). tms7f by scan on v1.1 with no SEN0253 gives `d18751e70f6c`.
- Host databoard tests: rank handover, born stale, window equals the tightest timeout of the lowest-rank tier,
  `fresh_below`.
- On the taster: the scan set equals `_TASTER` minus `0x29` (the VL53L1X, now on 7F) plus `0x36` (OV5647
  SCCB, while the camera is powered): i2c:0 {0x15, 0x18, 0x36, 0x40, 0x47, 0x69} and i2c:1 {0x25, 0x63}.
  `detect` gives `v1.1`. A
  `bustune taster i2c 0` ladder restores the frequency afterwards.

**Operator gains.** The detect-layout and bus-tune cards work. The dashboard layout column is real.

### M6. Control thread, primary IMU, barometer, separation (2 days)

**Scope**
- **`control.rs`:** a gptimer alarm ISR on core 1 at `flight.schedule_hz` (100) notifies the control thread
  (priority 20) and the bus owners. The tick skeleton is in §4.3.
- **LSM6DSO32 on SPI** (CS50, INT1 GPIO28):
  - write-then-verify bring-up (up to 64 rounds);
  - ±32 g and ±2000 dps;
  - ODR 104 Hz for accel and gyro with BDU, as plan B (`CTRL1_XL=0x44`, `CTRL2_G=0x4C`, `CTRL3_C=0x44`),
    read at the top of each tick;
  - `irq_runs`.
  If the taster's LSM6DSO32 is not fitted, the BMI323 (M9) is pulled forward and the LSM6DSO32 is verified
  on 7F.
- **ICP-10111** on whichever bus owner `layout.resolve()` assigns: i2c:1 on v1.x (taster, 7F), i2c:0 on v0.1
  (7C, next to the SEN0697). A two-phase measurement, the general call, and **re-arm by bus membership** (§7,
  R6).
- **Separation:** GPIO33 with pull-down. The ISR only wakes the control thread, which runs the 3-reads / 20 ms
  debounce.
- Streams `imu_lsm6dso32.csv`, `baro_icp10111.csv` and `separation.csv`.
- `inspect`, `update{rezero,ground}`, `probe`, `diagnose` and `calibrate` through owner mailboxes.
- `health.calibration` becomes real. `degraded` gains `needs-calibration`.

**Proof**
- Golden `drivers_icp10111.json`: `_compensate(11441968, 27034)` with OTP [211, 371, 553, 3833], pinned exactly
  from plan B under f32 and f64. This decides the f64 question for the polynomial (§7, Q8).
- Golden `drivers_lsm6dso32.json` (scaling, frame decode).
- On the taster: `pace` under FHD@25 plus a real I2C/SPI load shows 0 missed ticks and max ≤ 200 µs.
- Still-board noise matches the plan B `measure_noise` provenance numbers within their spread.

**Operator gains.** Real attitude-grade IMU and baro data in the telemetry pane. Probe all and calibrate work
for these parts.

### M7. Stage machine and a real arm/verify gate (2 days)

**Scope**
- **`crates/sequencer`** (the stage machine joins the existing `Stage`) in the control thread at 20 Hz:
  launch, apogee and burnout, land, still, and the flight-timeout backstop, all gated by `read().source`.
- The `StageCell` single writer is the control thread. Hold/resume/arm/disarm arrive as mailbox requests.
- `sequencer.csv`.
- **Arm and verify:**
  - `arm` refuses on failures, a stage problem, or any failing `probe_all` on the ground; it freezes the
    launch point.
  - `verify` gets the readiness gate. The Rust version uses `.get('enabled', true)`, the Controller's meaning
    (§6).
  - `probe` returns the two-map reply.
  - `not-flight-capable` is removed from `degraded`.
- **Ring gate:** `arm` and `verify` also require `ring_erased_s ≥ 300` (§3.7).
- **Wi-Fi policy** `auto`: the radio is **off** from the **detected** ignition (the stage machine's own
  SETTING→BOOSTING transition) through LANDING and resumed at DONE for recovery. This is a deliberate divergence
  from plan B, which keeps the pad link up in flight (§6). An operator hold (`stage boosting` from CC, manual)
  leaves the radio up: keying on a held stage would cut the very link the operator holds it from, and the
  console would be the only way back.
- **Video by stage:** recording starts when the glider stands **vertical on the rod** (attitude-based, from
  M9; until then arm is the trigger) at 1920×1080@25, 8 Mbit/s, and stops 30 s after DONE. The console default
  fps goes from 20 to 25 (`console.rs` `video`).
- **Pad sequence** (the operator's; the gates fit it): (1) assemble glider and booster, plug the battery,
  close the cover; (2) CC check, time sync and probe; (3) if there is time, wait 5 min for GNSS; (4) place on
  the rod; (5) tie to the ignition system; (6) launch.
  - The board boots at (1). The ring pre-erase runs in the background from then on, and the hub's clock sync
    on connect is normally done before (2).
  - (2) is `verify`, `probe` and the ring gate, all on the ground with the pad link up.
  - `arm` freezes the launch point, so it belongs after (3).
  - Video starts at (4) on the attitude trigger (M9); until then it starts at arm.
  - The radio stays up through (5) and goes off at BOOSTING, that is at (6).

**Proof**
- Golden `sequencer_trace.json`: plan B's `Sequencer._tick` driven with scripted `(value, source)` reads. It
  includes stale agl at apogee, GNSS elevation on the pad, a baro dropout, and the warm-entry rebase. Stage
  and reason must match EXACTLY.
- Golden `commons.json` (`dwell_step`, `apogee_step`).
- Contract tier `control`: arm, verify and probe shapes; `cc.py verify` exit codes 0/1/3.
- On the taster: a hand-shaken launch (accel) goes BOOSTING and the radio goes off; `stage setting` on the
  console brings it back. A `stage gliding` hold from CC keeps the link up.

**Operator gains.** The pre-flight gate is real: verify, arm, and readiness from the dashboard. A dummy flight
(drop or throw test) produces staged data and video.

### M8. Warm start, checkpoint, supervisor watchdog (1-2 days)

**Scope**
- `crates/warmstart` plus `warm.rs`. Restore runs **before** any thread spawns: stage, armed, ground for all
  baros, pitot tare, governor seed, checkpoint identity, sequencer timeout base.
- The checkpoint writes the NVS crumb only when armed, `stage` i32 last, then `checkpoint.csv`.
- The supervisor's watchdog feeds the TWDT. With the current sdkconfig a TWDT timeout only prints a warning,
  so the supervisor first calls `esp_task_wdt_reconfigure` with `trigger_panic = true` and the timeout from
  `wdt_timeout_ms`, as plan B's `machine.WDT` does (the conservative one of the two fixes; the other is
  `CONFIG_ESP_TASK_WDT_PANIC=y`). If the control thread stalls past `stall_ms` while controlling, it writes an
  event and calls `esp_restart`.
- The P4 reset-reason mapping (§4.6).
- `degraded` gains `WARM-STARTED (rebooted in flight)`.

**Proof**
- Golden `warmstart.json`: `should_restore` over every crumb, cause and age boundary, plus a torn crumb.
- On the taster: arm, pass through BOOSTING (M7's hand-shaken launch), then hold `stage gliding` with the
  separation latch reading separated; console `panic` then returns warm into GLIDING with the same pad, and
  `gettimeofday` has survived. (Plan B writes the crumb only while armed and freezes the pad only on entry to
  BOOSTING, so a disarmed hold straight from SETTING reboots cold.) The survival of `gettimeofday` across
  SW/panic/TWDT resets is itself measured here (§7, R7).

**Operator gains.** A mid-air reset resumes the flight. The board can no longer hang silently.

### M9. SEN0697: BMI323, BMP581, BMM350, and the attitude filter (2 days)

**Scope**
- **BMI323** (2 dummy bytes per read, start-up marker wait, INT1 on GPIO4 on v1.x). Accel and rate at p1.
- **BMP581** (re-arm out of standby). Backup baro at p1, reconciled against the ICP.
- **BMM350** (OTP power-off is mandatory, NVS `bmm350_cal` 16 B blob, octant coverage).
- **Attitude complementary filter** (`control::attitude`) at 50 Hz in the control thread. It publishes
  attitude at p1, writes `attitude.csv` with centidegree integer columns, and drives `attitude-blind` and
  `attitude-backup`.
- **Video start** moves from arm to the attitude trigger: the glider standing vertical on the rod (M7).

**Proof**
- Golden `drivers_bmi323.json`, `drivers_bmp581.json` and `drivers_bmm350.json` (decode, coverage,
  calibrate refusal).
- Golden `attitude.json`: trajectory tolerance 0.1 deg after N steps; flags `free`, `mag_known` and `blind`
  EXACT.
- On the taster: a move-and-hold axis check, LSM6DSO32 vs BMI323 in the same frame (§7, Q7). The mag
  calibration figure is saved and survives a reflash to plan B and back (shared NVS key).

**Operator gains.** Primary attitude with redundancy on every scanned board. The mag-calibration flow works from
the dashboard 'calibrate' button.

### M10. Pitot, power monitor, laser (2 days)

**Scope**
- **SDP810:** stop before start, all 3 CRCs, scale latch, the NVS `sdp810_zero` i32 that wins over the config,
  the restart ladder.
- **INA226:** separate 2-byte reads; the re-arm restores CONFIG, CALIB, ALERT_LIM and MASK.
- **VL53L1X/VL53L4CX:** one socket, told apart by model id. Data-ready is checked before every publish.
- Telemetry columns keep their integer units: dynamic_pressure Pa ×100, airspeed_cms, temperature ×100,
  mV/mA/mW.

**Proof**
- Golden `drivers_sdp810.json`: CRC 0xBEEF→0x92, `_pressure(8100)=13500`, tare 200→333, the airspeed band.
- Golden `drivers_ina226.json` (calibration math) and `drivers_vl53.json` (timing-budget macro period).
- On the taster: blow test on the pitot (P+ is the barb opposite the '1' mark). INA226 rail check during a
  single-fin probe. Laser on 7F.

**Operator gains.** Airspeed and power in health and telemetry. The servo probe's closed-loop verdict works.

### M11. GNSS, gnss_calib, field: passive flight ready (2 days)

**Scope**
- **`crates/gnss` on uart:2** (TX22/RX23):
  - the NMEA path (ATGM336H, NEO-6M, with PCAS/PUBX/CFG-NAV5 dynModel 8);
  - pad-only sky window;
  - `position` as i32 e7 into the mission's local metric frame;
  - the three legacy CSV streams, with lat/lon text formatted from the integers.
- **gnss_calib:** drift averaging in SETTING, reset at the first armed tick, frozen at BOOSTING.
- **field:** CC-less site selection and auto-arm with the accel gate.
- Health gains `position`, and the `flight` block is minimal (attitude, airspeed).
- The UBX NAV-PVT path follows when the NEO-M8N arrives (M16).

**Proof**
- Golden `gnss.json`: `test_gnss.py`'s captured bursts, never `doc/benches/gnss-20261004/raw`. The CASIC and
  UBX command bytes must match EXACTLY.
- Golden `gnss_calib.json` and `field.json`.
- On the taster: `gps taster` on the hub console. A NEO-6M fix with an active antenna.

**Operator gains.** **Passive-flight ready.** This covers the flight-test ladder's second rung (passive
electronics E16/F15, no active fins): stages, data, video, warm start, pull.

### M12. HITL over a UDP sim-link, realtime and passive (2 days)

**Scope**
- `doc/specs/sim-link.md`, `crates/sim_link`, `firmware/sim_link.rs`. It runs only with an enabled `hitl`
  component. It masks every sensor whose config **name** is in `_SIM_SENSORS` (`enabled = False`, as plan B's
  `config_hitl.mask`, again after layout), refuses to start if one is still enabled (new: plan B only masks),
  and publishes p0 snapshots with the timestamp of arrival.
- `tools/hitl_host.py`: `sim_model.Body`, `config_hitl` scenarios and `Faults`. A realtime accumulator mirrors
  `hitl.py:323-391`. It drives the board over CC (set-config board, reboot, arm) and prints the
  SESSION/STAGE/DONE lines that `hitl_collect.sh` parses.
- The real mid-glide reboot (console `panic`) replaces the simulated one.

**Proof**
- A passive scenario flies to DONE with the warm gate passing.
- Artefact counts are checked: streams pulled = streams expected, no empty stream, BUILD stamp present.

**Operator gains.** Bench flights of the Rust board without hardware motion, with the same harness output as
before.

### M13. Active control (2 days, possibly split: crate one day, integration one day)

**Scope**
- `crates/control`: pid, mixer, governor, airspeed, wind, guidance, and the rest of navigation (its subset
  came in M1), with the golden fixtures.
- Servos on LEDC at 50 Hz with ≥14-bit duty. Pins are held LOW until configured, and boot centring is
  staggered by `fins.concurrency`.
- `set_angle` compare-and-set, the fin probe sweep as a control-thread state machine (ground, disarmed only),
  and `update <servo name> {trim|angle}`.
- `flight.csv` at 10 Hz with plan B units.
- `health.flight` vitals complete: reach, r_min_m, fins, wind, heading_error.

**Proof**
- Golden `pid.json`, `mixer.json`, `fin_limit.json`, `governor.json`, `airspeed.json`, `wind.json`,
  `navigation.json` and `guidance.json`.
- Golden `control_trace_<scenario>.json`: virtual_flight (`VF_SEED`) per-tick inputs replayed open-loop. Stage
  transitions EXACT, fins within 1 deg.
- On the taster: the dashboard fin-zero card (reads `props.trim`, nudges, saves).

**Operator gains.** Active control in HITL. The fin-zero calibration UI works unchanged.

### M14. HITL acceptance and SITL (2 days)

**Scope**
- Fly the 12 `hitl_matrix.sh` scenarios × {E16, F15} on the taster (realtime).
- Lockstep mode with an injected `Clock`.
- `src/avionics/sitl` on the host behind the same sim-link.

**Proof.** `hitl_compare.py` against `doc/sims/TMS-7-postaudit_regression` r1..r3. Each scenario and panel must
stay inside its family's own run-to-run spread (wind median miss 7.7 m, low-noise 11.9 m; the high-noise and
corner families are not gated on miss). All silent-success guards are kept.

**Operator gains.** Evidence that the Rust board flies like plan B, which is the precondition to migrate 7F.

### Later milestones (not on the critical path)

| id | scope |
|---|---|
| M15 | Firmware OTA (esp_ota over HTTP POST to the file server, gated ground and disarmed). Replaces `push-*` for the Rust image (§7, Q5). |
| M16 | NEO-M8N UBX NAV-PVT (CFG-PRT baud-up, autobaud after a P4 reboot). |
| M17 | Post-landing recovery beacon: an AP `coludo_<BOARD>_beacon` carrying the last fix (operator feature). |
| M18 | Legacy parts as needed: ADXL375 (7C), BNO055, BMP280. |

---

## 3. Storage design

### 3.1 Data path

```
producers (any core)                          core 0, prio 7                core 0, prio 4
 control / i2c0 / i2c1 / gnss / supervisor     ┌──────────────┐  frames     ┌──────────┐  files
 / executor  ── binary records ──► PSRAM SPSC ─►│ flash writer │──────────►  │  mover   │──────────► /sdcard/recordings/
 log bridge  ── (one ring per thread)           │ (owner of    │  24 MiB     │ (journal)│
                                                │  `log` part.)│  `log` ring └──────────┘
                                                └──┬────────┬──┘     ▲  cursor (2×AtomicU32 pairs)
                                       CC tees ◄───┘        └── GC ◄─┘
video: encoder (prio 10) → 2 MB PSRAM ring → SD writer (prio 6) → /sdcard/video/<session>/   (independent)
```

Invariants:
1. **Exactly one owner per resource.** One writer per SPSC ring end, one owner of the `log` partition (the
   writer), one owner of each SD recording file (the mover).
2. **A record is always in PSRAM, in flash, or in both.** It leaves PSRAM only after its program call returns.
   This is the operator's rule: "better to have duplicates in both than lost data".
3. **GC never erases past the journal-committed cursor.**
4. **No mutex on any producer path.**

The whole 24 MiB flash ring serves logging. A background GC erases blocks once they are confirmed on the SD.
The pre-erase runs in the background, so boot is never blocked, and in the worst case PSRAM holds the data
while blocks erase (§3.8).

### 3.2 Producer side: `Telemetry` and the PSRAM rings

**Stream registry.** The registry is built during bring-up, before any thread spawns, and is then immutable.
Each entry holds:
- `id: u16`;
- routing (`<session>_<file>`, or the bare `session.csv`);
- the header field list;
- `decimate_us`. A stream's own value wins when it is non-zero; 0 means use the global
  `recorder.telemetry_ms*1000`, read at use; a negative value means never decimate (`<task>_events.csv`, -1).

**`Telemetry`** is owned by its producer:

```rust
pub struct Telemetry { stream: u16, decimate_us: i32, last_us: u64, header_sent: bool }
fn due(&self, now_us: u64) -> bool                        // false while the recorder is not running
fn push(&mut self, now_us: u64, row: &[Value]) -> Result<(), Overflow>  // telemetry raises (policy)
```

- The first push emits a HEADER record before the row, and always emits, whatever the decimation.
- `Recorder::log(desc, msg) -> bool` is best effort and drops with a counter.

**One SPSC byte ring per producing thread**, in PSRAM (`heap_caps` SPIRAM):
- the producer owns `head`, the writer owns `tail`, both `AtomicUsize` with Release/Acquire;
- sizes: control 1 MiB, i2c0 1 MiB, i2c1 512 KiB, gnss 256 KiB, supervisor 256 KiB, executor 128 KiB, plus one
  `misc` ring of 256 KiB;
- `misc` is guarded by a std `Mutex` with `try_lock` (drop and count on contention). It is used only by
  unregistered threads and IDF's vprintf hook, never on a hot path.

The config keys `recorder.tlm_capacity`, `log_capacity` and `cell_size` stay accepted and act as lower bounds.

**Ring record format** (little-endian, 4-byte aligned):

| off | size | field |
|---|---|---|
| 0 | u16 | `len` = total record bytes including this header, a multiple of 4. `0` = wrap marker (the consumer jumps to offset 0). |
| 2 | u8 | `kind`: 1 HEADER, 2 ROW, 3 LOG, 4 LINE (a preformatted payload, e.g. a session.csv row) |
| 3 | u8 | `flags`: bit0 durable (cut a frame right after it) |
| 4 | u16 | `stream` (HEADER/ROW/LINE) or descriptor id (LOG; 0 = inline text) |
| 6 | u8 | `count` = number of cells (ROW) |
| 7 | u8 | reserved 0 |
| 8 | u64 | `uptime_us` (esp_timer, unwrapped) |
| 16 | … | ROW cells, each `tag u8` plus payload: 0 None (prints `None`), 1 I32, 2 I64, 3 F32 (prints with Rust `Display`: shortest round-trip, never an exponent; `NaN`/`inf` allowed), 4 Bool (prints `0`/`1`), 5 E7 (i32 printed as `d.ddddddd` degrees), 6 Text (`u8 len` plus bytes; never `;`, `\n` or control bytes). LOG: `u8 desc_len`, desc, message (≤ 1 KiB, CR/LF replaced by spaces). LINE: raw payload bytes. |

Producers never format text. That keeps `core::fmt` (53 µs per 20-field float row) off core 1.

Durable streams: `sequencer`, `separation`, `checkpoint`, `<task>_events`, session.csv rows.

### 3.3 The flash writer (flash owner)

**Placement.** One thread on core 0 at priority 7: above the video SD writer (6), below the encoder (10).
- IDF flash programming busy-waits: L1 measured core 0 at 98.5 % when writing flat out.
- At the real rate (~35 KB/s, about 1.75 KB per 50 ms frame, about 3.5 ms per program) that is about 7 % of
  core 0 for programming. In flight, GC erases are paused (§3.7).
- L1 measured the opposite priority order (writer at 5) with 0 dropped frames. At 7 the writer can delay the
  video writer by at most one frame program plus one 4 KiB erase (≤ 50 ms). The 2 MB video ring absorbs about
  2 s. R3 confirms this (§7, R2).

**Loop:**
1. Drain the rings: telemetry rings first, then log records merged by `uptime_us` across rings. A log record
   is emitted only once it is older than a 20 ms merge horizon, so recorder.log stays in uptime order.
2. Format each record into a line:
   - routed: `<uptime>;v1;…;vn` with routing `<session>_<file>`;
   - log: `<uptime_us> <desc> :: <msg>`;
   - the status line every `stats_ms`: `<us> Recorder :: {'session': '<s>', 'tlm': {'count': n, 'max': n,
     'dropped': n}, 'log': {…}, 'ring': {…}}`, in Python dict repr with single quotes.
3. Tee the unwrapped line into the matching CC tee if its deadline is in the future (§3.12).
4. Wrap it: `[@<routing>@]{OPEN};<payload>;<CLOSE>\n` (`recorder-wire.md:25-50`; `{%08x};%s;<%08x>`,
   `tools/recorder_wire.py:75`). Append it to the open frame buffer (16 KiB, internal RAM).
5. **Cut** the frame when any of these holds:
   - the oldest pending record is 50 ms old;
   - the buffer reaches 16 KiB;
   - a durable record was appended;
   - the frame would cross the block end.
6. **Program** header and payload with **one** `esp_partition_write_raw` at the 8-byte-aligned offset. The
   header comes first in address order.
7. Publish `programmed = (block_seq, offset)` (two AtomicU32 written under a seqlock counter) for the mover.
8. GC step (§3.7).

**Power-cut bound:** 50 ms (cut age) + ≤ 17 ms (an in-progress 4 KiB erase) + the frame program (≈ 4 ms
typical, ≤ 32 ms for 16 KiB) + ≤ 16 ms (an NVS page erase holding the flash lock). That is about 70 ms
typical and ≤ ~115 ms worst case. Log records wait up to 20 ms more for the merge horizon (step 1), so a log
line's worst case is ~135 ms. Moving checkpoint NVS writes to frames only (§7, Q11) would bring the row bound
back to ~100 ms.

### 3.4 Flash frame format (`recorder_wire::frame`)

The frame is a 40-byte little-endian header at an 8-aligned offset, followed by the payload. The next frame
starts at `align8(offset + 40 + payload_len)`.

| off | type | field |
|---|---|---|
| 0 | [u8;4] | magic `F7 43 52 31` (`"\xF7CR1"`; 0xF7 never occurs in UTF-8, and the magic is neither all-0 nor all-1) |
| 4 | u8 | version = 1 |
| 5 | u8 | kind: 1 DATA, 2 BOOT, 3 CONFIRM |
| 6 | u8 | flags: bit0 spilled (waited in PSRAM for an erased block), bit1 durable cut |
| 7 | u8 | reserved 0 |
| 8 | u32 | payload_len |
| 12 | u32 | block_seq |
| 16 | u32 | boot_id (0 = uncounted) |
| 20 | u32 | frame_seq (per boot, from 0, +1 for every kind; a gap means frames were lost) |
| 24 | u64 | uptime_us of the first record |
| 32 | u32 | payload_crc (zlib CRC-32; `esp_rom_crc32_le` matches) |
| 36 | u32 | header_crc over bytes 0..36 |

Payloads:
- **DATA:** one or more complete wire lines. A line never splits across frames, and each line proves itself
  after a torn frame. One boot's DATA payloads, concatenated, have the old UART stream's format and make a
  valid `flight_telemetry` capture (the numbers are not byte-identical to MicroPython's: f32 `Display`, u64
  uptime). A raw ring dump is not one: it carries the binary frame headers, BOOT payloads, 0xFF tails and
  many boots, so it goes through `ring_split.py` first (as in M3).
- **BOOT:** `boot=<id>;session=<prefix>;board=<id>;firmware=<v>;config_id=<id>;reset=<cause>;card=<vsn|none>`.
- **CONFIRM** (24 B): moved `block_seq u32`, moved `offset u32`, `boot_id u32`, `frame_seq u32`,
  `journal_seq u32`, `card_vsn u32` (FAT volume serial).

### 3.5 Ring geometry and block life cycle

- **Partition:** `log`, found by label: 0x800000–0x2000000, 24 MiB (`partitions.csv:8`). It is accessed only
  through `esp_partition_read`/`write_raw`/`erase_range`, and never mmap while writes run.
- **Blocks:** 384 × 64 KiB, each 16 × 4 KiB sectors.
- **Sequence:** a global `block_seq: u32` increases on every block open, across boots. Block `k` lives in slot
  `k % 384`, a strict FIFO.
- **Block layout:** every block starts at offset 0 with a BOOT frame. Frames never cross the block end, and the
  tail stays erased (0xFF).
- **States:**

  | state | meaning |
  |---|---|
  | `ERASED` | all 0xFF |
  | `OPEN` | the writer's current block |
  | `CLOSED` | valid, not moved |
  | `MOVED` | `block_seq < cursor.block_seq` |
  | erased again | by GC |

- **A new boot never appends to a previous boot's block.** It opens slot `(max_block_seq + 1) % 384` with
  `block_seq = max + 1`.
- **Before the first write into a block,** the writer verifies it is fully erased (one 64 KiB read, ~6 ms). If
  it is not, the writer spills (§3.8).
- **Crash consistency of erase:** GC erases a block's sectors **in reverse order (15 → 0)**, so the header
  sector goes last.
  - A block whose offset-0 header reads all 0xFF is therefore fully erased.
  - A block whose header is still valid but whose tail is partly erased was already moved, and is simply
    erased again.
- **Foreign blocks:** a block whose offset-0 header is neither valid nor all 0xFF holds at most a torn BOOT
  frame, because blocks are written in address order with BOOT first. It carries no recoverable data and is
  free to erase. This covers MicroPython-era VFS data and the `flashwrite` bench patterns.

### 3.6 Boot scan and recovery

Before the writer opens its first block, with a 50 ms target and without blocking the rest of boot:
1. Read the 40-byte header at each of the 384 block starts (~8 ms). Classify each slot as valid
   `(block_seq, boot_id)`, `ERASED` or `FOREIGN`.
2. **No valid block anywhere** means an unformatted ring:
   - `degraded: ring-unformatted`;
   - GC formats in 4 KiB reverse-order steps ahead of the writer (~100 s for 24 MiB);
   - the writer starts as soon as slot 0 is erased (~0.3 s) and spills until then.
3. Otherwise the writer opens the slot after the max `block_seq`.
4. **Cursors** (computed in the background):
   - **GC cursor** = the max of the SD journal's last COMMIT (when the card is mounted) and the newest
     CONFIRM frame. The newest CONFIRM is found by walking the newest blocks backwards, at most 16 blocks;
     otherwise the oldest valid block is used, which is conservative (no erase until the mover catches up).
   - **Mover copy cursor:**
     - a valid journal on this card gives its COMMIT;
     - with no journal, a card whose VSN equals the newest CONFIRM's `card_vsn` gives that CONFIRM;
     - any other card starts at the oldest valid block, so the ring is re-copied (§3.9, duplicate rule).

### 3.7 GC and erase policy

- GC is owned by the writer thread (the single eraser). After each frame program, if allowed, it erases
  **one 4 KiB sector** (13-17 ms) of the oldest `MOVED` block lying ahead of the writer, in reverse sector
  order.
- A 64 KiB erase (95-107 ms) is never issued while the writer is live.
- **A block is erasable only if `block_seq < gc_cursor.block_seq`.** Unconfirmed data is never erased.
- **Flight pause:** no erases from BOOSTING through LANDING, unless fewer than 8 blocks (512 KiB, ~15 s) are
  erased ahead.
- **Pad pre-erase:** the same GC runs continuously on the ground, ahead of the writer through moved blocks.
- **Budget figures:**

  | figure | meaning |
  |---|---|
  | `ring_erased_s` | erased blocks ahead × 64 KiB / measured write rate |
  | `ring_reclaim_s` | moved but not yet erased blocks, by the same measure |
  | `arm` / `verify` gate | `ring_erased_s ≥ 300` (§7, Q10) |

- **Degraded tokens:**

  | token | when |
  |---|---|
  | `ring-low` | `ring_erased_s` < 300 |
  | `ring-full` | the writer is spilling |
  | `ring-dirty` | salvage pending |
  | `mover-behind` | SD mounted and lag growing for more than 60 s |
  | `ring-unformatted` | format in progress |

- Endurance is not an issue: 100k cycles × 24 MiB is about 2 years at a continuous 35 KB/s.

### 3.8 Spill

If the next slot is not fully erased:
- the writer keeps cutting frames, numbering them normally (flags.spilled), into a 2 MiB PSRAM spill queue;
- it programs them in order as soon as GC frees the block. Nothing is reordered or skipped.

This is the operator's rule (invariant 2): "better to have duplicates in both than lost data". PSRAM holds the
data while blocks erase.

When the spill queue and the producer rings are full, `Telemetry::push` returns `Err` (counted in
`tlm.dropped`) and logs drop with a count.

**The board never overwrites unconfirmed blocks.** Data is discarded only by the explicit, ground-only console
command `ring discard-unconfirmed`, after an HTTP `/ring` or esptool dump.

### 3.9 SD mover

**Thread:** core 0, priority 4, below the video writer (6) and the encoder (10): on the SD, video is priority 1
and logging priority 2, so the mover yields to the video writer. It reads frames in `block_seq` order with
`esp_partition_read`, in chunks of ≤ 16 KiB, never beyond the writer's published `programmed` position.

**Demux** (recorderd's rule, `recorderd.cpp:153-182`, plus new mover limits):
- Check `header_crc` and `payload_crc`.
- BOOT sets the current session directory. CONFIRM is skipped.
- For each DATA line:
  - recorderd's rule: a line starting with `@` whose next `@` at position 4 or later closes the routing (so it
    is at least 3 characters, and an `@` among its first three is kept) is appended to
    `<session dir>/<routing>`, as the bytes after that `@`. recorderd also drops every byte below 0x0A before demuxing (`recorderd.cpp:209-211`).
  - New mover rules, not recorderd's: the routing must match `[A-Za-z0-9._-]{3,64}` (no `/`). They differ from
    recorderd only on damaged or unusual lines.
  - The routing `session.csv` goes to **both** `<session dir>/session.csv` and the global
    `/sdcard/recordings/session.csv`.
  - Anything else goes to `<session dir>/recorder.log`.
- **Torn frame** (payload CRC fails): keep each line whose wrapper verifies under its own routing
  (`recorder_wire.verify`) and count the rest as lost.
- **Torn header:** scan forward on 8-byte alignment for the next valid header and salvage lines in between the
  same way.
- Every loss is logged into recorder.log as a full log line, `<uptime> ring :: frame <boot>/<seq>: k salvaged,
  m lost`, wrapped with `recorder_wire.wrap` (as `assemble_capture` does for its own host lines). An unwrapped
  line in a wrapped session counts as rejected damage in `recorder_flight` and `flight_telemetry`.

**Batches** are ≤ 2 s or 256 KiB on the pad and ≤ 1 s in flight. **A batch never spans two sessions** and
touches at most 32 files. Order per batch:
1. Write and fsync a journal **INTENT** `{journal_seq, card_vsn, from, to, [(path, size_before)…]}`.
2. Append each file's bytes from PSRAM in writes of up to 32 KiB (a 1 s flight batch holds only a few KB per
   file), then `fsync` every touched file. This is the conservative one of the two fixes; the other is to
   batch per file across a longer window.
3. Write and fsync a **COMMIT** `{journal_seq, to}`, where a cursor is `{block_seq, offset, boot_id,
   frame_seq}`.
4. Publish the cursor to the writer (two AtomicU32 pairs under a seqlock). The writer writes a CONFIRM frame,
   and GC may then erase older blocks.

**Journal:** `/sdcard/coludo/mover.journal`, two 4 KiB slots A and B.
- Each record (magic `CJN1`, version, `journal_seq`, kind INTENT/COMMIT, `card_vsn`, cursors, entries,
  CRC-32) is written to the slot **not** holding the latest valid record. The order is INTENT(k) < COMMIT(k) <
  INTENT(k+1).
- **Recovery:** read both slots and take the newest valid record.
  - INTENT(k): `ftruncate` each listed file to `size_before`, then redo from `from`.
  - COMMIT(k): the cursor is `to`.
  - A torn write leaves the other slot's previous record, and both cases above stay correct.
  - Without this rule, a power cut at landing (the normal end of a session) would re-move a batch of data
    rows. The duplicated rows' uptime goes backwards: `recorder_flight` reads that as a reboot and splits the
    boot, and `flight_telemetry` files the duplicates out of order. A duplicated header, and so a spliced
    flag, happens only if the interrupted batch was the session's first.

**Duplicate rule.** On a card with no journal, if a target session directory already exists, the mover writes
into `<session>.r<k>/` (k = first unused) and logs it. It never truncates or appends to files it did not
create.

**Handles:** the mover opens and closes files per batch, which is acceptable with per-session directories.
`max_files` goes to 40 for video, HTTP reads and the mover together, with `CONFIG_FATFS_FS_LOCK` at least as
large (M4). Nothing on the link or health path touches FatFs.

### 3.10 SD and flash file layout

```
/sdcard/
  recordings/
    session.csv                      global index, every boot's rows (wrapped lines, no routing)
    000123/                          one directory per session (the boot id %06u, or the recorder.session label)
      000123_imu_lsm6dso32.csv       <session>_<stream>.csv, exactly the bytes recorderd wrote
      000123_health.csv  000123_sequencer.csv  000123_flight.csv  …  000123_<task>_events.csv
      recorder.log                   this session's unrouted lines (status line carries 'session': '000123')
      session.csv                    this session's index rows
  video/<session>/<start uptime ms>.h264   (exists; raw Annex-B, SEI per frame)
  coludo/mover.journal
```

A pulled session directory, with the global `session.csv` copied over the per-session one, is a drop-in
replacement for a Luckfox `/userdata/recordings` dump. `recorder_flight.py <dir> --session <s> -o <out>` and
`assemble_capture.py <s> <dir> <out.txt> [note]` use it unchanged.

Per-session directories avoid FatFs's linear `f_open` scan, and the FAT32 limit of 65 536 entries per
directory that a flat layout reaches after about 1 400 boots.

File names are long (FatFs LFN, already on in `sdkconfig.defaults`), because the names carry the telemetry
routing. 32 GB cards are the safe size. A 64 GB card can be one FAT32 volume (`mkfs.vfat -F 32`); no split is
needed.

**Flash partitions** (proposal, `partitions.csv`):
- `nvs` stays at 0x9000/0x6000, shared with MicroPython: boot count, calibrations, crumb.
- **New `cfg`** is an NVS partition at **0x7E0000, size 0x20000**. It holds blobs `board` and `launch`, plus
  the `creds` namespace.
- 0x410000–0x7E0000 stays free for a later OTA slot (§7, Q5).
- `log` is unchanged.

### 3.11 Row, log and index text (byte rules)

- **uptime:** `esp_timer_get_time()` µs as an unwrapped u64 everywhere. There is no 2³⁰ wrap, and CLOSE uses
  `U mod 2³²`.
- **Schema:** fixnum-era integer columns stay integers. Examples: gyro `gx gy gz` in centideg/s, sdp810
  `dynamic_pressure` (Pa×100), `airspeed_cms`, `temperature` (×100), bno055 roll/pitch in cd, `hdop_cd`, and
  `flight.csv`/`attitude.csv` cd columns.
- **health.csv:** `temp;mem_free;load;oom_s;land_s;leak_kbps;rescues;rescue_ms` is unchanged.
  - `temp` is die °C with 0.1 resolution.
  - `mem_free` is free 8-bit heap (internal + PSRAM).
  - `load` is integer CPU % (busiest core, from FreeRTOS run-time stats).
  - `oom_s` is `None`, `leak_kbps` 0, `rescues` 0, `rescue_ms` 0.
  - `land_s` comes from the elevation slope once ported, else `None`.
- **session.csv:** header `uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon`.
  - Rows are written at `boot`, at every successful clock set (`cc-auto`/`dashboard`), and at `anchor` (60 s
    after Recorder start). The header goes out again before the boot and anchor rows.
  - `utc` and `utc_offset` are empty before 2001-01-01, and `utc_offset` is empty outside −720..+840.
  - Text cells are emptied unless printable ASCII, without `;`, and at most 32 characters (`recorder.py:76-110,602-697`).
- **Session prefix:**
  1. a valid `recorder.session` label (ASCII letters, digits and `-`, at least one letter, at most 32
     characters);
  2. else `%06u` of NVS `coludo/boot`;
  3. else `YYYYMMDD_HHMMSS_<6 random digits>`.

### 3.12 CC tees and the HTTP exits

- **Tees:** `Tee { deadline_ms: AtomicU32, ring: SPSC (writer → executor, 1024 × 256 B in PSRAM), dropped: AtomicU32 }`,
  one for logs and one for telemetry. The deadline is a wrapping u32 in uptime milliseconds (0 = closed), as
  M1's log tee already does: the P4 has no 64-bit atomics (§4).
  - `log [ms]` / `tlm [ms]` drain what was buffered since the last call and re-arm the window for `ms` more
    (default 1000; 0 or less stops it). A lapsed window is discarded on the next tee.
  - Replies are `ok {"lines":[…],"dropped":n}` and `ok {"samples":["@<session>_<file>@<row>",…],"dropped":n}`,
    without the CRC wrapper. A full tee (1024 × 256 B = 262 144 B) is about 350 KB in base64, plus JSON
    quoting; typical ~100 B rows give ~140 KB. Either is well under the hub's 2 MiB line limit.
- **`GET /sd/recordings/…`** exists (`files.rs`). The listing format is `<name>;<bytes>`.
- **`GET /ring[?boot=N]`** streams valid DATA lines in `block_seq` order, a byte-exact wire capture.
  `tools/ring_split.py` demuxes it, and an `esptool read_flash 0x800000 0x1800000` image, into a session
  directory. This is the recovery path when the card failed in flight, and the confirmation step before any
  forced ring discard.

### 3.13 Budget (v0.1 sensor set at the true Rust rates)

| item | value |
|---|---|
| rows | ~375 rows/s × ~68 B on SD ≈ 26 KB/s; in flash with routing ≈ 35 KB/s |
| ring holds | 24 MiB / 35 KB/s ≈ 11.7 min with no SD progress (≈ 4 min if the IMUs go to 1 kHz) |
| flash busy | program ≈ 7 % (flight). GC on the pad: ≈ 13 % more just to keep pace with the writer (35 KB/s ÷ 4 KiB ≈ 8.5 sectors/s × ~15 ms), but the §3.7 rule (one sector after every frame program) gives 20 sectors/s × 13-17 ms = 26-34 % at a 50 ms cut while a moved-block backlog exists |
| core 0 | video 13 % + writer ≈ 9 % (program + format) + mover < 1 % + link/health ≈ 1 % |
| frame overhead | ≈ 800 B/s at a 50 ms cut |

---

## 4. Runtime model

**Rule: no 64-bit atomics.** The P4 core (rv32imafc) has none. A shared 64-bit value goes through a seqlock,
or becomes a wrapping u32 in milliseconds (M1's log tee does that).

### 4.1 Python concept → Rust mapping

| plan B concept | Rust |
|---|---|
| One asyncio loop, cooperative | Threads, one owner per resource. Ordering that was implicit becomes explicit (warm restore before spawn; recorder first). |
| `Task` subclass + `@task.driver` registry by import | `DeviceEntry` + a `match` on the config `driver`/`activity` string; the owner thread runs the device's slot |
| `Controller.setup` serial with retries, `diagnose` on the last try | `controller.rs` bring-up, the same rules, on the main thread before any spawn |
| `_supervise`: a crashed loop goes DOWN and the board keeps flying | Device errors are `Result` → `DeviceEntry::Crashed` (the thread serves the others); a panic reboots (`PANIC_PRINT_REBOOT`) and the warm start is the crash path |
| `Controller.stage` int, 5 writers | `StageCell(AtomicU32)` = stage u8 \| source u8 \| epoch u16, single writer = control thread; others post requests |
| `armed` / `manual` / `warm_started` | AtomicBool; `armed` and `manual` written only by the control thread (and `warm.rs` before spawn) |
| `machine.Timer` + `ThreadSafeFlag` (flight 100 Hz) | gptimer alarm ISR on core 1 → `xTaskNotifyFromISR` (integer-only ISR) |
| `commons.Waiter`, IRQ + `wait_for_ms` | GPIO ISR → notify with timeout; `count > 1` → overrun counter |
| `Databoard` (latest value, rank, window; `value()` extrapolates) | `crates/databoard` seqlock channels; **no extrapolating accessor**; `read() -> Option<Reading{value, source, age_us}>` |
| reconcile learning inside `read()` | on the writer side: the primary's owner updates each fresh backup's offset EMA (α 0.1) in that backup's atomic |
| `Inspector` registry | a frozen sorted `&'static [&'static dyn Inspectable]`; `update`/`probe`/`calibrate` forwarded to the owner's mailbox |
| `Task.claim()` (device conversation guard) | gone: the owner thread services requests *between* its own transactions |
| `Task.note` / `strike` | `Strikes{count}` + `AtomicBool healthy` in the status block |
| `Task.event` → `<name>_events.csv` + log | the same stream (`decimate_us = -1`), a durable record |
| `Controller.find` / `query(waiting=True)` | static wiring: handles passed at spawn; a missing dependency fails bring-up |
| `machine.WDT` + watchdog task (stall check) | the supervisor reconfigures the TWDT to panic (`esp_task_wdt_reconfigure`, `trigger_panic = true`, timeout `wdt_timeout_ms`, as `machine.WDT` does; by default a timeout only warns) and feeds it; control stall > `stall_ms` while controlling → event + `esp_restart` |
| `Mission` singleton | owned by the supervisor (by the `cc` executor until a supervisor thread exists); the others post requests; `MissionGeometry` published by seqlock |
| `cc_client` Dispatcher in the asyncio loop | the `cc` executor thread, fed by link + console |
| GC off / rescue / OOM forecast / `fixed.py` | not ported (§6) |

### 4.2 Threads, cores, priorities

| thread | core | prio | owns / does | paced by |
|---|---|---|---|---|
| gptimer ISR | 1 | ISR | notify control + bus owners (`eIncrement`) | alarm at `schedule_hz` |
| `control` | 1 | 20 | SPI2 IMU devices, StageCell, armed/manual, the attitude filter, sequencer, separation debounce, flight law, LEDC fins | ISR notify |
| `i2c0` | 1 | 18 | I2C port 0 sensor devices (SEN0697, INA226 on v1.x; ICP, SDP810, laser on v0.1, …); the camera SCCB shares the bus handle | ISR notify + rate-monotonic table by tick count |
| `i2c1` | 1 | 17 | I2C port 1 (ICP, SDP810, laser on v1.x; INA226 on v0.1) | same |
| `gnss` | 1 | 14 | UART2 receiver, NMEA/UBX | blocking read with timeout |
| esp_hosted, Wi-Fi, lwIP | 0* | 18-23 | IDF; off from BOOSTING through LANDING (M7) | — |
| `video encoder` | 0 | 10 | exists | camera |
| `supervisor` | 0 | 9 | TWDT feed, health 1 Hz, checkpoint (NVS), gnss_calib, field, Mission | 100 ms base tick |
| `flash writer` | 0 | 7 | `log` partition, ring consumers, tees, GC | drain loop, ≤ 10 ms wait |
| `video SD writer` | 0 | 6 | exists | — |
| `cc` executor, `link`, `console`, httpd | 0 | 5 | command table; Wi-Fi + hub socket; UART0; HTTP | blocking |
| `mover` | 0 | 4 | SD recording files, journal | batches |
| `sim_link` (HITL only) | 0 | 12 | UDP; p0 snapshots | frames |

\* lwIP's TCP/IP task (priority 18) is pinned to core 0 (`CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0`, done in M1).
esp_hosted's tasks run unpinned at priority 23 until M1b pins them through the `_h_thread_create` hook (§7,
R5).

- I2C buses are created **from their owner thread on core 1**, so their `i2c_master` ISRs land on core 1.
- Port 0's bus handle is shared with the camera's SCCB. Camera register traffic is confined to pre-flight.
- No float operations in any ISR (`diagnostics.rs` notes that a float in an ISR aborts).

### 4.3 The flight tick (control thread, 100 Hz)

1. `ulTaskNotifyTake(clear)`: a count above 1 increments `overruns`. `now_us = esp_timer_get_time()`;
   `dt = now - last`, clamped to nominal if ≤ 0 or > 0.5 s (`flight.py:265-266`).
2. **IMU:** read 12 B from the LSM6DSO32 (+6 B from the ADXL375 on v0.1). Push `accel` and `rate`. Write the IMU
   telemetry row if `due`.
3. **Separation:** sample GPIO33 and step the debounce (≥ 3 agreeing reads spanning ≥ 20 ms). A confirmed
   separation in BOOSTING sets the stage to GLIDING and writes `separation.csv` (`stage` after the
   transition).
4. **Mailbox:** pop at most 2 requests (Arm, Disarm, Hold, Resume, SetTrim, SetAngle, ProbeSweep step) and fill
   their reply slots.
5. **Every 2nd tick:** the attitude filter (50 Hz; the 5-cycle blind window kept) publishes `attitude` at p1.
6. **Every 5th tick:** the sequencer (20 Hz) may transition, writing `sequencer.csv` (`reason`, `external` →
   the source field).
7. **If `flight` is enabled:** governor → guidance → PID → mixer → LEDC duty, compare-and-set. `flight.csv`
   every 100 ms, sampled before the gates. Disarmed or no setpoint gives neutral once (gated on `_active`).
   Stale attitude and the on-rod guidance gate apply neutral on every tick and leave `_active` set;
   compare-and-set makes the effect similar.
8. **Heartbeat:** `last_tick_us`, `steps`, `controlling`.

On a `Result` error exit the fins go neutral before returning (the `flight.run` finally invariant). Nothing
ever unwinds: the firmware target uses panic-strategy abort, so a panic aborts and resets the chip. LEDC stops,
the fins lose PWM and go limp rather than neutral, and they recentre only when the warm start runs.

Bus owners run their tables on the same notify:
- BMI323 100 Hz, SDP810 50 Hz, VL53 20 Hz (data-ready first), ICP/BMP581/BMM350/INA226 10 Hz;
- the ICP is a two-phase state machine (issue 0x48A3, collect ≥ 12 ms later);
- mailboxes (Probe, Calibrate, Update{ground, zero_offset_pa, rezero}, Bustune) are serviced between
  transactions;
- a general call is followed by `rearm()` on **every** device registered on that port.

### 4.4 CC executor and request plumbing

- link (hub socket) and console (UART0) → bounded channel → `cc` executor → `commands::handle(line, &Ctx)` →
  reply slot → the caller writes one line.
- Read-only answers (health, whoami, inspect) come from atomics and seqlock snapshots.
- Anything that touches a device or the stage is a request to the owner, waited for with a deadline: ≤ 8 s
  normally, ≤ 25 s for arm/verify/probe/calibrate. A reply therefore always beats the hub's 10 s and 30 s
  timeouts.

### 4.5 Bring-up order (`main.rs`)

1. NVS and the boot count (exists). Latch the reset reason. If the clock reads before 2000, set it to
   2000-01-01 + uptime.
2. Config load from `cfg` → `BootSource`.
3. **Storage first:** SD mount (exists), recorder registry and rings, writer thread (the boot scan runs in it),
   `log` bridge, provenance line `boot: board <id> | firmware <v> | config <id> <source> | reset_cause <n> |
   boot <id> | session <s>`.
4. Buses created on core 1 by their owner threads, which are spawned parked. `layout.resolve` by scan
   (100 kHz probes). Cache the post-layout `config_id`.
5. Mission (launch.config, `max_range_m` from the `field` component).
6. Device setup in config order (sensors, then components), with retries and diagnose. Register channels from
   `provides`. Then the alternatives pass.
7. Freeze the databoard topology and the inspector (`Box::leak` → `&'static`).
8. `warm.rs` restore, before any flight-relevant thread runs.
9. Unpark the bus owners. Spawn control (core 1), supervisor, mover. Arm the gptimer. Reconfigure the TWDT to
   panic at `wdt_timeout_ms` and subscribe the supervisor to it.
10. Console, link and httpd (as today).

### 4.6 Warm-start and reset classes

- **Gate** (`warmstart.py:132-176`):
  - the crumb has a stage and a stamp;
  - the reset is a reset class (below);
  - `0 ≤ now − stamp ≤ 600 s` via `gettimeofday`;
  - GLIDING and LANDING also need the separation latch read at setup.
- **Reset classes:** `SW`, `PANIC`, `INT_WDT`, `TASK_WDT`, `WDT` and the P4-only `CPU_LOCKUP` (an in-flight
  crash) count as resets. `POWERON`, `BROWNOUT`, `DEEPSLEEP`, the P4-only `USB`, `JTAG`, `PWR_GLITCH` and
  `EFUSE`, and unknown are cold.
  - The P4 never reports `EXT`. An EN/CHIP_PU pulse from DTR/RTS or esptool reads as `POWERON` (or `USB` on the
    USB-Serial-JTAG), a cold boot in both plan B and Rust.
  - On the P4, plan B's `HARD_RESET` means SW + PANIC only. The real path into the DONE + armed trap is `SW`:
    the CC `reboot` is allowed at DONE (§7.3, Q3).
- **NVS:** the same keys in namespace `coludo`: `crumb` (JSON ≤ 512 B) and the `stage` i32 written last. A
  rejected gate clears the stage.

---

## 5. Golden vectors and the contract runner

### 5.1 Golden scheme

1. **`src/avionics/golden/generate.py`** (CPython 3.12) imports `src/glider` through `shims.py`. The shims
   provide:
   - a `micropython` module (identity `viper`/`native`/`const`, plus `ptr8`/`ptr32`);
   - a `machine` stub, with an `RTC` stub;
   - `time.ticks_ms/us/add/diff`, and `time.time()` on MicroPython's 2000 epoch as an int;
   - `gc.mem_free`;
   - `version` blocked, so `config_default` stamps `firmware_version` 'dev' as on a clean checkout: the pin
     (`src/glider/version.py` is gitignored and generated by deploy).

   These are needed by `gnss`, `neo6mv2`, `atgm336h` (viper) and `bmi323` (`machine`), and by the dispatcher.
   One builder per ported module. Inputs are explicit literals lifted from the plan B tests, plus seeded
   sweeps (`random.Random(seed)`). **Inputs are written into the fixture and never re-derived in Rust.**
2. **Fixture schema:**
   ```json
   {"meta": {"source": "src/glider/pid.py", "tag": "micropython_plan_B", "commit": "<sha>",
             "glider_tree": "<git rev-parse HEAD:src/glider>", "python": "3.12.x", "generator": "pid"},
    "compare": {"kind": "exact|abs|rel|angle_deg|bytes_hex", "tol": 0.02},
    "cases": [{"name": "...", "input": {...}, "output": {...}}]}
   ```
   Stateful units (Pid, estimators, sequencer, filter) use `{"init": {...}, "steps": [{"in": ..., "out": ...}]}`.
   Floats are written as Python repr (shortest round-trip f64); bytes as hex.
3. **Rust side:** each `crates/X/tests/golden.rs` does
   `golden::run(include_str!("../../../golden/X.json"), |input| port(input))`.
   - Integers, bytes, strings, enums, stage transitions and `config_id` compare EXACTLY.
   - f32 paths use a per-function tolerance stated in the fixture and justified physically.
   - Cases stay away from decision thresholds, where one ulp flips the result.
4. **Freshness gate:** `generate.py --check` regenerates in memory and diffs. `src/control/test/test_golden_fresh.py`
   runs it, the same pattern as `test_gen_pinmap`. A plan B fix that changes behaviour fails CI until the
   fixtures are regenerated on purpose.
5. **Optional dual oracle:** the same builders run under MicroPython on 7C or the taster via `mpremote run`
   store a `board_output` column (f32 board truth next to CPython f64). The current
   `bench_cases.py`/`cases.rs` twin (12 identical `check;` lines) is the seed of this.

**Python semantics to port by hand** (all covered by fixtures):
- floor `//` and `%` → `div_euclid`/`rem_euclid`;
- `round()` half-even → `round_ties_even` (fin table `commons.py:247`, `sg90.py:251`);
- `int()` truncation;
- the `ticks_diff` 2³⁰ wrap is not reproduced, and no vector straddles it.

### 5.2 First fixtures (in milestone order)

| fixture | source | compare | first used |
|---|---|---|---|
| `cc_dispatch.json` | `cc_client.create_dispatcher(config_default.default())` on CPython, with a `mission.Mission`, a stub Controller (stage/armed/manual/`active()`/failures) and `config.BOOT_SOURCE = 'default'` (a bare dispatcher has no whoami `epoch`, an empty `config_source`, and a health without clock/epoch/launchpad/launchpad_set/site/armed/tasks): empty line → no reply; unknown → `err badcmd`; unparsable → `err badargs`; `base64:@@@` → the decided code (§7, R8); whoami/health key sets | exact (keys/types/codes) | M1 |
| `config_default.json` | `config_default.default()` (embedded in firmware until M2) + its `config_id`, with `board.firmware_version` pinned to 'dev' by the shims (§5.1), so `--check` does not flip with every deploy (`config_id` excludes it anyway) | bytes / exact | M1 |
| `config_id.json` | 9 vectors (§M2, §M5) + `config_hitl.default()` variants + launch_config outputs | exact | M2 |
| `config_validate.json` | `validate()` verdicts + messages over the corpus and `test_config` negatives | exact | M2 |
| `recorder_wire.json` | `_ROUTED_VECTORS`/`_LOG_VECTORS` (`test_recorder.py:23-44`), damage cases, Telemetry header/row text, session.csv rows, `_cell`/`_is_label` | exact | M3 |
| `bench_cases.json` | the 12 `check;` lines (`doc/benches/avionics-20261005/micropython-1.29.0/cases.log`) | exact / as logged | M3 |
| `layout.json` | `_TMS7C`/`_TASTER`/`_LEGACY` scans → verdict, edits, ranks | exact | M5 |
| `databoard.json` | Parameter rank handover, window, born-stale (no extrapolation cases) | exact | M5 |
| `drivers_*.json` | lsm6dso32 (scaling, frame decode), sdp810, bmi323, bmp581, bmm350, ina226, icp10111 (pinned exactly), bmp280, sg90 pulse map, vl53 timing; adxl375 with M18 | exact / abs | M6-M10 |
| `commons.json` | `dwell_step`, `apogee_step`, `fin_deflection_limit`, `altitude_m` | exact / abs 1e-3 m | M7 |
| `sequencer_trace.json` | `Sequencer._tick` with scripted post-freshness reads (`test_sequencer.py:43-140` + regressions) | exact | M7 |
| `warmstart.json` | `should_restore` boundaries, torn crumb | exact | M8 |
| `attitude.json`, `magnetic_yaw.json` | `tasks/attitude.py` | angle 0.1 deg; flags exact | M9 |
| `gnss.json` | `test_gnss.py` bursts; `_commands(hz)` bytes for hz 0/1/5/10 | exact; lat/lon 1e-6 deg | M11 |
| `gnss_calib.json`, `field.json` | drift averaging, auto-arm dwell | abs / exact | M11 |
| `pid.json`, `mixer.json`, `governor.json`, `airspeed.json`, `wind.json`, `navigation.json`, `guidance.json` | the corresponding tests' inputs + seeded sweeps | per-function (pid 0.02 deg/step, nav 0.5 m / 0.05 deg, mixer exact) | M13 |
| `control_trace_<scenario>.json` | `virtual_flight.py` with `VF_SEED`, per-tick pipeline dump (opt-in flag) | stages exact; fins 1 deg | M13 |

### 5.3 Contract runner

**Files:**
- `src/control/contract/cases.py`: tiered cases. Each is (tier, request, expected status, argc, JSON key set
  and types, invariants), taken from the `cc-protocol.md` catalog and the wire asserts in `test_cc_client.py`.
- `src/control/contract/transports.py`:
  - `fake`: the **real** plan B dispatcher on CPython over loopback, with the time/gc shims (including
    `time.time()` on the 2000 epoch as an int and the `machine.RTC` stub), a `mission.Mission`, a stub
    Controller (stage/armed/manual) and `config.BOOT_SOURCE = 'default'`. Without them `get-config launch`
    and `update mission` fail, `stage` is ignored and the whoami epoch is a float near 2.74e9;
  - `hub`: the live hub's `/api/cmd` for request lines and its `/api/boards` for the hub tier;
  - `serial`: UART0 with ESP log lines and the ROM boot banner filtered out and DTR/RTS held asserted.
- `src/control/contract/normalize.py`: for `--diff`, removes uptime, mem_free, temp, epoch, clock, session,
  boot_id, psram_free, internal_free and sd_free_mb (so a Rust-only one is no difference either), compares log
  `lines` and telemetry `samples` by shape only (no two boards log alike), and lists firmware_version and
  config_source apart as known differences, never counted.
- `tools/cc_contract.py BOARD=fake|taster|<id> [--tier …] [--diff fake] [--milestone M<n>] [--beats N]
  [--allow-…]` prints a capability report and exits 1 on a contract violation, 2 when the run cannot start or
  does not finish. A documented "not yet" answer (`unsupported`, `badcmd` for `push-*`) is reported, not
  failed, unless `--milestone` claims a milestone that owes the case (each case names the milestone that owes
  it; most are M1).
- Nothing that changes a board goes out without its flag: `--allow-active` (arm, probe, verify, calibrate
  <device>, detect, bustune), `--allow-stage` (the gliding hold, only on a board seen disarmed),
  `--allow-launch-write` (the launch-config round trip; never on a plan B board whose launch.config carries
  `sites`, which plan B's save drops), `--allow-config-write`, `--allow-reboot`. The preflight refuses an
  airborne or armed board, and with any flag one whose stage or arm state it cannot read. A run that stops on
  an error or Ctrl-C sends the undo of what it wrote before it exits.
- `src/control/test/test_contract.py` runs `BOARD=fake` in `make test`.

**Tiers and when they become mandatory:**

| tier | commands | must pass from |
|---|---|---|
| core | whoami, ping, health (the full key set), unknown → badcmd, parse error → badargs, empty line → no reply, one reply per line | M1 |
| system | reboot (ground-only, `ok` with no payload, then a whoami from a new boot: another boot_id, or an uptime shorter than the time since the `ok`) | M1 |
| stream | log/tlm window semantics; a window open across `probe mission` drains a line (a board only); samples `split()` GOOD with `recorder_wire` | M1 (log), M3 (tlm) |
| config | get-config (5 names, `saved` → ok or **badargs**), set-config, reset-config | M1 (launch), M2 (board) |
| inspect | objects, inspect (mission, gnss, servo_* `props.trim` numeric), update (`changed`), stats | M1 (mission), M13 (servos) |
| control | stage, arm/disarm, report; gating | M1 (stage; arm's refusal), M7 (a real arm) |
| diagnostics | probe (two maps), verify (keys; cc.py exit 0/1/3), calibrate, detect, bustune | M1 (probe, verify, calibrate answers), M5 (detect, bustune) |
| ota | push-* | M15 (redesigned) |
| hub | the dashboard row (GET /api/boards): registered, fields filled, online and fresh across `--beats` heartbeats, a clock past 2000 in setting | M1 |

**Hub-level checks.** The hub tier covers registration, the `board_rows()` fields, online across N beats and a
synced clock (it diagnoses a board left in 2000: a whoami epoch that is not a JSON integer, or a sync the hub
logged as refused). Checked by hand, because they need the board powered off or a capture: offline after 3
missed beats, and `start_stream('tlm')` → a capture `flight_telemetry` parses (M3).

**Capture acceptance:** `recorder_wire.split/recover`, `recorder_flight`, `assemble_capture`,
`flight_telemetry` and `flight_kpi` in strict wrapped mode. A `gen_schema --check` diff against a stream table
the firmware exports (`console schema`) keeps `doc/telemetry.md` honest.

---

## 6. What is deliberately not ported, or changed

**Not ported** (MicroPython workarounds, or YAGNI):
- `fixed.py` fixnums and every `_upy`/`_opt`/viper variant. Rust computes in f32. Integer units survive only
  at the telemetry and inspect boundary.
- **GC machinery:** GC disable at BOOSTING and enable at DONE, the airborne GC disable on a warm start, the
  memory rescue and OOM/land forecast, `Waiter`, reused read buffers, lazy `note()` formatting, `Task.claim()`,
  allocation-driven API shapes. Only their visible traces are kept: the health.csv columns, with defined
  values.
- `Databoard.value()` extrapolation (and `read()`'s, which returns the projected value with source None). It
  killed a flight at apogee when a production consumer (`sequencer._detect_landing` reading `agl.value()`)
  used it; that was fixed later. The inspect/status readouts and the sdp810 log strings still call `value()`,
  so dropping it is visible (below).
- `notify()`/`emit()` pub-sub (only separation emits; nothing subscribes), `Controller.close()`/`finish()`
  teardown, `query(waiting=True)`.
- `push-*` `.mpy` OTA, `tools/ota_push.sh` and the hub `push` composite for the Rust image. A Rust board answers
  `err badcmd`; these stay as plan B tools. A firmware OTA is M15.
- Servo coalescing (built, off, no measured benefit). The `led` driver (no pin, no user LED). `bluetooth`
  becomes a stub reporting `radio:false`.
- The Luckfox path: UART recorder output and recorderd stay frozen and unchanged (the logger recorderd freeze
  stands). Fallback 1.a (C6 + SD logger) is not built.
- The `layout` scan re-clock dance: `i2c_master` speed is per device.

**Changed on purpose** (each is visible; the host already accepts it):
- **SD layout:** one directory per session instead of a flat directory (§3.10).
- **uptime:** u64 µs with no 2³⁰ wrap. `recorder_wire.fit` reduces to k = 0 (`recorder_wire.py:268-281`);
  `recorder_flight._clock_times` does not, so the conformance case covers a session longer than 2³⁰ µs whose
  anchor row is dated (M4).
- **f32 text:** `1.764`, not `1.7639999`. lat/lon text comes from i32 1e-7 deg (7 decimals). Byte-for-byte
  board-vs-host comparisons (`hitl_compare`, golden captures) must compare numerically.
- **Rust-only degraded tokens:**

  | kind | tokens |
  |---|---|
  | added | `c6-reboot-guard-missing`, `c6-link-parked`, `no-sd`, `ring-low`, `ring-full`, `ring-dirty`, `ring-unformatted`, `mover-behind`, `not-flight-capable` (until M7) |
  | dropped | `memory-rescued` |
  | kept where the mechanism exists | `attitude-backup`, `attitude-blind`, `STAGE HELD`, `CONFIG FALLBACK -- …`, `needs-calibration`, `cc-less-fallback`, `WARM-STARTED (rebooted in flight)` |

- **Crash semantics:** a panic reboots and warm-starts, instead of marking a loop DOWN and flying on with dead
  data.
- **Wi-Fi in flight:** the radio is **off** from the detected ignition through LANDING and resumed at DONE for
  recovery (M7; an operator stage hold leaves it up). Plan B keeps the pad link up in flight and only stops new connections. This loses in-flight
  CC; the operator accepted that.
- **Stale values in inspect:** a stale channel reads null instead of plan B's extrapolated `value()`.
- **HITL sensor mask:** after masking the `_SIM_SENSORS` sensors by config name, `sim_link.rs` refuses to start
  if one is still enabled; plan B only masks (M12).
- **`_readiness` and `enabled`:** Rust reads a missing `enabled` as enabled everywhere, the Controller's
  meaning. Python's `_readiness` read it as disabled, an inconsistency that is not ported.
- **`config_id` floats:** the CPython f64-repr semantics are used. Identical for every committed file (all
  floats have ≤ 7 significant digits). Plan B's announced id for the same file is cross-checked once on 7C.
- **Mission `sites`** are kept on `set-config launch`. Plan B silently dropped them (`mission.py` `persisted()`).
- **ICP general-call re-arm** is done by bus membership, not by the stale name list (`icp10111.py:297`).
- **INA226 `rearm()`** also restores ALERT_LIM and MASK.
- **VL53 poll mode** checks data-ready, so a repeated range is never republished as fresh.

**Unchanged** (contract): the CC grammar and replies, the routing and CRC wrapper, the session.csv columns, the
stream names and field lists, the config JSON schema and `config_id` algorithm, the NVS namespace `coludo` and
the keys `boot`, `crumb`, `stage`, `sdp810_zero` (i32), `bno055_calib` (22 B), `bmm350_cal` (16 B), and the
`launches/` files.

---

## 7. Risks and open questions

### 7.1 Risks (with the mitigation built into the plan)

| # | risk | mitigation |
|---|---|---|
| R1 | Partial-page NOR programming (several program operations per 256 B page) on the GD `C84019` is assumed, not measured. | Bench gate G1 in M3. Fallback: pad frames to 256 B pages (≈ 2× flash at a 50 ms cut). |
| R2 | The loss bound (§3.3: ~70 ms typical, ≤ ~115 ms worst for rows) needs short flash-lock holds. NVS writes (checkpoint, calibrations), mover reads and the writer share the esp_flash lock. The writer at priority 7 can briefly delay the video writer. B4 was 10 s without video. | R3: a 10 min soak at the real flight rate with FHD@25, the 1 kHz `pace`, and commit-to-commit maximum ≤ 115 ms. Q11 below. |
| R3 | FatFs is not journaled. A power cut during FAT updates (mover or video) can damage the FAT itself, not just a file tail. | The ring keeps everything unconfirmed. `GET /ring` + `ring_split.py` recovery. Host fsck on bad cards. |
| R4 | The first Rust boot carries nothing from MicroPython's VFS (board.config, launch.config, `*.creds`). TMS-7F is sealed and Wi-Fi only. | `tools/provision.sh` writes a `cfg` image (config, launch, creds) with `flash.sh` over USB **before** the seal. Never flash a sealed board without it. |
| R5 | esp_hosted 2.7.0 tasks run at priority 23, unpinned (plain `xTaskCreate`); the streaming-mode length assert is still open. | lwIP's TCP/IP task is pinned to core 0 already (M1). M1b pins esp_hosted by wrapping `g_hosted_osi_funcs._h_thread_create`, the hook `components/hosted_no_reboot` uses for `_h_restart_host`. From M7 the radio is actually off from BOOSTING through LANDING in Rust (a deliberate divergence, §6; plan B keeps the pad link up). |
| R6 | On 7C (v0.1 + SEN0697) a plan B ICP general call resets the BMI323, BMM350 and BMP581 without re-arming them. | Fixed by design in Rust. Q9 asks about plan B. |
| R7 | `gettimeofday` surviving SW, panic and TWDT resets on the P4 is unverified, and gate 3 depends on it. | Measured in M8. Otherwise the crumb carries an RTC-memory stamp. |
| R8 | base64 strictness differs: CPython is lenient, Rust is strict. A bad token could answer differently from the fake. | Rust answers `err badargs` (strict). The contract runner records the fake's deviation as known. |
| R9 | Detector drift vs the host sim: `virtual_flight.py` grades the Python detectors once the board runs Rust. | `commons.json`, `sequencer_trace.json`, `control_trace_*.json` are the only guards. `generate.py --check` in CI. |
| R10 | HITL masking: a fresh bench sensor silently becomes the only source (pitot 7/27, mag 9/16). | `sim_link.rs` masks the `_SIM_SENSORS` sensors by config name (they are sensor names, not drivers), as plan B does, and re-applies the mask after layout. It then refuses to start if one is still enabled, which is new (plan B only masks). |
| R11 | Per-sensor axes: the LSM6DSO32 and BMI323 publish raw axes into one channel, and a failover may flip axes. gx/gy and the signs are unverified on any airframe. | M9 move-and-hold check. Q7. |
| R12 | Two physics implementations would repeat the sim-mirrors-flight-code trap. | `sim_model.py` stays the only physics. Rust never ports it. |
| R13 | Silent-success harness traps (1 of 12 flown reported done; empty streams passed). | Every new runner counts artefacts and fails loudly. |
| R14 | Port 0 is shared with the camera SCCB, and `i2c_master`'s internal mutex arbitrates. | Camera register traffic only pre-flight. Measure the bus-owner jitter with video on in M6. |
| R15 | One reply line must stay under the hub's 2 MiB limit (`server.py:697`). | Tee capped at 1024 × 256 B (~350 KB base64 when full, plus JSON quoting). `get-config` ~10 KB. |

### 7.2 Decided

The operator's decisions on the earlier questions, one line each:
- **Q1. SD layout:** one directory per session with long file names (the names carry the telemetry routing);
  video at `/sdcard/video/<session>/<start uptime ms>.h264` (§3.10).
- **Q4. `config_id` before vs after layout:** keep plan B's split, which is plan B behaviour: `set-config`
  returns the pre-layout id, `whoami` announces the post-layout one, and the boot log line names both (M5).
- **Q6. Watchdog timeout:** keep 5000 ms in board files (Python `validate` rejects < 4000) until Rust measures
  its own floor.
- **Q8. f64 (D18):** f32 by default. f64 needs a clear written reason and only goes into absolutely critical
  spots: lat/lon at the JSON boundary and the config document's fidelity (its internal `serde_json` f64, for
  hash and bytes only, never computed with); the ICP-10111 polynomial only if the golden f32-vs-f64
  comparison (M6) proves f32 insufficient. Never f64 for plain values such as the MCU temperature.
- **Q10. Arm gate on ring space:** `ring_erased_s ≥ 300 s` as a constant, with no config key (YAGNI).
- **Q12. Provisioning:** `tools/provision.sh` is approved as tooling. The `cfg` image is built in a temporary
  directory and never committed: it carries Wi-Fi credentials.
- **Q13. Video:** FHD@25 (FHD@30 is encoder-bound at about 26.6 fps on P4 rev v1.3), 8 Mbit/s, IDR every 1 s.
  Recording starts when the glider stands vertical on the rod (attitude, M9; arm until then) and stops 30 s
  after DONE. On the SD, video is priority 1 and logging priority 2.
- **Q14. Apogee dwell:** port plan B's reuse of `launch_ms` faithfully (`sequencer.py:364-366`), to stay
  comparable with the HITL baselines.

### 7.3 Open for the operator (each with a recommendation)

- **Q2. `update` mid-flight.** Plan B leaves `update` ungated in every stage. Object names have no wildcard
  (`update servo_*` answers `err badargs 'no object servo_*'`), and `all` broadcasts to every board, not every
  object. The real risk is `all update <exact servo name> {...}` or a stale dashboard selection moving a fin in
  flight; with the radio off from BOOSTING through LANDING in Rust (§6), that exposure narrows to the console
  and held stages. Recommended: keep `update mission` open (clock, pad), and make servo `angle`/`trim` and
  sensor `ground`/`rezero`/`zero` **ground-only** in Rust. This is a deliberate divergence from plan B.
- **Q3. Warm start after a SW reset at DONE?** The P4 never reports `EXT`: an EN/CHIP_PU pulse from DTR/RTS or
  esptool reads as `POWERON` (or `USB`), a cold boot in both plan B and Rust, and plan B's `HARD_RESET` means
  SW + PANIC on the P4. The real path into the DONE + armed ground trap is `SW`: the CC `reboot` is allowed at
  DONE, plan B writes the crumb on entering DONE while armed, and Rust already counts SW as a reset (§4.6).
  Should a SW reset at DONE with an armed crumb warm-start? Recommended: count SW as a reset only when the
  crumb stage is BOOSTING..LANDING, so a reboot at DONE boots cold.
- **Q5. OTA** (needed only by M15). The partition table has a 4 MB `factory` slot. Recommended:
  - shrink to `ota_0` and `ota_1` of 2 MB each, plus `otadata`, all inside 0x10000–0x7E0000;
  - `log` and `nvs` untouched;
  - the image goes up over HTTP POST to the file server (not 256 B CC chunks), and the command is gated to
    ground and disarmed.
- **Q7. Optional `axes` key per sensor** (needed by M9; a signed permutation such as `'-y+x+z'`, default
  identity). This is a schema addition, and old files stay valid. Approve before M9?
- **Q9. Plan B fix for the ICP re-arm on 7C.** It is a real in-flight failure mode on the board that still
  flies MicroPython in December. Fix it there (a plan B bug fix) or accept it for December?
- **Q11. Checkpoint NVS writes in flight** add up to ~16 ms of flash-lock time per NVS page erase. Plan B writes
  the crumb at 1 Hz while armed and airborne, and also on every stage change while armed, including entering
  DONE after landing (the crumb behind Q3's trap). Recommended: keep NVS for the crumb (warm start needs it),
  write it as plan B does, and accept ~115 ms worst-case loss. Alternative: a crumb frame inside the ring.
- **Q15. Post-landing beacon** (AP `coludo_<BOARD>_beacon` with the last fix). It is scheduled as M17. Should
  it come before the first passive Rust flight instead?
