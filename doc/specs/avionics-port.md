# Avionics port: src/glider (MicroPython) → src/avionics (Rust)

Status: accepted as the build plan; revised 2026-10-06 (operator ring redesign); M1, M1b, M2 done, M3 in rework.
Branch `lmoiseichuk/phase6_rustification`.
Done so far: M1 `e34f11e`, M1b `07ab6f2`, M2 `8efffa8`/`6303a2d`/`c06bf0a`, G1 `127bc0d`, M3 host crates `97f785f`
(to be reworked), `ring_split.py` `7aa92a7`.
Plan B is frozen at tag `micropython_plan_B`, and 7C still flies it. Its one sanctioned change, due with M5, is
the ICP re-init by bus membership for 7C's SEN0697 (§7.2, Q9).

This document is the build plan for moving the flight firmware from `src/glider` to the Rust firmware in
`src/avionics`. Four things stay as they are on the wire and on disk, and the Rust board must match them:
- the CC protocol (`doc/specs/cc-protocol.md`);
- the recorder wire format (`doc/specs/recorder-wire.md`);
- the config JSON and board files (`launches/<date>/<BOARD>/*.config`, `doc/specs/board-config.md`);
- the telemetry schema (`doc/telemetry.md`).

`src/control` (hub, dashboard) and `tools/` (`tools/cc.py` among them) keep working against the Rust board
without changes, apart from the duplicate filter the SD-reading tools gain (§3.8). The new host code listed below
(contract runner, pull tool, card preparation, HITL host) is additive.

Decisions this plan builds on:
- **D5:** the flight loop is paced by a timer interrupt on core 1.
- **D6:** the code runs XIP from PSRAM, and the flash ring is kept.
- **D18:** f32 by default. f64 needs a clear written reason and only goes into absolutely critical spots
  (§7.2, Q8); never for plain values such as the MCU temperature.
- No mutex in hot paths: one owner per resource, plus atomics and task notify.
- Storage tiers: telemetry → PSRAM rings → flash ring (top priority: power can drop at any moment) → SD (the
  ring_saver, below video). On the SD, video is priority 1 and logging priority 2: the saver yields to the video
  writer (§3).
- Changes run only in SETTING: `update`, `set-config`, `reset-config`, `calibrate <device>`, `probe`, `detect`,
  `bustune` and `reboot`. Reads, `stage` and `disarm` stay open, and there is no reboot at DONE (§6).
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
| `golden` | — | **exists** (M1). Dev-only. Loads a fixture JSON and applies its compare kind (`exact`/`abs`/`rel`/`angle_deg`/`bytes_hex`) with per-case messages. |
| `board_config` | `config.py`, `config_default.py` (as data) | **exists** (M2). `ConfigDocument(serde_json::Value)`, Python-repr canon, `config_id`, `validate` (the same accept set as Python), `load` → `BootSource`, a canonical pretty writer, and a typed read-only view. |
| `layout` | `layout.py` | `resolve(&mut Value, scan)` → `Resolved{Declared,Detected,Undecided}`, plus `verdict`/`apply`/`fit`/`rank`/`fitted`. The scan is injected as a closure. |
| `mission` | `mission.py` | **exists** (M1; it came forward from the supervisor: the `cc` executor owns the Mission until a supervisor thread exists). The launch.config document, merge/persist (keeps `sites`), the HPRC default, zone geometry, `select_site`/`fallback_zone`, `probe` checks. Positions are i32 1e-7 deg plus an f32 local metric frame. |
| `recorder_wire` | `recorder.py` (wire parts), `tools/recorder_wire.py` | **exists** (`97f785f`). CRC-32, `wrap`/`verify`/salvage, row/log/header/session.csv text (how the saver prints a record), label rules, and recorderd's demux `split`. Its flash frame codec goes in the M3 rework. |
| `recorder` | `recorder.py` (producer side) | **exists** (`97f785f`), extended in the M3 rework. Stream registry, `Telemetry` (`due`/`push`), the binary record codec (§3.2), per-producer SPSC byte rings, the CC tees and the counters. |
| `flash_ring` | — (new; replaces the UART + recorderd) | **exists** (`97f785f`), **in rework** for the operator's ring (§3). 4 KiB blocks, the block header and chunk codec, the bring-up scan and block table, the writer's append, the saver's walk, and the eraser's rules with newest wins. Written over two traits, `Flash` and `Fs`, so it is host-tested with an in-memory NOR flash that cuts power on demand and a temp directory. |
| `databoard` | `databoard.py` | `no_std`, core atomics. Seqlock `Channel<T>` and `Parameter<T>` (rank, window, `read() -> Option<Reading>`). There is no extrapolating accessor. Reconcile learning runs on the writer side. |
| `sequencer` | `controller.py` Stage, `tasks/sequencer.py`, `commons.dwell_step`/`apogee_step` | **exists** (M7; `Stage` since `83c02236`). One module per plan B module: `controller` (`Stage`, `#[repr(u8)]` ids 1..5 SETTING..DONE with names, `ground()` and `airborne()`; 0 is the NVS no-checkpoint sentinel, not a variant), `commons` (the leaky `dwell_step`, the IIR `apogee_step`) and `sequencer`, whose pure `StageMachine::step(now_ms, stage, &Readings, manual) -> Step` takes the stage the `StageCell` holds; the caller writes the step's advance into the `StageCell` before the next step. A `Step` carries up to two sequencer.csv rows, the external move seen first and then the machine's own advance, because one tick can record both (a hold released past the flight timeout). The crate is `std`, for the `f32::sqrt` of the reason text, which `core` lacks on stable. Golden: `commons.json` (`dwell_step`, `apogee_step`) and `sequencer_trace.json`. |
| `warmstart` | `warmstart.py` | The crumb serde (the same NVS keys), `should_restore(...) -> (bool, Reason)` with the boot rules (§4.6), and the apply plan as data. |
| `control` | `pid.py`, `mixer.py`, `governor.py`, `airspeed.py`, `wind.py`, `navigation.py`, `guidance.py`, `tasks/attitude.py` (filter), `commons` fin table and `altitude_m` | **exists** in part (M1: the navigation subset; the rest in M9/M13). f32 flight math with one module per Python file. Every function takes time as an argument; nothing reads a clock. |
| `gnss` | `gnss.py`, `drivers/neo6mv2.py`, `drivers/atgm336h.py` | NMEA checksum and parse with integer ddmm→e7, GSV/sky row, CASIC/PUBX/UBX command bytes, and a thin wrapper over the `ublox` crate (NAV-PVT) for the NEO-M8N. |
| `drivers` | `drivers/*.py` (device logic) | One module per part. Register init tables as data, conversions, the CRC-8, and `Device` trait impls over embedded-hal 1.0 `I2c`/`SpiDevice`. Host-tested with a scripted mock bus. |
| `sim_link` | `tasks/hitl.py` (interface only) | A versioned, allocation-free binary codec for SENSORS/FINS frames, shared by the firmware and `sitl`. |

### 1.2 Firmware modules (`src/avionics/firmware/src/`)

| module | role |
|---|---|
| `main.rs` | Bring-up in a fixed order (§4.5). |
| `board.rs`, `thermal.rs`, `sd.rs`, `video.rs`, `files.rs`, `diagnostics.rs`, `console.rs`, `link.rs` | **Exist.** `board.rs` loses its hard-coded `taster` (M2). `files.rs` gains `GET /ring` and `GET /ring/image` (M3) and an answer for locked files (M4). |
| `commands/{mod,identity,control,inspection,config,diagnostics,streaming,system}.rs` | Replaces `commands.rs`. One table and one executor thread (§4.4). |
| `config.rs` | Config store on the `cfg` NVS partition. Boot load, then `Box::leak` into a `&'static` resolved document, typed view and cached `config_id`. |
| `controller.rs` | Sequential bring-up: `DeviceEntry` table, setup retries, the alternatives pass, the static databoard topology. |
| `inspector.rs` | The frozen `&'static [&'static dyn Inspectable]`, sorted by name. |
| `recorder.rs` | The ring bring-up (scan, block table, positions, session-start record) and the three ring tasks: `ring_writer`, `ring_saver`, `ring_eraser` (§3). It glues `crates/recorder` and `crates/flash_ring` to `esp_partition_*` and FatFs. |
| `logging.rs`, `components/log_hook` | The `log` crate bridge and IDF's `esp_log_set_vprintf` hook, into recorder.log. |
| `beacon.rs` | The DONE beacon: the open AP `<BOARD>_glider` and its page (M11b); the recorder's beacon parameters in `beacon.csv` (M20). |
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
| `tools/board_pull.sh`, `tools/ring_split.py` | Pull a session over HTTP; turn a ring image or a `/ring` capture into session directories (`ring_split.py` exists, `7aa92a7`, and is re-pointed at the binary blocks in M3). |
| `tools/sd_prepare.sh` | The operator's card preparation: discard the card, FAT32 on 25–50 % of it, the rest untouched (§3.10). |
| `tools/hitl_host.py`, `doc/specs/sim-link.md` | HITL with the physics on the host (`sim_model.py` stays the one implementation). |
| `tools/provision.sh` | Writes a board's `cfg` NVS partition (board config, launch config, Wi-Fi creds) over USB with esptool, after `flash.sh`, once the board's partition table proves the `cfg` row; the image lives only in a temporary directory and is never committed (§7, Q12). |

---

## 2. Port order: milestones

Rules for every milestone:
- Each one fits in one or two days on the taster.
- Each ends with a commit of the passed stage. A failed experiment gets an enable commit plus a revert commit
  carrying the reason.
- `src/glider` is never edited, with one sanctioned exception: the ICP re-init by bus membership for 7C's
  SEN0697 (M5, §7.2 Q9). The generator only reads it.

The taster is v1.1. Its scan in `test_layout.py:27` (`_TASTER`, derived from its 2026-09-16 verdict plus the
ES8311 anchor, not a recorded scan) is i2c:0 {0x15, 0x18, 0x40, 0x47, 0x69} and i2c:1 {0x25, 0x29, 0x63}. Its
VL53L1X (0x29 on i2c:1) has since moved to 7F. It carries a NEO-6M, three SG90s, the camera and a
32 GB SD card. A part the taster lacks is verified on 7F (or 7C) when that milestone reaches it.

### M1. CC surface parity: the hub and dashboard treat the Rust board like a MicroPython one (2 days)

**Scope**
- **`commands/` table.** Add `ErrCode` (badcmd, badargs, invalid, unsafe, refused, unsupported, internal) and
  `Gate` (`Open`, `Setting` = the stage is setting). A parse failure answers `err badargs <repr>`, which
  replaces `badline` (`commands.rs:11`). An unknown command answers `err badcmd <cmd>`. A handler `Err`
  answers `err internal`.
  - `Setting` gates `update`, `set-config`, `reset-config`, `probe`, `detect`, `bustune` and `reboot`, and the
    `calibrate` handler gates `calibrate <device>` (a bare `calibrate` is a read). Reads, `stage` and `disarm`
    stay open in every stage (§6).
  - The gate refusal answers code `unsafe` with the text `err unsafe refused -- stage is X (changes only in
    setting)`. (`refused` is only the `update` driver-rejection code, even though the gate text starts with the
    word.)
  - M1 shipped plan B's gate (`Ground` = setting or done, with plan B's text `refused in flight -- stage is X
    (ground-only: setting/done)`); the operator's rule of 2026-10-06 narrowed it to SETTING.
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
  - Also: `inspect mission`, `get-config launch`, `set-config launch` (SETTING-only), and the health launchpad
    fields.
- **System and streaming.**
  - `reboot` joins the shared table (SETTING-only). It replies `ok` and calls `esp_restart` 200 ms later. The
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
  - `set-config board` validates, writes, and replies `{config_id of the payload}`, SETTING-only.
  - `reset-config` erases the key.
  - `get-config` follows the plan B meanings: `board`/`running` (the running document), `saved` (re-run
    load), `default`, `launch`.
  - `whoami.config_source` reports the BootSource. `degraded` adds `CONFIG FALLBACK -- …`.
- **`board.id` and `wifi`** come from the config: networks, `cc_host` (absent = dial `.1`, `''` = CC off) and
  `cc_port`. Passwords come from `cfg`/`creds` keyed `w<crc32(ssid) hex>`, because NVS keys are at most 15
  characters. The console `wifi` command stays as the fallback.
- **`tools/provision.sh [--port P] [--launch launch.config] [--ssid NAME]... [--dry-run] <board.config>`**
  builds a `cfg` NVS image with IDF `nvs_partition_gen.py` and writes it itself with esptool, after `flash.sh`
  (§7, Q12): the board config and launch.config as blobs, and per `--ssid` the password from `NAME.creds` in
  `CREDS_DIR` (default `src/glider`), read as plan B's MicroPython wifi driver reads it. It refuses what plan
  B's `config.validate` or the board's serde_json would refuse, reads the image back with IDF's NVS parser, and
  reads the board's partition table first: no NVS `cfg` at `partitions.csv`'s offset and size (a MicroPython
  board's filesystem lies there) means no write. The image is built in a temporary directory and never
  committed: it carries Wi-Fi credentials. `--dry-run` builds and verifies it and lists its keys only.

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

### M3. Recorder core, the ring_writer and the bring-up scan (2 days; DONE 2026-10-06 on the taster)

The host crates were built first (`97f785f`) to the earlier design: text in flash, 64 KiB blocks, never
overwrite. They are reworked to the operator's ring (§3) before M4. `tools/ring_split.py` (`7aa92a7`) is
re-pointed at the binary blocks.

**Scope**
- **Partition table first** (§3.10): `otadata`, `ota_0`/`ota_1` of 3 MB each, and the 1 MB `cfg` at 0x620000.
  `flash.sh` writes the app to `ota_0` and erases `otadata`; `provision.sh` follows `partitions.csv`; the
  taster is re-provisioned.
- **`crates/recorder_wire`:** wrap/verify, row/log/header/session.csv text, `split`. Its flash frame codec goes.
- **`crates/flash_ring`** (reworked): 4 KiB blocks, the block header and the chunk codec (§3.4), the bring-up
  scan and the block table (§3.6), the writer's append. It keeps the `Flash` trait, `MemoryFlash` (power cuts on
  demand), the CRC, the G1-proven append programming and the power-cut harness.
- **`crates/recorder`** (extended): every record carries its destination file name and uptime, and the kinds are
  HEADER, ROW and LOG (§3.2). It keeps the byte rings, the registry, `Telemetry`, the tees, the seqlock and the
  counters.
- **Firmware recorder** (`recorder.rs`, `logging.rs`, `components/log_hook`):
  - the bring-up routine: the scan, the positions and the session-start record (§3.6);
  - the ring_writer thread, copying records verbatim (§3.3);
  - CC tees (replacing M1's temporary one), formatted by the executor when it drains (§3.12);
  - session.csv `boot`, clock-set and `anchor` rows;
  - the `'session': '<s>'` status line every `stats_ms`;
  - `health.csv` at 1 Hz (`supervisor.rs`);
  - the `log` crate bridge plus IDF `esp_log_set_vprintf` (the `log_hook` component) into recorder.log;
  - `GET /ring[?boot=N]` and `GET /ring/image` on the file server (§3.12);
  - `board::reset_reason` made public, for the session-start record.
- **Tools:** `ring_split.py` decodes the binary blocks and keeps its demux and session layout. The duplicate
  filter (§3.8) goes into `tools/recorder_wire.py`.
- The Recorder comes up **first** in bring-up, so setup-time log lines are not lost (`recorder.py:21-23`). The
  old global-decimation latch bug is fixed separately, by reading the global decimation at use (§3.2), as plan
  B fixed it.
- The ring_eraser comes with the writer in M3: the writer needs erased blocks ahead. Until M4's saver exists
  nothing is saved, so in M3 the eraser runs newest wins only (§3.7), oldest non-flight blocks first.

**Proof**
- Golden `recorder_wire.json`: the 8 literal vectors (`test_recorder.py:23-44`), damage cases, row/log/index
  text.
- Host:
  - the power-cut sweeps, re-targeted at 4 KiB blocks: no valid chunk lost, and every reboot opens the right
    block;
  - a Rust-built ring image passes `ring_split.py` and then `flight_telemetry` strict, with every line 'good';
  - a chunk programmed twice reads as one set of lines once the duplicate filter has run.
- Bench gate G1 **passed** (`127bc0d`): 10 000 frames of 40-200 B appended at 8-byte alignment read back with 0
  corrupt, and 32 programs of 8 B into each of 64 pages left 0 bytes different. Partial-page programming on the
  GD `C84019` is safe (§7, R1).
- On the taster:
  - the contract runner with the log and tlm tiers live; the dashboard 'telemetry' stream shows health rows;
  - a `/ring` dump parses strictly;
  - 5 power pulls while writing at the flight rate with FHD@25 recording: each loses at most §3.3's worst case
    (about 45 ms of rows, 65 ms of log lines), and the scan after each is clean;
  - the 1 kHz core-1 `pace` stays at 0 missed;
  - the image boots from `ota_0` and stays under 3 MB.

**Results (2026-10-06, the taster):**
- Contract: `--milestone M3` gave 0 violations; the stream tier (log, tlm through the recorder's tees) passed
  11 of 11.
- `/ring` parses strictly: boots 267 and 268 decode as 425 and 139 lines, all 'good' in `flight_telemetry`, with 0
  unproven, 0 lost and 0 duplicates. `/ring/image` (24 MiB in 5.9 s) decodes to the same lines.
- First boot after MicroPython: the scan took 148 ms and found 3570 foreign slots. The eraser formatted them in
  about 5 minutes while the writer logged from block 0, and a reboot mid-format resumed it.
- Load, video and pace together:
  - `ring load lines_per_s=200` (≈ 23 KB/s, above the flight's 16 KB/s) with FHD@25 recording and `pace` on
    core 1;
  - pace: 20 000 ticks, max 38 µs late, 0 missed;
  - video: 954 frames, 0 dropped, 1 missed (around the Wi-Fi join);
  - ring: 229 blocks, 0 drops, 0 flash errors.
- 5 EN resets under that load (an EN reset, not a power pull):
  - each scan was clean (155-159 ms, 0 foreign, 0 other version), and each boot opened the next block;
  - 0 lost, 0 torn, no hole in the load lines;
  - each boot's last line lies within the method's ±70 ms of the reset moment.
  - Open: true power pulls, by the operator.
- Internal RAM: the encoder's reference buffer is one ~138 KB internal block at 1080p (`3 × 16 × 24 × 120`
  macroblock columns). After the ring threads' stacks and buffers (~40 KB) split the heap, the largest block was
  128 KB and `video start` failed. Boot now reserves the encoder right after the SD mount, before the ring threads
  (`video::reserve`):
  - 215 KB free after the reservation;
  - 110 KB free with Wi-Fi up and the camera open;
  - 4 × 9.65 MB downloads under `ring load` and video, with no esp_hosted assert.
- Image: 1.81 MB, inside the 3 MB `ota_0`.

**Operator gains.** Live telemetry on the dashboard. From boot, every log line and row is on flash with about
11 ms typical and at most about 45 ms at risk (65 ms for log lines, §3.3). No Luckfox and no SD card are needed
to get a record.

### M4. ring_saver, SD, HTTP pull (2 days)

**Scope**
- **`crates/flash_ring`:** the saver's walk, decode and batches (§3.9) over the `Flash` and `Fs` traits; the
  eraser's normal rule (erase behind the saver, §3.7) on top of M3's newest wins; the `ring_unsaved_s` figure.
- **Firmware:** the ring_saver thread at priority 4, set up by the bring-up routine (§3.6) beside M3's writer and
  eraser; the saver position written to `cfg` about once a second.
- The `ring-*` and `saver-behind` degraded tokens.
- Recorder `stats`/`inspect` grows `ring:{…}` (erased, unsaved and lost blocks, the positions) and `saver:{…}`.
- `sd_card.c` `max_files` goes from 8 to 40.
- `CONFIG_FATFS_FS_LOCK` goes from 0 (the IDF default) to at least `max_files` (40). Below that, a full lock
  table answers `FR_TOO_MANY_OPEN_FILES` (ENFILE).
- `files.rs` answers `503` on a locked file. Through `std::fs` and the VFS that arrives as errno EACCES (os
  error 13), not `FR_LOCKED`. Without the lock table, an HTTP read during a saver batch simply succeeds with
  the size recorded before the batch.
- `tools/sd_prepare.sh`, the operator's card preparation (§3.10). The firmware mounts the first partition and
  never formats.
- `tools/board_pull.sh <host> [session] [outdir]` verifies file count and byte sizes and flags 0-byte streams.

**Proof**
- Host end to end: N producers, then writer, saver and eraser over `MemoryFlash` plus a temp SD directory, then
  `recorder_flight` (0 rejected, board.log non-empty, session.csv dates the boot) and `assemble_capture` plus
  `flight_telemetry.parse` (all good). This includes:
  - a saver cut at every step of a batch, re-saving into duplicates that the tools drop;
  - newest wins with a dead SD: the pad blocks go first, the flight blocks stay, and every loss is logged;
  - a session longer than 2³⁰ µs whose `anchor` row is dated. The long-session case is limited to a dated
    anchor (the normal case: the hub syncs on connect, usually inside the 60 s before the anchor row).
    `recorder_flight._clock_times` still enumerates 2³⁰ wraps for every session.csv row, so in a boot longer
    than 2³⁰ µs a clock-set row that is the only dated row gets two places and the boot stays undated, as it
    already does for plan B. Restricting the test is the conservative one of the two fixes; the other is a tool
    change (no wraps seen in the streams means k = 0).
- On the taster: a session lands on the SD while video records, and parses through `assemble_capture`,
  `flight_telemetry` and `recorder_flight`. 5 power pulls during saver batches each resume the copy at the next
  boot with no line lost (duplicates only).

**Operator gains.** `tools/board_pull.sh taster` drops a session into the same tool chain as before
(`assemble_capture`, `flight_svg`, `flight_report`, `flight_kpi`, `recorder_flight`). The flash empties itself
to the SD in the background, and a board without a card keeps the newest ~26 min.

### M5. Buses, layout, databoard, device registry (2 days)

**Scope**
- **`I2cPort`** for ports 0 (SDA7/SCL8) and 1 (SDA31/SCL30). Each implements `embedded_hal::i2c::I2c` over
  `i2c_master` with per-device speed. Wedge = 4 consecutive failures, cured by `i2c_master_bus_reset` and
  re-adding the handles. Also `general_call_reset()` and `probe(addr)`.
- **Generic bus re-init:** after a general-call reset, the bus owner re-initializes **every** device registered
  on that bus, found by membership in the device registry, never by a hard-coded list (§4.3). The general call
  is rare, so there is no per-device fast path.
- **Plan B patch** (the one sanctioned `src/glider` change, §7.2 Q9): the same membership-based re-init in
  `drivers/icp10111.py`, so 7C's SEN0697 (BMI323, BMM350, BMP581) is re-armed after the ICP's general call.
  It is tested on a board, or with plan B's own test pattern, and committed on its own; the golden fixtures are
  regenerated with it (their meta pins the `src/glider` tree).
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
  `bustune taster i2c 0` ladder restores the frequency afterwards. A forced general call on each port is
  followed by a re-init of every device registered there, and each reads valid data afterwards.

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
  (7C, next to the SEN0697). A two-phase measurement, the general call, and the **re-init of every device on
  that bus** (M5; §7, R6).
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
- **Ring gate:** `arm` and `verify` also require the eraser to hold its reserve: at least ~1 % of the ring erased
  ahead of the writer (§3.7). No card is a `no-sd` warning, not a refusal: newest wins keeps the flight's
  blocks over the pad's.
- **Wi-Fi policy** `auto`: the radio is **off** from the **detected** ignition (the stage machine's own
  SETTING→BOOSTING transition) through LANDING and resumed at DONE for recovery (the beacon, M11b). This is a
  deliberate divergence from plan B, which keeps the pad link up in flight (§6). An operator hold (`stage
  boosting` from CC, manual) leaves the radio up: keying on a held stage would cut the very link the operator
  holds it from, and the console would be the only way back.
- **Video by stage:** recording starts when the glider stands **vertical on the rod** (attitude-based, from
  M9; until then arm is the trigger) at 1920×1080@25, 8 Mbit/s, and stops at DONE (M11b). The console default
  is 25 fps (GCL `video start`, done).
  - **Segments (operator, 2026-10-06):** on the pad the recorder starts a new file every 5 minutes, bounding what
    one damaged file can cost. Once ignition is detected the current file keeps running through the whole
    flight: a file switch never happens in the air.
  - **Image tuning (done 2026-10-06, components/coludo_video):** the first taster clip was sharp, but
    rotated 90° (the module sits sideways), flat and lavender-cast, with 24 % of the frame clipped and grain
    indoors. Espressif's esp_ipa does not run on rev v1.3, so nothing tuned the ISP. Now:
    - the sensor's own AEC aims at a mean linear luma of 48 instead of 84 (GCL `video start ae=`), under an 8×
      gain ceiling instead of 64×;
    - the ISP runs a Bayer denoise and the OV5647's colour matrix (both from esp_cam_sensor's
      `ov5647_default.json`), plus a BT.709 gamma curve;
    - a grey-world trim, once a second from 16 sampled rows, sets the ISP's red/blue gains over the sensor's
      AWB. It converged in about a second to R 0.94 / B 0.91.
    - Result: neutral whites (mean R 101 G 102 B 101), 7 % clipped (a sunlit window), no grain. Still open:
      the rotation (fix it in the mount, or with the PPA). Optionally, freeze the trim at ignition.
    - The taster's module (a Raspberry-Pi-style OV5647 board) is bench-only; it proves the pipeline. The flight
      camera is the Arducam OV5647 (B012V1HEP4, ~3 g). Same chip, so the code and the colour matrix carry over.
      Re-check per module:
      - its oscillator: `OV5647_CLOCK_HZ` (the taster's is 24 MHz), measured from the SEI fps (`video_sei.py`);
      - `bars`, to confirm the Bayer order;
      - a daylight clip: `ae`, the trim's converged gains, the focus and the IR filter;
      - the mounting rotation.
  - **The pipeline stays open across recordings:** a second `video start` failed `No memory for reference
    frame`. The H.264 encoder needs a contiguous block of INTERNAL RAM, and after a recording (or ~40 s of Wi-Fi
    traffic) the internal heap no longer had one (largest block 75–80 KB). The first start opens esp_video, the
    capture buffers, the ISP tuning and the encoder, and keeps them: ~160 KB internal plus ~12.6 MB PSRAM.
    - A stop is STREAMOFF. A start re-queues, sets rate control, forces an IDR, and does STREAMON (which also
      rewrites VTS and the AEC target).
    - The two halves close separately. The encoder opens first and closes only for another size or QP range. The
      camera closes for another mode or Bayer order, and after a failed start, without the encoder.
    - A failed STREAMOFF wedges the recorder until a reboot: DMA may still be writing the buffers, so nothing is
      freed.
    - This also suits the 5-minute segments above.
  - **Open: sporadic single missed frames:** about 1 in 300, with Wi-Fi up. At FHD@25 the encoder takes 36 of the
    40 ms on core 0, beside esp_hosted and lwIP. The radio is off in flight (from BOOSTING).
  - **Wi-Fi vs camera RAM:** with the encoder's internal block held, a full-speed download asserted inside
    esp_hosted (`transport_drv.c:267 copy_buff`: every TX frame is copied into a fresh INTERNAL|DMA buffer) and
    rebooted the board. The same would happen to any heavy pad traffic while recording. Fix:
    `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` (lwIP's heap in PSRAM). Proof:
    - 3/3 resets with a 14 s recording across the Wi-Fi join, then 3 × 14 MB downloads, at 3.3–4.4 MB/s;
    - 40 MB of traffic, then 4 start/stop/download cycles; 113 KB internal stays free.
  - **SD load (measured):** FHD@25 at 8 Mbit/s writes ~1.0 MB/s, and the card is busy 30–48 % of the wall time
    (64 KB writes plus an fsync each second, ~2.9 MB/s effective through FAT; `video stats` reports
    `sd_busy_pct`). The 2 MB ring peaked at 0.4–0.9 MB. Headroom without pre-cooking the card; pre-cooking stays
    the operator's option for latency.
- **Camera preview in CC (operator, 2026-10-06; the pad's killer feature):** check where the camera points
  without touching the airframe.
  - Board: once a second, the latest captured frame is scaled to 640×480 by the P4's PPA (which can also turn
    the 90° rotation upright) and compressed by the hardware JPEG encoder, so the H.264 encoder takes no extra
    load and the flight core is untouched. `GET /camera/preview.jpg` on the file server serves it. With no
    recording running, the camera runs preview-only (capture on, H.264 off).
  - CC: the hub proxies the image; the dashboard's board panel shows it, refreshed at 1 Hz.
  - Gated to SETTING: the preview stops at BOOSTING (Wi-Fi is off from then on anyway).
- **Pad sequence** (the operator's; the gates fit it): (1) assemble glider and booster, plug the battery,
  close the cover; (2) CC check, time sync and probe; (3) if there is time, wait 5 min for GNSS; (4) place on
  the rod; (5) tie to the ignition system; (6) launch.
  - The board boots at (1). The ring_eraser runs in the background from then on, and the hub's clock sync
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
- **The operator's boot rules** (§4.6): any power-up after DONE is a new session in SETTING; a crumb in
  BOOSTING..LANDING means a warm start; no crumb and speed 0 means SETTING.
- The checkpoint writes the NVS crumb only when armed, `stage` i32 last, then `checkpoint.csv`, as plan B does
  (§7.2, Q11). The session-start record (§3.6, written from M3 on) lives in `cfg`.
- The supervisor's watchdog feeds the TWDT. With the current sdkconfig a TWDT timeout only prints a warning,
  so the supervisor first calls `esp_task_wdt_reconfigure` with `trigger_panic = true` and the timeout from
  `wdt_timeout_ms`, as plan B's `machine.WDT` does (the conservative one of the two fixes; the other is
  `CONFIG_ESP_TASK_WDT_PANIC=y`). If the control thread stalls past `stall_ms` while controlling, it writes an
  event and calls `esp_restart`.
- The P4 reset-reason mapping (§4.6).
- `degraded` gains `WARM-STARTED (rebooted in flight)`.

**Proof**
- Golden `warmstart.json`: `should_restore` over every crumb, cause and age boundary, plus a torn crumb, exact
  (step 1 of Q19). Q19's step 2 comes after M8's HITL comparison. When it drops the age and power-on refusals,
  plan B's verdicts for those cases become listed divergences; the stage rules and the separation latch stay exact.
- On the taster: arm, pass through BOOSTING (M7's hand-shaken launch), then hold `stage gliding` with the
  separation latch reading separated; console `panic` then returns warm into GLIDING with the same pad, and
  `gettimeofday` has survived. (Plan B writes the crumb only while armed and freezes the pad only on entry to
  BOOSTING, so a disarmed hold straight from SETTING reboots cold.) The survival of `gettimeofday` across
  SW/panic/TWDT resets is itself measured here (§7, R7).
- On the taster: the same armed run carried on to DONE, then a console `panic`, boots a new session in SETTING.

**Operator gains.** A mid-air reset resumes the flight, and a reset after landing starts a fresh session. The
board can no longer hang silently.

### M9. SEN0697: BMI323, BMP581, BMM350, and the attitude filter (2 days)

**Scope**
- **BMI323** (2 dummy bytes per read, start-up marker wait, INT1 on GPIO4 on v1.x). Accel and rate at p1.
- **BMP581** (re-arm out of standby). Backup baro at p1, reconciled against the ICP.
- **BMM350** (OTP power-off is mandatory, NVS `bmm350_cal` 16 B blob, octant coverage).
- **Attitude complementary filter** (`control::attitude`) at 50 Hz in the control thread. It publishes
  attitude at p1, writes `attitude.csv` with centidegree integer columns, and drives `attitude-blind` and
  `attitude-backup`.
- **Axes:** the IMUs are mounted per their marks (x forward, y left, z up). The optional per-sensor `axes` key
  in the board config covers an exception (§7.2, Q7); it is a schema addition, and old files stay valid.
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

### M11. GNSS, gnss_calib, field (2 days)

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

**Operator gains.** Position in health and telemetry, the pad drift calibration, and CC-less field arming.

### M11b. DONE and the recovery beacon (1-2 days; before the first passive flight)

**Scope**
- **At DONE:**
  - the camera and the non-essential sensors stop (the IMUs, the pitot, the fins). The GNSS drops to 1 Hz, and
    the other sources the beacon page shows (the baros, the laser, the INA226) keep a slow rate;
  - logging drops to a trickle, and the ring_saver keeps emptying the flash to the SD while the board is powered
    (§3.9);
  - nothing reboots the board: `reboot` and every other change are SETTING-only (§6).
- **The beacon:** an open Wi-Fi AP `<BOARD>_glider` on channel 3 (operator, 2026-10-06), at the maximum TX power,
  Wi-Fi only (no BLE). The separate recorder's beacon, `<BOARD>_beacon` on channel 6 (M20), is the second one: two
  radios, two antennas, two batteries, so one buried or flat does not hide the glider. Its one HTTP page shows:
  - the last good fix and a Google Maps link to it;
  - the elevation from every source: GNSS, each baro, the laser AGL (how high the glider hangs, should it land
    in a tree);
  - the battery voltage and the remaining time, where an INA226 is fitted;
  - the flash→SD progress (`ring_unsaved_s` and the blocks left).
- Power-off or reset ends it. Any power-up after DONE is a new session in SETTING (§4.6), and every boot resumes
  the flash→SD copy when a card is present.

**Proof**
- On the taster at DONE: a phone sees `beacon_taster`, and the page shows the fix with a working Maps link, every
  elevation source, the battery figures and the flash→SD progress falling to 0. The camera has stopped and the
  GNSS rows come at 1 Hz.
- A power cycle after DONE comes back in SETTING as a new session, with the copy resumed.
- The battery time at DONE is measured on the INA226 (about 1.5-2 h expected on 800 mAh).

**Operator gains.** **Passive-flight ready.** This covers the flight-test ladder's second rung (passive
electronics E16/F15, no active fins): stages, data, video, warm start, pull, and a beacon that leads to the
airframe after landing (even without a fix, the phone's signal strength helps).

### M12. HITL over a UDP sim-link, realtime and passive (2 days)

**Scope**
- `doc/specs/sim-link.md`, `crates/sim_link`, `firmware/sim_link.rs`. It runs only with an enabled `hitl`
  component. It masks every sensor whose config **name** is in `_SIM_SENSORS` (`enabled = False`, as plan B's
  `config_hitl.mask`, again after layout), refuses to start if one is still enabled (new: plan B only masks),
  and publishes p0 snapshots with the timestamp of arrival.
- `tools/hitl_host.py`: `sim_model.Body`, `config_hitl` scenarios and `Faults`. A realtime accumulator mirrors
  `hitl.py:323-391`. It drives the board over CC (set-config board, reboot, arm) and prints the
  SESSION/STAGE/DONE lines that `hitl_collect.sh` parses. Both `set-config` and `reboot` are SETTING-only, so
  between scenarios the host first returns the board from DONE with `stage setting`.
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
- `set_angle` compare-and-set, the fin probe sweep as a control-thread state machine (SETTING, disarmed only),
  and `update <servo name> {trim|angle}` (SETTING-only, like every `update`).
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
| M15 | Firmware OTA over CC into the `ota_0`/`ota_1` slots that M3's partition table already has (esp_ota; SETTING-only and disarmed). Replaces `push-*` for the Rust image (§7.2, Q5). **No app rollback, ever** (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=n`, pinned): with rollback, a build that has not yet confirmed itself is reverted at the next reset, so a reboot in flight could come back on the OLD build. A reset always boots the slot otadata selects; the slot changes only by an OTA in SETTING followed by a reboot on the ground, and `arm` refuses when the running slot is not the selected one. USB flashing (`flash.sh`) writes ota_0, erases otadata and invalidates ota_1, so no stale build can be selected. |
| M16 | NEO-M8N UBX NAV-PVT (CFG-PRT baud-up, autobaud after a P4 reboot). |
| M17 | — (the post-landing recovery beacon moved forward to M11b, before the first passive flight). |
| M18 | Legacy parts as needed: ADXL375 (7C), BNO055, BMP280. |
| M21 | **Entry file: GCL at boot (operator, 2026-10-06; Q20).** `/sdcard/entry.gcl`: one GCL line per line, `#` comments. Once bring-up is done, each line runs in order through the executor (origin `entry`, the one point of injection), under the normal gates, so changes still run only in SETTING. Each line and its answer go to `recorder.log`. It replaces MicroPython's run-any-script flexibility for bench tests, soaks and HITL setup (for example `ring load seconds=600`, `video start`, `pace seconds=60`). Safety: while an entry file is present, `verify` and `arm` refuse with `entry-file`, so a bench mode never rides into a flight. Small; after M4. |
| M19 | **uart_saver: the remote recorder (operator, 2026-10-06).** The purpose: the data survives when the MCU and its SD card are both damaged. <ul><li>The ring_saver already decodes each block into wire lines (`[@routing@]{OPEN};payload;<CLOSE>`, binary → text + CRC) for the SD. It also hands the same lines to the uart_saver, which sends them out of a UART TX in its own time. One decode, two sinks.</li><li>These are the bytes plan B sent to the Luckfox, so the frozen recorderd is the receiver as it is. The Luckfox is the test bench now. Later it becomes a physically separate flight recorder: a C3/C6 with its own battery and SD card, which could also carry the DONE beacon.</li><li>The handoff never blocks the flash→SD path:<ul><li>a bounded PSRAM byte ring (SPSC: saver in, uart_saver out);</li><li>the saver pushes without waiting. When the ring is full, the oldest undelivered lines go (recency first: the last seconds before an impact matter most), counted;</li><li>the uart_saver runs its own thread below the saver, and shares no lock with it.</li></ul></li><li>Per block, the saver hands the lines to the uart_saver before its SD write, so an SD stall does not delay what is already decoded.</li><li>With no card, the saver still walks and decodes for the uart_saver. It uses a cursor of its own: `saver_position` keeps meaning "on the SD and fsynced", so the eraser never takes a block as saved because the UART had it.</li><li>Lag: the uart_saver trails the saver, not the writer. The saver keeps up by a wide margin (≈ 26 KB/s of text against an SD that takes MB/s), so normally the lag is a block or two.</li><li>Resends after a gap or a reboot are duplicates the tools drop (§3.8).</li><li>Hardware: its own battery, and only TX + GND to the main board, so a short or brown-out there does not take it down. Mounted apart from the main board.</li><li>Link sizing: plan B's link (UART1 TX GPIO20, 921600 baud) carries ≈ 92 KB/s, against ≈ 35 KB/s of routed text at the v0.1 flight rate (§3.13): about 38 % busy, so normally nothing is dropped. 1 kHz IMUs (about 3×) need ≈ 2 Mbaud.</li><li>Open: the UART pins on the v1.x boards.</li></ul> |
| M20 | **The separate flight recorder and its beacon (operator, 2026-10-06).** The operator builds a C6 + battery + SD card box; the receiver firmware is Rust (esp-idf on the C6, riscv32imac) and reuses `recorder_wire` (line verify, routing). It plugs in where the Luckfox sits now, and the boxes can be swapped. <ul><li>Receiver: reads the uart_saver's lines (M19), verifies each line's CRC, and writes the session layout that recorderd writes (§3.10) to its own SD card.</li><li>Beacon by data, not by protocol: the main board adds an ordinary recorder stream, `beacon.csv`. Its rows are what a finder needs, about once a second and on every change: the last good fix (latitude, longitude, GNSS altitude, fix age), every elevation source (each baro, laser AGL), the stage, and the battery voltage where an INA226 is fitted. The P4 keeps writing it through the flight and just before.</li><li>The receiver knows that one stream by its routing (`beacon.csv`) and keeps the latest row. When it beacons, it serves that position: the same page as M11b (last fix, a Google Maps link, the elevation sources) from an open AP `<BOARD>_beacon` on channel 6, at maximum TX power.</li><li>The P4 controls it (operator), all through `beacon.csv` lines:<ul><li>when it starts;</li><li>the network name: `<BOARD>_beacon`, beside the P4's own `<BOARD>_glider`;</li><li>the channel: low channels, 6 for the recorder and 3 for the P4, so the two never share one;</li><li>the page data: position, elevation from every source, stage, battery.</li></ul></li><li>It starts BEFORE launch: once the glider is installed on the rod, the P4 turns on the camera and the recorder's beacon together (the rod trigger, M9; until M9, `arm`). A P4 that dies in flight no longer matters, because no trigger is owed after launch.</li><li>Once on, it stays on until power-off. The on state and the parameters are kept in the recorder's NVS, so a recorder that reboots (an impact, a brown-out) beacons again at once, under the same name and channel. Only the P4's next session in SETTING turns it off.</li><li>The page shows the latest `beacon.csv` row: the pad position before launch, the last fix after.</li><li>Battery (ESP32-C6 datasheet v1.5, Tables 5-7 and 5-10):<ul><li>A SoftAP keeps its receiver on, so it cannot modem-sleep. Wi-Fi RX is 78 mA at any CPU clock (80 MHz is the floor for the Wi-Fi PLL). The beacon frames add about 4 mA (a ~1 ms frame at 1 Mbps every 102 ms, 354 mA at 21 dBm), so the AP draws ≈ 80-85 mA.</li><li>The modem-sleep figures (14-38 mA at 80-160 MHz) hold only with Wi-Fi clock-gated: a station between DTIMs, or the recording phase before the beacon starts.</li><li>A 120 mAh LiPo (about 85 % usable) gives ≈ 1.2 h of continuous AP, short of the 1.5-2 h search target.</li><li>Chosen (operator): Wi-Fi 1 minute on, 1 minute off. On average ≈ 41 mA plus the off-minute's draw: about 0.2 mA in light sleep, or 14 mA with the CPU idle in modem sleep. A 120 mAh cell then lasts ≈ 2.1-2.4 h, against ≈ 1.2 h always-on. The AP stays on while a phone is connected, and a searcher waits at most a minute to see it. That fits how a search goes (operator): you walk to the approximate location and circle the trees nearby, so sooner or later an on-minute is caught. The parts are more often in the grass or stuck in a tree, which is why the page shows every elevation source beside the position. A 300 mAh cell remains the option for longer searches.</li><li>The beacon frames add little either way: the receiver is the cost.</li></ul></li><li>The same `beacon.csv` rows can feed the P4's own beacon at DONE (M11b), so both beacons show one data set.</li><li>Two beacons by design: two radios, two antennas, two batteries, two mounting spots. One that is buried, shadowed or flat does not hide the glider. Both run at once from DONE (the recorder's has been on since the rod). Each has its own SSID and channel (`<BOARD>_glider` on 3, `<BOARD>_beacon` on 6), so a phone sees which one answers and the signal strength of each, which helps walking in on the glider.</li></ul> |

---

## 3. Storage design

The operator's ring (decided 2026-10-06). Power can drop at any moment, so getting every record into flash fast
is the critical path; the SD copy comes second.

### 3.1 Data path

```
producers (any core)                          core 0, prio 7          core 0, prio 4
 control / i2c0 / i2c1 / gnss / supervisor      ┌─────────────┐ chunks  ┌────────────┐ text + CRC
 / executor  ── binary records ──► PSRAM SPSC ─►│ ring_writer │───────► │ ring_saver │───────────► /sdcard/recordings/
 log bridge  ── (one ring per thread)           │ (programs)  │ 24 MiB  │  (reads)   │
                                                └──┬──────────┘ `log`   └─────┬──────┘
                                       CC tees ◄───┘            ring ▲        │ saver_position
                                                                     │  ┌─────▼───────┐
                                                               erase └──┤ ring_eraser │ core 0, prio 3
                                                                        └─────────────┘
video: encoder (prio 10) → 2 MB PSRAM ring → SD writer (prio 6) → /sdcard/video/<session>/   (independent)
```

Three tasks share the `log` partition, each in one role:
- the **ring_writer** appends binary records, verbatim (no text), into the current flash block as soon as it
  can while the block has space (§3.3);
- the **ring_saver** decodes the oldest unsaved block to text with the CRC wrapper and writes it to the SD,
  below the video writer (§3.9);
- the **ring_eraser** erases blocks behind the saver, and when the ring is full the oldest block even unsaved
  (§3.7).

Invariants:
1. **One role per resource.** One writer per SPSC ring end. Only the ring_writer programs the `log` partition,
   only the ring_eraser erases it, and the ring_saver only reads it; the saver owns every SD recording file. The
   three coordinate through two published positions (`writer_position`, `saver_position`: each two AtomicU32
   under a seqlock) and the block table's atomics (§3.6). The flash driver serialises the operations themselves.
2. **A record is always in PSRAM, in flash, or on the SD** (or in two of them). It leaves PSRAM only after its
   program call returns, and its block is erased only after the saver has written and fsynced it. The one
   exception is newest wins (§3.7): above ~99 % use the oldest unsaved block is erased, and the loss is logged.
   Everywhere else the operator's rule holds: "better to have duplicates in both than lost data" (§3.8).
3. **No mutex on any producer path.**
4. **Text is made only off the flight path.** Producers and the writer move binary records; the saver (and the
   `cc` executor, for the tees) turns them into text.

The whole 24 MiB flash ring serves logging. The eraser runs in the background, so boot is never blocked, and
PSRAM holds the records while a block erases.

### 3.2 Producer side: `Telemetry` and the PSRAM rings

**Stream registry.** The registry is built during bring-up, before any thread spawns, and is then immutable.
Each entry holds:
- `id: u16`;
- the destination file name: the stream file without the session prefix (`imu_lsm6dso32.csv`), or the bare
  `session.csv`;
- the header field list;
- `decimate_us`. A stream's own value wins when it is non-zero; 0 means use the global
  `recorder.telemetry_ms*1000`, read at use; a negative value means never decimate (`<task>_events.csv`, -1).

**`Telemetry`** is owned by its producer:

```rust
pub struct Telemetry { stream: u16, decimate_us: i32, last_us: u64, header_sent: bool }
fn due(&self, now_us: u64) -> bool                        // false while the recorder is not running
fn push(&mut self, now_us: u64, row: &[Value]) -> Result<(), Overflow>  // telemetry raises (policy)
```

- The first push emits a HEADER record before the row, and always emits, whatever the decimation. The HEADER
  record carries the column list itself, so every flash block reads without the registry, even under a later
  firmware.
- **HEADER records are repeated:** the writer appends the HEADER record of every registered stream this boot has
  started again about once per MiB of flash written (the registry keeps their bytes, so this is a copy, not
  formatting). The MiB is counted by the writer position, block headers and unused block tails included, so a
  reader that starts anywhere meets every header within the next MiB. A stream that never pushed is not repeated,
  so it gets no file. A session whose first blocks newest wins erased unsaved still gets its headers within the
  next MiB; the saver writes a header line only when a file starts or its columns change, so the repeats never
  reach the SD.
- `Recorder::log(desc, msg) -> bool` is best effort and drops with a counter.

**One SPSC byte ring per producing thread**, in PSRAM (`heap_caps` SPIRAM):
- the producer owns `head`, the writer owns `tail`, both `AtomicUsize` with Release/Acquire;
- sizes: control 1 MiB, i2c0 1 MiB, i2c1 512 KiB, gnss 256 KiB, supervisor 256 KiB, executor 128 KiB, plus one
  `misc` ring of 256 KiB;
- `misc` is guarded by a std `Mutex` with `try_lock` (drop and count on contention). It is used only by
  unregistered threads and IDF's vprintf hook, never on a hot path.

The config keys `recorder.tlm_capacity`, `log_capacity` and `cell_size` stay accepted and act as lower bounds.

**Record format** (little-endian, 4-byte aligned). The same bytes sit in the PSRAM ring and, verbatim, in a flash
chunk (§3.4):

| off | size | field |
|---|---|---|
| 0 | u16 | `len` = total record bytes including this header and the name, a multiple of 4. `0` = the PSRAM ring's wrap marker (the consumer jumps to offset 0); never in flash. |
| 2 | u8 | `kind`: 1 HEADER, 2 ROW, 3 LOG |
| 3 | u8 | `flags`: bit0 durable (the writer programs it at once, §3.3) |
| 4 | u8 | `name_len` = bytes of the destination file name |
| 5 | u8 | `count` = number of cells (ROW), else 0 |
| 6 | u16 | reserved 0 |
| 8 | u64 | `uptime_us` (esp_timer, unwrapped) |
| 16 | … | The destination file name (`[A-Za-z0-9._-]{3,64}`: `imu_lsm6dso32.csv`, `session.csv`, and `recorder.log` for every LOG), then the body. HEADER: the column list text, so the record reads as `file@columns`. ROW: cells, each `tag u8` plus payload: 0 None (prints `None`), 1 I32, 2 I64, 3 F32 (prints with Rust `Display`: shortest round-trip, never an exponent; `NaN`/`inf` allowed), 4 Bool (prints `0`/`1`), 5 E7 (i32 printed as `d.ddddddd` degrees), 6 Text (`u8 len` plus bytes; never `;`, `\n` or control bytes). LOG: `u8 desc_len`, desc, message (≤ 1 KiB, CR/LF replaced by spaces). |

There is no preformatted kind: a session.csv row is a ROW to `session.csv` with Text and integer cells.

Nobody on the flight path formats text (invariant 4). That keeps `core::fmt` (53 µs per 20-field float row) off
core 1 and off the flash path.

Durable streams: `sequencer`, `separation`, `checkpoint`, `<task>_events`, session.csv rows.

### 3.3 The ring_writer

**Placement.** One thread on core 0 at priority 7: above the video SD writer (6), below the encoder (10).
- IDF flash programming busy-waits: L1 measured core 0 at 98.5 % when writing flat out.
- At the real rate (about 16 KB/s in flash, §3.13: some 160 B per 10 ms tick, one program of ~0.4 ms each, G1)
  that is about 4 % of core 0.
- L1 measured the opposite priority order (writer at 5) with 0 dropped frames. At 7 the writer can delay the
  video writer by at most one chunk program: ≤ 1.5 ms for a tick's chunk (G1), about 10 ms (estimated) for a
  full 4 KiB chunk after a backlog. The 2 MB video ring absorbs about 2 s. R3 confirms this (§7, R2).

**Loop:**
1. Wait for the ~10 ms tick, or for the notify of a producer that pushed a durable record.
2. Drain the rings: telemetry rings first, then log records merged by `uptime_us` across rings. A log record is
   taken only once it is older than a 20 ms merge horizon, so recorder.log stays in uptime order.
3. Copy whole records, verbatim, into one chunk while they fit in the open block (§3.4). A record never
   straddles blocks.
4. Copy each record whose CC tee window is open into that tee (§3.12). The writer never formats text.
5. **Program** the chunk (header and records) with **one** `esp_partition_write_raw` at its 8-aligned offset,
   header first in address order. G1 proved that many small programs into one page are safe (§7, R1).
6. Release the records from their rings (invariant 2) and publish `writer_position = (block_sequence, offset)`.
7. A record that does not fit closes the block. The writer takes the next erased block (§3.5), checks it is
   fully erased (one 4 KiB read, ~0.4 ms), programs its header and goes on. A slot that fails the check, dirty or
   unreadable, goes to the eraser as `FOREIGN`, and the writer takes the next erased one.
   - With no erased block, records stay in PSRAM until the eraser frees one. Once a producer ring is full,
     `Telemetry::push` returns `Err` (counted in `tlm.dropped`) and logs drop with a count.
   - A failed program is retried at the next offset. If the failed call had landed whole, the chunk is in flash
     twice: a duplicate (§3.8).

**Power-cut bound** for rows: ≤ 10 ms (the tick) + ≤ 17 ms (an eraser's 4 KiB erase in progress) + ≤ 16 ms
(an NVS page erase holding the flash lock) + ≤ 2 ms (opening a block and programming the chunk, G1). That is
about 11 ms typical and ≤ ~45 ms worst case. Log records wait up to 20 ms more for the merge horizon (step 2),
so a log line's worst case is ~65 ms. A durable record skips the tick: ≤ ~35 ms.

### 3.4 Block and chunk format (`flash_ring`)

A block is one 4 KiB erase sector: a 56-byte header at offset 0, then appended chunks, then the erased (0xFF)
tail. A partly filled block is valid; its FF tail is unused space.

**Block header** (little-endian), programmed once, when the writer opens the block:

| off | type | field |
|---|---|---|
| 0 | [u8;4] | magic `F7 43 42 31` (`"\xF7CB1"`; 0xF7 never occurs in UTF-8, and the magic is neither all-0 nor all-1) |
| 4 | u8 | version = 1 |
| 5 | u8 | flags: bit0 flight (opened in BOOSTING..DONE) |
| 6 | u16 | reserved 0 |
| 8 | u32 | block_sequence (global: +1 on every block opened, across boots) |
| 12 | u32 | boot_id (0 = uncounted) |
| 16 | [u8;32] | session label (§3.11), ASCII, zero-padded |
| 48 | u32 | reserved 0 |
| 52 | u32 | header_crc over bytes 0..52 (zlib CRC-32; `esp_rom_crc32_le` matches) |

The 56 header bytes and the 4 KiB slot grid never change, in any build: they let a build order, keep and give up
another build's blocks (after an update, or a reboot in flight that comes back on an older image). Everything
behind the header is versioned: the version covers the chunk format and the record format (§3.2), any change to
either bumps it, and byte-exact test vectors on both sides pin it. A build keeps a block of another version, never
saves it, and gives it up only under newest wins, flight blocks last (§3.7). The host side is where both layouts
meet: `ring_split.py` reports a block of a version it does not read from its header (boot, session, sequence,
flight flag) and files nothing of it, and a new version's decoder goes in beside the old one, never in its place,
so the newest tool reads a ring that a revert left in two layouts.

**Chunk**, at an 8-aligned offset; the next starts at `align8(offset + 8 + len)`:

| off | type | field |
|---|---|---|
| 0 | u16 | len = payload bytes; `0xFFFF` = the erased tail, no chunk here |
| 2 | u16 | record count |
| 4 | u32 | crc over bytes 0..4 and the payload (zlib CRC-32) |
| 8 | … | payload: whole records (§3.2), verbatim |

- A chunk whose CRC fails, or whose length runs past the block end, is **torn**. A cut tears only the last chunk
  of its boot's last block, because a new boot never appends to an old block (§3.5). A failed program leaves
  debris anywhere, and the writer retries its records at the next offset, or in the boot's next block when they no
  longer fit (§3.3, step 7): the walk steps over the debris in 8-byte steps to the next whole chunk. So a torn
  chunk's records count as lost only when no whole chunk follows it in its block and no later block of its boot
  is in flash.
- A record never straddles chunks or blocks, so every valid chunk decodes on its own.
- One boot's blocks, decoded and wrapped, have the old UART stream's format and make a valid `flight_telemetry`
  capture (the numbers are not byte-identical to MicroPython's: f32 `Display`, u64 uptime). A raw ring image is
  not one: it holds binary blocks of many boots, so it goes through `ring_split.py` first (M3).

### 3.5 Ring geometry and block life cycle

- **Partition:** `log`, found by label: 0x800000–0x2000000, 24 MiB (§3.10). It is accessed only through
  `esp_partition_read`/`write_raw`/`erase_range`, and never mmap while writes run.
- **Blocks:** 6144 × 4 KiB, one erase sector each.
- **Sequence:** a global `block_sequence: u32` increases on every block open, across boots, and blocks are read
  in its order. The writer takes the next erased slot after its current one, so the ring runs as a FIFO while
  the saver keeps up. After a newest-wins erase out of order (§3.7), slot order and sequence order can differ;
  the block table (§3.6) maps one to the other.
- **States:**

  | state | meaning |
  |---|---|
  | `ERASED` | all 0xFF |
  | `OPEN` | the writer's current block |
  | `UNSAVED` | closed and valid, not yet on the SD |
  | `SAVED` | behind `saver_position`; the eraser erases it 100–200 ms later |
  | `FOREIGN` | an offset-0 header that is neither valid nor all 0xFF |

- **A new boot never appends to a previous boot's block.** It opens an erased block with
  `block_sequence = max + 1`.
- **Flight blocks:** the writer closes its block when the stage enters BOOSTING, so every record from BOOSTING
  on lies in a block with the flight flag. A reboot in flight brings storage up (§4.5, step 3) before the warm
  start decides (step 8), so the writer opens its first block with the flight flag when the crumb names
  BOOSTING..LANDING; a refused warm start closes that one block. The fall at DONE (or a refused warm start) closes
  the block only once every record queued before it is programmed: one completed writer step past the 20 ms merge
  horizon, with no chunk waiting. Applied at once, the last LANDING rows and log lines, drained after the call,
  would lie in a pad block, which newest wins gives up first.
- **Crash consistency of erase:** a cut during an erase leaves a block that reads as `FOREIGN`, or as a valid
  header over damaged chunks. Either is erased again; a damaged chunk fails its CRC and never decodes as a wrong
  record.
- **Foreign blocks** hold nothing recoverable (MicroPython-era VFS data, the `flashwrite` and `ring` bench
  patterns, a cut erase) and are free to erase.

### 3.6 Bring-up: the scan, the positions and the session-start record

One bring-up routine (`recorder.rs`) sets up all three tasks. It runs in the ring_writer thread before the first
append, so the rest of boot goes on and the producers queue in PSRAM meanwhile:
1. Read the 56-byte header at each of the 6144 block starts (about 0.15 s, estimated; measured in M3). Classify
   each slot as valid `(block_sequence, boot_id, session, flight)`, `ERASED` or `FOREIGN`, into the **block
   table** (6144 entries in PSRAM) that the three tasks share through atomics.
2. **No valid block anywhere** means an unformatted ring:
   - `degraded: ring-unformatted`;
   - the eraser erases the foreign blocks ahead of the writer (~15 ms each, about 90 s for 24 MiB);
   - the writer starts as soon as one block is erased.
3. **Writer:** the first erased block after the newest valid one, with `block_sequence = max + 1`.
4. **Saver:** at the `saver_position` hint when it names a valid block and the mounted card is the one the
   previous boot's session-start record names (FAT volume serial); otherwise at the oldest valid block, so a new
   card gets everything still in flash. Saved blocks are erased within 200 ms, so a stale or lost hint costs at
   most about a second of re-saved records: duplicates (§3.8), never lost data.
5. **Eraser:** the saved and foreign blocks first, then its normal rule (§3.7). The blocks behind the
   `saver_position` hint count as saved when the mounted card is the one the hint was saved to, or no card is
   mounted; with another card nothing does, so the new card gets everything.
6. **Session-start record:** written once per boot to the `cfg` NVS: boot id, session label, config_id, reset
   cause, card (VSN or none), and the ring positions this boot started from. It is what a fast start needs.

**Positions:** `writer_position` and `saver_position` (each `(block_sequence, offset)`) live in the `cfg` NVS as
hints, written about once a second. The ring_eraser writes the writer's hint between erase steps, never the
ring_writer itself: IDF holds one global NVS lock for every call, and a config save holds it for a whole blob write,
which would stall the writer. The scan wins over the writer's hint; the saver's hint is its only record of
progress, and without one it starts at the oldest valid block. A hint more than 2³¹ blocks ahead of the newest
block in flash is ignored: it is corrupt, or another build's encoding. `block_sequence = u32::MAX` is never given,
so a ring numbered up to it (only a corrupt hint or another build gets there; 2³² blocks are 16 TiB) refuses to
open a block rather than give a number twice.

### 3.7 The ring_eraser and newest wins

- **Thread:** core 0, priority 3, the only task that erases the `log` partition. It erases one 4 KiB block per
  call (13-17 ms); a 64 KiB erase is never issued.
- **Normal rule:** erase each block 100–200 ms after the saver has passed it (`block_sequence` behind
  `saver_position`), and keep about 1 % of the ring (~61 blocks, ~245 KB, about 15 s at the flight rate) erased
  ahead of the writer.
- **Newest wins:** above ~99 % use (no more than ~1 % of the blocks erased or saved: the reserve, plus one block
  of margin for the block the writer opens before the eraser's next step, so `ring-low` and the gate do not
  flicker), the eraser erases the **oldest** block even unsaved: non-flight blocks first, flight blocks only when
  no non-flight block is left.
  Each such erase is logged as a loss, `<uptime> ring :: block <sequence> (boot <id>, session <s>, flight
  <0|1>) erased unsaved`, and counted in `ring.lost`.
  - With no card, or a dead one, the ring therefore keeps the newest ~24 MiB, about 26 min at the flight rate
    (§3.13), and the flight's blocks outlive the pad's.
- **No flight pause:** the eraser runs on the pad and in flight alike. One erase holds the flash at most 17 ms,
  which the §3.3 bound includes.
- **A failing erase:** an erase that fails 3 times in a row on one slot gives the slot up until the next boot
  (counted in `ring.bad`). The eraser passes over it and takes the next block by the same rules, so one bad
  sector never stops the ring.
- **After DONE** the logging drops to a trickle (M11b), so the saver catches up and the eraser follows it.
- **Budget figures:**

  | figure | meaning |
  |---|---|
  | `ring_erased_s` | erased plus saved blocks × 4 KiB / measured write rate: the time before newest wins |
  | `ring_unsaved_s` | unsaved blocks by the same measure: the flash→SD backlog |
  | `arm` / `verify` gate | the erased reserve (~1 % ahead of the writer) is held; no card warns (`no-sd`), never refuses (§7.2, Q10) |

- **Degraded tokens:**

  | token | when |
  |---|---|
  | `ring-low` | the erased reserve ahead of the writer has fallen under ~1 % (the eraser is not keeping up) |
  | `ring-full` | newest wins is erasing unsaved blocks |
  | `saver-behind` | SD mounted and `ring_unsaved_s` growing for more than 60 s |
  | `ring-unformatted` | the foreign blocks of a first boot are being erased |

- Endurance is not an issue: 100k cycles × 24 MiB is about 5 years at a continuous 16 KB/s.

### 3.8 Duplicates

Duplicates between flash and the SD are fine (the operator's rule). They arise when:
- the saver starts again from a stale `saver_position` hint, or on a new card (§3.6);
- a program fails after its chunk has landed, and the writer programs it again (§3.3);
- a `/ring` or image pull overlaps a session already on the SD.

The board never removes them; the tools drop them. `tools/recorder_wire.py` keeps the first line of each
(stream, uptime, line CRC) key (`unique()`, M3), and `recorder_flight`, `assemble_capture` and `flight_telemetry`
read through it. Two different lines with the same uptime differ in their CRC, so both stay. The uptime is the
line's leading decimal, and the stream (its routing, or recorder.log) stays in the key even though the CRC covers
it, so a CRC collision across two streams drops nothing. Only a stamped line that verifies is keyed:
- a damaged line has no CRC to prove it a duplicate, and stays damage;
- a line with no uptime of its own, a stream's header, is never a duplicate. A stream's second header is what
  `flight_telemetry` flags as two boots appended into one file (a `recorder.session` label reused across boots),
  and nothing in the header line tells a re-printed one from a new boot's. So the saver must not print a header
  again into a file that already holds one, after a restart too (M4): a fresh decoder that re-saves from
  `saver_position` reaches the next header repeat, and a header line there would read as a splice.

`ring_split.py` writes a session as the saver would, duplicates included, and counts them in its summary; the
readers drop them.

### 3.9 The ring_saver

**Thread:** core 0, priority 4, below the video writer (6) and the encoder (10): on the SD, video is priority 1
and logging priority 2, so the saver yields to the video writer. It reads blocks in `block_sequence` order with
`esp_partition_read`, one 4 KiB read per block, never past the writer's published `writer_position`. Every boot
resumes the copy as soon as a card is present, and after DONE the saver keeps emptying the flash to the SD for as
long as the board is powered.

**Decode:**
- Check the block header. It names the session (the directory) and the boot.
- Walk the chunks and check each CRC. The walk steps over a torn chunk to the next whole one (§3.4).
- Turn each record into its line with `recorder_wire`: a ROW prints `<uptime>;v1;…;vn`, a HEADER its column
  list, a LOG `<uptime_us> <desc> :: <msg>` (§3.11). Wrap it with its routing, `<session>_<file>` (the bare
  `session.csv` for the index): `{OPEN};<payload>;<CLOSE>\n` (`recorder-wire.md:25-50`; `{%08x};%s;<%08x>`,
  `tools/recorder_wire.py:75`). These are the bytes recorderd wrote.
- Route it as recorderd did (`recorderd.cpp:153-182`):
  - a stream line to `<session dir>/<session>_<file>`;
  - `session.csv` to **both** `<session dir>/session.csv` and the global `/sdcard/recordings/session.csv`;
  - a LOG line to `<session dir>/recorder.log`.
- A file name must match `[A-Za-z0-9._-]{3,64}` (no `/`); a record whose name does not goes to recorder.log.
- Every loss is logged into recorder.log as a full log line, `<uptime> ring :: block <sequence>: torn chunk at
  <offset>, k records lost` (a torn chunk is a loss only with no whole chunk behind it in its block and no later
  block of its boot in flash, §3.4; a retried program is not), wrapped with `recorder_wire.wrap` (as
  `assemble_capture` does for its own host lines). `<uptime>` is the newest uptime decoded in that session, so
  recorder.log never goes back in time, and `k` is `?` when the count never landed. An unwrapped line in a wrapped
  session counts as rejected damage in `recorder_flight` and `flight_telemetry`.

**Batches** are ≤ 1 s of blocks or 64 blocks (256 KiB), whichever comes first. **A batch never spans two
sessions** and touches at most 32 files. Order per batch:
1. Append each file's lines in writes of up to 32 KiB.
2. `fsync` every touched file.
3. Advance and publish `saver_position` past the batch. The eraser may erase those blocks 100–200 ms later.

There is no journal. A cut inside a batch re-saves it at the next boot from `saver_position`: duplicates (§3.8).
Before its first append to a file after boot, the saver ends a torn last line with `\n`, so the fragment stays a
line of its own (counted as damage by the tools) and never merges with a re-saved line.

**Handles:** the saver opens and closes files per batch, which is acceptable with per-session directories.
`max_files` goes to 40 for video, HTTP reads and the saver together, with `CONFIG_FATFS_FS_LOCK` at least as
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
```

A pulled session directory, with the global `session.csv` copied over the per-session one, is a drop-in
replacement for a Luckfox `/userdata/recordings` dump. `recorder_flight.py <dir> --session <s> -o <out>` and
`assemble_capture.py <s> <dir> <out.txt> [note]` use it unchanged, apart from the duplicate filter (§3.8).

Per-session directories avoid FatFs's linear `f_open` scan, and the FAT32 limit of 65 536 entries per
directory that a flat layout reaches after about 1 400 boots.

File names are long (FatFs LFN, already on in `sdkconfig.defaults`), because the names carry the telemetry
routing. 32 GB cards are the safe size. A 64 GB card can be one FAT32 volume (`mkfs.vfat -F 32`); no split is
needed.

**Card preparation** (the operator's option): `tools/sd_prepare.sh` discards the whole card, then writes one FAT32
partition over 25–50 % of it and leaves the rest untouched, as spare area for the card's controller. The firmware
mounts the first partition and never formats a card.

**Flash partitions** (`partitions.csv`, decided 2026-10-06):

| partition | type | offset | size | holds |
|---|---|---|---|---|
| `nvs` | nvs | 0x9000 | 0x6000 | unchanged, shared with MicroPython: boot count, calibrations, crumb |
| `phy_init` | phy | 0xF000 | 0x1000 | |
| `otadata` | ota | 0x10000 | 0x2000 | which app slot boots |
| `ota_0` | app | 0x20000 | 0x300000 | 3 MB app slot (the image is 1.6 MB today) |
| `ota_1` | app | 0x320000 | 0x300000 | 3 MB app slot, for OTA (M15) |
| `cfg` | nvs | 0x620000 | 0x100000 | 1 MB: blobs `board` and `launch`, the `creds` namespace, the session-start record, the ring positions |
| `log` | data | 0x800000 | 0x1800000 | unchanged: the 24 MiB ring |

- `flash.sh` writes the app to `ota_0` and erases `otadata`, so the bootloader boots `ota_0`. OTA itself comes
  later, over CC (M15).
- `tools/provision.sh` reads the `cfg` row from `partitions.csv`, so it follows the move. A board flashed with the
  old table (the taster) is re-provisioned once.
- 0x720000–0x800000 stays free.

### 3.11 Row, log and index text (byte rules)

The saver prints records by these rules, and so does the `cc` executor for the tees.

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
    after Recorder start). A HEADER record goes into flash again before the boot and anchor rows, so a reader that
    starts at either finds the columns; the saver prints it only where the file starts (§3.2).
  - `utc` and `utc_offset` are empty before 2001-01-01, and `utc_offset` is empty outside −720..+840.
  - Text cells are emptied unless printable ASCII, without `;`, and at most 32 characters (`recorder.py:76-110,602-697`).
- **Session prefix**, fixed at boot and carried in every block header:
  1. a valid `recorder.session` label (ASCII letters, digits and `-`, at least one letter, at most 32
     characters);
  2. else `%06u` of NVS `coludo/boot`;
  3. else `YYYYMMDD_HHMMSS_<6 random digits>`.

### 3.12 CC tees and the HTTP exits

- **Tees:** `Tee { deadline_ms: AtomicU32, ring: SPSC (writer → executor, 256 KiB in PSRAM), dropped: AtomicU32 }`,
  one for logs and one for telemetry. The deadline is a wrapping u32 in uptime milliseconds (0 = closed), as
  M1's log tee already does: the P4 has no 64-bit atomics (§4). While a window is open, the writer copies each
  matching record into the tee, verbatim; the `cc` executor formats the records when it drains.
  - `log [ms]` / `tlm [ms]` drain what was buffered since the last call and re-arm the window for `ms` more
    (default 1000; 0 or less stops it). A lapsed window is discarded on the next tee.
  - Replies are `ok {"lines":[…],"dropped":n}` and `ok {"samples":["@<session>_<file>@<row>",…],"dropped":n}`,
    without the CRC wrapper. A full tee (256 KiB of records) prints as about 600 KB of text, about 0.9 MB in
    base64 with JSON quoting: under the hub's 2 MiB line limit.
- **`GET /sd/recordings/…`** exists (`files.rs`). The listing format is `<name>;<bytes>`.
- **`GET /ring[?boot=N]`** streams every valid block decoded to wire lines (`[@<routing>@]{OPEN};…;<CLOSE>`,
  the saver's text with its routing) in `block_sequence` order: a byte-exact wire capture.
- **`GET /ring/image`** streams the raw 24 MiB partition, as `esptool read_flash 0x800000 0x1800000` reads it.
- `tools/ring_split.py` turns either into session directories. This is the recovery path when the card failed
  in flight.

### 3.13 Budget (v0.1 sensor set at the true Rust rates)

| item | value |
|---|---|
| rows | ~375 rows/s. On the SD as text ≈ 26 KB/s (~68 B a line, unchanged). In flash as binary records ≈ 14.6 KB/s: about 2.4× denser than the 35 KB/s of routed text |
| chunk and block overhead | ≈ 1.2 KB/s at a 10 ms tick (an 8 B chunk header and its padding per chunk, a 56 B header per block): ≈ 16 KB/s in flash in all |
| ring holds | 24 MiB / 16 KB/s ≈ 26 min with no SD progress (≈ 9 min if the IMUs go to 1 kHz) |
| flash busy | program ≈ 4 %; erase ≈ 6 % to keep pace (16 KB/s ÷ 4 KiB ≈ 4 blocks/s × ~15 ms); saver reads < 1 % |
| core 0 | video 13 % + writer ≈ 4 % (program only) + eraser ≤ 6 % + saver ≈ 2 % (decode, format, SD writes) + link/health ≈ 1 % |
| after DONE | a trickle (GNSS at 1 Hz, the beacon's sources, health); the saver drains the backlog at SD speed |

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
| esp_hosted, Wi-Fi, lwIP | 0* | 18-23 | IDF; off from BOOSTING through LANDING (M7); the beacon AP at DONE (M11b) | — |
| `video encoder` | 0 | 10 | exists | camera |
| `supervisor` | 0 | 9 | TWDT feed, health 1 Hz, checkpoint (NVS), gnss_calib, field, Mission | 100 ms base tick |
| `ring_writer` | 0 | 7 | programs the `log` partition; ring consumers, tees | ~10 ms tick, or a durable record's notify |
| `video SD writer` | 0 | 6 | exists | — |
| `cc` executor, `link`, `console`, httpd | 0 | 5 | command table; Wi-Fi + hub socket; UART0; HTTP | blocking |
| `ring_saver` | 0 | 4 | reads the `log` partition; SD recording files | batches |
| `ring_eraser` | 0 | 3 | erases the `log` partition | the saver's progress and the erased reserve |
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
- a general call is followed by `rearm()` on **every** device registered on that port, found by membership,
  never by a hard-coded list (M5).

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
3. **Storage first:** SD mount (exists), recorder registry and rings, the ring_writer thread (the bring-up scan
   runs in it, writes the session-start record, then starts the ring_saver and the ring_eraser, §3.6), `log`
   bridge, provenance line `boot: board <id> | firmware <v> | config <id> <source> | reset_cause <n> |
   boot <id> | session <s>`.
4. Buses created on core 1 by their owner threads, which are spawned parked. `layout.resolve` by scan
   (100 kHz probes). Cache the post-layout `config_id`.
5. Mission (launch.config, `max_range_m` from the `field` component).
6. Device setup in config order (sensors, then components), with retries and diagnose. Register channels from
   `provides`. Then the alternatives pass.
7. Freeze the databoard topology and the inspector (`Box::leak` → `&'static`).
8. `warm.rs` restore, before any flight-relevant thread runs.
9. Unpark the bus owners. Spawn control (core 1) and the supervisor. Arm the gptimer. Reconfigure the TWDT to
   panic at `wdt_timeout_ms` and subscribe the supervisor to it.
10. Console, link and httpd (as today).

### 4.6 Warm-start, boot rules and reset classes

- **Boot rules** (the operator's, 2026-10-06): plan B's warm start as closely as possible, as a simple state
  machine on the crumb's stage. Boot looks at the situation; anything but BOOSTING..LANDING relaxes and stays where it
  is (most testing lives in SETTING):
  - no crumb (`stage` 0) or a crumb at SETTING: SETTING, cold and disarmed (Q16, Q17);
  - a crumb at DONE: a new session in SETTING, never a warm start;
  - a crumb in BOOSTING..LANDING: recover into that stage, and the normal detectors (separation, apogee, landing,
    the timeouts) walk it on from there.
- **Gate, in two steps (operator, 2026-10-06):** first exactly plan B's, because it is well tested; then the
  drift to the ideal Rust fit (Q19), measured against the first.
  - **Step 1, M8: plan B as it is** (`warmstart.py:132-176`):
    - the crumb has a stage and a stamp;
    - the reset is a reset class (below);
    - `0 ≤ now − stamp ≤ 600 s` via `gettimeofday`;
    - GLIDING and LANDING also need the separation latch read at setup.
    - The warmstart crate already matches it on golden vectors. M8 proves it in HITL against the plan B
      baselines and measures R7.
  - **Step 2, after M8's HITL comparison (Q19): the simple state machine.** Only physical evidence can refuse:
    - the crumb must carry a stage;
    - GLIDING and LANDING still need the separation latch (it is the latch no software can fake, so a nested glider
      never recovers into a glide).
  - Step 2 drops plan B's two time and cause checks:
    - the crumb age `0 ≤ now − stamp ≤ 600 s`: with no battery-backed RTC, a power cycle loses the time, so the boot
      cannot know whether a crumb is fresh, and makes no assumption;
    - the reset cause: a power-on is no longer "human hands, cold", because a battery bounce in flight is a
      power-on too.
  - A stale crumb from an interrupted flight therefore recovers. The state machine then walks it to DONE (a lying
    glider reaches DONE by stillness), and the next power-up after DONE is a new session.
- **Reset classes** (logged and kept in the session-start record; they gate the warm start in step 1 only): `SW`, `PANIC`, `INT_WDT`, `TASK_WDT`, `WDT` and the P4-only `CPU_LOCKUP` (an in-flight
  crash) count as resets. `POWERON`, `BROWNOUT`, `DEEPSLEEP`, the P4-only `USB`, `JTAG`, `PWR_GLITCH` and
  `EFUSE`, and unknown are cold.
  - The P4 never reports `EXT`. An EN/CHIP_PU pulse from DTR/RTS or esptool reads as `POWERON` (or `USB` on the
    USB-Serial-JTAG), a cold boot in both plan B and Rust.
  - On the P4, plan B's `HARD_RESET` means SW + PANIC only. Plan B's real path into the DONE + armed trap is
    `SW`, the CC `reboot` at DONE. Rust closes it twice: `reboot` is SETTING-only, and a crumb at DONE boots a
    new session (§7.2, Q3).
- **NVS:** the same keys in namespace `coludo`: `crumb` (JSON ≤ 512 B) and the `stage` i32 written last. A
  rejected gate clears the stage. The session-start record (technical data plus the ring positions, §3.6)
  lives in `cfg`.

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
| `commons.json` | `dwell_step`, `apogee_step`, `fin_deflection_limit`, `altitude_m` | `dwell_step`, `fin_deflection_limit` exact; `apogee_step` abs 1e-3 m; `altitude_m` abs 0.01 m (f32: a pressure ratio near 1 resolves to ~3.4 mm) | M7 |
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
| system | reboot (SETTING-only, `ok` with no payload, then a whoami from a new boot: another boot_id, or an uptime shorter than the time since the `ok`) | M1 |
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
- **Changes only in SETTING** (the operator's rule, 2026-10-06): `update`, `set-config`, `reset-config`,
  `calibrate <device>`, `probe`, `detect`, `bustune` and `reboot` answer `err unsafe` in every other stage, DONE
  included; reads, `stage` and `disarm` stay open (M1). Plan B leaves `update` open in flight and allows the
  rest at DONE. There is no reboot at DONE: the beacon and the flash→SD copy run until power-off (M11b).
- **Storage:** binary records in a flash ring that the board copies to its own SD (§3), instead of text lines
  over the UART to the Luckfox. With a full ring and no SD progress the newest data wins (§3.7). A line can
  reach the SD twice; the tools drop duplicates (§3.8, a filter added in M3).
- **DONE:** the camera and the non-essential sensors stop, the GNSS drops to 1 Hz, and the beacon AP comes up
  (M11b). Plan B has neither.
- **SD layout:** one directory per session instead of a flat directory (§3.10).
- **uptime:** u64 µs with no 2³⁰ wrap. `recorder_wire.fit` reduces to k = 0 (`recorder_wire.py:268-281`);
  `recorder_flight._clock_times` does not, so the conformance case covers a session longer than 2³⁰ µs whose
  anchor row is dated (M4).
- **f32 text:** `1.764`, not `1.7639999`. lat/lon text comes from i32 1e-7 deg (7 decimals). Byte-for-byte
  board-vs-host comparisons (`hitl_compare`, golden captures) must compare numerically.
- **Rust-only degraded tokens:**

  | kind | tokens |
  |---|---|
  | added | `c6-reboot-guard-missing`, `c6-link-parked`, `no-sd`, `ring-low`, `ring-full`, `ring-unformatted`, `saver-behind`, `not-flight-capable` (until M7) |
  | dropped | `memory-rescued` |
  | kept where the mechanism exists | `attitude-backup`, `attitude-blind`, `STAGE HELD`, `CONFIG FALLBACK -- …`, `needs-calibration`, `cc-less-fallback`, `WARM-STARTED (rebooted in flight)` |

- **Crash semantics:** a panic reboots and warm-starts, instead of marking a loop DOWN and flying on with dead
  data.
- **Wi-Fi in flight:** the radio is **off** from the detected ignition through LANDING and resumed at DONE for
  recovery, as the beacon AP (M7, M11b; an operator stage hold leaves it up). Plan B keeps the pad link up in
  flight and only stops new connections. This loses in-flight CC; the operator accepted that.
- **Stale values in inspect:** a stale channel reads null instead of plan B's extrapolated `value()`.
- **HITL sensor mask:** after masking the `_SIM_SENSORS` sensors by config name, `sim_link.rs` refuses to start
  if one is still enabled; plan B only masks (M12).
- **`_readiness` and `enabled`:** Rust reads a missing `enabled` as enabled everywhere, the Controller's
  meaning. Python's `_readiness` read it as disabled, an inconsistency that is not ported.
- **`config_id` floats:** the CPython f64-repr semantics are used. Identical for every committed file (all
  floats have ≤ 7 significant digits). Plan B's announced id for the same file is cross-checked once on 7C.
- **Mission `sites`** are kept on `set-config launch`. Plan B silently dropped them (`mission.py` `persisted()`).
- **ICP general-call re-arm** re-initializes every device registered on the bus, by membership, not by the
  stale name list (`icp10111.py:297`). Plan B gets the same patch, its one sanctioned change (M5, §7.2 Q9).
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
| R1 | Partial-page NOR programming (several program operations per 256 B page) on the GD `C84019`. | **Passed** by bench gate G1 (`127bc0d`): 10 000 frames of 40-200 B appended at 8-byte alignment, 0 corrupt; 32 programs of 8 B into each of 64 pages, 0 bytes differ. Chunks stay unpadded. |
| R2 | The loss bound (§3.3: ~11 ms typical, ≤ ~45 ms worst for rows) needs short flash-lock holds. NVS writes (checkpoint, ring positions, calibrations), saver reads, eraser erases and the writer share the esp_flash lock. The writer at priority 7 can briefly delay the video writer. B4 was 10 s without video. | R3: a 10 min soak at the real flight rate with FHD@25, the 1 kHz `pace`, and a chunk-to-chunk maximum ≤ 45 ms. The crumb stays in NVS (§7.2, Q11). |
| R3 | FatFs is not journaled. A power cut during FAT updates (saver or video) can damage the FAT itself, not just a file tail. | The ring keeps every block until the saver has fsynced it, and the saver re-saves from its position after a cut (duplicates, §3.8). `GET /ring` + `ring_split.py` recovery. Host fsck on bad cards. |
| R4 | The first Rust boot carries nothing from MicroPython's VFS (board.config, launch.config, `*.creds`). TMS-7F is sealed and Wi-Fi only. | `tools/provision.sh` writes the `cfg` partition (config, launch, creds) over USB with esptool, after `flash.sh` and **before** the seal. Never flash a sealed board without it. |
| R5 | esp_hosted 2.7.0 tasks run at priority 23, unpinned (plain `xTaskCreate`); the streaming-mode length assert is still open. | lwIP's TCP/IP task is pinned to core 0 already (M1). M1b pins esp_hosted by wrapping `g_hosted_osi_funcs._h_thread_create`, the hook `components/hosted_no_reboot` uses for `_h_restart_host`. From M7 the radio is actually off from BOOSTING through LANDING in Rust (a deliberate divergence, §6; plan B keeps the pad link up). |
| R6 | On 7C (v0.1 + SEN0697) a plan B ICP general call resets the BMI323, BMM350 and BMP581 without re-arming them. | Rust re-initializes every device registered on the bus after a general call (M5). Plan B gets the same patch (M5, §7.2 Q9). |
| R7 | `gettimeofday` surviving SW, panic and TWDT resets on the P4 is unverified, and step 1's age check (plan B's gate) depends on it. | Measured in M8. Q19's step 2 drops the age check, which closes it. |
| R8 | base64 strictness differs: CPython is lenient, Rust is strict. A bad token could answer differently from the fake. | Rust answers `err badargs` (strict). The contract runner records the fake's deviation as known. |
| R9 | Detector drift vs the host sim: `virtual_flight.py` grades the Python detectors once the board runs Rust. | `commons.json`, `sequencer_trace.json`, `control_trace_*.json` are the only guards. `generate.py --check` in CI. |
| R10 | HITL masking: a fresh bench sensor silently becomes the only source (pitot 7/27, mag 9/16). | `sim_link.rs` masks the `_SIM_SENSORS` sensors by config name (they are sensor names, not drivers), as plan B does, and re-applies the mask after layout. It then refuses to start if one is still enabled, which is new (plan B only masks). |
| R11 | Per-sensor axes: the LSM6DSO32 and BMI323 publish raw axes into one channel, and a failover may flip axes. gx/gy and the signs are unverified on any airframe. | The IMUs are mounted per their marks (x forward, y left, z up), and the optional `axes` key covers an exception (§7.2, Q7). M9 move-and-hold check. |
| R12 | Two physics implementations would repeat the sim-mirrors-flight-code trap. | `sim_model.py` stays the only physics. Rust never ports it. |
| R13 | Silent-success harness traps (1 of 12 flown reported done; empty streams passed). | Every new runner counts artefacts and fails loudly. |
| R14 | Port 0 is shared with the camera SCCB, and `i2c_master`'s internal mutex arbitrates. | Camera register traffic only pre-flight. Measure the bus-owner jitter with video on in M6. |
| R15 | One reply line must stay under the hub's 2 MiB limit (`server.py:697`). | Tee capped at 256 KiB of records (~0.9 MB in base64 when full, with JSON quoting). `get-config` ~10 KB. |

### 7.2 Decided

The operator's decisions on the earlier questions, one line each:
- **Q1. SD layout:** one directory per session with long file names (the names carry the telemetry routing);
  video at `/sdcard/video/<session>/<start uptime ms>.h264` (§3.10).
- **Q2. Changes in flight:** `update`, `set-config`, `reset-config`, `calibrate <device>`, `probe`, `detect`,
  `bustune` and `reboot` run only in SETTING; reads, `stage` and `disarm` stay open; no reboot at DONE. A
  deliberate divergence from plan B (§6).
- **Q3. Reset at DONE:** any power-up after DONE is a new session in SETTING, never a warm start; a crumb in
  BOOSTING..LANDING means a warm start; no crumb and speed 0 means SETTING (§4.6).
- **Q4. `config_id` before vs after layout:** keep plan B's split, which is plan B behaviour: `set-config`
  returns the pre-layout id, `whoami` announces the post-layout one, and the boot log line names both (M5).
- **Q5. Partitions and OTA:** `otadata` plus `ota_0` and `ota_1` of 3 MB each now, a 1 MB `cfg` at 0x620000,
  `nvs` and `log` unchanged (§3.10). OTA itself comes later, over CC (M15).
- **Q6. Watchdog timeout:** keep 5000 ms in board files (Python `validate` rejects < 4000) until Rust measures
  its own floor.
- **Q7. IMU axes:** the IMUs are mounted per their marks (x forward, y left, z up); an optional per-sensor `axes`
  key in the board config (a signed permutation such as `'-y+x+z'`, default identity) covers exceptions (M9).
- **Q8. f64 (D18):** f32 by default. f64 needs a clear written reason and only goes into absolutely critical
  spots: lat/lon at the JSON boundary and the config document's fidelity (its internal `serde_json` f64, for
  hash and bytes only, never computed with); the ICP-10111 polynomial only if the golden f32-vs-f64
  comparison (M6) proves f32 insufficient. Never f64 for plain values such as the MCU temperature.
- **Q9. Plan B ICP re-arm on 7C:** yes, patch plan B to re-initialize every device registered on the bus after
  the general call, as Rust does (M5). It is the one sanctioned plan B change.
- **Q10. Arm gate on ring space:** the eraser's reserve (~1 % erased ahead) as a constant, with no config key
  (YAGNI). A missing or dead card does not block arm: newest wins keeps the flight (operator, 2026-10-06).
- **Q11. The crumb:** stays in the shared `nvs` and is written as plan B writes it; a new session-start record
  (technical data plus the ring positions) goes to `cfg` (§3.6). Their page erases are inside the §3.3 bound.
- **Q12. Provisioning:** `tools/provision.sh` is approved as tooling. The `cfg` image is built in a temporary
  directory and never committed: it carries Wi-Fi credentials.
- **Q13. Video:** FHD@25 (FHD@30 is encoder-bound at about 26.6 fps on P4 rev v1.3), 8 Mbit/s, IDR every 1 s.
  Recording starts when the glider stands vertical on the rod (attitude, M9; arm until then) and stops at DONE
  (M11b). On the SD, video is priority 1 and logging priority 2.
- **Q14. Apogee dwell:** port plan B's reuse of `launch_ms` faithfully (`sequencer.py:364-366`), to stay
  comparable with the HITL baselines.
- **Q15. Post-landing beacon:** before the first passive Rust flight, as M11b: an open AP `<BOARD>_glider` on
  channel 3 at the maximum TX power, Wi-Fi only. The separate recorder's `<BOARD>_beacon` on channel 6 is the second
  (M20).
- **Q16. No crumb (operator, 2026-10-06):** the warm start reads what NVS holds (the crumb: stage, position and
  the rest). Nothing there means a cold boot in SETTING, whatever the speed: speed is not an input to `restore()`.
  A crumb in BOOSTING..LANDING means recover, and get the situation under control from there.
- **Q17. A crumb at SETTING (operator, 2026-10-06):** a cold boot, disarmed (reason OnThePad); only
  BOOSTING..LANDING warm-starts. This diverges from plan B, which restores SETTING re-armed with `warm_started` set;
  a reboot on the pad then needs a fresh `arm` (§6).
- **Q18. The hold hazard (operator, 2026-10-06):** kept as plan B. In plan B a pad bump, then `stage gliding` (a fin
  check), then `stage setting` launches on the first boosting sample after the hold, because the launch dwells are
  not reset; with the Rust Wi-Fi policy that launch also turns the radio off. The operator's call:
  - while the board is in SETTING, the pad is CC's and the operator's to watch and handle;
  - once out of SETTING the flight must go on, so the firmware adds no guard that could stop one.
  - The golden test `a_hold_hides_the_return_to_setting` pins the behaviour.

- **Q19. Warm-start gate (operator, 2026-10-06), in two steps (§4.6):**
  - first plan B's gate exactly, because it is well tested: M8, golden-exact, proved in HITL against the plan B
    baselines;
  - then the drift to the ideal Rust fit, a simple state machine on the crumb's stage. Boot makes no freshness
    assumption, because there is no RTC: plan B's crumb-age and power-on checks go, and only the separation latch
    can refuse a glide stage. It is measured against step 1.
- **Q20. Files and the console instead of scripts (operator, 2026-10-06):** MicroPython's flexibility (any script
  run on the board) is replaced by files on the filesystem and the console API (the Glider Command Language). An
  **entry file** carries GCL lines that run once the board is up, to drive the image outside the usual cycle (a
  bench test, a soak, a HITL setup). See M21.

### 7.3 Open for the operator

None.
