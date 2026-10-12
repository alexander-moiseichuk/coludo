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
  `bustune` and `reboot`. Reads, `stage` and `disarm` stay open, and there is no reboot at DONE (§6). From M7 the
  barometers' re-zero also runs at DONE, the recovery from a false launch (§6).
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
| `sequencer` | `controller.py` Stage, `tasks/sequencer.py`, `commons.dwell_step`/`apogee_step` | **exists** (M7; `Stage` since `83c02236`). One module per plan B module: `controller` (`Stage`, `#[repr(u8)]` ids 1..5 SETTING..DONE with names, `ground()` and `airborne()`; 0 is the NVS no-checkpoint sentinel, not a variant), `commons` (the leaky `dwell_step`, the IIR `apogee_step`) and `sequencer`, whose pure `StageMachine::step(now_ms, stage, &Readings, manual) -> Step` takes the stage the flight word holds (§4.1); the caller writes the step's advance into the flight word before the next step. M7 adds `Settings::from_config` (§6), the flight word with its apply rules, and the request slot (§4.3, step 4). A `Step` carries up to two sequencer.csv rows, the external move seen first and then the machine's own advance, because one tick can record both (a hold released past the flight timeout). The crate is `std`, for the `f32::sqrt` of the reason text, which `core` lacks on stable. Golden: `commons.json` (`dwell_step`, `apogee_step`) and `sequencer_trace.json`. |
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
| `devices.rs`, `sensors.rs` | Sequential bring-up: `DeviceEntry` table, setup retries, the alternatives pass, the static databoard topology; the parts over the drivers crate. |
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
| `tools/sd_prepare.sh` | The operator's card preparation: discard the card, two FAT32 partitions (the recordings, then the videos, half each) on 25–100 % of it (default 75), the rest untouched (§3.10). |
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

### M4. ring_saver, SD, HTTP pull (2 days; DONE 2026-10-06 on the taster)

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
- `tools/sd_prepare.sh`, the operator's card preparation (§3.10). The firmware never formats. (M4 itself mounted
  the first partition only; the two-partition mount closed it, below.)
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

**Results (2026-10-06, the taster):**
- Host: 234 crate tests, including a cut at every step of a batch, a dead card, and the real `recorder_flight`,
  `assemble_capture` and `flight_telemetry` on the card the saver writes. `src/control` tests 20/20.
- The saver lives in `crates/recorder` (`saver.rs`, `fs.rs`), not `flash_ring`: `flash_ring` cannot depend on the
  decoder and has no filesystem. It writes the NVS hint after every committed batch.
- Contract `--milestone M4`: 0 violations.
- Catch-up: the first boot on a new card saved 2342 blocks, 62 326 lines and 8.9 MB in about 40 s, with 0 errors.
- A session saved while FHD@25 recorded, pulled with `board_pull.sh`, parsed all 'good' through `assemble_capture` +
  `flight_telemetry`. `recorder_flight` needs an IMU stream, so it waits for M6.
- 7 EN resets under `ring load` plus video, each resumed on the same card: no hole, no duplicate in any numbered load
  run, and every session's last line within the reset estimate (−39…+23 ms, ±70–90 ms resolution).
- The internal DMA budget had to be fixed first (§4.7): the build as first written lost SD writes
  (`allocate_dma_buf`) and asserted in esp_hosted under video + saver + a Wi-Fi download.
- Open:
  - true power pulls (the hub's per-port cut is data-only: its switching is ganged);
  - one boot whose Wi-Fi associated but got no IP for minutes, not reproduced since.

**Closing M4: the decided layout (§3.10, session first on two partitions), built 2026-10-06, host-proven:**
- `crates/recorder` `layout`: the one place that maps (session, kind) to a path. The saver routes through it:
  `<session>/recordings/<session>_<file>`, `<session>/recordings/{session.csv,recorder.log}`, and the global index
  `sessions.csv` at the card's root; the firmware roots the saver at `/sdcard`. The camera writes
  `videos/<session>/`. A `recorder.session` label `videos`, `recordings` or `video` (any case) falls back to the boot
  id, with a warning, and an earlier boot's block under such a label is filed under `_<label>`.
- `components/sd_card`: one card init, its own disk I/O under one card lock, the table judged by the board
  (`partition_table.c`: two sound FAT partitions, else one volume; an unsound pair mounts partition 1 alone with no
  video; an unreadable sector 0 mounts nothing), partition 1 at `/sdcard` and partition 2 at `/sdcard/videos`,
  `max_files` 32 + 8, `CONFIG_VFS_MAX_COUNT` 12; the boot log names the mode, both serials and both volumes' sizes
  (read by FatFs drive: `vfs_fat` cannot find `/sdcard/videos` by its path), `sd_free_mb` the recordings volume's.
- `files.rs` lists `videos` in `/sd/`; `board_pull.sh` pulls `/sd/<session>/recordings/` and
  `/sd/videos/<session>/` and still reads an M4 card (a session in both places is refused); `sd_prepare.sh` writes
  `METRICS` + `VIDEOS` (plain decimal sizes: a leading zero is refused).
- Host: the crate tests (the cut at every step, the dead card and the tools' verdict on the new paths, the global
  index checked against each session's own, an earlier boot's reserved label under its escape), `src/control` tests
  (`board_pull` on both layouts, `sd_prepare` on images only: both partitions fsck-clean, sizes, labels, serials),
  and `test_sd_partition_table.py`: the board's own `partition_table.c`, built with the host's `cc`, over images of
  each card shape (an `sd_prepare.sh` card, two FAT16 partitions, a superfloppy, one partition, a non-FAT partition
  2, no table; overlapping partitions, one past the card, a file system past its partition, unreadable sectors).
- On the board (2026-10-07): the A/B above, both ways round; the stalls follow card AB58, not the layout. On each
  layout, video ran with the saver running, and `board_pull.sh` pulled the session (recordings plus video).
  `assemble_capture` and `flight_telemetry` read it strictly: 849 and 822 rows plus ~35,000 log lines, all good,
  0 duplicates. Card 267D, an M4 card carried forward, also gave up an old-layout session (`recordings/000340`,
  `video/000340`). Its reflash boot, 000347, sits in both layouts, and the pull refused it as designed.

**Operator gains.** `tools/board_pull.sh taster` drops a session into the same tool chain as before
(`assemble_capture`, `flight_svg`, `flight_report`, `flight_kpi`, `recorder_flight`). The flash empties itself
to the SD in the background, and a board without a card keeps the newest ~26 min.

### M5. Buses, layout, databoard, device registry (2 days; DONE 2026-10-07 on the taster and TMS-7F)

**Scope**
- **`I2cPort`** for ports 0 (SDA7/SCL8) and 1 (SDA31/SCL30). Each implements `embedded_hal::i2c::I2c` over
  `i2c_master` with per-device speed. Wedge = 4 consecutive failures, cured by `i2c_master_bus_reset` and
  re-adding the handles. Also `general_call_reset()` and `probe(addr)`.
- **Generic bus re-init:** after a general-call reset, the bus owner re-initializes **every** device registered
  on that bus, found by membership in the device registry, never by a hard-coded list (§4.3). The general call
  is rare, so there is no per-device fast path.
- **Plan B patch** (the one sanctioned `src/glider` change, §7.2 Q9): the same membership-based re-init in
  `drivers/icp10111.py`, so 7C's SEN0697 (BMI323, BMM350, BMP581) is re-armed after the ICP's general call.
  DONE: commit 76c138c, tag `micropython_plan_B_2`; every golden fixture is generated from that tree. Owed: one run of
  plan B's patched test (`test_icp10111.py`) on a MicroPython board when one is free (7C, after its post-crash
  validation); it does not block M5.
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

**Design decisions (2026-10-07, from plan B's code and IDF v5.5.5; the staged plan is in the M5 commits):**
- **Threads.** The `i2c0` and `i2c1` owners (core 1, priorities 18 and 17, internal stacks) create their buses,
  so each bus's interrupt runs on core 1, and lend the port to main. Main does the layout scan and the serial setup
  in config order (plan B's `Controller.setup`, nothing else running), then hands each port back with its members
  (§4.5 step 9). Port 0 is created exactly once, by its owner (the camera's SCCB shares it); `buses::i2c::port0()`
  only reads the published handle. Port 1 exists only when `buses.i2c.1` is defined.
- **Bus settings, as plan B's MicroPython:** internal pull-ups on, a glitch filter of 7, 50 ms transfer timeouts,
  synchronous driver (async refuses `i2c_master_probe`). One device handle per address, at the port's post-layout
  `freq` (the schema has no per-device key); the camera keeps esp_video's own 400 kHz handle.
- **Wedge:** plan B's counter, 4 consecutive failures across the port, repeating every 4; any success clears it.
  It counts wire failures (`ESP_ERR_INVALID_STATE`: a NAK, an SCL timeout, lost arbitration), the general call's NAK
  included. It does not count a bus-lock timeout (the camera can hold the lock up to 500 ms), caller errors, probes
  or scans. The cure is `i2c_master_bus_reset` (FSM reset, 9 SCL pulses); the handles are re-added only when the
  live speed differs from the configured one (after a bustune). Plan B's log line.
- **General call:** after any general call on a port, every member of that port that came up is re-armed
  (`Device::rearm`, never `setup`: that would re-zero the ICP and reload the SDP810 tare), by name, the sender
  excluded; each walk is an oops record (`cause=i2c-general-call`, §4.8). The camera (0x36) and the ES8311 (0x18)
  are no registry members.
- **SPI2:** a thin `spi_master` wrapper, DMA off, `intr_flags` 0, polling, one full-duplex chip-select frame of at
  most 64 B, `isr_cpu_id` core 1, one handle per chip select, 16 framed reads of register 0 discarded on first use
  (plan B counts nothing, and register 0 reads 0x00 on the LSM6DSO32 anyway). Its owner is M6's control thread; in
  M5 the SPI parts are set up on main and held. The taster carries an LSM6DSO32 too: WHO_AM_I 0x6C on CS 50
  (2026-10-07). No SPI code needs IRAM (§4.7): linking `spi_master` costs 1.2 KB of it.
- **The registry (`crates/controller`, plan B's controller.py rules; the firmware's `devices.rs`, with the parts in
  `sensors.rs`: a firmware module named `controller` would be ambiguous with the crate):**
  `directory()` (enabled, named entries; parts layout disabled are absent), Up/Failed/Unfitted/Crashed, retries
  200 ms apart with diagnose on the last, the alternatives pass with its four rules (XSHUT included), Crashed only
  for errors that escape the per-part glue (M6+). Entries with no M5 driver: native ones (recorder, cc, wifi,
  health, ov5647) are Up; known plan B implementations not ported yet fail `no driver/activity -- not ported yet`;
  anything else fails plan B's `no driver/activity`.
- **The databoard topology** is built from the Up entries' `provides` (a Parameter only for a quantity with an Up
  provider), frozen with `Box::leak`. Sample types: `f32`, `[f32; 3]` (accel, rate, mag, attitude), `[i32; 2]`
  (position in 1e-7 degrees).
- **`config_id` after layout:** whoami, the provenance line, session.csv, the session-start record and the panic tags
  carry the post-layout id; `set-config` keeps returning the pre-layout id (Q4); the provenance line also names
  `saved_config <pre-layout id>`. So the ring_writer starts after layout (§4.5). `get-config board`/`running`
  answer the post-layout document, as plan B does (`saved` stays pre-layout): CC's bustune sweep restores the
  frequency it reads there.
- **Device probes vs bus probe:** `I2cPort::probe` is an ACK check; `Part::probe` is plan B's device probe. The ICP
  and SDP810 probes, which waited for plan B's run loop, do one direct conversion or read on the owner.
- **Forced proofs:** GCL `i2c touch` (repeated reads: the wedge counter) and `i2c general-call` (SETTING-only,
  confirmed), since plan B has no command that sends a general call (Q20: the console API replaces board scripts).
- **Config values validate() admits but plan B cannot run**, as board_config's typed view reads them (its doc
  comments cite plan B's lines):
  - a `board.setup_retries` that is not an int reads as 1 (plan B's bring-up raises on text, null or a float);
  - a negative `provides.<quantity>.timeout_ms` reads as the default second. Plan B keeps it, and on a
    primary-tier source it becomes the quantity's shared window, so no source of that quantity is ever fresh;
  - a `freq` or `baud` that is not a whole number of Hz reads as the bus default (400 kHz, 5 MHz), and a whole
    float as its value. validate() never checks them, and MicroPython's `I2C()`/`SPI()` raise on text, null or a
    float, losing the bus;
  - an int `addr` outside 0..=0x7f reads as no address, as an absent or null one does (the entry's raw fields
    still tell the three apart). Plan B keeps it and the driver fails at its first transfer.
- **bustune (operator, 2026-10-07):** the verdicts are the parts that came up on that bus, probed after the retune.
  Plan B counts every config entry on the bus as down, disabled ones included (and, in the port, the parts whose
  drivers are not ported yet), so its sweep never found a ceiling on a bus with one. A wedge cure in the middle of a
  rung restores the configured speed and is logged as a warning (plan B did the same silently). `bustune spi` runs
  on SPI2's owner, the control thread (M6).
- **Who honours the general call** (measured 2026-10-07 on the taster, `i2c general-call rearm=no`, then a probe):
  the SDP810 (it NAKs until re-armed) and the INA226 (CALIB cleared: 0 mA and 0 mW while the die id still reads).
  The BMI323, the BMM350, the ICP-10111 and the VL53L1X carry on unchanged. The BMP581 reads the same 101032 Pa before
  and after, which is inconclusive at that resolution. The OV5647 ignores it: colour bars streamed through both
  ports' calls with 0 missed frames. So the open question of what an ICP general call does to the camera does not
  arise. With `rearm=yes` every part probes ok, and each call is an `i2c-general-call` oops record.

**Proof**
- Golden `layout.json`: the `_TMS7C`, `_TASTER` (derived, not recorded) and `_LEGACY` scans give the
  expected verdicts and edits. `tms7c.config` as-is (no `board.layout` key, which resolves as v0.1 and logs
  '(declared)') gives `f6a9c35d6372`; the generator must not add the key (an explicit `'layout': 'v0.1'`
  hashes to `92d537918c7c`). tms7f by scan on v1.1 with no SEN0253 gives `d18751e70f6c`.
- Host databoard tests: rank handover, born stale, window equals the tightest timeout of the lowest-rank tier,
  `fresh_below`.
- On the taster: the scan set the golden `m5_detect_taster` case records is `_TASTER` minus `0x29` plus `0x36`
  (OV5647 SCCB, while the camera is powered): i2c:0 {0x15, 0x18, 0x36, 0x40, 0x47, 0x69} and i2c:1 {0x25, 0x63}.
  `detect` gives `v1.1`. The taster has since been fitted with a SEN0253 and a VL53L1X. On 2026-10-07 its boot scan
  answered i2c:0 {0x15, 0x18, 0x28, 0x36, 0x40, 0x47, 0x69, 0x76} and i2c:1 {0x25, 0x29, 0x63}. Plan B's own
  `layout.resolve`, over the taster's saved config and that scan, logs the board's two lines word for word and
  hashes to the board's post-layout id (8090b3ddeea5; saved 3aa09342698a). That is the comparison the golden case
  stands for, now that the hardware has moved. A declared `v0.1` logs `(declared) already matched` and runs no scan,
  while `detect` still answers `detected` v1.1.
- Ghosts and strangers in the scans (2026-10-07). The taster's boot scan once saw 0x2b on i2c:1, and 7F's once saw
  0x15 there. Each appeared only in the first scan after the bus was created; rescans never repeated them, and a
  read of 7F's 0x15 NAKs. 7F's i2c:0 also answers 0x0c on every scan. It reads 0x00 at registers 0x00..0x02, so it
  is no AKM magnetometer, and the SEN0697 carries nothing there (BMI323, BMM350 and BMP581 only, per DFRobot's wiki
  and schematic). It is on 7F's Arducam module: with the CSI ribbon unplugged, the boot scan and `detect` lose 0x0c
  together with 0x36. Most likely a lens-motor (VCM) driver, 0x0c being the VCM address, although the module is sold
  as fixed-focus. None of these addresses is a layout vote or the anchor. The same test showed that the camera's
  entry comes up "up" with no camera fitted: the bring-up runs the camera natively and never asks the sensor (open,
  with the video work).
- The bring-up, on the taster and on TMS-7F (2026-10-07):
  - every fitted part comes up in config order: the LSM6DSO32 (SPI2, CS 50), the ICP-10111, BMI323, BMM350, BMP581,
    SDP810, VL53L1X and INA226. Bring-up takes 1056 ms on the taster and 761 ms on 7F;
  - the taster's VL53L4CX entry fails its three attempts 200 ms apart, then is reported as plan B words it,
    `vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:1 0x29`. That is the alternatives pass on hardware;
  - the parts not ported yet fail with `no driver/activity -- not ported yet`, so `arm` refuses on them;
  - `health.tasks`, `verify` (devices, problems, alternatives), `inspect controller` and `probe all` (its two maps)
    answer from the table;
  - the databoard topology is logged one quantity a line, its sources in rank order;
  - internal RAM 8 s after boot (before Wi-Fi joins) fell 5.2 KB with the bring-up (112.0 → 106.8 KB). The review
    fixes then put the topology in internal RAM on purpose (+~4.2 KB) and trimmed the owner stacks (−2 KB):
    104.4 KB. The bring-up's peak on main's stack is ~3.6 KB.
- Stage 7 (close):
  - the owner stacks are trimmed to 5 KB, against a measured peak of 3.4 KB (probe all, the bustune ladders, and
    general calls with every part re-armed);
  - the review fixes:
    - setup-time general calls make one oops per entry, and none when nothing was re-armed;
    - a ranger's XSHUT driver is never released;
    - a part on an undefined bus fails with no transport;
    - `probe all` lists every entry that came up;
    - each Parameter is boxed, which keeps it internal.
  - TMS-7F soaked ~21 minutes of FHD@25 colour bars plus the flight-rate ring load: 31,664 frames, 2 missed, 0 ring
    loss, internal low-water flat at 78.2–78.4 KB, no panic records.
  - M5's price in internal RAM, the stage 2 build against the close on the taster, measured 60 s after boot with
    Wi-Fi up: 113.5 → 104.0 KB free (DMA-capable 85.8 → 76.3 KB). Wi-Fi's join costs about 1 KB of it either
    way: 114.4 → 104.8 KB at 8 s.
- `bustune` ladders on the taster: i2c:0 at 100 kHz, 1 MHz, then 400 kHz, and i2c:1 at 400 kHz, 1 MHz, then
  100 kHz, all `all_ok`. Afterwards `i2c touch` shows each port at its post-layout speed (400000 and 100000).
- A forced general call on each port, measured part by part (the M5 decisions above), with every part probing ok
  after the re-arm.
- `tools/cc_contract.py taster --serial /dev/ttyACM0 --milestone M5 --allow-active`: 0 violations, diagnostics
  8 of 8.

**Operator gains.** The detect-layout and bus-tune cards work. The dashboard layout column is real.

### M6. Control thread, primary IMU, barometer, separation (2 days; DONE 2026-10-07 on the taster and TMS-7F)

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
- **Separation:** GPIO33 with pull-down. A both-edge ISR stamps the first and the latest edge of the contact's
  episode into two atomics (esp_timer us, u32; drivers::debounce `StampPair`). The control thread reads the pin every
  tick and takes the pair once no edge came for a quiet (`debounce_ms`, default 10 ms): 10-20 ms after the last edge
  on the 10 ms tick. Polled (the arm failed), the tick's reads feed the same pair.
- Streams `imu_lsm6dso32.csv`, `baro_icp10111.csv` and `separation.csv`.
- `inspect`, `update{rezero,ground}`, `probe`, `diagnose` and `calibrate` through owner mailboxes.
- `health.calibration` becomes real. `degraded` gains `needs-calibration`.

**Proof**
- Golden `drivers_icp10111.json`: `_compensate(11441968, 27034)` with OTP [211, 371, 553, 3833], pinned exactly
  from plan B under f32 and f64. This decides the f64 question for the polynomial (§7, Q8).
- Golden `drivers_lsm6dso32.json` (scaling, frame decode).
- On the taster: `pace` under FHD@25 plus a real I2C/SPI load shows 0 missed ticks and max ≤ 200 µs.
- Still-board noise matches the plan B `measure_noise` provenance numbers within their spread.

**Decisions (2026-10-07, as built):**
- **One clock.** `board_config::schedule_hz` is the flight entry's `schedule_hz`, enabled or not, 1..=1000 Hz, else
  100. It paces the whole control thread, where plan B's timer paced only its flight task (and `schedule_hz` 0 ran
  that task on asyncio). The gptimer is created on the control thread, so its interrupt lands on core 1; its ISR
  only notifies (eIncrement) the control thread and each I2C owner.
- **The owners wake on the tick or on a request** (`ask` notifies) and do their work by time, so an extra wake
  costs a clock read; they serve at most two requests a wake and clear their notification count on each take.
- **The IMU is read on every tick**, not on its INT1 edge (plan B sampled on its period too). A core-1 GPIO
  interrupt only counts edges; `irq_runs` is the tick's count. INT1 is an input with the internal pull-down, so a
  broken wire reads 0 rather than floating. The driver's silent verdict (three empty ticks in a row) logs one line.
- **Only a new conversion is published.** Each read is one 16-byte burst from STATUS_REG: `accel` is pushed on
  XLDA, `rate` on GDA, the row written on either. A part's ODR is its own (INTERNAL_FREQ_FINE, logged at bring-up):
  one slower than the 100 Hz tick reads stale now and then (the taster's 98.86 Hz: 1.2 % of reads), one faster skips
  a conversion now and then. Three stale reads in a row make the part unhealthy, with one line; an impossible status
  (a reserved bit set, a dead SPI part's 0xFF) is never fresh. Plan B pushed every read.
- **Health is the producer's.** Each part's table entry carries its runtime health, written by the thread that reads
  it (the control thread for the IMU, the owner for the barometer) on a change only; `inspect` reads it. Each own
  value in `inspect` is the part's latest while it is inside its quantity's window, else null.
- **The wire rule.** A failure the port says is not the wire's (`ErrorKind::Other`: the bus lock timed out, a driver
  error) is reported but neither strikes the ICP nor resets its run, and a general call the port refused counts as
  none, in the driver and in the port's own figures.
- **The ICP-10111 cycle is 120-130 ms** (about 8 Hz) on the 100 Hz tick: the collect lands on the tick after its
  12 ms conversion, and the next issue a period after the collect, as plan B's sleep-after-read (about 112 ms).
  Both are inside the quantities' 200 ms window.
- **Separation (operator, 2026-10-07).** The contact is a copper ring on the booster, 5 mm wide, bridging two copper
  strips on the glider, 2 cm long, laid across it like a П. At separation the glider slides out (П, Н, U, out); while
  the strips' 2 cm pass over the ring the contact may wobble 1->0->1 (the tubes do not fit perfectly), for under
  10 ms and less than 2 cm at 3 m/s (~7 ms), since the blast and the drag slow the booster as the glider is pushed on. The driver is a thin latch over a generic
  debounce (drivers::debounce, in the drivers' pattern: `Settings::from_config` infallible, `new(&settings)`), on the
  caller's microsecond clock. **Revised (operator, 2026-10-07, a ring buffer of edges rejected as too complex):** two
  stamps, start and finish: the ISR's first edge of an episode sets the start, every edge the finish. The tick takes
  the pair once the finish is a quiet old (`debounce_ms`, default 10 ms, "10 ms or less", above the wobble's gaps) and
  decides by the level it reads: `separated` (the latch nested, the pin 0), `nested` (the reverse) or `blip` (the
  latch's own level: the contact moved and came back). The tick's read is evidence too: with no episode open, a read
  off the latch opens one (a change before the arm, a lost edge, a board powered up unassembled and then assembled,
  which ends nested at 1). A polled pin feeds the same pair from its reads. Every event records its **moment** (the
  episode's first edge, in uptime us) and its **duration** (to its last edge, in us: the wobble, 0 for a clean edge),
  in `separation.csv` (`event;stage;moment;duration`, durable, blips included) and in its log line. The board's
  config validator refuses a `debounce_ms` outside 0..=1000 (tools/provision.sh too). Plan B's built-in default
  document still writes 20: a board that falls back to it debounces 20 ms; real boards carry 10 in their saved
  configs. The GPIO interrupt service is installed once by the control thread, which latches every GPIO interrupt on
  core 1.
- **The CC surface.** Every device that came up is an inspectable object (plan B's `Task.inspect`), its own keys read
  off the databoard: the IMU's `rate_dps` is in deg/s (D18; plan B's held centideg/s ints under that name). In M6 only the ICP-10111 has a calibration (it asks for its
  ground until it holds one, and setup captures it); the SDP810's tare and the BMM350's hard iron join `calibrate` and
  `health.calibration` with their restores (M9, M10).
- **`bustune spi 1 <hz>` runs on the control thread**, SPI2's owner, in plan B's answer shape; SPI2's only device is
  the IMU, so its verdict is the rung's. The rung drops the IMU's handle and adds one at the new clock (the first-use
  discard runs again), then probes. The order matters: a second handle on the same CS would take the pin's routing,
  and removing the first one then resets that GPIO under it. A clock the driver refuses re-adds the handle at the
  configured clock; if that is refused too, the IMU is unhealthy and logged. A clock above the LSM6DSO32's 10 MHz is
  never tried: it is answered as the IMU failing the rung (`all_ok` false, the handle untouched), because the CC's
  ladder runs 5, 8, 10, 16, 20 MHz and stops on any answer that is not `ok`. `control` shows `spi_clock_hz`. The
  rung's clock is never saved (the CC sweep restores the configured clock and applies a choice by set-config), and
  arming does not restore it (M7).
- **The flight core never prints in place** (found by the timing proof, below): a console write blocks once the
  UART's transmit buffer fills, and an IMU probe's two log lines made the control tick 4.5 ms late. Lines logged on
  core 1 go into a bounded queue (non-blocking send, dropped and counted when full) that a core-0 printer drains;
  their recorder.log copies are unchanged. An IDF C log line on core 1 still prints in place (rare; none on the tick's
  path).

**Proof (2026-10-07, the taster and TMS-7F):**
- Interrupt placement (`interrupts`, esp_intr_dump): the gptimer (TG0_T0), GPIO (INT1, the separation pin), SPI2 and
  both I2C ports (one shared vector) on CPU 1; SDIO, the H.264 encoder and UART0 on CPU 0.
- The tick under FHD@25 colour bars, the flight-rate ring load, `probe all` and bustune ladders on both buses, ~1-2
  minutes: max lateness 40 us on the taster and 77 us on 7F (budget 200 us), 0 overruns, every tick's IMU read, 0
  frames missed. Stack peaks: control 3.5 KB of 6 KB, the owners 3.5 KB of 5 KB.
- The IMU at rest: rows 10.000 ms apart (9989-10012 us); |a| 1.013 g (sd 0.0017 g); gyro bias < 0.5 deg/s, sd
  0.036 deg/s, inside plan B's measure_noise spread (0.03-0.095 deg/s, its only recorded noise number). TMS-7F, 30 s
  of the final build at rest: |a| 1.028 g (sd 0.0017 g), gyro sd 0.027-0.036 deg/s, biases +0.58/-0.03/-0.08 deg/s.
- The ICP at rest: pressure sd 1.55 Pa (datasheet 1.6 Pa RMS); `update {"rezero": true}` -> `[ground]`, a text
  ground refused, `ground 42.5` -> elevation 0.37 m; `calibrate baro_icp10111` -> null with plan B's ground line.
- `cc_contract --milestone M6 --allow-active --allow-stage` (with the M6 cases: the parts' inspect shapes, the
  barometer's ground round trip, calibrate, probe of each part, the IMU's bus at its own clock, calibrate refused
  while held): 0 violations, inspect 34, diagnostics 15, control 14.
- `bustune spi` on the taster: 1, 8, then 5 MHz all `all_ok`, `spi_clock_hz` following, reads uninterrupted, 0
  failures; 20 MHz answered untried with the clock unchanged; 0 Hz refused; refused outside SETTING. The CC's own
  sweep (`bustune taster spi 1`): 5, 8, 10 MHz pass, 16 MHz fails on the IMU; ceiling 10 MHz, chosen 8, the bus put
  back at 5.
- Stale reads null, on the taster: the ICP's temperature window narrowed to 50 ms (its cycle is 120-130 ms) by a
  saved config and a reboot, 20 `inspect baro_icp10111` 0.3 s apart read `temperature_c` null 13 times (60 %
  expected) and `pressure_pa` never, `healthy` true throughout; the config saved back gave 8090b3ddeea5 again.
- The GPIO interrupt service is installed once, by the control thread before either interrupt is armed (the boot
  log's `already installed` error is gone); `interrupts` shows GPIO_INT0 and TG0_T0 on CPU 1.
- The internal-RAM stress on TMS-7F (FHD@25 bars, the flight-rate ring load, four parallel 24 MB pulls, a POST
  flood, 20 set-configs): every pull hash-clean, 20/20 saves, `hosted_internal` 0, no TCP checksum errors. Each phase
  on its own boot: the control tick's lateness at most 105 us with the pulls and the flood, 7 us across the saves.
  `pace` (a priority-20 loop on core 1, the flight thread's stand-in before M6) now contends with the real control
  thread: run beside the pulls it pushed one tick to 651 us. It is left out of the stress from M6 on; the control
  tick's own figures are the measure. The low-water: 64.4 KB internal, 36.7 KB DMA-capable (81.9 / 54.2 KB before
  M5), M6's 13.5 KB on top of M5's 9.5 KB. Open: the video misses 21-26 frames when the flight-rate ring load and the four
  pulls run together (0 in every stress before M5). Each alone, or either with the POST flood, misses none; together
  the encoder's longest frame doubles (94 ms against 43-45). Both load the card (the saver writes the ring onto it,
  the pulls read it), and M6's IMU rows (100 a second) add to the saver's writes. Ground-only: no pull runs while a
  flight records, and the hour-long soak of the flight-rate load missed 11 in 90,036. Accepted (operator,
  2026-10-07): video holds when it delivers at least 20 frames a second, so at 25-27 fps up to 5-7 frames a second
  may go missing; the worst stress run delivered about 24.7 fps (26 missed in about 87 s).
- TMS-7F, one hour of FHD@25 colour bars and the flight-rate ring load (the stage 6 build, six 10-minute segments):
  90,036 frames, 11 missed, 0 ring loss, tick lateness at most 86 us, 0 over budget, 0 overruns, 0 IMU failures, the
  internal low-water flat at 61.4 KB (one 0.75 KB step in the fifth segment).
- The separation switch's live edge (the operator's hands on the taster, SETTING): the pads opened, then closed
  5.73 s later. `separated` and then `nested` were logged and the latch followed (`control`). Session 000409's
  `separation.csv` holds both rows, `345852929;separated;setting` and `351582929;nested;setting`, and the stage
  stayed SETTING (separation acts only in BOOSTING). The log's timestamps sit a constant 940 ms after the rows'
  uptimes: `esp_log_timestamp`'s base, not latency. The sub-20 ms blip that must not count is proved on the
  host, against plan B's debounce: every case ends with plan B's latch and stage (a 7 ms blip and a 40 ms open at a
  50 ms debounce both rejected). The edge-timed hold re-pins that comparison (golden_separation.rs): never a
  separation or a GLIDING plan B did not make, a deploy earlier only within one of plan B's 5 ms reads, and one
  documented exception, plan B's own 0 ms debounce that never settles there.

- The edge-timed separation on the taster (debounce 10 ms; the operator's hands, 2026-10-07, session 000417): 16
  open/close cycles, clean and wiggled, 32 rows alternating separated/nested, the stage SETTING throughout. The
  wobble from the first edge to the sustained level: on opening, median 277 us, at most 1,733 us (4 clean single
  edges); on closing, median 56 us, at most 19,019 us, still one event while no gap reached 10 ms. The decision came
  10,066-19,667 us after the sustained level began (mean 15,135 us): the hold plus the tick's phase. Two wiggles held a
  contact gap past 10 ms and so made their own pair (30.7 ms and 125.7 ms after the previous event), as the hold
  intends.
- The two-stamp separation on the taster (the operator's model: the interrupt stamps an episode's first and latest
  edge, the tick concludes it 10 ms after the latest by the pin's level; the operator's hands, 2026-10-07, session
  000418): 55 rows, the stage SETTING throughout, separated and nested strictly alternating with every wiggle that came
  back in between as a blip. Separated 17 (wobble median 1,161 us, at most 17,447 us, 2 clean single edges), nested 16
  (median 224 us, at most 18,000 us), blips 22 (median 2 us, at most 62,090 us, 11 single edges). Every decision came
  10.0-19.9 ms after the episode's last edge: the 10 ms quiet plus the tick's phase. Session 000419: powered up with the
  pin out, the latch started separated, and 22 touches that never held stayed blips at 0, the latch unmoved.

**Carried into M9 (measured at the M6 close):** both boards' LSM6DSO32 run below the tick (98.86 and 98.39 Hz), so
the fresh-only rows skip a tick about 1.5 % of the time and two in a row about 8-9 times a minute: a 30 ms gap that
lapses the 20 ms `accel`/`rate` window for ~10 ms, about every 7 s (taster 121 in 787 s, 7F 738 in 5257 s). Nothing
reads those channels in flight in M6; the attitude filter (M9) and the D term (M13) will, and the databoard would hand
over to the BMI323 for a tick. Choose with M9: run the part at 208 Hz (a fresh conversion every tick, twice the noise
bandwidth), widen the IMU windows to 30 ms, or close the status-read race.
From M7 the stage machine and the separation's thrust gate read `accel` in flight, so the gaps reach them. A missed
read drains the leaky launch dwell for its step instead of feeding it (`commons::dwell_step`), so a launch can come up
to two steps (~100 ms) later; the stillness dwell is contiguous, so a missed read restarts it
(`StageMachine::detect_stationary`); the gate's two dwells are leaky like the launch's. Accepted until M9's
choice ends it. M9's optional `axes` key then reaches the gate's +x as well.

**Carried into M7 (from the M6 review):**
- The control thread's mailbox answers through std mpsc, which takes a mutex and a condvar on core 1 when the
  executor waits. That is harmless for the SETTING-only probe, but M7 routes `disarm` and holds through it, and they
  stay open in flight: give them a lock-free reply slot.
- The stage has two writers now: the executor's `stage` command and the separation's BOOSTING -> GLIDING. M7's single
  writer moves `stage` into the control thread's mailbox, so an operator's hold cannot be overwritten between the
  separation's read and its write.

**Operator gains.** Real attitude-grade IMU and baro data in the telemetry pane. Probe all and calibrate work
for these parts.

### M7. Stage machine and a real arm/verify gate (2 days; DONE 2026-10-08 on the taster and TMS-7F)

**Scope**
- **`crates/sequencer`** (the stage machine joins the existing `Stage`) in the control thread, every `period_ms`
  (default 50 ms: 20 Hz) on the uptime clock: launch, apogee and burnout, land, still, and the flight-timeout
  backstop, all gated by `read().source` (§4.3, step 6). The entry's keys are read by `Settings::from_config`: what
  plan B flies is taken, what plan B cannot fly fails the entry at bring-up, so `arm` refuses (§6).
- **Separation gated on thrust end (operator, 2026-10-07):** the real separation completes within ~7 ms, so the 10 ms
  hold is too coarse; what it guarded was a contact blip under thrust. A separation now counts only after the IMU sees
  thrust end (the booster cannot separate while it pushes), so vibration under thrust can never deploy, and the hold
  drops to about 1 ms (`debounce_ms`; the decision still lands on the next tick). If the pads are already open when the
  gate opens, the separation counts then, its moment the recorded first departure. Plan B accepted a held separation
  anywhere in BOOSTING (§6). Every episode of the contact stays recorded in us (`separation.csv`, its blips too), so a
  flight shows the blast and the boost's vibration.
  - **Early separation (operator):** a destabilized stack can lose the glider early, for aerodynamic reasons, while the
    booster still burns. Thrust end is therefore the GLIDER's own: its IMU's axial specific force, which drops from
    +several g to drag deceleration the moment the glider stops being pushed -- by burnout, or by an early separation.
    So an early separation opens the gate itself and counts at once; a stack that holds together keeps the glider
    pushed, and its vibration still cannot deploy.
  - **Backstop:** a contact held open for a long run (about 100 ms, far beyond any wobble or blip) counts whatever the
    IMU says, so a dead or confused IMU can never hide a real separation.
  - **Kept simple (operator):** the nominal separation is the ejection charge, about 4 s after thrust end. No coast
    window and no queue: the interrupt stamps the episode's first and latest edge (two words); the control thread
    takes them once no edge came for the hold (one `debounce_ms`) and decides by the level it reads then --
    separated, nested or a blip, each recorded -- gated as above.
  - **The gate's numbers:** the axis is the body frame's +x, the nose (the body frame, operator 2026-10-07: +x nose,
    +y left, +z up; §7.2, IMU axes), read from the fresh `accel` beside the stage machine's read. Thrust is seen when a_x
    stays above the sequencer's `launch_g` for its `launch_ms` of net time (the launch's own leaky dwell; plan B's
    defaults without a sequencer entry), and has ended when a_x then stays below 0 g for 20 ms of net time; both
    start afresh at each BOOSTING entry. A separation concluded in BOOSTING before the gate opens is pending: it
    counts when the gate opens, or once the pin has stayed 0 for 100 ms after the episode's last edge (the
    backstop), dated by the episode's first edge either way; a re-nest clears it. A wrong axis or sign, no IMU, or a
    bench hold never sees thrust, so only the backstop decides: a mis-mounted IMU can never open the gate under
    thrust. `control` shows `thrust` (none, seen, ended) and `pending`.
  - **How a pending separation is recorded:** it writes no row while it waits, and the latch the warm-start gate
    reads stays nested. It counts only on a tick that reads the pin 0 with no episode open, by the gate and the
    backstop alike, so a gate that opens on a contact closing again counts nothing, even on a bounce that reads 0:
    that episode's conclusion decides. A touch that lets go again is part of it: the backstop runs from the touch's
    last edge, and an open gate counts it when the touch concludes. A re-nest makes the open one `blip` row, from its
    first edge to the re-nest's last. When BOOSTING ends otherwise (apogee, burnout, the flight timeout, a hold), it
    counts on the next such tick, as a separation outside BOOSTING: the row in the new stage, nothing advanced. A
    `debounce_ms` of 100 or more meets the backstop when the episode concludes. Costs at the defaults: an early
    separation counts 20-30 ms after a_x goes negative, a held open under a shut gate 100-110 ms after its last edge,
    and an open of up to 100 ms under thrust is a blip.
- **One writer of the flight state:** the control thread commits the stage, the hold, the arming and `launched` to
  the flight word (§4.1): the stage machine's advances, the separation's, and the operator's hold, resume, arm and
  disarm, which reach it through a polled request slot (§4.3, step 4). This closes both carry-overs from the M6
  review. `control` shows `launched` and the tick's work, `work_last_us` and `work_max_us` (from the wake to the end of
  the tick's work, which the lateness figures do not see).
- `sequencer.csv` (the entry's `<name>.csv`, `stage;reason`): durable and never decimated (§6). The recorder's
  flight flag follows the committed stage (§4.3, step 6).
- **Arm and verify:**
  - `arm` refuses on failures (an entry the firmware cannot run yet included: plan B's rule), a stage problem, the
    ring gate, or any failing probe in SETTING. The control thread checks the stage again when it applies the arm,
    and a request that waited over 25 s on the board is refused (§4.4).
  - The launch-point freeze moves to M11: before the GNSS is ported there is no fix to freeze (`atgm336h` and
    `neo6mv2` are not ported, `firmware/src/sensors.rs:64-81`), and a CC-set position already wins in `launch_point`
    (`crates/mission/src/lib.rs:297-307`).
  - `verify` gets the readiness gate, plan B's `_readiness` with its texts. The Rust version uses
    `.get('enabled', true)`, the Controller's meaning (§6). `fins` came with M13, the BNO055's `imu_calibration` with
    M18.
  - Both sweep their probes only in SETTING (§6).
  - `not-flight-capable` is removed from `degraded`.
  - `bluetooth` becomes a stub in plan B's no-BLE shape (§6).
- **Ring gate:** `arm` and `verify` also require the eraser to hold its reserve: at least ~1 % of the ring erased
  ahead of the writer (§3.7). No card is a `no-sd` warning, not a refusal: newest wins keeps the flight's
  blocks over the pad's.
  - It refuses when the bring-up scan has not run, when the eraser is not running, or when fewer blocks are erased
    ahead of the writer than the reserve (`reserve_blocks`, `crates/flash_ring/src/geometry.rs:46-56`). A full ring
    and a saver behind are not refusals either. Newest wins erases the oldest unsaved block once no more than the
    reserve is free (`crates/flash_ring/src/eraser.rs:279-283`), one block above the gate's line, so the gate does
    not flicker on a board without a card.
  - It is reported as plan B's recorder probe (`tasks/recorder.py:30-48`): under `recorder`, with its three log
    lines and the verdict `recorder rings: <why>`. It is keyed on the recorder, not on a config entry: the ring runs
    whenever the recorder opened. A recorder that never opened has nothing to probe, and a configured `recorder`
    entry is then already a failure. It never advances health's backlog trend.
- **False-launch recovery (operator, 2026-10-07):** an elevation offset above `launch_alt_m` (the weather's drift,
  or a glider carried up from a lower assembly point) fires BOOSTING within ~150 ms of `stage setting`, and every
  fix ran only in SETTING, so only a power cycle recovered. The barometers' re-zero (`calibrate <barometer>`,
  `update <barometer> {"rezero"|"ground"}`) now also runs at DONE: `stage done` (a hold), then the re-zero, then
  `stage setting`. `reboot` stays SETTING-only (§6).
- **SETTING-only requests are checked again where they are served** (an owner's mailbox, the control thread's): a
  launch detected during a probe sweep then never runs a probe in BOOSTING (§4.4).
- **The link by stage (operator, 2026-10-07; replaces the radio-off policy):** the radio is never switched off. In
  flight the glider leaves range and the link drops by itself; once the stage machine has detected the launch, the
  link never rejoins, hunts or dials again, and a session still up runs until it drops. At DONE the recovery beacon's
  AP (M11b) is the only network. An operator hold (`stage gliding` from CC on the bench) is no launch and keeps the
  normal reconnects. This avoids stopping and restarting Wi-Fi through esp_hosted on the C6, an unproven path that can
  park the link until a reboot. Plan B keeps hunting on the pad link through the flight (it only pauses) (§6).
- **Video by stage:** recording starts when the glider stands **vertical on the rod** (attitude-based, from
  M9; until then arm is the trigger) at 1920×1080@25, 8 Mbit/s, and stops at DONE (M7; it was M11b's). The console
  default is 25 fps (GCL `video start`, done).
  - **One owner, the `video` thread** (core 0, low priority, PSRAM stack; it runs only when a camera and a card are
    configured). It is the only caller of `coludo_video_start` and `coludo_video_stop`, whose C side assumes one
    caller ("starts and stops never overlap", `components/coludo_video/coludo_video.c:92-93`). It reads the flight
    word every 250 ms: arming in SETTING starts the flight recording, replacing an operator's recording (logged); a
    disarm in SETTING stops its own flight recording; DONE stops whatever records, without the free-space scan. It
    never starts outside SETTING, so the camera's register traffic on i2c:0 stays pre-flight (§7.1, the shared port 0). A start
    that fails at arm is logged once and not retried before the next arm: a dead camera never blocks arm. The GCL
    `video start` and `video stop` post to it and wait with a deadline (10 s; a busy or silent thread is answered
    as such): `video start` stays SETTING-only, while `video stop` also runs at DONE and is refused in the air. A
    return to SETTING while still armed (the false-launch recovery) starts the flight recording again. The supervisor
    makes no blocking camera call (it feeds the TWDT from M8).
  - **Segments and the preview:** M7b.
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
      - its oscillator: the board config's camera entry, `clock_hz` (board-config.md; the taster's 24 MHz,
        7F's Arducam 27 MHz), measured from the SEI fps (`video_sei.py`). Done 2026-10-07: colour bars at
        25 fps gave 25.010 fps on the taster and 25.004 on 7F, 0 missed frames. A board with no camera entry
        records no video and keeps the encoder's ~138 KB of internal RAM;
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
    - This also suits the 5-minute segments (M7b).
  - **Open: sporadic single missed frames:** about 1 in 300, with Wi-Fi up. At FHD@25 the encoder takes 36 of the
    40 ms on core 0, beside esp_hosted and lwIP. In flight the link carries nothing once it drops with range, and
    nothing rejoins it (the link by stage, above).
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
- **Camera preview in CC:** M7b.
- **Pad sequence** (the operator's; the gates fit it): (1) assemble glider and booster, plug the battery,
  close the cover; (2) CC check, time sync and probe; (3) if there is time, wait 5 min for GNSS; (4) place on
  the rod; (5) tie to the ignition system; (6) launch.
  - The board boots at (1). The ring_eraser runs in the background from then on, and the hub's clock sync
    on connect is normally done before (2).
  - (2) is `verify`, `probe` and the ring gate, all on the ground with the pad link up.
  - `arm` freezes the launch point (from M11, with the GNSS), so it belongs after (3).
  - Video starts at (4) on the attitude trigger (M9); until then it starts at arm.
  - The link stays up through (5). From the launch detected at (6) it never rejoins (the link by stage, above).

**Proof**
- Golden `sequencer_trace.json`: plan B's `Sequencer._tick` driven with scripted `(value, source)` reads. It
  includes stale agl at apogee, GNSS elevation on the pad, a baro dropout, and the warm-entry rebase, and from M7
  the flight timeout from BOOSTING and a separation while held (it advances, and the hold stays). Stage and reason
  must match EXACTLY.
- Golden `commons.json` (`dwell_step`, `apogee_step`).
- Contract tier `control`: arm, verify and probe shapes; `cc.py verify` exit codes 0/1/3.
- On the taster: after a hand-shaken launch (accel) the link, once dropped (the hub stopped), never rejoins or
  dials; `stage setting` on the console brings it back. A `stage gliding` hold from CC keeps the normal reconnects.

**Already met at the M7 start (proof lines):**
- The stage machine's goldens pass: `cargo test -p sequencer` ran 41 unit tests and 9 golden tests at `52ff0cd`,
  over the 41 scenarios `sequencer_trace.json` held then (the crate and both fixtures unchanged up to `c18e147`), and
  `StageMachine::step` is pure.
- `probe` answers the two-map reply (M5, `002ff9a`: `firmware/src/commands/diagnostics.rs:83-89, 113-118`; the
  contract reads the second map, `src/control/contract/cases.py:590`).
- The camera entry (an `ov5647` component, board-config.md) is a native implementation in the Rust registry (M5,
  `002ff9a`: `firmware/src/sensors.rs:61`), so `arm` and `verify` never count it as a failed device, which plan B
  does (§6).
- `cc.py`'s exit codes 0/1/3 are host-tested (`tools/cc.py:102-133`, `src/control/test/test_tools.py:224-248`).

**Proof (2026-10-07/08, as measured):**
- Host: the sequencer crate (the flight word, its one writer and the request slot, the claim/withdraw race over
  10^5 threaded rounds; plan B's settings parsing), the golden sequencer trace with the two M7 scenarios, readiness
  against plan B over the shipped configs, the ring gate's three refusals, the stale-arm rule, the bluetooth stub,
  and the separation's thrust gate (seen then ended on +x, the pending separation, the backstop, an early separation
  opening the gate, a wrong axis or sign and no IMU failing safe; every release waits for a quiet episode, swept
  over 2,610 phase cases). The gate's hand proof on the taster was dropped (its breadboard is too fragile for pin
  work): the first flight's separation row is its live evidence.
- The single writer (TMS-7F): holds log `controller :: stage -> X` on the console only, then `(held)`/`(auto)`
  recorded; at DONE the barometer re-zeroes while every other change is refused (the false-launch recovery);
  recorder.log carries no `controller :: stage ->`.
- The stage machine, hands-free on TMS-7F (the barometer's ground shifted -15 m; a bench copy with land_agl_m 20,
  since the landing reads the baro until M10): sequencer.csv `setting;external`, `boosting;launch alt=15m
  dwell=100ms`, `gliding;burnout timeout (no separation)` at +12.05 s, `landing;agl 14.8m` (+0.35 s),
  `done;stationary 1.0g` (+3.05 s), then after `stage done`, `calibrate baro_icp10111`, `stage setting`:
  `setting;external`, 30 s still with no row. Tick lateness at most 19 us, work at most 421 us across transitions,
  the control stack's peak 3.1 KB of 6 KB.
- Arm and verify: on their own configs both boards' verify and arm name exactly the not-ported entries; on a bench
  copy the taster's verify passes (ready false: watchdog, flight) and arm answers `ok {"armed":true}`; arm while held
  is refused with plan B's stage text; the ring probe logs plan B's three lines (6,141 blocks erased ahead, the
  reserve 61); an arm queued 26 s behind other work was answered at 26.1 s, `the request waited 26.1 s: send it
  again`, inside the hub's 30 s.
- The link, armed on the taster: after the altitude launch the hub was restarted under it -- one `launch detected
  -- no more joins or dials`, no dial for 65 s with the hub up; `stage done` brought the dial back 0.33 s later. A
  dead hub address times each dial out at 3.0 s.
- Video: the flight recording started 0.34 s after `controller :: armed`, ran through BOOSTING to DONE, and stopped
  0.13 s after DONE; it decoded at 25.01 delivered fps throughout (3,027 frames over 121.0 s, no gap over 60 ms);
  `stage setting` while armed started it again; an operator recording is replaced at arm and stopped at DONE;
  `video stop` is refused in the air.
- `cc_contract --milestone M7 --allow-active --allow-stage`: 0 violations on the taster's bench config and on
  TMS-7F.
- The taster's sensors latched up after the operator's power-up pin test (every bus and the SPI IMU silent); a full
  power cycle brought every part back. If all buses fail together, power-cycle before suspecting wiring.

**Operator gains.** The pre-flight gate is real: verify, arm, and readiness from the dashboard. A dummy flight
(drop or throw test) produces staged data and video.

### M7b. Camera preview and segments (after M7, before the first field day; DONE 2026-10-11 on the taster and TMS-7F)

Both rework the encode and write loops (`components/coludo_video/coludo_video.c:332-476`), so they are built and
proven once. Neither is on the HITL path.

**Scope**
- **Segments (operator, 2026-10-06):** on the pad the recorder starts a new file every 5 minutes, bounding what
  one damaged file can cost. Once ignition is detected the current file keeps running through the whole
  flight: a file switch never happens in the air.
  - The cut is in-band, at the first natural IDR at or after 300 s, and each file is zero-padded to the 64 KB chunk
    boundary (`tools/video_sei.py:39` strips the padding).
  - Copy the directory into memory the recorder owns (PSRAM) at start: `recorder->config = *config` keeps only the
    path's pointer (`coludo_video.c:772`), and the CString behind it dies when `video::start` returns
    (`firmware/src/video.rs:147-151`). Today the path is read only at the first open (`coludo_video.c:781`).
  - Give C a direct view of the flight word (a `*const AtomicU32` in `coludo_video_config_t`); a flag the
    supervisor sets would be up to a second stale.
  - The writer reads the word again before it closes a file: once the board has left SETTING it ignores the cut and
    keeps the old file, so a switch never happens in the air.
- **Camera preview in CC (operator, 2026-10-06; the pad's killer feature):** check where the camera points
  without touching the airframe.
  - Board: once a second, the latest captured frame is scaled to 640×480 by the P4's PPA (which can also turn
    the 90° rotation upright) and compressed by the hardware JPEG encoder, so the H.264 encoder takes no extra
    load and the flight core is untouched. `GET /camera/preview.jpg` on the file server serves it. With no
    recording running, the camera runs preview-only (capture on, H.264 off).
  - CC: the hub proxies the image; the dashboard's board panel shows it, refreshed at 1 Hz.
  - Gated to SETTING: the preview stops at BOOSTING (and after the detected launch the link never rejoins).
  - **What the hardware allows** (IDF v5.5.5, P4 rev v1.3): the PPA scales in steps of 1/16
    (`esp_driver_ppa/src/ppa_srm.c:226-227`, `PPA_LL_SRM_SCALING_FRAG_MAX` 16), so 640×480 is not reachable from
    1920×1080 and 720×405 (6/16) is; PPA SRM writes no YUV422 before rev 3.0
    (`hal/esp32p4/include/hal/ppa_ll.h:240-259`), and the JPEG encoder takes no YUV420 before rev 3.0
    (`esp_driver_jpeg/jpeg_encode.c:211-220`). So: PPA SRM to RGB565 at 720×405, then the hardware JPEG.
  - Unverified until the board shows it: the packing, the byte order and the colour range. Check with `bars`.
  - Cost (an estimate): ~1-1.5 days; ~3-4 KB internal RAM; ~0.6 MB PSRAM plus two JPEG buffers; 1-3 ms of core 0
    a frame; nothing in flight.

**Proof**
- Segments: a pad recording of over 10 minutes splits at the first IDR past each 5 minutes, every file decodes
  (`video_sei.py`), and a launch detected mid-file keeps that file to DONE.
  - **Segments DONE (2026-10-11, the taster).** `video start segment=<s>` (300 by default, the flight's too): the
    encode task pads the ring to the next 64 KB boundary at the first IDR past the segment and puts the IDR there;
    the writer opens the next file when its tail reaches it, after reading the flight word again (a `*const u32`
    to `status::FLIGHT`, `FlightWord::as_ptr`). The directory is copied at start (the caller's path is read during
    the call only). `video stats` counts `segments`. A 20 s bench run: five files, the closed ones whole chunks,
    each opening on an IDR, frame numbers continuous across them, ffmpeg 0 errors. A HITL flight with 20 s segments
    and a 50 s pad: cuts at 37 and 57 s, the launch at 71 s, the third file (124 MB) ran to DONE at 183 s. The
    10-minute pad run at the default 300 s is left to the field day's pad time.
- Preview: the dashboard shows the taster's camera at 1 Hz, `bars` confirming the packing, byte order and colour
  range; a hand-shaken launch stops the preview.
  - **Preview DONE while the camera records (2026-10-11, the taster and 7F).** Once a second on the pad (the flight
    word in SETTING, never launched) the encode task scales the 1080p frame's centre 1920x1024 by 6/16 with PPA SRM
    (YUV420 in, RGB565 out; its output must sit on 128-byte cache lines, address and size, so it is allocated
    aligned, not by the JPEG driver) to 720x384 (the JPEG's 4:2:0 wants multiples of 16), then the hardware JPEG
    (quality 60, 10-17 KB). Double buffers; `coludo_video_preview` copies the published one under a lock. The file
    server's `GET /camera/preview.jpg`, the hub's `GET /api/preview/<board>` (fetched from the board's CC peer
    address, port 8000), and the dashboard's `camera preview` button refreshing at 1 Hz. `bars`: white, yellow,
    cyan, green, magenta, red, blue, black exactly (BT.601, full range). The engines take 1 KB of internal RAM.
  - **Its cost:** the scale takes 72-81 ms (PPA reading the frame from PSRAM beside the H.264 encoder; the JPEG
    4 ms), the capture buffer held meanwhile, so the pad's recording misses ~2 frames a second while it runs: ~23
    delivered fps, above the operator's 20, and none in flight (the preview stops at BOOSTING). If it must go: PPA
    non-blocking with the buffer held, and a fourth capture buffer (+3.1 MB PSRAM).
  - **Preview-only DONE (2026-10-11):** `video preview` runs the camera with no file and the encoder idle
    (coludo_video `record` 0), the preview made as above; `video stop` ends it, and the rod trigger's flight
    recording replaces it as it does any operator's recording. On the taster: 375 frames captured, none encoded or
    written, 15 previews, through the hub.
  - **The 10-minute proof (2026-10-12, 7F):** a 16-minute recording at the default 300 s: cuts at 300.2, 300.0 and
    300.2 s, four files, the closed ones whole 64 KB chunks (~286 MB each), 0 write errors; 8.1 % frames missed (7F's
    slower encoder: ~23 fps delivered, above the operator's 20).

### M8. Warm start, checkpoint, supervisor watchdog (1-2 days; DONE 2026-10-08 on the taster and TMS-7F)

**Before M8 -- the altitude launch's rate gate (operator, 2026-10-08):** TMS-7F false-launched on the bench overnight,
unarmed: weather drift moved the barometer's elevation +10 m in 2.17 h and the altitude launch fired (plan B's rule:
elevation above `launch_alt_m` for `launch_ms`). The altitude launch now needs the rise to come fast: `launch_alt_m`
gained within a window the sequencer entry configures, by default 1 s or faster -- a rocket clears 10 m in a fraction
of a second, drift takes hours. The accel launch is unchanged. A plan B divergence (§6).
**The stage machine's sources (operator, the same day):** the stage machine must decide from several sources -- the
pitot (SDP810), both barometers (ICP-10111, BMP581), the IMU and the accelerometer -- not one quantity per detector.
The design lands with the sources: M9 (BMP581, the attitude IMU) and M10 (the pitot).
**Every barometer's quantities (operator, the same day):** temperature, pressure, altitude above sea level, elevation
above the zeroed ground, and **change** -- the elevation's rate in m/s, climb positive -- new beside plan B's four. It is
each barometer's climb rate for the multi-source detectors, and the launch rate gate's natural source once it exists.
**Measured -> reported (operator, the same day):** every sample a part measures is a telemetry row -- device streams
are never decimated, and plan B's `telemetry_ms` thinning (its MicroPython-era economy) is dropped (§6). The barometer
row is altitude;temperature;pressure;elevation;change.
**Measured before M8 (R7):** the wall clock survives a panic reset but returns off by the drift since boot -- +25.3 s
on the taster and -22.6 s on TMS-7F after ~9.7 h (~700 ppm, the RTC slow clock). A software reboot keeps the clock as
it was (the offsets unchanged across it); a power-on reset loses it (2000-01-01). The hub sets only an unset clock, in
SETTING (`src/control/server.py:298-322`), so a drifted clock stays drifted until a power-on: the crumb's re-anchoring
(sync the timekeeping timers, then set the time from the RTC, before each stamp) is what keeps a warm start's time
honest. TMS-7F arms on a bench copy of its config (the not-ported entries removed), as the taster did in M7.

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
- **From M7's flight word:** the restore commits the stage and the arming through the flight word's writer before
  any thread spawns, and a restore into BOOSTING..LANDING sets `launched`, so a warm-started board does not rejoin
  the link (watch it on the console). The crumb reads the one word. The gate's separation latch is the counted one:
  a separation still pending under the closed thrust gate is not a separation.
- **`panic`** is console-only in every stage (§4.8), with the kinds `twdt` (the supervisor stops feeding) and `stall`
  (the control tick stops), each refused where nothing would fire it.

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

**As built (2026-10-08):**
- The crumb is saved only while armed **and** launched (the machine's own launch, §4.1), about once a second on the
  supervisor's grid, `stage` written last; any edge out of that state clears it, and a failed save or clear is
  retried every poll (§6). A held armed stage therefore writes none, so the plan's held-GLIDING proof became the
  hands-free altitude launch on TMS-7F (a bench copy without the IMU entries, so the at-rest veto stays off).
- The restore runs on main after the owners' bring-up and before the control thread takes the flight word's writer
  (`FlightWriter::restore`); it commits the stage, the arming and `launched`, and rebases every barometer that
  publishes an elevation to the crumb's pad (the BMP581 says it does not, until M9). After a warm start
  `sequencer.csv` opens with `<stage>;external` only.
- The supervisor (core 0, a 100 ms grid) runs the checkpoint, the health row and the watchdog. It reconfigures the
  TWDT to panic and subscribes itself alone (§6); its console lines go through the printer's queue, so a long `tlm`
  drain on the console can never starve the feed. `controlling` = armed, launched, airborne and the tick seen, judged
  by the control tick's millisecond stamp; a stall past `stall_ms` logs `watchdog :: control loop stalled
  (stage=X) -> reset`, waits for the printer (at most 100 ms) and restarts. A stall restart's own warm start
  never stall-restarts again (`warmstart::stall_restart_suppressed`), so a stall that returns every boot leaves the
  board stalled rather than cycling it.
- Deferred: the pitot tare restore (M10), the airspeed seed (M13), the BMP581 rebase (M9), `freeze_launch` from a
  GNSS fix (M11), the HITL warm-gate comparison and Q19's step 2 (M12/M14), a BOOSTING crumb whose latch already reads
  separated (M13, with the boost law), a cap on warm cycles (M13/M14), a watchdog through bring-up (later: a guard
  shorter than a failing part's slowest bring-up would turn a slow boot into a reset loop).

**Proof (2026-10-08, as measured):**
- Host: warmstart 58 tests (the gate over every crumb, cause and age boundary, a torn crumb; the stall suppression
  over every reset), golden `warmstart.json` exact; sequencer, controller and board_config as in stage 1.
- The crumb (TMS-7F): about one save a second while armed and launched, 0 failures, the longest save 6.4 ms; a held
  arm saves none; the stamp is 1 s behind `whoami`; recovery (`stage done`, re-zero, `stage setting`) clears it.
- Warm GLIDING (TMS-7F): a console panic in GLIDING boots `gate: recover gliding, 3s after checkpoint (reset_cause
  panic)`, the ICP rebased exactly to the crumb's pad (10.119909 m), `WARM START -> gliding, armed`, then the machine
  walks it to DONE; `degraded` reads `WARM-STARTED (rebooted in flight)`.
- Warm BOOSTING (the taster): the same, rebased exactly to 25.133308 m; the backstop re-based.
- Cold: a crumb at DONE (the bench copy's 60 s flight timeout) gives `crumb reads done -> a new session`; a held
  arm's panic finds no crumb; a nested separation latch refuses GLIDING (the taster).
- The task watchdog (TMS-7F, timeout 5000 ms): `panic kind=twdt` in SETTING ends in `task_wdt: ... supervisor (CPU
  0)`, the panic record reads `task-watchdog`, and the boot is cold; the same in armed GLIDING boots `reset_cause
  task-watchdog` and warm.
- The stall (TMS-7F): `control loop stalled (stage=gliding) -> reset`, then a warm boot (`reset_cause software`)
  with `restart_suppressed: true`, and a second `panic kind=stall` refused. The taster (no watchdog entry) refuses
  both kinds.

**Operator gains.** A mid-air reset resumes the flight, and a reset after landing starts a fresh session. The
board can no longer hang silently.

### M9. SEN0697: BMI323, BMP581, BMM350, and the attitude filter (2 days; DONE 2026-10-08 on the taster and TMS-7F)

**Scope**
- **BMI323** (2 dummy bytes per read, start-up marker wait, INT1 on GPIO4 on v1.x). Accel and rate at p1.
- **BMP581** (re-arm out of standby). Backup baro at p1, reconciled against the ICP.
- **BMM350** (OTP power-off is mandatory, NVS `bmm350_cal` 16 B blob, octant coverage).
- **Attitude complementary filter** (`control::attitude`) at 50 Hz in the control thread. It publishes
  attitude at p1, writes `attitude.csv` with centidegree integer columns, and drives `attitude-blind` and
  `attitude-backup`.
- **Axes:** the IMUs are mounted per their marks, in the body frame (+x nose, +y left, +z up). The optional
  per-sensor `axes` key in the board config covers an exception (§7.2, Q7); it is a schema addition, and old files
  stay valid. It reaches the separation's thrust gate (+x, M7) too.
- **Video start** moves from arm to the attitude trigger: the glider standing vertical on the rod (M7).

**As built (2026-10-08; the plan and its review in the session's m9_plan.md, the operator's answers its §10):**
- **Measured -> reported:** no stream is decimated (`telemetry_ms` is read by nothing); every part runs at the rate
  its consumers need and every conversion is a row: LSM6DSO32 104 Hz, BMI323 100 Hz, BMP581 50 Hz (polled every
  tick), BMM350 12.5 Hz averaging 8 (AGGR_SET 0x37, polled every 40 ms), ICP-10111 every cycle, attitude.csv on every
  publish. Each barometer publishes `change` (m/s, the least-squares slope of its altitude over 500 ms, crates/control
  `change`), registered with its `elevation`; the barometer row has 5 columns.
- **Freshness:** the IMUs by their data-ready flags, the bytes only correcting them (a raced conversion pushed, its
  duplicate skipped; reserved bits, all-zero and 0x8000 frames never fresh), counted as raced/identical with every
  accel/rate channel's consumer-side `lapses`; the BMP581 and BMM350 by changed bytes. The BMP581 reads its burst twice
  and checks its operating range (about 0.1% of its bursts are bus garbage), its ground capture too; the BMM350's
  no-data frame after a re-arm is never judged or observed.
- **The BMI323** runs on its bus owner, first in each wake; its INT1 is counted as evidence (reads paced by the
  tick). The gptimer alarms at twice the tick: the control thread on the even alarms, the bus owners on the odd ones
  (D20), so a backup's push is about 5 ms old at the control read -- needed under the flight's load (the BMI323's
  lapses passed 1 a minute with the video and the ring load running; 0.35-0.55 at rest without). `axes` applies on
  both IMUs (converted values, push and row alike); a mirror fails setup. `imu mute ms=` is the failover's forced
  proof.
- **The multi-source stage machine** runs on the control thread (bound slots, decider by rank, peer veto, physics
  vetoes; vetoes.csv); the BMP581 publishes its elevation.
- **The BMP581** joins the re-zero (`calibrate`, `update ground/rezero`, DONE, the warm rebase). **The BMM350**'s
  calibration is computed on the owner, persisted to NVS by the executor, then applied (plan B's order); bring-up
  restores it; its instruction is worded on the executor.
- **The attitude filter** steps on the control thread every `period_ms`, after the requests and before the stage
  machine; `inspect/probe attitude`, `attitude-backup` and `attitude-blind` as plan B; the executor reads it through
  atomics.
- **The video** starts on the rod trigger: armed, and the fused +x >= 0.87 g for 8 looks (2 s); armed alone on a board
  with no accelerometer.
- **Diagnostics:** `i2c read` (register facts), `inspect databoard` / `stats databoard` (plan B's object), `inspect
  sequencer`; the bus diagnostics run only in SETTING while disarmed, `bustune` armed only back to the configured
  clock; `health` reports `sources-disagree` (information, the operator's Q5: never an arm gate).

**Proof (2026-10-08, as measured):**
- Register facts (stage 0): the BMI323's STATUS 0x02 carries data-ready bits that clear on the status read (flag set
  exactly on the bursts whose data changed, 100 Hz); no SEN0697 part honours the I2C general call.
- Rates (10-11 min at rest, both boards): ICP 8.0, LSM 99.0-99.6, BMI323 99.7, BMP581 50.1-50.2, BMM350 12.5 rows/s;
  `change` sd 0.31-0.33 m/s (ICP), 0.073 m/s (BMP581); LSM lapses at D1's predicted residual, M6's 30 ms race gaps gone;
  a ground step moves the elevation and never `change`.
- Failover: `imu mute ms=5000` hands accel and rate to the BMI323 within 0.4 s and back; the attitude filter never
  goes blind and steps at most 0.26 deg on TMS-7F.
- Axes, by hand on TMS-7F (180 deg yaw both ways, 45 deg nose up/down and each wing down): the LSM6DSO32 reads the
  marks frame; the SEN0697's BMI323 needs `-y+x+z` and its BMM350 `+x-y-z` (180 deg about x), proven end to end
  after the keys (both IMUs read the same tilts within 0.05 g); the yaw-rate sign is positive in a left turn on both.
- The stage machine on TMS-7F's bench copies: peer, at-rest and peer+at-rest veto episodes hold one-baro and
  at-rest altitude launches; with both baros agreeing and no IMU the altitude launch walks to DONE; the axes copy's
  accel launch walks to DONE; held `boosting` + `stage auto` walks to `done;stationary 1.0g`.
- Attitude, 10 min at rest on TMS-7F: 50.00 rows/s, roll and pitch sd 0.02 deg (range < 0.2 deg), heading drifting
  4.2 deg/min free-running.
- The rod trigger started the flight video 1.79 s after the arm on a nose-up copy and never on the flat board.
- The magnetometer refuses a still board and a half turn; its positive calibration needs a full level circle in one
  boot (left for the assembled airframe).
- The soak (TMS-7F, 1 h: colour-bar video at FHD@25, the flight-rate ring load of 200 lines/s, every M9 run): tick
  lateness at most 90 us, no overrun; video 90,026 frames at 25.0 fps, 29 missed, 0 dropped; the ring lost 0 with
  the saver 0.4 s behind; attitude 50.00, BMP581 50.09, ICP 8.04, BMI323 99.44, LSM 99.03, BMM350 12.56 rows/s; the
  internal low-water 41.2 KB with the video running; the largest tick work 1.0 ms (the flash ring's writes stall core
  1 briefly; inside the 10 ms tick). Its one miss, the BMI323's lapses at ~1.5 a minute, went to 0 in 21 min under
  the same load once the owners woke half a tick after the control thread (D20); its longest row gap 51 -> 20 ms, the
  BMP581's 71 -> 40 ms.
- R14, the camera's start on port 0: the BMI323's rows keep their 20 ms, the BMP581's at most 39 ms, the BMM350's
  120 ms.
- `cc_contract --milestone M9` on the taster: 0 violations (32 M9 cases).

**Operator gains.** Primary attitude with a backup IMU on every SEN0697 board, measured on the bench; the stage
machine decides from every source; the video starts when the glider stands on the rod.

### M10. Pitot, power monitor, laser (2 days; DONE 2026-10-10 on TMS-7F and the taster)

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

**As built (2026-10-10, lean: the operator's budget):**
- The SDP810 (every `period_ms`, 20: 50 Hz), the INA226 (100 ms) and the VL53L1X/L4CX (50 ms poll behind the
  data-ready check) run on their bus owners: `dynamic_pressure`/`airspeed`, `voltage`/`current`/`power`, `agl` on the
  databoard, a row of plan B's integers per read (PITOT_FIELDS, POWER_FIELDS, LASER_FIELDS; both laser entries
  record), health per read; `inspect` with plan B's keys; `health.agl` the fresh laser AGL.
- The pitot tare: `calibrate` and `update {"zero": true}` capture it on the owner, the executor persists the shared
  NVS `sdp810_zero` (Pa x 100 i32); bring-up restores it over the config's zero; the crumb freezes it at BOOSTING
  and a warm start puts it in force without persisting.
- The laser lands the machine: it was already bound to the multi-source machine's landing slot (M9), and now
  publishes.
- **The pitot in the stage machine (the operator, 2026-10-10):** a secondary source of speed and pressure, never
  the decider and never a veto alone -- its frontal airspeed is right only in straight flight under about 50 m/s, and
  the past launches' data was not reliable beyond that. With today's two barometers a tie-breaking pitot changes no
  decision, so the machine carries none yet; the rule applies where speed is fused (M13's airspeed estimator). If new
  launches confirm clean pitot data, the pitot can become source #1 while no yaw is actively commanded.
- Deferred: the INA226's over-current ALERT count (with the servos, M13; its column reads 0); per-barometer grounds
  in the crumb (both rebase to plan B's one `pad_altitude`, so the BMP581 carries the inter-baro offset, 5.3 m on
  TMS-7F, after a warm start); M10 contract cases.

**Proof (2026-10-10, as measured on TMS-7F):**
- 50.00 pitot, 10.00 INA226 and 10.11 laser rows/s (longest gaps 20, 109, 100 ms); the laser reads 0.03 m on the
  bench; the INA226 187 mV on USB (no battery), the taster's 4827 mV.
- The tare: `calibrate` -0.04 Pa, persisted, restored across a reboot (tared true); `air_density` 0 and text values
  refused.
- The operator's blow test: the P+ barb (opposite the "1" mark) up to +546.1 Pa -> 30.4 m/s, the SDP810's ceiling
  (32767 counts at scale 60); the other barb down to -137.5 Pa with the airspeed 0.
- The laser lands the machine: bench copy B (no IMU) launched on both barometers, then `landing;agl 2.7m` 0.5 s
  after GLIDING with the barometers still 15 m up, `done;landing timeout 10 s`.
- The warm start: the crumb carried `pitot_zero: -0.04`; a panic in flight came back `airspeed_sdp810 tare restored:
  -0.04 Pa` beside both barometers' rebase.

**Operator gains.** Airspeed, power and laser AGL in health and telemetry; the pitot tare survives a reboot and a
warm start. The servo probe's closed-loop verdict waits for the servos (M13).

### M11. GNSS, gnss_calib, field (2 days; DONE 2026-10-11 on the taster, the outdoor-fix proofs open)

**Scope**
- **`crates/gnss` on uart:2** (TX22/RX23):
  - the NMEA path (ATGM336H, NEO-6M, with PCAS/PUBX/CFG-NAV5 dynModel 8);
  - pad-only sky window;
  - `position` as i32 e7 into the mission's local metric frame;
  - the three legacy CSV streams, with lat/lon text formatted from the integers.
- **gnss_calib:** drift averaging in SETTING, reset at the first armed tick, frozen at BOOSTING.
- **field:** CC-less site selection and auto-arm with the accel gate. Plan B's auto-arm calls the Controller's arm
  directly, past every gate of the CC `arm` (`tasks/field.py:103-104`); the Rust auto-arm goes through the control
  thread's request slot like the CC `arm`, and which of `arm`'s gates it keeps is decided here.
- **`freeze_launch`** (moved from M7, where there was no fix to freeze): `arm` pins the live fix as the launch point,
  and BOOSTING entry pins a fix that arrived after the arm (plan B `warmstart.py:359`). An operator-set position
  still wins. `probe_mission` then reads the live fix (`firmware/src/commands/diagnostics.rs:170, 174` pass none).
- Health gains `position`, and the `flight` block is minimal (attitude, airspeed).
- The UBX NAV-PVT path follows when the NEO-M8N arrives (M16).

**Proof**
- Golden `gnss.json`: `test_gnss.py`'s captured bursts, never `doc/benches/gnss-20261004/raw`. The CASIC and
  UBX command bytes must match EXACTLY.
- Golden `gnss_calib.json` and `field.json`.
- On the taster: `gps taster` on the hub console. A NEO-6M fix with an active antenna.

**As built (2026-10-10/11, lean: the operator's budget):**
- **The GNSS thread** (`firmware/src/gnss.rs`, core 0, PSRAM stack) owns UART2: the driver's commands with plan B's
  gaps, then the NMEA lines into crates/gnss. It publishes `position`/`altitude`/`elevation`/`speed`/`course` and
  writes `<name>.csv`, `_gga`, `_sky` and the durable `_events` (registered through `controller::companions`). The
  engine's 1 s tick runs the outage sweep, the sky window by stage and the sky row. One GNSS, on uart:2 (`buses.uart.<id>`
  through `board_config::uart_bus`); a UART that will not open crashes the entry. `inspect gnss` has plan B's keys
  plus `lines`; `probe gnss` passes when the lines advance in 1.5 s; `health.position` is the live fix, which also
  stands behind `health.launchpad` and the mission probe.
- **A refused configuration is an event (the port's own):** a u-blox answers a sentence it does not take with
  `$GPTXT,01,01,01,<family> inv format`; plan B parsed that text and dropped it. The engine raises one event per
  family (PCAS, PMTK, PUBX) a session. Its first catch: the taster's GNSS is a NEO-6M that ran under an `atgm336h`
  config (switched to `neo6mv2` at 5 Hz, 2026-10-10).
- **freeze_launch:** `arm` pins the live fix as the launch point once the control thread applied the arm, unless
  the operator set one; the checkpoint's BOOSTING freeze takes the live fix when a zone is set and no point was
  pinned. In memory only, as plan B's: the pinned point counts as set from then on.
- **gnss_calib** (`control::drift`): a supervisor duty every `period_ms`, the fresh fused speed and course averaged
  east/north in SETTING, restarted at the first armed pass, frozen off the pad (at least `min_samples`, else zero).
  `gnss::drift()` hands it to M13's wind de-bias through atomics. `min_samples` 0 with no sample freezes zero, where
  plan B divided by zero.
- **field** (`mission::field`, `Mission::select_site`, `Mission::fallback_zone`): the executor ticks it between lines,
  since it owns the mission. It waits with `recv_timeout` only while a field entry runs. **The auto-arm keeps the
  CC `arm`'s passive gates (the operator, 2026-10-11):** the devices, the stage, the mission probe and the recorder's
  ring gate, never the probe sweep (it moves the fins with nobody there). A refusal logs every problem and restarts
  the dwell, so the agent asks again after another whole one. A knob plan B could not fly with fails the entry,
  naming the key.
- **health.flight:** the fused attitude (whole degrees, half to even) and airspeed (0.1 m/s, the pitot until M13's
  estimator), absent without either provider.

**Proof (2026-10-10/11, indoors: no fix):**
- Golden `gnss_calib.json` (25 cases) and `field.json` (31 cases), built by running plan B's own `run()`/`_tick()`;
  the still band's exact edges are left out (f32 and f64 round 1.3 g to opposite sides of it).
- The taster: ~6 NMEA lines/s, the sky rows every ~10.2 s once configured, no refusal events, `probe gnss` passing;
  an operator hold into GLIDING froze gnss_calib with plan B's `drift 0.000, 0.000 m/s (0 samples)`; `field`
  enabled for the check came up and the executor answered as before; contract core + inspect pass, 0 violations.
- TMS-7F: the NEO-6M streams NMEA but never takes its configuration. A UART1 receiver attached to GPIO22 heard no
  byte while UART2 transmitted (on the taster it read every command back exactly), and the module never answered
  `$PUBX,00`: the GPIO22 -> NEO RX line, a hardware check (the operator).
- **Open, outdoors:** a real fix through the launch-point freeze, gnss_calib's samples, the field's site pick and
  auto-arm, and `gps taster` on the hub console.

**Operator gains.** Position in health and telemetry, the pad drift calibration, and CC-less field arming.

### M11b. DONE and the recovery beacon (1-2 days; before the first passive flight; DONE 2026-10-11 on the taster, the battery time open)

**Scope**
- **At DONE:**
  - the non-essential sensors stop (the IMUs, the pitot, the fins); the camera already stops there from M7. The GNSS
    drops to 1 Hz, and the other sources the beacon page shows (the baros, the laser, the INA226) keep a slow rate;
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
- The link thread brings the AP up: it is the radio's one owner, and after the detected launch it never rejoins
  (M7). The switch from station to AP goes through esp_hosted on the C6, the restart path M7 chose not to take in
  flight, so it is proven on the taster first and on 7F with the operator present.
- Power-off or reset ends it. Any power-up after DONE is a new session in SETTING (§4.6), and every boot resumes
  the flash→SD copy when a card is present.

**Proof**
- On the taster at DONE: a phone sees `beacon_taster`, and the page shows the fix with a working Maps link, every
  elevation source, the battery figures and the flash→SD progress falling to 0. The camera has stopped and the
  GNSS rows come at 1 Hz.
- A power cycle after DONE comes back in SETTING as a new session, with the copy resumed.
- The battery time at DONE is measured on the INA226 (about 1.5-2 h expected on 800 mAh).

**As built (2026-10-11, firmware/src/beacon.rs):**
- At DONE the link thread ends any hub session (within the hub's ~2 s poll) and turns the C6 into an open AP
  `<BOARD>_glider`, channel 3, 21 dBm, up to 4 phones; the page on port 80 at 192.168.4.1 refreshes every 5 s, and
  opens by itself (a captive portal, operator 2026-10-11): the DHCP server offers the board as the DNS server, a DNS
  responder answers every name with the board, and every other path redirects to the page, so a phone's connectivity
  check finds a portal. No DHCP option 114: phones take it only as an HTTPS captive-portal API (RFC 8908). The fix shown is the newest any source pushed, whatever its age (a fix lost in the trees
  is still the best guess); the heights are each source's latest with its age; the battery is the INA226's voltage and
  current with a 1S LiPo charge estimate, and the hours left when the INA226 entry gives `battery_mah` (none above
  4.35 V: USB). Under the fix, a map link to the launch pad (the mission's launch point), there even before a fix.
  The file server serves the same page as `/beacon`, so the bench sees it before DONE.
- A HITL board never beacons: its frames ride the Wi-Fi, and the host restores it after DONE. A bench `stage done`
  beacons as a flight does; the AP and the GNSS's 1 Hz hold until power-off or a reset.
- DONE quiets the board: the IMUs (control's and the owner's), the magnetometer, the pitot and the attitude filter
  stop; the barometers, the laser and the INA226 run once a second; flight.csv takes a row a second; the GNSS module
  is reconfigured to 1 Hz; the fins lose their pulse a second in, after the law's centring (no holding current).

**Proof so far (2026-10-11, the taster, `stage done` on the console):**
- The fins rest at +1.0 s, the GNSS at 1 Hz at +1.2 s, the beacon open at +2.0 s
  (`beacon :: taster_glider open on channel 3, the page on http://192.168.4.1/`); the page renders every section.
- The recording at DONE, over 272 s: the IMUs (99/s each), the pitot and the attitude (50/s), the magnetometer (12/s)
  to 0; the BMP581 50/s to 1/s, the ICP-10111 to 0.5/s (its two-phase read), the laser, the INA226 and flight.csv to
  1/s: ~460 rows/s to ~6.
- A reset after DONE comes back in SETTING as a new session (000660 -> 000661) with the flash->SD copy running.
- The operator's phone joined `taster_glider` and read the page at 192.168.4.1 (the first build's portal answered
  the phones' probes with 404, so it had to be opened by hand; fixed). The page carries a map link to the launch pad
  always, and to the last fix once one came.
- Still owed: the battery time on a LiPo (an airframe, not the USB taster).

**Operator gains.** **Passive-flight ready.** This covers the flight-test ladder's second rung (passive
electronics E16/F15, no active fins): stages, data, video, warm start, pull, and a beacon that leads to the
airframe after landing (even without a fix, the phone's signal strength helps).

### M12. HITL over a UDP sim-link, realtime and passive (2 days; DONE 2026-10-11 on the taster)

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
- **The link and the gates (from M7):** plan B's HITL config turns the radios and the separation off
  (`config_hitl.py:6-7, 30`); here the sim-link and CC ride the Wi-Fi. A simulated launch is a detected launch, so
  a hub link that drops mid-scenario comes back only with `stage setting` from the console or the next boot. HITL
  configs disable the entries the firmware cannot run yet, so `arm` passes. The separation pin is not simulated:
  the thrust gate stays proven on the host and by the first flight.

**Proof**
- A passive scenario flies to DONE with the warm gate passing.
- Artefact counts are checked: streams pulled = streams expected, no empty stream, BUILD stamp present.

**As built (2026-10-11):**
- `doc/specs/sim-link.md` and `crates/sim_link`: UDP beside CC, the board on `sim_port` 1236; a 104-byte SENSORS
  frame per physics publish (12 quantities in the Rust databoard's units, presence flags for faults and the laser's
  range), a 32-byte FINS answer to each applied frame (stage, armed, launched, the fins or none); CSL1, version 1,
  CRC-32 trailer; frames refused for their length, magic, version, kind, CRC or a non-finite value, and ordered by a
  wrapping sequence. The host codec is pinned to the crate's bytes (test_tools).
- The board: `board_config::mask_simulated` after the layout (`_SIM_SENSORS`, by name), the `hitl` entry refusing to
  come up while one still runs; its 13 quantities at p0 / 1 s (the stage machine binds it unchanged); `sim_link.rs`
  (core 0, priority 12) binds once the TCP/IP stack is up (`link::stack_up`: a socket before it asserts in lwIP),
  stamps on arrival, derives `change`, records the simulated sensors' own streams, answers each frame;
  `inspect hitl`.
- `tools/hitl_host.py`: the board's own saved config made HITL over CC (not-ported entries removed, separation,
  watchdog, bluetooth, field off), the mission seeded, the sim-link gate (a frame answered before flying), plan B's
  realtime accumulator on the host clock, the board's stage choosing boost or glide; restored after. CC goes through
  the hub: the console splits long replies with log lines. `tools/hitl_collect_host.sh` pulls each session with
  `board_pull.sh` (every file against the board's listing, no empty stream), checks the BUILD stamp, cuts and plots.
  `tools/hitl_overlay.py` draws a Rust flight over a plan B capture, aligned on BOOSTING.
- **The real reboot (`--reboot`):** the console panics the board 1-6 s into the glide. Two HITL-only rules make it
  fly on: a board running the sim-link joins the Wi-Fi again after the launch (it carries the simulated sensors; a
  flying board never does), and its glide counts as separated for the warm gate (no separation is simulated; plan B's
  harness passed `separated=True` by construction).

**Proof (2026-10-11, the taster, F15 clean, 285 g; doc/sims/TMS-7-rust_hitl_m12):**
- Passive to DONE, twice (sessions 000547, 000554): BOOSTING at 3.0 g, apogee 273 m, LANDING on the barometric
  4.2 m, DONE stationary; the runs agree to 0.2 s; 5755 of 5755 frames answered on the second.
- Against plan B's control stack on the same physics (`virtual_flight`, equal mass): the same 3.0 g, the same
  273.3 m apogee, GLIDING 10.1 s vs 10.0 s after BOOSTING; the landing differs only by the passive glide.
- The warm gate: `WARM GATE: True recover gliding, 3s after checkpoint (reset_cause panic)`, the restored board
  rejoined and landed on time (sessions 000556 + 000557); the outage cost ~26 s of frames (boot + join).
- Artefacts counted by the collector: every session file against the board's own listing, no empty stream, the
  firmware stamp in recorder.log.

**Operator gains.** Bench flights of the Rust board without hardware motion, with the same harness output as
before.

### M13. Active control (2 days, possibly split: crate one day, integration one day; DONE 2026-10-11 on the taster)

**Scope**
- `crates/control`: pid, mixer, governor, airspeed, wind, guidance, and the rest of navigation (its subset
  came in M1), with the golden fixtures.
- Servos on LEDC at 50 Hz with ≥14-bit duty. Pins are held LOW until configured, and boot centring is
  staggered by `fins.concurrency`.
- `set_angle` compare-and-set, the fin probe sweep as a control-thread state machine (SETTING, disarmed only),
  and `update <servo name> {trim|angle}` (SETTING-only, like every `update`). The fin requests reach the control
  thread through request slots like M7's hold and arm. The sweep checks the stage again before each fin, where it
  is served, and stops once the flight word says launched (M7: a launch detected during an `arm` or `verify`
  sweep must not move a fin).
- `verify`'s readiness gains `fins` (plan B's `_readiness`).
- `flight.csv` at 10 Hz with plan B units.
- `health.flight` vitals complete: reach, r_min_m, fins, wind, heading_error.

**Proof**
- Golden `pid.json`, `mixer.json`, `fin_limit.json`, `governor.json`, `airspeed.json`, `wind.json`,
  `navigation.json` and `guidance.json`.
- Golden `control_trace_<scenario>.json`: virtual_flight (`VF_SEED`) per-tick inputs replayed open-loop. Stage
  transitions EXACT, fins within 1 deg.
- On the taster: the dashboard fin-zero card (reads `props.trim`, nudges, saves).

**As built (2026-10-11):**
- `crates/control::flight::FlightLaw`: plan B `tasks/flight.py`'s `_tick` in its order (the airspeed estimator and
  the governor's fin cap, guidance with the wind fed per GNSS course stamp, the PIDs with the gyro D term, the mixer);
  `step` takes one tick's fresh inputs and says whether it actuated: a law that never engaged writes no fin on the
  pad, so an operator's fin command stands; leaving control centres once.
- `firmware/fins.rs`: LEDC 50 Hz, 14-bit duty; the pins LOW from config load; boot centring staggered by
  `fins.concurrency`; the control thread owns the fins (`update <servo> {trim|angle}`, the probe sweep and the law
  reach them there). The probe sweep is bounded by `fins.mixer.limit_deg` (neutral ± limit inside the servo's range,
  operator 2026-10-11: plan B's 0..180 needed the wings open), judged by the INA226's power rise.
- `firmware/flight.rs`: the flight loop ticks after the stage machine on the control thread, reading the databoard
  (attitude, rate, accel, GNSS speed/course/fix, the pitot, AGL, elevation) and the mission's zone and launch point
  through a sequence-locked view (no mutex on the flight core); every 100 ms plan B's `flight.csv` row and the
  vitals `health.flight` shows (plan B's keys: airspeed, fin_cap, active, reach, r_min_m, attitude, fins = the
  mixer's commanded angles, heading_error, wind). A mixer surface with no servo is logged at boot and `verify`'s
  readiness says `fins: no driver for X -- reduced control authority`. The FINS answer carries the real fin angles.
- The warm start seeds the law's airspeed from the crumb, and the crumb keeps the law's estimate.
- The control thread's stack is 8 KB (was 6): with the loop's state in its frame, an IMU probe's float formatting
  overflowed 6 KB; the loop is boxed and its `new`/`tick` never inlined. Floats are not formatted on that thread.
- `tools/hitl_host.py` flies the board's loop with `config_hitl`'s test gains; `tools/hitl_overlay.py` draws the
  law's roll setpoint and command, a fin, the zone and the touchdown miss.

**Proof (2026-10-11, the taster; doc/sims/TMS-7-rust_hitl_m13):**
- Every golden above, and `control_trace.json`: 16175 ticks of two closed-loop virtual flights replayed through
  `FlightLaw`, the fins and the cap within 1 deg on every tick.
- Active HITL, F15, 285 g, against `virtual_flight` at equal mass: the setpoints, commands, fin angles and the
  figure-8 loiter overlay plan B's; clean 39.5 m and 41.5 m from the zone centre (plan B 49.2 m), wind 6 m/s 37.3 m
  (plan B 55.8 m), all in the zone; the wind estimated 6.11 m/s from 30 deg (plan B 6.00, true 6 from 30).
- A real mid-glide reboot: the warm gate restored GLIDING, the law's first row read the seeded 1400 cm/s, DONE.
- The fin-zero card's own requests: `inspect` gives a numeric `props.trim`; `update servo_yaw {trim}` moves the fin
  live (1511 -> 1522 us) and a word is refused (`trim must be a number`); the save (`get-config saved`, the trims
  patched, `set-config board`) survives a reboot.
- Negative: `servo_yaw` disabled -> the boot warning and the readiness `fins` problem.

**Settled from earlier milestones:** the airspeed seed (M8), the servo probe's closed-loop verdict (M10), `fins` in
the readiness gate (M7), the complete `health.flight` (M11), the D term reading `rate` (carried from M6).
**Still deferred:** the INA226's over-current ALERT count; a per-fin probe verdict on an INA3221 (one channel per
servo; ordered); a cap on warm cycles and a BOOSTING crumb that already reads separated (M14); the fast warm boot
(next: flight sensors and the control thread first, the camera and the Wi-Fi after them without blocking, operator
2026-10-11).

**Operator gains.** Active control in HITL. The fin-zero calibration UI works unchanged.

### M14. HITL acceptance and SITL (2 days; DONE 2026-10-11 on the taster and the host)

**Scope**
- Fly the 12 `hitl_matrix.sh` scenarios × {E16, F15} on the taster (realtime).
- Lockstep mode with an injected `Clock`.
- `src/avionics/sitl` on the host behind the same sim-link.

**Proof.** `hitl_compare.py` against `doc/sims/TMS-7-postaudit_regression` r1..r3. Each scenario and panel must
stay inside its family's own run-to-run spread (wind median miss 7.7 m, low-noise 11.9 m; the high-noise and
corner families are not gated on miss). All silent-success guards are kept.

**As built (2026-10-11):**
- **The reference is plan B's own code on today's sim, not the post-audit runs** (a divergence from the proof
  above): those board runs flew the sim of 2026-08-30, before the measured polar and the endgame work, and only
  their reduced numbers are kept. `tools/virtual_flight.py` is no reference either: its loop feeds no wind estimator
  and hands its law the clean total speed, where plan B's on-board HITL (`tasks/hitl.py`) publishes the noised ground
  speed and track. `tools/planb_sitl.py` runs plan B's Controller, `tasks/sequencer.py` and `tasks/flight.py` under the
  golden shims behind the sim-link; fed the same frames it flies the Rust stack's flight. The post-audit study's
  family spreads stay the miss tolerances.
- **Lockstep is the SITLs'**: `tools/hitl_host.py --sitl` flies a host stack frame by frame, its clock the frame's
  (doc/specs/sim-link.md); the board stays realtime on its gptimer.
- **`src/avionics/sitl`** composes the board's crates with the firmware's own plumbing, moved into crates/controller
  (the topology from the config, `HitlWriters`, `StageSources`, `FlightQuantities`), so the host cannot fly other
  plumbing than the board.
- `tools/hitl_matrix_host.sh` (the board's matrix, `hitl_matrix.sh`'s table, artefacts counted) and
  `tools/hitl_matrix_compare.py` (`reference`, `sitl`, `compare`; gates: the apogee within 3 %, GLIDING within 0.5 s
  and DONE everywhere, the miss on the wind and low-noise families within plan B's seeds widened by 7.7 m and
  11.9 m). `tools/recorder_flight.py` anchors a cut on the BOOSTING row when no |a| holds 3 g (the sim's boost).

**Proof (2026-10-11; doc/sims/TMS-7-rust_hitl_m14):**
- SITL, 3 seeds × 12 scenarios × 2 motors: **72 of 72** Rust flights inside plan B's; seed for seed GLIDING never
  differs, the miss differs by a median 0.3-1.1 m outside the high-noise family (where the flights fork).
- The taster, run r1, realtime: **23 of 24** pass. E16 corner_stress's apogee came 0.9 s early once; three re-flights
  and 10 seeds on each SITL stayed inside plan B's range. Not gated by the family rule; recorded.
- The board on the shared plumbing: F15 clean 38.7 m in the zone, as before the refactor.

- **r2 (2026-10-12):** 24 of 24 pass against 20 plan B seeds. With 3 seeds it failed two gates. E16 noise25 missed by
  156 m; plan B's 20 seeds give 72-189 m. F15 noise100 reached GLIDING at 7.8 s; plan B's give 7.5-10.5 s. r1 against
  r2: the wind family repeats within 2.7 m median and 9.8 m max, and the stage machine within 0.15 s outside high
  noise (doc/sims/TMS-7-rust_hitl_m14 §3b).

- **The drops on the SITLs (2026-10-12):** the sim now publishes the rate and the glide's accel in the marks frame, as
  the IMU drivers do (doc/specs/sim-link.md). The attitude drop then lands with the no-drop flights: the filter
  carries the glide, where it landed ~500 m off before. The GNSS drop agrees with plan B within 13 m. On the new
  frames both SITL matrices pass 72 of 72, seed for seed as before (doc/sims/TMS-7-rust_hitl_m14 §3c).

**Not here:** the reboot scenario on the SITLs: it needs the warm start, a board's (realtime only).

**Operator gains.** Evidence that the Rust board flies like plan B, which is the precondition to migrate 7F.

### Later milestones (not on the critical path)

| id | scope |
|---|---|
| M15 | Firmware OTA over CC into the `ota_0`/`ota_1` slots that M3's partition table already has (esp_ota; SETTING-only and disarmed). Replaces `push-*` for the Rust image (§7.2, Q5). **No app rollback, ever** (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=n`, pinned): with rollback, a build that has not yet confirmed itself is reverted at the next reset, so a reboot in flight could come back on the OLD build. A reset always boots the slot otadata selects; the slot changes only by an OTA in SETTING followed by a reboot on the ground, and `arm` refuses when the running slot is not the selected one. USB flashing (`flash.sh`) writes ota_0, erases otadata and invalidates ota_1, so no stale build can be selected. **DONE 2026-10-12 on the taster.** `firmware/src/ota.rs`, `tools/ota.py`. Two transports: the console (the operator: more reliable than Wi-Fi for big transfers; speed is no matter, progress in CC's log every 10 %) -- `console baud rate=2000000` (the receive interrupt at 32 bytes), `ota begin size= sha256=`, `ota write offset= crc= data=<base64url>` lines of 192 bytes (a whole line must fit UART0's receive ring and FIFO, ~384 B: 4 KB lines failed their CRC every time; a damaged line writes nothing and is sent again), `ota end`; and Wi-Fi, `POST /ota?sha256=` (it starves the CC link for seconds). **No flash mapping anywhere:** the slot is erased a 64 KB block ahead of SPI1 writes (esp_partition), checked by our SHA-256 and its header and app description read back, and selected by writing otadata as IDF's esp_rewrite_ota_data does (the alternating sector, the sequence number, its CRC). esp_ota_end and esp_ota_set_boot_partition map the image to verify it, which turns the cache off and parks the flight core: on a PSRAM stack it asserted, on an internal one the chip's watchdog reset the board in the verify. `ota commit confirm`; `arm` refuses while running != boot (both read by SPI1 at boot). Proven: 3.0 MB over the console in 95 s, 0 lines resent, ota_0 -> ota_1 -> ota_0, the board booting the build each time. **Open:** the image is 3.0 MB of a 3 MB slot (~100 KB left): the slots must grow before long (cfg and panic move; the settings migrate). |
| M16 | NEO-M8N UBX NAV-PVT (CFG-PRT baud-up, autobaud after a P4 reboot). |
| M17 | — (the post-landing recovery beacon moved forward to M11b, before the first passive flight). |
| M18 | Legacy parts as needed: BNO055, BMP280 (the SEN0253 on the v0.1 boards, 7C and 7D). The ADXL375 is off the table: no longer fitted, it stays unported (operator, 2026-10-12). **Host side DONE 2026-10-12; the proof on a board waits for 7C.** <ul><li>BMP280 (`crates/drivers/src/bmp280.rs`): plan B's driver with the BMP581's guards (double-burst read, freshness, guarded ground capture, rezero, re-arm); run as the BMP581, recorded as `baro_bmp280.csv`.</li><li>BNO055 (`crates/drivers/src/bno055.rs`): the Euler triple to `attitude` (priority 2, below the filter), the accelerometer through `axes` to `accel`; the stall judge withholds `attitude` while the part rotates with a frozen Euler, so the channel goes stale and the databoard moves on. The CALIB_STAT latch, `calibrate imu_bno055` persisting the learned profile in NVS (`bno055_calib`, plan B's key), the profile written back at bring-up. Shown in `inspect`, in health's `imu_calibration` (CC's IMU column) and as verify's `imu_calibration` readiness item; recorded as `imu_bno055.csv` in plan B's columns.</li><li>On 7C: both parts come up when the layout's scan finds 0x28 and 0x76; `probe`; the BNO055's figure-8 to mag 3, `calibrate imu_bno055`, a power cycle that restores the profile; a hand rotation with the filter's gyro gone, the databoard on the BNO055.</li></ul> |
| M21 | **Entry file: GCL at boot (operator, 2026-10-06; Q20).** `/sdcard/entry.gcl`: one GCL line per line, `#` comments. Once bring-up is done, each line runs in order through the executor (origin `entry`, the one point of injection), under the normal gates, so changes still run only in SETTING. Each line and its answer go to `recorder.log`. It replaces MicroPython's run-any-script flexibility for bench tests, soaks and HITL setup (for example `ring load seconds=600`, `video start`, `pace seconds=60`). Safety: while an entry file is present, `verify` and `arm` refuse with `entry-file`, so a bench mode never rides into a flight. Small; after M4. **DONE 2026-10-11 on the taster** (`firmware/src/entry.rs`): origin `Entry` (a console-only command is refused: the hub can pull the card); the runner thread starts 1 s after bring-up, one line at a time, each line and answer logged (`entry :: N > line`, `entry :: N < answer`). Written over CC, no card reader: `entry add text=<line>`, `entry show`, `entry clear confirm` (SETTING; `board_clean.py` keeps the file). Proof: three lines added over the hub, `verify` and `arm` refused with `entry-file`, the reboot ran them 2 s after bring-up (`video preview` on, `wifi` refused as console-only, `health` answered), `entry clear` lifted the refusal. |
| M19 | **uart_saver: the remote recorder (operator, 2026-10-06).** The purpose: the data survives when the MCU and its SD card are both damaged. <ul><li>The ring_saver already decodes each block into wire lines (`[@routing@]{OPEN};payload;<CLOSE>`, binary → text + CRC) for the SD. It also hands the same lines to the uart_saver, which sends them out of a UART TX in its own time. One decode, two sinks.</li><li>These are the bytes plan B sent to the Luckfox, so the frozen recorderd is the receiver as it is. The Luckfox is the test bench now. Later it becomes a physically separate flight recorder: a C3/C6 with its own battery and SD card, which could also carry the DONE beacon.</li><li>The handoff never blocks the flash→SD path:<ul><li>a bounded PSRAM byte ring (SPSC: saver in, uart_saver out);</li><li>the saver pushes without waiting. When the ring is full, the oldest undelivered lines go (recency first: the last seconds before an impact matter most), counted;</li><li>the uart_saver runs its own thread below the saver, and shares no lock with it.</li></ul></li><li>Per block, the saver hands the lines to the uart_saver before its SD write, so an SD stall does not delay what is already decoded.</li><li>With no card, the saver still walks and decodes for the uart_saver. It uses a cursor of its own: `saver_position` keeps meaning "on the SD and fsynced", so the eraser never takes a block as saved because the UART had it.</li><li>Lag: the uart_saver trails the saver, not the writer. The saver keeps up by a wide margin (≈ 26 KB/s of text against an SD that takes MB/s), so normally the lag is a block or two.</li><li>Resends after a gap or a reboot are duplicates the tools drop (§3.8).</li><li>Hardware: its own battery, and only TX + GND to the main board, so a short or brown-out there does not take it down. Mounted apart from the main board.</li><li>Link sizing: plan B's link (UART1 TX GPIO20, 921600 baud) carries ≈ 92 KB/s, against ≈ 35 KB/s of routed text at the v0.1 flight rate (§3.13): about 38 % busy, so normally nothing is dropped. 1 kHz IMUs (about 3×) need ≈ 2 Mbaud.</li><li>Open: the UART pins on the v1.x boards.</li><li>**Status 2026-10-11 (stage 1 + fixes).** The tee runs on the taster, UART1 TX GPIO20 at 921600, into the C6 receiver (M20). <ul><li>The tee names the session of a log line and a session.csv line (`<session>_recorder.log`, `<session>_session.csv`, the body unchanged with its checks under its own routing): a receiver that took the session from the last routed line filed a new boot's first lines under the boot before.</li><li>The uart_saver lives off internal RAM (a PSRAM stack, and no driver transmit ring): on internal RAM it cost the board its Wi-Fi (associated, but no DHCP request ever left).</li><li>A `reboot` waits (at most 1 s) until the queue is sent: the saver tees a part when it stages it and the card has it a commit later, so a part committed but still queued at a reset never reached the receiver (~1.5 s lost at a reboot).</li><li>**Stage 2 DONE (2026-10-11):** the uart_saver walks the ring itself (crates/recorder `Relay`), its own cursor, card or no card; a part counts sent once its bytes have left the UART (`uart_wait_tx_done`), published as `relay_position` and kept as the `uart` NVS hint, so a reset resends from there and never skips (the receiver drops what it has). The eraser erases behind the earlier of the saver and the relay (`erase_horizon`). More than 64 blocks (~10 s) behind the writer it jumps forward, counted (recency first). The saver's tee, its queue and the reboot drain went. Proof on the taster: a session ended by a panic is identical on the C6's card, 21 of 21 files.</li></ul></li></ul> |
| M20 | **The separate flight recorder and its beacon (operator, 2026-10-06).** The operator builds a C6 + battery + SD card box; the receiver firmware is Rust (esp-idf on the C6, riscv32imac) and reuses `recorder_wire` (line verify, routing). It plugs in where the Luckfox sits now, and the boxes can be swapped. <ul><li>Receiver: reads the uart_saver's lines (M19), verifies each line's CRC, and writes the session layout that recorderd writes (§3.10) to its own SD card.</li><li>Beacon by data, not by protocol: the main board adds an ordinary recorder stream, `beacon.csv`. Its rows are what a finder needs, about once a second and on every change: the last good fix (latitude, longitude, GNSS altitude, fix age), every elevation source (each baro, laser AGL), the stage, and the battery voltage where an INA226 is fitted. The P4 keeps writing it through the flight and just before.</li><li>The receiver knows that one stream by its routing (`beacon.csv`) and keeps the latest row. When it beacons, it serves that position: the same page as M11b (last fix, a Google Maps link, the elevation sources) from an open AP `<BOARD>_beacon` on channel 6, at maximum TX power.</li><li>The P4 controls it (operator), all through `beacon.csv` lines:<ul><li>when it starts;</li><li>the network name: `<BOARD>_beacon`, beside the P4's own `<BOARD>_glider`;</li><li>the channel: low channels, 6 for the recorder and 3 for the P4, so the two never share one;</li><li>the page data: position, elevation from every source, stage, battery.</li></ul></li><li>It starts BEFORE launch: once the glider is installed on the rod, the P4 turns on the camera and the recorder's beacon together (the rod trigger, M9; until M9, `arm`). A P4 that dies in flight no longer matters, because no trigger is owed after launch.</li><li>Once on, it stays on until power-off. The on state and the parameters are kept in the recorder's NVS, so a recorder that reboots (an impact, a brown-out) beacons again at once, under the same name and channel. Only the P4's next session in SETTING turns it off.</li><li>The page shows the latest `beacon.csv` row: the pad position before launch, the last fix after.</li><li>Battery (ESP32-C6 datasheet v1.5, Tables 5-7 and 5-10):<ul><li>A SoftAP keeps its receiver on, so it cannot modem-sleep. Wi-Fi RX is 78 mA at any CPU clock (80 MHz is the floor for the Wi-Fi PLL). The beacon frames add about 4 mA (a ~1 ms frame at 1 Mbps every 102 ms, 354 mA at 21 dBm), so the AP draws ≈ 80-85 mA.</li><li>The modem-sleep figures (14-38 mA at 80-160 MHz) hold only with Wi-Fi clock-gated: a station between DTIMs, or the recording phase before the beacon starts.</li><li>A 120 mAh LiPo (about 85 % usable) gives ≈ 1.2 h of continuous AP, short of the 1.5-2 h search target.</li><li>Chosen (operator): Wi-Fi 1 minute on, 1 minute off. On average ≈ 41 mA plus the off-minute's draw: about 0.2 mA in light sleep, or 14 mA with the CPU idle in modem sleep. A 120 mAh cell then lasts ≈ 2.1-2.4 h, against ≈ 1.2 h always-on. The AP stays on while a phone is connected, and a searcher waits at most a minute to see it. That fits how a search goes (operator): you walk to the approximate location and circle the trees nearby, so sooner or later an on-minute is caught. The parts are more often in the grass or stuck in a tree, which is why the page shows every elevation source beside the position. A 300 mAh cell remains the option for longer searches.</li><li>The beacon frames add little either way: the receiver is the cost.</li></ul></li><li>The same `beacon.csv` rows can feed the P4's own beacon at DONE (M11b), so both beacons show one data set.</li><li>Two beacons by design: two radios, two antennas, two batteries, two mounting spots. One that is buried, shadowed or flat does not hide the glider. Both run at once from DONE (the recorder's has been on since the rod). Each has its own SSID and channel (`<BOARD>_glider` on 3, `<BOARD>_beacon` on 6), so a phone sees which one answers and the signal strength of each, which helps walking in on the glider.</li><li>**Status 2026-10-11 (stages 1-3 on the taster + the C6 SuperMini).** <ul><li>Receiver (`src/avionics/recorder`, package `flight_recorder`): files each line where the P4's ring_saver files it, byte for byte: a session closed by a reboot matched the P4's card in all 20 files. A line the P4 sends again after a reboot (no newer than its file's last: every file's lines rise in uptime) stays out; a header printed again stays out of a file that holds one (the global index: one per session); a torn last line gets its `\n`; a damaged line goes raw to the session's `damaged.log`. Console (below the link's loop; `tools/recorder_console.py` is the host's end, its port found by its USB product): `ls [dir]`, `df`, `rm <path>` (never the root nor the session being filed), `get <path>` (base64 lines, checked by size + CRC-32), `files [dir]` (size + CRC-32 of each file; `pull <dir>` gets them all) and `read <path> <offset> <length>` (hex). Its lines go through newlib's locked `stdout`, as the logger's do: written past that lock (`println!`), a reply landed inside a status line.</li><li>Card: the WWZMDiB SPI module has its own 3.3 V regulator, so its VCC goes to the SuperMini's 5V; on 3V3 the card answered CMD0/CMD8 but never left ACMD41's idle. A mount is read back before any write. Flash with `recorder/flash.sh` (its own table: a 3 MB factory slot, the image is ~1.5 MB).</li><li>Beacon (crates/beacon, one page and one report for both ends): the P4 writes beacon.csv every 5 s (radio: name, password, channel, TX cap, on/off cycle; page: fix, pad, every elevation source, laser AGL, battery, copy backlog). The name stays empty, the beacon off, until the rod trigger fires (armed in SETTING, nose up 2 s: the video's `Trigger`) or the stage passes SETTING; then `<BOARD>_beacon` on channel 6 (the glider's 3 + 3) at 20 dBm, 60 s on and 60 s silent, until power-off. Never in a HITL flight. The receiver keeps every report in its card's root `beacon.csv` and replays the last one at boot (the operator, 2026-10-11, in place of NVS): proven, on the air 0.6 s after a reset. Its page is the glider's own, with how old the report is.</li><li>Open: the battery time on a LiPo; the field range of the two beacons; **a hardware page for the flight recorder (operator, 2026-10-11): written 2026-10-12, `doc/flight_recorder.md`, the photos to come**: its pinout (P4 UART1 TX GPIO20 + GND to the C6's GPIO17 RX; the SD module on SPI, CS/SCK/MOSI/MISO on GPIO 18/19/20/14, VCC on 5V) and its composition (the C6 SuperMini, the WWZMDiB SD module and its regulator, the LiPo on the charging pads, the ME2108 3.7->5 V boost for the card, the battery switch, the 470 uF option), with photos.</li></ul></li></ul> |

---

## 3. Storage design

The operator's ring (decided 2026-10-06). Power can drop at any moment, so getting every record into flash fast
is the critical path; the SD copy comes second.

### 3.1 Data path

```
producers (any core)                          core 0, prio 7          core 0, prio 4
 control / i2c0 / i2c1 / gnss / supervisor      ┌─────────────┐ chunks  ┌────────────┐ text + CRC
 / executor  ── binary records ──► PSRAM SPSC ─►│ ring_writer │───────► │ ring_saver │───────────► /sdcard/<session>/recordings/
 log bridge  ── (one ring per thread)           │ (programs)  │ 24 MiB  │  (reads)   │
                                                └──┬──────────┘ `log`   └─────┬──────┘
                                       CC tees ◄───┘            ring ▲        │ saver_position
                                                                     │  ┌─────▼───────┐
                                                               erase └──┤ ring_eraser │ core 0, prio 3
                                                                        └─────────────┘
video: encoder (prio 10) → 2 MB PSRAM ring → SD writer (prio 6) → /sdcard/videos/<session>/   (independent)
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
   `cc` executor, for the tees) turns them into text. The one exception off the pad is the control thread's log line for an
   event it decides itself: a stage machine transition (with its reason text, six or fewer a flight; M7) and a
   separation-switch event (M6). An operator's hold, resume, arm or disarm is logged by the executor from the
   request's result (§4.4); the IMU's probe and `bustune`, whose verdicts the control thread does write as text, run
   only in SETTING. A thread that refuses a change off the pad answers with the stage as a value
   (`sequencer::ChangeRefused`), and the executor writes the refusal.

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

- **Partition:** `log`, found by label: 0xC00000–0x2000000, 20 MiB (§3.10; 24 MiB at 0x800000 until 2026-10-12). It is accessed only through
  `esp_partition_read`/`write_raw`/`erase_range`, and never mmap while writes run.
- **Blocks:** 5120 × 4 KiB, one erase sector each (6144 until 2026-10-12).
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
- Route it as recorderd did (`recorderd.cpp:153-182`), into the session's recordings directory
  (`<session>/recordings`, §3.10):
  - a stream line to `<session>/recordings/<session>_<file>`;
  - `session.csv` to **both** `<session>/recordings/session.csv` and the global index `/sdcard/sessions.csv`
    (its lines keep the routing `session.csv`; only the file's name differs);
  - a LOG line to `<session>/recordings/recorder.log`.
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
`max_files` is 32 on the recordings volume (the saver and HTTP reads) and 8 on the videos volume (the video writer
and HTTP reads), 40 on a one-partition card's only volume; `CONFIG_FATFS_FS_LOCK` (one table for every volume) is
40, their sum, and `sd_card.c` refuses to build below it. Nothing on the link or health path touches FatFs.

### 3.10 SD and flash file layout

**Session first, on one partition (operator, 2026-10-06; one partition from 2026-10-11, the HITL A/B below).** Each
session keeps all its data in its own directory: FAT does not like many files in one directory, a damaged directory
then costs one session, not the whole collection, and a session is pulled, listed and removed as one. A
two-partition card still works: its videos go to its second partition.

```
Partition 1, the recordings (mounted at /sdcard):
/sdcard/
  sessions.csv                       the global index: every boot's session.csv rows (wrapped lines, no routing)
  entry.gcl                          service files that belong to no session (M21)
  000276/                            one directory per session (the boot id %06u, or the recorder.session label)
    recordings/
      000276_imu_lsm6dso32.csv       <session>_<stream>.csv, exactly the bytes recorderd wrote
      000276_health.csv  000276_sequencer.csv  000276_flight.csv  …  000276_<task>_events.csv
      recorder.log                   this session's unrouted lines (the status line carries 'session': '000276')
      session.csv                    this session's index rows
    videos/<start uptime ms>.h264    raw Annex-B, SEI per frame; on the pad a new file every 300 s (M7b)
    <kind>/                          later kinds slot in beside recordings/
A two-partition card's partition 2 (mounted at /sdcard/videos):
/sdcard/videos/
  000276/<start uptime ms>.h264      its videos, in place of 000276/videos/
```

- **The session object.** One place maps (session, kind) to a directory, so no consumer builds a path:
  - recordings go to `/sdcard/<session>/recordings`;
  - videos go to `/sdcard/<session>/videos` (a two-partition card's to `/sdcard/videos/<session>`,
    `layout::videos_partition_directory`);
  - any later kind goes to `/sdcard/<session>/<kind>`.
  The ring_saver asks it per block, with the session from the block header, because it also files earlier boots'
  blocks. The camera asks it with this boot's session. It is `crates/recorder` `layout` (`directory`, `file_path`,
  `GLOBAL_INDEX`), pure and host-tested.
- **A `recorder.session` label may not be `videos`, `recordings` or `video`**, in any case (FAT folds case): the
  names the card's root holds for something else (`layout` `RESERVED`). With `videos` the session's recordings would
  land in the videos directory, on a two-partition card the videos partition; `recordings` and `video` are an M4
  card's root directories, which the tools still read (a session `recordings` would be one directory in both
  layouts). The firmware's session prefix takes such a label as none and falls back to the boot id, and the boot log
  says the label was ignored (`layout` `is_reserved`). That checks only the running boot; the saver also files
  earlier boots' blocks by their header label, and the M4 firmware allowed these names. So `layout` itself files a
  reserved session under `_<label>` (`_videos/recordings/`): no session this firmware names begins with `_`, and
  nothing of a session lands on the videos partition. `tools/board_pull.sh` refuses the three names as a session and
  points to `_<label>`, which it pulls like any other.
- **Why videos on their own partition.** The lock that matters is per volume: one VFS lock and FatFs sync object
  per mounted FAT, held across a whole `write`/`fsync`. A 64 KB video write held it 23 ms on average, 143 ms at
  most. On their own volume, video and the ring_saver no longer serialise on it, and each has its own FAT, so a power
  cut mid-video (the impact) cannot tear a FAT sector that holds the chains of saved recordings. The SDMMC host
  mutex still serialises single SD transactions on both, and the card's controller is shared: the two streams still
  meet, but only per transaction.
- **How the board mounts them** (`components/sd_card`). The card is initialised once (host, slot 0, card, as
  `esp_vfs_fat_sdmmc_mount` does inside) and registered as one FatFs physical drive, through a disk I/O layer of
  our own: `diskio_sdmmc.c`'s calls, each under one card lock (a FreeRTOS mutex). FatFs's per-volume locks do not
  serialise two volumes, and both share the card and the persistent bounce buffer (§4.7). The board judges the
  table itself before anything mounts (`partition_table.c`, plain C, host-tested by
  `src/control/test/test_sd_partition_table.py` against `sd_prepare.sh` and `mkfs.fat` images). Two volumes need
  sector 0 to be an MBR (not a FAT boot sector, as FatFs's `check_fs` judges) whose partitions 1 and 2 both have a
  FAT type (`0x01 0x04 0x06 0x0b 0x0c 0x0e`), lie apart and within the card, and each hold a file system whose boot
  sector's size fits the partition. FatFs checks none of that: it takes a volume's size from its boot sector, so two
  volumes over shared sectors would each allocate clusters the other owns. Then `VolToPart` maps logical drive 0 to
  partition 1 and drive 1 to partition 2 (`FF_MULTI_PARTITION` is on; `FATFS_VOLUME_COUNT` is 2). IDF's VFS takes
  the longest matching prefix, so `/sdcard/videos` (14 characters, within `ESP_VFS_PATH_MAX`) shadows partition 1.
  - Anything else (a superfloppy, no table, one FAT partition, a partition 2 that is not FAT, an M4-era card) mounts
    one volume at `/sdcard` wherever FatFs finds it, and `videos/` is a plain directory on it beside the session
    directories. That is worse: one volume lock and one FAT for both. But it works.
  - Two FAT partitions that overlap, pass the card's end, hold a file system larger than the partition (an M4-era
    card whose partition 1 was shrunk in the table and given a partition 2 without being formatted again), or whose
    boot sector will not read: partition 1 is mounted alone and there is no video that boot. A partition 2 with a FAT
    type that does not mount ends the same way. In both, `video start` refuses: on partition 1 video would fill the
    recordings volume and stop the ring_saver.
  - A sector 0 that will not read in three tries mounts nothing: the card counts as missing that boot. Taking it for
    a card without a table would mount partition 1 of a two-partition card as the card's one volume, and let video
    fill it.
  - The boot log says which (`sd :: mounted card XXXX-XXXX, two partitions (videos XXXX-XXXX): recordings … MB,
    … MB free; videos … MB, … MB free`; `sd_card` logs the table's verdict and why). `health`'s `sd_free_mb` is the
    recordings volume's alone, the one the ring_saver needs: a sum would show gigabytes of videos space while the
    recordings had stopped. The videos figure is in the boot log and after each `video stop`. The card's name is
    partition 1's volume serial.
  - IDF's `vfs_fat` keeps a base path cut to 13 characters (v5.5.5 `esp_vfs_fat_register_cfg`: `strlcpy` into
    `base_path[ESP_VFS_PATH_MAX]` with `sizeof - 1`) and looks volumes up by it, while the VFS registers the whole
    path. File I/O on `/sdcard/videos` works; `esp_vfs_fat_info` and `esp_vfs_fat_unregister_path` never find it.
    So the free space is read by the FatFs drive (`sd_card_space`), and a videos volume that fails to mount is
    unregistered by both names (the VFS entry by the whole path first, then `vfs_fat`'s context by the cut one).
  - The VFS table (`CONFIG_VFS_MAX_COUNT`) goes from 8 to 12: `/dev/uart`, `/dev/secondary`, `/dev/null`,
    `/dev/console`, lwIP's sockets, `/sdcard`, and esp_video's `/dev/video0` and `/dev/video20` at `video start`
    filled the 8; `/sdcard/videos` would have left `video start` without an entry.
- **A/B on the bench (operator, 2026-10-06):** the taster keeps a one-partition card and 7F a two-partition one
  (MBR, two primary FAT32 partitions labelled `metrics` and `videos`). The cards are the same vendor and batch. Both
  layouts stay supported, so the same load script can be run on both and compared:
  - `video stats`: `write_us_max`, `sd_busy_pct`, missed frames;
  - the ring_saver: batch time, `behind_ms`, card errors;
  - any `allocate_dma_buf` failure.
  The firmware finds the partitions by number; the labels are for people.
  - **First run (2026-10-07, one run each, so not conclusive).** The load: video + `ring load` at the flight rate +
    four 20 MB pulls.

    | | taster, one partition | 7F, two partitions |
    |---|---|---|
    | missed frames | 1.9 % | 0.5 % |
    | SD write max | 167 ms | 406 ms |
    | SD busy | 48.6 % | 59.4 % |
    | saver behind at the end | 29 ms | 2.7 s |

    No failure on either. These are two different physical cards, and the card-level lock serialises every command
    on both volumes, so two partitions separate only the FAT-level locking. Swapping the cards between the boards
    would separate card from layout.
  - **Cards swapped (2026-10-07, same load).** The taster now has card AB58 (two partitions) and 7F card 267D (one).

    | | taster, AB58, two partitions | 7F, 267D, one partition |
    |---|---|---|
    | missed frames | 0.7 % (19 of 2576) | 0.6 % (14 of 2432) |
    | SD write max | 399 ms | 151 ms |
    | SD busy | 58.3 % | 47.2 % |
    | saver behind at the end | 4.5 s (34 blocks) | 39 ms (9 blocks) |
    | lowest internal DMA free | 1280 B | 1892 B |

    No failure on either; `pace` 0 missed on both. The slow writes and the saver lag moved with card AB58, so the
    board is ruled out.
  - **Card, not layout (raw `sdtest`, one writer, no video).** `sdtest` writes one file on partition 1 alone, so no
    second volume and no lock contention. Card AB58 still stalls 488–587 ms (4 of 8 runs above 480 ms, the rest
    48–53 ms). Card 267D peaks at 177–212 ms. AB58 is faster flat out on its first run (3.9 MB/s against 3.0) and
    slower on later ones (2.5–2.6 against 4.1–6.4). So the long stalls are the card's own, most likely its internal
    garbage collection. They are not a cost of the two-partition layout. A strict proof would re-prepare one card in
    the other layout (operator, card reader). Either way the flight budget must ride out a ~0.6 s card stall: the
    saver lags (the flash ring absorbs it) and the video writer drops a few frames.
  - **Re-run after several days of use (operator).** AB58 had just been put into service; 267D had been powered
    much longer. A fresh card may still be doing its first-use housekeeping, so repeat the A/B and the raw `sdtest`
    once both cards have run for days.
  - **Third run (2026-10-07, after the internal RAM work and the panic log; same load, real scene):**

    | | taster, AB58, two partitions | 7F, 267D, one partition |
    |---|---|---|
    | missed frames | 0.5 % (20 of 4091) | 1.8 % (63 of 3451) |
    | SD write max | 408 ms | 201 ms |
    | SD busy | 59.9 % | 47.4 % |
    | saver behind at the end | 91 ms | 290 ms |
    | 20 MB downloads under video and saver | 30–37 s | 25–35 s |

    No failure, no freeze and no card error on either. Raw `sdtest`: AB58 stalled 486–590 ms again (3 of 6 runs),
    267D at most 169 ms. Over three runs, both ways round, the long writes follow the card; neither layout has shown
    an error or a freeze.
  - **Missed frames do follow the card (operator, 2026-10-07):** AB58, the two-partition card, missed 0.5 / 0.7 /
    0.5 % over the three runs; 267D, the one-partition card, 1.9 / 0.6 / 1.8 %.
    - `missed` is a gap in the camera's capture sequence: the encode loop on core 0 came late and the receiver had no
      free buffer. `dropped` (the SD ring full) was 0 in every run on both cards. So the long card stalls never cost
      a frame (the 2 MB ring absorbs them), and the card's effect on `missed` has no SD-side mechanism yet.
    - Card, layout and scene (two cameras, two views) are still confounded.
    - The saver's backlog over each whole run (recorder.log's per-second positions, ~168 s under the ring load of
      ~24 KB/s) is small on both: AB58 mean 33 / 35 / 55 KB, peak 204–224 KB (16–32 s above 64 KB); 267D mean 24 /
      32 / 30 KB, peak 95–292 KB (6–12 s above 64 KB). The end-of-run figures (2.7 s, 4.5 s against 29 ms) were where
      each run happened to stop.
    - The data rate was the same on both (video 995 / 979 KB/s, the ring ~24 KB/s): AB58's higher SD busy share is
      more card time per byte (~0.60 against ~0.48 s per MB of video), and its run moved more data only because it
      ran longer (its downloads were slower).
    - Two partitions stays the layout (the operator, 2026-10-07: it does better on frames), one partition the
      fallback.
  - **Fourth run (2026-10-07): both cards discarded and prepared alike, layouts swapped.** Both were prepared with
    `sd_prepare.sh` in panda's own reader (Realtek RTS522A, which passes discards on: granularity 4 MiB): a whole-card
    discard, partitions on the 4 MiB grid, 32 KiB clusters, half the card. AB58 got `--partitions 1`, 267D the two
    partitions; each stayed in its board, so only the layout changed. AB58's old partitions had both started 1 MiB
    off the 4 MiB grid that is its preferred erase size (a hand-made table); 267D's old table was not read.

    | | taster, AB58, one partition | 7F, 267D, two partitions |
    |---|---|---|
    | missed frames | 0.10 % (3 of 2988) | 0.16 % (5 of 3060) |
    | SD write max | 136 ms | 141 ms |
    | SD busy | 38.3 % | 45.9 % |
    | saver backlog over the run, mean / peak | 27 / 137 KB | 32 / 113 KB |
    | 20 MB downloads under video and saver | 22–29 s | 23–28 s |
    | raw `sdtest` flat out | 9.2 MB/s, worst 30 ms | 4.9–7.5 MB/s, worst 128–198 ms |

    No failure on either. The preparation is what mattered: missed frames fell from 0.5–1.9 % to 0.10–0.16 % on both
    cards. AB58's 400–590 ms stalls left the video run (two paced `sdtest` runs still met 226 and 367 ms), and its
    flat-out speed rose from 2.5–3.9 to 9.2 MB/s. Between the layouts, with the cards prepared alike, what is left is
    within noise. Two partitions stays the layout for its own reasons (a lock and a FAT per volume, the recordings'
    FAT safe from a cut mid-video). **Rule: prepare every card with `sd_prepare.sh`** (a whole-card discard, the 4 MiB
    grid); a card partitioned by hand can sit off its erase grid.
    - The operator's next step: discard both cards and make their file systems again (`tools/sd_prepare.sh`
      discards first), the same layout on both or swapped, then repeat the A/B. AB58 may also have been doing its
      first-use housekeeping.
  - **Why two volumes cannot overlap on the card (operator's question: *nix has an elevator under the FS locks).**
    Two levels of locking sit under the two FatFs volume locks:
    - `components/sd_card`'s disk I/O layer takes one card mutex around every driver call from either volume;
    - the SDMMC host's own mutex serialises every transaction across the card (slot 0) and the C6 (slot 1).

    There is no elevator: requests go in priority order (the mutex inherits priority), never merged or reordered.
  - **Downloads were slower than in the first two runs** (16–23 s then). Stage 2 of §4.7 (esp_hosted's buffers in
    PSRAM, lwIP's receive copy) costs ~10–20 % idle (2.6–3.0 MB/s against 3.3) and about a third under video.
    Raising esp_hosted's queues and the TCP send buffer again won most of it back, for ~1.4 KB of internal RAM, and
    was reverted: internal RAM comes before Wi-Fi speed (§4.7). Also seen once on the taster in this run: `H_SDIO_DRV: task still writing Rx data to queue!`. esp_hosted
    drops a whole received chunk when its copy task has not finished the last one (`sdio_drv.c:1084`), and the copy
    is now PSRAM to PSRAM beside the encoder. TCP retransmits it; an RPC frame to the C6 has no retransmit.
- **Next: the A/B under a HITL flight (operator, 2026-10-11).** The bench load above is video + `ring load` +
  pulls; a flight adds the control loop, whose lateness is the latency that matters. So: the same HITL scenario on
  both layouts with video recording through the flight (a real flight records from the rod trigger), several runs
  each, then the layouts swapped between the cards (the operator re-prepares them) to part card from layout. Compare
  the control tick's lateness and overruns, the saver's backlog, the video's missed and dropped frames, SD write max
  and busy share.
  - **The decision rule (operator):** if two partitions are not clearly better on I/O and latency, one partition
    wins: it is the more space-efficient (no fixed split between recordings and videos), and every session keeps
    all its data in one directory, `<session>/recordings/`, `<session>/videos/`, `<session>/<kind>/`, so a session
    is pulled, listed and removed as one. That moves videos from `videos/<session>/` into the session's directory
    (`layout::directory`), and two-partition cards stay readable by the tools.
  - **Done (2026-10-11), 3 HITL flights per layout** (`tools/hitl_partition_ab.sh`, `tools/partition_ab.py`): F15,
    285 g, video through the flight, each card cleared first, the boards taking turns; the taster with AB58 (one
    partition), 7F with 267D (two), both `--no-power` (7F's servo rail is off; the fins commanded as ever). Mean
    [worst run]:

    | | one partition (taster) | two partitions (7F) |
    |---|---|---|
    | control lateness max | 49 us [60] | 46 us [55] |
    | overruns, dropped frames, unsaved at DONE | 0, 0, 0 | 0, 0, 0 |
    | SD write max | 225 ms [266] | 136 ms [165] |
    | SD busy | 42 % | 41 % |
    | saver backlog over the flight, mean / peak | 30 / 126 KB | 29 / 89 KB |
    | missed frames | 2.1 % [3.3] | 4.7 % [9.2] |

    What the flight needs is the same on both: the control tick, nothing dropped, the saver caught up at DONE. Two
    partitions trims the worst write and the backlog's peak, which the video ring (2 MB) and the flash ring absorb;
    its missed frames are 7F's encoder (45 ms a frame under HITL against the taster's 38), not the card. Board, card
    and camera stay confounded with the layout (the taster's breadboard is not to be touched). By the rule: **one
    partition**, videos in `<session>/videos/`. 267D is re-prepared with `sd_prepare.sh --partitions 1` when the
    operator has it out; until then 7F records to its second partition, as before.
- **Over HTTP.** `files.rs` lists `videos` in `/sd/` itself, because it is a mount point, not a directory on
  partition 1. A session is `GET /sd/<session>/recordings/`, its videos `GET /sd/videos/<session>/`.
- **Pulling a flight.** `tools/board_pull.sh` reassembles a flight on the host as `<session>/recordings/` and
  `<session>/videos/`. It still reads a card the M4 firmware wrote (`recordings/<session>/`, `video/<session>/`)
  when the session is not in the layout's place. A session in both places is refused: the boot running when an M4
  card's board is reflashed keeps its bulk in the old place and gets only its unsaved tail in the new one, and either
  alone would pass for the whole session. Pull such a session by hand (or pull the card's sessions before the
  update). `recorder_flight.py`, `assemble_capture.py` and `ring_split.py` take a session's `recordings/` directory;
  it holds the session's `recorder.log` and `session.csv` beside the streams. They keep reading Luckfox dumps
  (`/userdata/recordings`) as before.

Per-session directories avoid FatFs's linear `f_open` scan, and the FAT32 limit of 65 536 entries per
directory that a flat layout reaches after about 1 400 boots.

File names are long (FatFs LFN, already on in `sdkconfig.defaults`), because the names carry the telemetry
routing. 32 GB cards are the safe size. A 64 GB card can be one FAT32 volume (`mkfs.vfat -F 32`); no split is
needed.

**Card preparation** (the operator's option): `tools/sd_prepare.sh` discards the whole card. It then writes an MBR
with two primary FAT32 partitions over the share of the card in use (`--use`, 25–100 %, default 75), split in halves
(the operator, 2026-10-10): partition 1, label `METRICS`, the recordings, and partition 2, label `VIDEOS`, the
videos. Measured -> reported (M9) writes about 50 KB/s of text, ~180 MB an hour, so the old fixed 4 GiB of
recordings no longer fits the logs' share. Each gets a fresh volume serial. It leaves the rest untouched, as spare
area for the card's controller. A card too small for two halves of 512 MiB at the share asked is refused. The firmware mounts both and never formats a card. A card M4's
`sd_prepare.sh` wrote (one partition over the whole share) runs on the one-partition fallback; prepare it again,
after its sessions are pulled, for the two-partition layout.

**Flash partitions** (`partitions.csv`, decided 2026-10-06):

| partition | type | offset | size | holds |
|---|---|---|---|---|
| `nvs` | nvs | 0x9000 | 0x6000 | unchanged, shared with MicroPython: boot count, calibrations, crumb |
| `phy_init` | phy | 0xF000 | 0x1000 | |
| `otadata` | ota | 0x10000 | 0x2000 | which app slot boots |
| `ota_0` | app | 0x20000 | 0x400000 | 4 MB app slot (the image was 3.0 MB on 2026-10-12) |
| `ota_1` | app | 0x420000 | 0x400000 | 4 MB app slot, for OTA (M15) |
| `cfg` | nvs | 0x820000 | 0x100000 | 1 MB: blobs `board` and `launch`, the `creds` namespace, the session-start record, the ring positions |
| `panic` | data | 0x920000 | 0x20000 | 128 KB: the panic log, 16 records of 8 KB (§4.8; added 2026-10-07) |
| `log` | data | 0xC00000 | 0x1400000 | the 20 MiB ring (5120 blocks) |

- **2026-10-12 (operator): the app slots grow from 3 to 4 MB, the ring shrinks from 24 to 20 MiB.** The image had
  reached 3.0 MB of its 3 MB slot. The firmware's ring (`flash_ring::Geometry::BOARD`) is 5120 blocks, and a `log`
  partition of any other size parks the recorder with a message to flash the matching table (another layout's
  blocks would read as the ring's). Migrated on the taster and 7F: each board's saved and launch configs backed up
  over CC, the new `cfg`, `panic` and `log` ranges erased (the board held in download mode, so no boot met stale
  data), the table and `ota_0` by `flash.sh`, `cfg` by `provision.sh` (both configs and the Wi-Fi passwords); each
  came back with its own config id, a 5120-block ring, and OTA into the 4 MB `ota_1`. Lost on purpose: the panic
  log, and the ring's contents (already on the cards).
- `flash.sh` writes the app to `ota_0` and erases `otadata`, so the bootloader boots `ota_0`; OTA over the console or
  Wi-Fi (M15).
- `tools/provision.sh` reads the `cfg` row from `partitions.csv`, so it follows a move.
- 0x940000–0xC00000 stays free (2.75 MB).

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
- **`GET /sd/…`** exists (`files.rs`): `/sd/<session>/recordings/…`, `/sd/videos/<session>/…`, `/sd/sessions.csv`.
  The listing format is `<name>;<bytes>`; the root's listing names `videos` when it is the videos partition's mount
  point.
- **`GET /ring[?boot=N]`** streams every valid block decoded to wire lines (`[@<routing>@]{OPEN};…;<CLOSE>`,
  the saver's text with its routing) in `block_sequence` order: a byte-exact wire capture.
- **`GET /ring/image`** streams the raw 20 MiB partition, as `esptool read_flash 0xC00000 0x1400000` reads it.
- `tools/ring_split.py` turns either into session directories. This is the recovery path when the card failed
  in flight. `--blocks` lists an image's blocks instead, one line each and nothing written: the sequence, boot,
  session and flight flag, the span of the records' uptimes, the rows per file, and the torn and damaged chunks. It
  shows that every record from BOOSTING to DONE lies in a flight block ("Ring geometry and block life cycle").

### 3.13 Budget (v0.1 sensor set at the true Rust rates)

| item | value |
|---|---|
| rows | ~375 rows/s. On the SD as text ≈ 26 KB/s (~68 B a line, unchanged). In flash as binary records ≈ 14.6 KB/s: about 2.4× denser than the 35 KB/s of routed text |
| chunk and block overhead | ≈ 1.2 KB/s at a 10 ms tick (an 8 B chunk header and its padding per chunk, a 56 B header per block): ≈ 16 KB/s in flash in all |
| ring holds | 20 MiB / 16 KB/s ≈ 22 min with no SD progress (≈ 7 min if the IMUs go to 1 kHz) |
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
| `Controller.stage` int, 5 writers; `armed`, `manual` | **the flight word** (M7): one `AtomicU32` in internal .bss, stage (bits 0-2) \| manual (3) \| armed (4) \| launched (5), with plan B's meanings (`controller.py:89-91`). One Acquire load is a consistent snapshot, so no reader (health, inspect, the link, the video thread) sees a torn state. **One writer, the control thread** (and `warm.rs` before any spawn, M8): the stage machine's advances, the separation's, and the operator's hold, resume, arm and disarm, which reach it through a polled request slot (§4.3, step 4). No source byte and no epoch: the machine already tells its own moves from external ones, and M8 adds an epoch if its checkpoint needs one. Apply rules, plan B's Controller (`controller.py:572-620`): a hold at `s` sets the stage `s` and manual = (s ≠ SETTING); resume clears manual; disarm always clears armed; arm applies only in SETTING unheld, else it is refused with the stage problem's text (plan B arms whatever the stage became during its sweep, §6) |
| — (plan B pauses its Wi-Fi hunt on any airborne stage) | `launched`, the flight word's bit 5: set only by the stage machine's own SETTING→BOOSTING advance (the launch by altitude or by acceleration); cleared by every commit to SETTING and by every hold, since an operator's move is never the machine's launch; never set by a hold, by `stage auto` into an airborne stage, or by a separation. M8's warm restore into BOOSTING..LANDING sets it. While it is set the link never rejoins, hunts or dials (§4.4) |
| `warm_started` | AtomicBool, written by `warm.rs` before any spawn |
| `machine.Timer` + `ThreadSafeFlag` (flight 100 Hz) | gptimer alarm ISR on core 1 → `xTaskNotifyFromISR` (integer-only ISR) |
| `commons.Waiter`, IRQ + `wait_for_ms` | GPIO ISR → notify with timeout; `count > 1` → overrun counter |
| `Databoard` (latest value, rank, window; `value()` extrapolates) | `crates/databoard` seqlock channels; **no extrapolating accessor**; `read() -> Option<Reading{value, source, age_us}>` |
| reconcile learning inside `read()` | on the writer side: the primary's owner updates each fresh backup's offset EMA (α 0.1) in that backup's atomic |
| `Inspector` registry | a frozen sorted `&'static [&'static dyn Inspectable]`; `update`/`probe`/`calibrate` forwarded to the owner's mailbox |
| `Task.claim()` (device conversation guard) | gone: the owner thread services requests *between* its own transactions |
| `Task.note` / `strike` | `Strikes{count}` + `AtomicBool healthy` in the status block |
| `Task.event` → `<name>_events.csv` + log | the same stream (`decimate_us = -1`), a durable record |
| `Controller.find` / `query(waiting=True)` | static wiring: handles passed at spawn; a missing dependency fails bring-up |
| `machine.WDT` + watchdog task (stall check) | the supervisor reconfigures the TWDT to panic (`esp_task_wdt_reconfigure`, `trigger_panic = true`, timeout `wdt_timeout_ms`, as `machine.WDT` does; by default a timeout only warns) and feeds it; control stall > `stall_ms` while controlling (armed, launched, airborne, the tick seen; judged by the tick's ms stamp, floored at two periods) → event + `esp_restart`, never again in a stall restart's own warm start (M8) |
| `Mission` singleton | owned by the `cc` executor (M8); the checkpoint reads a snapshot it republishes after every line under a core-0 mutex, never on the flight core |
| `cc_client` Dispatcher in the asyncio loop | the `cc` executor thread, fed by link + console |
| GC off / rescue / OOM forecast / `fixed.py` | not ported (§6) |

### 4.2 Threads, cores, priorities

| thread | core | prio | owns / does | paced by |
|---|---|---|---|---|
| gptimer ISR | 1 | ISR | notify control + bus owners (`eIncrement`) | alarm at `schedule_hz` |
| `control` | 1 | 20 | SPI2 IMU devices, the flight word (its one writer) and the request slot, the stage machine, the separation and its thrust gate, the attitude filter, flight law, LEDC fins | ISR notify |
| `i2c0` | 1 | 18 | I2C port 0 sensor devices (SEN0697, INA226 on v1.x; ICP, SDP810, laser on v0.1, …); the camera SCCB shares the bus handle | ISR notify + rate-monotonic table by tick count |
| `i2c1` | 1 | 17 | I2C port 1 (ICP, SDP810, laser on v1.x; INA226 on v0.1) | same |
| `gnss` | 1 | 14 | UART2 receiver, NMEA/UBX | blocking read with timeout |
| esp_hosted, Wi-Fi, lwIP | 0* | 18-23 | IDF; never switched off: after the detected launch the link never rejoins (M7); the beacon AP at DONE (M11b) | — |
| `video encoder` | 0 | 10 | exists | camera |
| `supervisor` | 0 | 9 | TWDT feed, stall check, health 1 Hz, checkpoint (NVS); later gnss_calib, field | 100 ms base tick |
| `ring_writer` | 0 | 7 | programs the `log` partition; ring consumers, tees | ~10 ms tick, or a durable record's notify |
| `video SD writer` | 0 | 6 | exists | — |
| `video` (M7; only with a camera and a card configured) | 0 | low | the only caller of `coludo_video_start`/`stop`: the flight recording by stage, GCL `video start`/`stop` by mailbox; PSRAM stack | the flight word every 250 ms |
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
2. **IMU:** one 16 B burst from the LSM6DSO32's STATUS_REG (+6 B from the ADXL375 on v0.1). Push `accel` on XLDA
   and `rate` on GDA, only a new conversion; write the IMU telemetry row on either, if `due`.
3. **Separation:** sample GPIO33, read the clock, offer the read to the stamp pair (with no episode open, a read off
   the latch opens one; polled, a changed read is an edge), and take the pair once its latest edge is debounce_ms
   (default 10) old: the level read decides `separated`, `nested` or `blip`. A separation in BOOSTING sets the stage
   to GLIDING; every event writes `separation.csv` as `event;stage;moment;duration`: the stage after it, the
   episode's first edge in uptime us, and its span to the last edge in us.
   **The thrust gate (M7):** in BOOSTING a separation counts only once the gate is open. It reads a_x, the body
   frame's +x (the nose), from a fresh `accel`: open once thrust was seen (a_x above `launch_g` for `launch_ms` of net
   time) and then ended (a_x below 0 g for 20 ms of net time), both afresh at each BOOSTING entry. A separation
   concluded before that is pending, and counts when the gate opens or once the pin has stayed 0 for 100 ms after the
   episode's last edge (the backstop), either way on a tick that reads 0 with no episode open, dated by the episode's
   first edge; a re-nest clears it, as one `blip` row
   (the M7 scope says how a pending separation is recorded). A counted separation commits GLIDING to the flight word
   and logs the console line `controller :: stage -> gliding` (§4.4). In code: `drivers::separation::ThrustGate`,
   stepped with the uptime's ms and the fresh a_x, and `Separation::tick`, which takes the pair and decides.
4. **Requests** (M7):
   - **The request slot**, served first: one `AtomicU32` per requesting thread (in M7 only the executor), seq (bits
     16-31) \| phase (14-15: idle, posted, taken, done) \| kind (8-13: Hold, Resume, Arm, Disarm) \| argument or
     result (0-7). The tick claims a posted request (posted → taken), commits it to the flight word by the apply
     rules (§4.1), and stores done with the result. Polled, never notified: the control task's one notification
     entry counts ticks. Every slot access is SeqCst (a handful per request, so the cost is nil and no ordering has
     to be argued). Core 1 takes no lock, allocates nothing and makes no kernel call for a request: the executor
     logs it. M13's fin requests get slots of their own.
   - **The mailbox** (std mpsc) stays for the SETTING-only `probe` and `bustune`, at most 2 a tick. Each is checked
     against the stage again here and refused outside SETTING, so a launch detected during a probe sweep never
     runs the IMU's probe in BOOSTING. The refusal goes back as the stage, a value; the executor writes
     `refused -- stage is X (changes only in setting)`.
5. **Every 2nd tick:** the attitude filter (50 Hz; the 5-cycle blind window kept) publishes `attitude` at p1.
6. **Every `period_ms` (default 50) on the uptime clock:** the stage machine, built from the first enabled entry
   whose implementation is `sequencer` (none: no machine, as in plan B). `now_ms` is the uptime in ms as a wrapping
   u32 (`commons::elapsed_ms` takes the wrap). When the tick is slower than the period, it steps every tick, with a
   log line. It reads `accel`, `elevation` and `agl` at a fresh esp_timer value after the IMU and the separation (the
   databoard refuses a stamp from the future); the source name `gnss` maps to `Source::Gnss`. A transition commits
   to the flight word, then logs `controller :: stage -> X` (console only), then records `<entry> :: stage -> X
   (reason)` and the `<entry>.csv` row `stage;reason` (durable, never decimated), as plan B's `_advance` does
   (`tasks/sequencer.py:128-135`). An external move writes its row only, with no log line, before the advance's
   row; a full ring is logged once, never fatal. The recorder's flight flag follows `airborne()` of each committed
   stage, holds included: raised before the transition's rows, lowered after the DONE row (the writer applies a fall
   once everything queued before it is programmed). When `control::start` or its timer fails, the entry is crashed:
   "the control thread did not start".
7. **If `flight` is enabled:** governor → guidance → PID → mixer → LEDC duty, compare-and-set. `flight.csv`
   every 100 ms, sampled before the gates. Disarmed or no setpoint gives neutral once (gated on `_active`).
   Stale attitude and the on-rod guidance gate apply neutral on every tick and leave `_active` set;
   compare-and-set makes the effect similar.
8. **Heartbeat:** `LAST_TICK_MS` (u32 ms, one Relaxed store) and `TICKS`; the supervisor derives `controlling` from
   the flight word (M8).

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
- **The arrival stamp and the 25 s rule (M7).** Each line is stamped when it arrives, before the queue send
  (`firmware/src/commands/mod.rs:295-305`). Before it posts an arm, the executor refuses one older than 25 s, `err
  unsafe {"arm": "the request waited <n> s: send it again"}`; the slot adds at most 2 s, so the answer beats the
  hub's 30 s (`src/control/board.py:22`) whatever queued ahead, and an arm never lands after the hub gave up.
- **The request slot's deadline (M7).** The executor polls its slot every 1 ms (FreeRTOS at 1000 Hz). At 2 s it
  withdraws a request still posted (posted → idle) and answers that it was not applied; one already taken is waited
  for, a matter of microseconds. A flag the control thread sets before its first tick lets the executor answer at once
  when the thread never started: armed is then false by construction, so `disarm` answers ok.
- **No fast lane for `disarm`:** the executor serialises commands, so on the pad a console `disarm` can wait behind a
  hub probe sweep, ~18 s at worst (the IMU's probe ≤ 2 s, then each I2C owner ≤ 8 s).
- **SETTING-only requests are checked again where they are served** (M7): an owner's serve (`firmware/src/owner.rs`)
  and the control thread's mailbox refuse a request outside the stages its command may run in (the barometers'
  re-zero also at DONE, §6), answering the stage as a value: off the pad they make no text, and the executor answers
  `refused -- stage is X (changes only in setting)`. The executor checks the gate once, and a sweep then takes
  seconds, so a launch detected mid-sweep never runs a probe in BOOSTING.
- **The controller's log lines (M7)** are the executor's, from the slot's result, in plan B's order
  (`controller.py:561-620`): `controller :: stage -> X`, then `controller :: stage X (held|auto)`;
  `controller :: stage auto`; `controller :: armed`, `controller :: disarmed`. A hold logs even when the stage is
  unchanged. `controller :: stage -> X` goes to the `console` log target, which the recorder.log tee skips: plan B
  printed its Controller's lines to the console only (`main.py:75, 101`), and the tools find stage events in the
  recording by the text `stage ->` (`tools/flight_kpi.py:95`, `tools/flight_report.py:81`,
  `tools/flight_video.py:332`), which must stay the stage machine's `<entry> :: stage -> X (reason)`. The hold,
  arm and disarm lines stay in recorder.log, as M6 records them (§6).
- **The link (M7):** it checks `launched` (§4.1) before every join, hunt, association and dial; while it is set the
  link never initiates a connection, and a session still up runs until it drops. The dial is bounded (a 3 s connect
  timeout in `firmware/src/link.rs` `dial()`: lwIP's 12 SYN retries would otherwise hold a dial to a dead hub for over
  a minute), so a dead hub never delays the check. The radio itself is never switched off.

### 4.5 Bring-up order (`main.rs`)

1. NVS and the boot count (exists). Latch the reset reason. If the clock reads before 2000, set it to
   2000-01-01 + uptime.
2. Config load from `cfg` → `BootSource`.
3. **Storage first:** recorder registry and rings (every line from here is queued in PSRAM), the `log` bridge, the
   panic log, the SD mount and the camera's encoder reservation.
4. Buses created on core 1 by their owner threads, which are spawned parked. `layout.resolve` by scan
   (100 kHz probes). Cache the post-layout `config_id`. Then the ring_writer thread (the bring-up scan runs in it,
   writes the session-start record with the post-layout id, then starts the ring_saver and the ring_eraser, §3.6),
   and the provenance line `boot: board <id> | firmware <v> | config <id> <source> | reset_cause <n> | boot <id> |
   session <s> | saved_config <pre-layout id>`.
5. Mission (launch.config, `max_range_m` from the `field` component).
6. Device setup in config order (sensors, then components), with retries and diagnose. Register channels from
   `provides`. Then the alternatives pass.
7. Freeze the databoard topology and the inspector (`Box::leak` → `&'static`).
8. `warm.rs` restore, before any flight-relevant thread runs.
9. Unpark the bus owners. Spawn control (core 1) and the supervisor. Arm the gptimer. The supervisor itself
   reconfigures the TWDT to panic at `wdt_timeout_ms` and subscribes to it (M8).
10. Console, link and httpd (as today).

**The fast warm boot (operator, 2026-10-11: a reset resumes control in about a second; flight buses and sensors
first, the camera and the Wi-Fi after them, nothing blocking).** Measured on the taster, reset to the first control
tick: 3.6 s before, **1.0 s** after (power-on reset; the Wi-Fi joins later on its own, `link::run`).
- The bootloader never re-hashes the app (`BOOTLOADER_SKIP_VALIDATE_ALWAYS`, ~0.28 s: with no rollback and ota_1
  invalidated there is no other image to fall back to; flash.sh and the OTA verify on write) and logs at WARN.
- IDF's startup lines at WARN, raised to INFO first thing in main; the console's UART driver installed right after
  (without it a console write polls the FIFO), and every log line through the printer's queue (crate::logging): no
  line waits on the 115200-baud console (~8 ms each, ~150 lines a boot).
- Step 6's I2C parts come up one thread per lent port, in config order on each bus (the buses are independent);
  a part whose identity reads as another silicon (the shared laser socket) is not retried (`Attempt::Foreign`, a
  divergence: plan B retried it, ~0.45 s a boot on a board declaring both lasers).
- The fins' boot centring is the control thread's, stepped each tick after it starts (~0.9 s at
  `fins.concurrency` 1 had held its start); a warm start skips it, the restored law taking the fins at once.

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
    - The warmstart crate matches it on golden vectors, and M8 proved it on the bench (TMS-7F and the taster). The
      HITL comparison against the plan B baselines moves to M12/M14, as does step 2.
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

### 4.7 Internal RAM budget

Internal **DMA** memory is the P4's scarce resource. The cost of running out:
- esp_hosted asserts and reboots the board (`transport_drv.c:267 copy_buff` on TX, `sdio_drv.c:862 pkt_rxbuff` on
  RX);
- the SD driver fails writes (`allocate_dma_buf`).

Measured on the taster, 2026-10-06 (`heap`: free, largest block and low-water mark per class):

| consumer | internal DMA memory |
|---|---|
| H.264 encoder reference buffer, 1080p (`3 × 16 × 24 × 120`) | ~138 KB, one block, reserved at boot (`video::reserve`) |
| the camera's stream (CSI, ISP), while recording | ~13 KB |
| esp_hosted's frame pool after a download (it never shrinks) | ~25 KB at SDIO queues 10/10 (~37 KB at 20/20) |
| the ring_saver's working buffers under load | ~5 KB with `ALWAYSINTERNAL=4096` (~16 KB at the IDF default 16 KB) |
| SD bounce buffer | 8 KB, persistent (`components/sd_card`) |
| free at idle, Wi-Fi joined | ~57 KB |
| lowest seen: video + saver + four 32 MB downloads at once | ~11 KB |

What holds it together:
- **The encoder reserves its block at boot**, before anything fragments the heap. A board without a camera (no
  `ov5647` entry in its config) reserves nothing, so the ~138 KB block stays in the heap.
- **A persistent SD bounce buffer.** IDF's default allocates an 8 KB bounce per unaligned transfer, which every
  FatFs sector-buffer write is.
- **The 64 KB `SPIRAM_MALLOC_RESERVE_INTERNAL` pool is NOT a DMA reserve** (corrected 2026-10-07). IDF carves it out
  of existing internal RAM and registers it at the head of the heap list with the INTERNAL|DMA caps at priority 1
  (`esp_psram.c:644-654`, `heap_caps_init.c:339`). So every internal request takes from it: task stacks, malloc of
  4 KB or less, after the main RAM heap and before RETENT_RAM (`heap_caps_base.c:144`). On the P4 every L2 memory heap
  is DMA-capable anyway (`heap/port/esp32p4/memory_layout.c:48`), so the pool adds and protects no DMA memory. Kept:
  changing its size is DMA-neutral.
- **Plain malloc above 4 KB goes to PSRAM.** 4 KB itself stays internal: `esp_flash_read` borrows an internal temp
  buffer (up to 16 KB) for any destination outside internal RAM.
- **esp_hosted's SDIO queues at 10/10 and the TCP send buffer at 32 KB.** These were set when esp_hosted's pool was
  internal. With its buffers in PSRAM (below), 20/20 and 64 KB were measured again (2026-10-07): idle pulls rose
  11–21 % (to 3.2–3.3 MB/s) and ~8 % under video, for ~1.4 KB of internal RAM (the queues' own storage). Reverted: the
  operator's priorities are CPU, FPU, SD and computation, then internal RAM for features. Networking must work, but
  the board is no NAS: ~1 MB/s is enough, and an OTA can wait. At 10/10: 2.6–2.9 MB/s idle, ~0.7–1 MB/s under FHD
  video and the saver. **The networking bar (operator, 2026-10-07):**
  - the board is controllable from CC, over a stable link;
  - an OTA image uploads within ~5 minutes (M15; a 1.6–3 MB image takes seconds at these rates);
  - the camera preview shows in CC's window at 1 fps at least (M7b's preview).

  Anything past that bar is not worth internal RAM, CPU or SD time.
- **For the two-partition layout:** a second volume on the same card shares the bounce buffer, so every card call
  of `components/sd_card`'s own disk I/O layer holds one card lock (a static FreeRTOS mutex, ~80 B of internal RAM).
  The second volume adds a FatFs volume mutex and a VFS entry (~0.3 KB internal); its `FATFS` object and file
  handles are in PSRAM (`CONFIG_FATFS_ALLOC_PREFER_EXTRAM`).
- **Where the 768 KB go** (taster, after init, Wi-Fi joined, before the optimization below):
  - 128 KB is the L2 cache (`CACHE_L2_CACHE_128KB`);
  - ~124 KB is static: 92 KB `.iram0.text`, 18 KB `.dram0.data`, 13.5 KB `.dram0.bss`;
  - 513 KB is heap, plus 38 KB from LP RAM (RTC, 31.9 KB) and SPM (7.7 KB), so 551 KB of heap in all. The RTC and
    SPM heaps are internal but not DMA-capable, and they sit idle (`heap`'s `rtc_free`/`spm_free`). Since §4.8 the
    panic log's capture record and log tail take ~12 KB of the RTC RAM (`RTC_NOINIT_ATTR`, out of that heap);
  - ~95 KB of that heap was free.

**The optimization (2026-10-07, before M5).** An investigation of IDF v5.5.5 under XIP from PSRAM (five readers, then
an adversarial check of the riskiest claims) established three things:
- SPI1 flash operations take a mutex and never turn the cache off (`spi_flash_os_func_app.c:29`). The one runtime
  cache-off window left is `esp_mmu_map`/`esp_mmu_unmap`, reached today only by the first partition-table load (on
  main, at boot) and, from M15, by OTA's image checks (`esp_ota_end`, `esp_ota_set_boot_partition`,
  `esp_ota_get_boot_partition`).
- So a task may have a PSRAM stack, and code may leave IRAM, as long as neither runs inside that window. The test is
  "never calls `esp_mmu_map`/`esp_mmu_unmap`", not "never touches flash".
- No interrupt allocated in IRAM (systick, esp_timer, crosscore, brownout, the CSI/DW-GDMA camera interrupt) reaches
  the code moved below.

Measured on 7F (card 267D), the same stress each time: colour bars at FHD@25 (no scene), the ring load at the flight
rate, four parallel 24 MB pulls. From stage 2 on it also ran 40 refused 1 MB uploads (receive traffic), each pull
hashed against a reference read, the host's TCP checksum-error counters, and 20 `set-config` saves over Wi-Fi:

| stage | idle internal / DMA | lowest in the stress | result |
|---|---|---|---|
| before | 93.5 / 53.9 KB | 0 | assert `sdio_drv.c:862` (`pkt_rxbuff`), 2 of 2 |
| 1a: IRAM knobs (`.iram0.text` 94.2 → 85.5 KB) | 102.9 / 63.2 KB | DMA 1.5 KB | survived |
| 1b: four core-0 stacks in PSRAM (38.9 KB) | 141.9 / 102.3 KB | DMA 34.5 KB | `set-config` ×20 took internal to 20 B |
| the allocator | 143.8 / 104.2 KB | 76.4 / 36.8 KB | `set-config` leaves internal RAM alone |
| (uploads added) | 136.4 / 96.8 KB | DMA 1.5 KB | assert `sdio_drv.c:830` (RX double buffer) |
| 2a: esp_hosted's buffers in PSRAM | 149.2 / 109.6 KB | internal 0.3 KB, transient | survived |
| 2b: `LWIP_L2_TO_L3_COPY` | 148.4 / 108.8 KB | 95.4 / 55.8 KB (flat) | every hash matches, 0 checksum errors |

With stage 2b the low-water mark during the stress stays where boot left it. The taster (card AB58, its own camera)
gave the same with the final build: low-water at boot's values, every hash matching, 0 internal fallbacks. Pace
(gptimer, core 1) under the full stress: p99 17-23 µs, max 35-60 µs, 0 missed. The core-0 esp_timer path is noisier under Wi-Fi bursts (max 0.02 to
6.1 ms): esp_hosted's tasks run at priority 23, above esp_timer's. That is not the flight path, and in flight the link
carries no traffic once the pad session drops (it never rejoins after the detected launch, M7).

What each stage is:
- **1a** (`sdkconfig.defaults`): `RINGBUF_PLACE_FUNCTIONS_INTO_FLASH` and its ISR variant, `LIBC_LOCKS_PLACE_IN_IRAM=n`,
  `WIFI_RMT_SLP_IRAM_OPT=n`, `PM_SLEEP_FUNC_IN_IRAM=n`, `PM_SLP_IRAM_OPT=n`: −8.7 KB of `.iram0.text`. The RETENT_RAM
  heap starts right after the statics, so every byte becomes heap.
- **1b:** the four slow core-0 threads get PSRAM stacks (`ThreadSpawnConfiguration::stack_alloc_caps`, httpd's
  `task_caps`). The console was unpinned (pthread's default core); it is now on core 0.
- **The allocator** (`firmware/src/allocator.rs`, the `#[global_allocator]`): std's allocator, except that a Rust
  allocation made on a PSRAM stack (the stack pointer is the marker) goes to PSRAM. The cause: one `set-config` of a
  7 KB config is thousands of allocations under 4 KB (the parsed document, its validation, canonical text and hash),
  which IDF keeps internal. What such a thread needs internal it asks for by name: `allocator::InternalBuffer`, used
  for the file server's flash-read destinations (`esp_flash_read` into PSRAM bounces through internal memory per
  call).
- **2a** (`components/hosted_no_reboot`): esp_hosted's aligned malloc, which every buffer handed to the SDIO DMA comes
  from, now takes PSRAM. The caps are upstream 2.12.8's: SPIRAM|DMA|8BIT, cache-line aligned. A full PSRAM falls
  back to internal memory, counted (`heap`'s `hosted_internal`); a missing hook raises the degraded token
  `c6-buffers-internal`. `CACHE_L2_CACHE_LINE_64B` is pinned, because the SDMMC host refuses lines it cannot align
  to.
- **2b:** `LWIP_L2_TO_L3_COPY`. lwIP copies each received frame into its own pbuf (PSRAM first) and frees esp_hosted's
  internal per-frame copy (`sdio_drv.c:1139`, asserted on failure) at once.

**The open panic of 2026-10-07 (taster, after a 7 KB `set-config` over Wi-Fi) is most likely explained.** The stress
reproduced three esp_hosted assert sites on internal DMA exhaustion: `pkt_rxbuff`, the RX double buffer, and
`copy_payload` by margin. `set-config` itself drained internal RAM. Since stages 2a and 2b, none recurred. It stays
untraced: no console was attached.

**Rules from here:**
- every new consumer (M5's buses and DMA transfers, task stacks, drivers) is priced with `heap`, `heap stacks` and the
  stress (`hosted_internal` must stay 0);
- a task with a PSRAM stack never calls `esp_mmu_map`/`esp_mmu_unmap`: M15's OTA (and its `esp_ota_get_boot_partition`
  arm check) runs on a thread with an internal stack, or reads otadata with `esp_partition_read`;
- the flight threads (core 1) keep internal stacks and allocate internal. What the CC executor builds for them (the
  activated config, the mission) lives in PSRAM: copy what the flight loop reads into its own memory when M6/M7 hand
  it over;
- M5: SPI2 is polled (`buses/spi.rs`: no interrupt, no `post_cb`, `intr_flags` 0), so no SPI code needs IRAM.
  `SPI_MASTER_ISR_IN_IRAM` is off, and so are the defaults that would select the GDMA IRAM options for parts
  never used: the SPI slave, PARLIO and esp_cam_sensor's SPI camera. With them on, linking `spi_master` cost
  10.9 KB of IRAM. With them off it costs 1.2 KB (the bus lock), measured at M5 stage 4: `.iram0.text` 85.5 →
  86.7 KB. Install the GNSS UART driver from a core-0 thread.

**Not done, and why:**
- `SPI_FLASH_AUTO_SUSPEND`, and every knob only it unlocks (esp_timer, log, libc misc, FreeRTOS ISR functions in
  flash): this GD part (0xC84019) is not on IDF's suspend list, so `esp_flash_init` would abort every boot.
- `FREERTOS_PLACE_FUNCTIONS_INTO_FLASH` (~11.6 KB): on the P4 it moves part of every interrupt-exit context switch
  (`pxPortGetCoprocArea`, through `hwlp_restore_if_used`) into PSRAM, on both cores. It also gives up
  `SPI_MASTER_IN_IRAM`. Revisit only against M5's measured control step and I2C transaction p99.
- `HEAP_PLACE_FUNCTION_INTO_FLASH` (8.6 KB): malloc and free run on every hot path, Wi-Fi's and lwIP's included.
  It was first kept for the SPI master ISR, which SPI2's polling no longer needs; it has not been evaluated since.
- Fewer capture buffers: they are in PSRAM already, so nothing internal is saved.

**Remaining levers**, in order of yield, each measured the same way:
- esp_hosted's own task stacks in PSRAM (~28 KB; its data path, priority 23);
- the ring threads' stacks (saver and eraser 14 KB, writer 8 KB last), after moving their stack-local flash-read
  targets to internal buffers;
- the main task (16.9 KB), if `link::run` moves to a PSRAM-stack thread;
- the SD bounce buffer (8 KB) and the video tasks' stacks (8 KB while recording) in PSRAM;
- the idle RTC/SPM heaps (39 KB, not DMA-capable) for cold internal objects.

**For M15, a latent bug:** the camera's DW-GDMA interrupt is allocated in IRAM, but its frame-done callback calls
`dw_gdma_channel_config_transfer` from flash on every frame (`esp_cam_ctlr_csi.c:366`). It is harmless while no
cache-off window can meet a running camera: today's only window is at boot. Before OTA can run with the camera on,
pin that function in IRAM with a linker fragment, or stop the camera around OTA.

- `heap tasks` exists only in an investigation build with `CONFIG_HEAP_TASK_TRACKING`. Its heap walk holds the
  allocator's lock long enough for core 1's interrupt watchdog to fire, so treat it as a one-off and never ship it.

**The next untraced backtrace** is kept by the panic log (§4.8, operator 2026-10-07), not by IDF's flash core dump:
nothing is written to flash from the panic handler, so moving the SPI1 flash driver out of IRAM (~8.3 KB) stays
possible.

### 4.8 Panic log

The operator's design (2026-10-07), after Linux's mtdoops (which Richard Purdie wrote at the operator's request):
the board's last panics as tagged text records, round-robin over several flash pages, readable as text. A panic may
be fatal or not.

- **Storage:** the `panic` partition (128 KB at 0x720000): 16 slots of 8 KB (two erase sectors), each one record, a
  header line in its first 512 bytes (text tags cut to 64 characters, so it always fits) and up to 7680 bytes of text:
  ```text
  COLUDO-PANIC 1 seq=6 fatal=1 cause=panic boot=364 session=000364 uptime_ms=11221 firmware=avionics-0.1.0 config=3aa09342698a recorded=365 length=4246 intact=1
  Rust panic in task cc: panicked at firmware/src/main.rs:219:18:
  deliberate panic from a console command
  abort() was called at PC 0x480ccb33 on core 0
  Core  0 register dump: ...
  Stack memory: ...
  ```
  `seq` counts records over the partition's life, so the next record goes into the slot after the newest one,
  erasing the oldest. The text is written first and the header last: a power cut leaves no header, so a torn record
  is an empty slot. `crates/panics` holds the format, the scan and the append, tested on the host over `MemoryFlash`
  with power cuts at every step.
- **Fatal** (`fatal=1`: the board reset). `components/panic_capture` captures, the way Linux's ramoops does:
  - Two link-time wraps, so IDF is not patched. `esp_panic_handler` marks that a panic began.
    `uart_hal_write_txfifo` then tees every character the panic handler prints to the console UART into a record in
    RTC (LP) RAM (`RTC_NOINIT_ATTR`, ~10 KB), which survives the reset. Both cores may enter the handler at once;
    what does not fit is counted and makes the record not `intact`.
  - The log before it, as mtdoops keeps the kernel log's tail: every log line (IDF's through components/log_hook, the
    firmware's through logging.rs) also goes into a 2 KB ring in RTC RAM, and when a panic begins the ring is copied
    into the record. The record's text is the panic's, then `--- the log before it, oldest first ---` and those
    lines.
  - Not captured: what the ROM's printf prints outside the handler (`ESP_ERROR_CHECK`'s code and place before its
    abort, a heap-corruption report, the brownout line). Its abort is captured, and the log tail shows what led to
    it.
  - A Rust panic hook writes the message, its place and the task name first: Rust panics abort here
    (`panic_abort`), so each one becomes an IDF abort.
  - The record carries a running CRC (`intact`) and two magic words, so a power-on's random RAM is never taken for
    one.
  - The next boot, on main right after the recorder opens, writes it into the panic log with the OS up, then clears
    it; a write that fails keeps it for the boot after. Nothing is written to flash from the panic handler: with XIP
    from PSRAM, IDF's no-OS flash path would suspend the cache the code runs through.
  - The cause is that boot's reset reason (`panic`, `interrupt-watchdog`, `task-watchdog`, `cpu-lockup`); a capture
    met after another reset (a brownout, the reset pin, before the record was written) is a `panic`. A boot reset
    after writing a record but before clearing its capture finds it the newest record already, and does not write it
    twice.
  - A reset in `panic`, `interrupt-watchdog`, `task-watchdog`, `watchdog`, `brownout`, `power-glitch` or
    `cpu-lockup` is recorded even with nothing captured, as a header alone (`length=0`, `intact=0`). A power-on,
    `reboot` or the reset pin leaves no record, unless a capture was still waiting.
  - That boot logs the record's first 16 lines (`panic :: ...`), so the session's `recorder.log` and CC's log stream
    carry it.
- **Not fatal** (`fatal=0`, an oops: the board noticed something broken and went on). `panics_logger::oops(cause,
  text)` never waits: the record goes through a queue of 8 to the `panic_log` thread (core 0, priority 2, PSRAM
  stack), which writes it. A full queue drops it, counted. The first producer: the C6 link parked by
  `hosted_no_reboot` (`cause=c6-link-parked`). Later milestones add theirs, such as a bus re-initialised after a
  general-call reset (M5).
- **Read-out:**
  - `panic log` (console and CC) lists the records, `seq<N>=<fatal|oops>:<cause>:<boot>:<bytes>`, and counts
    dropped oopses;
  - `GET /panic` on the file server serves the records as text, newest first (`?seq=N` for one; an empty body when
    there are none, 404 for a seq no slot holds, 503 when the log is not open). Its reads go through internal
    buffers, never into PSRAM (the file server's own memory);
  - `tools/panic_decode.py <board|file>` decodes MEPC, RA and the stack's code words against the firmware ELF with
    addr2line.
- **Test command:** `panic kind=rust|fault|abort|oops|twdt|stall confirm`, console-only and open in every stage (the
  warm-start proofs need it in the air; the hub never sends it). The first three reboot through a Rust panic, a store
  to address 0 and C's `abort()`; `oops` records a non-fatal one; `twdt` stops the supervisor's feed until the task
  watchdog panics; `stall` stops the control tick until the stall check restarts the board (M8). Each is refused where
  it cannot fire: no armed task watchdog, no stall check, or a stall restart already suppressed.
- **Proof (taster, 2026-10-07):**
  - each fatal kind was recorded by the next boot with 4.1–4.2 KB of text, `intact=1`, tagged with the boot that
    crashed;
  - a Rust panic names its task (`cc`) and source line;
  - the fault decodes to `main.rs:207`, the store itself;
  - 19 records went round-robin into 16 slots, the 3 oldest overwritten and `seq` going on;
  - a `reboot` added nothing;
  - oopses were written live with 0 dropped;
  - the boot's lines are in the next session's `recorder.log` on the card.

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
| `attitude.json`, `magnetic_yaw.json`, `on_backup.json` | `tasks/attitude.py`, cc_client `_on_backup` | angle 0.1 deg; flags exact | M9 (the drivers' M9 divergences pinned in `drivers_bmi323/bmp581/bmm350.json`: the BMI323's status burst, the BMP581's doubled ground reads, the BMM350's AGGR_SET 0x37; the physics vetoes' in `sequencer_trace.json`) |
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
| inspect | objects, inspect (mission, gnss, servo_* `props.trim` numeric, bluetooth `radio` bool and `active` null or bool), update (`changed`), stats | M1 (mission), M7 (bluetooth), M13 (servos) |
| control | stage, arm/disarm, report; gating; an arm while held refused with plan B's held-stage text under `stage`, then disarmed | M1 (stage; arm's refusal), M7 (a real arm; the arm while held) |
| diagnostics | probe (two maps), verify (keys; readiness names only plan B's checks; cc.py exit 0/1/3), calibrate, detect, bustune | M1 (probe, verify, calibrate answers), M5 (detect, bustune) |
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
  becomes a stub in plan B's no-BLE shape (`drivers/bluetooth.py:67-77`; M7): `inspect` adds `radio` (the config's,
  default false) and `active: null`; `update bluetooth {"radio": X}` stores a changed value and answers
  `["radio"]`, else `[]`; its probe logs plan B's `ble radio` lines and passes.
- The Luckfox path: UART recorder output and recorderd stay frozen and unchanged (the logger recorderd freeze
  stands). Fallback 1.a (C6 + SD logger) is not built.
- The `layout` scan re-clock dance: `i2c_master` speed is per device.

**Changed on purpose** (each is visible; the host already accepts it):
- **Changes only in SETTING** (the operator's rule, 2026-10-06): `update`, `set-config`, `reset-config`,
  `calibrate <device>`, `probe`, `detect`, `bustune` and `reboot` answer `err unsafe` in every other stage, DONE
  included; reads, `stage` and `disarm` stay open (M1). Plan B leaves `update` open in flight and allows the
  rest at DONE. There is no reboot at DONE: the beacon and the flash→SD copy run until power-off (M11b).
  - **The barometers' re-zero also runs at DONE** (operator, 2026-10-07; M7): `calibrate <barometer>` and `update
    <barometer> {"rezero"|"ground"}`, nothing else. Any elevation offset above `launch_alt_m` fires BOOSTING within
    ~150 ms of `stage setting`, and `stage setting` is never a hold, so with every fix SETTING-only only a power cycle
    recovered. Now: `stage done` (a hold), the re-zero, then `stage setting`.
- **Storage:** binary records in a flash ring that the board copies to its own SD (§3), instead of text lines
  over the UART to the Luckfox. With a full ring and no SD progress the newest data wins (§3.7). A line can
  reach the SD twice; the tools drop duplicates (§3.8, a filter added in M3).
- **DONE:** the camera stops (M7), the non-essential sensors stop, the GNSS drops to 1 Hz, and the beacon AP comes up
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
- **The link after the launch** (operator, 2026-10-07; M7): the radio is never switched off. Once the stage machine
  has detected the launch (`launched`, §4.1), the link never rejoins, hunts or dials; a session still up runs until
  range ends it, and at DONE the beacon AP is the only network (M11b). A commit to SETTING or an operator hold
  clears `launched`, so a held board keeps its normal reconnects. Plan B keeps the pad link too, but keys its pause
  on the stage: it stops initiating connections on any airborne stage, an operator hold included, and hunts again
  at DONE (`drivers/wifi.py:197-200`). A join already waiting for its address when the launch lands finishes that
  wait (up to 15 s) and is never cancelled, though it is never dialed; plan B abandons the wait and clears the
  pending attempt (`drivers/wifi.py:255-262`).
- **The stage machine's record:** `<entry>.csv` (`sequencer.csv`) is durable and never decimated; plan B wrote it
  through the global window (`tasks/sequencer.py:110`) (M7).
- **The stage machine's clock:** it steps on an exact `period_ms` grid of the uptime clock; plan B sleeps a period,
  then ticks (`tasks/sequencer.py:455-458`), so its steps drift. HITL allows for it (M14).
- **The sequencer's keys** (`Settings::from_config`, M7): the g and m keys take any JSON number (f32), a `*_ms`
  key a whole number from 0 to 2147483647, about 24.8 days (`300.0` included, which plan B flies alike): the machine
  compares a signed 32-bit elapsed time, which never reaches a longer threshold. A non-number, a fractional, a
  negative or a longer `*_ms`, or a non-number g or m key fails the entry at bring-up with `<key>: <why>`, so `arm`
  refuses. An absent key takes plan B's `setup()` default (`boost_timeout_ms` 6000; the config files ship 12000).
  Plan B, with the same files: a text `launch_g` or `still_g` fails setup; a text `period_ms` or `launch_alt_m`
  crashes the task on the pad, so arm refuses there too; a text `launch_ms` passes arm and crashes at ignition; the
  other keys pass arm and crash in their stage's detector in flight; a fractional or negative `*_ms` flies.
- **The altitude launch's rate gate** (operator, 2026-10-08; before M8): the baro launch backup also needs the
  elevation to have risen more than `launch_alt_m` within `launch_alt_window_ms` above the lowest level it held for
  `launch_ms`, through the whole `launch_ms` dwell. `launch_alt_window_ms` is a new sequencer key (default 1000; a
  `*_ms` by the rules above, at most 2000). A level counts only once held as long as the climb must hold, so one low
  or glitched read on a drifted pad is no pad to climb from; a window shorter than `launch_ms` turns the altitude
  launch off. Plan B launches on the elevation over `launch_alt_m` alone, which weather drift (+10 m in 2.17 h)
  satisfied on TMS-7F overnight, unarmed. The accel launch is unchanged. The history is a fixed ring of 81 slots of
  25 ms (972 B in the stage machine, on the control thread's stack); faster steps share a slot, which only shortens
  the window. sequencer_trace pins the plan B launches it removes as divergences: six whose elevation is over the
  line from its first fresh baro reading, two that rose from one low reading, and one 8 m/s climb.
- **Arm is checked again when it applies** (M7): on the control thread, SETTING and not held, else refused with the
  stage problem's text. Plan B awaits its probe sweep and then arms whatever the stage has become
  (`cc_client.py:567-576`).
- **Arm and verify sweep their probes only in SETTING** (M7); plan B also sweeps at DONE (`_GROUND`,
  `cc_client.py:352-375`). `verify` at DONE answers `problems.probe: not run`.
- **The ring gate** (M7) is a refusal plan B never had: `arm` and `verify` refuse when the bring-up scan has not run,
  the eraser is not running, or the erased reserve is short, reported as plan B's recorder probe under `recorder`.
- **The camera entry:** `ov5647 -- up` means configured (the bring-up never asks the sensor), and a camera never
  blocks `arm`, video being a utility; the video thread logs a start that fails at arm. Plan B has no `ov5647`
  implementation, fails the entry (`no driver/activity`) and refuses arm.
- **The separation in BOOSTING** (operator, 2026-10-07; M7) counts only once the thrust gate is open (thrust seen,
  then ended, on the body frame's +x) or by the 100 ms backstop (§4.3, step 3); plan B counts any open held for its
  20 ms debounce anywhere in BOOSTING (`drivers/separation.py:84`). `separation.csv` also records a `blip` (the
  contact moved and came back), beside plan B's `separated` and `nested`.
- **The controller's lines:** the hold, arm and disarm lines are recorded (recorder.log); `controller :: stage -> X`
  stays on the console, as plan B prints all of them (§4.4; M7).
- **Stale values in inspect:** a stale channel reads null instead of plan B's extrapolated `value()`.
- **HITL sensor mask:** after masking the `_SIM_SENSORS` sensors by config name, `sim_link.rs` refuses to start
  if one is still enabled; plan B only masks (M12).
- **The fin probe sweeps the flown range (the operator, 2026-10-11):** neutral -/+ `fins.mixer.limit_deg`, inside
  each servo's `min_deg..max_deg`. Plan B sweeps the servo's whole 0..180 deg range, past the linkage's deflection, so
  the wings had to be opened for every probe. The boot centring is staggered by `fins.concurrency`, where plan B
  centred every servo at once (M13).
- **`_readiness` and `enabled`:** Rust reads a missing `enabled` as enabled everywhere, the Controller's
  meaning. Python's `_readiness` read it as disabled, an inconsistency that is not ported.
- **`config_id` floats:** the CPython f64-repr semantics are used. Identical for every committed file (all
  floats have ≤ 7 significant digits). Plan B's announced id for the same file is cross-checked once on 7C.
- **Mission `sites`** are kept on `set-config launch`. Plan B silently dropped them (`mission.py` `persisted()`).
- **ICP general-call re-arm** re-initializes every device registered on the bus, by membership. Plan B got the
  same patch, its one sanctioned change (commit 76c138c, M5, §7.2 Q9).
- **INA226 `rearm()`** also restores ALERT_LIM and MASK.
- **VL53 poll mode** checks data-ready, so a repeated range is never republished as fresh.
- **The crumb (M8)** is saved only while armed and launched (the machine's launch), so a held armed stage writes none,
  and it is cleared on any edge out of that state, every failed save or clear retried; plan B saves on any armed
  airborne stage. Cost: plan B's simulated HITL reboot (an outage modelled as a disarm) cannot pass the gate.
  `checkpoint.csv` is durable and never decimated (the stage column the id, as plan B), and the clock is re-anchored
  to the RTC before each stamp.
- **The watchdog (M8)** watches the supervisor alone, plus the control tick's stall check; plan B's watched its whole
  event loop, a hung bus included (here a hung bus shows as failing reads, I2C transfers time out at 50 ms). A stall
  restart flushes its line first, and its own warm start never stall-restarts again. Watchdog and checkpoint keys
  are refused at bring-up, where plan B's arm refused them after a run crash (stricter: a text `stall_ms`, bools).
- **A warm start (M8)** prints its gate as text, opens `sequencer.csv` with `<stage>;external` only, records no video
  and stays off the link (`launched` is set); the pad latch stops at `launch_alt_m`/2 of elevation.
- **`panic`** (§4.8) is console-only and open in every stage, with the kinds `twdt` and `stall`.

- **M9, measured -> reported (the operator):** no stream is decimated and `telemetry_ms` is read by nothing; each part
  runs at its consumers' rate (the BMM350 at 12.5 Hz averaging 8, AGGR_SET 0x37; the `period_ms` of the BMI323,
  BMP581 and BMM350 entries unused); attitude.csv gets a row on every publish.
- **M9, `change`:** each barometer's climb rate, a new quantity registered with its `elevation` (no board-file key);
  the barometer row has a fifth column.
- **M9, freshness:** the IMUs publish only fresh conversions by their data-ready flags with byte corrections; the
  BMP581 and BMM350 by changed bytes; the BMI323's burst starts at STATUS 0x02 (16 B); the BMP581 reads twice and
  range-checks (its ground capture too); the BMM350's no-data frame is never observed. Plan B read blind and pushed
  whatever came, a 0x8000 frame included.
- **M9, the BMI323** is read on the owner's tick; its INT1 is evidence only (plan B sampled on the edge).
- **M9, the BMP581** gets `calibrate` and joins the re-zero (DONE, the warm rebase); plan B had none.
- **M9, the multi-source stage machine** (crates/sequencer) and vetoes.csv; its plan B divergences are pinned in
  `sequencer_trace.json` with a counting test.
- **M9, the filter's frame maps** from the marks frame: rate (x, -y, -z), gravity (-x, y, z), field (x, -y, -z), each
  flipping plan B's real-board sign of pitch, yaw rate and magnetic heading (plan B assumed gx->roll, gy->pitch,
  gz->yaw).
- **M9, the video** starts on the rod trigger (Rust-only; plan B records no video).
- **M9, `sources-disagree`** in `health` (Rust-only, information).
- **M9, the BMM350 calibration** is persisted from the executor, then applied on the owner (plan B's order, split
  across threads).
- **M9, the I2C owners wake half a tick after the control thread** (D20; plan B's tasks ran on their own sleeps):
  the gptimer alarms at twice the tick rate.
- **M9, diagnostics:** the bus diagnostics run only while disarmed (`bustune` armed only back to its configured
  clock); `imu mute` and `i2c read` are Rust-only forced-proof tools; `inspect databoard` / `stats databoard` as plan B;
  the BMI323 inspects `rate_dps` (deg/s, as the LSM6DSO32) where plan B had `rate_cds`.

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
| R5 | esp_hosted 2.7.0 tasks run at priority 23, unpinned (plain `xTaskCreate`); the streaming-mode length assert is still open. | lwIP's TCP/IP task is pinned to core 0 already (M1). M1b pins esp_hosted by wrapping `g_hosted_osi_funcs._h_thread_create`, the hook `components/hosted_no_reboot` uses for `_h_restart_host`. From M7 the radio stays on in Rust, but after the detected launch the link never rejoins, hunts or dials (§6), so esp_hosted carries no traffic once the pad session drops. |
| R6 | On 7C (v0.1 + SEN0697) a plan B ICP general call resets the BMI323, BMM350 and BMP581 without re-arming them. | Rust re-initializes every device registered on the bus after a general call (M5). Plan B is patched the same way (commit 76c138c, §7.2 Q9). |
| R7 | `gettimeofday` surviving SW, panic and TWDT resets on the P4; step 1's age check (plan B's gate) depends on it. | **Measured (M8):** a panic keeps the clock but returns it off by the drift since boot (+25.3 s / -22.6 s after ~9.7 h, ~700 ppm); a software reboot keeps it; a power-on loses it. The crumb re-anchors the clock before each stamp; the rod is under 30 min, so no resync (operator). Q19's step 2 drops the age check. |
| R8 | base64 strictness differs: CPython is lenient, Rust is strict. A bad token could answer differently from the fake. | Rust answers `err badargs` (strict). The contract runner records the fake's deviation as known. |
| R9 | Detector drift vs the host sim: `virtual_flight.py` grades the Python detectors once the board runs Rust. | `commons.json`, `sequencer_trace.json`, `control_trace_*.json` are the only guards. `generate.py --check` in CI. |
| R10 | HITL masking: a fresh bench sensor silently becomes the only source (pitot 7/27, mag 9/16). | `sim_link.rs` masks the `_SIM_SENSORS` sensors by config name (they are sensor names, not drivers), as plan B does, and re-applies the mask after layout. It then refuses to start if one is still enabled, which is new (plan B only masks). |
| R11 | Per-sensor axes: the LSM6DSO32 and BMI323 publish raw axes into one channel, and a failover may flip axes. | **Measured (M9):** the LSM6DSO32 reads the marks frame; TMS-7F's SEN0697 does not (BMI323 `-y+x+z`, BMM350 `+x-y-z`; its silkscreen shows y to the right) and carries both keys; unmapped, the taster's breadboard parts made a 4.8 deg attitude step at a failover, mapped TMS-7F's 0.26 deg. Every SEN0697 airframe is checked by hand once (tilts and a yaw both ways), and `health` reports `sources-disagree` past 10 deg at rest. |
| R12 | Two physics implementations would repeat the sim-mirrors-flight-code trap. | `sim_model.py` stays the only physics. Rust never ports it. |
| R13 | Silent-success harness traps (1 of 12 flown reported done; empty streams passed). | Every new runner counts artefacts and fails loudly. |
| R14 | Port 0 is shared with the camera SCCB, and `i2c_master`'s internal mutex arbitrates. | Camera register traffic only pre-flight. **Measured (M9 soak):** around the camera's start the BMI323's rows keep their 20 ms spacing, the BMP581's open to at most 39 ms, the BMM350's to 120 ms. |
| R15 | One reply line must stay under the hub's 2 MiB limit (`server.py:697`). | Tee capped at 256 KiB of records (~0.9 MB in base64 when full, with JSON quoting). `get-config` ~10 KB. |

### 7.2 Decided

The operator's decisions on the earlier questions, one line each:
- **Q1. SD layout:** one directory per session with long file names (the names carry the telemetry routing);
  video at `/sdcard/videos/<session>/<start uptime ms>.h264`, on a partition of its own (§3.10).
- **Q2. Changes in flight:** `update`, `set-config`, `reset-config`, `calibrate <device>`, `probe`, `detect`,
  `bustune` and `reboot` run only in SETTING; reads, `stage` and `disarm` stay open; no reboot at DONE. A
  deliberate divergence from plan B (§6). One exception (operator, 2026-10-07): the barometers' re-zero also runs at
  DONE, so a false launch is recovered without a power cycle (M7).
- **Q3. Reset at DONE:** any power-up after DONE is a new session in SETTING, never a warm start; a crumb in
  BOOSTING..LANDING means a warm start; no crumb and speed 0 means SETTING (§4.6).
- **Q4. `config_id` before vs after layout:** keep plan B's split, which is plan B behaviour: `set-config`
  returns the pre-layout id, `whoami` announces the post-layout one, and the boot log line names both (M5).
- **Q5. Partitions and OTA:** `otadata` plus `ota_0` and `ota_1` of 4 MB each (3 MB until 2026-10-12), a 1 MB `cfg` at 0x820000,
  `nvs` and `log` unchanged (§3.10). OTA itself comes later, over CC (M15). The 128 KB `panic` partition at 0x720000
  followed on 2026-10-07 (§4.8).
- **Q6. Watchdog timeout:** keep 5000 ms in board files (Python `validate` rejects < 4000) until Rust measures
  its own floor.
- **Q7. IMU axes and the body frame:** the body frame is +x nose, +y left, +z up (operator, 2026-10-07); the IMUs
  are mounted per their marks to match it, and an optional per-sensor `axes` key in the board config (a signed
  permutation such as `'-y+x+z'`, default identity) covers exceptions (M9). The separation's thrust gate reads +x
  from M7.
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
  (M7). On the SD, video is priority 1 and logging priority 2.
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
  not reset; with the Rust link policy that launch also stops the link from reconnecting once it drops (M7). The
  operator's call:
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
