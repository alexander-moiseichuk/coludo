//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The warm start on the board (crates/warmstart, src/glider warmstart.py; doc/specs/avionics-port.md §4.6, M8): the
//! crumb's NVS namespace over ESP-IDF, the boot's restore, and the checkpoint the supervisor runs.
//!
//! [`peek`] reads the crumb before the recorder opens, so a reboot in flight opens it with its flight flag up. Then
//! [`restore`] decides the boot on main (doc/specs/avionics-port.md "Bring-up order"), after the devices came up and
//! before the bus owners and the control thread start: the parts are still main's, and so is the flight word's
//! writer, so a passed gate rebases the barometers and commits the flight with nothing running beside it. It applies
//! the plan in plan B's order (crates/warmstart `ApplyPlan`) with plan B's lines: the gate, `controller :: stage -> X`
//! (to the console only, as the operator's holds), `controller :: armed`, then `warmstart :: WARM START -> X[, armed]`.
//! The gate line names the reset as text, as the boot line does, where plan B printed MicroPython's number. Every
//! outcome but a warm start lowers the flight flag the peek raised: a stage i32 whose blob does not read is a cold
//! boot with nothing cleared, and a flag kept up there would stay up on every boot after.
//!
//! [`Keeping`] is plan B's Checkpoint task: crates/warmstart's Keeper over this board's NVS and databoard, polled by
//! the supervisor, with its checkpoint.csv row and log line on every checkpoint (a stage change, and every
//! `checkpoint_s` aloft). The crumb is saved only while the board flies (armed and launched), cleared on any edge out
//! of that, and a refused NVS write is retried every poll until it lands. The recovery identity it freezes at
//! BOOSTING entry comes from the mission, which the executor owns: it republishes the zone and the launch point after
//! every line it runs ([`publish_mission`]), and the supervisor reads them only at the freeze. A mission with a zone
//! and no launch point yet (none set by the operator, none pinned at arm) takes the live GNSS fix there, as plan B's
//! freeze_launch at BOOSTING entry did; the zone's centre stands in when there is none.
//!
//! The airspeed seed goes to the flight law when the control thread starts (crate::flight, M13); M10 restores the
//! pitot tare. Not restored yet, logged once at the restore when a crumb carries it: the ground of a barometer other
//! than the ICP-10111. A warm-started flight records no video (the camera starts only when armed in SETTING) and never
//! rejoins the link (it is launched).
//!
//! The crumb lives in the shared `nvs` partition, namespace `coludo`, which plan B and the boot count share, while
//! the session-start record stays in `cfg`.

use std::ffi::CString;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use std::sync::{Mutex, OnceLock, PoisonError};

use databoard::Parameter;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use esp_idf_svc::sys;
use mission::{Mission, Position};
use recorder::{Cell, Telemetry};
use sequencer::{FlightWriter, Stage};
use serde::Serialize;
use serde_json::{Value, json};
use warmstart::{ApplyPlan, Freeze, Identity, Keeper, Live, Nvs, NvsError, Reason, ResetReason, Restore, Write};

use crate::clock;
use crate::devices::{CheckpointMember, Member, Topology};
use crate::logging::CONSOLE_ONLY;
use crate::status::{FLIGHT, WARM_STARTED};

/// The warm start's NVS namespace as crates/warmstart reads and writes it ([`Nvs`]): raw IDF calls, so each failure
/// is the `esp_err_t` [`NvsError`] carries (a key never written is ESP_ERR_NVS_NOT_FOUND, a blob longer than the
/// buffer ESP_ERR_NVS_INVALID_LENGTH), and a blob is written with IDF's own update, which writes the new entry before
/// it erases the old: esp-idf-svc's set_blob erases first, and a cut there would leave a stage i32 with no blob.
pub struct ColudoNvs(EspNvs<NvsDefault>);

impl ColudoNvs {
    /// The namespace on `partition`, created when absent (the boot count usually made it already).
    ///
    /// Errors: the namespace would not open.
    pub fn open(partition: &EspDefaultNvsPartition) -> Result<ColudoNvs, sys::EspError> {
        Ok(ColudoNvs(EspNvs::new(partition.clone(), warmstart::NAMESPACE, true)?))
    }
}

/// `key` as the C string IDF takes; NVS's own refusal of a bad key name for one with a NUL in it.
fn c_key(key: &str) -> Result<CString, NvsError> {
    CString::new(key).map_err(|_| NvsError(sys::ESP_ERR_NVS_INVALID_NAME))
}

/// An IDF answer as crates/warmstart's: Ok for ESP_OK, else the code.
fn answer(code: sys::esp_err_t) -> Result<(), NvsError> {
    if code == sys::ESP_OK {
        Ok(())
    } else {
        Err(NvsError(code))
    }
}

impl Nvs for ColudoNvs {
    /// IDF's `nvs_get_i32`.
    fn get_i32(&mut self, key: &str) -> Result<i32, NvsError> {
        let key = c_key(key)?;
        let mut value = 0;
        // SAFETY: a valid C string and a live output on an open handle.
        answer(unsafe { sys::nvs_get_i32(self.0.handle(), key.as_ptr(), &mut value) })?;
        Ok(value)
    }

    /// IDF's `nvs_get_blob` into `buffer`, its length the most it reads.
    fn get_blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, NvsError> {
        let key = c_key(key)?;
        let mut length = buffer.len();
        // SAFETY: a valid C string on an open handle; IDF writes at most `length` bytes into the buffer and refuses a
        // longer blob (ESP_ERR_NVS_INVALID_LENGTH), then sets `length` to what it wrote.
        answer(unsafe { sys::nvs_get_blob(self.0.handle(), key.as_ptr(), buffer.as_mut_ptr().cast(), &mut length) })?;
        Ok(length)
    }

    /// IDF's `nvs_set_blob`: the new entry written before the old one is erased.
    fn set_blob(&mut self, key: &str, value: &[u8]) -> Result<(), NvsError> {
        let key = c_key(key)?;
        // SAFETY: a valid C string and a byte slice of the given length on an open handle.
        answer(unsafe { sys::nvs_set_blob(self.0.handle(), key.as_ptr(), value.as_ptr().cast(), value.len()) })
    }

    /// IDF's `nvs_set_i32`.
    fn set_i32(&mut self, key: &str, value: i32) -> Result<(), NvsError> {
        let key = c_key(key)?;
        // SAFETY: a valid C string on an open handle.
        answer(unsafe { sys::nvs_set_i32(self.0.handle(), key.as_ptr(), value) })
    }

    /// IDF's `nvs_commit`.
    fn commit(&mut self) -> Result<(), NvsError> {
        // SAFETY: an open handle.
        answer(unsafe { sys::nvs_commit(self.0.handle()) })
    }
}

/// Whether NVS holds a crumb [`warmstart::load`] reads in BOOSTING..LANDING: the recorder opens with its flight flag
/// up then, so a reboot in flight writes flight blocks from its first record. [`restore`] lowers it again for every
/// outcome but a warm start.
pub fn peek(partition: &EspDefaultNvsPartition) -> bool {
    ColudoNvs::open(partition)
        .ok()
        .and_then(|mut nvs| warmstart::load(&mut nvs))
        .and_then(|crumb| crumb.stage)
        .and_then(|id| u8::try_from(id).ok())
        .and_then(Stage::from_id)
        .is_some_and(Stage::airborne)
}

/// How this boot's restore ended: `inspect`'s gate, and the stall restart's guard.
#[derive(Clone, Copy, Debug)]
enum Outcome {
    /// No crumb read: a cold boot, nothing cleared.
    Cold,
    /// A crumb, but the checkpoint entry's `warm_start` is off: a cold boot, the crumb cleared.
    Disabled,
    /// The gate refused, for this reason: a cold boot, the crumb cleared.
    Refused(Reason),
    /// The gate passed, for this reason: the flight restored.
    Warm(Reason),
}

/// This boot's outcome, set once by [`restore`], before any thread that reads it runs.
static OUTCOME: OnceLock<Outcome> = OnceLock::new();

/// Whether this boot restarts the board for no control stall (crates/warmstart
/// `Restore::stall_restart_suppressed`), set once by [`restore`], before the supervisor runs.
static STALL_RESTART_SUPPRESSED: AtomicBool = AtomicBool::new(false);

/// Decides this boot from the crumb (plan B `restore`) and applies a warm start, on main after the devices came up
/// and before the bus owners and the control thread start (see the module notes). The gate takes the checkpoint
/// entry's `warm_start` toggle, the separation switch's counted latch, this boot's reset reason, and the clock in
/// seconds since 2000; a refusal clears the crumb, so the next boot is cold.
///
/// Args:
/// * `partition` - the shared `nvs` partition, the crumb's.
/// * `document` - the board config this boot runs: the toggle, and its sensors' drivers for the barometers' rebase.
/// * `separated` - the separation switch's counted latch at setup; false without a switch.
/// * `mission` - the mission, its launch point and zone restored from the crumb.
/// * `members` - the I2C parts that came up, by port: the barometers rebased to the crumb's pad.
/// * `writer` - the flight word's one writer, before the control thread takes it: the stage and the arming commit
///   through it.
/// * `flight_flag` - whether the recorder opened with its flight flag up ([`peek`]): lowered here unless the boot is
///   warm.
///
/// Returns: the restored recovery identity, the checkpoint's seed, on a warm start; None on any other boot.
pub fn restore(
    partition: &EspDefaultNvsPartition,
    document: &Value,
    separated: bool,
    mission: &mut Mission,
    members: &mut [Vec<Member>; 2],
    writer: &mut FlightWriter<'static>,
    flight_flag: bool,
) -> Option<Identity> {
    // SAFETY: a plain read of the reset cause latched at boot.
    let reset = ResetReason::from_code(unsafe { sys::esp_reset_reason() });
    let decision = match ColudoNvs::open(partition) {
        Ok(mut nvs) => warmstart::restore(
            &mut nvs,
            board_config::warm_start(document),
            separated,
            reset,
            clock::epoch() - warmstart::UNIX_SECONDS_AT_2000,
            sensors(document),
        ),
        Err(error) => {
            log::error!(
                "warmstart :: the NVS namespace '{}' would not open, so no crumb is read: {error}",
                warmstart::NAMESPACE
            );
            Restore::Cold
        }
    };
    STALL_RESTART_SUPPRESSED.store(decision.stall_restart_suppressed(reset), Ordering::Relaxed);
    if flight_flag != decision.keeps_flight()
        && let Some(recorder) = crate::recorder::get()
    {
        recorder.set_flight(decision.keeps_flight());
    }
    let gate = |reason: &Reason| {
        log::info!(
            "warmstart :: gate: {reason} (reset_cause {})",
            crate::board::reset_reason()
        );
    };
    let (outcome, seed) = match decision {
        Restore::Cold => (Outcome::Cold, None),
        Restore::Disabled => {
            log::info!("warmstart :: disabled by config");
            (Outcome::Disabled, None)
        }
        Restore::Refused(reason) => {
            gate(&reason);
            (Outcome::Refused(reason), None)
        }
        Restore::Warm { reason, plan } => {
            gate(&reason);
            apply(&plan, mission, members, writer);
            (Outcome::Warm(reason), Some(plan.identity))
        }
    };
    let _ = OUTCOME.set(outcome);
    seed
}

/// Each of `baros`' own ground at `now_us`, by their order: its altitude less its elevation, from its own channels
/// (a barometer pushes both from one conversion, so the difference is the ground it flew with). None for one with
/// neither fresh; slots past [`warmstart::GROUND_SLOTS`] are dropped.
fn own_grounds(
    topology: &'static Topology,
    baros: &[&'static str],
    now_us: u64,
) -> [Option<f32>; warmstart::GROUND_SLOTS] {
    let fresh = |quantity: &str, name: &str| {
        let parameter = topology.scalar(quantity)?;
        parameter.channel(name)?.recent(parameter.window_us(), now_us)
    };
    let mut grounds = [None; warmstart::GROUND_SLOTS];
    for (slot, name) in grounds.iter_mut().zip(baros) {
        *slot = fresh("altitude", name)
            .zip(fresh("elevation", name))
            .map(|(altitude, elevation)| altitude - elevation);
    }
    grounds
}

/// The board config's sensors as (name, driver), in config order, enabled or not: plan B's restore walks
/// `cfg['sensors']` for the barometers to rebase, and rebases those that came up.
fn sensors(document: &Value) -> Vec<(&str, &str)> {
    document["sensors"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|sensor| Some((sensor["name"].as_str()?, sensor["driver"].as_str()?)))
        .collect()
}

/// Applies a passed gate's `plan` in plan B's order (`_apply_restore`, then its `set_stage` and `arm`): the mission's
/// launch point and zone; each barometer that came up rebased to the pad, logged either way (the ICP-10111 takes it
/// as its `ground`; one whose driver takes none yet is not rebased); the lines for what is not restored yet; then the stage and the arming
/// committed through `writer` (launched, no hold), their lines, and the WARM-STARTED mark.
fn apply(plan: &ApplyPlan, mission: &mut Mission, members: &mut [Vec<Member>; 2], writer: &mut FlightWriter<'static>) {
    if let Some(update) = plan.mission_update() {
        mission.update(&update);
    }
    if let Some(rebase) = &plan.rebase {
        let now_us = uptime_us();
        for member in members.iter_mut().flatten() {
            let Some(index) = rebase.baros.iter().position(|name| name == member.name) else {
                continue;
            };
            // its own ground from the crumb, else the pad altitude (plan B's one ground for every barometer)
            let ground_m = rebase.ground_of(index);
            match member.part.update(None, &json!({"ground": ground_m}), now_us) {
                Ok(changed) if changed.is_empty() => log::info!(
                    "warmstart :: {} not rebased to the pad: its driver publishes no elevation yet",
                    member.name
                ),
                // the ground it took, every digit: its setup's own re-zero lands within the baro's noise of the pad,
                // so only this line tells the rebase happened
                Ok(_) => log::info!(
                    "warmstart :: {} rebased to the pad: ground {ground_m} m{}",
                    member.name,
                    if rebase.grounds_m.get(index).copied().flatten().is_some() {
                        " (its own)"
                    } else {
                        ""
                    }
                ),
                Err(refusal) => log::warn!("warmstart :: {} refused the pad as its ground: {refusal}", member.name),
            }
        }
    }
    if let Some(zero) = plan.pitot_zero_pa {
        let restored = members
            .iter_mut()
            .flatten()
            .filter_map(|member| member.part.restore_pitot_zero(zero).then_some(member.name))
            .next();
        match restored {
            Some(name) => log::info!("warmstart :: {name} tare restored: {zero} Pa"),
            None => log::info!("warmstart :: pitot tare {zero} Pa not restored: no pitot came up"),
        }
    }
    if let Some(airspeed) = plan.airspeed_ms {
        crate::flight::keep_warm_airspeed(airspeed); // the flight law's seed when the control thread starts
    }
    if writer.restore(plan.stage, plan.armed).is_none() {
        return;
    }
    let stage = plan.stage.name();
    log::info!(target: CONSOLE_ONLY, "controller :: stage -> {stage}");
    if plan.armed {
        log::info!("controller :: armed");
    }
    WARM_STARTED.store(true, Ordering::Relaxed);
    let armed = if plan.armed { ", armed" } else { "" };
    log::info!(
        "warmstart :: WARM START -> {stage}{armed} (warm start {} of this flight)",
        plan.identity.warm_starts
    );
    if plan.capped {
        log::warn!(
            "warmstart :: past {} warm starts this flight: recovered DISARMED -- the law and the fins stay off, the \
             stage machine walks the flight to DONE",
            warmstart::WARM_CAP
        );
    }
}

/// Whether this boot is a stall restart's own warm start, warm after a software reset: the supervisor then restarts
/// for no stall, so a stall that comes back every boot leaves the board stalled rather than cycling it
/// (crates/warmstart `Restore::stall_restart_suppressed`).
pub fn stall_restart_suppressed() -> bool {
    STALL_RESTART_SUPPRESSED.load(Ordering::Relaxed)
}

/// This boot's gate, as `inspect` shows it: the gate line's text (`<reason> (reset_cause <cause>)`), `no crumb` for
/// a boot that read none, `disabled by config` for a crumb the toggle turned away.
fn gate() -> String {
    let cause = crate::board::reset_reason();
    match OUTCOME.get() {
        Some(Outcome::Cold) | None => format!("no crumb (reset_cause {cause})"),
        Some(Outcome::Disabled) => "disabled by config".to_string(),
        Some(Outcome::Refused(reason) | Outcome::Warm(reason)) => format!("{reason} (reset_cause {cause})"),
    }
}

/// The mission's zone and launch point as the checkpoint freezes them at BOOSTING entry ([`Freeze`]): the executor
/// owns the mission and republishes them after every line it runs, main once before the threads start. Core 0 only:
/// the executor and the supervisor lock it, never the flight core.
static MISSION: Mutex<(Option<[Position; 2]>, Option<Position>)> = Mutex::new((None, None));

/// Publishes `mission`'s zone and launch point for the checkpoint's freeze: the operator's point or the one `arm`
/// pinned, None without one (the freeze then takes the live fix).
pub fn publish_mission(mission: &Mission) {
    // the flight loop's own view, lock-free: the control thread may take no lock
    crate::flight::publish_mission(mission);
    let zone = mission.zone.map(|zone| [zone.top_left, zone.bottom_right]);
    *MISSION.lock().unwrap_or_else(PoisonError::into_inner) = (zone, mission.launch_point(None));
}

/// The NVS action the checkpoint owes, as [`OWED`] holds it: none.
const OWED_NOTHING: u8 = 0;

/// [`OWED`]: the flight's crumb.
const OWED_CRUMB: u8 = 1;

/// [`OWED`]: the clear.
const OWED_CLEAR: u8 = 2;

/// The NVS action the checkpoint owes ([`Keeper::owed`]), for `inspect`: one of the `OWED_` values.
static OWED: AtomicU8 = AtomicU8::new(OWED_NOTHING);

/// Crumbs committed since boot.
static SAVES: AtomicU32 = AtomicU32::new(0);

/// NVS actions refused since boot, crumbs and clears alike.
static FAILURES: AtomicU32 = AtomicU32::new(0);

/// Attempts at an action an earlier pass's refusal left owed.
static RETRIES: AtomicU32 = AtomicU32::new(0);

/// The longest pass that tried an NVS action, from the flight word's load to the action's answer, µs.
static SAVE_MAX_US: AtomicU32 = AtomicU32::new(0);

/// The rebased barometers' names in config order, the [`Identity::grounds_m`] slots' (set when the keeper opens).
static BAROS: OnceLock<Vec<&'static str>> = OnceLock::new();

/// The recovery identity the crumbs carry now, as the supervisor's keeper last published it: what a reset would
/// recover to. Core 0 only (the supervisor writes it, the executor reads it for `inspect`).
static IDENTITY: Mutex<Option<Identity>> = Mutex::new(None);

/// The checkpoint as the supervisor runs it (plan B's Checkpoint task): the keeper over this board's NVS, the
/// quantities it reads, and its stream.
pub struct Keeping {
    /// The entry's name: its log lines'.
    name: &'static str,
    /// The schedule and the NVS rules (crates/warmstart).
    keeper: Keeper,
    /// The `coludo` namespace, the supervisor's own handle.
    nvs: ColudoNvs,
    /// The `altitude` quantity: the crumb's, the row's, and in SETTING the pad's.
    altitude: Option<&'static Parameter<f32>>,
    /// The `elevation` quantity: the pad latch's ceiling is read against it.
    elevation: Option<&'static Parameter<f32>>,
    /// The quantities, for the live GNSS fix the freeze pins when no launch point is set.
    topology: &'static Topology,
    /// The rebased barometers' names, in the config order of crates/warmstart `REBASED_BAROS` drivers: whose own
    /// grounds the freeze keeps.
    baros: Vec<&'static str>,
    /// checkpoint.csv, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// An NVS refusal was logged and no action has committed since: one line per run of refusals.
    refusing: bool,
    /// A row was refused and none written since: one line per run.
    overflowing: bool,
}

impl Keeping {
    /// The checkpoint of `member`: its NVS handle opened on `partition`, its quantities found in `topology`, its pad
    /// latch under `pad_ceiling_m` metres of elevation (half the stage machine's `launch_alt_m`; None without a stage
    /// machine, which latches as plan B), seeded with a warm start's restored identity when this boot is one, and
    /// freezing the own grounds of the barometers `document` rebases.
    ///
    /// Errors: the `coludo` namespace would not open (the entry is crashed with it: no crumb could be kept).
    pub fn open(
        member: CheckpointMember,
        partition: &EspDefaultNvsPartition,
        topology: &'static Topology,
        document: &'static Value,
        pad_ceiling_m: Option<f32>,
        seed: Option<Identity>,
    ) -> Result<Keeping, String> {
        let nvs = ColudoNvs::open(partition)
            .map_err(|error| format!("the NVS namespace '{}' would not open: {error}", warmstart::NAMESPACE))?;
        let mut keeper = Keeper::new(member.checkpoint_s, uptime_us(), pad_ceiling_m);
        if let Some(identity) = seed {
            keeper.seed(identity);
        }
        Ok(Keeping {
            name: member.name,
            keeper,
            nvs,
            altitude: topology.scalar("altitude"),
            elevation: topology.scalar("elevation"),
            topology,
            baros: BAROS
                .get_or_init(|| {
                    sensors(document)
                        .into_iter()
                        .filter(|(_name, driver)| warmstart::REBASED_BAROS.contains(driver))
                        .map(|(name, _driver)| name)
                        .collect()
                })
                .clone(),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(member.name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            refusing: false,
            overflowing: false,
        })
    }

    /// The entry's name.
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// How often the supervisor runs a [`pass`](Keeping::pass), ms: at most 500, so a stage change is caught
    /// promptly.
    pub fn poll_ms(&self) -> u64 {
        self.keeper.poll_ms()
    }

    /// One poll, its schedule at `due_us` (the supervisor's grid): the keeper's pass on one load of the flight word
    /// and the fresh readings, the clock read only when a crumb is tried (seconds since 2000, the crumb's epoch),
    /// then, for a checkpoint due, the row and the log line (plan B's `checkpoint <stage> alt=<altitude>`), after the
    /// crumb, so a full ring never costs it.
    pub fn pass(&mut self, due_us: u64) {
        // the readings take the uptime now, not the grid's: a sample pushed since would read as one from the future,
        // which the databoard counts stale
        let now_us = uptime_us();
        let flight = FLIGHT.load();
        let altitude_m = self
            .altitude
            .and_then(|parameter| parameter.read(now_us))
            .map(|reading| reading.value);
        // the pad latch's ceiling never trusts the GNSS's elevation, as the stage machine never launches on it
        let elevation_m = self
            .elevation
            .and_then(|parameter| parameter.read(now_us))
            .filter(|reading| reading.source != crate::control::GNSS_SOURCE)
            .map(|reading| reading.value);
        let live = Live {
            altitude_m,
            speed_ms: None,
            airspeed_ms: crate::flight::airspeed(),
            ticks_ms: now_us / 1000,
        };
        let topology = self.topology;
        let baros = &self.baros;
        let freeze = || {
            let (zone, set) = *MISSION.lock().unwrap_or_else(PoisonError::into_inner);
            // plan B warmstart.py:359: a fix that arrived after the arm pins here, the last ground moment; the
            // operator's point and the arm's win, and a mission with no zone freezes none
            let launch_point = set.or_else(|| {
                let fix = crate::gnss::position(topology).filter(|_| zone.is_some())?;
                let [latitude, longitude] = fix.degrees();
                log::info!("mission :: launch point frozen ({latitude:.5}, {longitude:.5})");
                Some(fix)
            });
            Freeze {
                zone,
                launch_point,
                fresh_altitude_m: altitude_m,
                // the pad's calibration, frozen with the rest of the recovery identity (plan B warmstart.py:373-384):
                // a pitot that runs gives its zero offset, 0.0 Pa a valid tare
                pitot_zero_pa: crate::sensors::pitot_inspection().map(|(_, _, zero_pa, _, _)| zero_pa),
                grounds_m: own_grounds(topology, baros, now_us),
            }
        };
        let stamp_s = || clock::epoch() - warmstart::UNIX_SECONDS_AT_2000;
        let pass = self
            .keeper
            .pass(&mut self.nvs, flight, due_us, elevation_m, live, freeze, stamp_s);
        if pass.wrote.is_some() || pass.failed.is_some() {
            let spent_us = uptime_us().saturating_sub(now_us);
            SAVE_MAX_US.fetch_max(u32::try_from(spent_us).unwrap_or(u32::MAX), Ordering::Relaxed);
        }
        self.publish();
        match (pass.failed, self.keeper.owed()) {
            (Some(NvsError(code)), Some(owed)) if !self.refusing => {
                log::warn!(
                    "{} :: NVS refused the {} ({code:#x}): tried again every poll",
                    self.name,
                    action(owed)
                );
                self.refusing = true;
            }
            (None, _) if pass.wrote.is_some() => self.refusing = false,
            _ => {}
        }
        if let Some(stage) = pass.row {
            self.record(now_us, stage, altitude_m);
        }
    }

    /// Publishes the keeper's owed action and tallies for `inspect`.
    fn publish(&self) {
        let owed = match self.keeper.owed() {
            None => OWED_NOTHING,
            Some(Write::Crumb) => OWED_CRUMB,
            Some(Write::Clear) => OWED_CLEAR,
        };
        OWED.store(owed, Ordering::Relaxed);
        let counters = self.keeper.counters();
        SAVES.store(counters.saves, Ordering::Relaxed);
        FAILURES.store(counters.failures, Ordering::Relaxed);
        RETRIES.store(counters.retries, Ordering::Relaxed);
        let identity = *self.keeper.identity();
        let mut published = IDENTITY.lock().unwrap_or_else(PoisonError::into_inner);
        if *published != Some(identity) {
            *published = Some(identity);
        }
    }

    /// One checkpoint's row at `now_us` (plan B warmstart.py `_checkpoint`): the stage's id, the altitude, the speed
    /// (none) and the flight law's airspeed, the uptime in ms; then its log line. A refused row is logged once a run: the
    /// crumb went first either way.
    fn record(&mut self, now_us: u64, stage: Stage, altitude_m: Option<f32>) {
        if let Some(telemetry) = &mut self.telemetry {
            let row = [
                Cell::I32(i32::from(stage as u8)),
                altitude_m.map_or(Cell::None, Cell::F32),
                Cell::None, // speed
                crate::flight::airspeed().map_or(Cell::None, Cell::F32),
                Cell::I64((now_us / 1000) as i64),
            ];
            match telemetry.push(now_us, &row) {
                Ok(()) => self.overflowing = false,
                Err(overflow) if !self.overflowing => {
                    log::warn!("{} :: record {overflow}", self.name);
                    self.overflowing = true;
                }
                Err(_) => {}
            }
        }
        let altitude = altitude_m.map_or_else(|| "None".to_string(), |altitude| altitude.to_string());
        log::info!("{} :: checkpoint {} alt={altitude}", self.name, stage.name());
    }
}

/// An NVS action's name, as `inspect` and the refusal line write it.
fn action(write: Write) -> &'static str {
    match write {
        Write::Crumb => "crumb",
        Write::Clear => "clear",
    }
}

/// What `inspect` shows of the checkpoint beyond its entry: the crumb NVS holds now, how this boot's restore ended,
/// the NVS action owed, and the tallies since boot.
#[derive(Serialize)]
pub struct Inspection {
    /// The crumb a fresh [`warmstart::load`] reads, as its JSON (the stage the i32's); null when none reads.
    crumb: Option<Value>,
    /// This boot's gate (see [`gate`]).
    gate: String,
    /// The NVS action not committed yet: `crumb`, `clear`, or null.
    owed: Option<&'static str>,
    /// Crumbs committed.
    saves: u32,
    /// NVS actions refused.
    failures: u32,
    /// Attempts at an action left owed.
    retries: u32,
    /// The longest pass that tried an NVS action, µs.
    save_max_us: u32,
    /// The recovery identity the crumbs carry now (the port's own): null until the keeper ran, its fields null until
    /// a BOOSTING entry froze them.
    identity: Option<IdentityView>,
}

/// The recovery identity as `inspect checkpoint` shows it: what a reset in flight would recover to.
#[derive(Serialize)]
pub struct IdentityView {
    /// The pad altitude, m: every barometer's ground when the crumb carries none of its own.
    pad_altitude_m: Option<f32>,
    /// Each rebased barometer's own ground, m, by name.
    grounds_m: serde_json::Map<String, Value>,
    /// The pitot's tare, Pa.
    pitot_zero_pa: Option<f32>,
    /// The flight's warm starts so far.
    warm_starts: u32,
}

/// The checkpoint's [`Inspection`], its crumb read now from `partition` (on the executor, beside the supervisor's
/// handle: NVS serialises its own calls).
pub fn inspect(partition: &EspDefaultNvsPartition) -> Inspection {
    let crumb = ColudoNvs::open(partition)
        .ok()
        .and_then(|mut nvs| warmstart::load(&mut nvs))
        .and_then(|crumb| serde_json::from_str(&crumb.to_json()).ok());
    let owed = match OWED.load(Ordering::Relaxed) {
        OWED_CRUMB => Some(action(Write::Crumb)),
        OWED_CLEAR => Some(action(Write::Clear)),
        _ => None,
    };
    Inspection {
        crumb,
        gate: gate(),
        owed,
        saves: SAVES.load(Ordering::Relaxed),
        failures: FAILURES.load(Ordering::Relaxed),
        retries: RETRIES.load(Ordering::Relaxed),
        save_max_us: SAVE_MAX_US.load(Ordering::Relaxed),
        identity: IDENTITY
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .map(|identity| IdentityView {
                pad_altitude_m: identity.pad_altitude_m,
                grounds_m: BAROS
                    .get()
                    .into_iter()
                    .flatten()
                    .zip(identity.grounds_m)
                    .map(|(name, ground)| ((*name).to_string(), ground.map_or(Value::Null, Value::from)))
                    .collect(),
                pitot_zero_pa: identity.pitot_zero_pa,
                warm_starts: identity.warm_starts,
            }),
    }
}

/// The uptime in µs (esp_timer, unwrapped).
fn uptime_us() -> u64 {
    // SAFETY: a plain read of the monotonic system timer, never negative.
    unsafe { sys::esp_timer_get_time() as u64 }
}
