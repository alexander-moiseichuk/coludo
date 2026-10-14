//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! In-flight reboot recovery (src/glider warmstart.py; doc/specs/coludo.md "In-flight reboot & warm start", and the
//! boot rules of doc/specs/avionics-port.md §4.6). A mid-air reset (a watchdog, a panic, a crash) must not turn the
//! glider ballistic. The checkpoint keeps a tiny CRUMB in NVS carrying the live flight state; at boot, before any
//! thread runs, the crumb and the physical signals decide whether the board resumes the saved stage (a warm start)
//! or boots cold, and the normal detectors (separation, apogee, landing) re-evaluate from there.
//!
//! The crumb lives in NVS, never in a file: a file write locks the scheduler and wears the data flash, while NVS
//! commits to its own partition in milliseconds. It is two keys in the namespace [`NAMESPACE`], the same as plan B's,
//! so a crumb plan B wrote reads back here and the other way round:
//!  * [`CRUMB_KEY`], one JSON blob of at most [`BLOB_MAX`] bytes (about 370 in practice): full precision, no
//!    per-field key bookkeeping, and a new field is a member rather than an NVS schema change;
//!  * [`STAGE_KEY`], the stage id as an i32, written LAST. It IS the flag: 0 ([`NO_STAGE`], what an unwritten key
//!    reads as) means no flight was checkpointed, anything else is the stage to recover. Written last, a torn
//!    write leaves the previous stage, never a new stage pointing at a half-written blob; the i32 is authoritative
//!    over the blob's own copy.
//!
//! Only a FLYING board writes a crumb: armed (a disarmed passive flight must never warm-start into an armed stage)
//! and launched by its own stage machine (an operator's held stage is a bench test, never a flight to resume). The
//! recovery identity (the launch point, the zone, the pad altitude, the pitot tare, the gyro biases) is frozen once,
//! at BOOSTING entry, and rides every later crumb.
//!
//! The pieces, each plan B's function as data, so the firmware's `warm.rs` and supervisor only move bytes:
//!  * [`Crumb`], with [`Crumb::from_json`] and [`Crumb::to_json`], and [`save`]/[`clear`]/[`load`] over the [`Nvs`]
//!    trait (the firmware implements it on esp-idf NVS; the tests on memory);
//!  * [`should_restore`], the gate, with [`ResetReason`] (the ESP-IDF reset reasons and which count as a reset);
//!  * [`ApplyPlan`], what a passed gate restores, in plan B's order;
//!  * [`restore`], plan B's boot decision end to end;
//!  * [`Checkpoint`], plan B's Checkpoint task as a pure schedule: when to write, and the identity it carries;
//!  * [`Keeper`], the schedule over NVS: a crumb only while flying, a clear when the flight ends, and a refused
//!    write retried until it lands.
//!
//! Nothing here reads a clock: the time is an argument. A crumb's `stamp` is whole seconds since 2000-01-01 UTC,
//! MicroPython's esp32 epoch, so plan B's crumbs and these share one clock; the firmware subtracts
//! [`UNIX_SECONDS_AT_2000`] from `gettimeofday`. Metres, m/s and pascals are f32 (decision D18); the launch and
//! zone are `Position` integers in 1e-7 degrees, f64 only as JSON numbers.
//!
//! Deliberate differences from plan B (doc/specs/avionics-port.md §4.6, M8; each one marked in the golden fixture,
//! but the last, which no exact fixture case can hold and the unit tests pin):
//!  * the operator's boot rules of 2026-10-06: any power-up after DONE is a new session in SETTING (plan B restored
//!    DONE after a reset within 600 s), and only a crumb in BOOSTING..LANDING warm-starts (plan B restored SETTING,
//!    re-armed, after a reset on the pad). No crumb is SETTING in both;
//!  * a crumb stage that is no flight stage is refused. Plan B passed it ('recover ?') and then raised in
//!    `Controller.set_stage()`, crashing the boot halfway through the restore;
//!  * Q19's step 2 (the operator, 2026-10-13): the gate makes no time or cause check. With no battery-backed RTC a
//!    power cycle loses the clock, so a crumb's age says nothing, and a battery bounce in flight is a power-on too;
//!    only the separation latch refuses a glide stage. Plan B refused a power-on (and every cause MicroPython maps
//!    to cold, the P4's CPU_LOCKUP among them) and a crumb outside 0..600 s. A crumb of an interrupted flight so
//!    recovers whenever the board comes back, and the stage machine walks it on to DONE (a lying glider reaches DONE
//!    by stillness); the next power-up after DONE is a new session;
//!  * a crumb blob that is JSON but not an object reads as no crumb; plan B's load() raised TypeError there and
//!    crashed the boot. A value of the wrong JSON type reads as absent, where plan B computed on whatever arrived;
//!  * the writer never puts NaN or infinity in the blob (they become null); CPython would write NaN, which a board's
//!    MicroPython json cannot read back;
//!  * the zone's centre, which stands in for a missing launch point at the freeze, is the integer mean of the corners'
//!    1e-7 degree units, rounded toward zero as the mission's zone centre is. Plan B averages the float degrees, so
//!    when the corners' units sum to an odd number the two differ by half a unit (5.6 mm). The checkpoint fixture
//!    compares exactly and holds coordinates of at most 7 decimals, so it cannot carry that half unit.
//!
//! The [`Keeper`] departs from plan B's Checkpoint task on purpose too, each departure pinned by the unit tests:
//!  * plan B saved a crumb whenever the board was armed; the keeper saves one only while armed AND launched, so an
//!    armed bench hold (`arm`, then `stage gliding`) writes none, and a reset never resumes it as a launched flight;
//!  * plan B left the last crumb in NVS after a disarm; the keeper clears it on any edge out of flying (a disarm, or
//!    a hold, which clears `launched`). The recovery from a false launch (disarm, `stage done`, the re-zero, `stage
//!    setting`) so leaves no armed BOOSTING crumb for a later boot to warm-start on the pad. The cost: plan B's
//!    simulated HITL reboot, whose outage begins with a disarm, finds no crumb;
//!  * plan B wrote each crumb once, whatever NVS answered; the keeper retries a refused crumb or clear on every poll
//!    until it lands. The schedule never re-checkpoints DONE, so a lost DONE crumb left the last airborne one, and a
//!    reset then warm-started a landed glider;
//!  * plan B latched every fresh altitude in SETTING as the pad; the keeper latches one only while the elevation is
//!    under a ceiling (half the sequencer's `launch_alt_m`). A launch the accel trigger missed stays in SETTING
//!    through the baro launch dwell, and plan B latched its "pad" metres up the climb.
//!
//! Not ported: the GC handling (plan B collected and disabled the GC on an airborne warm start), the log lines
//! (the firmware logs the [`Reason`] it gets back), and MicroPython's SOFT_RESET (an interpreter reboot without a
//! chip reset), which has no ESP-IDF reason.

use control::attitude::BIAS_SLOTS;
use control::navigation::Position;
use sequencer::{Flight, Stage};
use serde::Serialize;
use serde_json::{Map, Value, json};
use std::fmt;

/// The NVS namespace both firmwares keep their small state in (plan B `NVS('coludo')`).
pub const NAMESPACE: &str = "coludo";

/// The NVS key of the crumb blob, the JSON document.
pub const CRUMB_KEY: &str = "crumb";

/// The NVS key of the stage i32, the flag written last.
pub const STAGE_KEY: &str = "stage";

/// The stage i32 that means "no crumb": what an unwritten key reads as, and what [`clear`] writes (plan B
/// `Stage.NULL`). Never a live stage.
pub const NO_STAGE: i32 = 0;

/// The read buffer for the crumb blob (plan B `_BLOB_MAX`). A real crumb is about 370 bytes (two gyros' biases),
/// 410 with four; the largest the writer can produce, every field at the extreme of what it writes, is 507. A longer
/// blob does not fit the buffer, so it reads as no crumb (a cold boot), on both firmwares.
pub const BLOB_MAX: usize = 512;

/// Unix seconds at 2000-01-01T00:00:00Z: the stamp's epoch is MicroPython's, so the firmware subtracts this from
/// `gettimeofday` before it stamps a crumb or asks the gate.
pub const UNIX_SECONDS_AT_2000: i64 = 946_684_800;

/// The baro drivers whose setup re-zeroes the altitude, so a warm start rebases them to the pad (plan B
/// `_REBASED_BAROS`). Every one: the BMP581 (TMS-7F's backup elevation) was once missing, so after a mid-air reset
/// it read about 0 m aloft and an ICP outage handed the landing check the ground.
pub const REBASED_BAROS: [&str; 3] = ["icp10111", "bmp280", "bmp581"];

/// The rebased barometers whose own pad ground a crumb carries ([`Identity::grounds_m`]): one per driver in
/// [`REBASED_BAROS`].
pub const GROUND_SLOTS: usize = REBASED_BAROS.len();

/// A flight's warm starts after which the next recovers DISARMED (the port's own, with no plan B counterpart): the
/// law and the fins stay off, the stage machine still walks the flight to DONE and its beacon. A crash that returns
/// every boot (a fault in the law, the fins, or what they read) would otherwise reset the board over and over in the
/// air; a cold boot mid-air instead would never reach DONE, so nothing would beacon.
pub const WARM_CAP: u32 = 3;

/// The latitude limit in degrees, either side of the equator: a crumb coordinate past it reads as absent. f64, as
/// the JSON number it is checked against (the boundary decision D18 allows).
const LATITUDE_LIMIT: f64 = 90.0;

/// The longitude limit in degrees, either side of Greenwich (f64, as above).
const LONGITUDE_LIMIT: f64 = 180.0;

/// A failed NVS operation: the `esp_err_t` the firmware's NVS returned (0x1102 is ESP_ERR_NVS_NOT_FOUND).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NvsError(pub i32);

/// The NVS namespace as the warm start uses it: one i32 and one blob, read at boot and written by the checkpoint.
///
/// The firmware implements it over esp-idf's NVS handle; the tests over memory. A key that was never written is an
/// error, as esp-idf reports it.
pub trait Nvs {
    /// The i32 under `key`.
    fn get_i32(&mut self, key: &str) -> Result<i32, NvsError>;

    /// Read the blob under `key` into `buffer` and return its length. A blob longer than the buffer is an error
    /// (ESP_ERR_NVS_INVALID_LENGTH), as esp-idf reports it.
    fn get_blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, NvsError>;

    /// Store `value` as the blob under `key`.
    fn set_blob(&mut self, key: &str, value: &[u8]) -> Result<(), NvsError>;

    /// Store `value` as the i32 under `key`.
    fn set_i32(&mut self, key: &str, value: i32) -> Result<(), NvsError>;

    /// Make the writes durable.
    fn commit(&mut self) -> Result<(), NvsError>;
}

/// The recovery identity: what a flight launched with, frozen at BOOSTING entry and carried by every crumb after.
///
/// These do not change through a flight, so they are captured once (the launch fix pinned at the last ground
/// moment) rather than sampled per checkpoint: the pitot tare in particular is a calibration taken on the pad, and
/// re-reading it in flight would persist whatever a disturbed sensor happened to show.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Identity {
    /// The launch point, for the mission (`launch`).
    pub launch: Option<Position>,
    /// The landing zone's two corners, as the mission holds them (`zone`).
    pub zone: Option<[Position; 2]>,
    /// The pad altitude in metres, which the rebooted baros are rebased to (`pad_altitude`). Their setup re-zeroed
    /// mid-air, so without it the elevation reads about 0 aloft and the landing detector flares at once.
    pub pad_altitude_m: Option<f32>,
    /// The pitot tare in pascals (`pitot_zero`). The SDP810 keeps it in RAM, so a rebooted board would fly on
    /// dynamic pressure still holding the interior-static bias, and the fin governor caps off that airspeed.
    /// 0.0 is a real tare (a perfectly zeroed pitot), never "none": plan B once dropped it as falsy.
    pub pitot_zero_pa: Option<f32>,
    /// Each rebased barometer's own ground in metres at BOOSTING entry (`grounds`, the port's own), in the config
    /// order of [`REBASED_BAROS`] drivers; None for one that published none. A warm start puts each back, where plan
    /// B gave every one the pad altitude: a backup barometer carries its offset from the primary (5.3 m on TMS-7F),
    /// so its elevation read that much off after a warm start.
    pub grounds_m: [Option<f32>; GROUND_SLOTS],
    /// Each gyro's bias in degrees per second as the attitude filter learned it on the pad (`gyro_bias`, the port's
    /// own), by its rate-rank slot (control::attitude `BIAS_SLOTS`), rounded to 0.001 (0.1 degree over a 100 s
    /// flight, and four slots stay about 100 bytes of the crumb); None for a slot never learned. A warm start seeds
    /// the rebuilt filter with them, where plan B flew the raw gyro for the rest of the flight: a bias (up to 0.58
    /// deg/s measured) drifts the attitude through every gated turn.
    pub gyro_bias: [Option<[f32; 3]>; BIAS_SLOTS],
    /// The flight's warm starts so far (`warm_starts`, the port's own): [`WARM_CAP`] decides the arming.
    pub warm_starts: u32,
}

impl Identity {
    /// Nothing frozen yet (plan B's empty `_static`): the next BOOSTING entry captures afresh. The warm-start count
    /// is no frozen value: a flight that warm-started with nothing frozen still freezes at BOOSTING.
    pub fn is_empty(&self) -> bool {
        Identity {
            warm_starts: 0,
            ..*self
        } == Identity::default()
    }
}

/// The checkpoint crumb: the live flight state plus the recovery identity, as the NVS blob carries it.
///
/// Every field may be absent, as in plan B's dict: a torn or older crumb lacks some, and the gate and the apply
/// plan decide what an absent one means. A field of the wrong JSON type, or a coordinate off the globe, reads as
/// absent.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Crumb {
    /// The stage id (`stage`). [`load`] sets it from the NVS i32, which is authoritative over the blob's copy.
    pub stage: Option<i32>,
    /// When it was written, in whole seconds since 2000-01-01 (`stamp`).
    pub stamp_s: Option<i64>,
    /// Whether the flight was armed (`armed`); only an armed flight writes a crumb, and a warm start re-arms to
    /// match.
    pub armed: bool,
    /// The fused altitude in metres when written, absent when stale (`altitude`); recorded, never restored.
    pub altitude_m: Option<f32>,
    /// The GNSS ground speed in m/s when written, absent when stale (`speed`); recorded, never restored.
    pub speed_ms: Option<f32>,
    /// The fused airspeed in m/s, to 0.1 (`airspeed`). A warm start hands it back to the flight task so the fin cap
    /// comes off a real speed rather than the blunt unconfident floor.
    pub airspeed_ms: Option<f32>,
    /// The uptime in milliseconds when written (`ticks_ms`); recorded, never restored.
    pub ticks_ms: Option<u64>,
    /// The recovery identity frozen at BOOSTING (`launch`, `zone`, `pad_altitude`, `pitot_zero`).
    pub identity: Identity,
}

/// The crumb as the blob writes it: plan B's checkpoint key order, the identity first and only when present, the
/// live state always (null when unknown). f32 fields print as their shortest text (412.5, not 412.5000000), and a
/// NaN or infinity prints as null.
#[derive(Serialize)]
struct Written {
    /// `launch`, in decimal degrees: f64 because it is the JSON number itself, divided out of the exact units.
    #[serde(skip_serializing_if = "Option::is_none")]
    launch: Option<[f64; 2]>,
    /// `zone`, two corners in decimal degrees.
    #[serde(skip_serializing_if = "Option::is_none")]
    zone: Option<[[f64; 2]; 2]>,
    /// `pad_altitude`, metres.
    #[serde(skip_serializing_if = "Option::is_none")]
    pad_altitude: Option<f32>,
    /// `pitot_zero`, pascals.
    #[serde(skip_serializing_if = "Option::is_none")]
    pitot_zero: Option<f32>,
    /// `grounds`, metres: the rebased barometers' own grounds, up to the last one known (the port's own).
    #[serde(skip_serializing_if = "Option::is_none")]
    grounds: Option<Vec<Option<f32>>>,
    /// `gyro_bias`, degrees per second: each slot's [x, y, z], up to the last one known (the port's own).
    #[serde(skip_serializing_if = "Option::is_none")]
    gyro_bias: Option<Vec<Option<[f32; 3]>>>,
    /// `warm_starts`, the flight's warm starts so far (the port's own), written from the first.
    #[serde(skip_serializing_if = "is_zero")]
    warm_starts: u32,
    /// `stage`, the id.
    stage: Option<i32>,
    /// `armed`.
    armed: bool,
    /// `altitude`, metres.
    altitude: Option<f32>,
    /// `speed`, m/s.
    speed: Option<f32>,
    /// `airspeed`, m/s.
    airspeed: Option<f32>,
    /// `ticks_ms`.
    ticks_ms: Option<u64>,
    /// `stamp`, seconds since 2000.
    stamp: Option<i64>,
}

impl Crumb {
    /// The crumb a blob holds: None when the JSON is not an object (plan B raised TypeError there and crashed the
    /// boot). Every member is read on its own (see [`Crumb`]); an unknown member is ignored.
    pub fn from_json(value: &Value) -> Option<Crumb> {
        let members = value.as_object()?;
        let member = |key: &str| members.get(key).unwrap_or(&Value::Null);
        Some(Crumb {
            stage: member("stage").as_i64().and_then(|stage| i32::try_from(stage).ok()),
            stamp_s: member("stamp").as_i64(),
            armed: member("armed").as_bool() == Some(true),
            altitude_m: number(member("altitude")),
            speed_ms: number(member("speed")),
            airspeed_ms: number(member("airspeed")),
            ticks_ms: member("ticks_ms").as_u64(),
            identity: Identity {
                launch: position(member("launch")),
                zone: zone(member("zone")),
                pad_altitude_m: number(member("pad_altitude")),
                pitot_zero_pa: number(member("pitot_zero")),
                grounds_m: grounds(member("grounds")),
                gyro_bias: gyro_bias(member("gyro_bias")),
                warm_starts: member("warm_starts")
                    .as_u64()
                    .map_or(0, |count| u32::try_from(count).unwrap_or(u32::MAX)),
            },
        })
    }

    /// The blob text: compact JSON, well under [`BLOB_MAX`] bytes (about 370 for a full crumb).
    pub fn to_json(&self) -> String {
        let identity = &self.identity;
        let written = Written {
            launch: identity.launch.map(Position::degrees),
            zone: identity.zone.map(|[first, second]| [first.degrees(), second.degrees()]),
            pad_altitude: identity.pad_altitude_m,
            pitot_zero: identity.pitot_zero_pa,
            grounds: identity
                .grounds_m
                .iter()
                .rposition(Option::is_some)
                .map(|last| identity.grounds_m[..=last].to_vec()),
            gyro_bias: identity
                .gyro_bias
                .iter()
                .rposition(Option::is_some)
                .map(|last| identity.gyro_bias[..=last].to_vec()),
            warm_starts: identity.warm_starts,
            stage: self.stage,
            armed: self.armed,
            altitude: self.altitude_m,
            speed: self.speed_ms,
            airspeed: self.airspeed_ms,
            ticks_ms: self.ticks_ms,
            stamp: self.stamp_s,
        };
        serde_json::to_string(&written).expect("a crumb always serializes")
    }
}

/// Serde's skip test for a zero count.
fn is_zero(count: &u32) -> bool {
    *count == 0
}

/// The `grounds` list, each a number or null, up to [`GROUND_SLOTS`]; all absent for any other shape.
fn grounds(value: &Value) -> [Option<f32>; GROUND_SLOTS] {
    let mut grounds = [None; GROUND_SLOTS];
    if let Some(list) = value.as_array() {
        for (slot, item) in grounds.iter_mut().zip(list) {
            *slot = number(item);
        }
    }
    grounds
}

/// The `gyro_bias` list, each three numbers or null, up to [`BIAS_SLOTS`]; a slot of any other shape, or the whole
/// for a list of any other shape, absent.
fn gyro_bias(value: &Value) -> [Option<[f32; 3]>; BIAS_SLOTS] {
    let mut biases = [None; BIAS_SLOTS];
    if let Some(list) = value.as_array() {
        for (slot, item) in biases.iter_mut().zip(list) {
            *slot = match item.as_array().map(Vec::as_slice) {
                Some([x, y, z]) => number(x).zip(number(y)).zip(number(z)).map(|((x, y), z)| [x, y, z]),
                _ => None,
            };
        }
    }
    biases
}

/// A JSON number as f32, absent for anything else (or for a number past f32's range).
fn number(value: &Value) -> Option<f32> {
    value
        .as_f64()
        .map(|number| number as f32)
        .filter(|number| number.is_finite())
}

/// A `[latitude, longitude]` pair in decimal degrees as a Position, absent for any other shape or a point off the
/// globe.
fn position(value: &Value) -> Option<Position> {
    let [latitude, longitude] = value.as_array()?.as_slice() else {
        return None;
    };
    let (latitude, longitude) = (latitude.as_f64()?, longitude.as_f64()?);
    if latitude.abs() > LATITUDE_LIMIT || longitude.abs() > LONGITUDE_LIMIT {
        return None;
    }
    Some(Position::from_degrees(latitude, longitude))
}

/// A zone, `[[latitude, longitude], [latitude, longitude]]`, absent for any other shape.
fn zone(value: &Value) -> Option<[Position; 2]> {
    let [first, second] = value.as_array()?.as_slice() else {
        return None;
    };
    Some([position(first)?, position(second)?])
}

/// Commit the crumb to NVS: the blob, then the stage i32 LAST, then the commit (plan B `save`).
///
/// The i32 is the flag, written last, so a torn write leaves the previous stage, never a new stage pointing at a
/// half-written blob.
///
/// Returns true when committed; false, having written nothing, when the crumb has no stage, and false when NVS
/// refused a write (absent or full). Never panics: a failed checkpoint must never block the flight.
pub fn save(nvs: &mut impl Nvs, crumb: &Crumb) -> bool {
    crumb
        .stage
        .is_some_and(|stage| try_save(nvs, stage, &crumb.to_json()).is_ok())
}

/// [`save`]'s writes in its order: the blob, then the stage i32, then the commit; the first refusal is the error.
fn try_save(nvs: &mut impl Nvs, stage: i32, blob: &str) -> Result<(), NvsError> {
    nvs.set_blob(CRUMB_KEY, blob.as_bytes())?;
    nvs.set_i32(STAGE_KEY, stage)?;
    nvs.commit()
}

/// Reset the stage i32 to [`NO_STAGE`] (plan B `clear`), so the next boot is unambiguously cold: after a rejected
/// warm start, and when a flight ends ([`Keeper`]). The blob stays: the i32 alone decides, so the clear is one fast
/// write.
///
/// Returns: true when committed; false when NVS refused (the stage i32 may still hold the crumb's stage). Never
/// panics.
pub fn clear(nvs: &mut impl Nvs) -> bool {
    try_clear(nvs).is_ok()
}

/// [`clear`]'s writes: the stage i32, then the commit; the first refusal is the error.
fn try_clear(nvs: &mut impl Nvs) -> Result<(), NvsError> {
    nvs.set_i32(STAGE_KEY, NO_STAGE)?;
    nvs.commit()
}

/// The last checkpoint crumb (plan B `load`): the stage i32 gates it, then the blob fills it in.
///
/// Returns None when no flight was checkpointed (the stage unwritten or [`NO_STAGE`]) or the blob is missing,
/// longer than [`BLOB_MAX`], torn, not JSON or not a JSON object: a cold boot. The crumb's stage is the i32's.
pub fn load(nvs: &mut impl Nvs) -> Option<Crumb> {
    let stage = nvs.get_i32(STAGE_KEY).ok()?;
    if stage == NO_STAGE {
        return None;
    }
    let mut buffer = [0_u8; BLOB_MAX];
    let length = nvs.get_blob(CRUMB_KEY, &mut buffer).ok()?;
    let value: Value = serde_json::from_slice(buffer.get(..length)?).ok()?;
    let mut crumb = Crumb::from_json(&value)?;
    crumb.stage = Some(stage);
    Some(crumb)
}

/// Why the board reset, as ESP-IDF's `esp_reset_reason()` reports it (esp_system.h `esp_reset_reason_t`, IDF 5.5;
/// the discriminant is the code).
///
/// Plan B saw MicroPython's `machine.reset_cause()`, which the esp32 port derives from these
/// (ports/esp32/modmachine.c, v1.29.0): SW, PANIC and EXT are HARD_RESET; INT_WDT, TASK_WDT and WDT are WDT_RESET;
/// POWERON and BROWNOUT are PWRON_RESET; DEEPSLEEP is DEEPSLEEP_RESET; everything else is 0.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetReason {
    /// ESP_RST_UNKNOWN: the reason cannot be determined (also any code past the last below).
    Unknown = 0,
    /// ESP_RST_POWERON: power applied, or an EN/CHIP_PU pulse from DTR/RTS or esptool (the P4 reports no EXT).
    PowerOn = 1,
    /// ESP_RST_EXT: the external reset pin; the P4 never reports it.
    External = 2,
    /// ESP_RST_SW: `esp_restart()`, the firmware's own deliberate reset (and the control-stall restart).
    Software = 3,
    /// ESP_RST_PANIC: a panic or exception; the firmware aborts on panic, so a crash in flight lands here.
    Panic = 4,
    /// ESP_RST_INT_WDT: the interrupt watchdog.
    InterruptWatchdog = 5,
    /// ESP_RST_TASK_WDT: the task watchdog, which the supervisor reconfigures to panic.
    TaskWatchdog = 6,
    /// ESP_RST_WDT: another watchdog.
    Watchdog = 7,
    /// ESP_RST_DEEPSLEEP: waking from deep sleep.
    DeepSleep = 8,
    /// ESP_RST_BROWNOUT: the supply sagged below the brownout threshold.
    Brownout = 9,
    /// ESP_RST_SDIO: a reset over SDIO.
    Sdio = 10,
    /// ESP_RST_USB: the USB peripheral (on the P4, an esptool reset over the USB-Serial-JTAG).
    Usb = 11,
    /// ESP_RST_JTAG: a reset by JTAG.
    Jtag = 12,
    /// ESP_RST_EFUSE: an eFuse error.
    Efuse = 13,
    /// ESP_RST_PWR_GLITCH: a power glitch was detected.
    PowerGlitch = 14,
    /// ESP_RST_CPU_LOCKUP: the CPU locked up (a double exception): a crash.
    CpuLockup = 15,
}

impl ResetReason {
    /// Every reason, in code order.
    pub const ALL: [ResetReason; 16] = [
        ResetReason::Unknown,
        ResetReason::PowerOn,
        ResetReason::External,
        ResetReason::Software,
        ResetReason::Panic,
        ResetReason::InterruptWatchdog,
        ResetReason::TaskWatchdog,
        ResetReason::Watchdog,
        ResetReason::DeepSleep,
        ResetReason::Brownout,
        ResetReason::Sdio,
        ResetReason::Usb,
        ResetReason::Jtag,
        ResetReason::Efuse,
        ResetReason::PowerGlitch,
        ResetReason::CpuLockup,
    ];

    /// The reason for an `esp_reset_reason_t` code; [`ResetReason::Unknown`] for a code this list does not know.
    pub fn from_code(code: u32) -> ResetReason {
        usize::try_from(code)
            .ok()
            .and_then(|index| ResetReason::ALL.get(index).copied())
            .unwrap_or(ResetReason::Unknown)
    }

    /// The ESP-IDF name after `ESP_RST_`, as the golden fixture and the boot log write it.
    pub fn name(self) -> &'static str {
        match self {
            ResetReason::Unknown => "UNKNOWN",
            ResetReason::PowerOn => "POWERON",
            ResetReason::External => "EXT",
            ResetReason::Software => "SW",
            ResetReason::Panic => "PANIC",
            ResetReason::InterruptWatchdog => "INT_WDT",
            ResetReason::TaskWatchdog => "TASK_WDT",
            ResetReason::Watchdog => "WDT",
            ResetReason::DeepSleep => "DEEPSLEEP",
            ResetReason::Brownout => "BROWNOUT",
            ResetReason::Sdio => "SDIO",
            ResetReason::Usb => "USB",
            ResetReason::Jtag => "JTAG",
            ResetReason::Efuse => "EFUSE",
            ResetReason::PowerGlitch => "PWR_GLITCH",
            ResetReason::CpuLockup => "CPU_LOCKUP",
        }
    }

    /// The reason with this [`name`](ResetReason::name), None for any other text.
    pub fn from_name(name: &str) -> Option<ResetReason> {
        ResetReason::ALL.into_iter().find(|reason| reason.name() == name)
    }
}

/// Why the gate answered as it did: plan B's reason texts, plus the ones the boot rules add. Display gives the text
/// the boot log writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// No crumb: no flight was checkpointed. A cold boot in SETTING.
    NoCrumb,
    /// The crumb has no stage or no stamp: torn, a cold boot.
    Torn,
    /// The crumb's stage id is no flight stage (a corrupted i32): a cold boot. (New: plan B passed it.)
    NotAStage(i32),
    /// The crumb reads DONE: any power-up after DONE is a new session in SETTING. (New: the operator's rule.)
    AfterDone,
    /// The crumb reads SETTING: only BOOSTING..LANDING warm-starts, so the pad boot is cold. (New: the operator's
    /// rule.)
    OnThePad,
    /// A gliding stage, but the separation latch reads nested: this glider never left its booster.
    Nested,
    /// The gate passed: recover this stage, this many seconds after the checkpoint.
    Recover {
        /// The stage to restore.
        stage: Stage,
        /// The crumb's age in seconds, for the log only: after a power cycle the clock restarted, so it can read
        /// anything.
        age_s: i64,
    },
}

impl fmt::Display for Reason {
    /// Plan B's texts word for word where plan B has the case.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reason::NoCrumb => formatter.write_str("no crumb"),
            Reason::Torn => formatter.write_str("crumb missing stage/stamp -> cold boot"),
            Reason::NotAStage(stage) => write!(formatter, "crumb stage {stage} is no flight stage -> cold boot"),
            Reason::AfterDone => formatter.write_str("crumb reads done -> a new session in setting"),
            Reason::OnThePad => {
                formatter.write_str("crumb reads setting -> a cold boot (only boosting..landing warm-start)")
            }
            Reason::Nested => formatter.write_str("separation switch reads nested"),
            Reason::Recover { stage, age_s } => {
                write!(formatter, "recover {}, {age_s}s after checkpoint", stage.name())
            }
        }
    }
}

/// The flight stage a crumb's i32 names, None for an id that is no stage.
fn flight_stage(id: i32) -> Option<Stage> {
    u8::try_from(id).ok().and_then(Stage::from_id)
}

/// The unpowered glide after separation, GLIDING..LANDING (plan B `Stage.passive`): the boost stack is gone, so the
/// separation latch must read separated.
fn passive(stage: Stage) -> bool {
    matches!(stage, Stage::Gliding | Stage::Landing)
}

/// The warm-start gate (plan B `should_restore`, behind the operator's boot rules and Q19's step 2): does the crumb
/// name a flight stage to recover into?
///
/// The checkpoint keeps the crumb's STAGE current aloft and clears it on any edge out of flying, so a crumb is a
/// flight that was interrupted. Only physical evidence refuses it. In order, the first that fails is the reason:
///  * a crumb, with a stage and a stamp (a crumb without them is torn: its blob carries the identity a recovery
///    applies);
///  * the stage rules (§4.6): a flight stage, not DONE (a new session), not SETTING (a cold pad boot);
///  * GLIDING and LANDING need the separation latch SEPARATED: the physical latch no software can fake, so a
///    landed-then-nested glider never recovers into a glide. BOOSTING is the boost itself (latch nested).
///
/// No time or cause check (Q19's step 2, the operator 2026-10-13): with no battery-backed RTC a power cycle loses the
/// clock, so the crumb's age says nothing, and a battery bounce in flight is a power-on too. Plan B refused a
/// power-on and a crumb outside 0..600 s. `now_s` (seconds since 2000, like the stamp) only reports the age.
/// Returns (true, [`Reason::Recover`]) on a pass, else false with the first failing reason.
pub fn should_restore(crumb: Option<&Crumb>, separated: bool, now_s: i64) -> (bool, Reason) {
    let Some(crumb) = crumb else {
        return (false, Reason::NoCrumb);
    };
    let (Some(id), Some(stamp_s)) = (crumb.stage, crumb.stamp_s) else {
        return (false, Reason::Torn);
    };
    let stage = match flight_stage(id) {
        None => return (false, Reason::NotAStage(id)),
        Some(Stage::Done) => return (false, Reason::AfterDone),
        Some(Stage::Setting) => return (false, Reason::OnThePad),
        Some(stage) => stage,
    };
    let age_s = now_s.saturating_sub(stamp_s);
    if passive(stage) && !separated {
        return (false, Reason::Nested);
    }
    (true, Reason::Recover { stage, age_s })
}

/// The baros a warm start rebases, and the grounds it rebases them to.
#[derive(Clone, Debug, PartialEq)]
pub struct Rebase {
    /// The pad altitude in metres, a baro's new ground when the crumb carries none of its own (plan B
    /// `update({'ground': pad_altitude})`, for every baro).
    pub ground_m: f32,
    /// The config names of the sensors whose driver is in [`REBASED_BAROS`], in config order.
    pub baros: Vec<String>,
    /// Their own grounds, by the same order ([`Identity::grounds_m`]).
    pub grounds_m: [Option<f32>; GROUND_SLOTS],
}

impl Rebase {
    /// The ground of the `index`-th baro of [`Rebase::baros`]: its own, else the pad altitude.
    pub fn ground_of(&self, index: usize) -> f32 {
        self.grounds_m.get(index).copied().flatten().unwrap_or(self.ground_m)
    }
}

/// What a passed gate restores (plan B `_apply_restore`), as data. The firmware's `warm.rs` applies it before any
/// thread runs, in the field order, which is plan B's:
///  1. the mission's launch point and zone, BEFORE the arm: arming freezes the launch point from the live fix
///     unless one is set, and mid-air that fix is not the pad;
///  2. the baro rebase, the pitot tare, the gyro biases and the airspeed seed, before anything reads altitude,
///     airspeed or the gyro;
///  3. the checkpoint's identity, else the first checkpoint after the restore saves a crumb with none, and a second
///     reset skips the rebase and the tare (and a warm start into BOOSTING would freeze the mid-air altitude as the
///     pad);
///  4. the stage, the arm (only if the crumb was armed), and the warm-started mark (degraded `WARM-STARTED`).
#[derive(Clone, Debug, PartialEq)]
pub struct ApplyPlan {
    /// The launch point and zone for the mission, when the crumb carries both (frozen at BOOSTING).
    pub mission: Option<(Position, [Position; 2])>,
    /// The baro rebase, when the crumb carries a pad altitude.
    pub rebase: Option<Rebase>,
    /// The pitot tare in pascals for the SDP810 (plan B finds it as `airspeed_sdp810`), when the crumb carries one;
    /// 0.0 included. Absent leaves the driver's own tare alone, never zeroes it.
    pub pitot_zero_pa: Option<f32>,
    /// The gyro biases to seed the attitude filter with ([`Identity::gyro_bias`]), before the control thread builds
    /// it; all None leaves it to fly the raw gyro.
    pub gyro_bias: [Option<[f32; 3]>; BIAS_SLOTS],
    /// The fused airspeed in m/s to seed the flight task with, when the crumb carries one. Recovery order is pitot,
    /// then this, then GNSS: the accel backbone integrates on from it, and the first in-band pitot read overrides it.
    pub airspeed_ms: Option<f32>,
    /// The identity to seed the checkpoint with (plan B `Checkpoint.seed`).
    pub identity: Identity,
    /// The SAVED stage; the detectors re-evaluate from it.
    pub stage: Stage,
    /// Re-arm: only an armed flight ever checkpoints, so this matches what was flying -- unless the flight has warm
    /// started [`WARM_CAP`] times already ([`ApplyPlan::capped`]).
    pub armed: bool,
    /// The crumb was armed, but past [`WARM_CAP`] warm starts this one recovers disarmed.
    pub capped: bool,
}

impl ApplyPlan {
    /// The plan for a crumb whose stage is a flight stage; None otherwise (the gate never passes one).
    ///
    /// `sensors` are the board config's sensors as (name, driver) pairs, in config order; those whose driver is in
    /// [`REBASED_BAROS`] are rebased.
    pub fn new<'a>(crumb: &Crumb, sensors: impl IntoIterator<Item = (&'a str, &'a str)>) -> Option<ApplyPlan> {
        let stage = flight_stage(crumb.stage?)?;
        // this warm start counts, in the identity the next crumbs carry
        let identity = Identity {
            warm_starts: crumb.identity.warm_starts.saturating_add(1),
            ..crumb.identity
        };
        let capped = crumb.armed && crumb.identity.warm_starts >= WARM_CAP;
        Some(ApplyPlan {
            mission: identity.launch.zip(identity.zone),
            rebase: identity.pad_altitude_m.map(|ground_m| Rebase {
                ground_m,
                baros: sensors
                    .into_iter()
                    .filter(|(_name, driver)| REBASED_BAROS.contains(driver))
                    .map(|(name, _driver)| name.to_owned())
                    .collect(),
                grounds_m: identity.grounds_m,
            }),
            pitot_zero_pa: identity.pitot_zero_pa,
            gyro_bias: identity.gyro_bias,
            airspeed_ms: crumb.airspeed_ms,
            identity,
            stage,
            armed: crumb.armed && !capped,
            capped,
        })
    }

    /// The mission update as plan B sends it, `{latitude, longitude, zone}` in decimal degrees, for the mission's
    /// update path; None when the crumb carried no launch point and zone.
    pub fn mission_update(&self) -> Option<Map<String, Value>> {
        let (launch, [first, second]) = self.mission?;
        let [latitude, longitude] = launch.degrees();
        let update = json!({"latitude": latitude, "longitude": longitude, "zone": [first.degrees(), second.degrees()]});
        update.as_object().cloned()
    }
}

/// The boot decision (plan B `restore`).
#[derive(Clone, Debug, PartialEq)]
pub enum Restore {
    /// No crumb: the normal cold boot, nothing written.
    Cold,
    /// A crumb, but the checkpoint component's `warm_start` is off: a cold boot; the stage cleared, unless NVS
    /// refused ([`restore`]).
    Disabled,
    /// The gate refused: a cold boot; the stage cleared so the next boot is cold, unless NVS refused ([`restore`]).
    Refused(Reason),
    /// The gate passed: apply the plan before any thread runs.
    Warm {
        /// The passing reason ([`Reason::Recover`]), for the boot log.
        reason: Reason,
        /// What to restore, boxed (the decision is made once, at boot).
        plan: Box<ApplyPlan>,
    },
}

impl Restore {
    /// Whether the recorder's flight flag stays up: the recorder opens with it raised for a crumb [`load`] reads in
    /// BOOSTING..LANDING, and only a warm start keeps it. Cold, Disabled and Refused lower it, Cold included: a
    /// stage i32 whose blob does not read is Cold with nothing cleared, so a flag kept there would stay up all
    /// session, and on every later boot.
    pub fn keeps_flight(&self) -> bool {
        matches!(self, Restore::Warm { .. })
    }

    /// Whether this boot restarts the board for no control stall: a warm start after `reset`, this boot's reason, a
    /// software one. Off the pad only the supervisor's stall restart resets the board in software (`reboot` runs only
    /// in SETTING, and a failing Wi-Fi link parks rather than restarting), so such a boot is a stall restart's own warm
    /// start, and a stall that comes back every boot must leave the board stalled rather than cycle it through warm
    /// starts. A warm start after any other reset (a panic, a watchdog), and every boot that is not warm, restart for a
    /// stall as usual.
    pub fn stall_restart_suppressed(&self, reset: ResetReason) -> bool {
        matches!(self, Restore::Warm { .. }) && reset == ResetReason::Software
    }
}

/// Decide the boot from NVS (plan B `restore`, without its logging and GC handling): load the crumb, honour the
/// config toggle, ask the gate, and clear the stage on a refusal.
///
/// A refused clear is ignored, as plan B swallowed it, so the stage i32 keeps the crumb for the next boot. A
/// refusal that lasts (DONE, SETTING, torn, no flight stage) refuses it there again; one that can change by then
/// (the toggle off, a nested latch) can pass. The [`Keeper`] clears only a crumb it saved or was seeded with, so a
/// later boot can warm-start a landed board. Plan B has the same gap.
///
/// `warm_start` is the checkpoint component's toggle (default on), `separated` the separation latch read at
/// setup, `now_s` the clock in seconds since 2000 (it reports the crumb's age), and `sensors` the board config's
/// (name, driver) pairs for the baro rebase.
pub fn restore<'a>(
    nvs: &mut impl Nvs,
    warm_start: bool,
    separated: bool,
    now_s: i64,
    sensors: impl IntoIterator<Item = (&'a str, &'a str)>,
) -> Restore {
    let Some(crumb) = load(nvs) else {
        return Restore::Cold;
    };
    if !warm_start {
        clear(nvs);
        return Restore::Disabled;
    }
    let (allowed, reason) = should_restore(Some(&crumb), separated, now_s);
    match ApplyPlan::new(&crumb, sensors).filter(|_plan| allowed) {
        Some(plan) => Restore::Warm {
            reason,
            plan: Box::new(plan),
        },
        None => {
            clear(nvs);
            Restore::Refused(reason)
        }
    }
}

/// What one pass of the checkpoint schedule asks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Step {
    /// The recovery identity was frozen on this pass (BOOSTING entry with none held).
    pub froze: bool,
    /// Write a checkpoint now: the telemetry row always, and the NVS crumb ([`Checkpoint::crumb`]) when flying
    /// ([`Keeper`]).
    pub checkpoint: bool,
}

/// What [`Checkpoint::step`] freezes at BOOSTING entry, read by the caller when asked (plan B `_freeze_static`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Freeze {
    /// The mission's zone, None when it has none (then no launch or zone is frozen at all).
    pub zone: Option<[Position; 2]>,
    /// The mission's launch point after `freeze_launch` (a fix that arrived after arm pins here, the last ground
    /// moment); None without a fix or a CC point, when the zone's centre stands in.
    pub launch_point: Option<Position>,
    /// A fresh altitude reading in metres, used only when no pad altitude was latched in SETTING.
    pub fresh_altitude_m: Option<f32>,
    /// The pitot's tare in pascals, None without a pitot.
    pub pitot_zero_pa: Option<f32>,
    /// Each rebased barometer's own ground in metres, in the config order of [`REBASED_BAROS`] drivers; None for
    /// one with no fresh altitude and elevation.
    pub grounds_m: [Option<f32>; GROUND_SLOTS],
    /// The attitude filter's gyro biases, degrees per second, by its slots; None for one never learned.
    pub gyro_bias: [Option<[f32; 3]>; BIAS_SLOTS],
}

/// The live values a checkpoint records (plan B `_checkpoint`); each absent when its source is stale.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Live {
    /// The fused altitude in metres, from a FRESH reading only (an extrapolated one would lie in checkpoint.csv
    /// about how high the flight was when it reset, the record a post-mortem trusts).
    pub altitude_m: Option<f32>,
    /// The GNSS ground speed in m/s, fresh only.
    pub speed_ms: Option<f32>,
    /// The flight task's fused airspeed in m/s, None without a flight task; rounded to 0.1 in the crumb.
    pub airspeed_ms: Option<f32>,
    /// The uptime in milliseconds.
    pub ticks_ms: u64,
}

/// The checkpoint schedule (plan B's Checkpoint task): a crumb on every stage change and every period while
/// airborne, the recovery identity frozen at BOOSTING entry, the pad altitude latched in SETTING.
///
/// SETTING and DONE checkpoint once on entry and never periodically: nothing moves during the long pad dwell or the
/// post-landing wait, so re-writing every period would only wear the flash. The [`Keeper`] decides which
/// checkpoints also save the NVS crumb (only a flying board's); the telemetry row goes out on every one.
#[derive(Clone, Debug, PartialEq)]
pub struct Checkpoint {
    /// The airborne cadence in microseconds (`checkpoint_s`, floored at 1 s).
    period_us: u64,
    /// How often the owner polls the stage: fast enough to catch a change promptly, at most 500 ms.
    poll_ms: u64,
    /// The recovery identity: empty until BOOSTING entry or a seed.
    identity: Identity,
    /// The pad altitude latched while in SETTING, from fresh readings only.
    pad_m: Option<f32>,
    /// The stage of the last checkpoint, None before the first.
    last_stage: Option<Stage>,
    /// When the last checkpoint was written (or the schedule started), in microseconds.
    last_us: u64,
}

impl Checkpoint {
    /// The schedule for a `checkpoint_s` cadence (the component's config, default 1), started at `now_us`.
    ///
    /// Any number is a cadence, as plan B takes any: below 1 s (or NaN) is 1 s, and a cadence too long for the
    /// microsecond clock (past ~1.8e13 s, or infinite) saturates there, which never falls due, so the schedule
    /// only checkpoints on a stage change. Nothing validates `checkpoint_s`, and an overflow here would panic at
    /// bring-up and reboot the board into the same boot path.
    pub fn new(checkpoint_s: f32, now_us: u64) -> Checkpoint {
        let period_ms = (checkpoint_s.max(1.0) * 1000.0) as u64; // `as` saturates: infinity is u64::MAX
        Checkpoint {
            period_us: period_ms.saturating_mul(1000),
            poll_ms: period_ms.min(500),
            identity: Identity::default(),
            pad_m: None,
            last_stage: None,
            last_us: now_us,
        }
    }

    /// The polling period in milliseconds the owner sleeps between [`step`](Checkpoint::step)s.
    pub fn poll_ms(&self) -> u64 {
        self.poll_ms
    }

    /// The recovery identity the crumbs carry now.
    pub fn identity(&self) -> &Identity {
        &self.identity
    }

    /// Carry a restored crumb's identity forward (the warm start's [`ApplyPlan::identity`]). The schedule starts
    /// empty, which only BOOSTING entry fills, so without this the first checkpoint after a warm start went out
    /// with no identity.
    pub fn seed(&mut self, identity: Identity) {
        self.identity = identity;
    }

    /// One poll (plan B's `run` loop body): the stage now, the time, a FRESH altitude reading or None, and the
    /// identity inputs, read only when this pass freezes.
    ///
    /// In SETTING a fresh altitude becomes the pad and the identity is dropped (the next BOOSTING is a new flight).
    /// Entering BOOSTING with no identity freezes one, never over a seeded one: a warm start into BOOSTING would
    /// otherwise freeze the mid-air altitude as the pad. A checkpoint is due on a stage change, and every period
    /// while airborne.
    pub fn step(
        &mut self,
        stage: Stage,
        now_us: u64,
        fresh_altitude_m: Option<f32>,
        freeze: impl FnOnce() -> Freeze,
    ) -> Step {
        if stage == Stage::Setting {
            if fresh_altitude_m.is_some() {
                self.pad_m = fresh_altitude_m;
            }
            self.identity = Identity::default();
        }
        let changed = self.last_stage != Some(stage);
        let froze = changed && stage == Stage::Boosting && self.identity.is_empty();
        if froze {
            self.freeze(freeze());
        }
        let airborne = matches!(stage, Stage::Boosting | Stage::Gliding | Stage::Landing);
        let due = airborne && now_us.saturating_sub(self.last_us) >= self.period_us;
        let checkpoint = changed || due;
        if checkpoint {
            self.last_stage = Some(stage);
            self.last_us = now_us;
        }
        Step { froze, checkpoint }
    }

    /// Freeze the recovery identity (plan B `_freeze_static`): the launch and zone when the mission has a zone (the
    /// zone's centre standing in for a missing launch point, which keeps the crumb usable), the latched pad or else
    /// a fresh reading, the pitot tare (0.0 included), each barometer's ground, and the gyro biases rounded to
    /// 0.001 deg/s.
    fn freeze(&mut self, inputs: Freeze) {
        let mut identity = Identity::default();
        if let Some([first, second]) = inputs.zone {
            identity.zone = Some([first, second]);
            // the integer mean, rounded toward zero, as the mission's zone centre is (control::navigation::zone);
            // plan B averages the float degrees, at most half a unit (5.6 mm) away (see the crate notes)
            identity.launch = Some(inputs.launch_point.unwrap_or(Position {
                latitude_e7: first.latitude_e7.midpoint(second.latitude_e7),
                longitude_e7: first.longitude_e7.midpoint(second.longitude_e7),
            }));
        }
        identity.pad_altitude_m = self.pad_m.or(inputs.fresh_altitude_m);
        identity.pitot_zero_pa = inputs.pitot_zero_pa;
        identity.grounds_m = inputs.grounds_m;
        identity.gyro_bias = inputs
            .gyro_bias
            .map(|bias| bias.map(|axes| axes.map(|axis| (axis * 1000.0).round_ties_even() / 1000.0)));
        self.identity = identity;
    }

    /// The crumb to save for a checkpoint at `stage` (only an armed flight saves one): the identity, then the live
    /// state, the airspeed rounded to 0.1 m/s, stamped `stamp_s` (seconds since 2000).
    pub fn crumb(&self, stage: Stage, live: Live, stamp_s: i64) -> Crumb {
        Crumb {
            stage: Some(i32::from(stage as u8)),
            stamp_s: Some(stamp_s),
            armed: true,
            altitude_m: live.altitude_m,
            speed_ms: live.speed_ms,
            airspeed_ms: live
                .airspeed_ms
                .map(|airspeed| (airspeed * 10.0).round_ties_even() / 10.0),
            ticks_ms: Some(live.ticks_ms),
            identity: self.identity,
        }
    }
}

/// An NVS action of the [`Keeper`]: the one it owes, and the one a pass committed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Write {
    /// The flight's crumb, built from the pass that writes it, in [`save`]'s order.
    Crumb,
    /// The stage i32 back to [`NO_STAGE`] ([`clear`]): the flight ended, so no reset may resume it.
    Clear,
}

/// What one [`Keeper::pass`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pass {
    /// The recovery identity was frozen on this pass (BOOSTING entry with none held).
    pub froze: bool,
    /// A checkpoint is due at this stage, flying or not: the caller pushes its checkpoint.csv row and log line. An
    /// owed crumb went to NVS first, so a full telemetry ring never costs the crumb.
    pub row: Option<Stage>,
    /// The NVS action this pass committed.
    pub wrote: Option<Write>,
    /// Why NVS refused this pass's action; it stays owed ([`Keeper::owed`]), and the next pass tries it again.
    pub failed: Option<NvsError>,
}

/// The [`Keeper`]'s NVS tallies since boot, for `inspect checkpoint`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// Crumbs committed.
    pub saves: u32,
    /// Actions NVS refused, crumbs and clears alike.
    pub failures: u32,
    /// Attempts at an action an earlier pass's refusal left owed.
    pub retries: u32,
}

/// The checkpoint schedule over NVS: the supervisor runs one [`pass`](Keeper::pass) every
/// [`poll_ms`](Keeper::poll_ms).
///
/// The crumb is keyed on FLYING, `armed && launched` of one flight word load, so the stage and the arming a crumb
/// carries are of one moment:
///  * a checkpoint due while flying owes a crumb. Every hold clears `launched`, so an armed bench hold (`arm`, then
///    `stage gliding`) owes none: keyed on the arming alone, a reset would resume it as a launched flight;
///  * leaving flying (a disarm, or any hold) while a crumb may be in NVS owes a clear, which replaces an owed crumb.
///    The recovery from a false launch (disarm, `stage done`, the re-zero, `stage setting`) would otherwise leave an
///    armed BOOSTING crumb, and the next boot would warm-start the pad board armed;
///  * an owed action is tried on every pass until NVS commits it, an owed crumb rebuilt from the pass that tries
///    it. The schedule counts a checkpoint done whether its crumb landed or not and never re-checkpoints DONE, so
///    without the retry a refused DONE crumb leaves the last airborne one, and a reset warm-starts a landed glider.
///
/// A crumb may be in NVS from a [`seed`](Keeper::seed) (the warm start's restored crumb) or from any crumb attempt,
/// a refused one included (its stage i32 may have landed before the commit failed), until a clear commits.
///
/// The pad latch has a ceiling: the schedule latches a fresh altitude as the pad only while the fresh elevation is
/// under it. A launch the accel trigger missed stays in SETTING through the baro launch dwell, where plan B latched
/// the pad up the climb; the ceiling bounds that error, and a drifted pad's, to the ceiling itself.
#[derive(Clone, Debug, PartialEq)]
pub struct Keeper {
    /// When to checkpoint, and the identity a crumb carries.
    checkpoint: Checkpoint,
    /// The pad latch ceiling in metres of elevation; None latches every fresh altitude, as plan B.
    pad_ceiling_m: Option<f32>,
    /// A crumb may be in NVS: seeded, or a crumb attempted since the last committed clear.
    crumb_live: bool,
    /// The action NVS has not committed yet.
    owed: Option<Write>,
    /// The tallies since boot.
    counters: Counters,
}

impl Keeper {
    /// A keeper for a `checkpoint_s` cadence ([`Checkpoint::new`]) started at `now_us`, its pad latch under
    /// `pad_ceiling_m` metres of elevation: half the sequencer's `launch_alt_m`, or None without a sequencer, which
    /// latches as plan B does.
    pub fn new(checkpoint_s: f32, now_us: u64, pad_ceiling_m: Option<f32>) -> Keeper {
        Keeper {
            checkpoint: Checkpoint::new(checkpoint_s, now_us),
            pad_ceiling_m,
            crumb_live: false,
            owed: None,
            counters: Counters::default(),
        }
    }

    /// The polling period in milliseconds between [`pass`](Keeper::pass)es ([`Checkpoint::poll_ms`]).
    pub fn poll_ms(&self) -> u64 {
        self.checkpoint.poll_ms()
    }

    /// The action NVS has not committed yet, None when nothing is owed.
    pub fn owed(&self) -> Option<Write> {
        self.owed
    }

    /// The NVS tallies since boot.
    pub fn counters(&self) -> Counters {
        self.counters
    }

    /// The recovery identity the crumbs carry now ([`Checkpoint::identity`]).
    pub fn identity(&self) -> &Identity {
        self.checkpoint.identity()
    }

    /// Carry a warm start's restored identity forward ([`Checkpoint::seed`]) and mark its crumb live, so the edge out
    /// of the restored flight clears it.
    pub fn seed(&mut self, identity: Identity) {
        self.checkpoint.seed(identity);
        self.crumb_live = true;
    }

    /// One poll: the schedule's step, then the owed NVS action, if any (the rules are the [`Keeper`]'s).
    ///
    /// Args: `nvs` the warm start's namespace; `flight` one load of the flight word; `now_us` the uptime in
    /// microseconds; `fresh_elevation_m` the elevation above the zeroed pad, None when stale, which the pad latch
    /// needs under its ceiling (with a ceiling, a stale one latches nothing); `live` this checkpoint's fresh
    /// readings, whose altitude is also what SETTING latches as the pad; `freeze` the identity inputs, read only on
    /// the pass that freezes; `stamp_s` the clock in seconds since 2000, read only on a pass that tries a crumb.
    ///
    /// Returns: the freeze, the row due, and the NVS action committed or refused.
    #[expect(
        clippy::too_many_arguments,
        reason = "one poll's inputs: the store, the flight, the clock, two readings and two lazy sources"
    )]
    pub fn pass(
        &mut self,
        nvs: &mut impl Nvs,
        flight: Flight,
        now_us: u64,
        fresh_elevation_m: Option<f32>,
        live: Live,
        freeze: impl FnOnce() -> Freeze,
        stamp_s: impl FnOnce() -> i64,
    ) -> Pass {
        let under_ceiling = self
            .pad_ceiling_m
            .is_none_or(|ceiling| fresh_elevation_m.is_some_and(|elevation| elevation < ceiling));
        let latch_m = live.altitude_m.filter(|_altitude| under_ceiling);
        let step = self.checkpoint.step(flight.stage, now_us, latch_m, freeze);
        let earlier = self.owed;
        self.owed = if flight.armed && flight.launched {
            (step.checkpoint || earlier.is_some()).then_some(Write::Crumb)
        } else {
            self.crumb_live.then_some(Write::Clear)
        };
        let mut pass = Pass {
            froze: step.froze,
            row: step.checkpoint.then_some(flight.stage),
            wrote: None,
            failed: None,
        };
        let Some(action) = self.owed else {
            return pass;
        };
        if earlier == Some(action) {
            self.counters.retries = self.counters.retries.saturating_add(1);
        }
        let attempt = match action {
            Write::Crumb => {
                self.crumb_live = true;
                let crumb = self.checkpoint.crumb(flight.stage, live, stamp_s());
                try_save(nvs, i32::from(flight.stage as u8), &crumb.to_json())
            }
            Write::Clear => try_clear(nvs),
        };
        match attempt {
            Ok(()) => {
                match action {
                    Write::Crumb => self.counters.saves = self.counters.saves.saturating_add(1),
                    Write::Clear => self.crumb_live = false,
                }
                self.owed = None;
                pass.wrote = Some(action);
            }
            Err(error) => {
                self.counters.failures = self.counters.failures.saturating_add(1);
                pass.failed = Some(error);
            }
        }
        pass
    }
}

#[cfg(test)]
mod tests;
