//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The SDP810-500Pa differential-pressure sensor (Sensirion SDP8xx, thermal flow-through): the pitot/static
//! AIRSPEED channel (src/glider/drivers/sdp810.py). Command-based, not register-mapped: [`Device::setup`] clears any
//! prior continuous mode, starts continuous measurement (differential pressure, temperature-compensated,
//! average-till-read) and validates one CRC-checked frame; [`Sdp810::read`] reads the 9-byte frame each period,
//! scales the tared dynamic pressure and derives the airspeed once, for the channel and the telemetry row alike.
//!
//! Bench-verified: 0x25 on i2c:0 (SDA 7 / SCL 8), scale factor 60 (Pa = raw / 60), zero ~0.2 Pa. Tube polarity
//! (blow-verified 2026-08-14, and anchored to a PIN because left/right got it wrong once): P+ = the barb OPPOSITE
//! the '1'/SCL mark -> pitot (total); P- = the '1'/SCL-side barb -> interior static. The interior-static PRESSURE
//! bias is tared out by the zero offset (a pad tare, `update {"zero": true}`); the position-span error folds into
//! `air_density`, the single q -> v knob (a GNSS-vs-q calm pass trims it).
//!
//! What plan B learnt the hard way, and this port keeps:
//!  * STOP BEFORE START. A prior unclean reboot can leave the part mid-continuous, where a re-issued start is
//!    rejected; the datasheet requires a stop first. Setup and the restart both send stop, wait, start.
//!  * ALL THREE CRCs ([`frame_ok`]), because the scale word is latched at setup and divides every reading after it:
//!    one corrupted scale would mis-scale the airspeed for the whole flight. The ICP-10111 on the same bus was
//!    measured dropping a bit in 0.50 % of its frames.
//!  * THE SCALE LATCH: setup reads the part's own scale factor (60 for the -500Pa part, 240 for the -125Pa) and every
//!    reading divides by it; a part answering 0 keeps the -500Pa default.
//!  * THE RESTART LADDER: the part holds its MODE in volatile state, so anything that resets it (the ICP's
//!    general-call reset, by design) drops it to idle and every read after fails. Five consecutive read failures
//!    re-issue stop-then-start, once per run of failures. A CRC reject neither counts toward it nor rearms it.
//!  * THE TARE LADDER: the config's `zero_offset_pa` is a guess; a tare stored by an earlier session (NVS
//!    `sdp810_zero`, an i32 of Pa x 100, the key plan B shares) wins over it at boot; an operator tare captures the
//!    live reading AND is persisted; a direct offset (the warm start restoring its crumb, mid-air) is applied but
//!    never written to flash. A tare before the first frame is refused out loud, and 0 Pa is a real tare.
//!
//! Units: f32 Pa and m/s for the channels (decision D18). The integer units survive only at the telemetry and NVS
//! boundary, as plan B defines them: dynamic pressure in Pa x 100 (`raw x 100 // scale - zero`, computed from the
//! raw count in integers so the column is plan B's to the digit), airspeed in cm/s (truncated), temperature in
//! degrees C x 100 (`raw // 2`). The zero offset itself is kept in that NVS unit, so a tare reads back after a reboot
//! as exactly what was captured.
//!
//! Where the port departs from plan B, on purpose:
//!  * AN AIR DENSITY THAT IS NOT ABOVE ZERO IS REFUSED (an update) or replaced by [`AIR_DENSITY`] (the config).
//!    Plan B took any number: at 0 every positive q raised ZeroDivisionError in its read path, so no sample went
//!    out; below 0 it took a complex root. In f32 those become an infinite or NaN airspeed handed straight to the
//!    governor's estimator, which is worse than no sample, so neither is allowed in.
//!  * A Pa FIGURE MEANT AS A WHOLE HUNDREDTH IS THAT HUNDREDTH (`HUNDREDTH_SNAP`). Plan B truncated `Pa x 100`,
//!    binary noise included: 0.29 Pa became 28 (CPython) and on the single-precision board 1.18 became 117. The
//!    warm start carries the tare as f32 Pa and hands it back here, so truncating would lose a hundredth on about
//!    half of all tares (333, the plan's own anchor, came back 332); snapping makes that round trip exact.
//!
//! Not here (firmware glue): the NVS read and write themselves, the channel push, the probe's freshness wait, the
//! log lines and the inspect JSON.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::commons::sensirion_crc8;
use crate::device::{Device, Fault, Strikes};

/// The fixed I2C address (SDP800/810-500Pa and -125Pa; the -x1 variants answer at 0x26).
pub const ADDRESS: u8 = 0x25;

/// Start continuous measurement: differential pressure, temperature-compensated, average-till-read.
pub const COMMAND_START: [u8; 2] = [0x36, 0x15];

/// Stop continuous measurement, back to idle: required before re-starting after an unclean reboot.
pub const COMMAND_STOP: [u8; 2] = [0x3F, 0xF9];

/// The frame: differential pressure [0, 1] + CRC, temperature [3, 4] + CRC, scale factor [6, 7] + CRC.
pub const FRAME_LENGTH: usize = 9;

/// The wait after a start before the first frame is read: the first continuous result lands ~8 ms after the
/// start, with margin.
pub const FIRST_RESULT_MS: u32 = 20;

/// The wait after a stop before the part accepts the next command: it settles in ~0.5 ms, with margin.
pub const STOP_SETTLE_MS: u32 = 3;

/// Consecutive read failures before the restart re-issues stop-then-start.
pub const RESTART_AFTER: u32 = 5;

/// Start attempts at setup: a stop and a restart clear a part left mid-continuous.
pub const START_TRIES: u32 = 2;

/// The scale factor of the -500Pa part (Pa = raw / scale), kept when the part answers 0; the -125Pa reports 240.
pub const DEFAULT_SCALE: u16 = 60;

/// The default air density, kg/m^3: ~25 degrees C at sea level (Florida, not ISA's 15 degrees C). The calm-pass
/// trim overrides it per airframe.
pub const AIR_DENSITY: f32 = 1.18;

/// The default read period, ms: ~50 Hz (the part's tau63 under 3 ms allows a fast poll).
pub const DEFAULT_PERIOD_MS: u32 = 20;

/// The NVS key (namespace `coludo`) of the stored tare: an i32 of Pa x 100, shared with plan B.
pub const NVS_ZERO_KEY: &str = "sdp810_zero";

/// The raw temperature word per degree C (temperature = raw / 200).
const TEMPERATURE_LSB: f32 = 200.0;

/// The telemetry and NVS unit: hundredths of the SI unit (Pa x 100, cm/s, degrees C x 100).
const HUNDREDTHS: i32 = 100;

/// What [`Sdp810::calibrate`] answers before the first frame (plan B's calibrate() text).
pub const CALIBRATE_NO_READING: &str = "no reading yet";

/// Whether a 9-byte frame is whole and every word's CRC is right: differential pressure, temperature AND scale
/// (sdp810._frame_ok).
///
/// It once checked only the pressure word, as "the flight-relevant field". The scale word is more flight-relevant
/// than that: setup latches it and every reading afterwards divides by it, so one corrupted scale mis-scales the
/// airspeed for the rest of the flight. A 1-in-200 chance of that is not worth two skipped checks over six bytes.
pub fn frame_ok(frame: &[u8]) -> bool {
    frame.len() == FRAME_LENGTH
        && sensirion_crc8(frame[0], frame[1]) == frame[2]
        && sensirion_crc8(frame[3], frame[4]) == frame[5]
        && sensirion_crc8(frame[6], frame[7]) == frame[8]
}

/// A big-endian signed 16-bit word from its two bytes: the part's two's-complement fields (sdp810._signed16).
pub fn signed16(high: u8, low: u8) -> i16 {
    i16::from_be_bytes([high, low])
}

/// The telemetry's `airspeed_cms` of an airspeed in m/s: truncated toward zero, as plan B's `int(airspeed x 100)`.
pub fn airspeed_cms(airspeed: f32) -> i32 {
    (airspeed * HUNDREDTHS as f32) as i32
}

/// How far from a whole hundredth a Pa figure x 100 may land and still be read as that hundredth: one hundredth
/// of a hundredth (0.0001 Pa).
///
/// Wide enough for the binary noise of an f32 across the part's whole span: a figure of up to +-546 Pa carries at
/// most ~0.003 hundredths of its own rounding, and the x 100 product adds ~0.002 more, so a figure meant as a whole
/// hundredth (an operator's 1.18, a crumb's 3.33) always lands inside it. Measured over every tare of that span
/// (+-54613 hundredths, through `zero_offset_pa()` and back): at most 0.0039 off, where a window of 0.001 would
/// have missed 6772 tares from 256 Pa up. Narrow enough to leave alone anything written finer than the part
/// resolves: -1.259 Pa is -125.9 hundredths and still truncates to -125, as plan B's.
const HUNDREDTH_SNAP: f32 = 0.01;

/// Pa x 100 from a Pa figure: the NVS unit, truncated toward zero as plan B's `fixed.from_float`, except that a
/// figure within [`HUNDREDTH_SNAP`] of a whole hundredth is that hundredth (see the module notes for why).
///
/// In f32 (D18): 1.18 x 100 is 117.99999 there, which plan B's truncation would store as 117 (as the
/// single-precision board did); the snap reads it as the 118 that was meant, and the warm start's
/// `zero_offset_pa()` -> crumb -> [`Sdp810::set_zero_offset`] round trip returns every tare unchanged.
fn hundredths(pascal: f32) -> i32 {
    let scaled = pascal * HUNDREDTHS as f32;
    let nearest = scaled.round();
    let whole = if (scaled - nearest).abs() < HUNDREDTH_SNAP {
        nearest
    } else {
        scaled.trunc()
    };
    whole as i32
}

/// Whether an air density can divide a dynamic pressure: finite and above 0 kg/m^3 (see the module notes).
fn density_ok(density: f32) -> bool {
    density.is_finite() && density > 0.0
}

/// The SDP810's settings, from its config block (the keys plan B's setup() reads, with its defaults).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// The I2C address (`addr`, default [`ADDRESS`]).
    pub address: u8,
    /// The read period, ms (`period_ms`, default [`DEFAULT_PERIOD_MS`]).
    pub period_ms: u32,
    /// The q -> v span, kg/m^3 (`air_density`, default [`AIR_DENSITY`], which also replaces a density that is not
    /// finite and above 0).
    pub air_density: f32,
    /// The configured zero offset, Pa x 100 (`zero_offset_pa`, default 0.0 Pa): a stored tare wins over it.
    pub zero_offset: i32,
}

impl Settings {
    /// Read a config block; a missing or non-numeric key takes plan B's default, and so does an air density that
    /// cannot divide (0 or below: see the module notes).
    pub fn from_config(config: &Value) -> Settings {
        let number = |key: &str| config.get(key).and_then(Value::as_f64);
        Settings {
            address: config
                .get("addr")
                .and_then(Value::as_u64)
                .and_then(|address| u8::try_from(address).ok())
                .unwrap_or(ADDRESS),
            period_ms: config
                .get("period_ms")
                .and_then(Value::as_u64)
                .and_then(|period| u32::try_from(period).ok())
                .unwrap_or(DEFAULT_PERIOD_MS),
            air_density: number("air_density")
                .map(|density| density as f32)
                .filter(|density| density_ok(*density))
                .unwrap_or(AIR_DENSITY),
            zero_offset: hundredths(number("zero_offset_pa").map_or(0.0, |pascal| pascal as f32)),
        }
    }
}

/// One good frame, converted.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// The tared dynamic pressure, Pa (signed: reverse flow and sub-zero noise stay negative): the
    /// `dynamic_pressure` channel.
    pub dynamic_pressure: f32,
    /// The indicated airspeed, m/s, from the dynamic pressure (0 for q <= 0): the `airspeed` channel, which feeds the
    /// governor's estimator.
    pub airspeed: f32,
    /// The sensor's temperature, degrees C.
    pub temperature: f32,
    /// The telemetry row: `dynamic_pressure` (Pa x 100), `airspeed_cms`, `temperature` (degrees C x 100).
    pub row: [i32; 3],
}

/// Why a read produced no sample, and what the restart ladder did about it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Failure<BusError> {
    /// What failed: the bus (plan B's `OSError`, counted toward the restart) or a CRC (noted, never counted).
    pub fault: Fault<BusError>,
    /// Some on the one failure that completed a run of [`RESTART_AFTER`]: the outcome of re-issuing stop-then-start
    /// (on Ok plan B logs 'restarted continuous mode after 5 failures').
    pub restart: Option<Result<(), Fault<BusError>>>,
}

/// Why an operator `update` was refused; nothing has changed when it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// A tare before the first frame: there is nothing to capture yet. Refused OUT LOUD: plan B once returned
    /// `changed: []`, which looked like success on the one command whose whole purpose is a change.
    NoReading,
    /// A property that must be a number is not one.
    NotANumber(&'static str),
    /// A number outside what the property can be: an air density that is not finite and above 0, which would turn
    /// every positive q into an infinite or NaN airspeed (plan B accepted it, see the module notes).
    OutOfRange(&'static str),
}

impl core::fmt::Display for Refusal {
    /// The refusal text CC carries (`err refused ...`).
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::NoReading => write!(formatter, "no reading yet -- wait for the first frame before taring"),
            Refusal::NotANumber(key) => write!(formatter, "{key} must be a number"),
            Refusal::OutOfRange(key) => write!(formatter, "{key} must be above 0"),
        }
    }
}

/// What an accepted `update` did.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Update {
    /// The changed property names, in plan B's order: a tare and a direct offset in one update each report
    /// `zero_offset_pa`, so it can appear twice, as it does in plan B's reply.
    pub changed: Vec<&'static str>,
    /// Some(tare) when the update captured a tare: the Pa x 100 the caller writes to NVS ([`NVS_ZERO_KEY`]).
    pub persist: Option<i32>,
}

/// One SDP810 on a bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Sdp810 {
    /// The I2C address.
    address: u8,
    /// The q -> v span, kg/m^3.
    air_density: f32,
    /// The zero offset, Pa x 100: the NVS unit, so a stored tare reads back as exactly what was captured.
    zero: i32,
    /// Whether a tare was captured, restored or set: a FLAG, not `zero != 0`, because a still pitot very often reads
    /// exactly 0 counts. On TMS-7E the operator retried 3-4 times until noise gave a non-zero count, and after a
    /// reboot a stored 0 read as never tared, so the board sat on `needs-calibration` for good.
    tared: bool,
    /// The scale factor latched at setup (Pa = raw / scale); never 0.
    scale: u16,
    /// The last raw pressure count, the tare source; None until the first frame, NOT 0: initialising it to 0 made
    /// the before-the-first-frame guard dead code, and an operator taring straight after boot captured 0.
    raw: Option<i16>,
    /// The last raw temperature word.
    temperature_raw: i16,
    /// The run of consecutive read failures, toward the restart.
    strikes: Strikes,
}

impl Sdp810 {
    /// A driver for the part these settings describe, untared, at the default scale until setup latches the part's.
    pub fn new(settings: &Settings) -> Sdp810 {
        Sdp810 {
            address: settings.address,
            air_density: settings.air_density,
            zero: settings.zero_offset,
            tared: false,
            scale: DEFAULT_SCALE,
            raw: None,
            temperature_raw: 0,
            strikes: Strikes::default(),
        }
    }

    /// Restore the tare a previous session stored (NVS [`NVS_ZERO_KEY`], Pa x 100): it WINS over the config.
    ///
    /// It was measured on this airframe with this tube run, where the config value is a guess. Restoring it means a
    /// reboot -- including one mid-glide, where nobody can re-tare -- comes back with a real zero instead of the
    /// whole interior-static bias in the airspeed the governor caps off. A stored 0 is still a tare.
    pub fn restore_zero(&mut self, stored: i32) {
        self.zero = stored;
        self.tared = true;
    }

    /// Latch the scale word of setup's good frame: every reading after it divides by it. A part answering 0 keeps
    /// the -500Pa default rather than divide by zero.
    pub fn latch_scale(&mut self, word: u16) {
        self.scale = if word == 0 { DEFAULT_SCALE } else { word };
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// The scale factor in use (Pa = raw / scale).
    pub fn scale(&self) -> u16 {
        self.scale
    }

    /// The zero offset, Pa.
    pub fn zero_offset_pa(&self) -> f32 {
        self.zero as f32 / HUNDREDTHS as f32
    }

    /// The zero offset in the NVS unit, Pa x 100.
    pub fn zero_offset(&self) -> i32 {
        self.zero
    }

    /// Whether a tare was captured, restored or set (0 Pa included).
    pub fn tared(&self) -> bool {
        self.tared
    }

    /// The q -> v span, kg/m^3.
    pub fn air_density(&self) -> f32 {
        self.air_density
    }

    /// The sensor temperature of the last frame, degrees C (0 before the first).
    pub fn temperature(&self) -> f32 {
        f32::from(self.temperature_raw) / TEMPERATURE_LSB
    }

    /// The tared dynamic pressure, Pa, of a raw signed count (sdp810._pressure); the count becomes the tare source.
    pub fn pressure(&mut self, raw: i16) -> f32 {
        self.raw = Some(raw);
        f32::from(raw) / f32::from(self.scale) - self.zero as f32 / HUNDREDTHS as f32
    }

    /// The telemetry's dynamic pressure, Pa x 100, of a raw count: `raw x 100 // scale - zero`, plan B's column to
    /// the digit (floored, so -600 at scale 60 is -1000 and 200 is 333).
    pub fn pressure_hundredths(&self, raw: i16) -> i32 {
        (i32::from(raw) * HUNDREDTHS).div_euclid(i32::from(self.scale)) - self.zero
    }

    /// The indicated airspeed, m/s, of a dynamic pressure in Pa: q = rho v^2 / 2 inverted (sdp810._airspeed).
    ///
    /// A negative q (reverse flow, sub-zero noise) yields 0 m/s, never a complex root; so does a NaN. The governor
    /// consumes this ready airspeed and takes no root of its own.
    pub fn airspeed(&self, pressure: f32) -> f32 {
        if pressure > 0.0 {
            (2.0 * pressure / self.air_density).sqrt()
        } else {
            0.0
        }
    }

    /// Read and convert one frame: the run loop's pass (sdp810.run).
    ///
    /// A bus failure counts toward the restart: the one that completes a run of [`RESTART_AFTER`] re-issues
    /// stop-then-start right here (it waits [`STOP_SETTLE_MS`] + [`FIRST_RESULT_MS`]) and reports how that went. A
    /// CRC reject is only reported: it neither counts toward the restart nor rearms it. A good frame rearms it.
    pub fn read<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<Sample, Failure<Bus::Error>> {
        let mut frame = [0u8; FRAME_LENGTH];
        if let Err(error) = bus.read(self.address, &mut frame) {
            let restart = self
                .strikes
                .strike(true, RESTART_AFTER)
                .then(|| self.restart(bus, delay));
            return Err(Failure {
                fault: Fault::Bus(error),
                restart,
            });
        }
        if !frame_ok(&frame) {
            return Err(Failure {
                fault: Fault::Checksum,
                restart: None,
            });
        }
        let raw = signed16(frame[0], frame[1]);
        let dynamic_pressure = self.pressure(raw);
        let airspeed = self.airspeed(dynamic_pressure);
        self.temperature_raw = signed16(frame[3], frame[4]);
        self.strikes.strike(false, RESTART_AFTER);
        Ok(Sample {
            dynamic_pressure,
            airspeed,
            temperature: self.temperature(),
            row: [
                self.pressure_hundredths(raw),
                airspeed_cms(airspeed),
                i32::from(self.temperature_raw).div_euclid(2),
            ],
        })
    }

    /// Re-issue continuous measurement after the part was reset out from under the driver (sdp810._restart):
    /// stop, settle, start, wait for the first result.
    ///
    /// The concrete trigger was a PEER: the ICP-10111's latch-up recovery fires a general-call reset, which by design
    /// resets this part too. Observed as `sdp810 :: read OSError(19)` with the channel flat at 0.0 Hz while the part
    /// itself was healthy. Stop first, because the part refuses a fresh start while still streaming.
    fn restart<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        bus.write(self.address, &COMMAND_STOP).map_err(Fault::Bus)?;
        delay.delay_ms(STOP_SETTLE_MS);
        bus.write(self.address, &COMMAND_START).map_err(Fault::Bus)?;
        delay.delay_ms(FIRST_RESULT_MS);
        Ok(())
    }

    /// One start attempt: stop, settle, start, wait, read a frame.
    fn start<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<[u8; FRAME_LENGTH], Bus::Error> {
        bus.write(self.address, &COMMAND_STOP)?;
        delay.delay_ms(STOP_SETTLE_MS);
        bus.write(self.address, &COMMAND_START)?;
        delay.delay_ms(FIRST_RESULT_MS);
        let mut frame = [0u8; FRAME_LENGTH];
        bus.read(self.address, &mut frame)?;
        Ok(frame)
    }

    /// Best-effort stop of continuous mode (the setup-failure and teardown path); a dead bus is swallowed.
    pub fn stop<Bus: I2c>(&mut self, bus: &mut Bus) {
        let _ = bus.write(self.address, &COMMAND_STOP);
    }

    /// Capture the last reading as the zero offset: the pad tare (update `{"zero": true}`, calibrate).
    ///
    /// Returns the tare, Pa x 100, for the caller to PERSIST under [`NVS_ZERO_KEY`] (it lived in RAM only once, so a
    /// battery re-seat between the pad tare and launch brought back an older NVS tare), or None before the first
    /// frame: storing nothing then is the point, as a None tare once raised on every later read and was persisted.
    pub fn tare(&mut self) -> Option<i32> {
        let raw = self.raw?;
        self.zero = (i32::from(raw) * HUNDREDTHS).div_euclid(i32::from(self.scale));
        self.tared = true;
        Some(self.zero)
    }

    /// Calibrate: capture the still-air zero (sdp810.calibrate). Returns the tare to persist, or plan B's
    /// [`CALIBRATE_NO_READING`] before the first frame.
    pub fn calibrate(&mut self) -> Result<i32, &'static str> {
        self.tare().ok_or(CALIBRATE_NO_READING)
    }

    /// Set the zero offset directly, in Pa: the warm start restoring its crumb (what [`Sdp810::zero_offset_pa`]
    /// gave it, returned to the hundredth), or the operator. It is a tare (0.0 Pa included) but is NOT persisted:
    /// mid-air, a flash write is not wanted.
    pub fn set_zero_offset(&mut self, pascal: f32) {
        self.zero = hundredths(pascal);
        self.tared = true;
    }

    /// Set the q -> v span, kg/m^3 (the GNSS-vs-q calm-pass trim); a density that is not finite and above 0 is
    /// refused and the span is left as it was.
    pub fn set_air_density(&mut self, density: f32) -> Result<(), Refusal> {
        if !density_ok(density) {
            return Err(Refusal::OutOfRange("air_density"));
        }
        self.air_density = density;
        Ok(())
    }

    /// The still-air tare instruction while one is owed, None once a tare was captured, restored or set
    /// (sdp810.calibration); `dynamic_pressure` is the channel's latest, Pa, for the '(now ...)' part.
    pub fn calibration(&self, dynamic_pressure: Option<f32>) -> Option<String> {
        (!self.tared).then(|| {
            format!(
                "keep the pitot in STILL AIR -- do NOT blow into it, this captures the zero tare (now {:.2} Pa)",
                dynamic_pressure.unwrap_or(0.0)
            )
        })
    }

    /// Apply an operator property change (sdp810.update), in plan B's order: `zero` (a truthy flag: tare and
    /// persist), `zero_offset_pa` (a direct offset, not persisted), `air_density`.
    ///
    /// Refused as a whole, before anything changes, when a tare finds no reading yet, a number is not a number, or
    /// the air density is not above 0. (Plan B checked the numbers as it reached them, so an update that tared and
    /// then carried a bad number had already captured and persisted the tare when it raised; and it took a numeric
    /// string too. Neither is a path the operator's tools take. It took any density, which the port does not.)
    pub fn update(&mut self, properties: &Value) -> Result<Update, Refusal> {
        let number = |key: &'static str| -> Result<Option<f64>, Refusal> {
            properties
                .get(key)
                .map(|value| match value {
                    Value::Bool(flag) => Ok(f64::from(u8::from(*flag))),
                    _ => value.as_f64().ok_or(Refusal::NotANumber(key)),
                })
                .transpose()
        };
        let zero_offset = number("zero_offset_pa")?;
        let air_density = number("air_density")?.map(|density| density as f32);
        if air_density.is_some_and(|density| !density_ok(density)) {
            return Err(Refusal::OutOfRange("air_density"));
        }
        let tare = properties.get("zero").is_some_and(truthy);
        if tare && self.raw.is_none() {
            return Err(Refusal::NoReading);
        }
        let mut changed = Vec::new();
        let mut persist = None;
        if tare {
            persist = self.tare();
            changed.push("zero_offset_pa");
        }
        if let Some(pascal) = zero_offset {
            self.set_zero_offset(pascal as f32);
            changed.push("zero_offset_pa");
        }
        if let Some(density) = air_density {
            self.air_density = density; // checked above, before anything changed
            changed.push("air_density");
        }
        Ok(Update { changed, persist })
    }

    /// Classify a failed setup at the wire level (sdp810.diagnose): read one frame.
    ///
    /// The SDP8xx has no cheap product-id register (identity needs a two-command sequence), so presence is judged
    /// from the continuous frame: no ack, an ack with a bad CRC, or a clean frame setup still rejected.
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String {
        let mut frame = [0u8; FRAME_LENGTH];
        if bus.read(self.address, &mut frame).is_err() {
            return format!("absent -- no ack at 0x{:02x} (check wiring / bus id)", self.address);
        }
        if !frame_ok(&frame) {
            return format!(
                "garbage -- acks at 0x{:02x} but CRC fails (wrong device or bus noise)",
                self.address
            );
        }
        format!(
            "present at 0x{:02x} but setup rejected the frame (scale {}?)",
            self.address,
            u16::from_be_bytes([frame[6], frame[7]])
        )
    }
}

/// Python's truthiness of a JSON value: null, false, 0, "", [] and {} are false.
fn truthy(value: &Value) -> bool {
    match value {
        Value::Null => false,
        Value::Bool(flag) => *flag,
        Value::Number(number) => number.as_f64() != Some(0.0),
        Value::String(text) => !text.is_empty(),
        Value::Array(items) => !items.is_empty(),
        Value::Object(members) => !members.is_empty(),
    }
}

impl<Bus: I2c> Device<Bus> for Sdp810 {
    type BusError = Bus::Error;

    /// Stop (harmless when already idle), start, and read one CRC-checked frame as the presence and health test,
    /// [`START_TRIES`] times; the good frame's scale word is latched.
    ///
    /// Nothing acks across the tries -> absent ([`Fault::Bus`]). Acks but the frame is corrupt -> not a healthy
    /// SDP8xx here ([`Fault::Checksum`]), and the part is left stopped.
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        let mut attempt = 1;
        loop {
            match self.start(bus, delay) {
                Ok(frame) if frame_ok(&frame) => {
                    self.latch_scale(u16::from_be_bytes([frame[6], frame[7]]));
                    return Ok(());
                }
                Ok(_) if attempt == START_TRIES => {
                    self.stop(bus);
                    return Err(Fault::Checksum);
                }
                Err(error) if attempt == START_TRIES => return Err(Fault::Bus(error)),
                _ => attempt += 1,
            }
        }
    }

    /// Stop-then-start, at once: the restart the read ladder would otherwise reach only after [`RESTART_AFTER`]
    /// failed reads (plan B left this part to that ladder; the membership re-arm saves those reads).
    fn rearm(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.restart(bus, delay)
    }
}

#[cfg(test)]
mod tests;
