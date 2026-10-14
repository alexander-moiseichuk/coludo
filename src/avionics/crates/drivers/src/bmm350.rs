//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMM350 3-axis magnetometer on the SEN0697 (src/glider/drivers/bmm350.py): the `mag` channel.
//!
//! Why it is driven at all, having been dismissed once: heading. The attitude filter integrates gyro z for heading
//! and bounds the drift with the GNSS ground TRACK, which works only above the course gate and not at all while the
//! fix is out. Boost is exactly when a consumer GNSS module loses lock, so the one reference that corrects heading
//! is missing during the phase that introduces the most drift. A magnetometer is the only sensor on the board that
//! bounds heading WITHOUT a fix.
//!
//! RAW COUNTS, deliberately. Turning counts into microtesla needs each part's OTP compensation coefficients, and
//! that is worth skipping: heading needs a DIRECTION, and the hard/soft-iron fit this airframe needs anyway (carbon,
//! servo currents, a booster) also absorbs the per-axis sensitivity differences. So the channel carries counts --
//! raw until the operator calibrates, then centred and normalised to +/-[`UNIT`] -- and only the absolute field
//! strength is given up, which nothing here wants. The counts are integers by nature (24-bit), and the calibration
//! is integer extrema of them, so this module computes in integers, as plan B does, to the count: these are not
//! fixed-point stand-ins for a real number but the part's own unit, and the NVS blob and the telemetry columns
//! (mx, my, mz) carry them as they are.
//!
//! Two details worth stating: every I2C register read returns TWO DUMMY BYTES before the data (the BMI323's quirk
//! too), and each axis is 24-bit two's complement, little-endian.
//!
//! THE OTP POWER-OFF IS NOT OPTIONAL ([`CONFIGURATION`]). It is Bosch's own init step: without it every data
//! register reads 0x7F and the part never produces a sample, while its chip id -- and so the old probe -- reads
//! perfectly. That is how plan B's driver shipped and passed bring-up on the SEN0697 without its magnetometer ever
//! having measured a field: `mag` published a constant, the calibration refused it as a dead part, and the filter's
//! magnetic yaw would have pulled toward a fixed vector. [`not_measuring`] is the probe's check for exactly that.
//!
//! THE CALIBRATION is the operator's: run() accumulates the evidence continuously ([`Bmm350::observe`]: the
//! extrema of x and y, and which octants of the circle the samples visited), so by the time the operator presses
//! calibrate after turning the airframe through a level circle the evidence already exists. [`Bmm350::calibrate`]
//! removes what a single learned offset cannot: the circle's CENTRE (hard iron) and its OUT-OF-ROUNDNESS (soft iron
//! and per-axis scale). Both shift heading by an amount that depends on where the nose points, so the attitude
//! filter's one learned track offset cannot absorb either. It refuses a partial turn on ANGULAR COVERAGE, not on the
//! shape of the spans (see [`Bmm350::calibrate`]).
//!
//! Register values are Bosch's own (bmm350_defs.h).
//!
//! THE RATE (avionics-port.md M9 D2, a divergence): 12.5 Hz with averaging 8 (PMU_CMD_AGGR_SET 0x37), where plan B
//! ran 100 Hz with averaging 4 (0x24) and read it at 10 Hz. Heading moves slowly, so every conversion is read and
//! recorded at the rate its consumers need, with twice the averaging; the firmware polls every [`POLL_PERIOD_MS`],
//! half a conversion period, so a sample is at most that old when it is read.
//!
//! NEW CONVERSIONS ONLY (D3). [`Bmm350::read_fresh`] judges each read by its 9 axis bytes
//! ([`crate::freshness::Changes`]; the dummy bytes and the temperature are not compared): fresh when they changed,
//! which every conversion does (a still board's noise spans hundreds of counts). Polled twice a conversion, each
//! conversion is fresh once, and only a fresh field is [`Bmm350::observe`]d, so the evidence counts conversions,
//! never a read twice. A SUSPENDED part repeats its bytes, and once it has for
//! [`crate::freshness::STALE_PERIODS`] conversion periods the read is [`Change::Stale`], where the firmware calls it
//! unhealthy. No register is written for it. Plan B's `period_ms` is unused.
//!
//! NO SAMPLE IS NOT A SAMPLE. MEASURED on the board (M9 stage 4): right after a re-arm the axes read [`NO_DATA`]
//! until the first conversion lands. Changed bytes would call that pattern fresh (it differs from the last real
//! sample), the run would observe it, and the span's x and y maximum would be 8 355 711 for the rest of the boot,
//! refusing every calibration on coverage. So a field that is no sample ([`Field::measured`] false: raw x and y at
//! [`NO_DATA`], or all three axes zero) is never judged and never stored: [`Bmm350::read_fresh`] answers it
//! [`Change::Unchanged`], or [`Change::Stale`] once the stale clock of the last real change runs out, so a part stuck
//! there still turns unhealthy. And [`Bmm350::observe`] never accounts one as evidence, whoever reads it (the probe
//! observes too), a divergence from plan B, whose `_read` accounted every burst.
//!
//! THE AXES ([`crate::axes`], new in the port; plan B has none). The order against the calibration is fixed here, in
//! the driver, because it is the one place it can go wrong: the evidence ([`Bmm350::observe`]), the calibration
//! ([`Bmm350::calibrate`]) and the NVS blob are all in the SENSOR's own frame, and the map turns only the PUBLISHED
//! vector onto the airframe's axes, last ([`Bmm350::mounted`]); the firmware does not map `mag` again. Two reasons:
//! the blob is shared with plan B, which speaks sensor x/y, so a calibration taken on either firmware stays valid
//! after a reflash to the other whatever the mounting; and a level turn sweeps the sensor's x/y plane only while
//! that plane is level, so a mounting that tilts the sensor's z off the vertical is refused at construction
//! ([`crate::axes::AxesError::Tilted`]) rather than left to refuse every calibration on coverage, forever.
//!
//! THE CALIBRATION'S ORDER (D9), plan B's: persist, then apply. The bus owner answers a calibrate with the
//! [`Calibration`] [`Bmm350::calibrate`] computes, NOT applied; the executor writes its 16 bytes to NVS; only then does
//! the owner put it in force ([`Bmm350::set_calibration`]). A refused calibration has nothing to persist, and one
//! that could not be persisted is never applied. At boot the blob is restored after setup ([`Bmm350::restore`]: a
//! blob of any other length leaves the axes raw). The calibration column's text is [`format_instruction`] of the
//! values inspect carries (calibrated, sectors, samples), so the executor formats it from the owner's atomics without
//! borrowing the live part.
//!
//! Not here (firmware glue): the NVS read and write of the calibration (namespace `coludo`, key [`NVS_KEY`], 16
//! bytes, shared with plan B so a calibration survives a reflash between the two), the channel push, the telemetry
//! stream, the probe's log lines and the inspect JSON.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::axes::{Axes, AxesError};
use crate::device::{Device, Fault};
pub use crate::freshness::Change;
use crate::freshness::Changes;

/// The default I2C address (I2C_ADSEL high; 0x14 when pulled low).
pub const ADDRESS: u8 = 0x15;

/// CHIP_ID: the identity register.
pub const CHIP_ID: u8 = 0x00;

/// PMU_CMD_AGGR_SET: the output data rate and averaging.
pub const PMU_CMD_AGGR_SET: u8 = 0x04;

/// PMU_CMD: the power mode command.
pub const PMU_CMD: u8 = 0x06;

/// MAG_X_XLSB: mag x, y, z then temperature, three bytes each, twelve contiguous bytes.
pub const MAG_X_XLSB: u8 = 0x31;

/// OTP_CMD_REG: the OTP command register.
pub const OTP_CMD_REG: u8 = 0x50;

/// CMD: the command register (the soft reset).
pub const CMD: u8 = 0x7E;

/// What CHIP_ID reads on a BMM350 (past the dummy bytes).
pub const IDENTITY: u8 = 0x33;

/// One step of the bring-up table: a register write, or a wait the part needs before the next one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Write `value` to `register`.
    Write {
        /// The register.
        register: u8,
        /// The value.
        value: u8,
    },
    /// Wait this many milliseconds.
    Wait(u32),
}

/// The bring-up (and re-arm) table, in Bosch's order (bmm350._configure):
///  * CMD = 0xB6, the soft reset, then 25 ms: the part reloads its OTP after a reset;
///  * OTP_CMD_REG = 0x80 (PWR_OFF_OTP), then 5 ms: until this is written the part never measures;
///  * PMU_CMD_AGGR_SET = 0x37: averaging 8 (bits 5:4 = 3), 12.5 Hz (bits 3:0 = 7, Bosch's BMM350_DATA_RATE_12_5HZ;
///    100 Hz is 4, and plan B wrote 0x24, averaging 4 at 100 Hz). Averaging 8 is allowed at 50 Hz and below. The one
///    byte the port writes differently from plan B (D2), pinned in the golden fixture as a listed divergence;
///  * PMU_CMD = 0x02 (UPD_OAE), then 5 ms: a new rate needs its own apply command;
///  * PMU_CMD = 0x01 (normal mode), then 10 ms.
pub const CONFIGURATION: [Step; 9] = [
    Step::Write {
        register: CMD,
        value: 0xB6,
    },
    Step::Wait(25),
    Step::Write {
        register: OTP_CMD_REG,
        value: 0x80,
    },
    Step::Wait(5),
    Step::Write {
        register: PMU_CMD_AGGR_SET,
        value: 0x37,
    },
    Step::Write {
        register: PMU_CMD,
        value: 0x02,
    },
    Step::Wait(5),
    Step::Write {
        register: PMU_CMD,
        value: 0x01,
    },
    Step::Wait(10),
];

/// The bytes the part prepends to every I2C register read.
pub const DUMMY_BYTES: usize = 2;

/// The burst: the dummy bytes, three axes and the temperature, three bytes each.
pub const FRAME_LENGTH: usize = DUMMY_BYTES + 12;

/// The axis bytes a read's freshness is judged on: x, y and z, three bytes each, past the dummy bytes.
const AXIS_BYTES: usize = 9;

/// What every axis reads while the OTP is still powered: the part's own "no sample", never a field.
pub const NO_DATA: i32 = 0x7F_7F7F;

/// The NVS key of the calibration (namespace `coludo`): four i32 little-endian, x centre, y centre, x radius,
/// y radius.
pub const NVS_KEY: &str = "bmm350_cal";

/// The calibration blob's length.
pub const CALIBRATION_BYTES: usize = 16;

/// The corrected axes are normalised to +/- this, so the heading's atan2 sees equally scaled integers.
pub const UNIT: i64 = 1000;

/// The span, in counts, below which no field change was seen. MEASURED on the v1.1 bench (2026-09-23): the part's
/// field sphere fitted at ~10 000 counts radius, so a level turn spans roughly 8 000-10 000 counts while a still
/// board's noise spans ~200. It used to be 50 -- below that noise -- which let a board that never moved pass as
/// calibrated.
pub const MINIMUM_SPAN: i32 = 2000;

/// Counts from the centre before a sample may set a sector (still-board noise is ~+/-100).
pub const MINIMUM_REACH: i32 = 1000;

/// The circle is split into octants; covering them is what proves a real turn.
pub const SECTORS: u32 = 8;

/// Octants a turn must cover: 7 of 8 is 315 degrees, so a lap that stops just short still counts.
pub const NEEDED_SECTORS: u32 = 7;

/// Samples seen before a calibration is considered at all.
pub const MINIMUM_SAMPLES: u32 = 100;

/// The conversion period at the configured 12.5 Hz, microseconds.
pub const CONVERSION_PERIOD_US: u64 = 80_000;

/// How often the firmware reads the part, ms: half a conversion period, so each conversion is read within 40 ms of
/// landing (and read twice, fresh once).
pub const POLL_PERIOD_MS: u32 = 40;

/// The sign bit of a 24-bit two's-complement axis.
const SIGN_24: i32 = 0x80_0000;

/// One more than the largest 24-bit count: what a negative axis's raw value is offset by.
const RANGE_24: i32 = 0x100_0000;

/// One burst's axes, raw counts (the temperature that follows them is not used).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Field {
    /// x counts.
    pub x: i32,
    /// y counts.
    pub y: i32,
    /// z counts.
    pub z: i32,
}

impl Field {
    /// Decode a 14-byte burst past the dummy bytes: three 24-bit two's-complement axes, little-endian
    /// (bmm350._axis). A magnetometer reads NEGATIVE on half of every turn, so an unsigned decode would still give
    /// plausible numbers -- and a heading wrong in exactly one half-plane.
    pub fn decode(frame: &[u8; FRAME_LENGTH]) -> Field {
        let axis = |at: usize| {
            let raw = i32::from(frame[at]) | (i32::from(frame[at + 1]) << 8) | (i32::from(frame[at + 2]) << 16);
            if raw & SIGN_24 != 0 { raw - RANGE_24 } else { raw }
        };
        Field {
            x: axis(DUMMY_BYTES),
            y: axis(DUMMY_BYTES + 3),
            z: axis(DUMMY_BYTES + 6),
        }
    }

    /// Whether the burst is a sample at all (the raw burst, before any calibration): not x and y at [`NO_DATA`], the
    /// part's own "no sample" (until its OTP is powered off, and after a re-arm until the first conversion), and not
    /// all three axes zero (the field is never zero on Earth). [`not_measuring`]'s raw checks, without its reasons.
    pub fn measured(&self) -> bool {
        !(self.x == NO_DATA && self.y == NO_DATA) && *self != Field::default()
    }
}

/// Why a probe's sample says the part is not measuring (bmm350.probe), or None when it is.
///
/// `published` is what the channel would carry (raw, or corrected once calibrated) and `raw` the burst as read:
///  * all published axes zero: the field is never zero on Earth, so zero means the part is not measuring (a
///    SUSPENDED part answers every read with the same counts, which a bare "it answered" probe passes);
///  * the RAW x and y at 0x7F7F7F: the part's own "no sample", what it reads until its OTP is powered off. Checked
///    on the raw burst, because a calibration would disguise it.
pub fn not_measuring(raw: &Field, published: [i32; 3]) -> Option<&'static str> {
    if published == [0, 0, 0] {
        return Some("all axes zero -- the part is not measuring");
    }
    if raw.x == NO_DATA && raw.y == NO_DATA {
        return Some("axes read 0x7F7F7F -- the part is not measuring (OTP still powered?)");
    }
    None
}

/// Why the probe failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProbeFailure<BusError> {
    /// The identity or the read failed: no ack, or another part (plan B's 'chip id: ...' and 'read: ...').
    Fault(Fault<BusError>),
    /// The part answers but does not measure: [`not_measuring`]'s reason.
    NotMeasuring(&'static str),
}

/// The calibration column's text from the values inspect carries (plan B's calibration()): '' when a calibration
/// is in force; the turn asked for with no sample `seen` yet; otherwise the progress, `covered` octants of
/// [`SECTORS`] and the samples seen. Pure, so the executor formats it from the owner's atomics;
/// [`Bmm350::instruction`] is this of the live part.
pub fn format_instruction(calibrated: bool, covered: u32, seen: u32) -> String {
    if calibrated {
        return String::new();
    }
    if seen == 0 {
        return "turn the airframe through a full circle, LEVEL, then calibrate (no field seen yet)".to_string();
    }
    format!(
        "turn the airframe slowly through a FULL CIRCLE, keeping it level -- {covered} of {SECTORS} sectors covered, \
         {seen} samples"
    )
}

/// A hard/soft-iron calibration: the circle's centre and radii, in counts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Calibration {
    /// x of the centre (the hard iron).
    pub centre_x: i32,
    /// y of the centre.
    pub centre_y: i32,
    /// The x half-span (soft iron and per-axis scale).
    pub radius_x: i32,
    /// The y half-span.
    pub radius_y: i32,
}

impl Calibration {
    /// The NVS blob: four i32 little-endian (plan B's `struct.pack('<4i')`).
    pub fn to_bytes(&self) -> [u8; CALIBRATION_BYTES] {
        let mut blob = [0u8; CALIBRATION_BYTES];
        for (index, value) in [self.centre_x, self.centre_y, self.radius_x, self.radius_y]
            .iter()
            .enumerate()
        {
            blob[4 * index..4 * index + 4].copy_from_slice(&value.to_le_bytes());
        }
        blob
    }

    /// A calibration from an NVS blob; None for any length but [`CALIBRATION_BYTES`] (a short blob is not a
    /// calibration, and the axes stay raw).
    pub fn from_bytes(blob: &[u8]) -> Option<Calibration> {
        if blob.len() != CALIBRATION_BYTES {
            return None;
        }
        let value = |index: usize| {
            let mut word = [0u8; 4];
            word.copy_from_slice(&blob[4 * index..4 * index + 4]);
            i32::from_le_bytes(word)
        };
        Some(Calibration {
            centre_x: value(0),
            centre_y: value(1),
            radius_x: value(2),
            radius_y: value(3),
        })
    }

    /// Re-centre a raw field and scale x and y to +/-[`UNIT`] (the z axis passes raw): all a heading needs, since
    /// atan2 cares about the ratio, not the magnitude. Floored division, as plan B's `//`; a zero radius divides by 1
    /// (`radius or 1`). Computed wide and saturated to +/-[`i32::MAX`] -- symmetric, so the axes map can negate any
    /// corrected value: only a corrupt blob with a tiny radius gets there.
    pub fn correct(&self, field: &Field) -> [i32; 3] {
        let normalise = |value: i32, centre: i32, radius: i32| {
            let divisor = if radius == 0 { 1 } else { i64::from(radius) };
            let scaled = floor_divide((i64::from(value) - i64::from(centre)) * UNIT, divisor);
            scaled.clamp(-i64::from(i32::MAX), i64::from(i32::MAX)) as i32
        };
        [
            normalise(field.x, self.centre_x, self.radius_x),
            normalise(field.y, self.centre_y, self.radius_y),
            field.z,
        ]
    }
}

/// Python's floor division `//` for any signed divisor (div_euclid differs from it when the divisor is negative).
fn floor_divide(numerator: i64, divisor: i64) -> i64 {
    let quotient = numerator / divisor;
    if numerator % divisor != 0 && ((numerator < 0) != (divisor < 0)) {
        quotient - 1
    } else {
        quotient
    }
}

/// The calibration evidence: the extrema of x and y since boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Span {
    /// The smallest x seen.
    pub x_minimum: i32,
    /// The largest x seen.
    pub x_maximum: i32,
    /// The smallest y seen.
    pub y_minimum: i32,
    /// The largest y seen.
    pub y_maximum: i32,
}

impl Span {
    /// The centre of the box the extrema draw, floored as plan B's `//`.
    fn centre(&self) -> (i32, i32) {
        (
            (self.x_minimum + self.x_maximum).div_euclid(2),
            (self.y_minimum + self.y_maximum).div_euclid(2),
        )
    }

    /// The x extent.
    pub fn x_span(&self) -> i32 {
        self.x_maximum - self.x_minimum
    }

    /// The y extent.
    pub fn y_span(&self) -> i32 {
        self.y_maximum - self.y_minimum
    }
}

/// One BMM350 on an I2C bus, with its calibration evidence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bmm350 {
    /// The 7-bit address.
    address: u8,
    /// The extrema seen since boot; None before the first sample.
    span: Option<Span>,
    /// Bitmask of the octants visited around [`Bmm350::centre`].
    sectors: u8,
    /// The provisional centre the sector bits were accumulated against.
    centre: Option<(i32, i32)>,
    /// Samples seen since boot.
    seen: u32,
    /// The calibration in force (restored or just captured); None leaves the axes raw.
    calibration: Option<Calibration>,
    /// The sensor-to-airframe map, applied to the published vector only (see the module notes).
    axes: Axes,
    /// The axis bytes of the last [`Bmm350::read_fresh`] and when they last changed.
    changes: Changes<AXIS_BYTES>,
}

impl Bmm350 {
    /// A driver for the part at `address`, mounted per its marks, with no evidence and no calibration.
    pub fn new(address: u8) -> Bmm350 {
        Bmm350 {
            address,
            span: None,
            sectors: 0,
            centre: None,
            seen: 0,
            calibration: None,
            axes: Axes::IDENTITY,
            changes: Changes::default(),
        }
    }

    /// The same driver, mounted as the config's `axes` says ([`Axes::from_config`]): the published vector is
    /// turned onto the airframe's axes AFTER the calibration, which stays in the sensor frame (see the module
    /// notes). A map that tilts the sensor's z off the vertical is [`AxesError::Tilted`]: the firmware refuses the
    /// device at setup with it, naming the key.
    pub fn mounted(self, axes: Axes) -> Result<Bmm350, AxesError> {
        if !axes.keeps_vertical() {
            return Err(AxesError::Tilted(axes.to_string()));
        }
        Ok(Bmm350 { axes, ..self })
    }

    /// The sensor-to-airframe map the published vector goes through.
    pub fn axes(&self) -> Axes {
        self.axes
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// The calibration in force, if any.
    pub fn calibration(&self) -> Option<Calibration> {
        self.calibration
    }

    /// Put a calibration in force: one [`Bmm350::calibrate`] computed, once the firmware has persisted it (D9), or
    /// one restored from NVS ([`Bmm350::restore`]).
    pub fn set_calibration(&mut self, calibration: Calibration) {
        self.calibration = Some(calibration);
    }

    /// Restore the calibration NVS holds, after setup (bmm350._restore), so a power cycle does not cost the operator
    /// a turn. Returns the calibration put in force; None for a blob of any length but [`CALIBRATION_BYTES`], which is
    /// not a calibration: nothing changes and the axes stay raw.
    pub fn restore(&mut self, blob: &[u8]) -> Option<Calibration> {
        let calibration = Calibration::from_bytes(blob)?;
        self.set_calibration(calibration);
        Some(calibration)
    }

    /// Whether a hard/soft-iron calibration is in force (inspect's `calibrated`).
    pub fn calibrated(&self) -> bool {
        self.calibration.is_some()
    }

    /// The extrema seen since boot (inspect's `span`).
    pub fn span(&self) -> Option<Span> {
        self.span
    }

    /// Samples seen since boot (inspect's `samples`).
    pub fn seen(&self) -> u32 {
        self.seen
    }

    /// How many of the eight octants the turn has visited: the progress the operator is working on (inspect's
    /// `sectors`).
    pub fn covered(&self) -> u32 {
        self.sectors.count_ones()
    }

    /// The outstanding instruction, or '' once calibrated: what CC shows in the calibration column (plan B's text),
    /// [`format_instruction`] of this part's state. No sample seen is no span: both are set by the first
    /// [`Bmm350::observe`], and the count saturates rather than wrap back to 0.
    pub fn instruction(&self) -> String {
        format_instruction(self.calibrated(), self.covered(), self.seen)
    }

    /// Account one raw sample and return what the channel publishes (bmm350._read after the bus read): the raw
    /// counts while uncalibrated (so the channel works from the first boot and improves when the operator does the
    /// turn), the corrected ones once calibrated. On the run, only a [`Change::Fresh`] field is observed (the module
    /// notes), so the evidence never counts one conversion twice. A field that is no sample ([`Field::measured`]
    /// false) is published as it reads, for the probe's [`not_measuring`] verdict, but never accounted.
    ///
    /// `field` is the burst as the SENSOR reads it ([`Field::decode`]), never a remapped one: the evidence and the
    /// calibration live in the sensor frame, and only the returned vector goes through [`Bmm350::mounted`]'s map.
    pub fn observe(&mut self, field: &Field) -> [i32; 3] {
        if field.measured() {
            self.account(field);
        }
        let published = match self.calibration {
            None => [field.x, field.y, field.z],
            Some(calibration) => calibration.correct(field),
        };
        self.axes.apply(published)
    }

    /// Add one sample to the evidence: the extrema, the count and the octant.
    fn account(&mut self, field: &Field) {
        let span = match self.span {
            None => Span {
                x_minimum: field.x,
                x_maximum: field.x,
                y_minimum: field.y,
                y_maximum: field.y,
            },
            Some(span) => Span {
                x_minimum: span.x_minimum.min(field.x),
                x_maximum: span.x_maximum.max(field.x),
                y_minimum: span.y_minimum.min(field.y),
                y_maximum: span.y_maximum.max(field.y),
            },
        };
        self.span = Some(span);
        self.seen = self.seen.saturating_add(1);
        self.cover(&span, field);
    }

    /// Record which octant of the circle this sample falls in, around the CURRENT centre estimate (bmm350._cover).
    ///
    /// The centre is only known once the turn is done, so the estimate moves while the operator turns and early
    /// bits would describe a centre that no longer exists. So whenever the estimate MOVES materially (an eighth of
    /// the span) the mask is thrown away and coverage restarts -- the bits always describe one settled centre. A
    /// lap that shifts the estimate late asks for a bit more turning rather than accepting a half-measured circle;
    /// the instruction shows the count, so the operator sees it climbing instead of guessing.
    ///
    /// Samples hugging the centre have no meaningful angle. The floor is ABSOLUTE ([`MINIMUM_REACH`]): relative to
    /// the span alone, a board sitting still (span = its own noise, ~200 counts) reached 8 of 8 sectors in a few
    /// hundred samples, and calibrate() would have saved that noise as the hard iron. Octants come from three
    /// comparisons (two signs and |dx| against |dy|), so there is no atan2 in the sampling path.
    fn cover(&mut self, span: &Span, field: &Field) {
        let centre = span.centre();
        let extent = span.x_span().max(span.y_span());
        let moved = |previous: (i32, i32)| {
            (centre.0 - previous.0).abs() > extent / 8 || (centre.1 - previous.1).abs() > extent / 8
        };
        if self.centre.is_none_or(moved) {
            self.centre = Some(centre);
            self.sectors = 0;
        }
        let delta_x = i64::from(field.x) - i64::from(centre.0);
        let delta_y = i64::from(field.y) - i64::from(centre.1);
        let reach = i64::from((extent / 4).max(MINIMUM_REACH));
        if delta_x * delta_x + delta_y * delta_y < reach * reach {
            return;
        }
        let octant =
            (u8::from(delta_y >= 0) << 2) | (u8::from(delta_x >= 0) << 1) | u8::from(delta_x.abs() >= delta_y.abs());
        self.sectors |= 1 << octant;
    }

    /// Compute the hard/soft-iron calibration from the turn the operator has just done (bmm350.calibrate's
    /// arithmetic). Nothing is collected HERE: [`Bmm350::observe`] has been gathering the evidence all along, and
    /// nothing is applied here either.
    ///
    /// Returns the calibration for the firmware to PERSIST (key [`NVS_KEY`], [`Calibration::to_bytes`]) and then
    /// put in force with [`Bmm350::set_calibration`] -- plan B's order (D9): a calibration that could not be persisted
    /// is not applied. Err carries plan B's answer, the instruction still outstanding, never a bare success for a
    /// turn that never happened, and nothing to persist:
    ///  * no samples, or fewer than [`MINIMUM_SAMPLES`]: the instruction. That is '' when a calibration is already
    ///    in force (restored from NVS), plan B's answer to the letter, which CC shows as an empty result;
    ///  * the larger span under [`MINIMUM_SPAN`]: no field change seen -- a motionless or a dead part;
    ///  * fewer than [`NEEDED_SECTORS`] octants: a partial turn, REFUSED on angular coverage. Measured: a quarter
    ///    turn from 0 to 90 degrees has x span == y span exactly, so it scores a PERFECT roundness while putting the
    ///    centre 450 counts off (a 30 degree heading error); span shape cannot see it. Nor may it: a real soft iron
    ///    makes the circle genuinely elliptical, and that ellipse is what this calibration CORRECTS, so rejecting it
    ///    would refuse exactly the boards that need the correction most. The refusal is built here, not from the
    ///    instruction, which is '' once a calibration exists: a refused RE-calibration once read as success while
    ///    the old one silently stayed.
    pub fn calibrate(&self) -> Result<Calibration, String> {
        let span = match self.span {
            Some(span) if self.seen >= MINIMUM_SAMPLES => span,
            _ => return Err(self.instruction()),
        };
        let (x_span, y_span) = (span.x_span(), span.y_span());
        if x_span.max(y_span) < MINIMUM_SPAN {
            return Err(format!(
                "no field change seen (x span {x_span}, y span {y_span}) -- is the magnetometer alive?"
            ));
        }
        if self.covered() < NEEDED_SECTORS {
            return Err(format!(
                "refused -- only {} of {SECTORS} sectors covered: turn the airframe through a full LEVEL circle",
                self.covered()
            ));
        }
        let (centre_x, centre_y) = span.centre();
        Ok(Calibration {
            centre_x,
            centre_y,
            radius_x: x_span.div_euclid(2),
            radius_y: y_span.div_euclid(2),
        })
    }

    /// Read CHIP_ID PAST the two dummy bytes (a 3-byte read; the third byte is the identity).
    pub fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>> {
        let mut answer = [0u8; DUMMY_BYTES + 1];
        bus.write_read(self.address, &[CHIP_ID], &mut answer)
            .map_err(Fault::Bus)?;
        Ok(answer[DUMMY_BYTES])
    }

    /// Read one burst's raw axes (the evidence is accounted by [`Bmm350::observe`]).
    pub fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Field, Fault<Bus::Error>> {
        let mut frame = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[MAG_X_XLSB], &mut frame)
            .map_err(Fault::Bus)?;
        Ok(Field::decode(&frame))
    }

    /// Read one burst's raw axes and whether they are a new conversion ([`Change`]): the same 14-byte burst as
    /// [`Bmm350::read`], judged by whether its 9 axis bytes changed since the last judged read, at `now_us` on the
    /// owner's clock. Only a [`Change::Fresh`] field is observed and published; at [`Change::Stale`] the firmware
    /// calls the part unhealthy (a suspended part). A field that is no sample ([`Field::measured`] false, as after a
    /// re-arm) is not judged and not stored: [`Change::Unchanged`], or [`Change::Stale`] once the last real change is
    /// three conversion periods old. The probe's read does not touch the judge.
    pub fn read_fresh<Bus: I2c>(&mut self, bus: &mut Bus, now_us: u64) -> Result<(Change, Field), Fault<Bus::Error>> {
        let mut frame = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[MAG_X_XLSB], &mut frame)
            .map_err(Fault::Bus)?;
        let field = Field::decode(&frame);
        if !field.measured() {
            return Ok((self.changes.hold(now_us, CONVERSION_PERIOD_US), field));
        }
        let mut axes = [0u8; AXIS_BYTES];
        axes.copy_from_slice(&frame[DUMMY_BYTES..DUMMY_BYTES + AXIS_BYTES]);
        Ok((self.changes.judge(&axes, now_us, CONVERSION_PERIOD_US), field))
    }

    /// The on-demand self-test (bmm350.probe): the identity reads back, then one sample is read, accounted (as
    /// plan B's probe reads through `_read`, which feeds the calibration evidence; a field that is no sample is not
    /// accounted, see [`Bmm350::observe`]) and checked with [`not_measuring`]. Ok carries the published sample.
    pub fn probe<Bus: I2c>(&mut self, bus: &mut Bus) -> Result<[i32; 3], ProbeFailure<Bus::Error>> {
        let read = self.identity(bus).map_err(ProbeFailure::Fault)?;
        if read != IDENTITY {
            return Err(ProbeFailure::Fault(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            }));
        }
        let field = self.read(bus).map_err(ProbeFailure::Fault)?;
        let published = self.observe(&field);
        match not_measuring(&field, published) {
            Some(reason) => Err(ProbeFailure::NotMeasuring(reason)),
            None => Ok(published),
        }
    }

    /// Classify a failed setup (bmm350.diagnose), in plan B's words, on the port `port` (`i2c:<port>`). Not the
    /// shared one-byte id read, which on this part reads a dummy byte and would call a healthy sensor the wrong
    /// device.
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus, port: u8) -> String
    where
        Bus::Error: core::fmt::Debug,
    {
        let address = self.address;
        match self.identity(bus) {
            Err(fault) => format!("no ack at i2c:{port} 0x{address:02x} -- {fault}"),
            Ok(read) if read != IDENTITY => {
                format!("wrong device at i2c:{port} 0x{address:02x}: id 0x{read:02x}, expected 0x{IDENTITY:02x}")
            }
            Ok(_) => "present and answering, but setup did not complete".to_string(),
        }
    }

    /// Run the [`CONFIGURATION`] table: the reset, the OTP power-off, the rate and its apply, normal mode.
    fn configure<Bus: I2c>(&self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        for step in CONFIGURATION {
            match step {
                Step::Write { register, value } => bus.write(self.address, &[register, value]).map_err(Fault::Bus)?,
                Step::Wait(milliseconds) => delay.delay_ms(milliseconds),
            }
        }
        Ok(())
    }
}

impl<Bus: I2c> Device<Bus> for Bmm350 {
    type BusError = Bus::Error;

    /// The identity (past the dummy bytes), then the [`CONFIGURATION`] table (bmm350.setup's bus half). The
    /// evidence and the calibration are left as they are: the firmware restores the NVS calibration after setup.
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        self.configure(bus, delay)
    }

    /// The [`CONFIGURATION`] table again: a reset leaves the part SUSPENDED, answering reads with its last sample
    /// forever -- a heading that quietly stops moving, worse than one that stops arriving. The evidence and the
    /// calibration are kept.
    fn rearm(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.configure(bus, delay)
    }
}

#[cfg(test)]
mod tests;
