//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP280 barometer on the SEN0253 (src/glider/drivers/bmp280.py), M18: the backup altitude channel of the v0.1
//! boards (TMS-7C, 7D), where the SEN0697's BMP581 took its place on v1.x. It publishes what the BMP581 does, under
//! the same names: pressure (Pa), temperature (degrees C), altitude (m above the sea-level datum) and elevation (m above
//! its own startup ground zero). `update {"rezero": true}` re-captures the ground zero.
//!
//! THE PART. Unlike the BMP581 it outputs RAW counts: the factory calibration (24 bytes at 0x88: dig_T1..T3,
//! dig_P1..P9, T1 and P1 unsigned, the rest signed, little-endian) is read once at setup, and each conversion goes
//! through Bosch's 64-bit integer compensation, plan B's `_compensate` to the bit ([`Calibration::compensate`]; a
//! degenerate calibration gives 0 Pa, as plan B's). The data frame is 6 bytes from 0xF7, pressure then temperature,
//! each 20 bits big-endian (msb, lsb, xlsb's top nibble). The configuration is plan B's: IIR filter x4 with a 0.5 ms
//! standby (CONFIG 0x08), temperature x1 and pressure x4 oversampling in normal mode (CTRL_MEAS 0x2F), about one
//! conversion every 14 ms. Its power-on mode is SLEEP, so anything that resets it behind the driver's back (the
//! ICP-10111's general-call reset shares i2c:0 with it on v0.1) stops it converting and it returns its last sample
//! forever; [`Device::rearm`] writes the mode again, as plan B's `rearm`.
//!
//! The rest is the BMP581's (crates/drivers bmp581, M9), for one backup barometer to behave as the other: the
//! double-burst guard against bus garbage and the operating range ([`Outcome::Torn`], [`Outcome::Implausible`]), the
//! freshness judge on changed bytes ([`Outcome::Stale`] for a part in SLEEP), the guarded ground capture, the
//! operator's `ground` and `rezero`. The data registers are shadowed for a burst read (the datasheet), so a pair that
//! disagrees is a conversion landing between the two bursts or garbage, never a half-updated frame.
//!
//! Units: f32 throughout, the conversions as plan B's single-precision floats make them (the integer pressure to f32,
//! then the exact division by 256; the integer centidegrees to f32 over 100); the altitude is commons' ISA formula.
//!
//! Not here (firmware glue): the channel push, the telemetry stream, the freshness of the altitude a re-zero takes,
//! the probe's log lines and the inspect JSON.

use core::ops::RangeInclusive;

use control::commons::altitude_m;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::commons::{id_classify, python_float};
use crate::device::{Device, Fault};
use crate::freshness::{Change, Changes, STALE_PERIODS};

/// The default address (SDO low; 0x77 when high).
pub const ADDRESS: u8 = 0x76;

/// CHIP_ID: [`IDENTITY`].
pub const CHIP_ID: u8 = 0xD0;

/// The factory calibration's first register: [`CALIBRATION_LENGTH`] bytes.
pub const CALIBRATION: u8 = 0x88;

/// The calibration's length: dig_T1..T3 and dig_P1..P9, 2 bytes each.
pub const CALIBRATION_LENGTH: usize = 24;

/// CTRL_MEAS: oversampling and mode.
pub const CTRL_MEAS: u8 = 0xF4;

/// CONFIG: standby time and IIR filter.
pub const CONFIG: u8 = 0xF5;

/// The data frame's first register: [`FRAME_LENGTH`] bytes, pressure then temperature.
pub const DATA: u8 = 0xF7;

/// The BMP280's identity.
pub const IDENTITY: u8 = 0x58;

/// One register write: (register, value).
pub type RegisterWrite = (u8, u8);

/// Plan B's configuration, in its order: the IIR filter x4 and a 0.5 ms standby, then temperature x1, pressure x4,
/// normal mode.
pub const CONFIGURATION: [RegisterWrite; 2] = [(CONFIG, 0x08), (CTRL_MEAS, 0x2F)];

/// One conversion period in normal mode, us: 1 + 2 x 1 + 2 x 4 + 0.5 ms typical measuring (13.3 ms at most) and the
/// 0.5 ms standby, rounded up.
pub const CONVERSION_PERIOD_US: u64 = 14_000;

/// The re-zero's window: an altitude older than this is no ground to latch.
pub const REZERO_WINDOW_US: u64 = STALE_PERIODS * CONVERSION_PERIOD_US;

/// The wait after the configuration for the first normal-mode conversion (plan B's 50 ms).
pub const FIRST_CONVERSION_MS: u32 = 50;

/// The data frame's length.
pub const FRAME_LENGTH: usize = 6;

/// Readings averaged into the ground zero (plan B's 8).
pub const GROUND_SAMPLES: u32 = 8;

/// The gap between ground samples (plan B's 20 ms).
pub const GROUND_SAMPLE_MS: u32 = 20;

/// Pairs one ground sample may take before the capture fails ([`Fault::Garbage`]).
pub const GROUND_TRIES: u32 = 3;

/// The part's pressure range (the datasheet's 300..1100 hPa): agreeing bursts outside it are garbage.
pub const PRESSURE_RANGE_PA: RangeInclusive<f32> = 30_000.0..=110_000.0;

/// The part's temperature range.
pub const TEMPERATURE_RANGE_C: RangeInclusive<f32> = -40.0..=85.0;

/// The factory calibration: Bosch's dig_T1..T3 and dig_P1..P9.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Calibration {
    /// dig_T1 (unsigned), dig_T2, dig_T3.
    temperature: [i64; 3],
    /// dig_P1 (unsigned), dig_P2 .. dig_P9.
    pressure: [i64; 9],
}

impl Calibration {
    /// The calibration from its 24 bytes (little-endian words; T1 and P1 unsigned, the rest signed).
    pub fn decode(bytes: &[u8; CALIBRATION_LENGTH]) -> Calibration {
        let word = |index: usize| [bytes[2 * index], bytes[2 * index + 1]];
        let unsigned = |index: usize| i64::from(u16::from_le_bytes(word(index)));
        let signed = |index: usize| i64::from(i16::from_le_bytes(word(index)));
        Calibration {
            temperature: [unsigned(0), signed(1), signed(2)],
            pressure: [
                unsigned(3),
                signed(4),
                signed(5),
                signed(6),
                signed(7),
                signed(8),
                signed(9),
                signed(10),
                signed(11),
            ],
        }
    }

    /// Bosch's 64-bit integer compensation (plan B's `_compensate`, bit for bit): raw counts to (Pa, degrees C).
    /// The pressure is 0 when the calibration is degenerate (its divisor 0), as plan B's.
    pub fn compensate(&self, adc_temperature: i32, adc_pressure: i32) -> (f32, f32) {
        let [t1, t2, t3] = self.temperature;
        let [p1, p2, p3, p4, p5, p6, p7, p8, p9] = self.pressure;
        let adc_t = i64::from(adc_temperature);
        let first = (((adc_t >> 3) - (t1 << 1)) * t2) >> 11;
        let second = (((((adc_t >> 4) - t1) * ((adc_t >> 4) - t1)) >> 12) * t3) >> 14;
        let fine = first + second;
        let temperature = ((fine * 5 + 128) >> 8) as f32 / 100.0;
        let mut var1 = fine - 128_000;
        let mut var2 = var1 * var1 * p6;
        var2 += (var1 * p5) << 17;
        var2 += p4 << 35;
        var1 = ((var1 * var1 * p3) >> 8) + ((var1 * p2) << 12);
        var1 = (((1i64 << 47) + var1) * p1) >> 33;
        if var1 == 0 {
            return (0.0, temperature);
        }
        let mut pressure = 1_048_576 - i64::from(adc_pressure);
        pressure = floor_div(((pressure << 31) - var2) * 3125, var1);
        let var1 = (p9 * (pressure >> 13) * (pressure >> 13)) >> 25;
        let var2 = (p8 * pressure) >> 19;
        pressure = ((pressure + var1 + var2) >> 8) + (p7 << 4);
        (pressure as f32 / 256.0, temperature)
    }
}

/// One conversion, in the units the channels and the telemetry row carry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// Metres above the sea-level datum (commons.altitude_m of the pressure).
    pub altitude: f32,
    /// Degrees C.
    pub temperature: f32,
    /// Pa.
    pub pressure: f32,
}

impl Reading {
    /// Decode the 6-byte frame (bmp280._read): pressure then temperature, 20 bits each, big-endian, compensated.
    pub fn decode(frame: &[u8; FRAME_LENGTH], calibration: &Calibration) -> Reading {
        let raw = |at: usize| {
            (i32::from(frame[at]) << 12) | (i32::from(frame[at + 1]) << 4) | (i32::from(frame[at + 2]) >> 4)
        };
        let (pressure, temperature) = calibration.compensate(raw(3), raw(0));
        Reading {
            altitude: altitude_m(pressure),
            temperature,
            pressure,
        }
    }
}

/// What one [`Bmp280::read_fresh`] yields (as the BMP581's [`crate::bmp581::Outcome`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    /// Both bursts agree, in range, changed since the last judged read: a new conversion, publish it.
    Fresh(Reading),
    /// Both agree, in range, the last judged bytes again: no conversion since.
    Unchanged,
    /// Both agree, in range, the same bytes for [`crate::freshness::STALE_PERIODS`] periods: a part in SLEEP.
    Stale,
    /// The two bursts differ: garbage, or a conversion between them. Not judged; the next tick reads again.
    Torn,
    /// Both agree outside [`PRESSURE_RANGE_PA`] or [`TEMPERATURE_RANGE_C`]: garbage that repeated. Not judged.
    Implausible(Reading),
}

/// One pair of bursts, before any judging.
enum Pair {
    /// Both alike and in range: the burst and its reading.
    Agreed([u8; FRAME_LENGTH], Reading),
    /// The bursts differ.
    Torn,
    /// Both alike, out of range: the reading.
    Implausible(Reading),
}

/// Why an operator `update` was refused (as the BMP581's).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// A re-zero with no altitude fresher than [`REZERO_WINDOW_US`].
    NoFreshAltitude,
    /// `ground` is not a number [`python_float`] reads.
    NotANumber,
}

impl core::fmt::Display for Refusal {
    /// The refusal text: plan B's words for the stale re-zero, the port's own for a ground that is no number.
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::NoFreshAltitude => write!(formatter, "no fresh altitude to re-zero from"),
            Refusal::NotANumber => write!(formatter, "ground must be a number"),
        }
    }
}

/// One BMP280 on an I2C bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Bmp280 {
    /// The 7-bit address.
    address: u8,
    /// The factory calibration, read at setup.
    calibration: Calibration,
    /// The ground zero, m above the datum. Set at setup.
    ground: f32,
    /// The data bytes of the last [`Bmp280::read_fresh`] and when they last changed.
    changes: Changes<FRAME_LENGTH>,
}

impl Bmp280 {
    /// A driver for the part at `address`; the calibration and the ground zero are read at setup.
    pub fn new(address: u8) -> Bmp280 {
        Bmp280 {
            address,
            calibration: Calibration::default(),
            ground: 0.0,
            changes: Changes::default(),
        }
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// The ground zero, m above the datum.
    pub fn ground(&self) -> f32 {
        self.ground
    }

    /// Metres above the ground zero: the `elevation` channel.
    pub fn elevation(&self, reading: &Reading) -> f32 {
        reading.altitude - self.ground
    }

    /// Read CHIP_ID.
    pub fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>> {
        let mut identity = [0u8; 1];
        bus.write_read(self.address, &[CHIP_ID], &mut identity)
            .map_err(Fault::Bus)?;
        Ok(identity[0])
    }

    /// Read one conversion.
    pub fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>> {
        let mut frame = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[DATA], &mut frame).map_err(Fault::Bus)?;
        Ok(Reading::decode(&frame, &self.calibration))
    }

    /// Read one conversion and what it is ([`Outcome`]): the frame twice back to back, believed when both agree in
    /// range, then judged by its changed bytes at `now_us` (the BMP581's `read_fresh`).
    pub fn read_fresh<Bus: I2c>(&mut self, bus: &mut Bus, now_us: u64) -> Result<Outcome, Fault<Bus::Error>> {
        Ok(match self.read_pair(bus)? {
            Pair::Torn => Outcome::Torn,
            Pair::Implausible(reading) => Outcome::Implausible(reading),
            Pair::Agreed(frame, reading) => match self.changes.judge(&frame, now_us, CONVERSION_PERIOD_US) {
                Change::Fresh => Outcome::Fresh(reading),
                Change::Unchanged => Outcome::Unchanged,
                Change::Stale => Outcome::Stale,
            },
        })
    }

    /// The guard: the frame twice, believed only when both agree and lie in the part's ranges. Judges nothing.
    fn read_pair<Bus: I2c>(&self, bus: &mut Bus) -> Result<Pair, Fault<Bus::Error>> {
        let mut first = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[DATA], &mut first).map_err(Fault::Bus)?;
        let mut second = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[DATA], &mut second).map_err(Fault::Bus)?;
        if first != second {
            return Ok(Pair::Torn);
        }
        let reading = Reading::decode(&first, &self.calibration);
        if !PRESSURE_RANGE_PA.contains(&reading.pressure) || !TEMPERATURE_RANGE_C.contains(&reading.temperature) {
            return Ok(Pair::Implausible(reading));
        }
        Ok(Pair::Agreed(first, reading))
    }

    /// One guarded ground sample: up to [`GROUND_TRIES`] pairs; [`Fault::Garbage`] when none agrees in range.
    fn ground_sample<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>> {
        for _ in 0..GROUND_TRIES {
            if let Pair::Agreed(_, reading) = self.read_pair(bus)? {
                return Ok(reading);
            }
        }
        Err(Fault::Garbage)
    }

    /// Average [`GROUND_SAMPLES`] guarded readings, [`GROUND_SAMPLE_MS`] apart, into the ground zero (plan B's
    /// `_ground_zero`, and the port's calibrate as the BMP581's). Nothing changes when a sample fails.
    pub fn capture_ground<Bus: I2c>(
        &mut self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
    ) -> Result<f32, Fault<Bus::Error>> {
        let mut total = 0.0;
        for _ in 0..GROUND_SAMPLES {
            total += self.ground_sample(bus)?.altitude;
            delay.delay_ms(GROUND_SAMPLE_MS);
        }
        self.ground = total / GROUND_SAMPLES as f32;
        Ok(self.ground)
    }

    /// Apply an operator property change (bmp280.update): `{"ground": <m>}` sets the zero, `{"rezero": true}`
    /// takes `fresh_altitude` (None: [`Refusal::NoFreshAltitude`]). Returns the changed property names.
    pub fn update(&mut self, properties: &Value, fresh_altitude: Option<f32>) -> Result<Vec<&'static str>, Refusal> {
        if let Some(ground) = properties.get("ground") {
            self.ground = python_float(ground).ok_or(Refusal::NotANumber)?;
            return Ok(vec!["ground"]);
        }
        if properties.get("rezero").is_some_and(truthy) {
            self.ground = fresh_altitude.ok_or(Refusal::NoFreshAltitude)?;
            return Ok(vec!["ground"]);
        }
        Ok(Vec::new())
    }

    /// The on-demand self-test (bmp280.probe): the identity reads back, then one conversion reads.
    pub fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>> {
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        self.read(bus)
    }

    /// Classify a failed setup at the wire level (bmp280.diagnose through the one-byte id read).
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String {
        id_classify(self.identity(bus).ok(), IDENTITY)
    }

    /// The filter and the normal mode (plan B's setup and rearm writes).
    fn configure<Bus: I2c>(&self, bus: &mut Bus) -> Result<(), Fault<Bus::Error>> {
        for (register, value) in CONFIGURATION {
            bus.write(self.address, &[register, value]).map_err(Fault::Bus)?;
        }
        Ok(())
    }
}

/// Python's `//`: the quotient floored (Rust's `/` truncates toward zero; Bosch's C does too, so on a calibration
/// that turned the divisor negative the two would part, and plan B's floor is the reference).
fn floor_div(numerator: i64, divisor: i64) -> i64 {
    let quotient = numerator / divisor;
    if numerator % divisor != 0 && (numerator < 0) != (divisor < 0) {
        quotient - 1
    } else {
        quotient
    }
}

/// Python's truthiness of a JSON value, for the `rezero` flag.
fn truthy(value: &Value) -> bool {
    match value {
        Value::Null => false,
        Value::Bool(flag) => *flag,
        Value::Number(number) => number.as_f64().is_some_and(|number| number != 0.0),
        Value::String(text) => !text.is_empty(),
        Value::Array(items) => !items.is_empty(),
        Value::Object(members) => !members.is_empty(),
    }
}

impl<Bus: I2c> Device<Bus> for Bmp280 {
    type BusError = Bus::Error;

    /// The identity, the factory calibration, the configuration, the first conversion's wait, then the ground zero
    /// (bmp280.setup's bus half). Any failed read fails setup, as plan B's try around it does.
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        let mut calibration = [0u8; CALIBRATION_LENGTH];
        bus.write_read(self.address, &[CALIBRATION], &mut calibration)
            .map_err(Fault::Bus)?;
        self.calibration = Calibration::decode(&calibration);
        self.configure(bus)?;
        delay.delay_ms(FIRST_CONVERSION_MS);
        self.capture_ground(bus, delay)?;
        Ok(())
    }

    /// Write the filter and the mode again (out of the SLEEP a reset leaves it in); the calibration and the ground
    /// are kept.
    fn rearm(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.configure(bus)
    }
}

#[cfg(test)]
mod tests;
