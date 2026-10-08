//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP581 barometer on the SEN0697 (src/glider/drivers/bmp581.py): the BACKUP altitude channel. It publishes
//! pressure (Pa), temperature (degrees C), altitude (m above the sea-level datum) and elevation (m above its own
//! startup ground zero), exactly as the BMP280 it replaced, so nothing downstream learns a new name. `update
//! {"rezero": true}` re-captures the ground zero (after warm-up, just before launch).
//!
//! THE RE-ZERO (avionics-port.md M9 D7, a divergence). Plan B's BMP581 had no `calibrate`: an operator calibrate
//! sweep answered Task's no-op success for it and never re-zeroed the backup barometer, so the two barometers' zeros
//! split at the first re-zero. The port gives it the ICP-10111's surface, and the firmware runs it wherever it runs
//! the ICP's: `calibrate` re-captures the ground from [`GROUND_SAMPLES`] new readings ([`Bmp581::capture_ground`],
//! setup's own average), and so does the DONE recovery; `update {"ground": <m>}` sets it (M8's warm start rebases it
//! to the crumb's pad); `update {"rezero": true}` takes the channel's own altitude when fresher than
//! [`REZERO_WINDOW_US`].
//!
//! Simpler than the BMP280 in the way that matters: the part outputs COMPENSATED values, so there is no factory
//! calibration to read and no compensation polynomial to carry. Pressure is a 24-bit unsigned count of 1/64 Pa;
//! temperature a 24-bit two's-complement count of 1/65536 degrees C. Reading a sub-zero temperature as unsigned
//! yields ~+256 degrees C, the kind of wrong that still renders as a plausible number on a chart.
//!
//! Register values are Bosch's own (bmp5_defs.h), and the configuration is the one the TMS-7 nose logger flew:
//! pressure enabled at 4x oversampling, 50 Hz, normal mode. The part powers on in STANDBY, so anything that resets
//! it behind the driver's back stops it converting and it returns its last sample forever -- an altitude that
//! quietly stops moving. [`Device::rearm`] writes the mode again: on v1.1 the ICP's general call cannot reach it
//! (the two barometers are on different buses, much of the redundancy's value), but on v0.1 the SEN0697 shares i2c:0
//! with the ICP, and a later rewire must not silently reintroduce the failure.
//!
//! NEW CONVERSIONS ONLY (avionics-port.md M9 D2, D3). The firmware polls the part on every owner tick (100 Hz) and
//! [`Bmp581::read_fresh`] judges each read by its 6 data bytes ([`crate::freshness::Changes`]): fresh when they
//! changed, which every conversion does (its noise is tens of counts), so each 50 Hz conversion is published once.
//! No data-ready register is read: the BMP581's needs INT_SOURCE written, a register plan B never wrote. A part in
//! standby repeats its bytes, and once it has for [`crate::freshness::STALE_PERIODS`] conversion periods the read
//! is [`Outcome::Stale`], where the firmware calls it unhealthy. Plan B's `period_ms` is unused.
//!
//! BUS GARBAGE (measured on the taster and 7F, M9 stage 4): about 0.1 % of the 6-byte bursts at 50 Hz on every tick
//! are isolated single transfers that are not the part's sample -- all 0xFF (262 144 Pa), one byte repeated
//! (0x1B1B1B, 27 756 Pa), or tears that look plausible (94 577 Pa at a normal 27.7 C; 103 829 Pa at 101 C; 192 100
//! Pa at -44 C); a direct probe read `fffffffeffff` once in 2560. Changed bytes call every one of them fresh, and
//! each became a +-9000 m elevation spike and a +-5000 m/s `change`. So `read_fresh` reads the burst TWICE back to
//! back and believes it only when both are byte-identical ([`Outcome::Torn`] otherwise), then only inside the part's
//! operating range ([`Outcome::Implausible`] otherwise, for garbage that repeats). Neither is judged: the stored bytes
//! and the stale clock stay as they were, and the next tick reads again. A conversion that lands between the two
//! bursts tears the pair too, about one read in 80 (one burst takes ~0.25 ms at i2c:0's 400 kHz, against the 20 ms
//! period), so most torn pairs are that, not garbage; the next tick,
//! 10 ms later, reads that conversion twice, so none is lost, only delayed by a tick. The ground capture (setup, a
//! calibrate, the DONE re-zero) takes each of its samples through the same guard, which a latched ground needs most:
//! one garbage burst in plan B's single reads moved the 8-sample ground by about 1.1 km until the next re-zero. A
//! sample whose pair is torn or implausible is read again, up to [`GROUND_TRIES`] pairs, and a capture that runs out
//! fails ([`Fault::Garbage`]) and keeps the ground it had. The guard never touches the freshness judge.
//!
//! Units (decision D18): every value is f32, as plan B's float channels and telemetry columns are. The conversions
//! are exact (power-of-two scales over 24-bit counts); the altitude is commons' ISA formula, one definition for every
//! barometer so two of them can only disagree by the air.
//!
//! Not here (firmware glue): the channel push, the telemetry stream, the freshness of the altitude a re-zero takes
//! (the firmware passes it in), the probe's log lines and the inspect JSON.

use core::ops::RangeInclusive;

use control::commons::altitude_m;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::commons::{id_classify, python_float};
use crate::device::{Device, Fault};
use crate::freshness::{Change, Changes, STALE_PERIODS};

/// The default I2C address (0x46 when the address pin is pulled the other way).
pub const ADDRESS: u8 = 0x47;

/// CHIP_ID: the identity register.
pub const CHIP_ID: u8 = 0x01;

/// TEMP_DATA_XLSB: temperature xlsb, lsb, msb then pressure xlsb, lsb, msb, six contiguous bytes.
pub const TEMP_DATA_XLSB: u8 = 0x1D;

/// OSR_CONFIG: the oversampling and the pressure enable.
pub const OSR_CONFIG: u8 = 0x36;

/// ODR_CONFIG: the output data rate, the power mode and deep standby.
pub const ODR_CONFIG: u8 = 0x37;

/// What CHIP_ID reads on a BMP581.
pub const IDENTITY: u8 = 0x50;

/// One register write of a bring-up table: (register, value).
pub type RegisterWrite = (u8, u8);

/// The configuration, in plan B's order (the mode the re-arm writes again):
///  * OSR_CONFIG = 0x50: press_en (bit 6), osr_p = 4x (bits 5:3), osr_t = 1x;
///  * ODR_CONFIG = 0xBD: deep standby disabled (bit 7), 50 Hz (bits 6:2 = 0x0F, the 16th of Bosch's ODR table from
///    240 Hz down, as Linux's bmp580 driver numbers it), normal mode (bits 1:0 = 01): already the 50 Hz D2 asks for.
pub const CONFIGURATION: [RegisterWrite; 2] = [(OSR_CONFIG, 0x50), (ODR_CONFIG, 0xBD)];

/// The conversion period at the configured 50 Hz, microseconds.
pub const CONVERSION_PERIOD_US: u64 = 20_000;

/// How fresh the altitude a `rezero` takes must be: [`STALE_PERIODS`] conversion periods (plan B's three periods,
/// at the rate the part now converts and is read).
pub const REZERO_WINDOW_US: u64 = STALE_PERIODS * CONVERSION_PERIOD_US;

/// The wait after the configuration for the first normal-mode conversion to complete.
pub const FIRST_CONVERSION_MS: u32 = 50;

/// The burst: temperature then pressure, three bytes each, little-endian.
pub const FRAME_LENGTH: usize = 6;

/// Readings averaged into the ground-zero reference, at setup and at a calibrate.
pub const GROUND_SAMPLES: u32 = 8;

/// The wait after each ground-zero reading: the part converts at 50 Hz, so each reading is a fresh conversion.
pub const GROUND_SAMPLE_MS: u32 = 20;

/// The pairs of bursts one ground sample may read before the capture fails ([`Fault::Garbage`]): a torn or
/// implausible pair is read again at once. Garbage is about one burst in a thousand and a conversion tears about one
/// pair in 80, so three in a row is a bus that has stopped answering the part.
pub const GROUND_TRIES: u32 = 3;

/// Pressure counts per Pa.
pub const PRESSURE_COUNTS_PER_PASCAL: f32 = 64.0;

/// Temperature counts per degree C.
pub const TEMPERATURE_COUNTS_PER_DEGREE: f32 = 65536.0;

/// The part's operating pressure range, Pa (datasheet: 300 to 1250 hPa): a burst read twice alike outside it is
/// garbage that repeated, never a sample ([`Outcome::Implausible`]).
pub const PRESSURE_RANGE_PA: RangeInclusive<f32> = 30_000.0..=125_000.0;

/// The part's operating temperature range, degrees C (datasheet: -40 to 85 C), as [`PRESSURE_RANGE_PA`].
pub const TEMPERATURE_RANGE_C: RangeInclusive<f32> = -40.0..=85.0;

/// The sign bit of a 24-bit two's-complement count.
const SIGN_24: i32 = 0x80_0000;

/// One more than the largest 24-bit count: what a negative count's raw value is offset by.
const RANGE_24: i32 = 0x100_0000;

/// One conversion, in the units the channels and the telemetry row carry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// Metres above the sea-level datum (commons.altitude_m of the pressure; 0 for a dead read of 0 Pa).
    pub altitude: f32,
    /// Degrees C.
    pub temperature: f32,
    /// Pa.
    pub pressure: f32,
}

impl Reading {
    /// Decode the 6-byte burst (bmp581._read): temperature 24-bit two's complement, pressure 24-bit unsigned, both
    /// little-endian.
    pub fn decode(frame: &[u8; FRAME_LENGTH]) -> Reading {
        let raw = |at: usize| i32::from(frame[at]) | (i32::from(frame[at + 1]) << 8) | (i32::from(frame[at + 2]) << 16);
        let mut temperature = raw(0);
        if temperature & SIGN_24 != 0 {
            temperature -= RANGE_24;
        }
        let pressure = raw(3) as f32 / PRESSURE_COUNTS_PER_PASCAL;
        Reading {
            altitude: altitude_m(pressure),
            temperature: temperature as f32 / TEMPERATURE_COUNTS_PER_DEGREE,
            pressure,
        }
    }
}

/// What one [`Bmp581::read_fresh`] yields: the double burst's verdict, with the reading when there is one to use.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    /// Both bursts agree, the reading is in range and its bytes changed since the last judged read: a new
    /// conversion, publish it.
    Fresh(Reading),
    /// Both bursts agree, in range, and repeat the last judged read's bytes: no conversion since, normal on a part
    /// polled twice a conversion.
    Unchanged,
    /// Both bursts agree, in range, and have repeated the last judged bytes for
    /// [`crate::freshness::STALE_PERIODS`] conversion periods or longer: a part in standby. The firmware calls the
    /// part unhealthy.
    Stale,
    /// The two bursts differ: bus garbage in one of them, or a conversion that landed between them. Not judged:
    /// the stored bytes and the stale clock are untouched, and the next tick reads again.
    Torn,
    /// Both bursts agree, but the reading lies outside [`PRESSURE_RANGE_PA`] or [`TEMPERATURE_RANGE_C`]: garbage
    /// that repeated. Not judged, as [`Outcome::Torn`]; the reading is carried for the firmware's log line only,
    /// never to publish.
    Implausible(Reading),
}

/// One pair of bursts read back to back, before any judging: what the guard makes of it.
enum Pair {
    /// Both bursts alike and in the part's operating range: the burst and its reading.
    Agreed([u8; FRAME_LENGTH], Reading),
    /// The bursts differ.
    Torn,
    /// Both alike, outside the operating range: the reading.
    Implausible(Reading),
}

/// Why an operator `update` was refused; nothing has changed when it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// A re-zero with no altitude fresher than three periods ([`REZERO_WINDOW_US`]). A re-zero LATCHES a number that
    /// every later elevation is reported against, so a stale altitude here would bias the channel for the rest of the
    /// flight; refused out loud so the operator sees why.
    NoFreshAltitude,
    /// `ground` is not a number [`python_float`] reads: plan B's `float()` raised. Plan B answered with the
    /// interpreter's own text for a string, and with a TypeError its `update` command did not catch for a null, a
    /// list or an object; the port refuses them all alike, in its own words.
    NotANumber,
}

impl core::fmt::Display for Refusal {
    /// The refusal text: plan B's words for [`Refusal::NoFreshAltitude`]; the port's own for
    /// [`Refusal::NotANumber`], where plan B's were the interpreter's (see the variant).
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::NoFreshAltitude => write!(formatter, "no fresh altitude to re-zero from"),
            Refusal::NotANumber => write!(formatter, "ground must be a number"),
        }
    }
}

/// One BMP581 on an I2C bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Bmp581 {
    /// The 7-bit address.
    address: u8,
    /// The ground zero, m above the datum: the per-sensor reference elevation is measured from. Set at setup.
    ground: f32,
    /// The data bytes of the last [`Bmp581::read_fresh`] and when they last changed.
    changes: Changes<FRAME_LENGTH>,
}

impl Bmp581 {
    /// A driver for the part at `address`; the ground zero is captured at setup.
    pub fn new(address: u8) -> Bmp581 {
        Bmp581 {
            address,
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

    /// Metres above the ground zero: the `elevation` channel, offset-free per sensor.
    pub fn elevation(&self, reading: &Reading) -> f32 {
        reading.altitude - self.ground
    }

    /// Read CHIP_ID: one byte, no dummy bytes on this part.
    pub fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>> {
        let mut identity = [0u8; 1];
        bus.write_read(self.address, &[CHIP_ID], &mut identity)
            .map_err(Fault::Bus)?;
        Ok(identity[0])
    }

    /// Read one conversion.
    pub fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>> {
        let mut frame = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[TEMP_DATA_XLSB], &mut frame)
            .map_err(Fault::Bus)?;
        Ok(Reading::decode(&frame))
    }

    /// Read one conversion and what it is ([`Outcome`]): the 6-byte burst TWICE back to back (see the module notes).
    /// Two bursts that differ are [`Outcome::Torn`]; two alike outside the operating range are
    /// [`Outcome::Implausible`]; neither is judged. Otherwise the bytes are judged by whether they changed since the
    /// last judged read, at `now_us` on the owner's clock: only an [`Outcome::Fresh`] reading is published, and at
    /// [`Outcome::Stale`] the firmware calls the part unhealthy (a part in standby). A bus fault on either burst is
    /// the transport's, and judges nothing. Setup's and a calibrate's own reads ([`Bmp581::read`]) do not touch the
    /// judge.
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

    /// The guard (see the module notes): the 6-byte burst twice back to back, believed only when both agree and the
    /// reading lies in [`PRESSURE_RANGE_PA`] and [`TEMPERATURE_RANGE_C`]. Judges nothing.
    fn read_pair<Bus: I2c>(&self, bus: &mut Bus) -> Result<Pair, Fault<Bus::Error>> {
        let mut first = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[TEMP_DATA_XLSB], &mut first)
            .map_err(Fault::Bus)?;
        let mut second = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[TEMP_DATA_XLSB], &mut second)
            .map_err(Fault::Bus)?;
        if first != second {
            return Ok(Pair::Torn);
        }
        let reading = Reading::decode(&first);
        if !PRESSURE_RANGE_PA.contains(&reading.pressure) || !TEMPERATURE_RANGE_C.contains(&reading.temperature) {
            return Ok(Pair::Implausible(reading));
        }
        Ok(Pair::Agreed(first, reading))
    }

    /// One guarded ground sample: up to [`GROUND_TRIES`] pairs, the first that agrees in range taken;
    /// [`Fault::Garbage`] when none does.
    fn ground_sample<Bus: I2c>(&self, bus: &mut Bus) -> Result<Reading, Fault<Bus::Error>> {
        for _ in 0..GROUND_TRIES {
            if let Pair::Agreed(_, reading) = self.read_pair(bus)? {
                return Ok(reading);
            }
        }
        Err(Fault::Garbage)
    }

    /// Average [`GROUND_SAMPLES`] readings, [`GROUND_SAMPLE_MS`] apart, into the ground zero (bmp581._ground_zero,
    /// which plan B called from setup only; the port's `calibrate` and DONE recovery too, D7). Each sample goes through
    /// the garbage guard, a torn or implausible pair read again up to [`GROUND_TRIES`] pairs (the module notes; plan B
    /// read each sample once, the golden fixture's listed divergence). Nothing changes when a read fails or a sample
    /// runs out of tries ([`Fault::Garbage`]): setup fails its attempt, a calibrate reports it, and the ground in force
    /// stays. The freshness judge is not touched.
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

    /// Apply an operator property change (bmp581.update): `{"ground": <m>}` SETS the zero directly -- the warm
    /// start rebases a rebooted barometer to the crumb's pad altitude, because a mid-air re-zero would make elevation
    /// read ~0 at altitude and fake an immediate landing. `{"rezero": true}` re-captures it from `fresh_altitude`,
    /// the channel's own value when fresher than three periods (None otherwise: [`Refusal::NoFreshAltitude`]).
    ///
    /// Returns the changed property names: `["ground"]` on a set or a re-zero, `[]` otherwise. `ground` wins when
    /// both are given, as in plan B, and is read as plan B's `float()` reads it ([`python_float`]): a number, a
    /// numeral string such as "12.5", or a bool; anything else is [`Refusal::NotANumber`].
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

    /// The on-demand self-test (bmp581.probe): the identity reads back, then one conversion reads.
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

    /// Classify a failed setup at the wire level (bmp581.diagnose through the shared one-byte id read; correct on
    /// this part, which has no dummy bytes): sorted with [`id_classify`].
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String {
        id_classify(self.identity(bus).ok(), IDENTITY)
    }

    /// Enable pressure at 4x oversampling and start 50 Hz normal-mode conversion (bmp581._configure).
    fn configure<Bus: I2c>(&self, bus: &mut Bus) -> Result<(), Fault<Bus::Error>> {
        for (register, value) in CONFIGURATION {
            bus.write(self.address, &[register, value]).map_err(Fault::Bus)?;
        }
        Ok(())
    }
}

/// Python's truthiness of a JSON value, for the `rezero` flag: false, null, 0, "" and empty containers are false.
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

impl<Bus: I2c> Device<Bus> for Bmp581 {
    type BusError = Bus::Error;

    /// The identity, the configuration, the first conversion's wait, then the ground zero (bmp581.setup's bus half).
    /// A failed read while averaging fails setup, as plan B's try around it does.
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        self.configure(bus)?;
        delay.delay_ms(FIRST_CONVERSION_MS);
        self.capture_ground(bus, delay)?;
        Ok(())
    }

    /// Write the mode again (out of the STANDBY a reset leaves it in); the ground zero is kept.
    fn rearm(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.configure(bus)
    }
}

#[cfg(test)]
mod tests;
