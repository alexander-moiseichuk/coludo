//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP581 barometer on the SEN0697 (src/glider/drivers/bmp581.py): the BACKUP altitude channel. It publishes
//! pressure (Pa), temperature (degrees C), altitude (m above the sea-level datum) and elevation (m above its own
//! startup ground zero), exactly as the BMP280 it replaced, so nothing downstream learns a new name. `update
//! {"rezero": true}` re-captures the ground zero (after warm-up, just before launch). Unlike the ICP-10111, plan B's
//! BMP581 has no `calibrate`: an operator calibrate sweep answers Task's no-op success for it and never re-zeroes
//! the backup barometer, so only setup and `update` move its ground zero, here as there.
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
//! Units (decision D18): every value is f32, as plan B's float channels and telemetry columns are. The conversions
//! are exact (power-of-two scales over 24-bit counts); the altitude is commons' ISA formula, one definition for every
//! barometer so two of them can only disagree by the air.
//!
//! Not here (firmware glue): the channel push, the telemetry stream, the freshness of the altitude a re-zero takes
//! (the firmware passes it in), the probe's log lines and the inspect JSON.

use control::commons::altitude_m;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::commons::{id_classify, python_float};
use crate::device::{Device, Fault};

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
///  * ODR_CONFIG = 0xBD: deep standby disabled (bit 7), 50 Hz (bits 6:2), normal mode (bits 1:0).
pub const CONFIGURATION: [RegisterWrite; 2] = [(OSR_CONFIG, 0x50), (ODR_CONFIG, 0xBD)];

/// The wait after the configuration for the first normal-mode conversion to complete.
pub const FIRST_CONVERSION_MS: u32 = 50;

/// The burst: temperature then pressure, three bytes each, little-endian.
pub const FRAME_LENGTH: usize = 6;

/// Readings averaged at setup to fix the ground-zero reference (plan B's BMP581 has no calibrate path).
pub const GROUND_SAMPLES: u32 = 8;

/// The wait after each ground-zero reading: the part converts at 50 Hz, so each reading is a fresh conversion.
pub const GROUND_SAMPLE_MS: u32 = 20;

/// Pressure counts per Pa.
pub const PRESSURE_COUNTS_PER_PASCAL: f32 = 64.0;

/// Temperature counts per degree C.
pub const TEMPERATURE_COUNTS_PER_DEGREE: f32 = 65536.0;

/// The default read period, ms (~10 Hz; the part converts faster than it is read).
pub const DEFAULT_PERIOD_MS: u32 = 100;

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

/// Why an operator `update` was refused; nothing has changed when it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// A re-zero with no altitude fresher than three periods. A re-zero LATCHES a number that every later
    /// elevation is reported against, so a stale altitude here would bias the channel for the rest of the flight;
    /// refused out loud so the operator sees why.
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
}

impl Bmp581 {
    /// A driver for the part at `address`; the ground zero is captured at setup.
    pub fn new(address: u8) -> Bmp581 {
        Bmp581 { address, ground: 0.0 }
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

    /// Average [`GROUND_SAMPLES`] readings, [`GROUND_SAMPLE_MS`] apart, into the ground zero (bmp581._ground_zero,
    /// which plan B calls from setup only). Nothing changes when a read fails.
    pub fn capture_ground<Bus: I2c>(
        &mut self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
    ) -> Result<f32, Fault<Bus::Error>> {
        let mut total = 0.0;
        for _ in 0..GROUND_SAMPLES {
            total += self.read(bus)?.altitude;
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
