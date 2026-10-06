//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The INA226 high-side current, voltage and power monitor (src/glider/drivers/ina226.py): the battery (or 5 V)
//! supply line, for consumption tracking and the servo probe's closed-loop verdict. [`Device::setup`] verifies the
//! die id, programs the conversion configuration, and writes the calibration computed from the shunt resistance and
//! the expected full-scale current (the only board-specific numbers); [`Ina226::read`] polls the bus voltage,
//! current and power.
//!
//! The part measures the SHUNT VOLTAGE directly (2.5 uV/LSB), so the absolute accuracy comes from the CALIBRATION
//! register, not a precise resistor: Current_LSB = max_current / 2^15, CAL = 0.00512 / (Current_LSB x shunt). To
//! trust the watt-hours, calibrate `shunt_mohms` against a KNOWN current once and back out the effective value: a
//! 2-wire ohmmeter cannot resolve a 0.01 ohm shunt. Gotcha from the bench: the module needs VBS jumpered to IN-, or
//! the voltage and power read 0.
//!
//! INTEGER milli-units, exactly plan B's: the readings are mV, mA and mW, which are also the telemetry units, so no
//! float is involved anywhere (decision D18 asks for none here). From the datasheet:
//!  * bus voltage: 1.25 mV/LSB -> mV = raw x 5 // 4;
//!  * current: Current_LSB = max / 2^15 -> mA = raw x max_current_ma // 32768 (signed: a reversed shunt or a charging
//!    current reads negative);
//!  * power: Power_LSB = 25 x Current_LSB -> mW = raw x max_current_ma // 32768 x 25 (divided BEFORE the x 25, as
//!    plan B does it, so the digits match);
//!  * calibration = 0.00512 x 2^15 x 1e6 // (max_current_ma x shunt_mohms) = 167772160 // (mA x mOhm).
//!
//! THREE SEPARATE READS, DELIBERATELY: the register pointer does NOT auto-increment. Probed on the board, a 6-byte
//! read from 0x02 returns the bus voltage and then 0xFFFF padding, not power and current. Each register needs its own
//! pointer write. Do not "optimise" this back into one read.
//!
//! The re-arm restores CONFIGURATION, CALIBRATION, and with the hardware alert armed ALERT LIMIT and MASK/ENABLE: a
//! general call brings the part back with CALIBRATION = 0, which makes every current and power reading wrong rather
//! than absent, and with the alert disarmed. Plan B restored the first two only (avionics-port.md §6).
//!
//! Not here (firmware glue): the ALERT GPIO and its interrupt counter (the `alerts` column), the log-once of new
//! over-current trips, the channel push, the probe's log lines and the inspect JSON.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::commons::id_classify;
use crate::device::{Device, Fault};

/// The default I2C address (A1 = A0 = GND).
pub const ADDRESS: u8 = 0x40;

/// Configuration register.
pub const REGISTER_CONFIGURATION: u8 = 0x00;

/// Bus voltage register: u16, 1.25 mV/LSB.
pub const REGISTER_BUS_VOLTAGE: u8 = 0x02;

/// Power register: u16, Power_LSB = 25 x Current_LSB.
pub const REGISTER_POWER: u8 = 0x03;

/// Current register: s16, Current_LSB per bit.
pub const REGISTER_CURRENT: u8 = 0x04;

/// Calibration register.
pub const REGISTER_CALIBRATION: u8 = 0x05;

/// Mask/Enable register: which condition drives ALERT, plus the read-back flags.
pub const REGISTER_MASK_ENABLE: u8 = 0x06;

/// Alert limit register: the trip, in the selected condition's LSBs.
pub const REGISTER_ALERT_LIMIT: u8 = 0x07;

/// Die id register.
pub const REGISTER_DIE_ID: u8 = 0xFF;

/// The INA226's die id.
pub const DIE_ID: u16 = 0x2260;

/// The configuration plan B writes: continuous shunt and bus, 4-sample average, 1.1 ms conversions (~9 ms an
/// update).
pub const CONFIGURATION: u16 = 0x4327;

/// Mask/Enable SOL: ALERT on Shunt Over-Voltage, which is over-current independent of the rail voltage. Transient
/// (the latch bit LEN stays clear), so the line drops again with the current.
pub const MASK_ENABLE_SOL: u16 = 0x8000;

/// The calibration numerator for milli-unit inputs: 0.00512 x 2^15 x 1e6.
pub const CALIBRATION_NUMERATOR: i128 = 167_772_160;

/// Power_LSB / Current_LSB.
pub const POWER_LSB_RATIO: i128 = 25;

/// The current register's full scale, 2^15.
const CURRENT_FULL_SCALE: i128 = 32_768;

/// The default shunt, milliohms.
pub const DEFAULT_SHUNT_MOHMS: i64 = 10;

/// The default full-scale current, mA.
pub const DEFAULT_MAX_CURRENT_MA: i64 = 5_000;

/// The default over-current trip, mA (with an ALERT pin wired).
pub const DEFAULT_ALERT_MA: i64 = 3_000;

/// The default read period, ms.
pub const DEFAULT_PERIOD_MS: u32 = 100;

/// The CALIBRATION register value for a full-scale current and a shunt (plan B's `_CAL_NUM // (mA x mOhm)`).
///
/// None where plan B's setup failed: a zero product (its ZeroDivisionError) or a quotient outside the 16-bit
/// register (its `struct.pack('>H')` error) -- every negative product, whose floored quotient is negative. A
/// quotient of 0 (a product past the numerator) fits and is written, as plan B writes it; every current and power
/// then reads 0. The arithmetic is i128 because plan B's integers do not overflow: two i64 config values multiply
/// past i64, and the quotient is then a valid 0.
pub fn calibration(max_current_ma: i64, shunt_mohms: i64) -> Option<u16> {
    let product = i128::from(max_current_ma) * i128::from(shunt_mohms);
    if product <= 0 {
        return None;
    }
    u16::try_from(CALIBRATION_NUMERATOR / product).ok()
}

/// The ALERT LIMIT register value for an over-current trip: mA x mOhm -> shunt-voltage LSBs of 2.5 uV, that is
/// `alert_ma x shunt_mohms x 2 // 5` (floored, in i128 for the reason [`calibration`] gives). None when it does
/// not fit the register, a negative trip included (plan B then fell back to polling only).
pub fn alert_limit(alert_ma: i64, shunt_mohms: i64) -> Option<u16> {
    let product = i128::from(alert_ma) * i128::from(shunt_mohms) * 2;
    u16::try_from(product.div_euclid(5)).ok()
}

/// One reading, in the integer milli-units the channels and the telemetry row carry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measurement {
    /// Bus voltage, mV.
    pub voltage_mv: i32,
    /// Current, mA; signed (a reversed shunt or a charging current reads negative).
    pub current_ma: i32,
    /// Power, mW; the part reports it unsigned.
    pub power_mw: i32,
}

/// The INA226's settings, from its config block (the keys plan B's setup() reads, with its defaults).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// The I2C address (`addr`, default [`ADDRESS`]).
    pub address: u8,
    /// The read period, ms (`period_ms`, default [`DEFAULT_PERIOD_MS`]).
    pub period_ms: u32,
    /// The shunt, milliohms (`shunt_mohms`, default [`DEFAULT_SHUNT_MOHMS`]).
    pub shunt_mohms: i64,
    /// The full-scale current, mA (`max_current_ma`, default [`DEFAULT_MAX_CURRENT_MA`]).
    pub max_current_ma: i64,
    /// Whether `shunt_mohms` and `max_current_ma` are both integers, or absent and defaulted. False when either
    /// is present as anything else: setup then fails at the CALIBRATION, as plan B's did (see
    /// [`Settings::from_config`]).
    pub scaling_is_integer: bool,
    /// The over-current trip, mA (`alert_ma`, default [`DEFAULT_ALERT_MA`]), or None for poll only: no ALERT pin
    /// wired (the glue resolves `alert_pin` and passes the trip only for a pin it could arm; plan B: no pin, no
    /// alert writes), or a trip that is not an integer (plan B's struct.pack refused it before any alert write).
    pub alert_ma: Option<i64>,
}

impl Settings {
    /// Read a config block; `alert_wired` says whether the glue resolved an `alert_pin` to a GPIO. A missing key
    /// takes plan B's default.
    ///
    /// A key that is PRESENT but not an integer never takes the default. A fractional shunt is exactly what the
    /// module's own advice produces (back the effective shunt out of a known current: 10.4 mOhm, say), and plan B
    /// fed it straight into its arithmetic: the float CALIBRATION failed struct.pack, setup returned False and the
    /// monitor was skipped, out loud. Calibrating for the default 10 mOhm instead would read every current and
    /// power ~4 % wrong without a word, so the port fails setup at the same step ([`Settings::scaling_is_integer`]).
    /// A fractional `alert_ma` failed the same pack inside plan B's alert block, which fell back to polling only:
    /// so does the port (no trip). A whole float such as 10.0 counts as not an integer, as it did for struct.pack.
    pub fn from_config(config: &Value, alert_wired: bool) -> Settings {
        let integer = |key: &str, default: i64| match config.get(key) {
            None => Some(default),
            Some(value) => value.as_i64(),
        };
        let shunt_mohms = integer("shunt_mohms", DEFAULT_SHUNT_MOHMS);
        let max_current_ma = integer("max_current_ma", DEFAULT_MAX_CURRENT_MA);
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
            shunt_mohms: shunt_mohms.unwrap_or(DEFAULT_SHUNT_MOHMS),
            max_current_ma: max_current_ma.unwrap_or(DEFAULT_MAX_CURRENT_MA),
            scaling_is_integer: shunt_mohms.is_some() && max_current_ma.is_some(),
            alert_ma: if alert_wired {
                integer("alert_ma", DEFAULT_ALERT_MA)
            } else {
                None
            },
        }
    }
}

/// One INA226 on a bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ina226 {
    /// The I2C address.
    address: u8,
    /// The shunt, milliohms.
    shunt_mohms: i64,
    /// The full-scale current, mA: scales every current and power reading.
    max_current_ma: i64,
    /// Whether the two scaling numbers came as integers: when not, setup refuses to calibrate.
    scaling_is_integer: bool,
    /// The over-current trip requested, mA (None: no ALERT pin wired).
    alert_ma: Option<i64>,
    /// The CALIBRATION value setup wrote, kept so the re-arm writes the SAME value, never a re-derivation.
    calibration: Option<u16>,
    /// The ALERT LIMIT setup armed, None when the alert is off (no pin, or its writes failed: poll only).
    alert_limit: Option<u16>,
}

impl Ina226 {
    /// A driver for the part these settings describe; nothing is written until setup.
    pub fn new(settings: &Settings) -> Ina226 {
        Ina226 {
            address: settings.address,
            shunt_mohms: settings.shunt_mohms,
            max_current_ma: settings.max_current_ma,
            scaling_is_integer: settings.scaling_is_integer,
            alert_ma: settings.alert_ma,
            calibration: None,
            alert_limit: None,
        }
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// The CALIBRATION value setup wrote, the one the re-arm restores; None before a successful setup.
    pub fn calibration(&self) -> Option<u16> {
        self.calibration
    }

    /// The over-current trip in mA while the hardware alert is armed, else 0 (plan B's inspect and log value).
    pub fn alert_ma(&self) -> i64 {
        if self.alert_limit.is_some() {
            self.alert_ma.unwrap_or(0)
        } else {
            0
        }
    }

    /// Convert the three raw registers to milli-units (ina226._read's arithmetic): bus voltage (u16), current
    /// (s16) and power (u16).
    pub fn convert(&self, bus_voltage: u16, current: i16, power: u16) -> Measurement {
        let saturate = |value: i128| value.clamp(i128::from(i32::MIN), i128::from(i32::MAX)) as i32;
        let full_scale = i128::from(self.max_current_ma);
        Measurement {
            voltage_mv: saturate(i128::from(bus_voltage) * 5 / 4),
            // floored like Python's //: the divisor is positive, so div_euclid is the floor. The products are wide
            // because the full scale is any config integer; the result leaves i32 only for a full scale past
            // ~40 kA, so the saturation is never reached by a real configuration
            current_ma: saturate((i128::from(current) * full_scale).div_euclid(CURRENT_FULL_SCALE)),
            power_mw: saturate((i128::from(power) * full_scale).div_euclid(CURRENT_FULL_SCALE) * POWER_LSB_RATIO),
        }
    }

    /// Read one register's 16 bits: a pointer write, then two bytes.
    fn register<Bus: I2c>(&self, bus: &mut Bus, register: u8) -> Result<[u8; 2], Bus::Error> {
        let mut word = [0u8; 2];
        bus.write_read(self.address, &[register], &mut word)?;
        Ok(word)
    }

    /// Write one register's 16 bits, big-endian.
    fn write_register<Bus: I2c>(&self, bus: &mut Bus, register: u8, value: u16) -> Result<(), Bus::Error> {
        let [high, low] = value.to_be_bytes();
        bus.write(self.address, &[register, high, low])
    }

    /// Read the live bus voltage, current and power (ina226._read): THREE separate 2-byte reads, bus voltage,
    /// current, power, in plan B's order (see the module notes for why never one).
    pub fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Measurement, Fault<Bus::Error>> {
        let bus_voltage = u16::from_be_bytes(self.register(bus, REGISTER_BUS_VOLTAGE).map_err(Fault::Bus)?);
        let current = i16::from_be_bytes(self.register(bus, REGISTER_CURRENT).map_err(Fault::Bus)?);
        let power = u16::from_be_bytes(self.register(bus, REGISTER_POWER).map_err(Fault::Bus)?);
        Ok(self.convert(bus_voltage, current, power))
    }

    /// Read the die id; [`Fault::Identity`] when it is not the INA226's.
    pub fn identify<Bus: I2c>(&self, bus: &mut Bus) -> Result<(), Fault<Bus::Error>> {
        let read = u16::from_be_bytes(self.register(bus, REGISTER_DIE_ID).map_err(Fault::Bus)?);
        if read == DIE_ID {
            Ok(())
        } else {
            Err(Fault::Identity { expected: DIE_ID, read })
        }
    }

    /// The on-demand self-test (ina226.probe): the die id reads back, then one live read.
    pub fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Measurement, Fault<Bus::Error>> {
        self.identify(bus)?;
        self.read(bus)
    }

    /// Classify a failed setup at the wire level (ina226.diagnose through i2cbus): read the die id's high byte,
    /// 0x22, and sort it with [`id_classify`].
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String {
        let mut high = [0u8; 1];
        let read = bus
            .write_read(self.address, &[REGISTER_DIE_ID], &mut high)
            .ok()
            .map(|()| high[0]);
        id_classify(read, DIE_ID.to_be_bytes()[0])
    }

    /// Write the over-current trip and the SOL mask, in plan B's order.
    fn arm_alert<Bus: I2c>(&self, bus: &mut Bus, limit: u16) -> Result<(), Bus::Error> {
        self.write_register(bus, REGISTER_ALERT_LIMIT, limit)?;
        self.write_register(bus, REGISTER_MASK_ENABLE, MASK_ENABLE_SOL)
    }
}

impl<Bus: I2c> Device<Bus> for Ina226 {
    type BusError = Bus::Error;

    /// Verify the die id, write the configuration, compute and write the calibration, then arm the over-current
    /// alert when a pin is wired (ina226.setup's bus half).
    ///
    /// The order is plan B's to the transaction, failures included: a calibration that does not fit, or scaling
    /// numbers that are not integers ([`Fault::Configuration`]), is found AFTER the configuration write. An alert
    /// whose limit does not fit or whose writes fail does not sink the monitor: it falls back to polling only (no
    /// alert), and setup succeeds.
    fn setup(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.identify(bus)?;
        self.write_register(bus, REGISTER_CONFIGURATION, CONFIGURATION)
            .map_err(Fault::Bus)?;
        let calibration = if self.scaling_is_integer {
            calibration(self.max_current_ma, self.shunt_mohms)
        } else {
            None // plan B's arithmetic made a float CALIBRATION, which struct.pack refused
        };
        let calibration = calibration.ok_or(Fault::Configuration)?;
        self.write_register(bus, REGISTER_CALIBRATION, calibration)
            .map_err(Fault::Bus)?;
        self.calibration = Some(calibration);
        self.alert_limit = None;
        if let Some(alert_ma) = self.alert_ma
            && let Some(limit) = alert_limit(alert_ma, self.shunt_mohms)
            && self.arm_alert(bus, limit).is_ok()
        {
            self.alert_limit = Some(limit);
        }
        Ok(())
    }

    /// Restore CONFIGURATION and CALIBRATION (the value setup wrote), then ALERT LIMIT and MASK/ENABLE when the
    /// alert was armed: everything a general call cleared. Plan B restored only the first two.
    fn rearm(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        let calibration = self.calibration.ok_or(Fault::Configuration)?;
        self.write_register(bus, REGISTER_CONFIGURATION, CONFIGURATION)
            .map_err(Fault::Bus)?;
        self.write_register(bus, REGISTER_CALIBRATION, calibration)
            .map_err(Fault::Bus)?;
        if let Some(limit) = self.alert_limit {
            self.arm_alert(bus, limit).map_err(Fault::Bus)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
