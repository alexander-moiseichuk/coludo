//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The parts behind the board config's entries (doc/specs/avionics-port.md M5, "The registry"): what the port runs
//! for each implementation ([`Kind`], plan B's task registry), and the I2C parts as one [`Part`] over the drivers
//! crate. A part is built afresh from its entry for every setup attempt, as plan B creates a task per attempt; its
//! bus owner then sets it up, re-arms it after a general call and asks its diagnose when the last attempt failed.
//!
//! Not here yet, each with its milestone: the readings and their channels (M6, M9, M10), what a plan B setup
//! restores from storage (the SDP810's tare, the BMM350's hard-iron calibration: M9, M10), the `axes` of the parts
//! whose vectors reach the airframe through the glue (the BMI323's and the LSM6DSO32's, with their reads: M6, M9),
//! and the SPI LSM6DSO32's probe, which needs SPI2's owner (M6).

use std::mem::ManuallyDrop;

use board_config::Entry;
use databoard::Writer;
use drivers::axes::Axes;
use drivers::bmi323::Bmi323;
use drivers::bmm350::Bmm350;
use drivers::bmp581::Bmp581;
use drivers::device::Device;
use drivers::icp10111::{Icp10111, Poll, Reading};
use drivers::ina226::Ina226;
use drivers::lsm6dso32::{Lsm6dso32, OverI2c};
use drivers::sdp810::Sdp810;
use drivers::vl53l1x::Vl53;
use embedded_hal::digital::OutputPin;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::gpio::{AnyOutputPin, GpioError, Output, PinDriver};
use esp_idf_svc::sys::EspError;
use firmware::buses::i2c::I2cPort;
use recorder::{Cell, Telemetry};
use serde_json::Value;

use crate::devices::Topology;

/// The failure reason of a plan B implementation this port has not reached yet (D13 of the M5 plan): plan B's
/// `no driver/activity`, with why.
pub const NOT_PORTED_YET: &str = "no driver/activity -- not ported yet";

/// What runs an entry's implementation (its `driver`, else its `activity`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A [`Part`] on an I2C port.
    I2c,
    /// A part on SPI2: the LSM6DSO32 on its default transport.
    Spi,
    /// Run natively by the firmware, with nothing to set up: up.
    Native,
    /// A plan B implementation not ported yet: failed with [`NOT_PORTED_YET`].
    NotPorted,
    /// No implementation plan B knows: failed with plan B's own `no driver/activity`.
    Unknown,
}

/// Implementations the firmware runs natively: the recorder, the CC link, Wi-Fi, the supervisor's health and the
/// camera.
const NATIVE: [&str; 5] = ["recorder", "cc", "wifi", "health", "ov5647"];

/// Plan B implementations whose port comes with a later milestone.
const NOT_PORTED: [&str; 17] = [
    "adxl375",
    "bno055",
    "bmp280",
    "attitude",
    "atgm336h",
    "neo6mv2",
    "gnss_calib",
    "field",
    "separation",
    "sg90",
    "mg90s",
    "sequencer",
    "checkpoint",
    "watchdog",
    "flight",
    "bluetooth",
    "hitl",
];

/// What runs `entry`. The LSM6DSO32 is an SPI part unless its entry binds it to I2C (plan B's lsm6dso32.py:144: the
/// `bus` defaults to 'spi').
pub fn classify(entry: &Entry) -> Kind {
    match entry.implementation {
        "bmi323" | "bmm350" | "bmp581" | "icp10111" | "ina226" | "sdp810" | "vl53l1x" | "vl53l4cx" => Kind::I2c,
        "lsm6dso32" => match controller::binding(entry, "spi", Some(drivers::lsm6dso32::ADDRESS)) {
            controller::Binding::I2c { .. } => Kind::I2c,
            _ => Kind::Spi,
        },
        implementation if NATIVE.contains(&implementation) => Kind::Native,
        implementation if NOT_PORTED.contains(&implementation) => Kind::NotPorted,
        _ => Kind::Unknown,
    }
}

/// The datasheet address an I2C part's driver binds when its entry has no `addr` (plan B i2cbus.py:301).
pub fn default_address(implementation: &str) -> Option<u8> {
    match implementation {
        "bmi323" => Some(drivers::bmi323::ADDRESS),
        "bmm350" => Some(drivers::bmm350::ADDRESS),
        "bmp581" => Some(drivers::bmp581::ADDRESS),
        "icp10111" => Some(drivers::icp10111::ADDRESS),
        "ina226" => Some(drivers::ina226::ADDRESS),
        "lsm6dso32" => Some(drivers::lsm6dso32::ADDRESS),
        "sdp810" => Some(drivers::sdp810::ADDRESS),
        "vl53l1x" | "vl53l4cx" => Some(drivers::vl53l1x::ADDRESS),
        _ => None,
    }
}

/// What a part does on its owner's tick beyond its bus work: where its readings go (M6: the ICP-10111).
pub enum Run {
    /// A barometer's channels and stream.
    Barometer(Barometer),
}

impl Run {
    /// The run `part` needs on its owner's tick, its writers claimed from `topology` for source `name`; None for a
    /// part with nothing to run yet.
    pub fn new(name: &'static str, part: &Part, topology: &'static Topology) -> Option<Run> {
        match part {
            Part::Icp10111(_) => Some(Run::Barometer(Barometer::new(name, topology))),
            _ => None,
        }
    }
}

/// What a tick asks of the owner beyond the part itself.
pub enum Event {
    /// The part's recovery sent a general call and the bus has settled: re-arm the port's other parts.
    Rearm,
}

/// A barometer's outputs: its four databoard channels (plan B's push order) and its stream.
pub struct Barometer {
    /// The `altitude` writer (m AMSL), when the quantity is on the databoard.
    altitude: Option<Writer<'static, f32>>,
    /// The `temperature` writer (degrees C).
    temperature: Option<Writer<'static, f32>>,
    /// The `pressure` writer (Pa).
    pressure: Option<Writer<'static, f32>>,
    /// The `elevation` writer (m above the ground).
    elevation: Option<Writer<'static, f32>>,
    /// Its stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// A read failed and none succeeded since: one log line per run (plan B's `note`).
    failing: bool,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Barometer {
    /// The outputs of source `name`.
    fn new(name: &'static str, topology: &'static Topology) -> Barometer {
        let writer = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
        Barometer {
            altitude: writer("altitude"),
            temperature: writer("temperature"),
            pressure: writer("pressure"),
            elevation: writer("elevation"),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            failing: false,
            overflowing: false,
        }
    }

    /// Publishes a reading (plan B icp10111.py:346-362): altitude, temperature, pressure, then the elevation above
    /// the ground, and the row.
    fn publish(&mut self, name: &str, elevation: Option<f32>, reading: &Reading, now_us: u64) {
        self.failing = false;
        for (writer, value) in [
            (&mut self.altitude, Some(reading.altitude)),
            (&mut self.temperature, Some(reading.temperature)),
            (&mut self.pressure, Some(reading.pressure)),
            (&mut self.elevation, elevation),
        ] {
            if let (Some(writer), Some(value)) = (writer, value) {
                writer.push(value, now_us);
            }
        }
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        if !telemetry.due(now_us) {
            return;
        }
        let row = [
            Cell::F32(reading.altitude),
            Cell::F32(reading.temperature),
            Cell::F32(reading.pressure),
            elevation.map_or(Cell::None, Cell::F32),
        ];
        match telemetry.push(now_us, &row) {
            Ok(()) => self.overflowing = false,
            Err(overflow) => {
                if !self.overflowing {
                    log::warn!("{name} :: telemetry {overflow}");
                    self.overflowing = true;
                }
            }
        }
    }
}

/// A device's telemetry stream: its file, its columns after `uptime`, and its decimation window in microseconds
/// (positive its own, 0 the global `recorder.telemetry_ms`, negative none).
pub struct Stream {
    /// The file, without the session prefix.
    pub file: String,
    /// The columns after `uptime`.
    pub fields: &'static [&'static str],
    /// The decimation window, microseconds.
    pub decimate_us: i32,
}

/// The LSM6DSO32's columns (plan B lsm6dso32.py:124-126): accel in g, the gyro as raw centidegrees a second, and the
/// INT1 edges the tick saw.
pub const IMU_FIELDS: [&str; 7] = ["ax", "ay", "az", "gx", "gy", "gz", "irq_runs"];

/// The ICP-10111's columns (plan B icp10111.py:123-125): m AMSL, degrees C, Pa, m above the ground.
pub const BAROMETER_FIELDS: [&str; 4] = ["altitude", "temperature", "pressure", "elevation"];

/// The separation switch's columns (plan B separation.py:57): the event and the stage after it.
pub const SEPARATION_FIELDS: [&str; 2] = ["event", "stage"];

/// The stream `entry` writes, when its part is one the port records (M6: the LSM6DSO32, the ICP-10111, the
/// separation switch). The file is `<name>.csv`, except the fixed `separation.csv`; the window is the entry's
/// `telemetry_ms` (0 or absent: the global), as plan B's Telemetry took it, and the separation stream's always the
/// global (plan B passes no window for it).
pub fn stream(entry: &Entry) -> Option<Stream> {
    let window = || {
        entry
            .fields
            .get("telemetry_ms")
            .and_then(Value::as_i64)
            .map_or(0, |milliseconds| {
                milliseconds
                    .saturating_mul(1000)
                    .clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
            })
    };
    let (file, fields, decimate_us): (String, &'static [&'static str], i32) = match entry.implementation {
        "lsm6dso32" => (format!("{}.csv", entry.name), &IMU_FIELDS, window()),
        "icp10111" => (format!("{}.csv", entry.name), &BAROMETER_FIELDS, window()),
        "separation" => ("separation.csv".to_string(), &SEPARATION_FIELDS, 0),
        _ => return None,
    };
    Some(Stream {
        file,
        fields,
        decimate_us,
    })
}

/// A VL53 ranger's XSHUT line, when the config wires one (v0.1; layout nulls it on v1.x), driven for the rest of the
/// run once a part first takes it. Dropping an esp-idf-hal PinDriver disables the pad, so the line would float when a
/// failed attempt's part is dropped: on a pin two laser entries share (the default config's GPIO5), the fitted
/// laser's XSHUT after the other entry's attempts. Plan B's `Pin` was never released either.
pub struct Xshut(ManuallyDrop<PinDriver<'static, Output>>);

impl Xshut {
    /// GPIO `gpio` as an output for a ranger's XSHUT.
    ///
    /// Errors: the pin would not configure.
    fn output(gpio: u8) -> Result<Xshut, EspError> {
        // SAFETY: the board config gives this GPIO to the rangers' XSHUT alone; the driver is never released.
        let driver = PinDriver::output(unsafe { AnyOutputPin::steal(gpio) })?;
        Ok(Xshut(ManuallyDrop::new(driver)))
    }
}

impl embedded_hal::digital::ErrorType for Xshut {
    type Error = GpioError;
}

impl OutputPin for Xshut {
    fn set_low(&mut self) -> Result<(), GpioError> {
        OutputPin::set_low(&mut *self.0)
    }

    fn set_high(&mut self) -> Result<(), GpioError> {
        OutputPin::set_high(&mut *self.0)
    }
}

/// One I2C part, as its bus owner drives it.
pub enum Part {
    /// The SEN0697's IMU.
    Bmi323(Bmi323),
    /// The SEN0697's magnetometer.
    Bmm350(Bmm350),
    /// The SEN0697's barometer.
    Bmp581(Bmp581),
    /// The primary barometer.
    Icp10111(Icp10111),
    /// The power monitor.
    Ina226(Ina226),
    /// The IMU when its entry binds it to I2C.
    Lsm6dso32(Lsm6dso32<OverI2c>),
    /// The pitot's differential pressure sensor.
    Sdp810(Sdp810),
    /// A laser ranger: the VL53L1X or the VL53L4CX silicon.
    Vl53(Vl53<Xshut>),
}

/// Runs `$body` with `$driver` bound to whichever driver `$part` holds.
macro_rules! each {
    ($part:expr, $driver:ident => $body:expr) => {
        match $part {
            Part::Bmi323($driver) => $body,
            Part::Bmm350($driver) => $body,
            Part::Bmp581($driver) => $body,
            Part::Icp10111($driver) => $body,
            Part::Ina226($driver) => $body,
            Part::Lsm6dso32($driver) => $body,
            Part::Sdp810($driver) => $body,
            Part::Vl53($driver) => $body,
        }
    };
}

impl Part {
    /// A fresh driver for `entry`, an I2C part ([`Kind::I2c`]) at `address`, its binding's (plan B's task created
    /// for one attempt). Its INT, ALERT and XSHUT wires resolve through the board's pins (plan B `_pin_gpio`).
    ///
    /// Errors: the config refuses the part: a magnetometer `axes` that is no level rotation, an XSHUT pin that would
    /// not drive, an implementation that is not an I2C part.
    pub fn build(document: &Value, entry: &Entry, address: u8) -> Result<Part, String> {
        let wired = |key: &str| board_config::pin(document, entry, key).is_some();
        let fields = Value::Object(entry.fields.clone());
        Ok(match entry.implementation {
            "bmi323" => Part::Bmi323(Bmi323::new(address, wired("int_pin"))),
            "bmm350" => {
                let axes = Axes::from_config(&fields).map_err(|error| error.to_string())?;
                Part::Bmm350(Bmm350::new(address).mounted(axes).map_err(|error| error.to_string())?)
            }
            "bmp581" => Part::Bmp581(Bmp581::new(address)),
            "icp10111" => {
                let period_ms = fields
                    .get("period_ms")
                    .and_then(Value::as_u64)
                    .and_then(|period| u32::try_from(period).ok())
                    .unwrap_or(drivers::icp10111::DEFAULT_PERIOD_MS);
                Part::Icp10111(Icp10111::new(address, period_ms))
            }
            "ina226" => Part::Ina226(Ina226::new(&drivers::ina226::Settings::from_config(
                &fields,
                wired("alert_pin"),
            ))),
            "lsm6dso32" => Part::Lsm6dso32(Lsm6dso32::new(OverI2c { address }, wired("int_pin"))),
            "sdp810" => Part::Sdp810(Sdp810::new(&drivers::sdp810::Settings::from_config(&fields))),
            "vl53l1x" | "vl53l4cx" => {
                let silicon = match entry.implementation {
                    "vl53l1x" => &drivers::vl53l1x::VL53L1X,
                    _ => &drivers::vl53l4cx::VL53L4CX,
                };
                let xshut = match board_config::pin(document, entry, "xshut_pin") {
                    Some(gpio) => Some(Xshut::output(gpio).map_err(|error| format!("xshut GPIO{gpio}: {error}"))?),
                    None => None,
                };
                Part::Vl53(Vl53::new(
                    silicon,
                    &drivers::vl53l1x::Settings::from_config(&fields),
                    xshut,
                ))
            }
            other => return Err(format!("'{other}' is not an I2C part")),
        })
    }

    /// Plan B's setup (its bus half): the part from its power-on state to producing data.
    ///
    /// Errors: the fault the setup ended in, as one line.
    pub fn setup(&mut self, port: &mut I2cPort, delay: &mut Delay) -> Result<(), String> {
        each!(self, driver => Device::<I2cPort>::setup(driver, port, delay).map_err(|fault| fault.to_string()))
    }

    /// What a general call took from the part, restored (never its setup: that would re-zero a barometer's ground
    /// and reload the pitot's tare).
    ///
    /// Errors: the fault the re-arm ended in, as one line.
    pub fn rearm(&mut self, port: &mut I2cPort, delay: &mut Delay) -> Result<(), String> {
        each!(self, driver => Device::<I2cPort>::rearm(driver, port, delay).map_err(|fault| fault.to_string()))
    }

    /// Plan B's diagnose after the last failed attempt: what the part answers at the wire level.
    pub fn diagnose(&self, port: &mut I2cPort) -> String {
        let id = port.id();
        match self {
            Part::Bmi323(driver) => driver.diagnose(port, id),
            Part::Bmm350(driver) => driver.diagnose(port, id),
            Part::Bmp581(driver) => driver.diagnose(port),
            Part::Icp10111(driver) => driver.diagnose(port),
            Part::Ina226(driver) => driver.diagnose(port),
            Part::Lsm6dso32(driver) => driver.diagnose(port),
            Part::Sdp810(driver) => driver.diagnose(port),
            Part::Vl53(driver) => driver.diagnose(port),
        }
    }

    /// Plan B's on-demand self-test (`probe`): the part identifies itself and one reading comes back, logged in plan
    /// B's shape (`<name> :: probe: <step> ...`, then `probe: <step> ok <value>` or `probe FAILED: <step>: <why>`).
    /// None when healthy, else the failure, as plan B answers. The ICP-10111 and the SDP810 take one reading of their
    /// own, where plan B waited for its run loop's (D16 of the M5 plan: the owner cannot race itself). A driver
    /// identifies and reads in one call, so a two-step probe logs its steps as one.
    pub fn probe(&mut self, name: &str, port: &mut I2cPort, delay: &mut Delay) -> Option<String> {
        let step = match self {
            Part::Bmi323(_) | Part::Bmm350(_) | Part::Bmp581(_) => "chip id + read",
            Part::Icp10111(_) | Part::Sdp810(_) => "data",
            Part::Ina226(_) => "die id + read",
            Part::Lsm6dso32(_) => "who_am_i + sample",
            Part::Vl53(_) => "model id",
        };
        log::info!("{name} :: probe: {step} ...");
        let outcome = match self {
            Part::Bmi323(driver) => driver
                .probe(port)
                .map(|sample| format!("|a| {:.3} g", magnitude(sample.accel_g())))
                .map_err(|fault| fault.to_string()),
            Part::Bmm350(driver) => match driver.probe(port) {
                Ok([x, y, z]) => Ok(format!("({x}, {y}, {z})")),
                Err(drivers::bmm350::ProbeFailure::Fault(fault)) => Err(fault.to_string()),
                Err(drivers::bmm350::ProbeFailure::NotMeasuring(why)) => Err(why.to_string()),
            },
            Part::Bmp581(driver) => driver
                .probe(port)
                .map(|reading| format!("{:.0} Pa", reading.pressure))
                .map_err(|fault| fault.to_string()),
            Part::Icp10111(driver) => driver
                .probe(port, delay)
                .map(|reading| format!("{:.0} Pa", reading.pressure))
                .map_err(|fault| fault.to_string()),
            Part::Ina226(driver) => driver
                .probe(port)
                .map(|reading| {
                    format!(
                        "{} mV {} mA {} mW",
                        reading.voltage_mv, reading.current_ma, reading.power_mw
                    )
                })
                .map_err(|fault| fault.to_string()),
            Part::Lsm6dso32(driver) => driver
                .probe(port)
                .map(|sample| {
                    let [x, y, z] = sample.rate_degrees_per_second();
                    format!("{:.2}g ({x:.1},{y:.1},{z:.1}) dps", magnitude(sample.accel_g()))
                })
                .map_err(|fault| fault.to_string()),
            Part::Sdp810(driver) => driver
                .read(port, delay)
                .map(|sample| format!("{:.2} Pa ({:.1} m/s)", sample.dynamic_pressure, sample.airspeed))
                .map_err(|failure| failure.fault.to_string()),
            Part::Vl53(driver) => driver
                .probe(port)
                .map(|model| format!("0x{model:04x}"))
                .map_err(|fault| fault.to_string()),
        };
        match outcome {
            Ok(value) => {
                log::info!("{name} :: probe: {step} ok {value}");
                None
            }
            Err(why) => {
                let message = format!("{step}: {why}");
                log::warn!("{name} :: probe FAILED: {message}");
                Some(message)
            }
        }
    }

    /// One owner tick for the part named `name` (M6: the ICP-10111's two-phase measurement, plan B's run loop),
    /// publishing through `run`. Returns [`Event::Rearm`] when the part's recovery asks the owner to re-arm the port's
    /// other parts.
    pub fn tick(&mut self, name: &str, run: &mut Run, port: &mut I2cPort, now_us: u64) -> Option<Event> {
        match (self, run) {
            (Part::Icp10111(barometer), Run::Barometer(outputs)) => match barometer.poll(port, now_us) {
                Poll::Waiting | Poll::Issued | Poll::GeneralCall => None,
                Poll::Reading(reading) => {
                    outputs.publish(name, barometer.elevation(&reading), &reading, now_us);
                    None
                }
                Poll::Failed { fault, .. } => {
                    if !outputs.failing {
                        log::warn!("icp10111 :: read {fault}");
                        outputs.failing = true;
                    }
                    None
                }
                Poll::Rearm => Some(Event::Rearm),
            },
            _ => None,
        }
    }

    /// Whether the part sent a general call since the last ask: only the ICP-10111 sends one, to free itself.
    pub fn take_general_call(&mut self) -> bool {
        match self {
            Part::Icp10111(driver) => driver.take_general_call(),
            _ => false,
        }
    }
}

/// The length of a vector of three.
fn magnitude([x, y, z]: [f32; 3]) -> f32 {
    (x * x + y * y + z * z).sqrt()
}
