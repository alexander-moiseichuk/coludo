//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! [`Part`]: one I2C part's driver, as its bus owner drives it (plan B's task, built per attempt), and [`Run`]: what
//! it does on its owner's tick beyond its bus work.

use board_config::Entry;
use controller::{DeviceEntry, Topology};
use drivers::axes::Axes;
use drivers::bmi323::Bmi323;
use drivers::bmm350::Bmm350;
use drivers::bmp280::Bmp280;
use drivers::bmp581::Bmp581;
use drivers::bno055::Bno055;
use drivers::device::Device;
use drivers::icp10111::Icp10111;
use drivers::ina226::Ina226;
use drivers::lsm6dso32::{Lsm6dso32, OverI2c};
use drivers::sdp810::Sdp810;
use drivers::vl53l1x::Vl53;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::absolute::Absolute;
use crate::barometer::Barometer;
use crate::inertial::Inertial;
use crate::magnetometer::Magnetometer;
use crate::member::{Member, Shared};
use crate::pitot::{PITOT_INSTRUCTION, Pitot};
use crate::power::Power;
use crate::ranger::Ranger;
use crate::{Platform, imu_axes, magnitude};

/// What a tick asks of the owner beyond the part itself.
pub enum Event {
    /// The part's recovery sent a general call and the bus has settled: re-arm the port's other parts.
    Rearm,
}

/// What a part's `calibrate` answers on its owner.
pub enum Calibrated {
    /// Plan B's answer: None success, else why.
    Done(Option<String>),
    /// A calibration computed but not in force: the executor persists `blob` under NVS `key`, then has the owner apply
    /// it (D9 of the M9 plan: persist, then apply, as plan B; no flash work on the flight core).
    Persist { key: &'static str, blob: Vec<u8> },
}

/// One I2C part, as its bus owner drives it; `Pin` is a laser's XSHUT line.
pub enum Part<Pin> {
    /// The SEN0697's IMU.
    Bmi323(Bmi323),
    /// The SEN0697's magnetometer.
    Bmm350(Bmm350),
    /// The SEN0253's barometer (v0.1 boards, M18).
    Bmp280(Bmp280),
    /// The SEN0697's barometer.
    Bmp581(Bmp581),
    /// The SEN0253's 9-DOF IMU: the backup attitude of the v0.1 boards (M18).
    Bno055(Bno055),
    /// The primary barometer.
    Icp10111(Icp10111),
    /// The power monitor.
    Ina226(Ina226),
    /// The IMU when its entry binds it to I2C.
    Lsm6dso32(Lsm6dso32<OverI2c>),
    /// The pitot's differential pressure sensor.
    Sdp810(Sdp810),
    /// A laser ranger: the VL53L1X or the VL53L4CX silicon.
    Vl53(Vl53<Pin>),
}

/// Runs `$body` with `$driver` bound to whichever driver `$part` holds.
macro_rules! each {
    ($part:expr, $driver:ident => $body:expr) => {
        match $part {
            Part::Bmi323($driver) => $body,
            Part::Bmm350($driver) => $body,
            Part::Bmp280($driver) => $body,
            Part::Bmp581($driver) => $body,
            Part::Bno055($driver) => $body,
            Part::Icp10111($driver) => $body,
            Part::Ina226($driver) => $body,
            Part::Lsm6dso32($driver) => $body,
            Part::Sdp810($driver) => $body,
            Part::Vl53($driver) => $body,
        }
    };
}

impl<Pin: OutputPin> Part<Pin> {
    /// A fresh driver for `entry`, an I2C part ([`crate::Kind::I2c`]) at `address`, its binding's (plan B's task created
    /// for one attempt). Its INT and ALERT wires resolve through the board's pins (plan B `_pin_gpio`), its XSHUT
    /// through `xshut`, which makes the line's driver.
    ///
    /// Errors: the config refuses the part: a magnetometer `axes` that is no level rotation, an XSHUT pin that would
    /// not drive, an implementation that is not an I2C part.
    pub fn build(
        document: &Value,
        entry: &Entry,
        address: u8,
        xshut: impl FnOnce(u8) -> Result<Pin, String>,
    ) -> Result<Part<Pin>, String> {
        let wired = |key: &str| board_config::pin(document, entry, key).is_some();
        let fields = Value::Object(entry.fields.clone());
        Ok(match entry.implementation {
            "bmi323" => {
                imu_axes(entry)?;
                Part::Bmi323(Bmi323::new(address, wired("int_pin")))
            }
            "bmm350" => {
                let axes = Axes::from_config(&fields).map_err(|error| error.to_string())?;
                Part::Bmm350(Bmm350::new(address).mounted(axes).map_err(|error| error.to_string())?)
            }
            "bmp280" => Part::Bmp280(Bmp280::new(address)),
            "bmp581" => Part::Bmp581(Bmp581::new(address)),
            "bno055" => {
                imu_axes(entry)?;
                Part::Bno055(Bno055::new(address))
            }
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
                    Some(gpio) => Some(xshut(gpio).map_err(|error| format!("xshut GPIO{gpio}: {error}"))?),
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
    /// Errors: the fault the setup ended in, as one line, and whether another part answered at the address (an
    /// identity of another silicon: retrying cannot help).
    pub fn setup<Bus: I2c>(&mut self, port: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), (String, bool)> {
        each!(self, driver => Device::<Bus>::setup(driver, port, delay)
            .map_err(|fault| (fault.to_string(), matches!(fault, drivers::device::Fault::Identity { .. }))))
    }

    /// What a general call took from the part, restored (never its setup: that would re-zero a barometer's ground
    /// and reload the pitot's tare).
    ///
    /// Errors: the fault the re-arm ended in, as one line.
    pub fn rearm<Bus: I2c>(&mut self, port: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), String> {
        each!(self, driver => Device::<Bus>::rearm(driver, port, delay).map_err(|fault| fault.to_string()))
    }

    /// Plan B's diagnose after the last failed attempt: what the part answers at the wire level, on port `id`.
    pub fn diagnose<Bus: I2c>(&self, port: &mut Bus, id: u8) -> String {
        match self {
            Part::Bmi323(driver) => driver.diagnose(port, id),
            Part::Bmm350(driver) => driver.diagnose(port, id),
            Part::Bmp280(driver) => driver.diagnose(port),
            Part::Bmp581(driver) => driver.diagnose(port),
            Part::Bno055(driver) => driver.diagnose(port),
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
    pub fn probe<Bus: I2c>(&mut self, name: &str, port: &mut Bus, delay: &mut dyn DelayNs) -> Option<String> {
        let step = match self {
            Part::Bmi323(_) | Part::Bmm350(_) | Part::Bmp280(_) | Part::Bmp581(_) | Part::Bno055(_) => "chip id + read",
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
            Part::Bmp280(driver) => driver
                .probe(port)
                .map(|reading| format!("{:.0} Pa", reading.pressure))
                .map_err(|fault| fault.to_string()),
            Part::Bmp581(driver) => driver
                .probe(port)
                .map(|reading| format!("{:.0} Pa", reading.pressure))
                .map_err(|fault| fault.to_string()),
            Part::Bno055(driver) => driver
                .probe(port)
                .map(|sample| {
                    let [heading, roll, pitch] = sample.euler;
                    format!(
                        "heading {heading:.1} roll {roll:.1} pitch {pitch:.1} |a| {:.3} g",
                        magnitude(sample.accel)
                    )
                })
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
    pub fn tick<Bus: I2c>(
        &mut self,
        name: &str,
        run: &mut Run,
        port: &mut Bus,
        delay: &mut dyn DelayNs,
        now_us: u64,
    ) -> Option<Event> {
        match (self, run) {
            (Part::Icp10111(barometer), Run::Barometer(outputs)) => outputs.tick_icp(barometer, port, now_us),
            (Part::Bmi323(imu), Run::Imu(outputs)) => {
                outputs.tick(name, imu, port, now_us);
                None
            }
            (Part::Bmp280(barometer), Run::Barometer(outputs)) => {
                outputs.tick_bmp280(name, barometer, port, now_us);
                None
            }
            (Part::Bmp581(barometer), Run::Barometer(outputs)) => {
                outputs.tick_bmp581(name, barometer, port, now_us);
                None
            }
            (Part::Bmm350(magnetometer), Run::Magnetometer(outputs)) => {
                outputs.tick(name, magnetometer, port, now_us);
                None
            }
            (Part::Bno055(fused), Run::Absolute(outputs)) => {
                outputs.tick(name, fused, port, now_us);
                None
            }
            (Part::Sdp810(pitot), Run::Pitot(outputs)) => {
                outputs.tick(pitot, port, delay, now_us);
                None
            }
            (Part::Ina226(monitor), Run::Power(outputs)) => {
                outputs.tick(name, monitor, port, now_us);
                None
            }
            (Part::Vl53(ranger), Run::Ranger(outputs)) => {
                outputs.tick(name, ranger, port, now_us);
                None
            }
            _ => None,
        }
    }
}

impl<Pin> Part<Pin> {
    /// Whether the part is an IMU, whose run goes first in each owner wake (D13 of the M9 plan): the lowest latency
    /// for the backup IMUs, the BMI323 (v1.x) and the BNO055 (v0.1; a bench board can carry both).
    pub fn first(&self) -> bool {
        matches!(self, Part::Bmi323(_) | Part::Bno055(_))
    }

    /// The calibration the part waits for, in plan B's words (`calibration()`); empty when none. The ICP-10111 asks
    /// for its ground until it holds one (setup captures it, so an up ICP never asks); the BMM350's and the BNO055's
    /// are worded on the executor from their published state ([`crate::MagnetometerState::instruction`],
    /// [`crate::AbsoluteState::instruction`]).
    pub fn calibration(&self) -> &'static str {
        match self {
            Part::Icp10111(barometer) => barometer.instruction(),
            Part::Sdp810(pitot) if !pitot.tared() => PITOT_INSTRUCTION,
            _ => "",
        }
    }

    /// Whether the part re-zeroes a barometer's ground, as [`crate::rezeroes`] says of its implementation: its
    /// `update` and `calibrate` also run at DONE.
    pub fn rezeroes(&self) -> bool {
        matches!(self, Part::Icp10111(_) | Part::Bmp280(_) | Part::Bmp581(_))
    }

    /// Whether the part sent a general call since the last ask: only the ICP-10111 sends one, to free itself.
    pub fn take_general_call(&mut self) -> bool {
        match self {
            Part::Icp10111(driver) => driver.take_general_call(),
            _ => false,
        }
    }
}

/// What a part does on its owner's tick beyond its bus work: where its readings go (M6: the ICP-10111; M9: the
/// BMI323). Each variant is boxed, so the other members of a port do not pay for the largest inline (M9).
pub enum Run {
    /// The BNO055's attitude, accel and stream (M18).
    Absolute(Box<Absolute>),
    /// A barometer's channels, its climb rate and its stream.
    Barometer(Box<Barometer>),
    /// The second IMU's channels and stream.
    Imu(Box<Inertial>),
    /// The magnetometer's channel and stream.
    Magnetometer(Box<Magnetometer>),
    /// The pitot's channels and stream (M10).
    Pitot(Box<Pitot>),
    /// The power monitor's channels and stream (M10).
    Power(Box<Power>),
    /// A laser ranger's channel and stream (M10).
    Ranger(Box<Ranger>),
}

impl Run {
    /// The run `member`'s part needs on its owner's tick, its writers claimed from `topology` for its source name,
    /// writing the health of its table `entry`, publishing to its shared state; None for a part with nothing to run.
    pub fn new<Pin>(
        member: &Member<Pin>,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        platform: &'static dyn Platform,
    ) -> Option<Run> {
        let name = member.name;
        match (&member.part, member.shared) {
            (Part::Icp10111(_), _) => Some(Run::Barometer(Box::new(Barometer::new(
                name, topology, entry, None, platform,
            )))),
            // its elevation on the databoard too: the multi-source machine reads each barometer against itself, so an
            // ICP -> BMP581 handover is never a climb (M9)
            (Part::Bmp280(_) | Part::Bmp581(_), Some(Shared::Barometer(guard))) => Some(Run::Barometer(Box::new(
                Barometer::new(name, topology, entry, Some(guard), platform),
            ))),
            (Part::Bmm350(_), Some(Shared::Magnetometer(published))) => Some(Run::Magnetometer(Box::new(
                Magnetometer::new(name, topology, entry, published, platform),
            ))),
            (Part::Bno055(_), Some(Shared::Absolute(published))) => Some(Run::Absolute(Box::new(Absolute::new(
                name,
                member.period_ms.unwrap_or(drivers::bno055::DEFAULT_PERIOD_MS),
                member.axes,
                topology,
                entry,
                published,
                platform,
            )))),
            (Part::Sdp810(_), Some(Shared::Pitot(shared))) => Some(Run::Pitot(Box::new(Pitot::new(
                name,
                member.period_ms.unwrap_or(drivers::sdp810::DEFAULT_PERIOD_MS),
                topology,
                entry,
                shared,
                platform,
            )))),
            (Part::Ina226(_), Some(Shared::Power(published))) => Some(Run::Power(Box::new(Power::new(
                name,
                member.period_ms.unwrap_or(drivers::ina226::DEFAULT_PERIOD_MS),
                member.interrupt,
                topology,
                entry,
                published,
                platform,
            )))),
            (Part::Vl53(_), _) => Some(Run::Ranger(Box::new(Ranger::new(
                name,
                member.period_ms.unwrap_or(drivers::vl53l1x::DEFAULT_PERIOD_MS),
                topology,
                entry,
                platform,
            )))),
            (Part::Bmi323(_), Some(Shared::Inertial(published))) => Some(Run::Imu(Box::new(Inertial::new(
                name,
                member.axes,
                member.interrupt,
                topology,
                entry,
                published,
                platform,
            )))),
            (Part::Lsm6dso32(_), _) => {
                log::info!("{name} :: no read loop on I2C (SPI only)");
                None
            }
            // a member is made with its part's shared state (Member::new): never reached
            _ => None,
        }
    }

    /// Whether the run stops at DONE (M11b): the IMU, the magnetometer and the pitot; the beacon's sources go on.
    pub fn stops_at_done(&self) -> bool {
        matches!(self, Run::Imu(_) | Run::Magnetometer(_) | Run::Pitot(_))
    }
}
