//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! [`Member`]: a part that came up, as its port's owner keeps it, with its [`Shared`] state: what the owner publishes
//! and the executor reads (`inspect`, `verify`, the pending calibrations).

use drivers::axes::Axes;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::absolute::AbsoluteState;
use crate::barometer::BarometerState;
use crate::inertial::InertialState;
use crate::magnetometer::MagnetometerState;
use crate::part::{Calibrated, Part, Run};
use crate::pitot::PitotShared;
use crate::power::PowerState;
use crate::published::Published;

/// A part's published state, by kind: the owner writes it, any thread reads it. Each lives as long as the board
/// (made once, when its part came up).
#[derive(Clone, Copy)]
pub enum Shared {
    /// The BMI323's INT1, ODR and halves.
    Inertial(&'static Published<InertialState>),
    /// A backup barometer's double-read guard (the BMP280, the BMP581).
    Barometer(&'static Published<BarometerState>),
    /// The BMM350's calibration evidence.
    Magnetometer(&'static Published<MagnetometerState>),
    /// The BNO055's calibration and fusion.
    Absolute(&'static Published<AbsoluteState>),
    /// The SDP810's state and its owed tare.
    Pitot(&'static PitotShared),
    /// The INA226's latest reading.
    Power(&'static Published<Option<PowerState>>),
}

impl Shared {
    /// The shared state of `part`, as it stands now; None for a part with nothing of its own to publish (the
    /// ICP-10111, the lasers and the I2C LSM6DSO32 show only their channels).
    fn of<Pin>(part: &Part<Pin>) -> Option<Shared> {
        fn slot<T: Copy>(value: T) -> &'static Published<T> {
            Box::leak(Box::new(Published::new(value)))
        }
        Some(match part {
            Part::Bmi323(_) => Shared::Inertial(slot(InertialState::default())),
            Part::Bmp280(_) | Part::Bmp581(_) => Shared::Barometer(slot(BarometerState::default())),
            Part::Bmm350(magnetometer) => {
                Shared::Magnetometer(slot(MagnetometerState::of(magnetometer, MagnetometerState::default())))
            }
            Part::Bno055(fused) => Shared::Absolute(slot(AbsoluteState::of(fused))),
            Part::Sdp810(pitot) => {
                let shared: &'static PitotShared = Box::leak(Box::new(PitotShared::new()));
                shared.note(pitot);
                Shared::Pitot(shared)
            }
            Part::Ina226(_) => Shared::Power(slot(None)),
            Part::Icp10111(_) | Part::Lsm6dso32(_) | Part::Vl53(_) => return None,
        })
    }

    /// Publishes what the part's calibration or tare is now (a restore, a calibrate, an apply, an update).
    pub(crate) fn note<Pin>(self, part: &Part<Pin>) {
        match (self, part) {
            (Shared::Magnetometer(published), Part::Bmm350(magnetometer)) => {
                published.publish(MagnetometerState::of(magnetometer, published.read()))
            }
            (Shared::Absolute(published), Part::Bno055(fused)) => published.publish(AbsoluteState::of(fused)),
            (Shared::Pitot(shared), Part::Sdp810(pitot)) => shared.note(pitot),
            _ => {}
        }
    }
}

/// An I2C part that came up, as its port's owner keeps it.
pub struct Member<Pin> {
    /// The entry's name.
    pub name: &'static str,
    /// The driver.
    pub part: Part<Pin>,
    /// What it runs on the owner's tick, from the owner's start ([`Run::new`]).
    pub run: Option<Run>,
    /// Its published state, None for a part with none.
    pub shared: Option<Shared>,
    /// How its axes map onto the airframe's (`axes`, M9): what the BMI323's and the BNO055's runs apply; the identity
    /// for a part with no vectors, and for the BMM350, which maps its own after its calibration.
    pub axes: Axes,
    /// Its interrupt line, when the config wires one: the BMI323's INT1 (`int_pin`), counted as evidence (M9); the
    /// INA226's over-current ALERT (`alert_pin`), counted as trips.
    pub interrupt: Option<u8>,
    /// Its read period, ms (`period_ms`), when the config sets one: the SDP810's, the INA226's, the BNO055's and the
    /// lasers' runs take their driver's default otherwise (M10).
    pub period_ms: Option<u32>,
}

impl<Pin> Member<Pin> {
    /// The part named `name` that came up, its state published from now on, no run yet.
    pub fn new(
        name: &'static str,
        part: Part<Pin>,
        axes: Axes,
        interrupt: Option<u8>,
        period_ms: Option<u32>,
    ) -> Member<Pin> {
        Member {
            name,
            shared: Shared::of(&part),
            part,
            run: None,
            axes,
            interrupt,
            period_ms,
        }
    }

    /// Publishes the part's calibration state again.
    fn note(&self) {
        if let Some(shared) = self.shared {
            shared.note(&self.part);
        }
    }

    /// A tare captured on the owner, published and owed to NVS.
    fn owe_tare(&self, tare: i32) {
        if let Some(Shared::Pitot(shared)) = self.shared {
            shared.owe(tare);
        }
    }

    /// Plan B's `calibrate()`: None on success, else why. The barometers capture their ground again from fresh
    /// conversions (plan B icp10111.py:337-344); the pitot captures its tare, owed to NVS; a part with nothing to
    /// calibrate succeeds when nothing is pending (plan B's default `calibration() or None`).
    ///
    /// The BMM350's calibration is computed from the evidence its run gathered and handed back NOT in force
    /// ([`Calibrated::Persist`]): the executor persists it, then applies it ([`Member::apply_calibration`]); a
    /// calibration it refuses answers its instruction. So is the BNO055's learned profile, once its figure-8
    /// converged.
    pub fn calibrate<Bus: I2c>(&mut self, port: &mut Bus, delay: &mut dyn DelayNs) -> Calibrated
    where
        Pin: OutputPin,
    {
        let name = self.name;
        let ground = |captured: Result<f32, String>| match captured {
            Ok(ground) => {
                log::info!("{name} :: calibrated: ground {ground:.1} m AMSL");
                None
            }
            Err(fault) => Some(format!("ground zero: {fault}")),
        };
        // the pitot's captured tare, published and owed once the part is free again
        let mut tared = None;
        let result = match &mut self.part {
            Part::Icp10111(barometer) => {
                ground(barometer.capture_ground(port, delay).map_err(|fault| fault.to_string()))
            }
            Part::Bmp280(barometer) => ground(barometer.capture_ground(port, delay).map_err(|fault| fault.to_string())),
            Part::Bmp581(barometer) => ground(barometer.capture_ground(port, delay).map_err(|fault| fault.to_string())),
            Part::Sdp810(pitot) => match pitot.calibrate() {
                Ok(tare) => {
                    log::info!("{name} :: calibrated: zero_offset {:.2} Pa", tare as f32 / 100.0);
                    tared = Some(tare);
                    None
                }
                Err(why) => Some(why.to_string()),
            },
            Part::Bmm350(magnetometer) => match magnetometer.calibrate() {
                Ok(calibration) => {
                    return Calibrated::Persist {
                        key: drivers::bmm350::NVS_KEY,
                        blob: calibration.to_bytes().to_vec(),
                    };
                }
                Err(why) => Some(why),
            },
            // the figure-8 is the operator's; once it converged, the learned profile is read here and persisted on
            // the executor (plan B bno055.calibrate), then marked saved by apply_calibration
            Part::Bno055(fused) if !fused.calibrated() => fused.owed(),
            Part::Bno055(fused) => match fused.read_profile(port, delay) {
                Ok(profile) => {
                    return Calibrated::Persist {
                        key: drivers::bno055::NVS_KEY,
                        blob: profile.to_vec(),
                    };
                }
                Err(fault) => Some(format!("profile read failed: {fault}")),
            },
            other => Some(other.calibration().to_string()).filter(|pending| !pending.is_empty()),
        };
        if let Some(tare) = tared {
            self.note();
            self.owe_tare(tare);
        }
        Calibrated::Done(result)
    }

    /// Puts the calibration `blob` in force once the executor persisted it (D9 of the M9 plan): the BMM350's, logged
    /// in plan B's words; the BNO055's profile is in force already, so it is only marked saved. None on success, else
    /// why.
    pub fn apply_calibration(&mut self, blob: &[u8]) -> Option<String> {
        let name = self.name;
        let line = match &mut self.part {
            Part::Bno055(fused) => {
                fused.mark_saved();
                format!(
                    "{name} :: calibration profile saved ({} bytes) -- survives power cycles",
                    blob.len()
                )
            }
            Part::Bmm350(magnetometer) => match magnetometer.restore(blob) {
                Some(calibration) => format!(
                    "{name} :: calibrated: centre ({}, {}) radii ({}, {})",
                    calibration.centre_x, calibration.centre_y, calibration.radius_x, calibration.radius_y
                ),
                None => return Some("the calibration blob is no calibration".to_string()),
            },
            _ => return Some(format!("{name} takes no calibration blob")),
        };
        self.note();
        log::info!("{line}");
        None
    }

    /// Plan B's `update()`: the changed properties. The barometers take `ground`, and `rezero` from their own altitude
    /// channel when fresh (the ICP-10111: three periods); the pitot its zero, owed to NVS; a part with nothing writable
    /// changes nothing.
    ///
    /// Errors: the part refused the change (CC answers `err refused <text>`).
    pub fn update(&mut self, properties: &Value, now_us: u64) -> Result<Vec<&'static str>, String> {
        let run = &self.run;
        let altitude = |window_us: u32| match run {
            Some(Run::Barometer(outputs)) => outputs
                .altitude
                .as_ref()
                .and_then(|writer| writer.recent(window_us, now_us)),
            _ => None,
        };
        let pitot = match &mut self.part {
            Part::Icp10111(barometer) => {
                let fresh = altitude(barometer.period_ms().saturating_mul(3).saturating_mul(1000));
                return barometer
                    .update(properties, fresh)
                    .map_err(|refusal| refusal.to_string());
            }
            Part::Bmp280(barometer) => {
                return barometer
                    .update(properties, altitude(drivers::bmp280::REZERO_WINDOW_US as u32))
                    .map_err(|refusal| refusal.to_string());
            }
            Part::Bmp581(barometer) => {
                return barometer
                    .update(properties, altitude(drivers::bmp581::REZERO_WINDOW_US as u32))
                    .map_err(|refusal| refusal.to_string());
            }
            Part::Sdp810(pitot) => pitot.update(properties).map_err(|refusal| refusal.to_string())?,
            _ => return Ok(Vec::new()),
        };
        self.note();
        if let Some(tare) = pitot.persist {
            self.owe_tare(tare);
        }
        Ok(pitot.changed)
    }

    /// The warm start's pitot tare (the crumb's, Pa): put in force without persisting (a flash write mid-air is not
    /// wanted). Whether the part is a pitot that took it.
    pub fn restore_pitot_zero(&mut self, pascal: f32) -> bool {
        let Part::Sdp810(pitot) = &mut self.part else {
            return false;
        };
        pitot.set_zero_offset(pascal);
        self.note();
        true
    }
}
