//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The I2C parts' own `inspect` keys (plan B's drivers' `inspect`): [`own`] reads a part's published state and its
//! channels' latest values. Typed: a `json!` value would widen every f32 to f64 (a reading of 0.774944 would print as
//! 0.774944007396698).

use controller::Topology;
use serde::Serialize;
use serde_json::{Map, Value};

use crate::Platform;
use crate::member::Shared;

/// What an inspect answer reads besides the part's published state.
pub struct Lookup<'a> {
    /// The board's quantities.
    pub topology: &'a Topology,
    /// The part's name: its source on the databoard.
    pub name: &'a str,
    /// Now, for its channels' windows, microseconds.
    pub now_us: u64,
    /// Its address as `inspect` shows it, for the parts whose keys carry it.
    pub addr: Option<String>,
}

impl Lookup<'_> {
    /// The part's own latest value of scalar `quantity` while inside its window, else None (§6: stale reads null).
    fn scalar(&self, quantity: &str) -> Option<f32> {
        let parameter = self.topology.scalar(quantity)?;
        parameter.channel(self.name)?.recent(parameter.window_us(), self.now_us)
    }

    /// The same of a vector quantity.
    fn vector(&self, quantity: &str) -> Option<[f32; 3]> {
        let parameter = self.topology.vector(quantity)?;
        parameter.channel(self.name)?.recent(parameter.window_us(), self.now_us)
    }
}

/// An I2C part's own inspect keys (plan B icp10111.py:455-459, bmi323.py:305-317, sdp810.py, ina226.py, bno055.py,
/// vl53l1x.py, bmm350.py:382-387), in plan B's order.
#[derive(Serialize)]
#[serde(untagged)]
pub enum Own {
    /// The BMI323 (M9): the LSM6DSO32's keys -- its INT1 wired and silent (None: no wire declared, polled by
    /// design, plan B bmi323.py:310-315), its own channels' latest values -- and the edges its latest read took; the
    /// port's own: its measured ODR (null before two conversions), its channels' lapses by quantity, and its raced
    /// and identical halves (accel, rate).
    Bmi323 {
        addr: Option<String>,
        interrupt: bool,
        interrupt_silent: Option<bool>,
        accel_g: Option<[f32; 3]>,
        rate_dps: Option<[f32; 3]>,
        irq_runs: u32,
        odr_hz: Option<f32>,
        lapses: Map<String, Value>,
        raced: [u32; 2],
        identical: [u32; 2],
    },
    /// The ICP-10111: its own channels' latest values, and its climb rate (m/s, M9).
    Barometer {
        altitude_m: Option<f32>,
        temperature_c: Option<f32>,
        pressure_pa: Option<f32>,
        elevation_m: Option<f32>,
        change_m_s: Option<f32>,
    },
    /// A backup barometer, the BMP581 or the BMP280 (M9, M18): the barometer keys, and its double read's guard: the
    /// pairs that disagreed and the agreeing reads outside the part's operating range, since boot.
    Bmp581 {
        addr: Option<String>,
        altitude_m: Option<f32>,
        temperature_c: Option<f32>,
        pressure_pa: Option<f32>,
        elevation_m: Option<f32>,
        change_m_s: Option<f32>,
        torn: u32,
        implausible: u32,
    },
    /// The SDP810 (plan B sdp810.py inspect): its channels' latest, its temperature, scale, zero offset, whether a tare
    /// is in force, and its q -> v air density.
    Pitot {
        dynamic_pressure_pa: Option<f32>,
        airspeed_ms: Option<f32>,
        temperature_c: Option<f32>,
        scale: Option<i32>,
        zero_offset_pa: Option<f32>,
        tared: Option<bool>,
        air_density: Option<f32>,
    },
    /// The INA226 (plan B ina226.py inspect): mV, mA, mW and the over-current ALERT trips.
    Power {
        voltage_mv: Option<i32>,
        current_ma: Option<i32>,
        power_mw: Option<i32>,
        alerts: u32,
    },
    /// The BNO055 (plan B bno055.py inspect, M18): its channels' latest, whether its fusion is stalled (the attitude
    /// withheld: a channel going quiet looks like a missing sensor), the live CALIB_STAT (sys, gyr, acc, mag), the
    /// convergence latch, and whether the latch came from a restored profile.
    Absolute {
        addr: Option<String>,
        attitude_deg: Option<[f32; 3]>,
        accel_g: Option<[f32; 3]>,
        fusion_stalled: bool,
        calibration: Option<[u8; 4]>,
        calibrated: bool,
        calibration_restored: bool,
    },
    /// A laser ranger (plan B vl53l1x.py inspect): whether its data-ready line is wired, its channel's latest.
    Ranger { interrupt: bool, agl_m: Option<f32> },
    /// The BMM350 (plan B bmm350.py:382-387): its channel's latest counts, whether its calibration is in force, the
    /// octants its turn covered and the samples it has seen.
    Magnetometer {
        addr: Option<String>,
        mag_counts: Option<[f32; 3]>,
        calibrated: bool,
        span: Option<[i32; 4]>,
        samples: u32,
        sectors: u32,
    },
}

/// The own keys of the I2C part `implementation` named in `lookup`, from its `shared` state (a part that came up has
/// one; one that did not shows its kind's empty state); None for an implementation with no keys here: the
/// LSM6DSO32 (on I2C it has no run; on SPI it is the control thread's), a BNO055 with no state, any non-I2C part.
pub fn own(implementation: &str, shared: Option<Shared>, lookup: &Lookup, platform: &dyn Platform) -> Option<Own> {
    Some(match implementation {
        "bmi323" => {
            let state = match shared {
                Some(Shared::Inertial(published)) => published.read(),
                _ => Default::default(),
            };
            Own::Bmi323 {
                addr: lookup.addr.clone(),
                interrupt: state.interrupt_silent.is_some(),
                interrupt_silent: state.interrupt_silent,
                accel_g: lookup.vector("accel"),
                rate_dps: lookup.vector("rate"),
                irq_runs: state.irq_runs,
                odr_hz: state.odr_hz(),
                lapses: platform
                    .lapses(lookup.name)
                    .into_iter()
                    .map(|(quantity, count)| (quantity, Value::from(count)))
                    .collect(),
                raced: state.raced,
                identical: state.identical,
            }
        }
        "bno055" => {
            let Some(Shared::Absolute(published)) = shared else {
                return None;
            };
            let state = published.read();
            Own::Absolute {
                addr: lookup.addr.clone(),
                attitude_deg: lookup.vector("attitude"),
                accel_g: lookup.vector("accel"),
                fusion_stalled: state.fusion_stalled,
                calibration: state.calibration,
                calibrated: state.calibrated,
                calibration_restored: state.calibration_restored,
            }
        }
        "icp10111" => Own::Barometer {
            altitude_m: lookup.scalar("altitude"),
            temperature_c: lookup.scalar("temperature"),
            pressure_pa: lookup.scalar("pressure"),
            elevation_m: lookup.scalar("elevation"),
            change_m_s: lookup.scalar("change"),
        },
        "bmp280" | "bmp581" => {
            let guard = match shared {
                Some(Shared::Barometer(published)) => published.read(),
                _ => Default::default(),
            };
            Own::Bmp581 {
                addr: lookup.addr.clone(),
                altitude_m: lookup.scalar("altitude"),
                temperature_c: lookup.scalar("temperature"),
                pressure_pa: lookup.scalar("pressure"),
                elevation_m: lookup.scalar("elevation"),
                change_m_s: lookup.scalar("change"),
                torn: guard.torn,
                implausible: guard.implausible,
            }
        }
        "sdp810" => {
            let state = match shared {
                Some(Shared::Pitot(pitot)) => pitot.state(),
                _ => None,
            };
            Own::Pitot {
                dynamic_pressure_pa: lookup.scalar("dynamic_pressure"),
                airspeed_ms: lookup.scalar("airspeed"),
                temperature_c: state.map(|state| state.temperature_c),
                scale: state.map(|state| state.scale),
                zero_offset_pa: state.map(|state| state.zero_offset_pa),
                tared: state.map(|state| state.tared),
                air_density: state.map(|state| state.air_density),
            }
        }
        "ina226" => {
            let reading = match shared {
                Some(Shared::Power(published)) => published.read(),
                _ => None,
            };
            Own::Power {
                voltage_mv: reading.map(|reading| reading.voltage_mv),
                current_ma: reading.map(|reading| reading.current_ma),
                power_mw: reading.map(|reading| reading.power_mw),
                alerts: reading.map_or(0, |_| platform.alert_edges()),
            }
        }
        "vl53l1x" | "vl53l4cx" => Own::Ranger {
            interrupt: false,
            agl_m: lookup.scalar("agl"),
        },
        "bmm350" => {
            let state = match shared {
                Some(Shared::Magnetometer(published)) => published.read(),
                _ => Default::default(),
            };
            Own::Magnetometer {
                addr: lookup.addr.clone(),
                mag_counts: lookup.vector("mag"),
                calibrated: state.calibrated,
                span: state.extrema(),
                samples: state.seen,
                sectors: state.covered,
            }
        }
        _ => return None,
    })
}
