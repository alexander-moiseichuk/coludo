//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ESP32-P4's on-die temperature sensor.
//!
//! Native code at full load, profiling and the camera pipeline heat the die; the readings go on the health
//! heartbeat and on every bench run, so a number always comes with its thermal context.

use esp_idf_svc::hal::temp_sensor::{TempSensor, TempSensorConfig, TempSensorDriver};

/// The lowest reading the configured range resolves (the sensor's 20..100 °C band: a running board sits in
/// it, and a heat test does not saturate below 100 °C).
const RANGE_MIN: i32 = 20;
/// The highest reading the configured range resolves.
const RANGE_MAX: i32 = 100;

/// The started sensor.
pub struct Thermometer(TempSensorDriver<'static>);

impl Thermometer {
    /// Install and start the sensor.
    pub fn new(sensor: TempSensor<'static>) -> anyhow::Result<Self> {
        let mut config = TempSensorConfig::new();
        config.range_min = RANGE_MIN;
        config.range_max = RANGE_MAX;
        let mut driver = TempSensorDriver::new(&config, sensor)?;
        driver.enable()?;
        Ok(Self(driver))
    }

    /// The die temperature in °C, None when the read fails.
    pub fn celsius(&self) -> Option<f32> {
        self.0.get_celsius().ok()
    }
}
