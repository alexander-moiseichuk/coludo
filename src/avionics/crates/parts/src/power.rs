//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The INA226's run (M10): the battery's voltage, current and power, and the over-current ALERT's trips.

use controller::{DeviceEntry, Topology};
use databoard::Writer;
use drivers::ina226::Ina226;
use embedded_hal::i2c::I2c;
use recorder::{Cell, Rows};

use crate::Platform;
use crate::due::Due;
use crate::published::Published;

/// The INA226's latest reading for `inspect` (plan B ina226.py inspect).
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct PowerState {
    /// mV.
    pub voltage_mv: i32,
    /// mA.
    pub current_ma: i32,
    /// mW.
    pub power_mw: i32,
}

/// The INA226's outputs (M10, plan B ina226.run): `voltage` (mV), `current` (mA) and `power` (mW), read every
/// `period_ms` (100), a row of plan B's integers per read.
pub struct Power {
    /// The `voltage`, `current` and `power` writers.
    writers: [Option<Writer<'static, f32>>; 3],
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When a read is due.
    due: Due,
    /// The ALERT GPIO (`alert_pin`), when the config wires one.
    alert: Option<u8>,
    /// Its interrupt was armed (or tried): once, after the control thread installed the GPIO service.
    armed: bool,
    /// The trips logged so far: a new one logs once, as plan B's run did.
    logged_alerts: u32,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// Where its latest reading is published (None before the first).
    published: &'static Published<Option<PowerState>>,
    /// The board's interrupts.
    platform: &'static dyn Platform,
}

impl Power {
    /// The outputs of source `name`, read every `period_ms`, its ALERT on `alert` when wired, writing the health of
    /// `entry`, its readings published to `published`.
    pub(crate) fn new(
        name: &'static str,
        period_ms: u32,
        alert: Option<u8>,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        published: &'static Published<Option<PowerState>>,
        platform: &'static dyn Platform,
    ) -> Power {
        let writer = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
        Power {
            writers: [writer("voltage"), writer("current"), writer("power")],
            rows: platform.rows(name),
            entry,
            due: Due::every(period_ms),
            alert,
            armed: false,
            logged_alerts: 0,
            failing: false,
            published,
            platform,
        }
    }

    /// One owner wake: when due, a reading published, or the failure noted.
    pub(crate) fn tick<Bus: I2c>(&mut self, name: &str, monitor: &Ina226, port: &mut Bus, now_us: u64) {
        // the over-current ALERT (plan B ina226.setup): counted only where the driver armed its limit
        if let (Some(gpio), false, true) = (self.alert, self.armed, monitor.alert_ma() > 0)
            && self.platform.interrupts_ready()
        {
            self.armed = true;
            match self.platform.arm_alert_interrupt(gpio) {
                Ok(()) => log::info!(
                    "{name} :: ALERT GPIO{gpio} counted (over-current > {} mA)",
                    monitor.alert_ma()
                ),
                Err(error) => log::warn!("{name} :: ALERT GPIO{gpio} not armed ({error}): poll only"),
            }
        }
        if !self.due.take(now_us) {
            return;
        }
        match monitor.read(port) {
            Ok(reading) => {
                self.failing = false;
                if let Some(entry) = self.entry {
                    entry.set_healthy(true);
                }
                let values = [reading.voltage_mv, reading.current_ma, reading.power_mw];
                for (writer, value) in self.writers.iter_mut().zip(values) {
                    if let Some(writer) = writer {
                        writer.push(value as f32, now_us);
                    }
                }
                self.published.publish(Some(PowerState {
                    voltage_mv: values[0],
                    current_ma: values[1],
                    power_mw: values[2],
                }));
                // the trips' running total, a per-flight series (plan B's fourth column)
                let alerts = self.platform.alert_edges();
                let row = [
                    Cell::I32(values[0]),
                    Cell::I32(values[1]),
                    Cell::I32(values[2]),
                    Cell::I32(alerts as i32),
                ];
                self.rows.push(now_us, &row);
                if alerts != self.logged_alerts {
                    log::warn!(
                        "{name} :: OVER-CURRENT alerts: {alerts} (> {} mA) -- servo stall / short?",
                        monitor.alert_ma()
                    );
                    self.logged_alerts = alerts;
                }
            }
            Err(fault) => {
                if let Some(entry) = self.entry {
                    entry.set_healthy(false);
                }
                if !self.failing {
                    log::warn!("ina226 :: read {fault}");
                    self.failing = true;
                }
            }
        }
    }
}
