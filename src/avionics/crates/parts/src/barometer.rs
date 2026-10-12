//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A barometer's run (M6, M9, M18): the ICP-10111's two-phase measurement and the backups' double reads (the BMP581
//! of the SEN0697, the BMP280 of the SEN0253), each conversion published to five channels and a row.

use control::change::Change;
use controller::{DeviceEntry, Topology};
use databoard::Writer;
use drivers::icp10111::{Icp10111, Poll};
use drivers::{bmp280, bmp581};
use embedded_hal::i2c::I2c;
use recorder::{Cell, Rows};

use crate::Platform;
use crate::part::Event;
use crate::published::Published;

/// How many backup-barometer reads in a row may go unjudged (torn or implausible) before the part is unhealthy: 60
/// ms, three conversion periods, as its stale rule.
const UNJUDGED_LIMIT: u32 = 6;

/// What `inspect` shows of a backup barometer's double read (M9): the pairs that disagreed (mostly a conversion
/// landing between the two bursts, about 1 read in 80; the rest the bus garbage the double read exists for) and the
/// agreeing pairs outside the part's operating range, since boot.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct BarometerState {
    /// Read pairs that disagreed (drivers' `Outcome::Torn`).
    pub torn: u32,
    /// Agreeing pairs outside the operating range (`Outcome::Implausible`): bus garbage read twice alike.
    pub implausible: u32,
}

/// One barometer conversion, whichever part made it.
#[derive(Clone, Copy)]
struct Conversion {
    /// m AMSL.
    altitude: f32,
    /// Degrees C.
    temperature: f32,
    /// Pa.
    pressure: f32,
}

impl From<&drivers::icp10111::Reading> for Conversion {
    fn from(reading: &drivers::icp10111::Reading) -> Conversion {
        Conversion {
            altitude: reading.altitude,
            temperature: reading.temperature,
            pressure: reading.pressure,
        }
    }
}

impl From<&bmp280::Reading> for Conversion {
    fn from(reading: &bmp280::Reading) -> Conversion {
        Conversion {
            altitude: reading.altitude,
            temperature: reading.temperature,
            pressure: reading.pressure,
        }
    }
}

impl From<&bmp581::Reading> for Conversion {
    fn from(reading: &bmp581::Reading) -> Conversion {
        Conversion {
            altitude: reading.altitude,
            temperature: reading.temperature,
            pressure: reading.pressure,
        }
    }
}

/// What a backup's double read came to, whichever part made it.
enum Judged {
    /// A new conversion, and the elevation above its ground.
    Fresh(Conversion, f32),
    /// The same conversion as the last read's, not yet stale.
    Unchanged,
    /// No new conversion for three periods.
    Stale,
    /// The two reads disagreed.
    Torn,
    /// The two reads agreed on a value outside the part's range.
    Implausible,
    /// The read failed, as one line.
    Failed(String),
}

/// A barometer's outputs: its five databoard channels (plan B's four in its push order, then `change`) and its stream.
pub struct Barometer {
    /// The `altitude` writer (m AMSL), when the quantity is on the databoard.
    pub(crate) altitude: Option<Writer<'static, f32>>,
    /// The `temperature` writer (degrees C).
    temperature: Option<Writer<'static, f32>>,
    /// The `pressure` writer (Pa).
    pressure: Option<Writer<'static, f32>>,
    /// The `elevation` writer (m above the ground).
    elevation: Option<Writer<'static, f32>>,
    /// The `change` writer (m/s, climb positive; M9).
    change: Option<Writer<'static, f32>>,
    /// The climb rate's samples: the least-squares slope of the altitude over the last half second.
    climb: Change,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its table entry, whose `healthy` the run writes (plan B's `Task._healthy`).
    entry: Option<&'static DeviceEntry<'static>>,
    /// A backup's guard counters and where they are published; None for the ICP-10111.
    guard: Option<(BarometerState, &'static Published<BarometerState>)>,
    /// A read failed and none succeeded since: one log line per run (plan B's `note`).
    failing: bool,
    /// The part read stale (the BMP581's unchanged bytes for three conversions) and nothing fresh since: one line.
    stale: bool,
    /// The backup's reads in a row that were neither judged nor published (torn pairs, implausible values).
    unjudged_run: u32,
}

impl Barometer {
    /// The outputs of source `name`, writing the health of `entry`, a backup's guard published to `guard`.
    pub(crate) fn new(
        name: &'static str,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        guard: Option<&'static Published<BarometerState>>,
        platform: &'static dyn Platform,
    ) -> Barometer {
        let writer = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
        Barometer {
            altitude: writer("altitude"),
            temperature: writer("temperature"),
            pressure: writer("pressure"),
            elevation: writer("elevation"),
            change: writer("change"),
            climb: Change::default(),
            rows: platform.rows(name),
            entry,
            guard: guard.map(|published| (published.read(), published)),
            failing: false,
            stale: false,
            unjudged_run: 0,
        }
    }

    /// A read that failed: unhealthy, one line per run.
    fn fail(&mut self, name: &str, fault: &dyn std::fmt::Display) {
        self.set_healthy(false);
        if !self.failing {
            log::warn!("{name} :: read {fault}");
            self.failing = true;
        }
    }

    /// A part that stopped converting (its bytes unchanged for three conversion periods): unhealthy, one line.
    fn freeze(&mut self, name: &str) {
        self.set_healthy(false);
        if !self.stale {
            log::warn!("{name} :: no new conversion -- stale");
            self.stale = true;
        }
    }

    /// Records the latest read's outcome in the table entry.
    fn set_healthy(&self, healthy: bool) {
        if let Some(entry) = self.entry {
            entry.set_healthy(healthy);
        }
    }

    /// Publishes a reading (plan B icp10111.py:346-362): altitude, temperature, pressure, then the elevation above
    /// the ground, then the climb rate once its window holds enough (M9), and the row -- one for every conversion,
    /// never decimated (measured -> reported, the operator).
    fn publish(&mut self, elevation: Option<f32>, reading: Conversion, now_us: u64) {
        self.failing = false;
        self.stale = false;
        self.set_healthy(true);
        let change = self.climb.push(reading.altitude, now_us);
        for (writer, value) in [
            (&mut self.altitude, Some(reading.altitude)),
            (&mut self.temperature, Some(reading.temperature)),
            (&mut self.pressure, Some(reading.pressure)),
            (&mut self.elevation, elevation),
            (&mut self.change, change),
        ] {
            if let (Some(writer), Some(value)) = (writer, value) {
                writer.push(value, now_us);
            }
        }
        if !self.rows.recording() {
            return;
        }
        let row = [
            Cell::F32(reading.altitude),
            Cell::F32(reading.temperature),
            Cell::F32(reading.pressure),
            elevation.map_or(Cell::None, Cell::F32),
            change.map_or(Cell::None, Cell::F32),
        ];
        self.rows.push(now_us, &row);
    }

    /// One owner wake of the ICP-10111: its two-phase measurement advanced; a conversion published. Returns
    /// [`Event::Rearm`] when its recovery asks the owner to re-arm the port's other parts.
    pub(crate) fn tick_icp<Bus: I2c>(
        &mut self,
        barometer: &mut Icp10111,
        port: &mut Bus,
        now_us: u64,
    ) -> Option<Event> {
        match barometer.poll(port, now_us) {
            Poll::Waiting | Poll::Issued | Poll::GeneralCall => None,
            Poll::Reading(reading) => {
                self.publish(barometer.elevation(&reading), Conversion::from(&reading), now_us);
                None
            }
            Poll::Failed { fault, .. } => {
                self.set_healthy(false);
                if !self.failing {
                    log::warn!("icp10111 :: read {fault}");
                    self.failing = true;
                }
                None
            }
            Poll::Rearm => Some(Event::Rearm),
        }
    }

    /// One owner wake of the BMP280 named `name`: its double read judged and, when fresh, published.
    pub(crate) fn tick_bmp280<Bus: I2c>(
        &mut self,
        name: &str,
        barometer: &mut bmp280::Bmp280,
        port: &mut Bus,
        now_us: u64,
    ) {
        let judged = match barometer.read_fresh(port, now_us) {
            Ok(bmp280::Outcome::Fresh(reading)) => {
                Judged::Fresh(Conversion::from(&reading), barometer.elevation(&reading))
            }
            Ok(bmp280::Outcome::Unchanged) => Judged::Unchanged,
            Ok(bmp280::Outcome::Stale) => Judged::Stale,
            Ok(bmp280::Outcome::Torn) => Judged::Torn,
            Ok(bmp280::Outcome::Implausible(_)) => Judged::Implausible,
            Err(fault) => Judged::Failed(fault.to_string()),
        };
        self.judge(name, judged, now_us);
    }

    /// One owner wake of the BMP581 named `name`: its double read judged and, when fresh, published.
    pub(crate) fn tick_bmp581<Bus: I2c>(
        &mut self,
        name: &str,
        barometer: &mut bmp581::Bmp581,
        port: &mut Bus,
        now_us: u64,
    ) {
        let judged = match barometer.read_fresh(port, now_us) {
            Ok(bmp581::Outcome::Fresh(reading)) => {
                Judged::Fresh(Conversion::from(&reading), barometer.elevation(&reading))
            }
            Ok(bmp581::Outcome::Unchanged) => Judged::Unchanged,
            Ok(bmp581::Outcome::Stale) => Judged::Stale,
            Ok(bmp581::Outcome::Torn) => Judged::Torn,
            Ok(bmp581::Outcome::Implausible(_)) => Judged::Implausible,
            Err(fault) => Judged::Failed(fault.to_string()),
        };
        self.judge(name, judged, now_us);
    }

    /// A backup's read, judged: published, frozen, counted or failed. A part that answers only garbage is never
    /// judged stale: unhealthy after a run of it all the same.
    fn judge(&mut self, name: &str, judged: Judged, now_us: u64) {
        let unjudged = match judged {
            Judged::Fresh(reading, elevation) => {
                self.publish(Some(elevation), reading, now_us);
                false
            }
            Judged::Unchanged => false,
            Judged::Stale => {
                self.freeze(name);
                false
            }
            Judged::Torn => {
                self.count(|state| state.torn = state.torn.wrapping_add(1));
                true
            }
            Judged::Implausible => {
                self.count(|state| state.implausible = state.implausible.wrapping_add(1));
                true
            }
            Judged::Failed(fault) => {
                self.fail(name, &fault);
                false
            }
        };
        self.unjudged_run = if unjudged { self.unjudged_run + 1 } else { 0 };
        if self.unjudged_run >= UNJUDGED_LIMIT {
            self.freeze(name);
        }
    }

    /// Counts a guard event and publishes the counters.
    fn count(&mut self, event: impl FnOnce(&mut BarometerState)) {
        if let Some((state, published)) = &mut self.guard {
            event(state);
            published.publish(*state);
        }
    }
}
