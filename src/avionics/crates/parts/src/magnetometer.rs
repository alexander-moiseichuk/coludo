//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMM350's run (M9): its `mag` channel and the evidence its calibration is computed from.

use controller::{DeviceEntry, Topology};
use databoard::Writer;
use drivers::bmm350::Bmm350;
use drivers::freshness;
use embedded_hal::i2c::I2c;
use recorder::{Cell, Rows};

use crate::Platform;
use crate::published::Published;

/// What `inspect` and the pending calibrations show of the magnetometer beyond its channel (plan B
/// bmm350.py:382-387).
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct MagnetometerState {
    /// Whether its hard/soft-iron calibration is in force.
    pub calibrated: bool,
    /// How many of the eight octants its turn has covered.
    pub covered: u32,
    /// How many samples it has seen since boot.
    pub seen: u32,
    /// The extrema its evidence holds (x min, x max, y min, y max), once it has seen a sample.
    pub span: [i32; 4],
}

impl MagnetometerState {
    /// The magnetometer's state now.
    pub(crate) fn of(magnetometer: &Bmm350, previous: MagnetometerState) -> MagnetometerState {
        MagnetometerState {
            calibrated: magnetometer.calibrated(),
            covered: magnetometer.covered(),
            seen: magnetometer.seen(),
            span: magnetometer.span().map_or(previous.span, |span| {
                [span.x_minimum, span.x_maximum, span.y_minimum, span.y_maximum]
            }),
        }
    }

    /// Its pending calibration, in plan B's words (bmm350.calibration()); empty when it is calibrated.
    pub fn instruction(&self) -> String {
        drivers::bmm350::format_instruction(self.calibrated, self.covered, self.seen)
    }

    /// The extrema, None before a sample.
    pub fn extrema(&self) -> Option<[i32; 4]> {
        (self.seen > 0).then_some(self.span)
    }
}

/// The BMM350's outputs (M9): its `mag` channel and stream, read every [`drivers::bmm350::POLL_PERIOD_MS`] (twice a
/// conversion at 12.5 Hz), each fresh conversion observed once for the calibration's evidence (plan B bmm350.run).
pub struct Magnetometer {
    /// The `mag` writer (counts: raw, or centred and normalised once calibrated).
    mag: Option<Writer<'static, [f32; 3]>>,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When the next read is due, microseconds.
    next_due_us: u64,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// The part read stale and nothing fresh since: one log line.
    stale: bool,
    /// Where its calibration state is published.
    published: &'static Published<MagnetometerState>,
}

impl Magnetometer {
    /// The outputs of source `name`, writing the health of `entry`, its calibration state published to `published`.
    pub(crate) fn new(
        name: &'static str,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        published: &'static Published<MagnetometerState>,
        platform: &'static dyn Platform,
    ) -> Magnetometer {
        Magnetometer {
            mag: topology.vector("mag").and_then(|parameter| parameter.writer(name)),
            rows: platform.rows(name),
            entry,
            next_due_us: 0,
            failing: false,
            stale: false,
            published,
        }
    }

    /// Records the latest read's outcome in the table entry.
    fn set_healthy(&self, healthy: bool) {
        if let Some(entry) = self.entry {
            entry.set_healthy(healthy);
        }
    }

    /// One owner wake for the part named `name`: when due, the read; a fresh conversion observed, pushed and rowed.
    pub(crate) fn tick<Bus: I2c>(&mut self, name: &str, magnetometer: &mut Bmm350, port: &mut Bus, now_us: u64) {
        if now_us < self.next_due_us {
            return;
        }
        let period_us = u64::from(drivers::bmm350::POLL_PERIOD_MS) * 1000;
        self.next_due_us = (self.next_due_us + period_us).max(now_us + period_us / 2);
        let field = match magnetometer.read_fresh(port, now_us) {
            Ok((freshness::Change::Fresh, field)) => field,
            Ok((freshness::Change::Unchanged, _)) => return,
            Ok((freshness::Change::Stale, _)) => {
                self.set_healthy(false);
                if !self.stale {
                    log::warn!("{name} :: no new conversion -- stale");
                    self.stale = true;
                }
                return;
            }
            Err(fault) => {
                self.set_healthy(false);
                if !self.failing {
                    log::warn!("bmm350 :: read {fault}");
                    self.failing = true;
                }
                return;
            }
        };
        self.failing = false;
        self.stale = false;
        self.set_healthy(true);
        let [x, y, z] = magnetometer.observe(&field);
        self.published
            .publish(MagnetometerState::of(magnetometer, self.published.read()));
        if let Some(writer) = &mut self.mag {
            writer.push([x as f32, y as f32, z as f32], now_us);
        }
        self.rows.push(now_us, &[Cell::I32(x), Cell::I32(y), Cell::I32(z)]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extrema_wait_for_a_sample() {
        let mut state = MagnetometerState {
            span: [1, 2, 3, 4],
            ..MagnetometerState::default()
        };
        assert_eq!(state.extrema(), None);
        state.seen = 1;
        assert_eq!(state.extrema(), Some([1, 2, 3, 4]));
    }

    #[test]
    fn a_calibrated_magnetometer_owes_nothing() {
        let owed = MagnetometerState::default();
        assert!(!owed.instruction().is_empty());
        let calibrated = MagnetometerState {
            calibrated: true,
            ..MagnetometerState::default()
        };
        assert_eq!(calibrated.instruction(), "");
    }
}
