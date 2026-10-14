//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A laser ranger's run (M10): the VL53L1X's or the VL53L4CX's height above the ground.

use controller::{DeviceEntry, Topology};
use databoard::Writer;
use drivers::vl53l1x::Vl53;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use recorder::{Cell, Rows};

use crate::Platform;
use crate::due::Due;

/// A laser ranger's outputs (M10, plan B vl53l1x.run): `agl` (m), polled every `period_ms` (50), published only for a
/// waiting, valid range (the data-ready check plan B lacked; an invalid status is read, cleared, never published).
pub struct Ranger {
    /// The `agl` writer.
    agl: Option<Writer<'static, f32>>,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When a poll is due.
    due: Due,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
}

impl Ranger {
    /// The outputs of source `name`, polled every `period_ms`, writing the health of `entry`.
    pub(crate) fn new(
        name: &'static str,
        period_ms: u32,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        platform: &'static dyn Platform,
    ) -> Ranger {
        Ranger {
            agl: topology.scalar("agl").and_then(|parameter| parameter.writer(name)),
            rows: platform.rows(name),
            entry,
            due: Due::every(period_ms),
            failing: false,
        }
    }

    /// One owner wake: when due, a range read and, when valid, published.
    pub(crate) fn tick<Bus: I2c, Pin: OutputPin>(
        &mut self,
        name: &str,
        ranger: &Vl53<Pin>,
        port: &mut Bus,
        now_us: u64,
    ) {
        if !self.due.take(now_us) {
            return;
        }
        match ranger.range(port) {
            Ok(ranging) => {
                self.failing = false;
                if let Some(entry) = self.entry {
                    entry.set_healthy(true);
                }
                let Some(agl) = ranging.agl() else {
                    return;
                };
                if let Some(writer) = &mut self.agl {
                    writer.push(agl, now_us);
                }
                // irq_runs: the lasers are polled (no data-ready wire on v1.x), plan B's 0
                self.rows.push(now_us, &[Cell::F32(agl), Cell::I32(0)]);
            }
            Err(fault) => {
                if let Some(entry) = self.entry {
                    entry.set_healthy(false);
                }
                if !self.failing {
                    log::warn!("{name} :: read {fault}");
                    self.failing = true;
                }
            }
        }
    }
}
