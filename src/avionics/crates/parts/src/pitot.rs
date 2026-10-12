//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The SDP810's run (M10): the pitot's dynamic pressure and airspeed, and the tare its owner captures for the
//! executor to persist.

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use controller::{DeviceEntry, Topology};
use databoard::Writer;
use drivers::sdp810::Sdp810;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use recorder::{Cell, Rows};

use crate::Platform;
use crate::due::Due;
use crate::published::Published;

/// The pitot's pending calibration in plan B's words (sdp810.calibration, its live reading left out: the text is a
/// constant on the owner).
pub(crate) const PITOT_INSTRUCTION: &str =
    "keep the pitot in STILL AIR -- do NOT blow into it, this captures the zero tare";

/// What `inspect` shows of the pitot beyond its channels (plan B sdp810.py inspect).
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct PitotState {
    /// Degrees C.
    pub temperature_c: f32,
    /// The scale factor (Pa per count).
    pub scale: i32,
    /// The zero offset, Pa.
    pub zero_offset_pa: f32,
    /// Whether a tare is in force.
    pub tared: bool,
    /// The q -> v air density, kg/m^3.
    pub air_density: f32,
}

impl PitotState {
    /// The part's state now, in the resolution `inspect` shows (C x 100, Pa x 100, kg/m^3 x 10000), as plan B's
    /// integers round it.
    pub(crate) fn of(pitot: &Sdp810) -> PitotState {
        PitotState {
            temperature_c: (pitot.temperature() * 100.0).round() / 100.0,
            scale: i32::from(pitot.scale()),
            zero_offset_pa: pitot.zero_offset() as f32 / 100.0,
            tared: pitot.tared(),
            air_density: (pitot.air_density() * 10_000.0).round() / 10_000.0,
        }
    }
}

/// The pitot's shared state: its published state (None until first published) and the tare its owner captured that
/// the executor has not persisted yet -- the NVS write is the executor's, never the flight core's.
pub struct PitotShared {
    /// Its state for `inspect`, once known.
    state: Published<Option<PitotState>>,
    /// The captured tare (Pa x 100), valid while `pending`.
    tare: AtomicI32,
    /// Whether `tare` waits to be persisted.
    pending: AtomicBool,
}

impl PitotShared {
    /// Nothing known, nothing owed.
    pub(crate) const fn new() -> PitotShared {
        PitotShared {
            state: Published::new(None),
            tare: AtomicI32::new(0),
            pending: AtomicBool::new(false),
        }
    }

    /// Publishes the pitot's state.
    pub(crate) fn note(&self, pitot: &Sdp810) {
        self.state.publish(Some(PitotState::of(pitot)));
    }

    /// Its state, None before its first publish.
    pub fn state(&self) -> Option<PitotState> {
        self.state.read()
    }

    /// A tare the owner captured (Pa x 100), for the executor to persist.
    pub(crate) fn owe(&self, tare: i32) {
        self.tare.store(tare, Ordering::Relaxed);
        self.pending.store(true, Ordering::Release);
    }

    /// The tare owed to NVS (Pa x 100), taken: None when none waits.
    pub fn take_owed(&self) -> Option<i32> {
        self.pending
            .swap(false, Ordering::Acquire)
            .then(|| self.tare.load(Ordering::Relaxed))
    }
}

/// The SDP810's outputs (M10, plan B sdp810.run): `dynamic_pressure` (Pa) and `airspeed` (m/s), read every
/// `period_ms` (20: 50 Hz) from the part's continuous mode, a row of plan B's integers per read; a bus failure counts
/// toward the driver's restart ladder, a CRC reject is only noted.
pub struct Pitot {
    /// The `dynamic_pressure` writer.
    pressure: Option<Writer<'static, f32>>,
    /// The `airspeed` writer.
    airspeed: Option<Writer<'static, f32>>,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When a read is due.
    due: Due,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// Where its state is published.
    shared: &'static PitotShared,
}

impl Pitot {
    /// The outputs of source `name`, read every `period_ms`, writing the health of `entry`, its state published to
    /// `shared`.
    pub(crate) fn new(
        name: &'static str,
        period_ms: u32,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        shared: &'static PitotShared,
        platform: &'static dyn Platform,
    ) -> Pitot {
        let writer = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
        Pitot {
            pressure: writer("dynamic_pressure"),
            airspeed: writer("airspeed"),
            rows: platform.rows(name),
            entry,
            due: Due::every(period_ms),
            failing: false,
            shared,
        }
    }

    /// One owner wake: when due, a frame read and published, or the failure noted (and the restart's outcome).
    pub(crate) fn tick<Bus: I2c>(&mut self, pitot: &mut Sdp810, port: &mut Bus, delay: &mut dyn DelayNs, now_us: u64) {
        if !self.due.take(now_us) {
            return;
        }
        match pitot.read(port, delay) {
            Ok(sample) => {
                self.failing = false;
                if let Some(entry) = self.entry {
                    entry.set_healthy(true);
                }
                for (writer, value) in [
                    (&mut self.pressure, sample.dynamic_pressure),
                    (&mut self.airspeed, sample.airspeed),
                ] {
                    if let Some(writer) = writer {
                        writer.push(value, now_us);
                    }
                }
                self.shared.note(pitot);
                self.rows.push(now_us, &sample.row.map(Cell::I32));
            }
            Err(failure) => {
                if let Some(entry) = self.entry {
                    entry.set_healthy(false);
                }
                if !self.failing {
                    log::warn!("sdp810 :: read {}", failure.fault);
                    self.failing = true;
                }
                match failure.restart {
                    Some(Ok(())) => log::warn!(
                        "sdp810 :: restarted continuous mode after {} failures",
                        drivers::sdp810::RESTART_AFTER
                    ),
                    Some(Err(fault)) => log::warn!("sdp810 :: restart failed: {fault}"),
                    None => {}
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tare_is_owed_once() {
        let shared = PitotShared::new();
        assert_eq!(shared.take_owed(), None);
        shared.owe(-125);
        assert_eq!(shared.take_owed(), Some(-125));
        assert_eq!(shared.take_owed(), None, "taken");
        shared.owe(10);
        shared.owe(20);
        assert_eq!(shared.take_owed(), Some(20), "the latest tare wins");
    }

    #[test]
    fn the_state_is_unknown_until_published() {
        let shared = PitotShared::new();
        assert_eq!(shared.state(), None);
        let pitot = Sdp810::new(&drivers::sdp810::Settings::from_config(&serde_json::json!({})));
        shared.note(&pitot);
        let state = shared.state().expect("published");
        assert!(!state.tared);
        assert_eq!(state.scale, i32::from(pitot.scale()));
    }
}
