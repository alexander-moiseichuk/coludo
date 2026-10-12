//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMI323's run (M9): a backup to the SPI IMU, read on every owner wake.

use controller::{DeviceEntry, Topology};
use databoard::Writer;
use drivers::axes::Axes;
use drivers::bmi323::Bmi323;
use embedded_hal::i2c::I2c;
use recorder::{Cell, Rows};

use crate::Platform;
use crate::published::Published;

/// What `inspect` shows of the BMI323 beyond its channels (plan B bmi323.py:305-317 and the port's own).
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct InertialState {
    /// Its INT1 line: None when no wire is declared (polled by design), else whether it is silent.
    pub interrupt_silent: Option<bool>,
    /// The INT1 edges its latest read took (`irq_runs`).
    pub irq_runs: u32,
    /// Its fresh accel conversions since boot.
    pub conversions: u32,
    /// The uptime of the first fresh accel conversion, ms.
    pub first_ms: u32,
    /// The uptime of the latest, ms.
    pub last_ms: u32,
    /// Its raced halves (accel, rate): the data-ready flag clear but the bytes changed.
    pub raced: [u32; 2],
    /// Its halves flagged over bytes identical to the last read's (accel, rate).
    pub identical: [u32; 2],
}

impl InertialState {
    /// Its measured ODR, Hz, over its fresh accel conversions: None before two of them.
    pub fn odr_hz(&self) -> Option<f32> {
        let span_ms = self.last_ms.wrapping_sub(self.first_ms);
        (self.conversions > 1 && span_ms > 0).then(|| (self.conversions - 1) as f32 * 1000.0 / span_ms as f32)
    }
}

/// The BMI323's outputs (M9): its `accel` and `rate` channels (a backup to the SPI IMU, at its config priority) and
/// its stream, every conversion through the airframe's axes. Read every owner wake, paced by the control tick; its
/// INT1, when wired, is evidence only (D12 of the M9 plan; plan B sampled on the edge).
pub struct Inertial {
    /// The `accel` writer (g).
    accel: Option<Writer<'static, [f32; 3]>>,
    /// The `rate` writer (deg/s).
    rate: Option<Writer<'static, [f32; 3]>>,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// How its axes map onto the airframe's.
    axes: Axes,
    /// Its INT1 GPIO until armed: armed once the control thread runs (its GPIO interrupt service first).
    interrupt: Option<u8>,
    /// Whether INT1 is armed: its edges are counted.
    armed: bool,
    /// Reads in a row with no new conversion.
    stale_run: u32,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// Its state, and where it is published on every wake.
    state: InertialState,
    /// The published copy.
    published: &'static Published<InertialState>,
    /// The board's interrupts.
    platform: &'static dyn Platform,
}

impl Inertial {
    /// The outputs of source `name`, mapped through `axes`, counting INT1 on `interrupt` once armed, writing the health
    /// of `entry`, its state published to `published`.
    pub(crate) fn new(
        name: &'static str,
        axes: Axes,
        interrupt: Option<u8>,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        published: &'static Published<InertialState>,
        platform: &'static dyn Platform,
    ) -> Inertial {
        let writer = |quantity: &str| topology.vector(quantity).and_then(|parameter| parameter.writer(name));
        Inertial {
            accel: writer("accel"),
            rate: writer("rate"),
            rows: platform.rows(name),
            entry,
            axes,
            interrupt,
            armed: false,
            stale_run: 0,
            failing: false,
            state: published.read(),
            published,
            platform,
        }
    }

    /// Records the latest read's outcome in the table entry: true when that changed it.
    fn set_healthy(&self, healthy: bool) -> bool {
        self.entry.is_some_and(|entry| entry.set_healthy(healthy))
    }

    /// One owner wake for the part named `name`: its INT1 edges, the 16-byte status burst, and what is new published
    /// through the airframe's axes, with a row (plan B bmi323.py's run loop, read on the tick); its state published.
    pub(crate) fn tick<Bus: I2c>(&mut self, name: &str, imu: &mut Bmi323, port: &mut Bus, now_us: u64) {
        self.read(name, imu, port, now_us);
        self.published.publish(self.state);
    }

    /// The wake's work (see [`Inertial::tick`]).
    fn read<Bus: I2c>(&mut self, name: &str, imu: &mut Bmi323, port: &mut Bus, now_us: u64) {
        if let (Some(gpio), false) = (self.interrupt, self.armed)
            && self.platform.interrupts_ready()
        {
            self.armed = true;
            match self.platform.arm_second_interrupt(gpio) {
                Ok(()) => log::info!("{name} :: INT1 GPIO{gpio} counted (evidence; reads paced by the tick)"),
                Err(error) => log::warn!("{name} :: INT1 GPIO{gpio} not armed ({error}): no edges counted"),
            }
        }
        let runs = if self.armed {
            self.platform.take_second_edges()
        } else {
            0
        };
        self.state.irq_runs = runs;
        if self.armed && imu.wake(runs) {
            log::warn!("{name} :: INT1 silent -- sampling on the tick, the line unproven");
        }
        self.state.interrupt_silent = imu.interrupt_silent();
        let (fresh, sample) = match imu.read_fresh(port) {
            Ok(read) => read,
            Err(fault) => {
                self.set_healthy(false);
                if !self.failing {
                    log::warn!("{name} :: read {fault}");
                    self.failing = true;
                }
                return;
            }
        };
        self.failing = false;
        for (half, raced, identical) in [
            (0, fresh.accel_raced, fresh.accel_identical),
            (1, fresh.rate_raced, fresh.rate_identical),
        ] {
            if raced {
                self.state.raced[half] = self.state.raced[half].wrapping_add(1);
            }
            if identical {
                self.state.identical[half] = self.state.identical[half].wrapping_add(1);
            }
        }
        if !fresh.accel && !fresh.rate {
            self.stale_run = self.stale_run.saturating_add(1);
            if self.stale_run >= drivers::bmi323::STALE_LIMIT && self.set_healthy(false) {
                log::warn!("{name} :: no new conversion -- stale");
            }
            return;
        }
        self.stale_run = 0;
        self.set_healthy(true);
        if fresh.accel {
            let now_ms = (now_us / 1000) as u32;
            if self.state.conversions == 0 {
                self.state.first_ms = now_ms;
            }
            self.state.conversions = self.state.conversions.wrapping_add(1);
            self.state.last_ms = now_ms;
        }
        let accel = self.axes.apply(sample.accel_g());
        if let (true, Some(writer)) = (fresh.accel, &mut self.accel) {
            writer.push(accel, now_us);
        }
        if let (true, Some(writer)) = (fresh.rate, &mut self.rate) {
            writer.push(self.axes.apply(sample.rate_degrees_per_second()), now_us);
        }
        if !self.rows.recording() {
            return;
        }
        let [gx, gy, gz] = self.axes.apply(sample.rate_centidegrees_per_second());
        let row = [
            Cell::F32(accel[0]),
            Cell::F32(accel[1]),
            Cell::F32(accel[2]),
            Cell::I32(gx),
            Cell::I32(gy),
            Cell::I32(gz),
            Cell::I32(runs as i32),
        ];
        self.rows.push(now_us, &row);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_odr_needs_two_conversions_over_a_span() {
        let mut state = InertialState::default();
        assert_eq!(state.odr_hz(), None);
        state.conversions = 1;
        state.first_ms = 1000;
        state.last_ms = 1000;
        assert_eq!(state.odr_hz(), None, "one conversion");
        state.conversions = 2;
        assert_eq!(state.odr_hz(), None, "two at the same millisecond: no span");
        state.conversions = 101;
        state.last_ms = 2000;
        assert_eq!(state.odr_hz(), Some(100.0));
    }

    #[test]
    fn the_odr_spans_the_millisecond_wrap() {
        let state = InertialState {
            conversions: 11,
            first_ms: u32::MAX - 49,
            last_ms: 50,
            ..InertialState::default()
        };
        assert_eq!(state.odr_hz(), Some(100.0));
    }
}
