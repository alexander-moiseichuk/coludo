//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BNO055's run (M18): the fused attitude of the v0.1 boards' backup, withheld while the fusion is stalled.

use controller::{DeviceEntry, Topology};
use databoard::Writer;
use drivers::axes::Axes;
use drivers::bno055::{self, Bno055};
use embedded_hal::i2c::I2c;
use recorder::{Cell, Rows};

use crate::Platform;
use crate::due::Due;
use crate::published::Published;

/// What `inspect`, `verify` and the heartbeat show of the BNO055 beyond its channels (plan B bno055.inspect).
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub struct AbsoluteState {
    /// The fusion is latched stalled: the attitude withheld.
    pub fusion_stalled: bool,
    /// The live CALIB_STAT (sys, gyr, acc, mag), None before the first read.
    pub calibration: Option<[u8; 4]>,
    /// The convergence latch (mag reached 3 once, or a profile was restored), which the live register regularly
    /// disagrees with.
    pub calibrated: bool,
    /// The latch came from a restored profile, not a figure-8 this session.
    pub calibration_restored: bool,
    /// The learned profile is saved in NVS.
    pub saved: bool,
}

impl AbsoluteState {
    /// The part's state now.
    pub(crate) fn of(part: &Bno055) -> AbsoluteState {
        AbsoluteState {
            fusion_stalled: part.stalled(),
            calibration: part
                .calibration_state()
                .map(|state| [state.system, state.gyro, state.accel, state.mag]),
            calibrated: part.calibrated(),
            calibration_restored: part.restored(),
            saved: part.saved(),
        }
    }

    /// Its pending calibration, in plan B's words (bno055.calibration()); None when nothing is owed.
    pub fn instruction(&self) -> Option<String> {
        let state = self
            .calibration
            .map(|[system, gyro, accel, mag]| bno055::CalibrationState {
                system,
                gyro,
                accel,
                mag,
            });
        bno055::owed(state, self.calibrated, self.saved)
    }
}

/// The BNO055's outputs (M18, plan B bno055.run): every period one block read; the fused Euler, in the filter's
/// aerospace frame through the part's `axes`, to `attitude` unless
/// the fusion is stalled (the channel then goes stale and the databoard falls to the next source), the accelerometer
/// through the airframe's axes to `accel` always (that half still works), a row to its stream, and CALIB_STAT read
/// about once a second for the operator.
pub struct Absolute {
    /// The `attitude` writer (heading, roll, pitch, degrees).
    attitude: Option<Writer<'static, [f32; 3]>>,
    /// The `accel` writer (g, the airframe's axes).
    accel: Option<Writer<'static, [f32; 3]>>,
    /// How the part's axes map onto the airframe's (`axes`): the accelerometer's, and the Euler's onto the filter's
    /// frame.
    axes: Axes,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When the next block read is due.
    due: Due,
    /// When the next CALIB_STAT read is due.
    calibration_due: Due,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// Where its calibration and fusion state is published.
    published: &'static Published<AbsoluteState>,
}

impl Absolute {
    /// The outputs of source `name`, read every `period_ms`, writing the health of `entry`, its state published to
    /// `published`.
    pub(crate) fn new(
        name: &'static str,
        period_ms: u32,
        axes: Axes,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
        published: &'static Published<AbsoluteState>,
        platform: &'static dyn Platform,
    ) -> Absolute {
        let writer = |quantity: &str| topology.vector(quantity).and_then(|parameter| parameter.writer(name));
        Absolute {
            attitude: writer("attitude"),
            accel: writer("accel"),
            axes,
            rows: platform.rows(name),
            entry,
            due: Due::every(period_ms),
            calibration_due: Due::every(drivers::bno055::CALIBRATION_PERIOD_MS),
            failing: false,
            published,
        }
    }

    /// Records a read's outcome in the table entry; a failure logged once per run of them.
    fn outcome<Error: core::fmt::Debug>(&mut self, name: &str, fault: Option<drivers::device::Fault<Error>>) {
        if let Some(entry) = self.entry {
            entry.set_healthy(fault.is_none());
        }
        match fault {
            None => self.failing = false,
            Some(fault) if !self.failing => {
                log::warn!("{name} :: read {fault}");
                self.failing = true;
            }
            Some(_) => {}
        }
    }

    /// One owner wake for the part named `name`: when due, the block read, judged, pushed and rowed; when its own
    /// period is due, CALIB_STAT.
    pub(crate) fn tick<Bus: I2c>(&mut self, name: &str, part: &mut Bno055, port: &mut Bus, now_us: u64) {
        if !self.due.take(now_us) {
            return;
        }
        let sample = match part.read(port) {
            Ok(sample) => sample,
            Err(fault) => return self.outcome(name, Some(fault)),
        };
        self.outcome::<()>(name, None);
        match part.judge(&sample) {
            bno055::Fusion::Live => {
                if let Some(writer) = &mut self.attitude {
                    // the filter's frame, not the chip's (drivers bno055 `aerospace`)
                    writer.push(bno055::aerospace(sample.euler, &self.axes), now_us);
                }
            }
            // withheld: the channel goes stale and the databoard moves on; logged on the transition only
            bno055::Fusion::Stalled => {
                log::warn!("{name} :: fusion STALLED (frozen euler while the part rotates) -- attitude withheld")
            }
            // the fusion's first samples after a switch into NDOF read zero: no attitude yet
            bno055::Fusion::StillStalled | bno055::Fusion::Starting => {}
        }
        let accel = self.axes.apply(sample.accel);
        // CONFIG-mode zeros are no data (plan B's `_configuring`): a still or moving accelerometer never reads exactly
        // 0 on all three axes, and a part whose fusion was refused reads them on every block
        if sample.accel != [0.0; 3]
            && let Some(writer) = &mut self.accel
        {
            writer.push(accel, now_us);
        }
        // plan B's row: heading in degrees, roll and pitch in centidegrees (raw x 100 / 16, floored), accel in g
        let [_, roll, pitch] = sample.euler_raw.map(i32::from);
        let [ax, ay, az] = accel;
        self.rows.push(
            now_us,
            &[
                Cell::F32(sample.euler[0]),
                Cell::I32((roll * 100).div_euclid(16)),
                Cell::I32((pitch * 100).div_euclid(16)),
                Cell::F32(ax),
                Cell::F32(ay),
                Cell::F32(az),
            ],
        );
        if self.calibration_due.take(now_us) {
            match part.poll_calibration(port) {
                Ok(true) => log::info!(
                    "{name} :: calibration converged (mag 3) -- run `calibrate {name}` to keep it across reboots"
                ),
                Ok(false) => {}
                Err(fault) => self.outcome(name, Some(fault)),
            }
        }
        self.published.publish(AbsoluteState::of(part));
    }
}
