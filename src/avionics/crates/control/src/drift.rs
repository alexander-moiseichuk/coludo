//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The GNSS pad drift (plan B tasks/gnss_calib.py). A stationary receiver's position walks slowly (the satellite
//! geometry, the ionosphere, multipath), and over a minute on the pad the walk is close to a constant velocity
//! that the minute of flight keeps. A fixed antenna is not carried by the wind, so on the pad the whole reported
//! ground velocity is that drift: averaged through SETTING and frozen at launch, it is subtracted from the GNSS
//! ground velocity before the wind triangle, where it would otherwise read as wind (the flight loop, M13).
//!
//! One [`DriftCalibrator::pass`] per `period_ms` (1 s): in SETTING the fused speed and course, both fresh, fold into
//! the mean, restarted once at the first armed pass (arming is the operator saying "on the rail, still"; before it
//! the airframe was carried out, a walk and no drift); off the pad (BOOSTING on, a warm start into GLIDING included)
//! the mean freezes when it has `min_samples`, else the drift stays zero. Frozen is final.
//!
//! One divergence: plan B divides by its count when `min_samples` is 0 and nothing was sampled (the task dies on
//! the ZeroDivisionError); the port freezes a zero drift there.

use sequencer::Stage;

/// The pad drift calibration's state.
#[derive(Clone, Debug, PartialEq)]
pub struct DriftCalibrator {
    /// Fewer samples than this freeze a zero drift (`min_samples`).
    min_samples: u32,
    /// The sampled ground velocity's east and north sums, m/s.
    sums: [f32; 2],
    /// The samples since the restart.
    count: u32,
    /// The first armed pass restarted the sums.
    armed_seen: bool,
    /// The frozen drift, east and north, m/s: zero until the freeze, and when too few samples came.
    drift: [f32; 2],
    /// The drift froze.
    frozen: bool,
}

impl DriftCalibrator {
    /// A calibrator that freezes a mean of at least `min_samples` samples.
    pub fn new(min_samples: u32) -> DriftCalibrator {
        DriftCalibrator {
            min_samples,
            sums: [0.0; 2],
            count: 0,
            armed_seen: false,
            drift: [0.0; 2],
            frozen: false,
        }
    }

    /// One pass (plan B's run loop body): `stage` and `armed` as the flight word holds them, `ground` the fused
    /// GNSS speed (m/s) and course (degrees) when both are fresh.
    ///
    /// Returns the frozen drift and its sample count on the pass that froze it, for the log line.
    pub fn pass(&mut self, stage: Stage, armed: bool, ground: Option<[f32; 2]>) -> Option<([f32; 2], u32)> {
        if self.frozen {
            return None;
        }
        if stage != Stage::Setting {
            if self.count >= self.min_samples && self.count > 0 {
                let count = self.count as f32;
                self.drift = self.sums.map(|sum| sum / count);
            }
            self.frozen = true;
            return Some((self.drift, self.count));
        }
        if armed && !self.armed_seen {
            self.armed_seen = true;
            self.sums = [0.0; 2];
            self.count = 0;
        }
        if let Some([speed, course]) = ground {
            let (east, north) = course.to_radians().sin_cos();
            self.sums[0] += speed * east;
            self.sums[1] += speed * north;
            self.count += 1;
        }
        None
    }

    /// The frozen drift, east and north, m/s; zero until the freeze (plan B `drift()`).
    pub fn drift(&self) -> [f32; 2] {
        self.drift
    }

    /// The samples since the restart.
    pub fn samples(&self) -> u32 {
        self.count
    }

    /// Whether the drift froze.
    pub fn frozen(&self) -> bool {
        self.frozen
    }
}
