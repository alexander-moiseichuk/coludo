//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A barometer's `change`: its climb rate in m/s, climb positive -- the least-squares slope of the barometer's
//! ALTITUDE over its own samples of the last `WINDOW_US` (doc/specs/avionics-port.md M8, "Every barometer's
//! quantities"). A quantity new beside plan B's four, which published no rate: each barometer's run holds one
//! [`Change`], pushes what it answers as `change` and writes it as its row's fifth column (crates/controller
//! `BAROMETER_FIELDS`); M12's sim link derives the simulated barometer's the same way.
//!
//! The ALTITUDE, not the elevation: the two differ by the ground, so their slopes are one, but a re-zero of the
//! ground (`calibrate`, `update <baro> {"ground": ...}`, the DONE recovery, a warm start) steps the elevation with no
//! climb at all. Fed the altitude, `change` never sees one.
//!
//! A LEAST-SQUARES slope over the window, not the difference of two readings: every sample in the window counts, so
//! an altitude noise of sigma reads as sigma / sqrt(sum (t - tbar)^2) of slope, the sum over the window's own stamps
//! in seconds -- the more samples and the wider they spread, the quieter. Each barometer is fitted only against
//! itself, so the offset between two of them never reads as a climb.
//!
//! No reading until the window holds `MINIMUM_SAMPLES` samples spanning `MINIMUM_SPAN_US`: two samples are a
//! difference, not a fit, and a short span multiplies the noise. A gap longer than the window empties it, so a
//! barometer back from a dropout reads None until it has refilled that much: its samples from before the gap are
//! never fitted against those after it. A non-finite altitude (NaN, an infinity) is refused with the history kept:
//! it would poison every slope for a whole window.
//!
//! The ring is fixed, `CAPACITY` samples, and nothing allocates (the firmware boxes the whole, so the other members
//! of a port do not pay for it inline). The BMP581's 50 Hz keeps 26 samples in the window, the ICP-10111's ~8 Hz four
//! or five. A source faster than one sample per 16 ms would keep only its newest 32, spanning less than the window,
//! and one faster than one per 8 ms never spans `MINIMUM_SPAN_US`, so it never reads a change: no barometer here
//! runs that fast (plan B's HITL sim publishes at 50 Hz).
//!
//! Stamps are microseconds of uptime, u64, as the databoard's. The ring keeps each sample's low 32 bits, so a sample
//! is 8 bytes; the newest stamp is kept whole and a gap is judged on it, so the samples kept are never a second apart
//! and the low words' wrapping difference is exact across their 71-minute wrap. f32 throughout (decision D18): the
//! sums run on each sample's time and altitude relative to the newest's, small numbers.

/// The span of samples the slope is fitted over, in microseconds: the last half second, the newest sample's age 0.
/// A sample exactly this old is still in it.
const WINDOW_US: u32 = 500_000;

/// The fewest samples a slope is fitted to: two are a difference, not a fit.
const MINIMUM_SAMPLES: usize = 3;

/// The shortest span, oldest sample kept to the newest, a slope is fitted over, in microseconds: half the window.
const MINIMUM_SPAN_US: u32 = 250_000;

/// The samples the ring holds: the 26 a 50 Hz barometer puts in the window, and some.
const CAPACITY: usize = 32;

/// Seconds in a microsecond, as a sample's age is scaled for the fit.
const SECONDS_PER_MICROSECOND: f32 = 1e-6;

/// One sample in the ring.
#[derive(Clone, Copy, Debug, Default)]
struct Sample {
    /// Its stamp's low 32 bits, microseconds of uptime.
    stamp_us: u32,
    /// Its altitude, m AMSL.
    altitude_m: f32,
}

/// One barometer's `change`: its samples of the last `WINDOW_US`, in a fixed ring.
#[derive(Clone, Debug, Default)]
pub struct Change {
    /// The ring, the oldest kept sample at `oldest`.
    samples: [Sample; CAPACITY],
    /// The ring index of the oldest sample kept.
    oldest: usize,
    /// How many samples are kept, 0 to [`CAPACITY`].
    count: usize,
    /// The newest sample's whole stamp, microseconds of uptime; meaningless while `count` is 0.
    newest_us: u64,
}

impl Change {
    /// Takes one reading and answers the slope with it.
    ///
    /// The reading joins the window; a gap since the previous one longer than `WINDOW_US` first empties it, as does
    /// a stamp before the previous one (which the uptime never gives). With the ring full, the oldest sample makes
    /// room. Then every sample older than the window is dropped.
    ///
    /// Args:
    /// * `altitude_m` - the reading's altitude, m AMSL.
    /// * `now_us` - its stamp, microseconds of uptime.
    ///
    /// Returns the climb rate in m/s, climb positive, over the window ending at `now_us`; None while the window holds
    /// fewer than `MINIMUM_SAMPLES` samples or spans less than `MINIMUM_SPAN_US`, and for a non-finite altitude,
    /// which is refused: the window stays as it was.
    pub fn push(&mut self, altitude_m: f32, now_us: u64) -> Option<f32> {
        if !altitude_m.is_finite() {
            return None;
        }
        let gap_us = now_us.checked_sub(self.newest_us);
        if self.count > 0 && !gap_us.is_some_and(|gap_us| gap_us <= u64::from(WINDOW_US)) {
            self.count = 0;
        }
        if self.count == CAPACITY {
            self.forget_oldest();
        }
        self.samples[(self.oldest + self.count) % CAPACITY] = Sample {
            stamp_us: now_us as u32, // the low word: every sample kept is within a second of it
            altitude_m,
        };
        self.count += 1;
        self.newest_us = now_us;
        let newest_us = now_us as u32;
        while newest_us.wrapping_sub(self.samples[self.oldest].stamp_us) > WINDOW_US {
            self.forget_oldest(); // never the newest: its age is 0
        }
        self.slope()
    }

    /// Drops the oldest sample kept.
    fn forget_oldest(&mut self) {
        self.oldest = (self.oldest + 1) % CAPACITY;
        self.count -= 1;
    }

    /// The samples kept, oldest first.
    fn kept(&self) -> impl Iterator<Item = Sample> + '_ {
        (0..self.count).map(|index| self.samples[(self.oldest + index) % CAPACITY])
    }

    /// The least-squares slope of the samples kept, in m/s: sum (t - tbar)(a - abar) / sum (t - tbar)^2, with t and a
    /// taken from the newest sample's, so the f32 sums run on small numbers. None for fewer than [`MINIMUM_SAMPLES`] or
    /// a span short of [`MINIMUM_SPAN_US`]. Called with one sample kept at least.
    fn slope(&self) -> Option<f32> {
        let newest = self.samples[(self.oldest + self.count - 1) % CAPACITY];
        let span_us = newest.stamp_us.wrapping_sub(self.samples[self.oldest].stamp_us);
        if self.count < MINIMUM_SAMPLES || span_us < MINIMUM_SPAN_US {
            return None;
        }
        let relative = || {
            self.kept().map(move |sample| {
                let age_us = newest.stamp_us.wrapping_sub(sample.stamp_us);
                (
                    -(age_us as f32) * SECONDS_PER_MICROSECOND,
                    sample.altitude_m - newest.altitude_m,
                )
            })
        };
        let count = self.count as f32;
        let (time_sum, altitude_sum) = relative().fold((0.0, 0.0), |(times, altitudes), (time, altitude)| {
            (times + time, altitudes + altitude)
        });
        let (time_mean, altitude_mean) = (time_sum / count, altitude_sum / count);
        let (spread, covariance) = relative().fold((0.0, 0.0), |(spread, covariance), (time, altitude)| {
            let time = time - time_mean;
            (spread + time * time, covariance + time * (altitude - altitude_mean))
        });
        // the span is a quarter second at least, so two stamps differ and the spread is positive
        Some(covariance / spread)
    }
}

#[cfg(test)]
mod tests {
    use super::{CAPACITY, Change};

    /// The ICP-10111's cycle as the board sees it, jittered between 120 and 130 ms: one step per reading, in us.
    const ICP_STEPS_US: [u64; 7] = [120_000, 127_000, 130_000, 123_000, 125_000, 121_000, 129_000];

    /// A stamp 0.6 s short of the low word's wrap, so a run started there fits samples across it.
    const BEFORE_THE_WRAP_US: u64 = (1 << 32) - 600_000;

    /// An altitude well above the sea, so the f32 altitudes carry few fractional bits.
    const PAD_M: f32 = 1234.5;

    /// What `change` answers to a climb at `rate` m/s from [`PAD_M`] on `steps` (cycled), `readings` of them from
    /// `start_us`: each reading's stamp and slope.
    fn climb(rate: f32, start_us: u64, steps: &[u64], readings: usize) -> Vec<(u64, Option<f32>)> {
        let mut change = Change::default();
        let mut now_us = start_us;
        let mut answers = Vec::new();
        for step in steps.iter().cycle().take(readings) {
            let seconds = (now_us - start_us) as f32 * 1e-6;
            answers.push((now_us, change.push(PAD_M + rate * seconds, now_us)));
            now_us += step;
        }
        answers
    }

    /// The deterministic noise of the spread test: xorshift64*, then Box-Muller to a unit normal.
    struct Noise(u64);

    impl Noise {
        /// The next uniform in (0, 1].
        fn uniform(&mut self) -> f64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            ((self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) + 1) as f64 / (1_u64 << 53) as f64
        }

        /// The next unit normal.
        fn normal(&mut self) -> f64 {
            let (radius, angle) = (self.uniform(), self.uniform());
            (-2.0 * radius.ln()).sqrt() * (std::f64::consts::TAU * angle).cos()
        }
    }

    #[test]
    fn a_climb_on_the_icp_s_jittered_cycle_reads_its_rate() {
        // POSITIVE: 10 m/s on 120..130 ms stamps that run across the low word's wrap. The first reading is alone, the
        // second makes two, the third three spanning 247 ms: no slope yet. From the fourth on, the rate.
        let answers = climb(10.0, BEFORE_THE_WRAP_US, &ICP_STEPS_US, 40);
        assert_eq!(
            answers[..3].iter().map(|(_, slope)| *slope).collect::<Vec<_>>(),
            [None; 3]
        );
        assert!(answers.last().unwrap().0 > 1 << 32, "the run crossed the wrap");
        for (now_us, slope) in &answers[3..] {
            let slope = slope.expect("three samples over a quarter second");
            assert!((slope - 10.0).abs() <= 0.01, "{slope} at {now_us}");
        }
    }

    #[test]
    fn a_descent_reads_negative() {
        // climb positive: a sink at 7.5 m/s reads -7.5
        for (now_us, slope) in &climb(-7.5, 0, &ICP_STEPS_US, 20)[3..] {
            let slope = slope.expect("a slope");
            assert!((slope + 7.5).abs() <= 0.01, "{slope} at {now_us}");
        }
    }

    #[test]
    fn a_constant_altitude_reads_no_change() {
        for (now_us, slope) in &climb(0.0, 7_000_000, &[20_000], 60)[13..] {
            assert_eq!(*slope, Some(0.0), "at {now_us}");
        }
    }

    #[test]
    fn the_noise_reads_as_the_least_squares_spread() {
        // POSITIVE: independent windows of a 50 Hz barometer at rest (26 samples, 0..500 ms), each altitude noised with
        // sigma: the slopes' sd is sigma / sqrt(sum (t - tbar)^2) over the window's stamps, their mean 0
        const TRIALS: usize = 4000;
        const SIGMA_M: f64 = 0.1;
        let stamps_s: Vec<f64> = (0..=25).map(|index| f64::from(index) * 0.02).collect();
        let mean_s = stamps_s.iter().sum::<f64>() / stamps_s.len() as f64;
        let spread: f64 = stamps_s.iter().map(|stamp| (stamp - mean_s).powi(2)).sum();
        let expected = SIGMA_M / spread.sqrt();
        let mut noise = Noise(0x0c01_ad00_2026_1008);
        let slopes: Vec<f64> = (0..TRIALS)
            .map(|trial| {
                let mut change = Change::default();
                let start_us = 1_000_000 + trial as u64 * 3_000_000;
                let mut slope = None;
                for stamp_s in &stamps_s {
                    let altitude = f64::from(PAD_M) + SIGMA_M * noise.normal();
                    slope = change.push(altitude as f32, start_us + (stamp_s * 1e6).round() as u64);
                }
                f64::from(slope.expect("26 samples over the whole window"))
            })
            .collect();
        let mean = slopes.iter().sum::<f64>() / TRIALS as f64;
        let deviation = (slopes.iter().map(|slope| (slope - mean).powi(2)).sum::<f64>() / (TRIALS - 1) as f64).sqrt();
        // the sd of an sd over 4000 trials is ~1.1 %; 4 % is beyond 3.5 of those
        assert!(
            (deviation / expected - 1.0).abs() < 0.04,
            "sd {deviation} against {expected}"
        );
        assert!(mean.abs() < 4.0 * expected / (TRIALS as f64).sqrt(), "mean {mean}");
    }

    #[test]
    fn two_samples_are_no_slope() {
        // NEGATIVE: two readings 300 ms apart span enough, but two are a difference, not a fit
        let mut change = Change::default();
        assert_eq!(change.push(PAD_M, 1_000_000), None);
        assert_eq!(change.push(PAD_M + 3.0, 1_300_000), None);
        assert!(
            change.push(PAD_M + 6.0, 1_400_000).is_some(),
            "the third, 400 ms after the first, fits"
        );
    }

    #[test]
    fn three_samples_must_span_a_quarter_second() {
        let mut short = Change::default();
        short.push(PAD_M, 0);
        short.push(PAD_M + 1.0, 100_000);
        assert_eq!(short.push(PAD_M + 2.4999, 249_999), None, "249.999 ms");
        let mut quarter = Change::default();
        quarter.push(PAD_M, 0);
        quarter.push(PAD_M + 1.0, 100_000);
        let slope = quarter.push(PAD_M + 2.5, 250_000).expect("250 ms");
        assert!((slope - 10.0).abs() <= 0.01, "{slope}");
    }

    #[test]
    fn the_window_keeps_a_sample_exactly_its_length_old() {
        let mut edge = Change::default();
        edge.push(PAD_M, 0);
        edge.push(PAD_M + 2.5, 250_000);
        assert!(
            edge.push(PAD_M + 5.0, 500_000).is_some(),
            "the first is 500 ms old: still in"
        );
        let mut past = Change::default();
        past.push(PAD_M, 0);
        past.push(PAD_M + 2.5, 250_000);
        assert_eq!(
            past.push(PAD_M + 5.0, 500_001),
            None,
            "the first is 500.001 ms old: out, two left"
        );
    }

    #[test]
    fn a_gap_longer_than_the_window_empties_it_and_it_recovers() {
        // NEGATIVE then POSITIVE: a climb at 10 m/s, a 600 ms dropout, and the barometer back 20 m higher than the
        // climb would put it. Fitted across the gap, the step would read as a climb; instead the first reading after
        // it stands alone and the slope comes back once three readings span a quarter second, at the rate.
        let mut change = Change::default();
        let mut now_us = 5_000_000;
        for index in 0..8 {
            change.push(PAD_M + index as f32 * 1.25, now_us);
            now_us += 125_000;
        }
        let last_us = now_us - 125_000;
        let mut now_us = last_us + 600_000;
        let altitude = |now_us: u64| PAD_M + 20.0 + (now_us - 5_000_000) as f32 * 1e-5;
        assert_eq!(change.push(altitude(now_us), now_us), None, "alone after the gap");
        now_us += 125_000;
        assert_eq!(change.push(altitude(now_us), now_us), None, "two after the gap");
        now_us += 125_000;
        let slope = change.push(altitude(now_us), now_us).expect("three spanning 250 ms");
        assert!(
            (slope - 10.0).abs() <= 0.01,
            "{slope}: the samples before the gap are gone"
        );
    }

    #[test]
    fn a_non_finite_altitude_is_refused_and_the_history_kept() {
        // NEGATIVE: NaN and the infinities answer None and leave the window as it was, so the next reading is fitted
        // exactly as by a twin that never saw them
        let mut refusing = Change::default();
        let mut twin = Change::default();
        for (index, now_us) in (0..5_u64).map(|index| (index, index * 125_000)) {
            let altitude = PAD_M + index as f32 * 1.25;
            assert_eq!(refusing.push(altitude, now_us), twin.push(altitude, now_us));
        }
        for (offset_us, bad) in [(10_000, f32::NAN), (20_000, f32::INFINITY), (30_000, f32::NEG_INFINITY)] {
            assert_eq!(refusing.push(bad, 500_000 + offset_us), None, "{bad}");
        }
        let next = refusing.push(PAD_M + 6.25, 625_000);
        assert_eq!(next, twin.push(PAD_M + 6.25, 625_000));
        assert!((next.expect("the history kept") - 10.0).abs() <= 0.01);
    }

    #[test]
    fn a_stamp_before_the_newest_restarts_it() {
        // NEGATIVE: the uptime never steps back; a stamp that does starts the window afresh, as a gap does
        let mut change = Change::default();
        for index in 0..5_u64 {
            change.push(PAD_M + index as f32, 1_000_000 + index * 125_000);
        }
        assert_eq!(change.push(PAD_M, 1_400_000), None, "behind the newest (1.5 s): alone");
        assert_eq!(change.push(PAD_M + 1.25, 1_525_000), None);
        assert!(change.push(PAD_M + 2.5, 1_650_000).is_some(), "refilled");
    }

    #[test]
    fn a_source_faster_than_the_ring_fits_its_newest_samples() {
        // 100 Hz overruns the ring: the 26th reading spans a quarter second, and from the 33rd on the newest 32 span
        // 310 ms, and read the rate
        let fast = climb(10.0, 0, &[10_000], 80);
        assert_eq!(fast[24].1, None, "25 readings span 240 ms");
        assert!(
            fast[25..]
                .iter()
                .all(|(_, slope)| slope.is_some_and(|slope| (slope - 10.0).abs() <= 0.01))
        );
        // NEGATIVE, the documented limit: at 200 Hz the newest 32 span 155 ms, so no slope ever
        assert!(climb(10.0, 0, &[5_000], 200).iter().all(|(_, slope)| slope.is_none()));
    }

    #[test]
    fn a_sample_is_eight_bytes() {
        // the ring is the bulk of it (the firmware boxes one per barometer, on the internal heap)
        assert!(
            size_of::<Change>() <= CAPACITY * 8 + 3 * size_of::<u64>(),
            "{}",
            size_of::<Change>()
        );
    }
}
