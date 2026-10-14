//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage machine's detector primitives, from src/glider commons.py: the leaky dwell ([`dwell_step`]), the
//! apogee tracker ([`apogee_step`]) and the squared magnitude the accel thresholds compare
//! ([`magnitude_squared`]). The flight-math half of commons.py (the fin table, `altitude_m`) is in crates/control,
//! module `commons`.
//!
//! Pure and stateless: the caller owns the state ([`Dwell`], [`Apogee`]) and hands it back on every call. Plan B
//! made them so because each detector lived in two worlds -- the board's sequencer and the host sim
//! (`tools/virtual_flight.py`) -- and the sim's hand-kept mirror had drifted: its apogee had decayed to a
//! timeout-only deploy, and its launch dwell was the copy the 100 % noise study had really been grading. One
//! implementation per detector, no mirror; the golden fixture commons.json holds this port to plan B's.
//!
//! Time is the caller's monotonic clock in milliseconds, a `u32`, and every interval is the signed wrapping
//! difference [`elapsed_ms`]: MicroPython's `ticks_diff`, at a 2^32 period instead of 2^30. A signed interval is
//! what plan B compares, so a wrap of the clock costs nothing, and a clock that steps backwards (the plan B tests
//! feed one) never satisfies a [`reached`] threshold -- the timeouts, the arming window, the contiguous dwells.
//! The leaky dwell is the exception: there a backwards step turns the step's sign, draining an active dwell but
//! FEEDING an inactive one, which can complete it ([`dwell_step`]).
//!
//! f32 throughout (decision D18). The decisions compare metres and g against thresholds of whole metres and
//! tenths of a g, and an f32 holds an elevation to 3e-5 m at 500 m.

/// The IIR weight divisor of the apogee tracker: the smoothed elevation moves a quarter of the way to each new
/// sample (plan B `commons.APOGEE_SMOOTH`).
///
/// A running maximum latches any single high spike forever, so the peak -- not just the descent -- needs noise
/// immunity. It costs about 4 samples of lag, far inside `apogee_drop_m` of altitude.
pub const APOGEE_SMOOTH: f32 = 4.0;

/// The signed milliseconds from `then_ms` to `now_ms` on a wrapping `u32` clock: negative when `now_ms` is the
/// earlier one.
///
/// MicroPython's `ticks_diff` at a 2^32 period. Plan B chose `ticks_diff` over a raw subtraction because a bench
/// board left in SETTING across the 2^30 wrap would otherwise see a huge positive step and satisfy any dwell at
/// once -- a false launch from arithmetic, on the detector that starts the flight.
pub fn elapsed_ms(now_ms: u32, then_ms: u32) -> i32 {
    now_ms.wrapping_sub(then_ms) as i32
}

/// Whether `elapsed_ms(now_ms, since_ms)` has reached `threshold_ms`: the comparison every dwell, arming window
/// and timeout of the stage machine makes. Widened to i64 so a negative interval (a clock stepping backwards)
/// compares below every threshold instead of wrapping into a large one.
pub fn reached(now_ms: u32, since_ms: u32, threshold_ms: u32) -> bool {
    i64::from(elapsed_ms(now_ms, since_ms)) >= i64::from(threshold_ms)
}

/// |(x, y, z)|^2: the accel magnitude squared, in g^2 (plan B `commons.magnitude_sq`).
///
/// Squared so the thresholds compare without a root; only the rare transition reason takes one.
pub fn magnitude_squared([x, y, z]: [f32; 3]) -> f32 {
    x * x + y * y + z * z
}

/// The state of one leaky dwell (plan B's `(credit_ms, last_ms)` pair).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Dwell {
    /// The net time the condition has held so far, in ms; never above the threshold.
    pub credit_ms: u32,
    /// The clock of the previous sample, or None when no dwell is in progress.
    pub last_ms: Option<u32>,
}

/// One step of a LEAKY dwell: the condition must hold `threshold_ms` of NET time, and a sample that misses only
/// drains the credit (plan B `commons.dwell_step`).
///
/// The plain "sustained since" dwell it replaced reset its start on any single sample that missed. That is fine
/// for a clean signal and useless for a noisy one: at 100 % accel noise the magnitude crosses the launch
/// threshold in both directions every few samples, so an unbroken run of `launch_ms` never happens, and
/// MEASURED (doc/sims/TMS-7-noise_tolerance) the glider never left the pad at all.
///
/// Draining rather than resetting keeps the tolerance without changing the UNIT. That distinction cost plan B a
/// revert: a leaky count of samples also worked, but it silently made `launch_ms` a sample count, so the
/// trigger's timing drifted with the tick rate. Time in, time out: a `threshold_ms` of 100 still means 100 ms,
/// and a lone spike still drains to nothing and resets.
///
/// The corners are plan B's too: the first active sample starts the clock at zero credit (a dwell never fires
/// on its first sample, even with a zero threshold); a credit that drains to zero or below ends the dwell, so
/// a repeated instant with no credit ends it as well; and a clock stepping backwards turns the step's sign, so
/// it drains an active dwell and feeds an inactive one.
///
/// Returns the new state and whether the dwell is satisfied.
pub fn dwell_step(active: bool, now_ms: u32, dwell: Dwell, threshold_ms: u32) -> (Dwell, bool) {
    let Some(last_ms) = dwell.last_ms else {
        // no dwell in progress: an active sample starts the clock, at zero credit
        let last_ms = active.then_some(now_ms);
        return (Dwell { credit_ms: 0, last_ms }, false);
    };
    let step = i64::from(elapsed_ms(now_ms, last_ms));
    let credit = i64::from(dwell.credit_ms) + if active { step } else { -step };
    if credit <= 0 {
        return (Dwell::default(), false); // fully drained: the dwell is over and starts afresh
    }
    let threshold = i64::from(threshold_ms);
    // the credit is positive and capped at a u32 threshold, so it fits
    let credit_ms = credit.min(threshold) as u32;
    (
        Dwell {
            credit_ms,
            last_ms: Some(now_ms),
        },
        credit >= threshold,
    )
}

/// The state of the apogee tracker (plan B's `(peak, since_ms, smooth)` triple).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Apogee {
    /// The highest SMOOTHED elevation seen so far, or None before the first reading.
    pub peak: Option<f32>,
    /// When the fall below `peak - drop_m` began, or None while not below it.
    pub since_ms: Option<u32>,
    /// The smoothed elevation carried from the previous reading, or None to seed it.
    pub smooth: Option<f32>,
}

/// One step of APOGEE detection: track the smoothed baro peak and report when it has fallen `drop_m` below it
/// for `dwell_ms` (plan B `commons.apogee_step`).
///
/// SMOOTH BEFORE COMPARING, because the peak is a running MAXIMUM and a maximum has no noise immunity at all:
/// one high sample is latched forever, and every later reading is judged against that spike instead of the
/// trajectory. MEASURED at 100 % baro noise: apogee fired 2.8 s early, separating at 168 m instead of 268 m and
/// cutting the flight from 48.8 s to 32.8 s. The drop band rejects noise on the descending side; the smoothing
/// protects the peak itself. A first-order IIR (weight 1/[`APOGEE_SMOOTH`]) needs one scalar of state where a
/// sample window needs a ring buffer.
///
/// No reading (`elevation` None: not armed yet, or no fresh source) changes nothing -- an open dwell stays open
/// through a baro dropout, and fires on the first fresh sample still below the line once `dwell_ms` has passed.
/// The caller keeps the arming window and the burnout fallback, which need its own clock and stage entry.
///
/// A NaN elevation poisons the smoother for good, as in plan B: every comparison with it is false, so the
/// tracker never fires again and the burnout timeout is the way out.
///
/// Returns the new state and true exactly when apogee is confirmed.
pub fn apogee_step(elevation: Option<f32>, now_ms: u32, apogee: Apogee, drop_m: f32, dwell_ms: u32) -> (Apogee, bool) {
    let Some(elevation) = elevation else {
        return (apogee, false);
    };
    let smooth = match apogee.smooth {
        None => elevation,
        Some(previous) => previous + (elevation - previous) / APOGEE_SMOOTH,
    };
    let state = |peak: f32, since_ms: Option<u32>| Apogee {
        peak: Some(peak),
        since_ms,
        smooth: Some(smooth),
    };
    match apogee.peak {
        // still climbing: raise the peak, clear the dwell
        None => (state(smooth, None), false),
        Some(peak) if smooth > peak => (state(smooth, None), false),
        // fallen off the peak: descending, for as long as the dwell asks
        Some(peak) if smooth < peak - drop_m => {
            let since_ms = apogee.since_ms.unwrap_or(now_ms);
            (state(peak, Some(since_ms)), reached(now_ms, since_ms, dwell_ms))
        }
        // inside the drop band: noise, not a descent yet
        Some(peak) => (state(peak, None), false),
    }
}

#[cfg(test)]
mod tests {
    use super::{Apogee, Dwell, apogee_step, dwell_step, elapsed_ms, magnitude_squared, reached};

    /// A running dwell: `credit_ms` of credit, the previous sample at `last_ms`.
    fn running(credit_ms: u32, last_ms: u32) -> Dwell {
        Dwell {
            credit_ms,
            last_ms: Some(last_ms),
        }
    }

    #[test]
    fn elapsed_is_signed_and_survives_the_wrap() {
        assert_eq!(elapsed_ms(1100, 1000), 100);
        assert_eq!(elapsed_ms(1000, 1100), -100);
        assert_eq!(elapsed_ms(50, u32::MAX - 49), 100);
        assert!(reached(50, u32::MAX - 49, 100));
        assert!(
            !reached(1000, 1100, 0),
            "a clock stepping backwards reaches no threshold"
        );
    }

    #[test]
    fn magnitude_is_squared() {
        assert_eq!(magnitude_squared([0.0, 3.0, 4.0]), 25.0);
        assert_eq!(magnitude_squared([0.0, 0.0, -2.0]), 4.0);
    }

    #[test]
    fn a_dwell_starts_accrues_and_fires_on_net_time() {
        let (started, fired) = dwell_step(true, 1000, Dwell::default(), 100);
        assert_eq!((started, fired), (running(0, 1000), false));
        let (half, fired) = dwell_step(true, 1050, started, 100);
        assert_eq!((half, fired), (running(50, 1050), false));
        let (full, fired) = dwell_step(true, 1300, half, 100);
        assert_eq!((full, fired), (running(100, 1300), true), "capped at the threshold");
    }

    #[test]
    fn a_dip_drains_and_a_long_one_ends_the_dwell() {
        assert_eq!(
            dwell_step(false, 1110, running(80, 1100), 100),
            (running(70, 1110), false)
        );
        assert_eq!(
            dwell_step(false, 1180, running(80, 1100), 100),
            (Dwell::default(), false)
        );
        assert_eq!(
            dwell_step(false, 1200, running(80, 1100), 100),
            (Dwell::default(), false)
        );
    }

    #[test]
    fn an_idle_dwell_ignores_an_inactive_sample_and_a_leftover_credit() {
        let leftover = Dwell {
            credit_ms: 70,
            last_ms: None,
        };
        assert_eq!(dwell_step(false, 1000, leftover, 100), (Dwell::default(), false));
        assert_eq!(dwell_step(true, 1000, leftover, 100), (running(0, 1000), false));
    }

    #[test]
    fn the_corners_plan_b_reaches() {
        // a repeated instant with no credit ends the dwell; a zero threshold fires on the SECOND sample only
        assert_eq!(dwell_step(true, 1000, running(0, 1000), 100), (Dwell::default(), false));
        assert_eq!(dwell_step(true, 1000, Dwell::default(), 0), (running(0, 1000), false));
        assert_eq!(dwell_step(true, 1010, running(0, 1000), 0), (running(0, 1010), true));
        // a clock stepping backwards drains an active dwell and feeds an inactive one
        assert_eq!(dwell_step(true, 990, running(50, 1000), 100), (running(40, 990), false));
        assert_eq!(
            dwell_step(false, 950, running(80, 1000), 100),
            (running(100, 950), true)
        );
    }

    #[test]
    fn a_huge_step_cannot_overflow_the_credit() {
        let (dwell, fired) = dwell_step(true, u32::MAX / 2, running(u32::MAX, 0), u32::MAX);
        assert!(fired);
        assert_eq!(dwell.credit_ms, u32::MAX);
        let (dwell, fired) = dwell_step(false, 0, running(10, u32::MAX / 2 + 1), 100);
        assert!(
            fired,
            "a step of -2^31 ms reads as a clock 24.8 days backwards, feeding the dwell"
        );
        assert_eq!(dwell.credit_ms, 100);
    }

    #[test]
    fn apogee_seeds_climbs_and_holds_the_band() {
        let (seeded, fired) = apogee_step(Some(240.0), 2200, Apogee::default(), 5.0, 100);
        assert!(!fired);
        assert_eq!(seeded.peak, Some(240.0), "seeded, not smoothed toward from zero");
        let (band, fired) = apogee_step(Some(233.0), 2210, seeded, 5.0, 100);
        assert!(!fired);
        assert_eq!(
            (band.peak, band.since_ms, band.smooth),
            (Some(240.0), None, Some(238.25))
        );
        let (climb, _) = apogee_step(Some(260.0), 2220, band, 5.0, 100);
        assert_eq!(climb.peak, climb.smooth, "a rise lifts the peak");
    }

    #[test]
    fn apogee_fires_once_the_fall_has_held() {
        let falling = Apogee {
            peak: Some(240.0),
            since_ms: None,
            smooth: Some(236.0),
        };
        let (opened, fired) = apogee_step(Some(200.0), 2300, falling, 5.0, 100);
        assert_eq!((opened.since_ms, fired), (Some(2300), false));
        let (held, fired) = apogee_step(Some(200.0), 2399, opened, 5.0, 100);
        assert_eq!((held.since_ms, fired), (Some(2300), false));
        let (_, fired) = apogee_step(Some(200.0), 2400, held, 5.0, 100);
        assert!(fired);
        // and a rise back into the band (smoothed 220.25 -> 237.69) clears the dwell instead
        let (back, fired) = apogee_step(Some(290.0), 2400, held, 5.0, 100);
        assert_eq!((back.peak, back.since_ms, fired), (Some(240.0), None, false));
    }

    #[test]
    fn the_drop_line_itself_is_still_the_band() {
        // 236 + (232 - 236) / 4 is exactly 235.0, exactly 240 - 5: not below the line, no dwell
        let near = Apogee {
            peak: Some(240.0),
            since_ms: None,
            smooth: Some(236.0),
        };
        let (on_the_line, fired) = apogee_step(Some(232.0), 2300, near, 5.0, 100);
        assert_eq!(
            (on_the_line.smooth, on_the_line.since_ms, fired),
            (Some(235.0), None, false)
        );
        // and 230 smooths to 233.75, below it: the dwell opens
        let (below, fired) = apogee_step(Some(230.0), 2350, on_the_line, 5.0, 100);
        assert_eq!((below.smooth, below.since_ms, fired), (Some(233.75), Some(2350), false));
    }

    #[test]
    fn no_reading_keeps_an_open_dwell() {
        let open = Apogee {
            peak: Some(120.0),
            since_ms: Some(2250),
            smooth: Some(110.0),
        };
        assert_eq!(apogee_step(None, 2400, open, 5.0, 100), (open, false));
        let (_, fired) = apogee_step(Some(105.0), 2400, open, 5.0, 100);
        assert!(fired, "the first fresh sample after the gap fires");
    }

    #[test]
    fn a_nan_elevation_poisons_the_tracker_for_good() {
        let falling = Apogee {
            peak: Some(240.0),
            since_ms: Some(2300),
            smooth: Some(226.0),
        };
        let (poisoned, fired) = apogee_step(Some(f32::NAN), 2400, falling, 5.0, 100);
        assert!(!fired);
        assert_eq!(poisoned.since_ms, None);
        assert!(poisoned.smooth.unwrap().is_nan());
        let (after, fired) = apogee_step(Some(100.0), 3000, poisoned, 5.0, 100);
        assert!(!fired && after.smooth.unwrap().is_nan(), "never fires again");
    }
}
