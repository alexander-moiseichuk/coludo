//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The PID controller of one control axis (src/glider pid.py), sibling of [`crate::mixer`]: an angle error in,
//! a fin deflection command out, both in degrees. One instance per axis (roll, pitch, yaw); [`Pid::reset`] on
//! (re)entering a control stage.
//!
//! Anti-windup is TWO mechanisms, as in plan B:
//!  * the integral and output CLAMPS bound the magnitude, and follow the governor's live fin cap through
//!    [`Pid::set_limit`], so the integral cannot wind up to a stale 45 degrees behind a tight 5 degree cap and
//!    dump a saturating deflection when the cap reopens;
//!  * BACK-CALCULATION bleeds the integral by whatever demand the saturated fin could not fly, and stops at
//!    zero rather than cross it (see [`Pid::step`], which also records where that guard does not hold).
//!
//! f32 degrees and seconds (decision D18). Plan B ran this law in fixed point (fixed.py fixnums: centidegree
//! errors and outputs, integer-millisecond time, gains scaled by 100) for one reason: MicroPython boxes a heap
//! float on every float operation, and with the GC disabled in flight the old float PID leaked 176 B a step,
//! about 56 KB/s over three axes at 100 Hz. Rust allocates nothing either way, so this computes the real-number
//! law those integers stood for. What the fixed point added on top is NOT ported, because it is quantisation and
//! not behaviour:
//!  * every result floored to 0.01 (degree, degree-second, degree per second): up to 0.0141 degrees of output a
//!    step against this port, measured over the golden fixture pid.json (its tolerance is the plan's 0.02, with
//!    gains up to 3 / 0.4 / 0.4); the port's own f32 rounding adds nothing measurable at that scale;
//!  * gains truncated to 0.01 by `int(gain * 100)`: `kp=0.29` flew as 0.28, since 0.29 x 100 is 28.999999999999996
//!    in binary, and any gain under 0.01 as 0;
//!  * the integral ceiling of 2^28 centidegree-milliseconds (2684 degree-seconds) under the "unbounded" default,
//!    a small-int bound no flight reaches (the limit follows the 45 degree cap);
//!  * the floor of a positive integral under 0.01 degree-seconds to zero, which made the stop-at-zero guard
//!    below take its wrong branch and wind the integral AGAINST the error (plan B: a 0.1 degree error at kp=100
//!    against a 5 degree cap drove it to -1.25 degree-seconds in one step). The real law keeps that integral
//!    positive and stops it at zero.
//!
//! Plan B's `dt_ms` was `dt_us // 1000`, so a sub-millisecond step counted as no time at all; here the elapsed
//! time is the caller's seconds, as measured. Nothing reads a clock: the caller passes the time.

use crate::commons::between;

/// The back-calculation strength plan B ships (pid._ANTI_WINDUP_SHIFT): each saturated step bleeds the integral
/// by 1/2^2 = 1/4 of the demand the fin could not fly, so it eases out rather than snapping.
pub const ANTI_WINDUP_SHIFT: u32 = 2;

/// The deepest anti-windup shift with a non-zero f32 factor: 2^-149 is the smallest subnormal, and any deeper
/// shift bleeds nothing (the clamp-only behaviour a shift of 30 already gives).
const ANTI_WINDUP_SHIFT_FLOOR: u32 = 149;

/// The three gains of one axis (pid.Pid `kp`, `ki`, `kd`): output degrees per degree of error, per
/// degree-second of integral, and per degree per second of rate. Zero by default, as in plan B, and every
/// shipped config leaves the `gains` block empty, so ki is 0 in flight today.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Gains {
    /// kp: degrees of fin per degree of error.
    pub proportional: f32,
    /// ki: degrees of fin per degree-second of accumulated error. Zero turns back-calculation off: there is no
    /// integral to unwind.
    pub integral: f32,
    /// kd: degrees of fin per degree per second of angular rate.
    pub derivative: f32,
}

/// A PID controller for one control axis: error (degrees) -> fin deflection command (degrees).
///
/// `kp * error + ki * integral + kd * derivative`, each part clamped. The derivative is the measured rate (the
/// gyro) when the caller has a fresh one -- derivative-on-measurement, clean and with no kick when the setpoint
/// steps -- else the change of the error over the elapsed time.
#[derive(Clone, Debug, PartialEq)]
pub struct Pid {
    /// The axis gains, fixed at construction.
    gains: Gains,
    /// The back-calculation factor 2^-shift: the fraction of the unflyable demand bled per saturated step.
    anti_windup_factor: f32,
    /// The integral clamp, in degree-seconds: the integral never leaves [-limit, limit].
    integral_limit: f32,
    /// The output clamp, in degrees: the fin authority the command may use.
    output_limit: f32,
    /// The accumulated error, in degree-seconds: what the ki term consumes.
    integral: f32,
    /// The previous step's error, in degrees; None until the first step after construction or a reset, so the
    /// first step takes no derivative (a 0 baseline would read the whole error as one step's change: a large
    /// spurious kick on entry).
    previous: Option<f32>,
}

impl Pid {
    /// A controller with these gains and limits (pid.Pid).
    ///
    /// Args: `integral_limit` in degree-seconds and `output_limit` in degrees, both symmetric (plan B defaults
    /// both to 1e6 degrees, "unbounded"; the flight task passes the mixer limit); `anti_windup_shift` the
    /// back-calculation strength, [`ANTI_WINDUP_SHIFT`] in flight (a large shift such as 30 bleeds next to
    /// nothing: the clamp-only behaviour).
    pub fn new(gains: Gains, integral_limit: f32, output_limit: f32, anti_windup_shift: u32) -> Pid {
        Pid {
            gains,
            anti_windup_factor: 0.5_f32.powi(anti_windup_shift.min(ANTI_WINDUP_SHIFT_FLOOR + 1) as i32),
            integral_limit,
            output_limit,
            integral: 0.0,
            previous: None,
        }
    }

    /// Clear the integral and the derivative history (pid.Pid.reset).
    ///
    /// On entering a control stage, so a fresh glide does not inherit wind-up from an earlier one. The previous
    /// error is forgotten too, so the first step after a reset takes no derivative.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous = None;
    }

    /// Retune the output clamp and the integral clamp to the live fin authority, in whole degrees
    /// (pid.Pid.set_limit).
    ///
    /// The governor rewrites the fin cap by dynamic pressure every update; driving BOTH clamps from that same cap
    /// stops the integral winding up to a stale 45 degrees behind a tight (say 5 degree) cap and then dumping a
    /// saturating deflection when the cap reopens (overshoot). The integral is clamped to the new limit at the
    /// next step, not here, as in plan B.
    pub fn set_limit(&mut self, limit: i32) {
        self.output_limit = limit as f32;
        self.integral_limit = limit as f32;
    }

    /// The integral's state in degree-seconds: how much accumulated error the ki term is acting on. For the
    /// tests and the operator's view of wind-up; the control law never needs to read it.
    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// One control step (pid.Pid.step): the fin command in degrees for this error, clamped to the output limit.
    ///
    /// Args: `error` the setpoint minus the measured angle, degrees; `elapsed` the seconds since the previous
    /// step (the caller's measured interval, not the nominal one); `rate` the fresh measured angular rate of this
    /// axis, degrees per second, or None without a gyro (or with a stale one: a stale rate in a derivative damps
    /// against a motion that already ended), which falls back to differentiating the error. With no previous
    /// error, or no time elapsed, that fallback takes no derivative this step.
    ///
    /// BACK-CALCULATION. The clamps bound the integral's MAGNITUDE but never unwind it: while the fin is
    /// saturated, the integral sits at its limit demanding what the airframe cannot fly, and dumps the moment
    /// authority returns. So when the output saturates (and ki is not 0), the integral is bled by the part of the
    /// demand that did NOT reach the fin, converted back into integral units through ki and scaled by 2^-shift.
    /// This matters most on a WARM START: after a mid-air reset the fins sit wherever the servos landed while the
    /// GPIO floated -- possibly near 90 degrees to the airflow, an airbrake rather than a control surface -- and
    /// the loop restarts against a large sustained disturbance under a tight unconfident cap: deep saturation
    /// for many steps, exactly where a clamp-only integral winds to its bound.
    ///
    /// STOP AT ZERO, plan B's deliberate deviation from textbook back-calculation: the textbook law drives the
    /// integral until it CANCELS the excess, so 30 degrees of error against a 5 degree cap settled it at the
    /// opposite sign (plan B measured -1067 centidegree-seconds where the error was +3000 centidegrees), which
    /// commands a REVERSED fin the instant the cap reopens. A positive integral is therefore bled no lower than
    /// zero. Kept exactly as plan B writes it, `max(0, unwound) if integral > 0 else min(0, unwound)`, which
    /// guards only an integral moving TOWARD zero:
    ///  * an integral of exactly 0 takes the `else` branch, so a POSITIVE saturation (a D-term kick with the
    ///    error on the setpoint) winds it negative, toward cancelling the excess;
    ///  * an integral whose sign is OPPOSITE to the excess is pushed further away from zero, again toward
    ///    cancelling it. A +1 degree phase (integral +2 degree-seconds) followed by -30 degrees of error under a
    ///    5 degree cap winds it UP to +10.7 degree-seconds, and when the cap reopens at -1 degree of error the
    ///    command is +20.4 degrees: the reversed fin the guard was written against. Latent while ki is 0 (every
    ///    shipped config); the golden fixture pid.json carries both cases.
    ///
    /// NON-FINITE INPUTS, a failure plan B could not have: its error was an integer fixnum (the bno055 driver
    /// reads centidegrees as integers, and `fixed.from_float` raises on a NaN or an infinity) and its time integer
    /// milliseconds. Here both are f32, and one NaN folded into the integral would LATCH: with ki 0 (every shipped
    /// config) nothing ever bleeds it, `0 x NaN` is NaN, and every later output would be NaN until [`Pid::reset`],
    /// which the mixer could only turn into some arbitrary fin angle. So a non-finite `error` or `elapsed` is
    /// answered with a neutral 0 degree demand and leaves the integral and the previous error exactly as they
    /// were: the same as plan B's degraded tick for a stale or absent attitude (flight `_tick` centres the fins and
    /// never steps the PIDs), and the next finite step carries on from the last good one. A non-finite `rate` is
    /// no rate: the step falls back to differentiating the error, plan B's documented degraded mode for a gyro.
    pub fn step(&mut self, error: f32, elapsed: f32, rate: Option<f32>) -> f32 {
        if !(error.is_finite() && elapsed.is_finite()) {
            return 0.0;
        }
        let rate = rate.filter(|rate| rate.is_finite());
        let mut integral = between(
            -self.integral_limit,
            self.integral + error * elapsed,
            self.integral_limit,
        );
        let derivative = match (rate, self.previous) {
            (Some(rate), _) => -rate,
            (None, Some(previous)) if elapsed > 0.0 => (error - previous) / elapsed,
            (None, _) => 0.0,
        };
        self.previous = Some(error);
        let output =
            self.gains.proportional * error + self.gains.integral * integral + self.gains.derivative * derivative;
        let limited = between(-self.output_limit, output, self.output_limit);
        if limited != output && self.gains.integral != 0.0 {
            let unwound = integral - (output - limited) / self.gains.integral * self.anti_windup_factor;
            integral = if integral > 0.0 {
                unwound.max(0.0)
            } else {
                unwound.min(0.0)
            };
        }
        self.integral = integral;
        limited
    }
}

#[cfg(test)]
mod tests {
    use super::{ANTI_WINDUP_SHIFT, Gains, Pid};

    /// The plan B constructor's "unbounded" default limit, in degrees (pid._UNBOUNDED_DEG).
    const UNBOUNDED: f32 = 1_000_000.0;

    /// A controller with these gains, unbounded, shipping anti-windup: plan B's `pid.Pid(kp=, ki=, kd=)`.
    fn unbounded(proportional: f32, integral: f32, derivative: f32) -> Pid {
        let gains = Gains {
            proportional,
            integral,
            derivative,
        };
        Pid::new(gains, UNBOUNDED, UNBOUNDED, ANTI_WINDUP_SHIFT)
    }

    /// A controller with these gains and limits, at this anti-windup shift.
    fn limited(proportional: f32, integral: f32, integral_limit: f32, output_limit: f32, shift: u32) -> Pid {
        let gains = Gains {
            proportional,
            integral,
            derivative: 0.0,
        };
        Pid::new(gains, integral_limit, output_limit, shift)
    }

    /// Whether two degree values agree to f32 rounding.
    fn close(answer: f32, expected: f32) -> bool {
        (answer - expected).abs() <= 1e-4 * expected.abs().max(1.0)
    }

    #[test]
    fn each_term_acts_alone() {
        // proportional: kp=2 on 3 degrees -> 6
        assert!(close(unbounded(2.0, 0.0, 0.0).step(3.0, 0.1, None), 6.0));
        // integral: 2 degrees for 0.5 s -> 1 degree-second -> ki x 1 = 1, then 2
        let mut integral = unbounded(0.0, 1.0, 0.0);
        assert!(close(integral.step(2.0, 0.5, None), 1.0));
        assert!(close(integral.step(2.0, 0.5, None), 2.0));
        // derivative on the error: 1 degree over 0.1 s = 10 degrees per second
        let mut derivative = unbounded(0.0, 0.0, 1.0);
        assert_eq!(derivative.step(0.0, 0.1, None), 0.0);
        assert!(close(derivative.step(1.0, 0.1, None), 10.0));
    }

    #[test]
    fn a_measured_rate_feeds_the_derivative_negated_and_needs_no_first_step() {
        let mut rated = unbounded(0.0, 0.0, 1.0);
        assert!(close(rated.step(0.0, 0.1, Some(10.0)), -10.0));
        assert!(close(rated.step(0.0, 0.1, Some(-4.0)), 4.0));
    }

    #[test]
    fn the_first_step_after_construction_or_reset_takes_no_derivative() {
        let mut spike = unbounded(0.0, 0.0, 1.0);
        assert_eq!(spike.step(5.0, 0.1, None), 0.0);
        spike.reset();
        assert_eq!(spike.step(9.0, 0.1, None), 0.0);
        // and no time elapsed is no derivative either, rather than a division by zero
        assert_eq!(spike.step(12.0, 0.0, None), 0.0);
    }

    #[test]
    fn a_full_heading_swing_stays_signed() {
        let mut swing = unbounded(1.5, 0.0, 0.0);
        assert!(close(swing.step(180.0, 0.01, None), 270.0));
        assert!(close(swing.step(-180.0, 0.01, None), -270.0));
    }

    #[test]
    fn the_integral_clamp_pins_the_integral() {
        let mut integral = limited(0.0, 1.0, 5.0, UNBOUNDED, ANTI_WINDUP_SHIFT);
        let mut output = 0.0;
        for _ in 0..20 {
            output = integral.step(10.0, 1.0, None);
        }
        assert!(close(output, 5.0));
        assert!(close(integral.integral(), 5.0));
    }

    #[test]
    fn the_output_clamp_holds_both_ways() {
        let mut clamped = limited(100.0, 0.0, UNBOUNDED, 45.0, ANTI_WINDUP_SHIFT);
        assert_eq!(clamped.step(10.0, 0.1, None), 45.0);
        assert_eq!(clamped.step(-10.0, 0.1, None), -45.0);
    }

    #[test]
    fn reset_clears_the_integral() {
        let mut integral = unbounded(0.0, 1.0, 0.0);
        integral.step(5.0, 1.0, None);
        integral.reset();
        assert_eq!(integral.integral(), 0.0);
        assert_eq!(integral.step(0.0, 1.0, None), 0.0);
    }

    #[test]
    fn set_limit_retunes_both_clamps_live() {
        let mut clamped = limited(100.0, 0.0, UNBOUNDED, 45.0, ANTI_WINDUP_SHIFT);
        assert_eq!(clamped.step(10.0, 0.1, None), 45.0);
        clamped.set_limit(5);
        assert_eq!(clamped.step(10.0, 0.1, None), 5.0);
        clamped.set_limit(45);
        assert_eq!(clamped.step(10.0, 0.1, None), 45.0);
        // the integral clamp follows too: held at the tight cap, nothing to dump when it reopens
        let mut integral = limited(0.0, 1.0, 45.0, UNBOUNDED, ANTI_WINDUP_SHIFT);
        integral.set_limit(5);
        for _ in 0..20 {
            integral.step(10.0, 1.0, None);
        }
        integral.set_limit(45);
        assert!(close(integral.step(0.0, 1.0, None), 5.0));
    }

    /// Plan B's test_back_calculation: 50 steps of 30 degrees against a 5 degree cap, then the cap reopens.
    fn saturate(shift: u32) -> (f32, f32) {
        let mut axis = limited(1.0, 2.0, 45.0, 5.0, shift);
        for _ in 0..50 {
            axis.step(30.0, 0.02, None);
        }
        let wound = axis.integral();
        axis.set_limit(45);
        (wound, axis.step(0.0, 0.02, None))
    }

    #[test]
    fn back_calculation_unwinds_a_saturated_integral() {
        let (wound, dump) = saturate(ANTI_WINDUP_SHIFT);
        assert_eq!(wound, 0.0, "unwound, not pinned");
        assert_eq!(dump, 0.0, "nothing to dump");
        // NEGATIVE: a shift so deep the bleed is nothing is the clamp-only law, and it dumps the full deflection
        let (pinned, spike) = saturate(30);
        assert!(close(pinned, 30.0), "{pinned}");
        assert_eq!(spike, 45.0);
    }

    #[test]
    fn a_positive_integral_stops_at_zero_and_never_reverses() {
        let mut reversal = limited(1.0, 2.0, 45.0, 5.0, ANTI_WINDUP_SHIFT);
        for _ in 0..200 {
            reversal.step(30.0, 0.02, None);
            assert!(reversal.integral() >= 0.0, "{}", reversal.integral());
        }
    }

    #[test]
    fn back_calculation_leaves_an_unsaturated_integral_alone() {
        let mut free = limited(0.0, 2.0, 45.0, 45.0, ANTI_WINDUP_SHIFT);
        for _ in 0..50 {
            free.step(1.0, 0.02, None);
        }
        assert!(close(free.integral(), 1.0), "{}", free.integral());
        assert!(free.step(1.0, 0.02, None) > 0.0);
    }

    #[test]
    fn a_symmetric_error_leaves_no_integral_and_a_small_one_still_accumulates() {
        // plan B's RATCHET (floor division is not symmetric) and DEADBAND (error x dt under one unit truncated
        // away) were both fixed-point defects; the real law has neither
        let mut balanced = limited(0.0, 1.0, 45.0, 45.0, ANTI_WINDUP_SHIFT);
        for index in 0..200 {
            balanced.step(if index % 2 == 0 { 0.5 } else { -0.5 }, 0.01, None);
        }
        assert_eq!(balanced.integral(), 0.0);
        let mut creeping = limited(0.0, 1.0, 45.0, 45.0, ANTI_WINDUP_SHIFT);
        for _ in 0..100 {
            creeping.step(0.5, 0.01, None);
        }
        assert!(close(creeping.integral(), 0.5), "{}", creeping.integral());
        assert!(creeping.step(0.5, 0.01, None) > 0.0);
    }

    #[test]
    fn without_ki_the_output_clamp_alone_holds() {
        let mut proportional = limited(100.0, 0.0, UNBOUNDED, 5.0, ANTI_WINDUP_SHIFT);
        assert_eq!(proportional.step(30.0, 0.02, None), 5.0);
        assert!(close(proportional.integral(), 0.6), "accumulates, nothing bleeds it");
    }

    #[test]
    fn a_kick_on_the_setpoint_winds_an_empty_integral_against_it() {
        // plan B's guard sends an integral of exactly 0 down its `else` branch: a positive D-term saturation with
        // the error on the setpoint winds it negative (plan B: -1.25, -2.18 ... degree-seconds)
        let gains = Gains {
            proportional: 1.0,
            integral: 1.0,
            derivative: 1.0,
        };
        let mut kicked = Pid::new(gains, 45.0, 5.0, ANTI_WINDUP_SHIFT);
        assert_eq!(kicked.step(0.0, 0.01, Some(-10.0)), 5.0);
        assert!(close(kicked.integral(), -1.25), "{}", kicked.integral());
        kicked.step(0.0, 0.01, Some(-10.0));
        assert!(close(kicked.integral(), -2.1875), "{}", kicked.integral());
    }

    #[test]
    fn an_integral_of_the_opposite_sign_winds_on_and_reverses_the_fin() {
        // the guard only stops an integral moving toward zero: after a +1 degree phase, -30 degrees against a
        // 5 degree cap winds it UP to +10.7, and the reopened cap commands a positive fin for a negative error
        let mut axis = limited(1.0, 2.0, 45.0, 5.0, ANTI_WINDUP_SHIFT);
        for _ in 0..100 {
            axis.step(1.0, 0.02, None);
        }
        assert!(close(axis.integral(), 2.0), "{}", axis.integral());
        for _ in 0..60 {
            assert_eq!(axis.step(-30.0, 0.02, None), -5.0);
        }
        assert!((axis.integral() - 10.7).abs() < 0.01, "{}", axis.integral());
        axis.set_limit(45);
        let reversed = axis.step(-1.0, 0.02, None);
        assert!((reversed - 20.36).abs() < 0.02, "{reversed}");
    }

    /// Plan B's shipped gains (kp 1.5, kd 0.1, ki 0) and the same with an integral (ki 0.5), both under the
    /// 45 degree cap: the two NaN cases, as the latch differs by whether back-calculation runs.
    fn shipped(integral: f32) -> Pid {
        let gains = Gains {
            proportional: 1.5,
            integral,
            derivative: 0.1,
        };
        Pid::new(gains, 45.0, 45.0, ANTI_WINDUP_SHIFT)
    }

    #[test]
    fn a_non_finite_error_or_elapsed_time_is_a_neutral_demand_and_leaves_no_trace() {
        // NEGATIVE: one NaN must not latch. Each pair of controllers sees the same finite history, but the poisoned
        // one also takes a non-finite step in the middle; it answers 0 there and every later answer is the clean
        // one's, with ki 0 (nothing would ever bleed a NaN integral) and with ki 0.5 (back-calculation runs)
        for integral in [0.0, 0.5] {
            for (error, elapsed) in [
                (f32::NAN, 0.01),
                (f32::INFINITY, 0.01),
                (f32::NEG_INFINITY, 0.01),
                (5.0, f32::NAN),
                (5.0, f32::INFINITY),
            ] {
                let (mut clean, mut poisoned) = (shipped(integral), shipped(integral));
                clean.step(2.0, 0.01, None);
                poisoned.step(2.0, 0.01, None);
                assert_eq!(poisoned.step(error, elapsed, None), 0.0, "{error} over {elapsed}");
                assert_eq!(poisoned, clean, "the state is untouched: {error} over {elapsed}");
                for _ in 0..3 {
                    let answer = poisoned.step(5.0, 0.01, None);
                    assert!(answer.is_finite(), "{error} over {elapsed} latched: {answer}");
                    assert_eq!(answer, clean.step(5.0, 0.01, None));
                }
                assert!(poisoned.integral().is_finite());
            }
        }
    }

    #[test]
    fn a_non_finite_rate_counts_as_no_rate() {
        // NEGATIVE: a NaN gyro sample falls back to the error derivative instead of poisoning the output
        let mut rated = unbounded(0.0, 0.0, 1.0);
        let mut unrated = unbounded(0.0, 0.0, 1.0);
        for (index, error) in [0.0, 1.0, 3.0].into_iter().enumerate() {
            let rate = if index == 0 { f32::INFINITY } else { f32::NAN };
            assert_eq!(rated.step(error, 0.1, Some(rate)), unrated.step(error, 0.1, None));
        }
        assert!(close(rated.step(4.0, 0.1, Some(f32::NAN)), 10.0));
        // POSITIVE: a finite rate still wins over the error derivative
        assert!(close(rated.step(4.0, 0.1, Some(2.0)), -2.0));
    }

    #[test]
    fn a_sub_centidegree_integral_is_kept_where_plan_b_floored_it_to_zero() {
        // plan B floored 0.1 degree x 10 ms (0.001 degree-seconds) to 0 and then wound it to -1.25; the real law
        // keeps it positive, so the guard stops it at zero
        let mut axis = limited(100.0, 1.0, UNBOUNDED, 5.0, ANTI_WINDUP_SHIFT);
        assert_eq!(axis.step(0.1, 0.01, None), 5.0);
        assert_eq!(axis.integral(), 0.0);
    }
}
