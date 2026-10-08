//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The supervisor's stall rule (doc/specs/avionics-port.md M8, "Warm start, checkpoint, supervisor watchdog"): while
//! the board controls a flight, the control thread must keep ticking, and one whose last tick is older than
//! `stall_ms` has stalled, so the supervisor restarts the board. Plan B's watchdog task judged the same by the flight
//! task's heartbeat TIMESTAMP (tasks/watchdog.py `_stalled`), independent of its own poll: a supervisor that checks
//! late, or a check that lands between two ticks, sees the true age of the last tick.
//!
//! The port, against plan B:
//!  * The heartbeat is the uptime in ms as a wrapping u32, which the control thread stores beside its tick count (the
//!    P4 has no 64-bit atomics), and the age is the signed wrapping difference. So the u32 wrap (every 49.7 days) is
//!    no stall, and neither is a `now` the supervisor read just before the control thread stored a newer tick (the
//!    two run on different cores): that age is negative. An age past `i32::MAX` ms (24.8 days) reads negative too,
//!    but the supervisor checks every `period_ms`, so a stall is caught long before it is that old.
//!  * The limit is at least two tick periods: one late tick is jitter, not a stall, so a `stall_ms` below that
//!    cannot restart a healthy board.
//!  * A control thread that has never ticked is a failed bring-up, not a stall: the rule judges only one that has.
//!  * Whether the board controls a flight is the caller's to say: armed, launched, in an airborne stage. Plan B
//!    judged only while its flight task controlled the fins, so never on a passive flight.

/// How many tick periods the limit is at least: one late tick is jitter, two missed ones are not.
const TICKS_OF_JITTER: f32 = 2.0;

/// The stall rule, its limit fixed at bring-up from the watchdog's `stall_ms` and the control tick's period.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stall {
    /// The oldest the last tick may be and the control thread still live, in whole ms: the larger of `stall_ms` and
    /// two tick periods, rounded down by the `as` conversion, which truncates a non-negative limit (an age in whole ms
    /// passes a fractional limit exactly when it passes the limit's whole part), from 0 to `i32::MAX`.
    longest_age_ms: i32,
}

impl Stall {
    /// The rule for a `stall_ms` limit on a control thread that ticks every `tick_ms`.
    ///
    /// Args:
    /// * `stall_ms` - the watchdog's `stall_ms`: how long the control thread may go without a tick, a fraction
    ///   included (board_config's `watchdog` admits any number above zero).
    /// * `tick_ms` - the control tick's period (1000 / `schedule_hz`).
    ///
    /// Returns the rule. Its limit is never under two tick periods; one past `i32::MAX` ms (an infinite one
    /// included) is `i32::MAX`, an age the signed difference never passes, so the rule never fires.
    pub fn new(stall_ms: f32, tick_ms: f32) -> Stall {
        let limit_ms = stall_ms.max(TICKS_OF_JITTER * tick_ms);
        // `as` truncates toward zero and saturates: 500.5 is 500 and an infinite limit i32::MAX; `max` lifts one below
        // zero to 0
        Stall {
            longest_age_ms: (limit_ms as i32).max(0),
        }
    }

    /// Whether the control thread has stalled: the board controls a flight, the thread has ticked, and its last tick
    /// is older than the limit.
    ///
    /// The rule trusts `ticked` to vouch for `last_tick_ms`, and the two are separate atomics, so they must be read in
    /// this order: the control thread stores `last_tick_ms` and then publishes `ticked` (a Release store), and the
    /// supervisor loads `ticked` (Acquire) and only then `last_tick_ms`. Read the other way round, a check that
    /// straddles the first tick sees the boot's `last_tick_ms` of 0 with `ticked` true, an age of the whole uptime, and
    /// restarts a live board; on a warm start the board already controls a flight at that first tick. So `ticked`
    /// means a tick has stored `last_tick_ms`, not that the thread has started.
    ///
    /// Args:
    /// * `now_ms` - the uptime in ms as a wrapping u32, read by the supervisor for this check.
    /// * `last_tick_ms` - the uptime in ms as a wrapping u32 the control thread stored at its last tick, loaded after
    ///   `ticked`.
    /// * `ticked` - whether a tick has stored `last_tick_ms` this boot (`last_tick_ms` means nothing before), loaded
    ///   first.
    /// * `controlling` - whether the board controls a flight now: armed, launched and in an airborne stage.
    ///
    /// Returns true for a stall, which restarts the board; false otherwise, a `now_ms` read before `last_tick_ms`
    /// included.
    pub fn step(&self, now_ms: u32, last_tick_ms: u32, ticked: bool, controlling: bool) -> bool {
        // the signed wrapping difference: negative when `now_ms` was read before the tick was stored
        let age_ms = now_ms.wrapping_sub(last_tick_ms) as i32;
        controlling && ticked && age_ms > self.longest_age_ms
    }
}

#[cfg(test)]
mod tests {
    use super::Stall;

    /// Plan B's defaults: a 500 ms limit on a 100 Hz tick.
    fn defaults() -> Stall {
        Stall::new(500.0, 10.0)
    }

    #[test]
    fn a_tick_older_than_the_limit_while_controlling_is_a_stall() {
        let stall = defaults();
        assert!(!stall.step(1500, 1000, true, true), "500 ms is the limit itself");
        assert!(stall.step(1501, 1000, true, true));
        assert!(stall.step(60_000, 1000, true, true));
    }

    #[test]
    fn a_stale_tick_is_no_stall_while_not_controlling() {
        let stall = defaults();
        assert!(!stall.step(1501, 1000, true, false));
        assert!(
            !stall.step(600_000, 1000, true, false),
            "a held or disarmed board never restarts"
        );
    }

    #[test]
    fn a_control_thread_that_never_ticked_is_no_stall() {
        let stall = defaults();
        assert!(!stall.step(600_000, 0, false, true));
        assert!(!stall.step(u32::MAX, 0, false, true));
    }

    #[test]
    fn a_late_supervisor_judges_the_ticks_age_not_its_own_gap() {
        let stall = defaults();
        // checks 2 s apart, four times the limit, while the control thread ticked 5 ms before each
        for now_ms in [10_000_u32, 12_000, 14_000] {
            assert!(!stall.step(now_ms, now_ms - 5, true, true), "{now_ms}");
        }
    }

    #[test]
    fn controlling_after_minutes_of_live_ticks_is_no_stall() {
        let stall = defaults();
        let mut now_ms = 0_u32;
        while now_ms < 300_000 {
            assert!(!stall.step(now_ms, now_ms.saturating_sub(3), true, false));
            now_ms += 200;
        }
        assert!(
            !stall.step(now_ms, now_ms - 3, true, true),
            "the launch turns control on"
        );
    }

    #[test]
    fn a_now_read_before_the_tick_was_stored_is_no_stall() {
        let stall = defaults();
        assert!(!stall.step(1000, 1001, true, true));
        assert!(!stall.step(1000, 1010, true, true));
        // across the wrap too
        assert!(!stall.step(u32::MAX, 2, true, true));
    }

    #[test]
    fn the_uptimes_wrap_is_no_stall() {
        let stall = defaults();
        let last_tick_ms = u32::MAX - 100;
        assert!(
            !stall.step(399, last_tick_ms, true, true),
            "500 ms after the last tick, across the wrap"
        );
        assert!(stall.step(400, last_tick_ms, true, true), "501 ms after it");
    }

    #[test]
    fn the_limit_is_at_least_two_tick_periods() {
        for stall in [Stall::new(5.0, 10.0), Stall::new(0.5, 10.0), Stall::new(20.0, 10.0)] {
            assert!(!stall.step(1020, 1000, true, true), "{stall:?}");
            assert!(stall.step(1021, 1000, true, true), "{stall:?}");
        }
        // a slow tick raises the floor past the configured limit
        let slow = Stall::new(500.0, 400.0);
        assert!(!slow.step(1800, 1000, true, true));
        assert!(slow.step(1801, 1000, true, true));
        // a fractional period: 3 1/3 ms at 300 Hz floors at 6 2/3 ms, so 7 ms is past it
        let fast = Stall::new(1.0, 1000.0 / 300.0);
        assert!(!fast.step(1006, 1000, true, true));
        assert!(fast.step(1007, 1000, true, true));
    }

    #[test]
    fn a_limit_below_zero_is_zero_so_a_tick_this_ms_is_live() {
        // board_config's view refuses such a limit; the rule still never calls a tick of this very ms a stall
        for stall in [Stall::new(0.0, 0.0), Stall::new(-5.0, -1.0)] {
            assert!(!stall.step(1000, 1000, true, true), "{stall:?}");
            assert!(stall.step(1001, 1000, true, true), "{stall:?}");
        }
    }

    #[test]
    fn a_fractional_limit_is_passed_by_the_next_whole_ms() {
        let stall = Stall::new(500.5, 10.0);
        assert!(!stall.step(1500, 1000, true, true));
        assert!(stall.step(1501, 1000, true, true));
    }

    #[test]
    fn a_limit_past_the_signed_age_never_fires() {
        for stall in [Stall::new(1e12, 10.0), Stall::new(f32::INFINITY, 10.0)] {
            assert!(!stall.step(i32::MAX as u32, 0, true, true), "{stall:?}");
        }
        // a long limit within the signed age still fires past it
        let long = Stall::new(1_000_000.0, 10.0);
        assert!(!long.step(1_000_000, 0, true, true));
        assert!(long.step(1_000_001, 0, true, true));
    }
}
