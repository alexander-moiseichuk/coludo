//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The arm's age rule (doc/specs/avionics-port.md §4.4, "The arrival stamp and the 25 s rule"): the executor stamps
//! each request line as it arrives, and before it posts an `arm` it refuses one that has waited too long. A line waits
//! behind the commands queued ahead of it (a hub probe sweep is up to ~18 s) and then through its own sweep, and the
//! hub gives up on an answer after 30 s (src/control/board.py): an arm posted later than that would arm a board whose
//! operator was told it did not answer. Plan B had no such rule: its one asyncio loop served the hub's lines in
//! lockstep.

use std::time::Duration;

/// The longest an `arm` may have waited since it arrived and still be posted: the request slot adds at most 2 s, so
/// the answer beats the hub's 30 s whatever queued ahead.
const ARM_WAIT_LIMIT: Duration = Duration::from_secs(25);

/// Nanoseconds in a tenth of a second.
const NANOSECONDS_PER_TENTH: u128 = 100_000_000;

/// Why an `arm` that has waited `waited` since it arrived must not be posted, as `arm` refuses it under the key
/// `arm`: `the request waited <n> s: send it again`, `<n>` in tenths of a second rounded up, so a wait just past the
/// limit never reads as the limit itself. None while the wait is within [`ARM_WAIT_LIMIT`].
pub fn stale_arm(waited: Duration) -> Option<String> {
    (waited > ARM_WAIT_LIMIT).then(|| {
        let tenths = waited.as_nanos().div_ceil(NANOSECONDS_PER_TENTH);
        format!("the request waited {}.{} s: send it again", tenths / 10, tenths % 10)
    })
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{ARM_WAIT_LIMIT, stale_arm};

    #[test]
    fn an_arm_within_the_limit_is_posted() {
        for waited in [
            Duration::ZERO,
            Duration::from_millis(1),
            Duration::from_secs(18),
            ARM_WAIT_LIMIT,
        ] {
            assert_eq!(stale_arm(waited), None, "{waited:?}");
        }
    }

    #[test]
    fn an_arm_past_the_limit_is_refused_with_its_wait() {
        assert_eq!(
            stale_arm(Duration::from_millis(27_340)).as_deref(),
            Some("the request waited 27.4 s: send it again")
        );
        assert_eq!(
            stale_arm(Duration::from_secs(30)).as_deref(),
            Some("the request waited 30.0 s: send it again"),
            "the hub's own deadline"
        );
        assert_eq!(
            stale_arm(Duration::from_secs(600)).as_deref(),
            Some("the request waited 600.0 s: send it again")
        );
    }

    #[test]
    fn a_wait_just_past_the_limit_reads_past_it() {
        assert_eq!(
            stale_arm(ARM_WAIT_LIMIT + Duration::from_nanos(1)).as_deref(),
            Some("the request waited 25.1 s: send it again"),
            "never `25.0`, which would read as allowed"
        );
        assert_eq!(
            stale_arm(ARM_WAIT_LIMIT + Duration::from_millis(100)).as_deref(),
            Some("the request waited 25.1 s: send it again")
        );
    }
}
