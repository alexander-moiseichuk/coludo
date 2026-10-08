//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The freshness judges' unit tests, on bytes alone: the flag decides and the bytes correct it ([`Flagged`]: the
//! raced mark lasts one read, the halves are judged apart), and changed bytes with the standby rule ([`Changes`]).
//! The drivers' own tests run the same rules through their bursts.

use super::{Change, Changes, Flagged, Fresh, HALF_LENGTH, STALE_PERIODS};

/// One half's bytes, every byte `value`.
const fn half(value: u8) -> [u8; HALF_LENGTH] {
    [value; HALF_LENGTH]
}

#[test]
fn a_flag_after_a_clear_read_is_a_new_conversion_even_over_identical_bytes() {
    let mut judge = Flagged::default();
    judge.judge(true, &half(1), true, &half(1));
    // a raced accel conversion: flag clear, bytes changed, pushed
    let raced = judge.judge(false, &half(2), false, &half(1));
    assert!(raced.accel && raced.accel_raced && !raced.rate);
    // the read after it shows the flag clear: the raced conversion's flag is gone...
    assert_eq!(judge.judge(false, &half(2), false, &half(1)), Fresh::default());
    // ...so a flag over the same bytes now is a NEW conversion that repeated them, pushed and counted identical
    let repeated = judge.judge(true, &half(2), false, &half(1));
    assert!(repeated.accel && repeated.accel_identical && !repeated.accel_raced);
}

#[test]
fn the_duplicate_is_skipped_once_and_the_next_flag_is_new() {
    let mut judge = Flagged::default();
    judge.judge(true, &half(1), true, &half(1));
    assert!(judge.judge(false, &half(1), false, &half(2)).rate_raced);
    // NEGATIVE: the raced conversion's own flag, read late over its bytes, is not pushed again
    let duplicate = judge.judge(false, &half(1), true, &half(2));
    assert!(!duplicate.rate && duplicate.rate_identical);
    // the duplicate consumed that flag: the next flag over the same bytes is a conversion that repeated them
    let next = judge.judge(false, &half(1), true, &half(2));
    assert!(next.rate && next.rate_identical);
}

#[test]
fn the_halves_are_judged_apart() {
    let mut judge = Flagged::default();
    judge.judge(true, &half(1), true, &half(1));
    // the gyro raced while the accel converted normally: one push each, one raced mark
    let fresh = judge.judge(true, &half(2), false, &half(3));
    assert_eq!(
        fresh,
        Fresh {
            accel: true,
            rate: true,
            rate_raced: true,
            ..Fresh::default()
        }
    );
    // the accel's next flag over its bytes is pushed (its last push was not raced); the gyro's is the duplicate
    let after = judge.judge(true, &half(2), true, &half(3));
    assert!(after.accel && !after.rate && after.accel_identical && after.rate_identical);
}

#[test]
fn a_forgotten_judge_reads_the_flags_alone() {
    let mut judge = Flagged::default();
    judge.judge(true, &half(1), true, &half(1));
    judge.forget();
    // NEGATIVE: with nothing stored, changed bytes under a clear flag are not a raced conversion
    assert_eq!(judge.judge(false, &half(9), false, &half(9)), Fresh::default());
    judge.forget();
    let flagged = judge.judge(true, &half(9), false, &half(9));
    assert!(flagged.accel && !flagged.accel_identical && !flagged.rate);
}

#[test]
fn changed_bytes_are_fresh_and_the_first_read_is_too() {
    let mut judge = Changes::<3>::default();
    assert_eq!(judge.judge(&[1, 2, 3], 0, 20_000), Change::Fresh);
    assert_eq!(judge.judge(&[1, 2, 3], 10_000, 20_000), Change::Unchanged);
    // one byte is enough: the last byte of the burst
    assert_eq!(judge.judge(&[1, 2, 4], 20_000, 20_000), Change::Fresh);
}

#[test]
fn a_part_in_standby_goes_stale_after_three_periods() {
    // NEGATIVE: the same bytes from the last change on, polled every 10 ms on a 20 ms part
    let mut judge = Changes::<3>::default();
    judge.judge(&[7, 7, 7], 1_000_000, 20_000);
    let verdicts: Vec<Change> = (1..=8)
        .map(|tick| judge.judge(&[7, 7, 7], 1_000_000 + tick * 10_000, 20_000))
        .collect();
    let stale_from = usize::try_from(STALE_PERIODS * 2 - 1).unwrap();
    assert!(
        verdicts[..stale_from].iter().all(|change| *change == Change::Unchanged),
        "{verdicts:?}"
    );
    assert!(
        verdicts[stale_from..].iter().all(|change| *change == Change::Stale),
        "{verdicts:?}"
    );
    // a conversion brings it back at once, and the clock starts over
    assert_eq!(judge.judge(&[7, 7, 8], 1_200_000, 20_000), Change::Fresh);
    assert_eq!(judge.judge(&[7, 7, 8], 1_250_000, 20_000), Change::Unchanged);
}

#[test]
fn a_late_read_with_new_bytes_is_fresh_however_late() {
    // a stalled owner is not a stale part: the bytes changed, so the conversions went on
    let mut judge = Changes::<2>::default();
    judge.judge(&[1, 1], 0, 80_000);
    assert_eq!(judge.judge(&[1, 2], 5_000_000, 80_000), Change::Fresh);
    // NEGATIVE: the same bytes after the same gap are a part that stopped
    assert_eq!(judge.judge(&[1, 2], 10_000_000, 80_000), Change::Stale);
}
