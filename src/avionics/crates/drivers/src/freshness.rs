//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! New conversions only (avionics-port.md M6, M9 D1, D1b and D3): how a driver read on its owner's tick tells a
//! conversion it has not returned before from one it has. Plan B pushed every read, but its reads were slower than
//! the parts; the port's tick is not, so without this a part slower than the tick would publish an old sample as a
//! new one, and a part in standby would publish its last sample forever. No register is written for it: the parts
//! are read as plan B configured them. Two judges, one per kind of part:
//!
//! * [`Flagged`], for the IMUs (the LSM6DSO32 and the BMI323), whose one burst carries a data-ready flag per half
//!   (accel, gyro) next to the sample. THE FLAG DECIDES; THE BYTES ONLY CORRECT IT. Bytes cannot change without a
//!   conversion, but a new conversion can repeat the last one's bytes: the gyro's LSB is 0.061-0.07 dps, so at rest
//!   two conversions in a row are often bit-identical, and bytes alone would drop real pushes. The flags and the
//!   output are read in one frame, yet not at one instant, so a conversion can land between the status byte and the
//!   data: the burst then carries a NEW sample under a CLEAR flag. Two corrections: (a) a half with its flag clear
//!   but its bytes changed since the last read is that raced conversion: fresh, and marked raced; (b) a half with
//!   its flag set over bytes identical to the last read's, when that read pushed it as raced, is the raced
//!   conversion's own flag read late: the duplicate, not fresh. A flagged half is otherwise never called stale for
//!   identical bytes (it is counted as `identical`, the measured residual: a raced conversion whose half is
//!   bit-identical to the one before is lost, as one with no flag is). After [`Flagged::forget`] nothing is stored
//!   and a read is judged by its flag alone.
//!
//! * [`Changes`], for the BMP581 and the BMM350, read with no data-ready register (the BMP581's needs INT_SOURCE
//!   written, a register plan B never wrote): fresh when the data bytes changed. Their noise is tens to hundreds of
//!   counts, so two real conversions practically never repeat every byte, and a part in standby, which returns its
//!   last sample forever, reads the same bytes: [`Change::Unchanged`] while that is a part slower than its poll, and
//!   [`Change::Stale`] once it has lasted [`STALE_PERIODS`] conversion periods, where the firmware calls the part
//!   unhealthy.
//!
//! A frame a driver knows is not a sample (an impossible status, an all-zero burst, a 0x8000 start-up marker) never
//! reaches a judge: the driver answers it not fresh and the stored bytes stay those of the last real sample.

/// The bytes of one half of an IMU sample: three axes, two bytes each.
pub const HALF_LENGTH: usize = 6;

/// The conversion periods a [`Changes`] part may repeat its bytes before it is [`Change::Stale`] (D3: a standby
/// part goes stale, then unhealthy after three periods; plan B's re-zero takes an altitude fresher than three, too).
pub const STALE_PERIODS: u64 = 3;

/// Which halves of an IMU burst are new conversions (push `accel`, push `rate`), and how the bytes corrected the
/// flags, per half (`inspect control`'s `raced` and `identical` counters).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Fresh {
    /// The accel half is a conversion no read has returned before: push `accel`.
    pub accel: bool,
    /// The gyro half is a conversion no read has returned before: push `rate`.
    pub rate: bool,
    /// The accel half is a raced conversion, its flag clear but its bytes changed (correction (a), pushed).
    pub accel_raced: bool,
    /// The gyro half is a raced conversion (correction (a), pushed).
    pub rate_raced: bool,
    /// The accel flag is set over bytes identical to the last read's: pushed, or skipped as the duplicate of a raced
    /// push (correction (b)) when `accel` is false.
    pub accel_identical: bool,
    /// The gyro flag is set over bytes identical to the last read's, pushed or skipped as `accel_identical` is.
    pub rate_identical: bool,
}

/// One half's verdict: what [`Fresh`] carries for it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Verdict {
    /// A new conversion.
    fresh: bool,
    /// Pushed by correction (a).
    raced: bool,
    /// Flagged over unchanged bytes.
    identical: bool,
}

/// Judge one half: its flag, its bytes, the bytes the last judged read stored (None after a forget), and whether
/// that read pushed this half as raced.
fn judge_half(flag: bool, bytes: &[u8; HALF_LENGTH], last: Option<&[u8; HALF_LENGTH]>, raced: bool) -> Verdict {
    let changed = last.map(|last| last != bytes);
    match (flag, changed) {
        // correction (b): the raced conversion's own flag, read late, is the duplicate
        (true, Some(false)) => Verdict {
            fresh: !raced,
            raced: false,
            identical: true,
        },
        (true, _) => Verdict {
            fresh: true,
            ..Verdict::default()
        },
        // correction (a): bytes cannot change without a conversion
        (false, Some(true)) => Verdict {
            fresh: true,
            raced: true,
            identical: false,
        },
        (false, _) => Verdict::default(),
    }
}

/// The flag-with-byte-corrections judge of an IMU's two halves (see the module notes): the last judged read's bytes
/// and, per half, whether that read pushed it as raced.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flagged {
    /// The last judged read's accel and gyro halves; None until the first read and after [`Flagged::forget`].
    last: Option<([u8; HALF_LENGTH], [u8; HALF_LENGTH])>,
    /// The last judged read pushed its accel half as raced.
    accel_raced: bool,
    /// The last judged read pushed its gyro half as raced.
    rate_raced: bool,
}

impl Flagged {
    /// Judge one burst's halves by their data-ready flags, corrected by their bytes, and store the bytes for the next.
    ///
    /// The raced mark lasts exactly one read: correction (b) applies only to the read right after the raced push. A
    /// read between them that showed the flag CLEAR proves the raced conversion's flag was already gone, so a flag set
    /// after it is a new conversion, pushed even over identical bytes.
    pub fn judge(
        &mut self,
        accel_flag: bool,
        accel: &[u8; HALF_LENGTH],
        rate_flag: bool,
        rate: &[u8; HALF_LENGTH],
    ) -> Fresh {
        let last = self.last.as_ref();
        let accel_verdict = judge_half(accel_flag, accel, last.map(|(accel, _)| accel), self.accel_raced);
        let rate_verdict = judge_half(rate_flag, rate, last.map(|(_, rate)| rate), self.rate_raced);
        self.last = Some((*accel, *rate));
        self.accel_raced = accel_verdict.raced;
        self.rate_raced = rate_verdict.raced;
        Fresh {
            accel: accel_verdict.fresh,
            rate: rate_verdict.fresh,
            accel_raced: accel_verdict.raced,
            rate_raced: rate_verdict.raced,
            accel_identical: accel_verdict.identical,
            rate_identical: rate_verdict.identical,
        }
    }

    /// Forget the stored bytes and the raced marks: the next read is judged by its flags alone (at setup, a re-arm, a
    /// bus re-add, and when the firmware resumes reads it skipped).
    pub fn forget(&mut self) {
        *self = Flagged::default();
    }
}

/// What a [`Changes`] read says about its sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
    /// The bytes differ from the last read's (or nothing was read before): a new conversion, publish it.
    Fresh,
    /// The last read's bytes again: no conversion since, normal on a part polled faster than it converts.
    Unchanged,
    /// The same bytes for [`STALE_PERIODS`] conversion periods or longer: a part in standby. Not published; the
    /// firmware calls the part unhealthy.
    Stale,
}

/// The changed-bytes judge of a part read without a data-ready flag (see the module notes): the last read's data
/// bytes and when they last changed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Changes<const LENGTH: usize> {
    /// The last judged read's data bytes; None before the first: the first read is fresh.
    last: Option<[u8; LENGTH]>,
    /// When the bytes last changed (the first judged read counts), or when the first held read came before any
    /// judged one, microseconds on the caller's clock; None before either.
    changed_us: Option<u64>,
}

impl<const LENGTH: usize> Changes<LENGTH> {
    /// Judge one read's data `bytes`, taken at `now_us`, on a part converting every `period_us`.
    pub fn judge(&mut self, bytes: &[u8; LENGTH], now_us: u64, period_us: u64) -> Change {
        if self.last.as_ref() != Some(bytes) {
            self.last = Some(*bytes);
            self.changed_us = Some(now_us);
            return Change::Fresh;
        }
        self.since_change(now_us, period_us)
    }

    /// Answer a read that is no sample at all (a part's own "no data" pattern), taken at `now_us`, WITHOUT judging
    /// it: nothing is stored, so the next real conversion is judged against the last real one. It is
    /// [`Change::Unchanged`] until the stale clock runs out, then [`Change::Stale`], so a part that never leaves the
    /// pattern still turns unhealthy. With nothing judged yet, the clock starts at the first such read.
    pub fn hold(&mut self, now_us: u64, period_us: u64) -> Change {
        self.changed_us.get_or_insert(now_us);
        self.since_change(now_us, period_us)
    }

    /// [`Change::Stale`] once [`STALE_PERIODS`] conversion periods have passed since the last change, else
    /// [`Change::Unchanged`].
    fn since_change(&self, now_us: u64, period_us: u64) -> Change {
        let changed_us = self.changed_us.unwrap_or(now_us);
        if now_us.saturating_sub(changed_us) >= STALE_PERIODS * period_us {
            Change::Stale
        } else {
            Change::Unchanged
        }
    }
}

#[cfg(test)]
mod tests;
