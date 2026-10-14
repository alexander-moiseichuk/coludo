//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The shared latest-value store for hot data: the port of src/glider databoard.py (doc/specs/coludo.md "Task
//! Data-Flow and Message Propagation"; doc/specs/avionics-port.md §4.1 and M5).
//!
//! The structure is plan B's:
//!  * [`Parameter`] -- one fused quantity (`altitude`, `accel`, ...) for its consumers. It holds one [`Channel`]
//!    per source, KEPT IN RANK ORDER (the lowest rank, the primary, first), and ONE freshness window shared by every
//!    channel: the tightest timeout of the lowest-rank tier.
//!  * [`Channel`] -- one source's stream: its latest value and when it was pushed, behind a seqlock, so a reader on
//!    any thread sees a whole reading or none, and the writer never waits for a reader.
//!  * [`Writer`] -- the one handle a source pushes through: plan B's `provide()` handed the driver its channel, and
//!    the hot write path is one step, with no lookup.
//!
//! Fusion is a pure read: [`Parameter::read`] answers the FIRST fresh channel in rank order (same-rank channels are
//! equivalent, and the one registered first wins), with its source and its age. "Rank 0 answers while fresh; a
//! backup takes over only while itself THIS fresh" is emergent: every read judges freshness again, against the
//! one shared window, so a backup that declared 5 s is still used only while it is as fresh as the primary must be.
//!
//! Offset reconciliation (opt-in: `reconcile: true` on any provider of the parameter) decides WHAT a fallback
//! reports. While the primary is fresh, each fresh backup's BIAS against it is learned (an EMA, one look per
//! [`LEARN_PERIOD_US`]); on handover the backup reads offset-corrected, so it reports what the primary would have --
//! closing the bias gap between, say, the ICP-10111 and the BMP280 instead of jumping across it. Offsets freeze while
//! the primary is stale. It is for additive SCALARS only (altitude, pressure), never vectors or unlike quantities: a
//! [`Sample`] that has no [`Sample::difference`] never learns one.
//!
//! What is deliberately different from plan B (avionics-port.md §4.1, §6):
//!  * NO EXTRAPOLATION. Plan B's `value()` (and `read()` with source None) projected the primary's last two readings
//!    forward when nothing was fresh. A production consumer (the sequencer's `_detect_landing`, reading
//!    `agl.value()`) took a stale laser height extrapolated past the landing threshold at apogee, and the flight
//!    ended there. Here a parameter with no fresh source has NO reading: [`Parameter::read`] answers None, and no
//!    other accessor hands out a number nobody vouches for. A channel keeps one reading, not two: the second existed
//!    only to extrapolate.
//!  * Reconcile learning runs on the WRITER side, at plan B's flown cadence. Plan B learned inside `read()`, once
//!    per new primary reading AT A READ, so on the board its rate was its readers': warmstart polls `altitude` every
//!    500 ms on the pad and checkpoints it every second aloft, and `pressure` is read only by the CC inspect. That
//!    is one look every 0.5-1 s, an EMA time constant (alpha 0.1) of about 5-10 s. Readers here run on other threads
//!    and must not write, so the primary's own push learns (the same EMA, the same rules) and a reader only applies
//!    the offset -- but a push is every primary reading, 10 Hz from the ICP (§4.3), which would shrink the time
//!    constant to about 1 s and let the offset follow a boost transient (the nose baro read +83 m at speed on
//!    2026-10-03) instead of the slow bias it exists to close. So the push learns at most once per
//!    [`LEARN_PERIOD_US`], warmstart's pad poll: the time constant plan B flew with. The golden fixture
//!    (golden/build_databoard.py) reads plan B at exactly that cadence.
//!  * Born stale is a state, not a timestamp. Plan B backdated a new channel by 1 s + 1 us, which is stale only under
//!    windows up to 1 s: a GNSS-only tier at 2000 ms read FRESH, with no value, for its first second. A channel here
//!    that never pushed has no reading under any window, and [`Parameter::stamp`] answers None for it (plan B
//!    answered the backdated tick).
//!  * Time is an argument (microseconds of uptime, u64, with no 2**30 wrap) and nothing reads a clock.
//!  * No allocation, no lock, no 64-bit atomic (the P4 core has none): `no_std`, a fixed number of sources per
//!    parameter ([`MAXIMUM_SOURCES`]), and the 64-bit push time travels inside the seqlock as two words.
//!  * The registry (plan B's `Databoard` class: get-or-create by name, `provide()` over a config `provides` map,
//!    `write()` that creates a source on first use, the inspect payload) is not here. The firmware's topology is
//!    static: controller.rs builds it once at bring-up from the config, each parameter with its own value type, and
//!    freezes it (§4.5). [`Parameter::add_source`] is what `provide()` did for one quantity.

#![cfg_attr(not(test), no_std)]

use core::hint::spin_loop;
use core::marker::PhantomData;
use core::ops::Deref;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering, fence};

/// The default freshness window, and the one a timeout of 0 means: about twice the slowest sensor's period (plan B
/// `_DEFAULT_EXPIRE_US`).
pub const DEFAULT_EXPIRE_US: u32 = 1_000_000;

/// The EMA weight of one new bias sample when a backup's offset against the fresh primary is learned (plan B
/// `_OFFSET_ALPHA`): slow enough that one noisy pair barely moves it, quick enough to follow a baro's drift.
pub const OFFSET_ALPHA: f32 = 0.1;

/// The shortest time between two looks at the backups' bias: the primary's push learns only when this long has
/// passed since the last push that learned (see the crate notes on the writer side).
///
/// 500 ms is warmstart's pad poll, the reader that set plan B's learning rate where the bias that matters is
/// learned: on the pad, before a handover in flight. Aloft plan B looked once a second (the checkpoint), so this is
/// the faster of its two flown rates, never the 10 Hz of the push. The time since the last look is measured on the
/// low 32 bits of the uptime, wrapping (no 64-bit atomic): only a primary silent for a whole multiple of about 71.6
/// minutes, to within this period, skips one look, which costs one period of learning and nothing else.
pub const LEARN_PERIOD_US: u32 = 500_000;

/// How many sources one parameter can hold. The most any parameter has in the default config is four (accel:
/// LSM6DSO32, BMI323, ADXL375, BNO055; altitude: three baros and the GNSS), plus the HITL sim beside them: five.
/// Eight leaves room for an experimental board without growing a channel set the firmware keeps per parameter.
pub const MAXIMUM_SOURCES: usize = 8;

/// How many 32-bit words a channel stores a value in: four, a quaternion's floats, the widest value a sensor here
/// publishes. Every [`Sample`] packs into exactly this many.
pub const SAMPLE_WORDS: usize = 4;

/// How many times a read retries a channel whose writer is mid-push before it gives up and calls the channel stale.
///
/// A push is a handful of stores, so a writer on the other core is done after a retry or two. A writer on the SAME
/// core that this reader preempted cannot finish until the reader yields, so no count of retries helps there; the
/// read must give up rather than spin a higher-priority thread forever. Giving up reads as STALE, the safe direction
/// plan B chose for its tick wrap: a false stale costs one cycle of a fallback source, a false fresh costs the
/// flight.
const READ_ATTEMPTS: u32 = 4;

/// The sequence a channel that never pushed holds: no reading, under any window (see the crate notes, born stale).
const NEVER_PUSHED: u32 = 0;

/// A value a channel can carry: packed into [`SAMPLE_WORDS`] 32-bit words, so the seqlock moves it with plain
/// atomic word stores and a reader can never see half of one.
///
/// Reconciliation needs [`Sample::difference`] and [`Sample::shifted`]; their defaults (no difference, no shift) mark
/// a value that is never reconciled, as plan B's `TypeError` on a tuple did.
pub trait Sample: Copy {
    /// The value as words; the unused words are zero.
    fn to_words(self) -> [u32; SAMPLE_WORDS];

    /// The value [`Sample::to_words`] packed.
    fn from_words(words: [u32; SAMPLE_WORDS]) -> Self;

    /// `self - other` when the value is an additive scalar a bias can be learned for, else None (the default).
    fn difference(self, _other: Self) -> Option<f32> {
        None
    }

    /// The value corrected by a learned offset (`self + offset`); the default leaves it as it is, since a value with
    /// no difference never has an offset learned.
    fn shifted(self, _offset: f32) -> Self {
        self
    }
}

impl Sample for f32 {
    fn to_words(self) -> [u32; SAMPLE_WORDS] {
        [self.to_bits(), 0, 0, 0]
    }

    fn from_words(words: [u32; SAMPLE_WORDS]) -> Self {
        f32::from_bits(words[0])
    }

    /// A scalar's bias sample: the reference (the primary) minus this source.
    fn difference(self, other: Self) -> Option<f32> {
        Some(self - other)
    }

    /// The backup's reading as the primary would have reported it.
    fn shifted(self, offset: f32) -> Self {
        self + offset
    }
}

/// A vector of floats (accel, rate, the magnetometer, attitude as heading/roll/pitch): never reconciled.
impl<const N: usize> Sample for [f32; N] {
    fn to_words(self) -> [u32; SAMPLE_WORDS] {
        const {
            assert!(
                N <= SAMPLE_WORDS,
                "a vector wider than SAMPLE_WORDS does not fit a channel"
            )
        };
        let mut words = [0; SAMPLE_WORDS];
        for (word, value) in words.iter_mut().zip(self) {
            *word = value.to_bits();
        }
        words
    }

    fn from_words(words: [u32; SAMPLE_WORDS]) -> Self {
        core::array::from_fn(|index| f32::from_bits(words[index]))
    }
}

/// A vector of integers: a GNSS fix as latitude and longitude in 1e-7 degrees (how the port holds positions,
/// exactly), never reconciled.
impl<const N: usize> Sample for [i32; N] {
    fn to_words(self) -> [u32; SAMPLE_WORDS] {
        const {
            assert!(
                N <= SAMPLE_WORDS,
                "a vector wider than SAMPLE_WORDS does not fit a channel"
            )
        };
        let mut words = [0; SAMPLE_WORDS];
        for (word, value) in words.iter_mut().zip(self) {
            *word = value.cast_unsigned();
        }
        words
    }

    fn from_words(words: [u32; SAMPLE_WORDS]) -> Self {
        core::array::from_fn(|index| words[index].cast_signed())
    }
}

/// Is a reading pushed at `stamp_us` fresh at `now_us` under `window_us`?
///
/// THE LOWER BOUND IS NOT OPTIONAL (plan B `_Channel.fresh`). A push stamped AFTER `now` -- the caller sampled the
/// clock, then another thread pushed before the check -- reads stale for that one cycle rather than fresh. Plan B
/// needed the bound for its 2**30 tick wrap (a sensor silent 8.95-17.9 minutes gave a negative age and read fresh, on
/// the pad, past the readiness gate); the uptime here does not wrap, but the direction stays the safe one.
fn within(stamp_us: u64, now_us: u64, window_us: u32) -> bool {
    now_us >= stamp_us && now_us - stamp_us <= u64::from(window_us)
}

/// One source's stream within a [`Parameter`]: its latest value and push time, behind a seqlock.
///
/// The seqlock, all of it atomic words (so no read is ever a data race, only a retry): `sequence` is
/// `NEVER_PUSHED` (0) until the first push, odd while a push is in progress and even otherwise; a reader takes the
/// words between two equal even loads of it. Exactly one [`Writer`] pushes at a time, which the claim enforces.
///
/// Beside the reading, the bias learned against the primary while this channel is a backup ([`Parameter`]'s
/// reconciliation); only the primary's writer stores it.
pub struct Channel<T> {
    /// The seqlock sequence (see the type notes).
    sequence: AtomicU32,
    /// The push time, microseconds of uptime, low word.
    stamp_low: AtomicU32,
    /// The push time, high word: a u64 without a 64-bit atomic.
    stamp_high: AtomicU32,
    /// The value, as [`Sample::to_words`] packs it.
    words: [AtomicU32; SAMPLE_WORDS],
    /// The learned bias's f32 bits, meaningful once `offset_learned` is set.
    offset_bits: AtomicU32,
    /// Whether a bias was ever learned (plan B's `offset` was None until then).
    offset_learned: AtomicBool,
    /// Whether a [`Writer`] holds this channel: one writer at a time, or two pushes could interleave.
    claimed: AtomicBool,
    /// The value type, carried by the words.
    value: PhantomData<fn() -> T>,
}

impl<T: Sample> Channel<T> {
    /// An empty channel: never pushed, no offset, unclaimed.
    const fn new() -> Self {
        Channel {
            sequence: AtomicU32::new(NEVER_PUSHED),
            stamp_low: AtomicU32::new(0),
            stamp_high: AtomicU32::new(0),
            words: [const { AtomicU32::new(0) }; SAMPLE_WORDS],
            offset_bits: AtomicU32::new(0),
            offset_learned: AtomicBool::new(false),
            claimed: AtomicBool::new(false),
            value: PhantomData,
        }
    }

    /// Store a reading: the seqlock write. Only the claiming [`Writer`] calls this, so the plain load of the
    /// sequence reads this thread's own last store.
    fn write(&self, value: T, now_us: u64) {
        let sequence = self.sequence.load(Ordering::Relaxed);
        self.sequence.store(sequence.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release); // the odd sequence is visible before any word changes
        self.stamp_low.store(now_us as u32, Ordering::Relaxed);
        self.stamp_high.store((now_us >> 32) as u32, Ordering::Relaxed);
        for (word, packed) in self.words.iter().zip(value.to_words()) {
            word.store(packed, Ordering::Relaxed);
        }
        // the next even sequence, skipping the never-pushed mark when it wraps (after 2**31 pushes)
        let next = match sequence.wrapping_add(2) {
            NEVER_PUSHED => 2,
            next => next,
        };
        self.sequence.store(next, Ordering::Release);
    }

    /// The latest reading and its push time, read whole; None when the channel never pushed, or when a push stayed in
    /// progress through [`READ_ATTEMPTS`] tries (read as stale: see there).
    fn snapshot(&self) -> Option<(u64, T)> {
        for _ in 0..READ_ATTEMPTS {
            let before = self.sequence.load(Ordering::Acquire);
            if before == NEVER_PUSHED {
                return None;
            }
            if before & 1 == 0 {
                let stamp = u64::from(self.stamp_low.load(Ordering::Relaxed))
                    | (u64::from(self.stamp_high.load(Ordering::Relaxed)) << 32);
                let words = core::array::from_fn(|index| self.words[index].load(Ordering::Relaxed));
                fence(Ordering::Acquire); // the words are read before the sequence is checked again
                if self.sequence.load(Ordering::Relaxed) == before {
                    return Some((stamp, T::from_words(words)));
                }
            }
            spin_loop();
        }
        None
    }

    /// The latest value, whatever its age; None before the first push, or, for one read, while a push on the same
    /// core was preempted mid-write (see `READ_ATTEMPTS`) (plan B `_Channel.value()`).
    ///
    /// For a source's own status display only: it answers the last push forever, so a sensor that stopped an hour
    /// ago still looks present. Never decide on it; [`Channel::recent`] and [`Parameter::read`] are the readings.
    pub fn latest(&self) -> Option<T> {
        self.snapshot().map(|(_, value)| value)
    }

    /// The latest value if it was pushed within `window_us` of `now_us`, else None (plan B `_Channel.recent`); a
    /// push preempted mid-write on the same core reads as not recent for that read (see `READ_ATTEMPTS`).
    ///
    /// For a source reading back its OWN channel to latch or vouch for a number -- a baro re-zero, a probe's "the
    /// run loop is producing". [`Channel::latest`] cannot answer that, and the parameter's read is no substitute
    /// either: it answers the WINNING source, which for a backup is somebody else.
    pub fn recent(&self, window_us: u32, now_us: u64) -> Option<T> {
        self.snapshot()
            .filter(|(stamp, _)| within(*stamp, now_us, window_us))
            .map(|(_, value)| value)
    }

    /// When the latest value was pushed, microseconds of uptime; None before the first push, or, for one read, while
    /// a push on the same core was preempted mid-write (see `READ_ATTEMPTS`). So one None is not proof that the
    /// channel never pushed: a probe that declares a sensor dead on it must see None again after yielding.
    pub fn pushed_at(&self) -> Option<u64> {
        self.snapshot().map(|(stamp, _)| stamp)
    }

    /// Whether this channel pushed within `window_us` of `now_us` (plan B `_Channel.fresh`, lower bound included).
    pub fn fresh(&self, now_us: u64, window_us: u32) -> bool {
        self.snapshot()
            .is_some_and(|(stamp, _)| within(stamp, now_us, window_us))
    }

    /// The bias learned against the primary, if any was.
    fn offset(&self) -> Option<f32> {
        self.offset_learned
            .load(Ordering::Acquire)
            .then(|| f32::from_bits(self.offset_bits.load(Ordering::Relaxed)))
    }

    /// Store a learned bias. Only the primary's writer calls this, so each offset has one writer; the bits go
    /// before the flag, so a reader that sees the flag sees a learned value.
    fn set_offset(&self, offset: f32) {
        self.offset_bits.store(offset.to_bits(), Ordering::Relaxed);
        self.offset_learned.store(true, Ordering::Release);
    }
}

/// One fused reading: the winner's value (offset-corrected when it is a reconciled backup), the source that pushed
/// it, and how old it is. There is no "extrapolated" form: no fresh source is no reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading<T> {
    /// The value.
    pub value: T,
    /// The device that pushed it (the raw provider, even when the value is offset-corrected), as registered.
    pub source: &'static str,
    /// Microseconds since that push, at most the parameter's window.
    pub age_us: u32,
}

/// A parameter already holds [`MAXIMUM_SOURCES`] sources and a new one cannot register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TooManySources;

/// One fused quantity: up to [`MAXIMUM_SOURCES`] channels in rank order and the window they share (plan B
/// `Parameter`).
///
/// Sources register through `&mut self` at bring-up ([`Parameter::add_source`]); pushes and reads take `&self`, so a
/// parameter shared between threads (a `&'static` once the topology is frozen) cannot change its sources while
/// anyone holds a [`Writer`].
pub struct Parameter<T> {
    /// The quantity's name, as consumers and the inspect payload know it.
    name: &'static str,
    /// The channels, one per registered source, in REGISTRATION slots (a slot never moves; `order` ranks them).
    channels: [Channel<T>; MAXIMUM_SOURCES],
    /// Each slot's source name.
    sources: [&'static str; MAXIMUM_SOURCES],
    /// Each slot's rank (the config's `priority`; lower is preferred, and may be negative as plan B's int).
    ranks: [i32; MAXIMUM_SOURCES],
    /// The slots in rank order, stable among equal ranks (plan B's sorted list): `order[0]` is the primary.
    order: [u8; MAXIMUM_SOURCES],
    /// How many slots are registered.
    count: usize,
    /// The shared freshness window: the tightest timeout of the primary tier, applied to EVERY channel.
    window_us: u32,
    /// The rank of the tier the window was set from; None until a source registers (plan B's `1 << 30`).
    primary_rank: Option<i32>,
    /// Whether any provider asked for offset reconciliation (sticky, as plan B's flag).
    reconcile: bool,
    /// The low 32 bits of the uptime of the last primary push that learned, meaningful once `looked` is set (plan
    /// B's `_learned_t1`, but a time rather than a reading: see [`LEARN_PERIOD_US`]). Only the primary's writer
    /// touches it, one writer at a time, and the claim orders a handover between threads, so relaxed access is
    /// enough; it is atomic because the parameter is shared.
    looked_at_us: AtomicU32,
    /// Whether the primary's push ever learned: the first one always does.
    looked: AtomicBool,
}

impl<T: Sample> Parameter<T> {
    /// An empty parameter: no sources, the default window. `const`, so a fixed topology can live in a static.
    pub const fn new(name: &'static str) -> Self {
        Parameter {
            name,
            channels: [const { Channel::new() }; MAXIMUM_SOURCES],
            sources: [""; MAXIMUM_SOURCES],
            ranks: [0; MAXIMUM_SOURCES],
            order: [0; MAXIMUM_SOURCES],
            count: 0,
            window_us: DEFAULT_EXPIRE_US,
            primary_rank: None,
            reconcile: false,
            looked_at_us: AtomicU32::new(0),
            looked: AtomicBool::new(false),
        }
    }

    /// The quantity's name.
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// Register (or re-register) `source` at rank `priority`, with its config `timeout_ms` (0 is the default
    /// second) and its `reconcile` flag: plan B's `Databoard.provide` for this quantity, through
    /// `Parameter.add_source`.
    ///
    /// The channels stay in rank order, so no write ever looks anything up. The shared window is maintained here --
    /// the only place a timeout is consulted -- with plan B's incremental rule:
    ///  * a source of a LOWER rank than the current primary tier starts a new tier, and the window resets to its
    ///    timeout;
    ///  * another source of the tier's rank takes the window down to its timeout if that is tighter;
    ///  * any other leaves the window alone.
    ///
    /// Re-registering keeps the source's slot and data and takes the new rank, re-sorting; it never recomputes the
    /// tier, so re-ranking the primary worse keeps the tier's rank and window (a setup retry re-registers at the same
    /// rank, which changes nothing). `reconcile` from any provider turns reconciliation on for good.
    ///
    /// A timeout past what microseconds hold in a u32 (about 71 minutes) saturates there.
    ///
    /// Errors: [`TooManySources`] when a new source finds [`MAXIMUM_SOURCES`] registered (plan B grew without
    /// bound); nothing changes then.
    pub fn add_source(
        &mut self,
        source: &'static str,
        priority: i32,
        timeout_ms: u32,
        reconcile: bool,
    ) -> Result<(), TooManySources> {
        match self.slot(source) {
            Some(slot) => self.ranks[slot] = priority,
            None => {
                if self.count == MAXIMUM_SOURCES {
                    return Err(TooManySources);
                }
                let slot = self.count;
                self.sources[slot] = source;
                self.ranks[slot] = priority;
                self.order[slot] = slot as u8; // appended last in rank order, as plan B appends, before the sort
                self.count += 1;
            }
        }
        self.sort();
        self.reconcile |= reconcile;
        let expire_us = match timeout_ms {
            0 => DEFAULT_EXPIRE_US,
            timeout_ms => timeout_ms.saturating_mul(1000),
        };
        match self.primary_rank {
            Some(tier) if priority > tier => {}
            Some(tier) if priority == tier => self.window_us = self.window_us.min(expire_us),
            _ => {
                self.primary_rank = Some(priority);
                self.window_us = expire_us;
            }
        }
        Ok(())
    }

    /// Re-sort `order` by rank, keeping the present order among equal ranks (an insertion sort: stable, like
    /// Python's list.sort, and over a handful of slots).
    fn sort(&mut self) {
        for position in 1..self.count {
            let mut index = position;
            while index > 0
                && self.ranks[usize::from(self.order[index - 1])] > self.ranks[usize::from(self.order[index])]
            {
                self.order.swap(index - 1, index);
                index -= 1;
            }
        }
    }

    /// The slot `source` registered in, if it did.
    fn slot(&self, source: &str) -> Option<usize> {
        self.sources[..self.count].iter().position(|name| *name == source)
    }

    /// The registered slots in rank order, the primary first.
    fn ranked(&self) -> impl Iterator<Item = usize> + '_ {
        self.order[..self.count].iter().map(|slot| usize::from(*slot))
    }

    /// The handle `source` pushes through; None when it never registered or another writer holds it already.
    ///
    /// One writer per channel at a time, so a seqlock write never interleaves with another; dropping the writer
    /// releases the channel (a device set up again takes a new one).
    pub fn writer(&self, source: &str) -> Option<Writer<'_, T>> {
        let slot = self.slot(source)?;
        self.channels[slot]
            .claimed
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .ok()?;
        Some(Writer { parameter: self, slot })
    }

    /// One source's channel, to read it alone (plan B `raw(source)` and a driver's look at its own stream); None
    /// for a source that never registered.
    pub fn channel(&self, source: &str) -> Option<&Channel<T>> {
        self.slot(source).map(|slot| &self.channels[slot])
    }

    /// The fused reading at `now_us`: the first channel in rank order that is fresh within the shared window, with
    /// its value, source and age; None when no source is fresh (or none registered).
    ///
    /// A reconciled backup (the parameter reconciles, the winner is not the primary, and it has a learned offset)
    /// reads offset-corrected, with its own name as the source. There is no fallback past "no fresh source": see the
    /// crate notes on extrapolation.
    pub fn read(&self, now_us: u64) -> Option<Reading<T>> {
        for (position, slot) in self.ranked().enumerate() {
            let channel = &self.channels[slot];
            let Some((stamp, value)) = channel.snapshot() else {
                continue;
            };
            if !within(stamp, now_us, self.window_us) {
                continue;
            }
            let value = match (self.reconcile && position != 0, channel.offset()) {
                (true, Some(offset)) => value.shifted(offset),
                _ => value,
            };
            return Some(Reading {
                value,
                source: self.sources[slot],
                age_us: (now_us - stamp) as u32, // within the window, so within u32
            });
        }
        None
    }

    /// Whether a source ranked BELOW `rank` (a larger number) is fresh at `now_us` (plan B `fresh_below`).
    ///
    /// For a provider deciding whether it may fall silent: the attitude filter withholds a blind, frozen estimate only
    /// while something under it can carry the parameter -- with nothing fresh below, going quiet would hand the
    /// consumer no attitude at all.
    pub fn fresh_below(&self, rank: i32, now_us: u64) -> bool {
        self.ranked()
            .any(|slot| self.ranks[slot] > rank && self.channels[slot].fresh(now_us, self.window_us))
    }

    /// The rank `source` registered at, or None when it never did (plan B `rank`).
    ///
    /// For a provider that must tell a source that OUTRANKS it from one it outranks: the attitude filter mirrors
    /// the first (the HITL sim) and must never copy the second (the SEN0253's BNO055, standing by below it).
    pub fn rank(&self, source: &str) -> Option<i32> {
        self.slot(source).map(|slot| self.ranks[slot])
    }

    /// When the primary last pushed, microseconds of uptime; None with no source, before the primary's first push,
    /// or, for one read, while a push on the same core was preempted mid-write (see `READ_ATTEMPTS`) (plan B
    /// `stamp`).
    ///
    /// For change detection: a consumer feeds once per NEW reading by watching this advance, so it follows the
    /// provider's own rate (a 1/5/25 Hz GNSS) with no polling-period constant. A change detector must therefore
    /// read None as "no news", never as a reset: comparing it with the last stamp seen, None is no new reading.
    pub fn stamp(&self) -> Option<u64> {
        self.ranked().next().and_then(|slot| self.channels[slot].pushed_at())
    }

    /// The learned bias per source, in rank order, for diagnostics; empty until something was reconciled (plan B
    /// `offsets`).
    pub fn offsets(&self) -> impl Iterator<Item = (&'static str, f32)> + '_ {
        self.ranked()
            .filter_map(|slot| Some((self.sources[slot], self.channels[slot].offset()?)))
    }

    /// The registered sources in rank order, the primary first (plan B kept its channels in this order; its
    /// `sources()` sorted them by name, which the caller can do).
    pub fn sources(&self) -> impl Iterator<Item = &'static str> + '_ {
        self.ranked().map(|slot| self.sources[slot])
    }

    /// The shared freshness window in microseconds.
    pub fn window_us(&self) -> u32 {
        self.window_us
    }

    /// Whether the primary's push at `now_us` is this period's look at the backups; when it is, the next period
    /// starts at `now_us`.
    ///
    /// The look is taken whether or not a backup is fresh to learn from, as plan B's reader marked a primary reading
    /// learned (`_learned_t1`) with no fresh backup beside it: the period is the reader's cadence, not the backups'.
    fn look_due(&self, now_us: u64) -> bool {
        let now = now_us as u32; // the low word, compared wrapping (see LEARN_PERIOD_US)
        if self.looked.load(Ordering::Relaxed)
            && now.wrapping_sub(self.looked_at_us.load(Ordering::Relaxed)) < LEARN_PERIOD_US
        {
            return false;
        }
        self.looked_at_us.store(now, Ordering::Relaxed);
        self.looked.store(true, Ordering::Relaxed);
        true
    }

    /// Learn each fresh backup's bias against the primary's new reading (plan B `_learn`, on the writer side).
    ///
    /// Called from the primary's own push when [`Parameter::look_due`] says so, so the primary is fresh and the
    /// reading is new by construction. Co-primaries (the primary's rank) are not reconciled; a stale backup is
    /// skipped, so offsets FREEZE while their pair is not both fresh; a value with no [`Sample::difference`] (a
    /// vector) never learns.
    fn learn(&self, primary: usize, reference: T, now_us: u64) {
        for slot in self.ranked().skip(1) {
            if self.ranks[slot] == self.ranks[primary] {
                continue;
            }
            let backup = &self.channels[slot];
            let Some((stamp, value)) = backup.snapshot() else {
                continue;
            };
            if !within(stamp, now_us, self.window_us) {
                continue;
            }
            let Some(sample) = reference.difference(value) else {
                continue;
            };
            let offset = match backup.offset() {
                None => sample,
                Some(offset) => offset + OFFSET_ALPHA * (sample - offset),
            };
            backup.set_offset(offset);
        }
    }
}

/// The handle one source pushes its readings through; it also reads its own channel back (it derefs to the
/// [`Channel`]: [`Channel::recent`], [`Channel::latest`]).
///
/// Exactly one exists per channel at a time ([`Parameter::writer`]), and `push` takes `&mut self`, so the seqlock
/// always has a single writer. Dropping it releases the channel.
pub struct Writer<'parameter, T: Sample> {
    /// The parameter the channel belongs to: the primary's push learns its backups' offsets there.
    parameter: &'parameter Parameter<T>,
    /// The channel's registration slot.
    slot: usize,
}

impl<T: Sample> Writer<'_, T> {
    /// Record a reading taken at `now_us`. When this source is the parameter's primary and the parameter reconciles,
    /// the push also learns each fresh backup's bias against it, at most once per [`LEARN_PERIOD_US`] (the
    /// writer-side learning of the crate notes).
    pub fn push(&mut self, value: T, now_us: u64) {
        let parameter = self.parameter;
        parameter.channels[self.slot].write(value, now_us);
        if parameter.reconcile && parameter.ranked().next() == Some(self.slot) && parameter.look_due(now_us) {
            parameter.learn(self.slot, value, now_us);
        }
    }

    /// The source's name, as registered.
    pub fn source(&self) -> &'static str {
        self.parameter.sources[self.slot]
    }

    /// The source's rank (plan B's `channel.rank`): what a provider passes to [`Parameter::fresh_below`].
    pub fn rank(&self) -> i32 {
        self.parameter.ranks[self.slot]
    }
}

impl<T: Sample> Deref for Writer<'_, T> {
    type Target = Channel<T>;

    /// The writer's own channel, to read back.
    fn deref(&self) -> &Channel<T> {
        &self.parameter.channels[self.slot]
    }
}

impl<T: Sample> Drop for Writer<'_, T> {
    /// Release the channel for the next writer.
    fn drop(&mut self) {
        self.parameter.channels[self.slot]
            .claimed
            .store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::{
        Channel, DEFAULT_EXPIRE_US, LEARN_PERIOD_US, MAXIMUM_SOURCES, NEVER_PUSHED, OFFSET_ALPHA, Parameter, Reading,
        Sample, TooManySources,
    };
    use core::sync::atomic::Ordering;

    /// A parameter with sources (name, rank, timeout_ms) registered in order, none reconciling.
    fn parameter(sources: &[(&'static str, i32, u32)]) -> Parameter<f32> {
        let mut parameter = Parameter::new("altitude");
        for (source, rank, timeout_ms) in sources {
            parameter.add_source(source, *rank, *timeout_ms, false).unwrap();
        }
        parameter
    }

    /// Push one value from `source` at `now_us`.
    fn push<T: Sample>(parameter: &Parameter<T>, source: &str, value: T, now_us: u64) {
        parameter.writer(source).expect("a free writer").push(value, now_us);
    }

    /// The sources in rank order.
    fn order<T: Sample>(parameter: &Parameter<T>) -> Vec<&'static str> {
        parameter.sources().collect()
    }

    #[test]
    fn a_channel_that_never_pushed_has_no_reading_under_any_window() {
        // plan B's backdated birth made a 2 s window read FRESH (with no value) for the first second: not here
        for timeout_ms in [0, 1000, 2000, 5000, u32::MAX] {
            let parameter = parameter(&[("gnss", 0, timeout_ms)]);
            assert_eq!(parameter.read(0), None, "{timeout_ms}");
            assert_eq!(parameter.read(1), None, "{timeout_ms}");
            assert!(!parameter.fresh_below(-1, 1), "{timeout_ms}");
            assert_eq!(parameter.stamp(), None);
            let channel = parameter.channel("gnss").unwrap();
            assert_eq!(
                (channel.latest(), channel.recent(u32::MAX, 0), channel.pushed_at()),
                (None, None, None)
            );
            assert!(!channel.fresh(0, u32::MAX));
        }
        // and the empty parameter answers nothing
        let empty: Parameter<f32> = Parameter::new("agl");
        assert_eq!(
            (empty.read(5), empty.stamp(), empty.window_us()),
            (None, None, DEFAULT_EXPIRE_US)
        );
        assert!(!empty.fresh_below(i32::MIN, 5));
    }

    #[test]
    fn rank_zero_answers_while_fresh_and_a_backup_only_inside_the_shared_window() {
        let parameter = parameter(&[("icp", 0, 50), ("bmp", 1, 500)]);
        push(&parameter, "icp", 100.0, 1_000);
        push(&parameter, "bmp", 200.0, 1_000);
        assert_eq!(
            parameter.read(1_000),
            Some(Reading {
                value: 100.0,
                source: "icp",
                age_us: 0
            })
        );
        // the primary lapses past its 50 ms; the backup pushes and takes over
        push(&parameter, "bmp", 201.0, 71_000);
        assert_eq!(
            parameter.read(71_000),
            Some(Reading {
                value: 201.0,
                source: "bmp",
                age_us: 0
            })
        );
        // the backup declared 500 ms, but only the shared 50 ms counts: ON the window fresh, one us past not
        assert_eq!(parameter.read(121_000).map(|reading| reading.age_us), Some(50_000));
        assert_eq!(parameter.read(121_001), None, "no extrapolation, no stale backup");
        // the primary back wins at once
        push(&parameter, "icp", 99.5, 130_000);
        assert_eq!(parameter.read(130_000).unwrap().source, "icp");
    }

    #[test]
    fn a_window_past_one_second_reads_its_whole_length() {
        // a GNSS-only altitude tier at 2000 ms, a 5 s backup below it: the window is the tier's 2 s, all of it
        let parameter = parameter(&[("gnss", 0, 2000), ("bmp280", 1, 5000)]);
        assert_eq!(parameter.window_us(), 2_000_000);
        push(&parameter, "gnss", 312.5, 0);
        push(&parameter, "bmp280", 310.0, 0);
        assert_eq!(parameter.read(1_500_000).map(|reading| reading.age_us), Some(1_500_000));
        assert!(
            parameter.fresh_below(0, 1_500_000),
            "the backup is fresh at 1.5 s in the 2 s window"
        );
        assert_eq!(
            parameter.read(2_000_000).map(|reading| reading.source),
            Some("gnss"),
            "ON the window"
        );
        // a driver's own probe: recent(3 x a 500 ms period) at 1.2 s
        let gnss = parameter.channel("gnss").unwrap();
        assert_eq!(gnss.recent(1_500_000, 1_200_000), Some(312.5));
        // NEGATIVE: one us past the window, and a probe window shorter than the age
        assert_eq!(parameter.read(2_000_001), None, "the backup's own 5 s does not count");
        assert!(!parameter.fresh_below(0, 2_000_001));
        assert_eq!(gnss.recent(1_000_000, 1_200_000), None);
    }

    #[test]
    fn a_push_stamped_after_now_reads_stale_for_that_cycle() {
        let parameter = parameter(&[("imu", 0, 20)]);
        push(&parameter, "imu", 1.0, 5_000);
        assert_eq!(
            parameter.read(4_999),
            None,
            "a push newer than the caller's now is not fresh"
        );
        assert!(!parameter.channel("imu").unwrap().fresh(4_999, 20_000));
        assert!(parameter.read(5_000).is_some());
    }

    #[test]
    fn the_window_is_the_tightest_timeout_of_the_lowest_rank_tier() {
        let mut parameter: Parameter<f32> = Parameter::new("altitude");
        let mut add = |source: &'static str, rank: i32, timeout_ms: u32| {
            parameter.add_source(source, rank, timeout_ms, false).unwrap();
            parameter.window_us()
        };
        assert_eq!(add("gnss", 3, 2000), 2_000_000, "the first tier sets it");
        assert_eq!(add("bmp280", 2, 300), 300_000, "a lower tier resets it");
        assert_eq!(add("bmp581", 1, 500), 500_000, "a lower tier resets it, even looser");
        assert_eq!(add("hitl", 1, 200), 200_000, "the same tier takes the min");
        assert_eq!(add("laser", 1, 400), 200_000, "the same tier, looser: unchanged");
        assert_eq!(add("bno055", 4, 10), 200_000, "a higher tier: unchanged");
        assert_eq!(add("icp", 0, 0), DEFAULT_EXPIRE_US, "0 is the default second");
        assert_eq!(add("attitude", 0, 100), 100_000);
        assert_eq!(
            add("icp", 5, 50),
            100_000,
            "re-ranking keeps the tier and its window (plan B)"
        );
        assert_eq!(add("bmp581", 0, 30), 30_000, "back in the tier's rank: the min again");
    }

    #[test]
    fn a_timeout_past_u32_microseconds_saturates() {
        let parameter = parameter(&[("gnss", 0, u32::MAX)]);
        assert_eq!(parameter.window_us(), u32::MAX);
    }

    #[test]
    fn equal_ranks_keep_registration_order_and_reregistration_resorts() {
        let mut parameter = parameter(&[("b", 1, 100), ("a", 1, 100), ("c", 0, 100)]);
        assert_eq!(
            order(&parameter),
            ["c", "b", "a"],
            "stable among equals, as Python's sort"
        );
        push(&parameter, "b", 7.0, 10);
        parameter.add_source("b", 2, 100, false).unwrap();
        assert_eq!(order(&parameter), ["c", "a", "b"]);
        parameter.add_source("a", 1, 100, false).unwrap(); // a setup retry: nothing moves
        assert_eq!(order(&parameter), ["c", "a", "b"]);
        assert_eq!(parameter.rank("b"), Some(2));
        assert_eq!(parameter.rank("nobody"), None);
        assert_eq!(
            parameter.channel("b").unwrap().latest(),
            Some(7.0),
            "re-registering keeps the data"
        );
        // both co-primaries fresh: the first registered answers; it lapsing hands over to the second
        let both = parameter_with_pushes();
        assert_eq!(both.read(10_000).unwrap().source, "first");
        assert_eq!(both.read(60_000).unwrap().source, "second");
    }

    /// Two rank-0 sources in a 20 ms window: both pushed at 0, the second again at 50 ms.
    fn parameter_with_pushes() -> Parameter<f32> {
        let parameter = parameter(&[("first", 0, 20), ("second", 0, 20)]);
        push(&parameter, "first", 1.0, 0);
        push(&parameter, "second", 2.0, 0);
        push(&parameter, "second", 2.5, 50_000);
        parameter
    }

    #[test]
    fn too_many_sources_is_refused_and_changes_nothing() {
        let names = ["s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7"];
        let mut parameter: Parameter<f32> = Parameter::new("accel");
        for (rank, name) in names.into_iter().enumerate() {
            parameter.add_source(name, rank as i32, 100, false).unwrap();
        }
        assert_eq!(names.len(), MAXIMUM_SOURCES);
        assert_eq!(parameter.add_source("s8", -1, 5, true), Err(TooManySources));
        assert_eq!(order(&parameter), names, "a refused source is not registered");
        assert_eq!(parameter.window_us(), 100_000, "nor does it touch the window");
        // a known source still re-registers at capacity
        assert_eq!(parameter.add_source("s7", -1, 50, false), Ok(()));
        assert_eq!(order(&parameter)[0], "s7");
    }

    #[test]
    fn one_writer_per_channel_until_it_is_dropped() {
        let parameter = parameter(&[("icp", 0, 50)]);
        let mut writer = parameter.writer("icp").unwrap();
        assert!(
            parameter.writer("icp").is_none(),
            "a second writer would interleave pushes"
        );
        assert!(parameter.writer("nobody").is_none());
        writer.push(3.0, 1);
        assert_eq!((writer.source(), writer.rank(), writer.latest()), ("icp", 0, Some(3.0)));
        drop(writer);
        assert!(parameter.writer("icp").is_some(), "dropped: free again");
    }

    #[test]
    fn recent_and_latest_read_a_source_back() {
        let parameter = parameter(&[("icp", 0, 50), ("bmp", 1, 200)]);
        push(&parameter, "bmp", 120.25, 1_000);
        let bmp = parameter.writer("bmp").unwrap();
        assert_eq!(
            bmp.recent(150_000, 151_000),
            Some(120.25),
            "its own window, not the parameter's"
        );
        assert_eq!(bmp.recent(149_999, 151_000), None);
        assert_eq!(bmp.latest(), Some(120.25), "any age");
        assert_eq!(bmp.pushed_at(), Some(1_000));
        assert_eq!(parameter.read(151_000), None, "the parameter's 50 ms window has lapsed");
    }

    #[test]
    fn fresh_below_counts_only_larger_ranks_fresh_in_the_window() {
        let parameter = parameter(&[("attitude", 1, 40), ("bno055", 2, 40), ("hitl", 0, 40)]);
        push(&parameter, "bno055", 1.5, 0);
        assert!(parameter.fresh_below(1, 40_000), "ON the window");
        assert!(!parameter.fresh_below(1, 40_001));
        assert!(!parameter.fresh_below(2, 0), "an equal rank is not below");
        assert!(parameter.fresh_below(-1, 0));
    }

    #[test]
    fn stamp_follows_the_primary_only() {
        let parameter = parameter(&[("gnss", 0, 500), ("hitl", 1, 500)]);
        push(&parameter, "hitl", 9.0, 5);
        assert_eq!(parameter.stamp(), None, "a backup push is no primary stamp");
        push(&parameter, "gnss", 84.0, 10);
        push(&parameter, "hitl", 9.0, 20);
        assert_eq!(parameter.stamp(), Some(10));
        assert_eq!(parameter.stamp(), Some(10), "it holds while the primary is stale");
        push(&parameter, "gnss", 85.0, 1_000_000);
        assert_eq!(parameter.stamp(), Some(1_000_000));
    }

    #[test]
    fn a_write_in_progress_reads_as_no_reading() {
        // a writer preempted mid-push by a reader on its own core: the read gives up, as stale, instead of spinning
        let parameter = parameter(&[("imu", 0, 20), ("backup", 1, 20)]);
        push(&parameter, "imu", 1.0, 0);
        push(&parameter, "backup", 2.0, 0);
        let channel = parameter.channel("imu").unwrap();
        let even = channel.sequence.load(Ordering::Relaxed);
        channel.sequence.store(even + 1, Ordering::Relaxed);
        assert_eq!(channel.latest(), None);
        assert_eq!(
            (channel.pushed_at(), parameter.stamp()),
            (None, None),
            "None here is not 'never pushed'"
        );
        assert_eq!(
            parameter.read(0).unwrap().source,
            "backup",
            "the fallback carries the cycle"
        );
        channel.sequence.store(even, Ordering::Relaxed);
        assert_eq!(parameter.read(0).unwrap().source, "imu");
        assert_eq!(
            (channel.pushed_at(), parameter.stamp()),
            (Some(0), Some(0)),
            "back once the push is done"
        );
    }

    #[test]
    fn the_sequence_wrap_skips_the_never_pushed_mark() {
        let channel: Channel<f32> = Channel::new();
        channel.sequence.store(u32::MAX - 1, Ordering::Relaxed);
        channel.write(4.0, 9);
        assert_eq!(channel.sequence.load(Ordering::Relaxed), 2);
        assert_ne!(channel.sequence.load(Ordering::Relaxed), NEVER_PUSHED);
        assert_eq!(channel.snapshot(), Some((9, 4.0)));
    }

    #[test]
    fn a_stamp_past_32_bits_survives_the_two_words() {
        let parameter = parameter(&[("gnss", 0, 0)]);
        let late = (1_u64 << 40) + 12_345;
        push(&parameter, "gnss", 1.0, late);
        assert_eq!(parameter.stamp(), Some(late));
        assert_eq!(parameter.read(late + 1_000_000).unwrap().age_us, 1_000_000);
    }

    /// The altitude pair of test_databoard.py, reconciling: icp the primary (50 ms), bmp the backup.
    fn reconciling() -> Parameter<f32> {
        let mut parameter: Parameter<f32> = Parameter::new("altitude");
        parameter.add_source("icp", 0, 50, true).unwrap();
        parameter.add_source("bmp", 1, 5000, false).unwrap();
        parameter
    }

    /// One look period, in the microseconds a push takes.
    const PERIOD: u64 = LEARN_PERIOD_US as u64;

    #[test]
    fn the_primary_push_learns_the_backup_bias_and_a_handover_corrects_it() {
        let parameter = reconciling();
        let mut expected = None;
        let mut now = 0;
        for (index, backup) in [102.5_f32, 102.25, 103.0].into_iter().enumerate() {
            now = index as u64 * PERIOD;
            push(&parameter, "bmp", backup, now);
            push(&parameter, "icp", 100.0, now);
            let sample = 100.0 - backup;
            expected = Some(expected.map_or(sample, |offset: f32| offset + OFFSET_ALPHA * (sample - offset)));
            assert_eq!(parameter.read(now).unwrap().value, 100.0, "the fresh primary reads raw");
        }
        let offsets: Vec<_> = parameter.offsets().collect();
        assert_eq!(offsets, [("bmp", expected.unwrap())]);
        // the primary lapses; the backup reads as the primary would have, under its own name
        push(&parameter, "bmp", 102.5, now + 60_000);
        let reading = parameter.read(now + 60_000).unwrap();
        assert_eq!(reading.source, "bmp");
        assert_eq!(reading.value, 102.5 + expected.unwrap());
    }

    #[test]
    fn a_backup_push_teaches_nothing_and_offsets_freeze_while_the_primary_is_stale() {
        let parameter = reconciling();
        push(&parameter, "icp", 100.0, 0); // the backup never pushed: nothing to learn
        assert_eq!(parameter.offsets().count(), 0);
        push(&parameter, "bmp", 102.0, 0);
        assert_eq!(parameter.offsets().count(), 0, "only the primary's push learns");
        push(&parameter, "bmp", 102.0, PERIOD);
        push(&parameter, "icp", 100.0, PERIOD);
        assert_eq!(parameter.offsets().collect::<Vec<_>>(), [("bmp", -2.0)]);
        push(&parameter, "bmp", 150.0, PERIOD + 100_000); // the primary stale: frozen
        push(&parameter, "icp", 100.0, 2 * PERIOD); // a look, but the backup is stale at it: frozen
        assert_eq!(parameter.offsets().collect::<Vec<_>>(), [("bmp", -2.0)]);
    }

    #[test]
    fn the_primary_learns_once_per_period_however_often_it_pushes() {
        // plan B learned at its readers' cadence (warmstart: 500 ms on the pad), not at the 10 Hz push
        let parameter = reconciling();
        push(&parameter, "bmp", 101.0, 0);
        push(&parameter, "icp", 100.0, 0);
        assert_eq!(
            parameter.offsets().collect::<Vec<_>>(),
            [("bmp", -1.0)],
            "the first push looks"
        );
        // NEGATIVE: every push inside the period teaches nothing, however far the backup moves
        for now in [100_000, 200_000, 300_000, 400_000, PERIOD - 1] {
            push(&parameter, "bmp", 110.0, now);
            push(&parameter, "icp", 100.0, now);
            assert_eq!(parameter.offsets().collect::<Vec<_>>(), [("bmp", -1.0)], "at {now}");
        }
        // ON the period: the next look, one EMA step
        push(&parameter, "bmp", 103.0, PERIOD);
        push(&parameter, "icp", 100.0, PERIOD);
        let learned = -1.0 + OFFSET_ALPHA * (-3.0 - -1.0);
        assert_eq!(parameter.offsets().collect::<Vec<_>>(), [("bmp", learned)]);
        // the period runs from the last look, not from the last push
        push(&parameter, "bmp", 120.0, 2 * PERIOD - 1);
        push(&parameter, "icp", 100.0, 2 * PERIOD - 1);
        assert_eq!(parameter.offsets().collect::<Vec<_>>(), [("bmp", learned)]);
    }

    #[test]
    fn a_look_with_no_fresh_backup_still_starts_a_period() {
        // the period is the reader's cadence: plan B marked a reading learned with nothing fresh beside it
        let parameter = reconciling();
        push(&parameter, "icp", 100.0, 0); // a look, with no backup reading yet
        push(&parameter, "bmp", 104.0, 100_000);
        push(&parameter, "icp", 100.0, 100_000);
        assert_eq!(
            parameter.offsets().count(),
            0,
            "inside the period begun by the empty look"
        );
        push(&parameter, "bmp", 104.0, PERIOD);
        push(&parameter, "icp", 100.0, PERIOD);
        assert_eq!(parameter.offsets().collect::<Vec<_>>(), [("bmp", -4.0)]);
    }

    #[test]
    fn the_look_period_is_measured_across_the_low_word_wrap() {
        let parameter = reconciling();
        let before_wrap = (1_u64 << 32) - 100_000;
        push(&parameter, "bmp", 101.0, before_wrap);
        push(&parameter, "icp", 100.0, before_wrap);
        push(&parameter, "bmp", 103.0, before_wrap + PERIOD - 1);
        push(&parameter, "icp", 100.0, before_wrap + PERIOD - 1);
        assert_eq!(
            parameter.offsets().collect::<Vec<_>>(),
            [("bmp", -1.0)],
            "the wrap is no early look"
        );
        push(&parameter, "bmp", 103.0, before_wrap + PERIOD);
        push(&parameter, "icp", 100.0, before_wrap + PERIOD);
        assert_eq!(
            parameter.offsets().collect::<Vec<_>>(),
            [("bmp", -1.0 + OFFSET_ALPHA * (-3.0 - -1.0))]
        );
    }

    #[test]
    fn co_primaries_and_vectors_never_reconcile() {
        let mut parameter: Parameter<f32> = Parameter::new("agl");
        parameter.add_source("laser", 0, 100, true).unwrap();
        parameter.add_source("l1x", 0, 100, false).unwrap();
        push(&parameter, "l1x", 3.0, 0);
        push(&parameter, "laser", 2.0, 0);
        assert_eq!(parameter.offsets().count(), 0, "the same rank is not a backup");
        push(&parameter, "l1x", 3.5, 150_000);
        assert_eq!(parameter.read(150_000).unwrap().value, 3.5, "a co-primary reads raw");

        let mut vectors: Parameter<[f32; 3]> = Parameter::new("accel");
        vectors.add_source("lsm6dso32", 0, 20, true).unwrap();
        vectors.add_source("bno055", 1, 40, false).unwrap();
        push(&vectors, "bno055", [0.0, 0.0, 1.0625], 0);
        push(&vectors, "lsm6dso32", [0.0, 0.0, 1.0], 0);
        assert_eq!(vectors.offsets().count(), 0);
        push(&vectors, "bno055", [0.0, 0.0, 1.125], 30_000);
        assert_eq!(
            vectors.read(30_000).unwrap().value,
            [0.0, 0.0, 1.125],
            "raw: a vector has no bias"
        );
    }

    #[test]
    fn without_reconcile_nothing_is_learned_or_corrected() {
        let parameter = parameter(&[("icp", 0, 50), ("bmp", 1, 500)]);
        push(&parameter, "bmp", 102.0, 0);
        push(&parameter, "icp", 100.0, 0);
        assert_eq!(parameter.offsets().count(), 0);
        push(&parameter, "bmp", 102.0, 60_000);
        assert_eq!(parameter.read(60_000).unwrap().value, 102.0);
    }

    #[test]
    fn values_pack_and_unpack_exactly() {
        let position = [-481_234_567_i32, i32::MAX];
        assert_eq!(<[i32; 2]>::from_words(position.to_words()), position);
        let quaternion = [1.0_f32, -0.5, f32::MIN_POSITIVE, -0.0];
        assert_eq!(
            <[f32; 4]>::from_words(quaternion.to_words()).map(f32::to_bits),
            quaternion.map(f32::to_bits)
        );
        assert_eq!(f32::from_words((-3.25_f32).to_words()), -3.25);
        // FULL mantissas, every low bit set somewhere: the values above have none, so a packing that dropped the
        // low bits (half precision is about 50 Pa at sea level) would pass them
        let pressure = f32::from_bits(0x47C5_E6AB); // 101325.336 Pa
        assert_eq!(f32::from_words(pressure.to_words()).to_bits(), pressure.to_bits());
        let full = [
            core::f32::consts::PI,
            -core::f32::consts::E,
            f32::from_bits(0x3F7F_FFFF), // the largest float below 1
            f32::from_bits(0x0000_0001), // the smallest subnormal
        ];
        assert_eq!(
            <[f32; 4]>::from_words(full.to_words()).map(f32::to_bits),
            full.map(f32::to_bits)
        );
        let vector = [core::f32::consts::FRAC_1_SQRT_2, -0.123_456_79, 0.999_999];
        assert_eq!(
            <[f32; 3]>::from_words(vector.to_words()).map(f32::to_bits),
            vector.map(f32::to_bits)
        );
        assert_eq!([1.0_f32, 2.0, 3.0].difference([0.0; 3]), None, "a vector has no bias");
        assert_eq!(2.5_f32.difference(1.0), Some(1.5));
        assert_eq!(2.5_f32.shifted(-1.0), 1.5);
    }
}
