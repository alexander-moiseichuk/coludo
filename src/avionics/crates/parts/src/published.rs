//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! [`Published`]: a part's state, written by its owner and read whole by the executor (a seqlock). The owner never
//! waits: a write is two counter stores around a copy. A reader retries while a write is under way, so it gets one
//! write's state, never a mix of two. Plan B read its tasks' fields directly: one interpreter, no torn reads.
//!
//! One writer at a time: the part's owner on its thread, or main before the owners start (bring-up, the restores).
//! RV32 has no 64-bit atomics, so the counter is a u32; it would have to wrap twice during one read to fool a
//! reader, and a read takes microseconds.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, Ordering, fence};

/// A `T` one writer publishes and any thread reads whole.
pub struct Published<T: Copy> {
    /// Even: the value is whole; odd: a write is under way. Each write adds two.
    sequence: AtomicU32,
    /// The latest write.
    value: UnsafeCell<T>,
}

// SAFETY: the value is copied in and out under the sequence counter, and a reader discards any copy a write
// overlapped; T is Copy, so a discarded copy owns nothing.
unsafe impl<T: Copy + Send> Sync for Published<T> {}

impl<T: Copy> Published<T> {
    /// A slot holding `value`.
    pub const fn new(value: T) -> Published<T> {
        Published {
            sequence: AtomicU32::new(0),
            value: UnsafeCell::new(value),
        }
    }

    /// Replaces the value. One writer at a time (see the module notes).
    pub fn publish(&self, value: T) {
        let sequence = self.sequence.load(Ordering::Relaxed);
        self.sequence.store(sequence.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        // SAFETY: the only writer; a reader overlapping this copy sees the odd counter or a changed one and retries.
        unsafe { core::ptr::write_volatile(self.value.get(), value) };
        self.sequence.store(sequence.wrapping_add(2), Ordering::Release);
    }

    /// The latest value, whole.
    pub fn read(&self) -> T {
        loop {
            let before = self.sequence.load(Ordering::Acquire);
            if before & 1 == 1 {
                core::hint::spin_loop();
                continue;
            }
            // SAFETY: a copy of plain data; kept only when no write overlapped it (the counter unchanged and even).
            let value = unsafe { core::ptr::read_volatile(self.value.get()) };
            fence(Ordering::Acquire);
            if self.sequence.load(Ordering::Relaxed) == before {
                return value;
            }
        }
    }
}

impl<T: Copy + Default> Default for Published<T> {
    fn default() -> Published<T> {
        Published::new(T::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::atomic::AtomicBool;

    #[test]
    fn a_read_returns_the_latest_write() {
        let slot = Published::new([0u32; 4]);
        assert_eq!(slot.read(), [0; 4]);
        slot.publish([1, 2, 3, 4]);
        assert_eq!(slot.read(), [1, 2, 3, 4]);
        slot.publish([5, 6, 7, 8]);
        assert_eq!(slot.read(), [5, 6, 7, 8]);
    }

    #[test]
    fn the_counter_wraps_without_a_torn_read() {
        let slot = Published::new(7u64);
        slot.sequence.store(u32::MAX - 1, Ordering::Relaxed);
        slot.publish(9);
        assert_eq!(slot.sequence.load(Ordering::Relaxed), 0);
        assert_eq!(slot.read(), 9);
    }

    #[test]
    fn readers_never_see_two_writes_mixed() {
        // every write is four equal words: a read mixing two writes has unequal ones
        let slot = Arc::new(Published::new([0u32; 16]));
        let stop = Arc::new(AtomicBool::new(false));
        let readers: Vec<_> = (0..3)
            .map(|_| {
                let (slot, stop) = (Arc::clone(&slot), Arc::clone(&stop));
                std::thread::spawn(move || {
                    let mut reads = 0u32;
                    while !stop.load(Ordering::Relaxed) {
                        let value = slot.read();
                        assert!(value.iter().all(|word| *word == value[0]), "torn: {value:?}");
                        reads += 1;
                    }
                    reads
                })
            })
            .collect();
        for round in 1..200_000u32 {
            slot.publish([round; 16]);
        }
        stop.store(true, Ordering::Relaxed);
        for reader in readers {
            assert!(reader.join().expect("a reader panicked") > 0);
        }
    }
}
