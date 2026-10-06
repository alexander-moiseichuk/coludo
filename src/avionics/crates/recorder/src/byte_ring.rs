//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The single-producer, single-consumer byte ring under every recorder queue (doc/specs/avionics-port.md
//! "Producer side"): a producing thread's records on their way to the flash writer, and the writer's lines on
//! their way to a CC tee.
//!
//! A record is a u16 length (the record's whole size, a multiple of 4) followed by its bytes, padded with zeros
//! to that size. A length of 0 is the wrap marker: the record that did not fit before the buffer's end starts
//! at offset 0. The producer owns `head` and the consumer owns `tail`, both AtomicUsize: the producer publishes
//! a record with a Release store of `head`, and the consumer frees its bytes with a Release store of `tail`,
//! each read by the other side with Acquire. Neither side ever writes the other's index, so no lock is taken
//! and no 64-bit atomic is needed (the P4 has none).
//!
//! `head == tail` means empty, so a record may never make `head` reach `tail`: a full ring refuses the record
//! and keeps everything queued. The free space can be split in two by the buffer's end, and a record takes one
//! piece whole, so a ring takes records much smaller than itself (the writer's are at most ~1.3 KiB, the
//! smallest ring 128 KiB).
//!
//! The buffer is the caller's, `&'static mut [u8]`: the firmware hands over PSRAM it allocated with
//! `heap_caps_malloc` (and never frees), the host tests a leaked Vec. Owning it for good is what makes the raw
//! accesses below sound: no other reference to those bytes can exist.

use core::ptr::NonNull;
use core::sync::atomic::{AtomicUsize, Ordering};

/// Records start on this alignment, and their sizes are multiples of it.
pub(crate) const ALIGNMENT: usize = 4;

/// The record's length field, its first bytes. The ring writes it; a length of 0 is the wrap marker.
pub(crate) const LENGTH_BYTES: usize = 2;

/// The largest record: the largest multiple of [`ALIGNMENT`] the u16 length holds.
pub(crate) const MAX_RECORD_BYTES: usize = u16::MAX as usize & !(ALIGNMENT - 1);

/// `length` rounded up to the alignment: the bytes a record of `length` takes.
pub(crate) const fn padded(length: usize) -> usize {
    (length + ALIGNMENT - 1) & !(ALIGNMENT - 1)
}

/// A byte ring over a buffer it owns for good. See the module notes for the protocol.
pub(crate) struct ByteRing {
    /// The buffer's first byte.
    start: NonNull<u8>,
    /// The buffer's usable length: its length rounded down to the alignment.
    capacity: usize,
    /// Producer-owned: where the next record goes. Always below `capacity`.
    head: AtomicUsize,
    /// Consumer-owned: the first byte not yet freed. Always below `capacity`.
    tail: AtomicUsize,
}

// SAFETY: the ring owns its buffer (moved in as `&'static mut`), and its bytes are only reached through the
// methods below, whose contracts keep the producer's writes and the consumer's reads on disjoint bytes, ordered
// by the Release/Acquire pairs on `head` and `tail`.
unsafe impl Send for ByteRing {}
// SAFETY: as for Send; the unsafe methods name which one thread may call them at a time.
unsafe impl Sync for ByteRing {}

impl ByteRing {
    /// A ring over `buffer`, empty. A length that is not a multiple of the alignment loses its last bytes.
    pub(crate) fn new(buffer: &'static mut [u8]) -> ByteRing {
        let capacity = buffer.len() & !(ALIGNMENT - 1);
        ByteRing {
            start: NonNull::from(buffer).cast(),
            capacity,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    /// Appends one record of `length` bytes and publishes it to the consumer. `fill` writes the record's bytes
    /// after its length field (which the ring writes itself, after `fill`); the padding is already zero.
    ///
    /// Returns false, writing nothing, when the record does not fit the free space, or is larger than
    /// [`MAX_RECORD_BYTES`].
    ///
    /// # Safety
    /// One thread at a time is the ring's producer: no other `push` on this ring may run concurrently. A thread
    /// that takes the producer role over from another must be ordered after it (a lock, a claimed flag).
    pub(crate) unsafe fn push(&self, length: usize, fill: impl FnOnce(&mut [u8])) -> bool {
        let length = length.max(LENGTH_BYTES);
        let size = padded(length);
        if size > MAX_RECORD_BYTES {
            return false;
        }
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);
        let at = if head >= tail {
            // free: [head, capacity) and [0, tail); a record ending at the buffer's end wraps head to 0, which must
            // not meet tail
            if head + size < self.capacity || (head + size == self.capacity && tail != 0) {
                head
            } else if size < tail {
                // SAFETY: [head, capacity) is free and at least ALIGNMENT long (head < capacity, both aligned).
                unsafe { self.bytes_mut(head, LENGTH_BYTES) }.fill(0);
                0
            } else {
                return false;
            }
        } else if head + size < tail {
            head
        } else {
            return false;
        };
        // SAFETY: [at, at + size) is free space: the producer alone writes it, and the consumer reads nothing
        // there until the head store below publishes it.
        let record = unsafe { self.bytes_mut(at, size) };
        record[length..].fill(0);
        fill(&mut record[..length]);
        record[..LENGTH_BYTES].copy_from_slice(&(size as u16).to_le_bytes());
        self.head.store(self.after(at, size), Ordering::Release);
        true
    }

    /// The record at `read`, following a wrap marker there: its start, and its bytes (padded, length field
    /// included). None when nothing is queued at `read`.
    ///
    /// # Safety
    /// Only the ring's one consumer calls it. `read` is the ring's tail, or the start of a record this consumer
    /// read, or the [`ByteRing::after`] of one, and no [`ByteRing::release`] has passed it. The bytes must not
    /// be used after a release passes them: the producer may then overwrite them.
    pub(crate) unsafe fn record(&self, read: usize) -> Option<(usize, &[u8])> {
        let head = self.head.load(Ordering::Acquire);
        if read == head {
            return None;
        }
        // SAFETY: `read` is a published record boundary (the contract), so its length field is written.
        let mut at = read;
        let mut size = unsafe { self.length(at) };
        if size == 0 {
            at = 0;
            if at == head {
                return None;
            }
            // SAFETY: the producer writes the wrap marker and then the record at 0 before publishing either.
            size = unsafe { self.length(at) };
        }
        if size < LENGTH_BYTES || at + size > self.capacity {
            return None; // not a record this ring wrote: unreachable while the contract holds
        }
        // SAFETY: [at, at + size) is a published record, which the producer will not touch until it is released.
        Some((at, unsafe {
            core::slice::from_raw_parts(self.start.as_ptr().add(at), size)
        }))
    }

    /// The position after a record of `size` bytes at `start`: the next record's (or wrap marker's) start.
    pub(crate) fn after(&self, start: usize, size: usize) -> usize {
        let next = start + size;
        if next >= self.capacity { 0 } else { next }
    }

    /// The bytes the ring holds at most, for `inspect`.
    pub(crate) fn capacity(&self) -> usize {
        self.capacity
    }

    /// The consumer's freed-up-to position.
    pub(crate) fn tail(&self) -> usize {
        self.tail.load(Ordering::Relaxed)
    }

    /// Frees everything before `tail` for the producer to reuse.
    ///
    /// # Safety
    /// Only the ring's one consumer calls it, with a record boundary between the current tail and the head (as
    /// [`ByteRing::record`] and [`ByteRing::after`] give them), and holds no record bytes before it afterwards.
    pub(crate) unsafe fn release(&self, tail: usize) {
        self.tail.store(tail, Ordering::Release);
    }

    /// The u16 length at `at`.
    ///
    /// # Safety
    /// `at` is a published record boundary inside the buffer.
    unsafe fn length(&self, at: usize) -> usize {
        // SAFETY: the caller's contract; a boundary is aligned, so both bytes lie inside the buffer.
        let bytes = unsafe { core::slice::from_raw_parts(self.start.as_ptr().add(at), LENGTH_BYTES) };
        usize::from(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    /// `length` bytes from `at`, writable.
    ///
    /// # Safety
    /// The range lies inside the buffer, and is free space no reader can reach while the slice lives.
    #[allow(clippy::mut_from_ref)] // the ring hands out disjoint free space; the contract above is the proof
    unsafe fn bytes_mut(&self, at: usize, length: usize) -> &mut [u8] {
        // SAFETY: the caller's contract.
        unsafe { core::slice::from_raw_parts_mut(self.start.as_ptr().add(at), length) }
    }
}

#[cfg(test)]
mod tests {
    use super::{ALIGNMENT, ByteRing, LENGTH_BYTES, MAX_RECORD_BYTES, padded};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::vec::Vec;

    /// A ring over a leaked buffer of `capacity` bytes.
    fn ring(capacity: usize) -> ByteRing {
        ByteRing::new(Vec::leak(vec![0xAA; capacity]))
    }

    /// Push a record carrying `content` after its length field (the test is the ring's one producer).
    fn push(ring: &ByteRing, content: &[u8]) -> bool {
        // SAFETY: each test thread that pushes is its ring's only producer.
        unsafe {
            ring.push(LENGTH_BYTES + content.len(), |record| {
                record[LENGTH_BYTES..].copy_from_slice(content)
            })
        }
    }

    /// Pop the oldest record's content (padding included), freeing it; None when empty.
    fn pop(ring: &ByteRing) -> Option<Vec<u8>> {
        // SAFETY: each test thread that pops is its ring's only consumer, reading at the tail.
        let (start, bytes) = unsafe { ring.record(ring.tail()) }?;
        let content = bytes[LENGTH_BYTES..].to_vec();
        // SAFETY: as above; the content was copied out first.
        unsafe { ring.release(ring.after(start, bytes.len())) };
        Some(content)
    }

    #[test]
    fn records_come_back_in_order_padded_with_zeros() {
        let ring = ring(64);
        assert!(push(&ring, b"one"));
        assert!(push(&ring, b"second"));
        assert_eq!(pop(&ring).unwrap(), b"one\0\0\0", "2 + 3 bytes take 8");
        assert_eq!(pop(&ring).unwrap(), b"second");
        assert_eq!(pop(&ring), None);
    }

    #[test]
    fn a_full_ring_refuses_and_keeps_what_it_holds() {
        let ring = ring(32);
        assert!(push(&ring, &[1; 10])); // 12 bytes
        assert!(push(&ring, &[2; 10])); // 24
        assert!(!push(&ring, &[3; 6]), "8 more would make head meet tail");
        assert!(push(&ring, &[3; 2]), "4 more leave one alignment step free");
        assert!(!push(&ring, &[4; 2]), "and nothing fits now");
        assert_eq!(pop(&ring).unwrap(), [1; 10]);
        assert_eq!(pop(&ring).unwrap(), [2; 10]);
        assert_eq!(pop(&ring).unwrap(), [3; 2]);
        assert_eq!(pop(&ring), None);
    }

    #[test]
    fn a_record_larger_than_a_u16_length_is_refused() {
        let ring = ring(1 << 17);
        assert!(
            !push(&ring, &vec![0; MAX_RECORD_BYTES]),
            "with its length field it is past the limit"
        );
        assert!(push(&ring, &vec![0; MAX_RECORD_BYTES - LENGTH_BYTES]));
        assert_eq!(padded(MAX_RECORD_BYTES), MAX_RECORD_BYTES);
    }

    #[test]
    fn an_empty_or_tiny_buffer_takes_nothing() {
        for capacity in [0, 3, 4] {
            let ring = ring(capacity);
            assert!(!push(&ring, b""), "capacity {capacity}");
            assert_eq!(pop(&ring), None);
        }
        assert_eq!(ring(7).capacity, 4, "rounded down to the alignment");
    }

    #[test]
    fn records_wrap_at_every_offset() {
        const CAPACITY: usize = 64;
        for offset in (0..CAPACITY).step_by(ALIGNMENT) {
            for content in 2..=26 {
                let ring = ring(CAPACITY);
                // move both indexes to `offset` through filler records
                let mut moved = 0;
                while moved < offset {
                    let filler = (offset - moved).min(16);
                    assert!(push(&ring, &vec![0xEE; filler - LENGTH_BYTES]));
                    assert!(pop(&ring).is_some());
                    moved += filler;
                }
                assert_eq!(ring.tail(), offset % CAPACITY);
                let mut expected = Vec::new();
                for index in 0..6u8 {
                    let record: Vec<u8> = (0..content as u8).map(|byte| byte ^ index ^ 0x5A).collect();
                    if push(&ring, &record) {
                        expected.push(record);
                    } else {
                        break;
                    }
                }
                assert!(!expected.is_empty(), "offset {offset}, {content} bytes: nothing fit");
                for record in expected {
                    let back = pop(&ring).expect("queued");
                    assert_eq!(&back[..record.len()], record, "offset {offset}, {content} bytes");
                    assert!(back[record.len()..].iter().all(|&byte| byte == 0), "zero padding");
                }
                assert_eq!(pop(&ring), None);
            }
        }
    }

    #[test]
    fn a_wrap_marker_is_written_only_when_the_record_then_fits() {
        let ring = ring(32);
        assert!(push(&ring, &[1; 14])); // 16 bytes at 0
        assert!(pop(&ring).is_some()); // head = tail = 16
        assert!(
            !push(&ring, &[2; 18]),
            "20 bytes fit neither [16, 32) nor before the tail"
        );
        assert!(push(&ring, &[3; 10]), "12 bytes at 16, nothing was disturbed");
        assert_eq!(pop(&ring).unwrap(), [3; 10]);
        assert!(
            push(&ring, &[4; 10]),
            "12 bytes: [28, 32) is too short, so it wraps to 0"
        );
        assert_eq!(pop(&ring).unwrap(), [4; 10]);
    }

    /// A deterministic record for (index): its length and bytes come from the index alone.
    fn stress_record(index: u32) -> Vec<u8> {
        let length = 4 + (index.wrapping_mul(2_654_435_761) >> 24) as usize % 120;
        (0..length)
            .map(|offset| (index as usize + offset * 7) as u8)
            .collect::<Vec<u8>>()
    }

    #[test]
    fn a_producer_and_a_consumer_thread_lose_and_reorder_nothing() {
        const RECORDS: u32 = 200_000;
        let ring = ring(4096);
        let done = AtomicBool::new(false);
        std::thread::scope(|scope| {
            scope.spawn(|| {
                let mut index = 0;
                while index < RECORDS {
                    let mut record = index.to_le_bytes().to_vec();
                    record.extend(stress_record(index));
                    if push(&ring, &record) {
                        index += 1;
                    } else {
                        std::thread::yield_now();
                    }
                }
                done.store(true, Ordering::Release);
            });
            let mut expected = 0;
            loop {
                // read before the pop: once the producer is done, an empty ring means everything came through
                let finished = done.load(Ordering::Acquire);
                match pop(&ring) {
                    Some(content) => {
                        let index = u32::from_le_bytes(content[..4].try_into().unwrap());
                        assert_eq!(index, expected, "in order, none lost");
                        let record = stress_record(index);
                        assert_eq!(&content[4..4 + record.len()], record, "record {index} intact");
                        expected += 1;
                    }
                    None if finished => break,
                    None => std::thread::yield_now(),
                }
            }
            assert_eq!(expected, RECORDS);
        });
    }
}
