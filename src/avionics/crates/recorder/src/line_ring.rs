//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The uart_saver's queue (M19, doc/specs/avionics-port.md M19): the ring_saver pushes each decoded wire line, the
//! uart_saver's thread pops them and sends them out of a UART, and neither ever waits for the other. Recency first:
//! when a line does not fit, the producer drops the OLDEST lines still undelivered (the last seconds before an impact
//! matter most), counted; the queue never refuses the new one (one longer than the whole ring aside).
//!
//! A line is a u16 length (little-endian) and its bytes, laid around the buffer end as they fall. `head` and `tail`
//! are monotonic byte counters (u32, wrapping: the P4 has no 64-bit atomics); `head - tail` bytes are queued. The
//! producer alone writes `head` and the bytes; both sides move `tail`, each by a compare-and-swap past one whole line:
//! the consumer after copying a line out (a delivery), the producer to make room (a drop). A consumer whose swap fails
//! lost its line to a drop while it copied: the copy may be torn, and it is thrown away. The buffer is `AtomicU8`, so a
//! byte the producer overwrites while the consumer reads it is a defined, if stale, value.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

/// The length prefix's bytes.
const PREFIX: usize = 2;

/// A single-producer, single-consumer queue of whole lines that drops its oldest when full (see the module notes).
pub struct LineRing {
    /// The bytes.
    buffer: Box<[AtomicU8]>,
    /// Where the next line goes: a monotonic byte counter, the producer's alone.
    head: AtomicU32,
    /// The oldest line still queued: a monotonic byte counter, moved by either side's compare-and-swap.
    tail: AtomicU32,
    /// Lines pushed.
    pushed: AtomicU32,
    /// Lines dropped: the oldest, to make room, and any longer than the ring.
    dropped: AtomicU32,
}

impl LineRing {
    /// A ring of `capacity` bytes (at least a few lines; at most 2^31).
    pub fn new(capacity: usize) -> LineRing {
        LineRing {
            buffer: (0..capacity.min(1 << 31)).map(|_| AtomicU8::new(0)).collect(),
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            pushed: AtomicU32::new(0),
            dropped: AtomicU32::new(0),
        }
    }

    /// Queues `line` (the producer's call, never waiting): the oldest lines go to make room, each counted; a line
    /// longer than the ring (or than a u16) is dropped itself.
    pub fn push(&self, line: &[u8]) {
        let need = PREFIX + line.len();
        if need > self.buffer.len() || line.len() > usize::from(u16::MAX) {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let head = self.head.load(Ordering::Relaxed);
        loop {
            let tail = self.tail.load(Ordering::Acquire);
            if head.wrapping_sub(tail) as usize + need <= self.buffer.len() {
                break;
            }
            // the oldest line goes; a failed swap means the consumer took it meanwhile: look again
            let length = self.length(tail);
            if self
                .tail
                .compare_exchange(
                    tail,
                    tail.wrapping_add((PREFIX + length) as u32),
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
        let length = (line.len() as u16).to_le_bytes();
        self.store(head, &length);
        self.store(head.wrapping_add(PREFIX as u32), line);
        self.head.store(head.wrapping_add(need as u32), Ordering::Release);
        self.pushed.fetch_add(1, Ordering::Relaxed);
    }

    /// Takes the oldest line into `out` (cleared first; the consumer's call, never waiting); false when none is queued.
    pub fn pop(&self, out: &mut Vec<u8>) -> bool {
        loop {
            let tail = self.tail.load(Ordering::Acquire);
            let head = self.head.load(Ordering::Acquire);
            if tail == head {
                return false;
            }
            let length = self.length(tail);
            out.clear();
            // a length past what is queued is a prefix the producer overwrote: the swap below fails, so look again
            if PREFIX + length <= head.wrapping_sub(tail) as usize {
                let start = tail.wrapping_add(PREFIX as u32);
                out.extend((0..length).map(|index| self.byte(start.wrapping_add(index as u32))));
            }
            if self
                .tail
                .compare_exchange(
                    tail,
                    tail.wrapping_add((PREFIX + length) as u32),
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                return true;
            }
        }
    }

    /// Lines pushed so far.
    pub fn pushed(&self) -> u32 {
        self.pushed.load(Ordering::Relaxed)
    }

    /// Lines dropped so far: the oldest, to make room, and any too long for the ring.
    pub fn dropped(&self) -> u32 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Bytes queued now.
    pub fn queued(&self) -> usize {
        self.head
            .load(Ordering::Acquire)
            .wrapping_sub(self.tail.load(Ordering::Acquire)) as usize
    }

    /// The byte at counter `position`.
    fn byte(&self, position: u32) -> u8 {
        self.buffer[position as usize % self.buffer.len()].load(Ordering::Relaxed)
    }

    /// The length prefix at counter `position`.
    fn length(&self, position: u32) -> usize {
        usize::from(u16::from_le_bytes([
            self.byte(position),
            self.byte(position.wrapping_add(1)),
        ]))
    }

    /// `bytes` from counter `position` on, around the buffer's end.
    fn store(&self, position: u32, bytes: &[u8]) {
        for (index, &byte) in bytes.iter().enumerate() {
            self.buffer[position.wrapping_add(index as u32) as usize % self.buffer.len()]
                .store(byte, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::LineRing;

    #[test]
    fn lines_come_out_whole_and_in_order() {
        let ring = LineRing::new(64);
        let mut out = Vec::new();
        assert!(!ring.pop(&mut out), "empty");
        ring.push(b"one\n");
        ring.push(b"two two\n");
        assert!(ring.pop(&mut out));
        assert_eq!(out, b"one\n");
        assert!(ring.pop(&mut out));
        assert_eq!(out, b"two two\n");
        assert!(!ring.pop(&mut out));
        assert_eq!((ring.pushed(), ring.dropped(), ring.queued()), (2, 0, 0));
    }

    #[test]
    fn a_full_ring_drops_its_oldest_lines_never_the_new_one() {
        let ring = LineRing::new(32);
        for index in 0..10u8 {
            ring.push(&[b'a' + index; 8]); // 10 bytes a line: 3 fit
        }
        let mut out = Vec::new();
        let mut kept = Vec::new();
        while ring.pop(&mut out) {
            kept.push(out[0]);
        }
        assert_eq!(kept, [b'h', b'i', b'j'], "the newest three");
        assert_eq!((ring.pushed(), ring.dropped()), (10, 7));
        // NEGATIVE: a line longer than the ring is dropped itself, the queue untouched
        ring.push(b"x");
        ring.push(&[0; 40]);
        assert!(ring.pop(&mut out));
        assert_eq!(out, b"x");
        assert_eq!(ring.dropped(), 8);
    }

    #[test]
    fn lines_wrap_around_the_buffer_end_intact() {
        let ring = LineRing::new(23);
        let mut out = Vec::new();
        for index in 0..200u32 {
            let line = format!("{index}:{}\n", "z".repeat(index as usize % 7));
            ring.push(line.as_bytes());
            assert!(ring.pop(&mut out));
            assert_eq!(out, line.as_bytes(), "line {index}");
        }
    }

    #[test]
    fn a_racing_producer_and_consumer_deliver_only_whole_lines_in_order() {
        let ring = std::sync::Arc::new(LineRing::new(512));
        let producer = {
            let ring = ring.clone();
            std::thread::spawn(move || {
                for index in 0..200_000u32 {
                    // a line that proves itself: its index, then a body the index decides
                    let body = (index % 23) as usize + 1;
                    let mut line = index.to_le_bytes().to_vec();
                    line.extend(std::iter::repeat_n((index % 251) as u8, body));
                    ring.push(&line);
                }
            })
        };
        let mut out = Vec::new();
        let mut delivered = 0u32;
        let mut last: Option<u32> = None;
        while !producer.is_finished() || ring.queued() > 0 {
            if !ring.pop(&mut out) {
                std::thread::yield_now();
                continue;
            }
            let index = u32::from_le_bytes(out[..4].try_into().expect("an index"));
            assert_eq!(out.len(), 4 + (index % 23) as usize + 1, "line {index} whole");
            assert!(
                out[4..].iter().all(|&byte| byte == (index % 251) as u8),
                "line {index} not torn"
            );
            assert!(last.is_none_or(|last| index > last), "in order: {index} after {last:?}");
            last = Some(index);
            delivered += 1;
        }
        producer.join().expect("the producer");
        assert_eq!(
            delivered + ring.dropped(),
            200_000,
            "every line delivered or counted dropped"
        );
    }
}
