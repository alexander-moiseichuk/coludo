//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The spill queue (doc/specs/avionics-port.md, "Spill"). Frames are cut while the next block is not yet
//! erased, and they wait here in arrival order until GC frees the block. They are held unstamped: a frame gets
//! its block, its number and its header when it is programmed, because the BOOT frames of the blocks it lands
//! in must be numbered in between.
//!
//! One byte deque, allocated once at its capacity (2 MiB of PSRAM on the board), holds a 14-byte record head
//! per frame (kind, durable, payload length u32, uptime u64) and then the payload. The queue never allocates
//! after mount and never splits a frame.

use alloc::collections::VecDeque;

use crate::frame::Kind;

/// A record's head: kind u8, durable u8, payload length u32, uptime u64.
const RECORD_BYTES: usize = 14;

/// The front frame's head.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Spilled {
    /// DATA or CONFIRM. BOOT frames are the ring's own and never wait here.
    pub(crate) kind: Kind,
    /// The durable bit the producer asked for.
    pub(crate) durable: bool,
    /// The payload's length.
    pub(crate) payload_length: usize,
    /// The uptime of its first record.
    pub(crate) uptime_us: u64,
}

/// A bounded FIFO of frames waiting for an erased block.
#[derive(Debug)]
pub(crate) struct Spill {
    /// The records, back to back.
    bytes: VecDeque<u8>,
    /// The most bytes `bytes` may hold.
    capacity: usize,
    /// How many frames it holds.
    frames: u32,
}

impl Spill {
    /// An empty queue holding at most `capacity` bytes of records.
    pub(crate) fn new(capacity: usize) -> Spill {
        Spill {
            bytes: VecDeque::with_capacity(capacity),
            capacity,
            frames: 0,
        }
    }

    /// Whether no frame waits.
    pub(crate) fn is_empty(&self) -> bool {
        self.frames == 0
    }

    /// How many frames wait.
    pub(crate) fn frames(&self) -> u32 {
        self.frames
    }

    /// How many bytes the waiting records take.
    pub(crate) fn bytes(&self) -> usize {
        self.bytes.len()
    }

    /// Queues a frame behind the others. Returns false, queueing nothing, when it does not fit.
    pub(crate) fn push(&mut self, kind: Kind, durable: bool, uptime_us: u64, payload: &[u8]) -> bool {
        if self.bytes.len() + RECORD_BYTES + payload.len() > self.capacity {
            return false;
        }
        self.bytes.push_back(kind as u8);
        self.bytes.push_back(u8::from(durable));
        self.bytes.extend((payload.len() as u32).to_le_bytes());
        self.bytes.extend(uptime_us.to_le_bytes());
        self.bytes.extend(payload.iter().copied());
        self.frames += 1;
        true
    }

    /// The head of the oldest waiting frame, None when none waits.
    pub(crate) fn front(&self) -> Option<Spilled> {
        if self.is_empty() {
            return None;
        }
        let mut head = [0u8; RECORD_BYTES];
        self.copy(0, &mut head);
        Some(Spilled {
            kind: if head[0] == Kind::Confirm as u8 {
                Kind::Confirm
            } else {
                Kind::Data
            },
            durable: head[1] != 0,
            payload_length: u32::from_le_bytes(head[2..6].try_into().expect("4 bytes")) as usize,
            uptime_us: u64::from_le_bytes(head[6..14].try_into().expect("8 bytes")),
        })
    }

    /// The waiting frames' payload lengths, oldest first: where they will land decides the next frame's room.
    pub(crate) fn payload_lengths(&self) -> impl Iterator<Item = usize> + '_ {
        let mut at = 0;
        (0..self.frames).map(move |_| {
            let mut length = [0u8; 4];
            self.copy(at + 2, &mut length);
            let length = u32::from_le_bytes(length) as usize;
            at += RECORD_BYTES + length;
            length
        })
    }

    /// Copies the oldest frame's payload into `payload`, which is exactly its length.
    pub(crate) fn copy_front_payload(&self, payload: &mut [u8]) {
        self.copy(RECORD_BYTES, payload);
    }

    /// Drops the oldest frame, once it is programmed.
    pub(crate) fn pop_front(&mut self) {
        if let Some(front) = self.front() {
            self.bytes.drain(..RECORD_BYTES + front.payload_length);
            self.frames -= 1;
        }
    }

    /// Copies the bytes from logical index `start` into `out`, in at most two slices (the deque may wrap).
    fn copy(&self, start: usize, out: &mut [u8]) {
        let (first, second) = self.bytes.as_slices();
        let end = start + out.len();
        if end <= first.len() {
            out.copy_from_slice(&first[start..end]);
        } else if start >= first.len() {
            out.copy_from_slice(&second[start - first.len()..end - first.len()]);
        } else {
            let split = first.len() - start;
            out[..split].copy_from_slice(&first[start..]);
            out[split..].copy_from_slice(&second[..end - first.len()]);
        }
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    #[test]
    fn keeps_frames_in_order_across_the_deque_wrapping() {
        let mut spill = Spill::new(100);
        for round in 0..20u8 {
            assert!(spill.push(Kind::Data, round % 2 == 0, u64::from(round), &[round; 30]));
            assert!(spill.push(Kind::Confirm, false, 1000, &[round, round]));
            let front = spill.front().unwrap();
            assert_eq!(
                (front.kind, front.durable, front.payload_length),
                (Kind::Data, round % 2 == 0, 30)
            );
            assert_eq!(front.uptime_us, u64::from(round));
            let mut payload = vec![0; 30];
            spill.copy_front_payload(&mut payload);
            assert_eq!(payload, vec![round; 30]);
            spill.pop_front();
            assert_eq!(spill.front().unwrap().kind, Kind::Confirm);
            spill.pop_front();
            assert!(spill.is_empty());
        }
    }

    #[test]
    fn lists_the_waiting_payload_lengths_oldest_first() {
        let mut spill = Spill::new(150);
        assert_eq!(spill.payload_lengths().count(), 0, "none while empty");
        for (round, length) in [30usize, 0, 25, 17, 30, 4].into_iter().enumerate() {
            if spill.frames() == 3 {
                spill.pop_front(); // the deque wraps as frames come and go
            }
            assert!(spill.push(Kind::Data, false, round as u64, &vec![1; length]));
        }
        assert!(spill.payload_lengths().eq([17, 30, 4]));
    }

    #[test]
    fn refuses_a_frame_past_its_capacity() {
        let mut spill = Spill::new(64);
        assert!(spill.push(Kind::Data, false, 0, &[1; 40])); // 54 bytes
        assert!(!spill.push(Kind::Data, false, 0, &[2; 0]), "14 more do not fit in 64");
        assert_eq!((spill.frames(), spill.bytes()), (1, 54));
        spill.pop_front();
        assert!(spill.push(Kind::Data, false, 0, &[2; 50]), "exactly 64 fits");
        assert!(!Spill::new(13).push(Kind::Data, false, 0, &[]));
        spill.pop_front();
        spill.pop_front(); // nothing left: a no-op
        assert_eq!(spill.front(), None);
    }
}
