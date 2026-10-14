//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The writer and the eraser on two threads, as on the board, over one flash that a mutex serialises the way the
//! IDF flash driver does. [`MemoryFlash`] is single-threaded, so the other tests drive both tasks on one thread;
//! these run the block table's Release/Acquire protocol for real (the writer takes ERASED slots to OPEN or
//! FOREIGN and its OPEN slot to VALID, the eraser everything else to ERASED), with and without a moving saver:
//! * every chunk the writer programmed is in flash at the end, byte for byte, unless the eraser reported erasing
//!   its block, and nothing else is;
//! * the eraser never erases the writer's open block nor an erased slot: each erase is checked against the table
//!   under the flash lock, where only the eraser can have changed the slot since it chose it;
//! * no program meets a programmed byte: [`MemoryFlash`] refuses a 0 -> 1 change, and the writer would report it.

mod common;

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use common::*;
use flash_ring::{
    BLOCK_BYTES, BlockTable, Eraser, Erasure, Error, Flash, Geometry, Hints, MemoryError, MemoryFlash, Position, Ring,
    Slot,
};

/// Chunks the writer appends per run: enough to wrap the ring many times.
const CHUNKS: u32 = 1500;

/// Runs per variant: each one interleaves the two threads differently.
const RUNS: u32 = 6;

/// The logical milliseconds one eraser step takes: the saver's delay elapses after 20 steps, whatever the host's
/// speed, so a run never waits on the wall clock.
const STEP_MS: u32 = 5;

/// The saver position meaning "no saver" in [`Saver`].
const NO_SAVER: u32 = u32::MAX;

/// One flash shared by the two threads, one call at a time.
struct Shared {
    /// The flash.
    flash: Mutex<MemoryFlash>,
}

/// The writer's handle: reads and programs, never an erase.
struct WriterHandle<'a> {
    /// The shared flash.
    shared: &'a Shared,
}

impl Flash for WriterHandle<'_> {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        self.shared.flash.lock().unwrap().read(offset, buffer)
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), MemoryError> {
        let flash = self.shared.flash.lock().unwrap();
        let mut handle: &MemoryFlash = &flash;
        handle.program(offset, data)
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        panic!("the writer erased offset {offset}: only the eraser erases");
    }

    fn size(&self) -> u32 {
        self.shared.flash.lock().unwrap().size()
    }
}

/// The eraser's handle: reads and erases, never a program, and every erase checked against the table.
struct EraserHandle<'a> {
    /// The shared flash.
    shared: &'a Shared,
    /// The table the two threads share.
    table: &'a BlockTable,
}

impl Flash for EraserHandle<'_> {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        self.shared.flash.lock().unwrap().read(offset, buffer)
    }

    fn program(&mut self, offset: u32, _data: &[u8]) -> Result<(), MemoryError> {
        panic!("the eraser programmed offset {offset}: only the writer programs");
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        let flash = self.shared.flash.lock().unwrap();
        let slot = offset / BLOCK_BYTES;
        let state = self.table.slot(slot);
        // only the eraser moves a slot out of these, so it is still what the step chose
        assert!(
            matches!(state, Slot::Valid { .. } | Slot::OtherVersion { .. } | Slot::Foreign),
            "erasing slot {slot} while it is {state:?}"
        );
        let mut handle: &MemoryFlash = &flash;
        handle.erase_sector(offset)
    }

    fn size(&self) -> u32 {
        self.shared.flash.lock().unwrap().size()
    }
}

/// The saver's published block, as the firmware publishes `saver_position` (here one AtomicU32: the test's saver
/// passes whole blocks, so the offset is always 0).
struct Saver {
    /// Every block before this one is saved; [`NO_SAVER`] while there is no saver.
    block_sequence: AtomicU32,
}

impl Saver {
    /// The position the eraser reads.
    fn position(&self) -> Option<Position> {
        let block_sequence = self.block_sequence.load(Ordering::Acquire);
        (block_sequence != NO_SAVER).then_some(Position {
            block_sequence,
            offset: 0,
        })
    }
}

/// One run: the writer appends [`CHUNKS`] chunks on one thread while the eraser steps on another, the saver (if
/// `saving`) a block behind the writer. Then every chunk is looked for in the image.
fn run(geometry: Geometry, saving: bool, round: u32) {
    let context = format!("saving {saving}, round {round}");
    let shared = Shared {
        flash: Mutex::new(erased(geometry)),
    };
    let scanned = {
        let flash = shared.flash.lock().unwrap();
        bring_up(&flash, geometry, Hints::default())
    };
    let table = &scanned.table;
    let saver = Saver {
        block_sequence: AtomicU32::new(NO_SAVER),
    };
    let done = AtomicBool::new(false);
    let (appended, erased_blocks) = std::thread::scope(|scope| {
        let writer = scope.spawn(|| {
            let handle = WriterHandle { shared: &shared };
            let mut ring = Ring::new(handle, table, scanned.next_sequence, 1, session(1), false).unwrap();
            let mut appended = Vec::new();
            for index in 0..CHUNKS {
                // the pad, a flight that wraps the ring, then DONE: the first block after it is the only non-flight
                // one, and open, which newest wins must pass over for the oldest flight block
                ring.set_flight((CHUNKS / 3..2 * CHUNKS / 3).contains(&index));
                let at = loop {
                    match ring.append(&payload(1, index), records(index)) {
                        Ok(at) => break at,
                        Err(Error::WouldBlock) => std::thread::yield_now(),
                        Err(other) => panic!("chunk {index}: {other:?}"),
                    }
                };
                appended.push((index, at));
                if saving {
                    let behind = ring.position().block_sequence.saturating_sub(1);
                    saver.block_sequence.store(behind, Ordering::Release);
                }
            }
            done.store(true, Ordering::Release);
            appended
        });
        let eraser = scope.spawn(|| {
            let handle = EraserHandle { shared: &shared, table };
            let mut eraser = Eraser::new(handle, table, None).unwrap();
            let mut erased_blocks = BTreeSet::new();
            let mut now_ms = 0u32;
            while !done.load(Ordering::Acquire) {
                now_ms = now_ms.wrapping_add(STEP_MS);
                match eraser.erase_step(saver.position(), now_ms).unwrap() {
                    Erasure::Saved { block_sequence, .. } => assert!(erased_blocks.insert(block_sequence)),
                    Erasure::Lost(loss) => assert!(erased_blocks.insert(loss.block_sequence)),
                    Erasure::Foreign { slot } => panic!("slot {slot}: a fresh ring has no foreign slot"),
                    Erasure::Idle => std::thread::yield_now(),
                }
            }
            erased_blocks
        });
        (writer.join().unwrap(), eraser.join().unwrap())
    });
    assert!(
        erased_blocks.len() as u32 > 2 * geometry.blocks(),
        "{context}: the ring wrapped: {} erased",
        erased_blocks.len()
    );
    let lost = table.figures(None).lost;
    if saving {
        assert_eq!(lost, 0, "{context}: a saver a block behind keeps the ring free");
    } else {
        assert_eq!(
            lost as usize,
            erased_blocks.len(),
            "{context}: with no saver, every erase is a loss"
        );
    }

    let image = shared.flash.into_inner().unwrap().into_image();
    let found: BTreeMap<Position, Vec<u8>> = walk(&image, geometry)
        .into_iter()
        .map(|found| {
            let at = Position {
                block_sequence: found.header.block_sequence,
                offset: found.offset,
            };
            (at, found.payload)
        })
        .collect();
    let kept: BTreeMap<Position, u32> = appended
        .iter()
        .filter(|(_, at)| !erased_blocks.contains(&at.block_sequence))
        .map(|&(index, at)| (at, index))
        .collect();
    for (at, index) in &kept {
        assert_eq!(
            found.get(at),
            Some(&payload(1, *index)),
            "{context}: chunk {index} at {at:?}"
        );
    }
    assert_eq!(
        found.len(),
        kept.len(),
        "{context}: nothing in flash the writer did not program"
    );
    for block in blocks(&image, geometry) {
        assert!(
            block.torn.is_empty(),
            "{context}: block {}: {:?}",
            block.header.block_sequence,
            block.torn
        );
    }
}

#[test]
fn the_writer_and_the_eraser_on_two_threads_with_no_saver() {
    for round in 0..RUNS {
        run(Geometry::new(16).unwrap(), false, round);
    }
}

#[test]
fn the_writer_and_the_eraser_on_two_threads_behind_a_moving_saver() {
    for round in 0..RUNS {
        run(Geometry::new(16).unwrap(), true, round);
    }
}
