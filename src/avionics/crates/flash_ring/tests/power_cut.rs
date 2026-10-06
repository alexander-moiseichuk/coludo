//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Power cuts at every step, in two workloads:
//! * boot 2, over a ring boot 1 left, wraps the five slots, spills while the next block is unconfirmed, drains,
//!   and erases behind a lagging mover;
//! * boot 1 formats a ring of foreign bytes: it spills until slot 0 is erased, then writes its first blocks while
//!   GC formats ahead of it.
//!
//! Each workload is run once whole. Then for each of its flash operations it is run again with the power cut
//! there, the operation torn at several lengths. A torn program leaves only a prefix of the frame; a torn erase
//! leaves some sectors erased, or a sector's prefix. The oracle is the test's own mover, never the ring's idea of
//! it. After every cut:
//! * the cut changed one slot at most, and erased only a slot GC was allowed to erase: a foreign one, or a block
//!   wholly before the cursor the test's mover committed;
//! * every frame valid before the cut is still there, byte for byte, unless its slot was being erased;
//! * the blocks from the committed cursor on hold every DATA frame programmed into them, in order and numbered
//!   without a gap, plus at most the one frame the cut caught;
//! * a reboot scans correctly, opens the slot after the newest block, and never appends to an old block.

mod common;

use std::collections::HashMap;

use common::*;
use flash_ring::frame::{Confirm, Kind, frame_bytes};
use flash_ring::scan::Slot;
use flash_ring::{Erasure, Error, Geometry, MemoryError, MemoryFlash, Position, Ring};

/// The DATA frames boot 2 appends: enough to fill five blocks around the five slots, spill once the ring is
/// full of unconfirmed data, and drain behind GC.
const FRAMES: u32 = 220;

/// The format workload stops once its writer programs into this block. Slot 4 still lies ahead, so a reboot
/// after any cut opens a block without waiting for a confirmation.
const FORMAT_BLOCK: u32 = 3;

/// The tear lengths tried at each operation: inside the header, the header short of its last byte, the whole
/// header, one byte more, inside the payload (or a sector's head), and a call that completed but never
/// returned. Nothing torn is the image before the cut each is compared against.
const TEARS: [usize; 6] = [1, 39, 40, 41, 100, 5000];

/// What a boot accepted, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Entry {
    /// A DATA frame by its index.
    Data(u32),
    /// A CONFIRM frame.
    Confirm,
}

/// A workload's boot, as far as it got.
struct Run {
    /// Its ring, dead when the power was cut.
    ring: Ring<MemoryFlash>,
    /// The frames it accepted, programmed or spilled.
    accepted: Vec<Entry>,
    /// The DATA frame whose append the cut interrupted.
    interrupted: Option<u32>,
    /// Whether the power was cut.
    cut: bool,
    /// The most frames the spill queue held.
    most_spilled: u32,
    /// The highest cursor the test's mover committed and passed to [`Ring::confirm_moved`]: what GC may erase
    /// behind, kept apart from the ring's own copy so that a ring bug cannot grade itself.
    committed: Position,
}

impl Run {
    /// A run of `ring` that has accepted nothing yet.
    fn new(ring: Ring<MemoryFlash>) -> Run {
        Run {
            ring,
            accepted: Vec::new(),
            interrupted: None,
            cut: false,
            most_spilled: 0,
            committed: Position::default(),
        }
    }

    /// The DATA indexes known programmed: everything accepted but what still waits in the spill queue.
    fn durable(&self) -> Vec<u32> {
        let programmed = self.accepted.len() - self.ring.figures().spill_frames as usize;
        data_indexes(&self.accepted[..programmed])
    }

    /// The DATA indexes the cut may have caught programmed, in order: the spilled ones, then the interrupted one.
    fn pending(&self) -> Vec<u32> {
        let programmed = self.accepted.len() - self.ring.figures().spill_frames as usize;
        let mut pending = data_indexes(&self.accepted[programmed..]);
        pending.extend(self.interrupted);
        pending
    }

    /// One GC step, checked against what GC may erase. Returns false when it hit the power cut.
    fn gc_step(&mut self, context: &str) -> bool {
        let slots = self.ring.slots().to_vec();
        match survived(self.ring.erase_step(), "erase") {
            Some(Erasure::Sector { slot, .. }) => {
                check_erasable(slots[slot as usize], self.committed, context);
                true
            }
            Some(Erasure::Idle | Erasure::Paused) => true,
            None => false,
        }
    }
}

/// Where the uncut run programmed each DATA frame, by (boot, index): its block. A run is deterministic up to its
/// cut, so a cut run programmed every frame it programmed into the same block.
type Placed = HashMap<(u32, u32), u32>;

/// Records the block of every DATA frame in `image` that `placed` does not hold yet.
fn record(placed: &mut Placed, image: &[u8]) {
    for found in walk(image, small()) {
        if found.header.kind == Kind::Data {
            placed
                .entry(identify(&found.payload))
                .or_insert(found.header.block_sequence);
        }
    }
}

/// The DATA indexes among `entries`.
fn data_indexes(entries: &[Entry]) -> Vec<u32> {
    entries
        .iter()
        .filter_map(|entry| match entry {
            Entry::Data(index) => Some(*index),
            Entry::Confirm => None,
        })
        .collect()
}

/// Boot `boot`'s DATA frames `indexes`, as (boot, index).
fn tagged(boot: u32, indexes: &[u32]) -> Vec<(u32, u32)> {
    indexes.iter().map(|&index| (boot, index)).collect()
}

/// A ring call's value, or None when it hit the power cut. Any other failure fails the test.
fn survived<T>(result: Result<T, Error<MemoryError>>, what: &str) -> Option<T> {
    match result {
        Ok(value) => Some(value),
        Err(Error::Flash(MemoryError::PowerCut)) => None,
        Err(other) => panic!("{what}: {other:?}"),
    }
}

/// Checks that GC may erase a slot holding `slot`: a foreign one, or a block wholly before `committed`.
fn check_erasable(slot: Slot, committed: Position, context: &str) {
    match slot {
        Slot::Foreign => {}
        Slot::Valid { block_sequence, .. } => assert!(
            block_sequence < committed.block_sequence,
            "{context}: GC erased block {block_sequence}, but the mover committed only {committed:?}"
        ),
        Slot::Erased | Slot::OtherVersion => panic!("{context}: GC erased a slot holding {slot:?}"),
    }
}

/// Boot 1, never cut: blocks 0 and 1 and half of block 2 of a fresh ring, nothing confirmed. Returns the
/// image and its DATA indexes.
fn first_boot() -> (Vec<u8>, Vec<u32>) {
    let mut ring = mount(erased(small()), small(), 1);
    let count = fill_to(&mut ring, 1, 0, 2, 8000);
    (ring.into_flash().into_image(), (0..count).collect())
}

/// Boot 2 over `image`, the power cut by `cut` (operations, torn bytes) or never. With `placed`, it records
/// where each DATA frame went after every step.
///
/// It appends FRAMES DATA frames with one GC step after each. A mover commits everything before the newest
/// block's predecessor at frame 30, and again whenever the spill queue holds 6 frames, each time appending a
/// CONFIRM frame. Every erase is checked against what the mover committed. The run stops at the cut.
fn second_boot(image: &[u8], cut: Option<(u32, usize)>, mut placed: Option<&mut Placed>) -> Run {
    let mut flash = MemoryFlash::from_image(image.to_vec());
    if let Some((operations, torn)) = cut {
        flash.cut_power(operations, torn);
    }
    let mut run = Run::new(mount(flash, small(), 2));
    assert_eq!(run.ring.recover_confirm(), Ok(None), "boot 1 confirmed nothing");
    let mut journal = 0;
    for index in 0..FRAMES {
        let ring = &mut run.ring;
        if (index == 30 || ring.figures().spill_frames >= 6)
            && let Some(at) = ring.programmed()
        {
            let moved = Position {
                block_sequence: at.block_sequence.saturating_sub(1),
                offset: 0,
            };
            if moved > run.committed {
                journal += 1;
                run.committed = moved;
                ring.confirm_moved(moved);
                let confirm = Confirm {
                    moved,
                    boot_id: 2,
                    journal_sequence: journal,
                    ..Confirm::default()
                };
                let appended = ring.append(Kind::Confirm, &confirm.encode(), false, uptime(index));
                if survived(appended, "confirm").is_none() {
                    run.cut = true;
                    return run;
                }
                run.accepted.push(Entry::Confirm);
            }
        }
        let appended = run
            .ring
            .append(Kind::Data, &payload(2, index), index % 9 == 0, uptime(index));
        if survived(appended, "append").is_none() {
            run.interrupted = Some(index);
            run.cut = true;
            return run;
        }
        run.accepted.push(Entry::Data(index));
        run.most_spilled = run.most_spilled.max(run.ring.figures().spill_frames);
        if !run.gc_step(&format!("frame {index}")) {
            run.cut = true;
            return run;
        }
        if let Some(placed) = placed.as_deref_mut() {
            record(placed, run.ring.flash().image());
        }
    }
    run
}

/// Boot 1 over a ring of foreign bytes, the power cut by `cut` (operations, torn bytes) or never. It appends
/// DATA frames with one GC step after each until it programs into block FORMAT_BLOCK. Nothing is committed, so
/// GC may erase only foreign slots. The run stops at the cut.
fn format_boot(cut: Option<(u32, usize)>) -> Run {
    let geometry = small();
    let mut flash = MemoryFlash::from_image(noise(23, geometry.bytes() as usize));
    if let Some((operations, torn)) = cut {
        flash.cut_power(operations, torn);
    }
    let mut run = Run::new(mount(flash, geometry, 1));
    let mut index = 0;
    while run.ring.programmed().is_none_or(|at| at.block_sequence < FORMAT_BLOCK) {
        let appended = run
            .ring
            .append(Kind::Data, &payload(1, index), index % 9 == 0, uptime(index));
        if survived(appended, "append").is_none() {
            run.interrupted = Some(index);
            run.cut = true;
            return run;
        }
        run.accepted.push(Entry::Data(index));
        run.most_spilled = run.most_spilled.max(run.ring.figures().spill_frames);
        if !run.gc_step(&format!("format frame {index}")) {
            run.cut = true;
            return run;
        }
        index += 1;
    }
    run
}

/// Checks the image after a cut (`after_cut`) against the image just before it (`before_cut`, the same run cut
/// with nothing torn): one slot changed at most, an erase only where GC may erase behind `committed`, and every
/// frame intact elsewhere.
fn check_tear(committed: Position, before_cut: &[u8], after_cut: &[u8], context: &str) {
    let geometry = small();
    let changed: Vec<u32> = (0..geometry.blocks())
        .filter(|&slot| slot_bytes(before_cut, geometry, slot) != slot_bytes(after_cut, geometry, slot))
        .collect();
    assert!(changed.len() <= 1, "{context}: one operation touched slots {changed:?}");
    let erased = changed.iter().copied().find(|&slot| {
        let before = slot_bytes(before_cut, geometry, slot);
        let after = slot_bytes(after_cut, geometry, slot);
        before.iter().zip(after).any(|(&old, &new)| !old & new != 0) // a bit went 0 -> 1
    });
    if let Some(slot) = erased {
        let erased_context = format!("{context}, the cut erase");
        check_erasable(
            classify(before_cut, geometry)[slot as usize],
            committed,
            &erased_context,
        );
    }

    for found in walk(before_cut, geometry) {
        if Some(found.slot) == erased {
            continue;
        }
        let start = (geometry.slot_offset(found.slot) + found.offset) as usize;
        let end = start + frame_bytes(found.payload.len());
        assert_eq!(
            before_cut[start..end],
            after_cut[start..end],
            "{context}: a frame valid before the cut changed"
        );
    }
}

/// Checks that the blocks of `image` from block `from` on hold exactly `expected`, the DATA frames programmed into
/// them, in order, then at most the first of `pending`, the one the cut may have caught programmed. Each boot's
/// frames there must be numbered without a gap.
fn check_kept(expected: &[(u32, u32)], pending: &[(u32, u32)], image: &[u8], from: u32, context: &str) {
    let kept = walk_from(image, small(), from);
    let found = data(&kept);
    let whole = found.len() >= expected.len() && found[..expected.len()] == *expected;
    let caught = &found[expected.len().min(found.len())..];
    assert!(
        whole && caught.len() <= 1 && pending.starts_with(caught),
        "{context}: the blocks from {from} on hold {found:?}, not {expected:?} and at most the first of {pending:?}"
    );
    for boot in [1, 2] {
        let numbers: Vec<u32> = kept
            .iter()
            .filter(|found| found.header.boot_id == boot)
            .map(|found| found.header.frame_sequence)
            .collect();
        assert!(
            numbers.windows(2).all(|pair| pair[1] == pair[0] + 1),
            "{context}: boot {boot} numbers {numbers:?}"
        );
    }
}

/// The DATA frames `run` (boot 2) must have kept from its committed cursor on: those of boot 1 (`first`) and its
/// own programmed ones that `placed` puts in a block at or after that cursor's.
fn expected(first: &[u32], run: &Run, placed: &Placed) -> Vec<(u32, u32)> {
    tagged(1, first)
        .into_iter()
        .chain(tagged(2, &run.durable()))
        .filter(|frame| placed[frame] >= run.committed.block_sequence)
        .collect()
}

/// Boots a third time over `after_cut`: the scan, the CONFIRM walk, and the first frame, which must open the slot
/// after the newest block and leave every old block as it was.
fn reboot(run: &Run, after_cut: &[u8], context: &str) {
    let geometry = small();
    // At mount, boot 2's bound is offset 0 of the block after boot 1's newest.
    let at = run.ring.programmed().expect("boot 2 mounts over boot 1's blocks");
    let last = if at.offset == 0 {
        at.block_sequence - 1
    } else {
        at.block_sequence
    };
    let newest = blocks(after_cut, geometry).last().expect("valid blocks survive").0;
    assert!(
        newest == last || newest == last + 1,
        "{context}: newest block {newest}, boot 2 last programmed into {last}"
    );
    let mut ring = mount(MemoryFlash::from_image(after_cut.to_vec()), geometry, 3);
    if let Some(found) = ring.recover_confirm().unwrap() {
        assert!(found.moved <= run.committed, "{context}: a CONFIRM ahead of the mover");
    }
    ring.confirm_moved(Position {
        block_sequence: newest,
        offset: 0,
    });
    let first = payload(3, 0);
    ring.append(Kind::Data, &first, false, uptime(0)).unwrap();
    let mut steps = 0;
    while ring.figures().spill_frames > 0 {
        assert!(
            matches!(ring.erase_step(), Ok(Erasure::Sector { .. })),
            "{context}: stuck spilling"
        );
        ring.drain().unwrap();
        steps += 1;
        assert!(
            steps <= 2 * geometry.sectors_per_block(),
            "{context}: too many erases to open a block"
        );
    }
    let opened = newest + 1;
    let end = frame_bytes(boot_payload(3).len()) + frame_bytes(first.len());
    assert_eq!(
        ring.programmed(),
        Some(Position {
            block_sequence: opened,
            offset: end as u32
        }),
        "{context}"
    );
    let image = ring.flash().image();
    let frames: Vec<(Kind, u32, u32)> = block_frames(image, geometry, geometry.slot(opened), opened)
        .iter()
        .map(|found| (found.header.kind, found.header.boot_id, found.header.frame_sequence))
        .collect();
    assert_eq!(frames, [(Kind::Boot, 3, 0), (Kind::Data, 3, 1)], "{context}");
    assert_eq!(
        slot_bytes(image, geometry, geometry.slot(newest)),
        slot_bytes(after_cut, geometry, geometry.slot(newest)),
        "{context}: the newest old block was appended to"
    );
}

/// Boots again over a cut format run: the first frame opens the slot after the newest block (block 0 when there
/// is none) without touching an old block, and GC then finishes the format.
fn reboot_after_format(run: &Run, after_cut: &[u8], context: &str) {
    let geometry = small();
    let newest = blocks(after_cut, geometry).last().map(|&(block, _)| block);
    let last = run.ring.programmed().map(|at| at.block_sequence);
    let beyond = last.map_or(0, |last| last + 1); // a torn BOOT whose header survived
    assert!(
        newest == last || newest == Some(beyond),
        "{context}: newest block {newest:?}, the run last programmed into {last:?}"
    );
    let mut ring = mount(MemoryFlash::from_image(after_cut.to_vec()), geometry, 2);
    assert_eq!(ring.recover_confirm(), Ok(None), "{context}: nothing was confirmed");
    let first = payload(2, 0);
    ring.append(Kind::Data, &first, false, uptime(0)).unwrap();
    let mut steps = 0;
    while ring.figures().spill_frames > 0 {
        assert!(
            matches!(ring.erase_step(), Ok(Erasure::Sector { .. })),
            "{context}: stuck spilling"
        );
        ring.drain().unwrap();
        steps += 1;
        assert!(
            steps <= 2 * geometry.sectors_per_block(),
            "{context}: too many erases to open a block"
        );
    }
    let opened = newest.map_or(0, |newest| newest + 1);
    let end = frame_bytes(boot_payload(2).len()) + frame_bytes(first.len());
    assert_eq!(
        ring.programmed(),
        Some(Position {
            block_sequence: opened,
            offset: end as u32
        }),
        "{context}"
    );
    for (block, slot) in blocks(after_cut, geometry) {
        assert_eq!(
            slot_bytes(ring.flash().image(), geometry, slot),
            slot_bytes(after_cut, geometry, slot),
            "{context}: old block {block} changed"
        );
    }
    while let Ok(Erasure::Sector { .. }) = ring.erase_step() {}
    assert_eq!(
        ring.figures().foreign,
        0,
        "{context}: the format goes on after the reboot"
    );
}

#[test]
fn a_cut_at_every_operation_loses_nothing_programmed() {
    let (image, first) = first_boot();
    let mut placed = Placed::new();
    record(&mut placed, &image);
    let whole = second_boot(&image, None, Some(&mut placed));
    assert!(!whole.cut);
    let operations = whole.ring.flash().programs() + whole.ring.flash().erases();
    assert!(
        whole.ring.programmed().unwrap().block_sequence >= 7,
        "the workload wraps the 5 slots"
    );
    assert!(whole.most_spilled >= 6, "the workload spills");
    assert!(whole.ring.flash().erases() >= 12, "the workload erases");
    assert_eq!(
        whole.ring.confirmed(),
        whole.committed,
        "the ring took the mover's cursor"
    );
    assert!(whole.pending().is_empty(), "the uncut run ends drained");
    let from = whole.committed.block_sequence;
    check_kept(
        &expected(&first, &whole, &placed),
        &[],
        whole.ring.flash().image(),
        from,
        "uncut",
    );

    for operation in 0..operations {
        let untorn = second_boot(&image, Some((operation, 0)), None);
        assert!(untorn.cut, "operation {operation} of {operations} is reached");
        let before_cut = untorn.ring.flash().image().to_vec();
        for torn in TEARS {
            let context = format!("cut at operation {operation} of {operations}, {torn} bytes done");
            let run = second_boot(&image, Some((operation, torn)), None);
            assert!(run.cut, "{context}");
            let after_cut = run.ring.flash().image().to_vec();
            check_tear(run.committed, &before_cut, &after_cut, &context);
            let pending = tagged(2, &run.pending());
            let from = run.committed.block_sequence;
            check_kept(&expected(&first, &run, &placed), &pending, &after_cut, from, &context);
            reboot(&run, &after_cut, &context);
        }
        let context = format!("cut before operation {operation}");
        let pending = tagged(2, &untorn.pending());
        let from = untorn.committed.block_sequence;
        check_kept(
            &expected(&first, &untorn, &placed),
            &pending,
            &before_cut,
            from,
            &context,
        );
        reboot(&untorn, &before_cut, &context);
    }
}

#[test]
fn a_cut_at_every_operation_of_a_format_loses_nothing_programmed() {
    let whole = format_boot(None);
    assert!(!whole.cut);
    let operations = whole.ring.flash().programs() + whole.ring.flash().erases();
    assert!(whole.most_spilled >= 1, "the writer spills until slot 0 is erased");
    assert_eq!(
        whole.ring.flash().erases(),
        5 * 4,
        "GC formats all five slots, once each"
    );
    assert_eq!(whole.ring.figures().foreign, 0);
    assert!(whole.pending().is_empty(), "the uncut run ends drained");
    let image = whole.ring.flash().image();
    check_kept(&tagged(1, &whole.durable()), &[], image, 0, "uncut format");

    for operation in 0..operations {
        let untorn = format_boot(Some((operation, 0)));
        assert!(untorn.cut, "operation {operation} of {operations} is reached");
        let before_cut = untorn.ring.flash().image().to_vec();
        for torn in TEARS {
            let context = format!("format cut at operation {operation} of {operations}, {torn} bytes done");
            let run = format_boot(Some((operation, torn)));
            assert!(run.cut, "{context}");
            let after_cut = run.ring.flash().image().to_vec();
            check_tear(run.committed, &before_cut, &after_cut, &context);
            let (expected, pending) = (tagged(1, &run.durable()), tagged(1, &run.pending()));
            check_kept(&expected, &pending, &after_cut, 0, &context);
            reboot_after_format(&run, &after_cut, &context);
        }
        let context = format!("format cut before operation {operation}");
        let (expected, pending) = (tagged(1, &untorn.durable()), tagged(1, &untorn.pending()));
        check_kept(&expected, &pending, &before_cut, 0, &context);
        reboot_after_format(&untorn, &before_cut, &context);
    }
}

#[test]
fn a_torn_data_frame_loses_only_itself() {
    let geometry = small();
    for torn in [0, 25, 40, 60] {
        let mut flash = erased(geometry);
        flash.cut_power(11, torn); // the BOOT and 10 frames complete; the 11th frame is torn
        let mut ring = mount(flash, geometry, 1);
        for index in 0..10 {
            ring.append(Kind::Data, &payload(1, index), false, uptime(index))
                .unwrap();
        }
        let torn_append = ring.append(Kind::Data, &payload(1, 10), false, uptime(10));
        assert_eq!(torn_append, Err(Error::Flash(MemoryError::PowerCut)));
        let image = ring.into_flash().into_image();
        assert_eq!(
            data(&walk(&image, geometry)),
            (0..10).map(|index| (1, index)).collect::<Vec<_>>()
        );

        let mut reboot = mount(MemoryFlash::from_image(image.clone()), geometry, 2);
        reboot.append(Kind::Data, &payload(2, 0), false, uptime(0)).unwrap();
        assert_eq!(
            reboot.programmed().unwrap().block_sequence,
            1,
            "torn {torn}: the next slot, not block 0"
        );
        assert_eq!(
            slot_bytes(reboot.flash().image(), geometry, 0),
            slot_bytes(&image, geometry, 0)
        );
    }
}

#[test]
fn a_torn_boot_frame_is_foreign_or_a_block_without_frames() {
    let geometry = Geometry::new(4, 1).unwrap();
    for (torn, header_survives) in [(20, false), (45, true)] {
        let mut flash = erased(geometry);
        flash.cut_power(2, torn); // block 0's BOOT and one full frame complete; block 1's BOOT is torn
        let mut ring = mount(flash, geometry, 1);
        let largest = vec![b'f'; ring.max_payload()];
        ring.append(Kind::Data, &largest, false, 0).unwrap();
        assert_eq!(
            ring.append(Kind::Data, b"x", false, 1),
            Err(Error::Flash(MemoryError::PowerCut))
        );
        let image = ring.into_flash().into_image();

        let mut reboot = mount(MemoryFlash::from_image(image.clone()), geometry, 2);
        if header_survives {
            let torn_block = Slot::Valid {
                block_sequence: 1,
                boot_id: 1,
            };
            assert_eq!(
                reboot.slots()[1],
                torn_block,
                "a whole header names a block, even over a torn payload"
            );
            assert!(block_frames(&image, geometry, 1, 1).is_empty());
            reboot.append(Kind::Data, b"y", false, 2).unwrap();
            assert_eq!(reboot.programmed().unwrap().block_sequence, 2);
            assert_eq!(
                slot_bytes(reboot.flash().image(), geometry, 1),
                slot_bytes(&image, geometry, 1)
            );
        } else {
            assert_eq!(reboot.slots()[1], Slot::Foreign);
            reboot.append(Kind::Data, b"y", false, 2).unwrap();
            assert_eq!(reboot.figures().spill_frames, 1, "the torn slot must be erased first");
            assert_eq!(
                reboot.erase_step(),
                Ok(Erasure::Sector { slot: 1, sector: 0 }),
                "no confirmation needed"
            );
            reboot.drain().unwrap();
            let reopened = block_frames(reboot.flash().image(), geometry, 1, 1);
            assert_eq!((reopened[0].header.kind, reopened[0].header.boot_id), (Kind::Boot, 2));
            assert_eq!(reopened[1].payload, b"y");
        }
    }
}

/// Boot 1 fills blocks 0..3 of the small ring and confirms everything before block 1. Boot 2 cuts the power
/// on its erase of slot 0 after `operations` whole sectors, with `torn` bytes of the next one erased.
/// Returns the image after the cut.
fn cut_erase(operations: u32, torn: usize) -> Vec<u8> {
    let geometry = small();
    let mut ring = mount(erased(geometry), geometry, 1);
    fill_to(&mut ring, 1, 0, 3, 100);
    let moved = Position {
        block_sequence: 1,
        offset: 0,
    };
    let confirm = Confirm {
        moved,
        ..Confirm::default()
    };
    ring.append(Kind::Confirm, &confirm.encode(), false, 0).unwrap();
    let mut flash = MemoryFlash::from_image(ring.into_flash().into_image());
    flash.cut_power(operations, torn);
    let mut ring = mount(flash, geometry, 2);
    assert_eq!(ring.recover_confirm().unwrap().map(|found| found.moved), Some(moved));
    for sector in (4 - operations..4).rev() {
        assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector }));
    }
    assert_eq!(ring.erase_step(), Err(Error::Flash(MemoryError::PowerCut)));
    ring.into_flash().into_image()
}

#[test]
fn a_cut_between_sector_erases_leaves_a_valid_header_and_the_block_is_erased_again() {
    let geometry = small();
    let image = cut_erase(2, 0); // sectors 3 and 2 erased, 1 and 0 (the header) not
    assert!(slot_bytes(&image, geometry, 0)[8192..].iter().all(|&byte| byte == 0xFF));
    let mut ring = mount(MemoryFlash::from_image(image), geometry, 3);
    let moved_block = Slot::Valid {
        block_sequence: 0,
        boot_id: 1,
    };
    assert_eq!(ring.slots()[0], moved_block, "the header sector goes last");
    assert_eq!(
        ring.erase_step(),
        Ok(Erasure::Idle),
        "not before its confirmation is recovered"
    );
    ring.recover_confirm().unwrap();
    for sector in [3, 2, 1, 0] {
        assert_eq!(
            ring.erase_step(),
            Ok(Erasure::Sector { slot: 0, sector }),
            "erased again, in full"
        );
    }
    assert!(
        slot_bytes(ring.flash().image(), geometry, 0)
            .iter()
            .all(|&byte| byte == 0xFF)
    );
}

#[test]
fn a_torn_header_sector_erase_is_caught_by_the_full_read() {
    let geometry = small();
    let image = cut_erase(3, 100); // sectors 3, 2, 1 erased; sector 0 only for its first 100 bytes
    let slot = slot_bytes(&image, geometry, 0);
    assert!(slot[..100].iter().all(|&byte| byte == 0xFF) && slot[100..4096].iter().any(|&byte| byte != 0xFF));
    let mut ring = mount(MemoryFlash::from_image(image), geometry, 3);
    assert_eq!(ring.slots()[0], Slot::Erased, "the header reads erased");
    ring.recover_confirm().unwrap();
    let mut index = 0;
    while ring.figures().spill_frames == 0 {
        ring.append(Kind::Data, &payload(3, index), false, uptime(index))
            .unwrap(); // block 4, then block 5
        index += 1;
    }
    assert_eq!(
        ring.slots()[0],
        Slot::Foreign,
        "block 5's full read found the dirty sector"
    );
    while ring.figures().spill_frames > 0 {
        assert!(matches!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, .. })));
        ring.drain().unwrap();
    }
    let reopened = block_frames(ring.flash().image(), geometry, 0, 5);
    assert_eq!(reopened[0].header.boot_id, 3);
    assert_eq!(data(&reopened).last(), Some(&(3, index - 1)));
}
