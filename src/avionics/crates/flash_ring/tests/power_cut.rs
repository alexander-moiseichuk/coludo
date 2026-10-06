//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Power cuts at every flash operation, in two workloads:
//! * boot 2, over a ring that boot 1 left with a foreign slot in it: a saver comes up late and dies, the stage
//!   enters BOOSTING and never leaves, the eraser stalls for a while, and the ring wraps twice. So every kind of
//!   operation is cut: block headers, chunks, and erases of a foreign slot, of saved blocks, and of unsaved pad
//!   and flight blocks under newest wins;
//! * boot 1 formats a ring of foreign bytes, waiting on the eraser for its first block.
//!
//! Each workload runs once whole. Then, for each of its flash operations, it runs again with the power cut there,
//! the operation torn at several lengths: inside the chunk header's length, count and CRC, the block header short
//! of its last byte and whole, inside the payload or a sector, and a call that completed but never returned.
//! The image before the cut is the same run cut with nothing torn. After every cut:
//! * the cut changed one slot at most: the one the whole run touched with that operation;
//! * every chunk programmed before the cut is still in place, byte for byte, unless a completed erase took its
//!   block or the cut erase was taking its slot;
//! * the blocks hold exactly those chunks in order, plus the interrupted one, whole or torn, at most;
//! * the scan agrees with the oracle's reading of every slot, and numbers past the newest block and the hint;
//! * a reboot opens a new block and never appends to an old one, and its eraser finishes any format.
//!
//! Every erase of the whole runs is graded against the test's own model of the eraser's rules.

mod common;

use std::collections::BTreeSet;
use std::ops::Range;

use common::*;
use flash_ring::chunk::{self, Torn, chunk_bytes};
use flash_ring::header::HEADER_BYTES;
use flash_ring::{
    BLOCK_BYTES, BlockTable, ERASE_DELAY_MS, Eraser, Erasure, Error, Hints, MemoryError, MemoryFlash, Position, Slot,
};

/// The tear lengths tried at each operation (see the module doc). `usize::MAX` completes the call.
const TEARS: [usize; 10] = [1, 2, 4, 7, 8, 55, 56, 57, 300, usize::MAX];

/// A workload: who boots over what, and what happens while it writes.
struct Plan {
    /// The boot id.
    boot: u32,
    /// The image it boots over.
    image: Vec<u8>,
    /// The chunks boot 1 left there (boot, index, where), oldest first.
    earlier: Vec<Appended>,
    /// The ring positions the previous boot left.
    hints: Hints,
    /// How many chunks it appends.
    chunks: u32,
    /// The chunk indexes the saver is up for: it passes every block before the writer's on every third.
    saver: Range<u32>,
    /// The chunk the stage enters BOOSTING at, for good.
    boosting: Option<u32>,
    /// The chunk indexes during which the eraser runs only when the writer would block, so the ring fills.
    stalled: Range<u32>,
}

/// A chunk a boot programmed.
#[derive(Clone, Copy, Debug)]
struct Appended {
    /// The boot.
    boot: u32,
    /// Its index in the boot.
    index: u32,
    /// Where it starts.
    at: Position,
    /// Its slot.
    slot: u32,
}

/// What a run did, up to its end or its cut.
#[derive(Debug, Default)]
struct Run {
    /// The flash content when it stopped.
    image: Vec<u8>,
    /// The chunks it programmed, in order.
    appended: Vec<Appended>,
    /// The chunk whose append the cut interrupted.
    interrupted: Option<u32>,
    /// The erasures that completed, in order.
    erasures: Vec<Erasure>,
    /// Each flash operation in order: the erasure an erase was, None for a program.
    operations: Vec<Option<Erasure>>,
    /// Whether the power was cut.
    cut: bool,
    /// The writer's published position when it stopped.
    position: Position,
    /// How many appends would have blocked.
    would_block: u32,
}

/// The test's own model of the eraser's rules: the saver positions it published, and when.
struct Model {
    /// (position, uptime ms first published), in order.
    published: Vec<(Position, u32)>,
    /// The reserve.
    reserve: u32,
}

impl Model {
    /// The saver as the eraser sees it now.
    fn saver(&self) -> Option<Position> {
        self.published.last().map(|&(position, _)| position)
    }

    /// Publishes `position` at `now_ms`, if it moved.
    fn publish(&mut self, position: Position, now_ms: u32) {
        if self.saver() != Some(position) {
            self.published.push((position, now_ms));
        }
    }

    /// The newest block sequence the saver published at least [`ERASE_DELAY_MS`] before `now_ms`: the normal rule
    /// may erase the blocks before it, and no others.
    fn erasable_before(&self, now_ms: u32) -> u32 {
        self.published
            .iter()
            .filter(|&&(_, published_ms)| published_ms + ERASE_DELAY_MS <= now_ms)
            .map(|&(position, _)| position.block_sequence)
            .max()
            .unwrap_or(0)
    }

    /// Grades `erasure` against the table `before` the step, at `now_ms`.
    fn check(&self, before: &[Slot], erasure: &Erasure, now_ms: u32, context: &str) {
        let current = self.saver().map_or(0, |saver| saver.block_sequence);
        let foreign = before.contains(&Slot::Foreign);
        let free = before
            .iter()
            .filter(|slot| match **slot {
                Slot::Erased => true,
                Slot::Valid { block_sequence, .. } => block_sequence < current,
                _ => false,
            })
            .count() as u32;
        let unsaved: Vec<(u32, u32, bool)> = (0..)
            .zip(before)
            .filter_map(|(slot, state)| match *state {
                Slot::Valid { block_sequence, flight } if block_sequence >= current => {
                    Some((block_sequence, slot, flight))
                }
                Slot::OtherVersion {
                    block_sequence, flight, ..
                } => Some((block_sequence, slot, flight)),
                _ => None,
            })
            .collect();
        match *erasure {
            Erasure::Foreign { slot } => assert_eq!(before[slot as usize], Slot::Foreign, "{context}"),
            Erasure::Saved { slot, block_sequence } => {
                assert!(
                    matches!(before[slot as usize], Slot::Valid { block_sequence: found, .. } if found == block_sequence),
                    "{context}: saved slot {slot} held {:?}",
                    before[slot as usize]
                );
                let erasable = self.erasable_before(now_ms);
                assert!(
                    block_sequence < erasable,
                    "{context}: block {block_sequence} erased before its delay"
                );
            }
            Erasure::Lost(loss) => {
                // newest wins starts at the reserve, one block of margin for the writer's next open
                assert!(
                    !foreign && free <= self.reserve,
                    "{context}: newest wins with {free} free"
                );
                let non_flight = unsaved.iter().filter(|(_, _, flight)| !flight).min();
                let expected = non_flight.or(unsaved.iter().min()).copied();
                assert_eq!(
                    Some((loss.block_sequence, loss.slot, loss.flight)),
                    expected,
                    "{context}"
                );
                let header = loss.header.expect("a loss names its boot and session");
                assert_eq!(
                    (header.block_sequence, header.flight),
                    (loss.block_sequence, loss.flight)
                );
            }
            Erasure::Idle => {
                assert!(!foreign, "{context}: idle with a foreign slot");
                assert!(
                    free > self.reserve || unsaved.is_empty(),
                    "{context}: idle on a full ring"
                );
            }
        }
    }
}

/// The program and erase calls `flash` has taken.
fn operations(flash: &MemoryFlash) -> u32 {
    flash.programs() + flash.erases()
}

/// One eraser step with the model's saver, graded and recorded. Returns false when the power was cut.
fn step(
    eraser: &mut Eraser<'_, &MemoryFlash>,
    table: &BlockTable,
    flash: &MemoryFlash,
    model: &Model,
    now_ms: u32,
    run: &mut Run,
) -> bool {
    let before = snapshot(table);
    let calls = operations(flash);
    match eraser.erase_step(model.saver(), now_ms) {
        Ok(erasure) => {
            model.check(&before, &erasure, now_ms, &format!("step at {now_ms} ms"));
            if operations(flash) > calls {
                run.operations.push(Some(erasure));
            }
            if erasure != Erasure::Idle {
                run.erasures.push(erasure);
            }
            true
        }
        Err(Error::Flash(MemoryError::PowerCut)) => {
            run.cut = true;
            false
        }
        Err(other) => panic!("step at {now_ms} ms: {other:?}"),
    }
}

/// Runs `plan`, the power cut by `cut` (operations, torn bytes) or never. The run stops at the cut.
fn run(plan: &Plan, cut: Option<(u32, usize)>) -> Run {
    let geometry = small();
    let flash = MemoryFlash::from_image(plan.image.clone());
    if let Some((operations, torn)) = cut {
        flash.cut_power(operations, torn);
    }
    let scanned = bring_up(&flash, geometry, plan.hints);
    let table = &scanned.table;
    let mut ring = writer(&flash, &scanned, plan.boot);
    let mut eraser = eraser(&flash, table);
    let mut model = Model {
        published: Vec::new(),
        reserve: geometry.reserve_blocks(),
    };
    let mut outcome = Run::default();
    'chunks: for index in 0..plan.chunks {
        let now_ms = index * 40;
        if plan.boosting == Some(index) {
            ring.set_flight(true);
        }
        if plan.saver.contains(&index) && index % 3 == 0 {
            let saver = Position {
                block_sequence: ring.position().block_sequence,
                offset: 0,
            };
            model.publish(saver, now_ms);
        }
        loop {
            let calls = operations(&flash);
            let appended = ring.append(&payload(plan.boot, index), records(index));
            outcome.operations.extend((calls..operations(&flash)).map(|_| None));
            match appended {
                Ok(at) => {
                    let slot = slot_of(table, at.block_sequence);
                    outcome.appended.push(Appended {
                        boot: plan.boot,
                        index,
                        at,
                        slot,
                    });
                    break;
                }
                Err(Error::WouldBlock) => {
                    outcome.would_block += 1;
                    if !step(&mut eraser, table, &flash, &model, now_ms, &mut outcome) {
                        break 'chunks;
                    }
                }
                Err(Error::Flash(MemoryError::PowerCut)) => {
                    outcome.interrupted = Some(index);
                    outcome.cut = true;
                    break 'chunks;
                }
                Err(other) => panic!("chunk {index}: {other:?}"),
            }
        }
        if !plan.stalled.contains(&index) && !step(&mut eraser, table, &flash, &model, now_ms + 20, &mut outcome) {
            break;
        }
    }
    outcome.position = ring.position();
    drop((ring, eraser));
    outcome.image = flash.image();
    outcome
}

/// Boot 1, never cut: blocks 0, 1 and half of 2, with slot 7 holding MicroPython-era bytes.
fn first_boot() -> (Vec<u8>, Vec<Appended>, Position) {
    let geometry = small();
    let mut image = vec![0xFF; geometry.bytes() as usize];
    image[7 * BLOCK_BYTES as usize..].copy_from_slice(&noise(41, BLOCK_BYTES as usize));
    let flash = MemoryFlash::from_image(image);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut appended = Vec::new();
    let mut index = 0;
    while ring.position()
        < (Position {
            block_sequence: 2,
            offset: 2000,
        })
    {
        let at = ring.append(&payload(1, index), records(index)).unwrap();
        let slot = slot_of(&scanned.table, at.block_sequence);
        appended.push(Appended {
            boot: 1,
            index,
            at,
            slot,
        });
        index += 1;
    }
    let position = ring.position();
    drop(ring);
    (flash.into_image(), appended, position)
}

/// The second-boot workload over boot 1's ring.
fn second_boot_plan() -> Plan {
    let (image, earlier, position) = first_boot();
    Plan {
        boot: 2,
        image,
        earlier,
        hints: Hints {
            writer: Some(position),
            saver: None,
        },
        chunks: 110,
        saver: 8..30,
        boosting: Some(45),
        stalled: 60..82,
    }
}

/// The format workload: boot 1 over a ring of foreign bytes, with no saver.
fn format_plan() -> Plan {
    Plan {
        boot: 1,
        image: noise(23, small().bytes() as usize),
        earlier: Vec::new(),
        hints: Hints::default(),
        chunks: 40,
        saver: 0..0,
        boosting: None,
        stalled: 0..0,
    }
}

/// The slot an erasure erased.
fn erased_slot(erasure: &Erasure) -> Option<u32> {
    match *erasure {
        Erasure::Foreign { slot } | Erasure::Saved { slot, .. } => Some(slot),
        Erasure::Lost(loss) => Some(loss.slot),
        Erasure::Idle => None,
    }
}

/// The chunk bytes chunk `index` of `boot` programs.
fn chunk_of(boot: u32, index: u32) -> Vec<u8> {
    let mut bytes = Vec::new();
    chunk::encode(&mut bytes, &payload(boot, index), records(index)).unwrap();
    bytes
}

/// Checks `cut`, the plan run with the power cut at `operation` after `torn_bytes` of it took effect, against
/// `untorn` (the same cut with nothing torn: the flash just before the operation) and `whole` (the run never cut).
fn check_cut(plan: &Plan, whole: &Run, untorn: &Run, cut: &Run, operation: u32, torn_bytes: usize, context: &str) {
    let geometry = small();
    assert!(cut.cut, "{context}: the cut is reached");
    // the operation changed one slot at most: the one the whole run touched with it
    let changed: Vec<u32> = (0..geometry.blocks())
        .filter(|&slot| slot_bytes(&untorn.image, slot) != slot_bytes(&cut.image, slot))
        .collect();
    assert!(changed.len() <= 1, "{context}: one operation changed slots {changed:?}");
    let cut_erase = whole.operations[operation as usize];
    let cut_slot = cut_erase.as_ref().and_then(erased_slot);
    if let Some(&slot) = changed.first() {
        let old = slot_bytes(&untorn.image, slot);
        let raised = old
            .iter()
            .zip(slot_bytes(&cut.image, slot))
            .any(|(&old, &new)| !old & new != 0);
        match cut_slot {
            Some(erased) => assert_eq!(slot, erased, "{context}: the erase hit another slot"),
            None => assert!(!raised, "{context}: a program set a bit"),
        }
    }

    // every chunk programmed before the cut is in place, unless its block was erased
    let erased_blocks: BTreeSet<u32> = cut
        .erasures
        .iter()
        .filter_map(|erasure| match *erasure {
            Erasure::Saved { block_sequence, .. } => Some(block_sequence),
            Erasure::Lost(loss) => Some(loss.block_sequence),
            _ => None,
        })
        .collect();
    let kept: Vec<&Appended> = plan
        .earlier
        .iter()
        .chain(&cut.appended)
        .filter(|appended| !erased_blocks.contains(&appended.at.block_sequence) && Some(appended.slot) != cut_slot)
        .collect();
    for appended in &kept {
        let bytes = chunk_of(appended.boot, appended.index);
        let start = (appended.slot * BLOCK_BYTES + appended.at.offset) as usize;
        assert_eq!(
            cut.image[start..start + bytes.len()],
            bytes,
            "{context}: chunk {}:{} changed",
            appended.boot,
            appended.index
        );
    }

    // the blocks hold exactly those chunks, plus the interrupted one whole or torn, at most. The block a cut erase
    // was taking may still read (when nothing of it was torn yet), and then holds what it held: no wrong chunk.
    let blocks = blocks(&cut.image, geometry);
    let (erasing, others): (Vec<&Block>, Vec<&Block>) = blocks.iter().partition(|block| Some(block.slot) == cut_slot);
    for block in erasing {
        let held: Vec<(u32, u32)> = plan
            .earlier
            .iter()
            .chain(&cut.appended)
            .filter(|appended| appended.at.block_sequence == block.header.block_sequence)
            .map(|appended| (appended.boot, appended.index))
            .collect();
        assert!(
            held.starts_with(&data(&block.chunks)),
            "{context}: the block under the cut erase"
        );
    }
    let found: Vec<(u32, u32)> = others.iter().flat_map(|block| data(&block.chunks)).collect();
    let expected: Vec<(u32, u32)> = kept.iter().map(|appended| (appended.boot, appended.index)).collect();
    let mut landed = expected.clone(); // the interrupted chunk landed whole before the power went
    landed.extend(cut.interrupted.map(|index| (plan.boot, index)));
    assert!(
        found == expected || found == landed,
        "{context}: the blocks hold {found:?}, not {expected:?} and at most {:?}",
        cut.interrupted
    );
    // a torn chunk is the interrupted chunk program, and that one must be reported: once any byte of it landed and
    // not all of it did, the walk finds it at its offset, with nothing whole behind it (a loss), and its record
    // count once the count field landed. A walk that dropped it silently would lose its records with no loss line.
    let torn: Vec<(u32, Torn)> = others
        .iter()
        .flat_map(|block| block.torn.iter().map(|torn| (block.header.block_sequence, *torn)))
        .collect();
    let tearing = cut.interrupted.and_then(|index| {
        // the same run up to the cut, so the whole run says where the interrupted chunk goes
        let appended = whole.appended.iter().find(|appended| appended.index == index)?;
        let bytes = chunk_of(plan.boot, index);
        let start = (appended.slot * BLOCK_BYTES + appended.at.offset) as usize;
        let landed = &cut.image[start..start + bytes.len()];
        let started = landed.iter().any(|&byte| byte != 0xFF); // else the cut took its block header, or nothing
        (started && landed != bytes.as_slice()).then_some((index, appended.at))
    });
    match tearing {
        Some((index, at)) => {
            assert_eq!(torn.len(), 1, "{context}: the torn chunk, reported once: {torn:?}");
            let (block_sequence, found) = torn[0];
            assert_eq!(
                (block_sequence, found.offset, found.followed),
                (at.block_sequence, at.offset, false),
                "{context}"
            );
            if torn_bytes >= 4 {
                assert_eq!(found.records, Some(records(index)), "{context}: the count");
            }
        }
        None => assert!(
            torn.is_empty(),
            "{context}: only a chunk the cut caught is torn: {torn:?}"
        ),
    }

    // the scan agrees with the oracle, and numbers past the newest block and the hint
    let hints = Hints {
        writer: Some(cut.position),
        saver: None,
    };
    let flash = MemoryFlash::from_image(cut.image.clone());
    let scanned = bring_up(&flash, geometry, hints);
    let oracle = oracle_slots(&cut.image, geometry);
    assert_eq!(snapshot(&scanned.table), oracle, "{context}: the scan");
    let newest = oracle.iter().filter_map(|slot| slot.block_sequence()).max();
    let next = newest
        .map_or(0, |newest| newest + 1)
        .max(cut.position.block_sequence + 1);
    assert_eq!(scanned.next_sequence, next, "{context}");
    reboot(&cut.image, hints, context);
}

/// Boots again over `image`: the first chunk opens a new block, at the next sequence, in a slot that was erased,
/// and every other slot is left as it was unless the eraser freed it (a foreign slot, or newest wins: there is no
/// saver). Then the eraser finishes any format.
fn reboot(image: &[u8], hints: Hints, context: &str) {
    let geometry = small();
    let flash = MemoryFlash::from_image(image.to_vec());
    let scanned = bring_up(&flash, geometry, hints);
    let before = snapshot(&scanned.table);
    let mut ring = writer(&flash, &scanned, 3);
    let mut eraser = eraser(&flash, &scanned.table);
    let mut freed = BTreeSet::new();
    let first = payload(3, 0);
    let at = loop {
        match ring.append(&first, records(0)) {
            Ok(at) => break at,
            Err(Error::WouldBlock) => match eraser.erase_step(None, 0) {
                Ok(Erasure::Foreign { slot }) => assert!(freed.insert(slot), "{context}: slot {slot} twice"),
                Ok(Erasure::Lost(loss)) => assert!(freed.insert(loss.slot), "{context}: slot {} twice", loss.slot),
                other => panic!("{context}: {other:?} with no saver, and nothing erased"),
            },
            Err(other) => panic!("{context}: the reboot's append: {other:?}"),
        }
    };
    let expected = Position {
        block_sequence: scanned.next_sequence,
        offset: HEADER_BYTES as u32,
    };
    assert_eq!(at, expected, "{context}: the reboot's first chunk");
    let slot = slot_of(&scanned.table, at.block_sequence);
    assert!(
        before[slot as usize] == Slot::Erased || freed.contains(&slot),
        "{context}: block opened over {:?}",
        before[slot as usize]
    );
    let after = flash.image();
    for other in (0..geometry.blocks()).filter(|&other| other != slot && !freed.contains(&other)) {
        assert_eq!(
            slot_bytes(&after, other),
            slot_bytes(image, other),
            "{context}: slot {other} changed"
        );
    }
    let opened = blocks(&after, geometry);
    let opened = opened.iter().find(|block| block.slot == slot).unwrap();
    assert_eq!(
        (opened.header.boot_id, data(&opened.chunks)),
        (3, vec![(3, 0)]),
        "{context}"
    );
    while let Ok(Erasure::Foreign { .. }) = eraser.erase_step(None, 0) {}
    assert_eq!(scanned.table.figures(None).foreign, 0, "{context}: the format goes on");
}

/// Runs `plan` whole, then cut at every operation with every tear, checking each.
fn sweep(plan: &Plan, name: &str) -> Run {
    let whole = run(plan, None);
    assert!(!whole.cut, "{name}: the whole run");
    let operations = whole.operations.len() as u32;
    for operation in 0..operations {
        let untorn = run(plan, Some((operation, 0)));
        let context = format!("{name}: cut before operation {operation} of {operations}");
        check_cut(plan, &whole, &untorn, &untorn, operation, 0, &context);
        for torn in TEARS {
            let cut = run(plan, Some((operation, torn)));
            let context = format!("{name}: cut at operation {operation} of {operations}, {torn} bytes done");
            check_cut(plan, &whole, &untorn, &cut, operation, torn, &context);
        }
    }
    whole
}

#[test]
fn a_cut_at_every_operation_of_a_wrapping_boot_loses_no_programmed_chunk() {
    let plan = second_boot_plan();
    let whole = sweep(&plan, "second boot");
    let kinds = |wanted: fn(&Erasure) -> bool| whole.erasures.iter().filter(|erasure| wanted(erasure)).count();
    assert!(
        whole.position.block_sequence >= 3 + 2 * 8,
        "the boot wraps the 8 slots twice: {:?}",
        whole.position
    );
    assert_eq!(
        kinds(|erasure| matches!(erasure, Erasure::Foreign { .. })),
        1,
        "boot 1's foreign slot"
    );
    assert!(
        kinds(|erasure| matches!(erasure, Erasure::Saved { .. })) >= 4,
        "the saver's blocks"
    );
    assert!(
        kinds(|erasure| matches!(erasure, Erasure::Lost(loss) if !loss.flight)) >= 2,
        "pad losses"
    );
    assert!(
        kinds(|erasure| matches!(erasure, Erasure::Lost(loss) if loss.flight)) >= 2,
        "flight losses"
    );
    assert!(whole.would_block >= 1, "the stalled eraser blocks the writer");
    let programs = whole.operations.iter().filter(|operation| operation.is_none()).count() as u32;
    let headers = whole.position.block_sequence - 2; // blocks 3 up to the last one
    assert_eq!(
        programs,
        plan.chunks + headers,
        "every chunk and every block header is one cut point"
    );
}

#[test]
fn a_cut_at_every_operation_of_a_format_loses_no_programmed_chunk() {
    let plan = format_plan();
    let whole = sweep(&plan, "format");
    let formatted = whole
        .erasures
        .iter()
        .filter(|erasure| matches!(erasure, Erasure::Foreign { .. }))
        .count();
    assert_eq!(formatted, 8, "the eraser formats every slot, once");
    assert!(whole.would_block >= 1, "the writer waits for its first slot");
}

#[test]
fn a_torn_chunk_loses_only_itself() {
    let geometry = small();
    let short = |index: u32| {
        format!("1:{index}:")
            .into_bytes()
            .into_iter()
            .chain([b'c'; 90])
            .collect::<Vec<u8>>()
    };
    for torn in [1, 4, 8, 9, 100] {
        let flash = erased(geometry);
        flash.cut_power(11, torn); // block 0's header and 10 chunks complete; the 11th is torn
        let scanned = bring_up(&flash, geometry, Hints::default());
        let mut ring = writer(&flash, &scanned, 1);
        for index in 0..10 {
            ring.append(&short(index), 1).unwrap();
        }
        assert_eq!(ring.append(&[b'x'; 200], 1), Err(Error::Flash(MemoryError::PowerCut)));
        let position = ring.position();
        drop(ring);
        let image = flash.image();
        let block = &blocks(&image, geometry)[0];
        assert_eq!(
            data(&block.chunks),
            (0..10).map(|index| (1, index)).collect::<Vec<_>>(),
            "torn {torn}"
        );
        let offset = (HEADER_BYTES + 10 * chunk_bytes(short(0).len())) as u32;
        let records = (torn >= 4).then_some(1);
        let followed = false; // nothing whole behind it, in the boot's last block: a loss
        assert_eq!(
            block.torn,
            [Torn {
                offset,
                records,
                followed
            }],
            "torn {torn}: the last chunk, its count as read"
        );
        let hints = Hints {
            writer: Some(position),
            saver: None,
        };
        reboot(&image, hints, &format!("torn chunk, {torn} bytes"));
    }
}

#[test]
fn a_torn_block_header_is_foreign_or_an_empty_block() {
    let geometry = small();
    for torn in [1, 30, 55, 56] {
        let flash = erased(geometry);
        flash.cut_power(2, torn); // block 0's header and one full chunk, then block 1's header is torn
        let scanned = bring_up(&flash, geometry, Hints::default());
        let mut ring = writer(&flash, &scanned, 1);
        ring.append(&[b'f'; chunk::MAX_PAYLOAD], 1).unwrap();
        assert_eq!(ring.append(b"x", 1), Err(Error::Flash(MemoryError::PowerCut)));
        let position = ring.position();
        drop(ring);
        let image = flash.image();
        let hints = Hints {
            writer: Some(position),
            saver: None,
        };
        let rescanned = bring_up(&MemoryFlash::from_image(image.clone()), geometry, hints);
        let whole = torn >= HEADER_BYTES;
        let expected = if whole {
            Slot::Valid {
                block_sequence: 1,
                flight: false,
            }
        } else {
            Slot::Foreign
        };
        assert_eq!(rescanned.table.slot(1), expected, "torn {torn}");
        assert_eq!(
            rescanned.next_sequence,
            if whole { 2 } else { 1 },
            "torn {torn}: no number given twice"
        );
        if whole {
            assert!(
                blocks(&image, geometry)[1].chunks.is_empty(),
                "a whole header over no chunk"
            );
        }
        reboot(&image, hints, &format!("torn header, {torn} bytes"));
    }
}

#[test]
fn a_torn_erase_reads_foreign_or_erased_over_dirty_bytes_and_is_erased_again() {
    let geometry = small();
    for torn in [1, 55, 56, 2000, 4095] {
        // boot 1 fills slots 0..3, the saver passes blocks 0 and 1, and the erase of block 0 is cut
        let flash = erased(geometry);
        let scanned = bring_up(&flash, geometry, Hints::default());
        let mut ring = writer(&flash, &scanned, 1);
        let mut eraser = eraser(&flash, &scanned.table);
        for _ in 0..4 {
            ring.append(&[b'a'; chunk::MAX_PAYLOAD], 1).unwrap();
        }
        let saver = Some(Position {
            block_sequence: 2,
            offset: 0,
        });
        assert_eq!(eraser.erase_step(saver, 0), Ok(Erasure::Idle));
        flash.cut_power(0, torn);
        assert_eq!(eraser.erase_step(saver, 100), Err(Error::Flash(MemoryError::PowerCut)));
        let position = ring.position();
        drop((ring, eraser));
        let image = flash.image();
        let header_gone = torn >= HEADER_BYTES;
        let expected = if header_gone { Slot::Erased } else { Slot::Foreign };
        let rescanned = bring_up(&MemoryFlash::from_image(image.clone()), geometry, Hints::default());
        assert_eq!(rescanned.table.slot(0), expected, "torn {torn}");
        assert_eq!(rescanned.next_sequence, 4);
        let hints = Hints {
            writer: Some(position),
            saver: None,
        };
        reboot(&image, hints, &format!("torn erase, {torn} bytes"));

        // the writer reaches slot 0 last: it refuses the dirty bytes, or the eraser took the foreign slot first
        let flash = MemoryFlash::from_image(image);
        let scanned = bring_up(&flash, geometry, hints);
        let mut ring = writer(&flash, &scanned, 2);
        let mut eraser = common::eraser(&flash, &scanned.table);
        for _ in 0..4 {
            ring.append(&[b'b'; chunk::MAX_PAYLOAD], 1).unwrap(); // blocks 4..7 in slots 4..7
        }
        assert_eq!(
            ring.append(b"x", 1),
            Err(Error::WouldBlock),
            "torn {torn}: slot 0 is not clean"
        );
        assert_eq!(scanned.table.slot(0), Slot::Foreign, "torn {torn}");
        assert_eq!(
            eraser.erase_step(None, 0),
            Ok(Erasure::Foreign { slot: 0 }),
            "no loss for it"
        );
        assert_eq!(
            ring.append(b"x", 1)
                .map(|at| slot_of(&scanned.table, at.block_sequence)),
            Ok(0)
        );
    }
}

#[test]
fn a_valid_header_over_damaged_chunks_keeps_its_whole_chunks_and_is_erased_again() {
    // an erase cut so that the header still reads but bits went back to 1 in the middle of the block
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut index = 0;
    while ring.position()
        < (Position {
            block_sequence: 2,
            offset: 0,
        })
    {
        ring.append(&payload(1, index), records(index)).unwrap();
        index += 1;
    }
    drop(ring);
    let mut image = flash.into_image();
    let damaged = 1000..1500; // in block 1 (slot 1)
    for byte in &mut image[BLOCK_BYTES as usize..][damaged.clone()] {
        *byte |= 0xF0;
    }
    let block = blocks(&image, geometry)
        .into_iter()
        .find(|block| block.slot == 1)
        .unwrap();
    assert_eq!(block.torn.len(), 1, "one damaged stretch");
    assert!(
        (block.torn[0].offset as usize) < damaged.end,
        "it starts where the damage does"
    );
    for found in &block.chunks {
        let end = found.offset as usize + chunk::CHUNK_HEADER_BYTES + found.payload.len();
        let clear = end <= damaged.start || found.offset as usize >= damaged.end;
        assert!(
            clear,
            "only undamaged chunks decode, never a wrong record: {}",
            found.offset
        );
    }
    assert!(
        block.chunks.iter().any(|found| found.offset as usize >= damaged.end),
        "the walk finds its way past the damage"
    );

    let flash = MemoryFlash::from_image(image);
    let scanned = bring_up(&flash, geometry, Hints::default());
    assert_eq!(
        scanned.table.slot(1),
        Slot::Valid {
            block_sequence: 1,
            flight: false
        },
        "the header still reads"
    );
    let mut eraser = eraser(&flash, &scanned.table);
    let saver = Some(Position {
        block_sequence: 2,
        offset: 0,
    });
    assert_eq!(eraser.erase_step(saver, 0), Ok(Erasure::Idle));
    assert_eq!(
        eraser.erase_step(saver, 100),
        Ok(Erasure::Saved {
            slot: 0,
            block_sequence: 0
        })
    );
    assert_eq!(
        eraser.erase_step(saver, 100),
        Ok(Erasure::Saved {
            slot: 1,
            block_sequence: 1
        }),
        "erased again"
    );
    assert!(slot_bytes(&flash.image(), 1).iter().all(|&byte| byte == 0xFF));
}
