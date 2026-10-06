//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring's behaviour over an in-memory flash: append and read back, block rollover, the board's 6144-slot wrap,
//! reboots, would-block, the eraser's normal rule and its delay, newest wins with a dead saver, the reserve and its
//! margin, foreign and other-version blocks, the flight flag and a reboot in flight, failed programs, a sector that
//! never erases and a slot that never reads, the hints and their reach, a trusted saver hint, the end of the
//! numbering and the refusals. Power cuts are in power_cut.rs, the two tasks on two threads in concurrent.rs.

mod common;

use std::collections::HashMap;

use common::*;
use flash_ring::chunk::{self, ALIGNMENT, CHUNK_HEADER_BYTES, MAX_PAYLOAD, Torn, Walked, chunk_bytes};
use flash_ring::header::{BlockHeader, HEADER_BYTES, VERSION};
use flash_ring::{
    BLOCK_BYTES, BlockTable, ERASE_ATTEMPTS, Eraser, Erasure, Error, Figures, Flash, Geometry, HINT_REACH, Hints, Loss,
    MemoryError, MemoryFlash, Position, Ring, Slot, scan,
};

/// A position, briefly.
fn at(block_sequence: u32, offset: u32) -> Position {
    Position { block_sequence, offset }
}

/// A valid slot, briefly.
fn valid(block_sequence: u32, flight: bool) -> Slot {
    Slot::Valid { block_sequence, flight }
}

/// The header boot `boot` gives block `block_sequence`.
fn header(block_sequence: u32, boot: u32, flight: bool) -> BlockHeader {
    BlockHeader {
        version: VERSION,
        flight,
        block_sequence,
        boot_id: boot,
        session: session(boot),
    }
}

#[test]
fn round_trips_chunks_in_order() {
    let geometry = Geometry::new(16).unwrap();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    assert_eq!(scanned.next_sequence, 0, "a fresh ring starts at block 0");
    let mut ring = writer(&flash, &scanned, 7);
    assert_eq!(ring.position(), at(0, 0), "nothing readable yet");
    let starts: Vec<Position> = (0..60)
        .map(|index| ring.append(&payload(7, index), records(index)).unwrap())
        .collect();
    let image = flash.image();
    let found = walk(&image, geometry);
    assert_eq!(data(&found), (0..60).map(|index| (7, index)).collect::<Vec<_>>());
    for (index, (found, start)) in (0..).zip(found.iter().zip(&starts)) {
        assert_eq!(at(found.header.block_sequence, found.offset), *start);
        assert_eq!(found.offset as usize % ALIGNMENT, 0);
        assert_eq!(found.records, records(index));
        assert_eq!(
            found.slot, found.header.block_sequence,
            "block k in slot k on a fresh ring"
        );
        assert_eq!(found.header, header(found.header.block_sequence, 7, false));
    }
    let blocks = blocks(&image, geometry);
    assert!(blocks.len() > 5, "60 chunks fill several blocks");
    for block in &blocks {
        assert!(block.torn.is_empty());
        let bytes = slot_bytes(&image, block.slot);
        let mut covered = HEADER_BYTES;
        for found in &block.chunks {
            let padding = &bytes[covered..found.offset as usize];
            assert!(padding.iter().all(|&byte| byte == 0xFF), "padding is never programmed");
            covered = found.offset as usize + CHUNK_HEADER_BYTES + found.payload.len();
        }
        assert!(
            bytes[covered..].iter().all(|&byte| byte == 0xFF),
            "the tail stays erased"
        );
    }
    let last = found.last().unwrap();
    let end = last.offset + chunk_bytes(last.payload.len()) as u32;
    assert_eq!(
        ring.position(),
        at(last.header.block_sequence, end),
        "the end of the last chunk"
    );
    assert_eq!(
        flash.programs() as usize,
        60 + blocks.len(),
        "one program per chunk, and one per block header"
    );
}

#[test]
fn a_chunk_that_does_not_fit_opens_the_next_block_header_first() {
    let geometry = Geometry::new(4).unwrap();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    assert_eq!(
        ring.room(),
        MAX_PAYLOAD,
        "no block open: a new one takes the largest chunk"
    );
    assert_eq!(ring.append(&[b'a'; 3000], 1), Ok(at(0, 56)));
    let second = (HEADER_BYTES + chunk_bytes(3000)) as u32;
    let room = (BLOCK_BYTES - second) as usize - CHUNK_HEADER_BYTES;
    assert_eq!(ring.room(), room);
    assert_eq!(ring.append(&vec![b'b'; room], 2), Ok(at(0, second)));
    assert_eq!(ring.position(), at(0, BLOCK_BYTES), "filled to its last byte");
    assert_eq!(
        ring.room(),
        MAX_PAYLOAD,
        "no room for a chunk header: the next append opens a block"
    );
    let full = slot_bytes(&flash.image(), 0).to_vec();
    let programs = flash.programs();
    assert_eq!(ring.append(b"c", 3), Ok(at(1, 56)));
    assert_eq!(flash.programs(), programs + 2, "the header, then the chunk");
    assert_eq!(slot_bytes(&flash.image(), 0), full, "the full block is left as it was");
    let room = ring.room();
    assert_eq!(room, BLOCK_BYTES as usize - 72 - CHUNK_HEADER_BYTES);
    assert_eq!(
        ring.append(&vec![b'd'; room + 1], 4),
        Ok(at(2, 56)),
        "past the room: a new block"
    );
    assert_eq!(
        snapshot(&scanned.table),
        [
            valid(0, false),
            valid(1, false),
            Slot::Open {
                block_sequence: 2,
                flight: false
            },
            Slot::Erased
        ],
        "a block closes when a chunk does not fit, with room left or none"
    );
    let image = flash.image();
    let counts: Vec<Vec<u16>> = blocks(&image, geometry)
        .iter()
        .map(|block| block.chunks.iter().map(|found| found.records).collect())
        .collect();
    assert_eq!(counts, [vec![1, 2], vec![3], vec![4]]);
}

#[test]
fn wraps_the_6144_slots_of_the_board_behind_a_saver_that_keeps_up() {
    let geometry = Geometry::BOARD;
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    assert_eq!(scanned.table.figures(None).erased_ahead, 6144);
    let mut ring = writer(&flash, &scanned, 9);
    let mut eraser = eraser(&flash, &scanned.table);
    let (mut index, mut now_ms, mut saved) = (0u32, 0u32, 0u32);
    while ring.position().block_sequence < 6144 + 64 {
        let mut chunk = format!("9:{index}:").into_bytes();
        chunk.resize(1990, b'r'); // two chunks a block
        ring.append(&chunk, 1)
            .expect("the saver keeps up, so a block is always erased");
        index += 1;
        now_ms += 10;
        let saver = at(ring.position().block_sequence, 0);
        match eraser.erase_step(Some(saver), now_ms).unwrap() {
            Erasure::Saved { .. } => saved += 1,
            Erasure::Idle => {}
            other => panic!("{other:?}: a saver that keeps up leaves only the normal rule"),
        }
    }
    let newest = ring.position().block_sequence;
    assert_eq!(
        slot_of(&scanned.table, newest),
        newest % 6144,
        "block k in slot k mod 6144"
    );
    assert!(saved > 6144, "every slot was erased behind the saver at least once");
    assert_eq!(scanned.table.figures(None).lost, 0);
    drop((ring, eraser));

    let image = flash.into_image();
    let kept = blocks(&image, geometry);
    let last = kept.last().unwrap();
    assert_eq!(last.header.block_sequence, newest);
    assert_eq!(
        data(&last.chunks),
        [(9, index - 1)],
        "the loop stopped at the newest block's first chunk"
    );
    assert_eq!(data(&kept[kept.len() - 2].chunks), [(9, index - 3), (9, index - 2)]);
    let flash = MemoryFlash::from_image(image);
    let rescanned = bring_up(&flash, geometry, Hints::default());
    assert_eq!(rescanned.next_sequence, newest + 1);
    let mut ring = writer(&flash, &rescanned, 10);
    assert_eq!(ring.append(b"10:0:after the reboot", 1), Ok(at(newest + 1, 56)));
    assert_eq!(
        slot_of(&rescanned.table, newest + 1),
        (newest + 1) % 6144,
        "the slot after the newest"
    );
}

#[test]
fn a_reboot_opens_a_new_block_and_never_appends_to_an_old_one() {
    let geometry = small();
    let flash = erased(geometry);
    let first = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &first, 1);
    let mut index = 0;
    while ring.position() < at(2, 2000) {
        ring.append(&payload(1, index), records(index)).unwrap();
        index += 1;
    }
    let position = ring.position();
    drop(ring);
    let image = flash.image();

    let flash = MemoryFlash::from_image(image.clone());
    let hints = Hints {
        writer: Some(position),
        saver: None,
    };
    let second = bring_up(&flash, geometry, hints);
    assert_eq!(second.next_sequence, 3);
    assert_eq!(
        snapshot(&second.table)[..4],
        [valid(0, false), valid(1, false), valid(2, false), Slot::Erased]
    );
    let mut ring = writer(&flash, &second, 2);
    assert_eq!(ring.position(), at(3, 0), "every old block is readable whole");
    assert_eq!(
        ring.append(b"2:0:", 1),
        Ok(at(3, 56)),
        "block 2 had room, and is left closed"
    );
    let after = flash.image();
    for slot in 0..3 {
        assert_eq!(
            slot_bytes(&after, slot),
            slot_bytes(&image, slot),
            "old block {slot} untouched"
        );
    }
    let opened = &blocks(&after, geometry)[3];
    assert_eq!((opened.slot, opened.header), (3, header(3, 2, false)));
    assert_eq!(data(&opened.chunks), [(2, 0)]);
}

#[test]
fn a_reboot_searches_past_kept_blocks_for_an_erased_slot() {
    // boot 1 fills the 8 slots, block 0 in flight; newest wins then takes block 1, the oldest non-flight one
    let geometry = small();
    let flash = erased(geometry);
    let first = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &first, 1);
    let mut eraser = eraser(&flash, &first.table);
    ring.set_flight(true);
    ring.append(&[b'f'; MAX_PAYLOAD], 1).unwrap();
    ring.set_flight(false);
    for _ in 1..8 {
        ring.append(&[b'g'; MAX_PAYLOAD], 1).unwrap();
    }
    assert_eq!(ring.append(b"x", 1), Err(Error::WouldBlock));
    let Ok(Erasure::Lost(loss)) = eraser.erase_step(None, 0) else {
        panic!("a full ring with a dead saver loses a block");
    };
    assert_eq!((loss.slot, loss.block_sequence, loss.flight), (1, 1, false));
    drop((ring, eraser));

    let flash = MemoryFlash::from_image(flash.into_image());
    let second = bring_up(&flash, geometry, Hints::default());
    assert_eq!(
        second.table.slot(0),
        valid(0, true),
        "the slot after the newest holds a kept flight block"
    );
    let mut ring = writer(&flash, &second, 2);
    assert_eq!(ring.append(b"2:0:", 1), Ok(at(8, 56)));
    assert_eq!(slot_of(&second.table, 8), 1, "the first erased slot after the newest");
}

#[test]
fn appends_would_block_without_an_erased_slot_and_resume_once_one_is_freed() {
    let geometry = Geometry::new(4).unwrap();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    for _ in 0..4 {
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap();
    }
    let (position, programs) = (ring.position(), flash.programs());
    for _ in 0..3 {
        assert_eq!(ring.append(b"waits in PSRAM", 1), Err(Error::WouldBlock));
    }
    assert_eq!(
        (ring.position(), flash.programs()),
        (position, programs),
        "nothing programmed, nothing published"
    );
    assert_eq!(scanned.table.slot(3), valid(3, false), "the full block closed");
    assert_eq!(ring.room(), MAX_PAYLOAD);
    let mut eraser = eraser(&flash, &scanned.table);
    let Ok(Erasure::Lost(loss)) = eraser.erase_step(None, 0) else {
        panic!("newest wins frees a slot");
    };
    assert_eq!((loss.slot, loss.block_sequence), (0, 0));
    assert_eq!(ring.append(b"waits in PSRAM", 1), Ok(at(4, 56)));
    assert_eq!(slot_of(&scanned.table, 4), 0, "the ring wrapped into the freed slot");
}

#[test]
fn erases_behind_the_saver_no_sooner_than_100_ms_and_never_the_open_block() {
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, &scanned.table);
    for _ in 0..3 {
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap();
    }
    ring.append(b"x", 1).unwrap(); // block 3, open
    let saver = Some(at(2, 0));
    let figures = Figures {
        erased_ahead: 4,
        saved: 2,
        unsaved: 1,
        ..Figures::default()
    };
    assert_eq!(scanned.table.figures(saver), figures, "the open block is in no count");
    for now_ms in [1000, 1050, 1099] {
        assert_eq!(
            eraser.erase_step(saver, now_ms),
            Ok(Erasure::Idle),
            "{now_ms} ms: too soon"
        );
    }
    let saved = |slot, block_sequence| Ok(Erasure::Saved { slot, block_sequence });
    assert_eq!(eraser.erase_step(saver, 1100), saved(0, 0), "oldest first");
    assert_eq!(eraser.erase_step(saver, 1100), saved(1, 1));
    assert_eq!(
        eraser.erase_step(saver, 1101),
        Ok(Erasure::Idle),
        "block 2 is only where the saver is"
    );

    let past = Some(at(10, 0)); // past the open block: the guard, not a real saver
    assert_eq!(eraser.erase_step(past, 2000), Ok(Erasure::Idle));
    assert_eq!(eraser.erase_step(past, 2100), saved(2, 2));
    assert_eq!(eraser.erase_step(past, 2200), Ok(Erasure::Idle), "block 3 is open");
    ring.set_flight(true); // closes it
    assert_eq!(eraser.erase_step(past, 2201), saved(3, 3));
    assert_eq!(
        eraser.erase_step(None, 2202),
        Ok(Erasure::Idle),
        "nothing left, and the reserve holds"
    );
}

#[test]
fn a_saver_that_moves_on_every_step_has_its_blocks_erased_100_to_200_ms_after_it() {
    let geometry = Geometry::new(64).unwrap();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, &scanned.table);
    let mut passed_ms: HashMap<u32, u32> = HashMap::new();
    let mut waits = Vec::new();
    for tick in 0..600 {
        let now_ms = tick * 10; // the eraser steps every 10 ms
        if tick % 3 == 0 {
            ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap(); // a block every 30 ms
        }
        let saver = at(ring.position().block_sequence, 0); // the saver passes each block as it closes
        for block_sequence in 0..saver.block_sequence {
            passed_ms.entry(block_sequence).or_insert(now_ms);
        }
        if let Erasure::Saved { block_sequence, .. } = eraser.erase_step(Some(saver), now_ms).unwrap() {
            waits.push(now_ms - passed_ms[&block_sequence]);
        }
    }
    assert!(waits.len() > 150, "{} erases", waits.len());
    // 100 ms of delay, at most another 100 ms for the position before it to settle, and a step per block queued
    assert!(waits.iter().all(|&wait| (100..=220).contains(&wait)), "{waits:?}");
    assert!(
        waits.iter().any(|&wait| wait > 150),
        "a moving saver waits out the one before"
    );
}

/// One block-sized chunk, then one eraser step with no saver. A loss is checked against the newest-wins rule on
/// the table as it was before the step: the oldest non-flight block, else the oldest flight one, and only once no
/// more than the reserve is free.
fn block_then_step(
    ring: &mut Ring<'_, &MemoryFlash>,
    eraser: &mut Eraser<'_, &MemoryFlash>,
    table: &BlockTable,
) -> Erasure {
    ring.append(&[b'n'; MAX_PAYLOAD], 1).unwrap();
    let before = snapshot(table);
    let erasure = eraser.erase_step(None, 0).unwrap();
    let free = before.iter().filter(|slot| **slot == Slot::Erased).count() as u32;
    if let Erasure::Lost(loss) = erasure {
        let candidates: Vec<(u32, bool)> = before
            .iter()
            .filter_map(|slot| match *slot {
                Slot::Valid { block_sequence, flight } => Some((block_sequence, flight)),
                _ => None,
            })
            .collect();
        let non_flight = candidates.iter().filter(|(_, flight)| !flight).min();
        let expected = non_flight.or(candidates.iter().min()).copied();
        assert_eq!(Some((loss.block_sequence, loss.flight)), expected);
        assert!(
            free <= table.geometry().reserve_blocks(),
            "newest wins with {free} free"
        );
        let header = header(loss.block_sequence, 1, loss.flight);
        assert_eq!(loss.header, Some(header), "the loss line has its boot and session");
    } else {
        assert!(free > table.geometry().reserve_blocks(), "idle with {free} free");
    }
    erasure
}

#[test]
fn newest_wins_with_a_dead_saver_gives_up_flight_blocks_last() {
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let table = &scanned.table;
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, table);
    let mut erasures = Vec::new();
    for _ in 0..2 {
        erasures.push(block_then_step(&mut ring, &mut eraser, table)); // pad blocks 0 and 1
    }
    ring.set_flight(true);
    for _ in 0..3 {
        erasures.push(block_then_step(&mut ring, &mut eraser, table)); // flight blocks 2, 3, 4
    }
    ring.set_flight(false);
    for _ in 0..12 {
        erasures.push(block_then_step(&mut ring, &mut eraser, table)); // pad blocks 5..16, at the reserve from 6 on
    }
    assert_eq!(
        lost(&erasures),
        [0, 1, 5, 6, 7, 8, 9, 10, 11, 12, 13],
        "the pad goes, the flight stays"
    );
    for block_sequence in 2..5 {
        assert_eq!(table.slot(slot_of(table, block_sequence)), valid(block_sequence, true));
    }
    // a long flight: once no pad block is left, the oldest flight blocks go
    ring.set_flight(true);
    erasures.clear();
    for _ in 0..8 {
        erasures.push(block_then_step(&mut ring, &mut eraser, table)); // flight blocks 17..24
    }
    assert_eq!(lost(&erasures), [14, 15, 16, 2, 3, 4, 17, 18]);
    assert_eq!(table.figures(None).lost, 19, "every loss is counted");
}

#[test]
fn the_reserve_holds_between_the_writers_open_and_the_erasers_step() {
    // the steady state of newest wins (no card): the writer runs above the eraser and opens a block before the
    // eraser's next step, so `ring-low` and the arm gate are sampled there too; they must never see the ring short
    let geometry = Geometry::new(300).unwrap();
    let reserve = geometry.reserve_blocks();
    assert_eq!(reserve, 3);
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, &scanned.table);
    let mut opened = 0;
    for index in 0..4000 {
        let before = ring.position().block_sequence;
        ring.append(&payload(1, index), records(index))
            .expect("the eraser keeps a block ready: the writer never blocks");
        opened += u32::from(ring.position().block_sequence != before);
        let after_open = scanned.table.figures(None);
        assert!(
            after_open.erased_ahead >= reserve,
            "chunk {index}, before the eraser's step: {after_open:?}"
        );
        eraser.erase_step(None, index * 10).unwrap();
        let after_step = scanned.table.figures(None);
        assert!(after_step.erased_ahead > reserve, "chunk {index}: {after_step:?}");
    }
    let figures = scanned.table.figures(None);
    assert!(
        figures.lost > 100 && opened > geometry.blocks(),
        "the workload wrapped the ring into newest wins: {figures:?}, {opened} opened"
    );
    assert_eq!(
        figures.erased_ahead,
        reserve + 1,
        "the reserve and the margin for one open, no more"
    );
}

#[test]
fn an_unformatted_ring_is_formatted_ahead_of_the_writer_without_a_loss() {
    let geometry = small();
    let flash = MemoryFlash::from_image(noise(23, geometry.bytes() as usize));
    let scanned = bring_up(&flash, geometry, Hints::default());
    assert_eq!(scanned.next_sequence, 0);
    assert!(scanned.unformatted, "no block anywhere");
    let foreign = Figures {
        foreign: 8,
        ..Figures::default()
    };
    assert_eq!(
        scanned.table.figures(None),
        foreign,
        "no block anywhere: ring-unformatted"
    );
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, &scanned.table);
    assert_eq!(
        ring.append(b"1:0:", 1),
        Err(Error::WouldBlock),
        "until a slot is formatted"
    );
    for slot in 0..8 {
        let formatted = eraser.erase_step(None, 0);
        assert_eq!(
            formatted,
            Ok(Erasure::Foreign { slot }),
            "in the order the writer takes them"
        );
        if slot == 0 {
            assert_eq!(ring.append(b"1:0:", 1), Ok(at(0, 56)), "the writer starts at once");
        }
    }
    assert_eq!(eraser.erase_step(None, 0), Ok(Erasure::Idle));
    let formatted = Figures {
        erased_ahead: 7,
        ..Figures::default()
    };
    assert_eq!(
        scanned.table.figures(None),
        formatted,
        "no loss: foreign bytes hold nothing"
    );
    assert!(scanned.unformatted, "the scan remembers what the table no longer shows");
}

#[test]
fn a_ring_with_any_block_is_formatted_and_an_erased_one_is_not() {
    let geometry = small();
    let fresh = bring_up(&erased(geometry), geometry, Hints::default());
    assert!(
        fresh.unformatted,
        "all 0xFF holds no block either, and no foreign slot to format"
    );
    assert_eq!(fresh.table.figures(None).foreign, 0, "so `ring-unformatted` stays off");
    // one block of this layout, or of another, among foreign slots: formatted, whatever else is there
    for version in [VERSION, 2] {
        let mut image = noise(31, geometry.bytes() as usize);
        let header = BlockHeader {
            version,
            ..header(4, 1, false)
        };
        image[3 * BLOCK_BYTES as usize..][..HEADER_BYTES].copy_from_slice(&header.encode());
        let scanned = bring_up(&MemoryFlash::from_image(image), geometry, Hints::default());
        assert!(!scanned.unformatted, "version {version}");
        assert_eq!(scanned.table.figures(None).foreign, 7);
    }
}

#[test]
fn a_foreign_slot_goes_before_saved_blocks_and_before_newest_wins() {
    let geometry = Geometry::new(4).unwrap();
    let flash = erased(geometry);
    let first = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &first, 1);
    for _ in 0..4 {
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap();
    }
    drop(ring);
    let mut image = flash.into_image();
    image[BLOCK_BYTES as usize..][..16].copy_from_slice(&noise(5, 16)); // slot 1: a bench pattern
    let flash = MemoryFlash::from_image(image);
    let second = bring_up(&flash, geometry, Hints::default());
    assert_eq!(second.table.slot(1), Slot::Foreign);
    let mut eraser = eraser(&flash, &second.table);
    let saver = Some(at(3, 0));
    assert_eq!(
        eraser.erase_step(saver, 0),
        Ok(Erasure::Foreign { slot: 1 }),
        "full, but no loss needed"
    );
    assert_eq!(
        eraser.erase_step(saver, 100),
        Ok(Erasure::Saved {
            slot: 0,
            block_sequence: 0
        })
    );
}

#[test]
fn blocks_of_another_layout_are_kept_ordered_and_given_up_last_as_their_flag_says() {
    // a newer build left blocks 12 (flight) and 13 beside this build's 10 and 11, then this build booted
    let geometry = small();
    let mut image = vec![0xFF; geometry.bytes() as usize];
    for (slot, block_sequence, version, flight) in [
        (0, 10, 1, false),
        (1, 11, 1, false),
        (2, 12, 2, true),
        (3, 13, 2, false),
    ] {
        let start = slot * BLOCK_BYTES as usize;
        let header = BlockHeader {
            version,
            ..header(block_sequence, 4, flight)
        };
        image[start..start + HEADER_BYTES].copy_from_slice(&header.encode());
        image[start + HEADER_BYTES..start + HEADER_BYTES + 16].fill(0x5A); // chunks this build cannot read
    }
    let flash = MemoryFlash::from_image(image);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let other = |block_sequence, flight| Slot::OtherVersion {
        block_sequence,
        flight,
        version: 2,
    };
    assert_eq!(
        snapshot(&scanned.table)[..4],
        [valid(10, false), valid(11, false), other(12, true), other(13, false)]
    );
    assert_eq!(scanned.next_sequence, 14, "numbered past the other build's blocks too");
    assert_eq!(
        scanned.table.saver_start(None),
        at(10, 0),
        "the saver starts at what it can read"
    );
    let figures = Figures {
        erased_ahead: 4,
        unsaved: 2,
        other_version: 2,
        ..Figures::default()
    };
    assert_eq!(scanned.table.figures(None), figures);

    // the saver passes all four (it skips what it cannot read): the normal rule erases this build's only
    let mut eraser = eraser(&flash, &scanned.table);
    let saver = Some(at(14, 0));
    let erasures: Vec<Erasure> = [0, 100, 101, 102]
        .into_iter()
        .map(|now_ms| eraser.erase_step(saver, now_ms).unwrap())
        .collect();
    let saved = |slot, block_sequence| Erasure::Saved { slot, block_sequence };
    assert_eq!(erasures, [Erasure::Idle, saved(0, 10), saved(1, 11), Erasure::Idle]);

    // the card dies; the ring fills: the other build's pad block goes first, its flight block only after the pad
    let mut ring = writer(&flash, &scanned, 5);
    let mut erasures = Vec::new();
    for _ in 0..8 {
        ring.append(&[b'p'; MAX_PAYLOAD], 1).unwrap(); // blocks 14..21
        erasures.push(eraser.erase_step(saver, 200).unwrap());
    }
    assert_eq!(lost(&erasures), [13, 14, 15, 16]);
    let Erasure::Lost(loss) = erasures
        .iter()
        .find(|erasure| matches!(erasure, Erasure::Lost(_)))
        .unwrap()
    else {
        unreachable!()
    };
    assert_eq!(
        loss.header.map(|header| header.version),
        Some(2),
        "the loss line still names its boot"
    );
    ring.set_flight(true);
    erasures.clear();
    for _ in 0..8 {
        ring.append(&[b'f'; MAX_PAYLOAD], 1).unwrap(); // flight blocks 22..29
        erasures.push(eraser.erase_step(saver, 200).unwrap());
    }
    assert_eq!(
        lost(&erasures),
        [17, 18, 19, 20, 21, 12, 22, 23],
        "pad first, then the oldest flight block"
    );
}

#[test]
fn a_flight_change_closes_the_block_and_marks_the_next() {
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let table = &scanned.table;
    let mut ring = writer(&flash, &scanned, 1);
    assert_eq!(ring.append(b"pad", 1), Ok(at(0, 56)));
    assert!(!ring.flight());
    ring.set_flight(true); // BOOSTING
    assert!(ring.flight());
    assert_eq!(table.slot(0), valid(0, false), "closed at the change");
    assert_eq!(ring.room(), MAX_PAYLOAD);
    assert_eq!(ring.append(b"boost", 1), Ok(at(1, 56)));
    ring.set_flight(true); // no change: the block stays open
    assert_eq!(ring.append(b"glide", 1), Ok(at(1, 72)));
    assert_eq!(
        table.slot(1),
        Slot::Open {
            block_sequence: 1,
            flight: true
        }
    );
    ring.set_flight(false);
    assert_eq!(ring.append(b"beacon", 1), Ok(at(2, 56)));
    let image = flash.image();
    let flags: Vec<bool> = blocks(&image, geometry)
        .iter()
        .map(|block| block.header.flight)
        .collect();
    assert_eq!(flags, [false, true, false]);
}

#[test]
fn a_failed_chunk_program_is_retried_at_the_next_offset() {
    let geometry = small();
    let retry = vec![b'r'; 100];
    let next = 72 + chunk_bytes(100) as u32;
    for torn in [0, 1, 5, 8, 40, usize::MAX] {
        let flash = erased(geometry);
        let scanned = bring_up(&flash, geometry, Hints::default());
        let mut ring = writer(&flash, &scanned, 1);
        assert_eq!(ring.append(b"before", 1), Ok(at(0, 56)));
        flash.fail(0, torn);
        assert_eq!(
            ring.append(&retry, 4),
            Err(Error::Flash(MemoryError::Failed)),
            "torn {torn}"
        );
        assert_eq!(
            ring.position(),
            at(0, 72),
            "torn {torn}: a failed chunk is not published"
        );
        assert_eq!(
            ring.append(&retry, 4),
            Ok(at(0, next)),
            "torn {torn}: behind whatever landed"
        );
        let image = flash.image();
        let walked: Vec<Walked<'_>> = chunk::chunks(slot_bytes(&image, 0)).collect();
        let whole = |offset| {
            Walked::Chunk(chunk::Chunk {
                offset,
                records: 4,
                payload: &retry,
            })
        };
        // the debris is followed by the retry: no loss line for it
        let expected = match torn {
            0 => vec![whole(next)], // nothing landed: an erased gap
            1 => vec![
                Walked::Torn(Torn {
                    offset: 72,
                    records: None,
                    followed: true,
                }),
                whole(next),
            ],
            usize::MAX => vec![whole(72), whole(next)], // it landed whole: a duplicate, for the tools to drop
            _ => vec![
                Walked::Torn(Torn {
                    offset: 72,
                    records: Some(4),
                    followed: true,
                }),
                whole(next),
            ],
        };
        assert_eq!(walked[1..], expected, "torn {torn}");
    }
}

#[test]
fn a_failed_program_whose_retry_does_not_fit_is_retried_in_the_next_block_of_the_boot() {
    // the torn chunk is its block's last, as after a cut, but the boot's next block holds the retry: the saver
    // tells the two apart by that block (see chunk::Torn)
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let first = 3000;
    assert_eq!(ring.append(&vec![b'a'; first], 1), Ok(at(0, 56)));
    let retry = vec![b'r'; ring.room()]; // fits once, not twice
    let end = (HEADER_BYTES + chunk_bytes(first)) as u32;
    flash.fail(0, 20);
    assert_eq!(ring.append(&retry, 6), Err(Error::Flash(MemoryError::Failed)));
    assert_eq!(ring.append(&retry, 6), Ok(at(1, 56)), "no room behind the debris");
    let image = flash.image();
    let blocks = blocks(&image, geometry);
    let torn = Torn {
        offset: end,
        records: Some(6),
        followed: false,
    };
    assert_eq!((blocks[0].torn.as_slice(), blocks[0].chunks.len()), (&[torn][..], 1));
    assert_eq!(
        (blocks[1].header.boot_id, blocks[1].chunks[0].records),
        (blocks[0].header.boot_id, 6),
        "the same boot's next block opens with the retried records"
    );
}

#[test]
fn a_failed_block_header_spends_its_sequence_and_leaves_the_slot_to_the_eraser() {
    let geometry = small();
    for torn in [0, 30, usize::MAX] {
        let flash = erased(geometry);
        let scanned = bring_up(&flash, geometry, Hints::default());
        let mut ring = writer(&flash, &scanned, 1);
        flash.fail(0, torn); // block 0's header
        assert_eq!(ring.append(b"x", 1), Err(Error::Flash(MemoryError::Failed)));
        assert_eq!(scanned.table.slot(0), Slot::Foreign, "torn {torn}");
        assert_eq!(
            ring.append(b"x", 1),
            Ok(at(1, 56)),
            "torn {torn}: the next slot, the next number"
        );
        assert_eq!(slot_of(&scanned.table, 1), 1);
        // a reboot now sees block 0 empty (its header landed whole) or foreign: never two blocks numbered alike
        let rescanned = bring_up(&MemoryFlash::from_image(flash.image()), geometry, Hints::default());
        let expected = match torn {
            0 => Slot::Erased, // nothing landed
            usize::MAX => valid(0, false),
            _ => Slot::Foreign,
        };
        assert_eq!(rescanned.table.slot(0), expected, "torn {torn}");
        assert_eq!(rescanned.next_sequence, 2);
        let mut eraser = eraser(&flash, &scanned.table);
        assert_eq!(eraser.erase_step(None, 0), Ok(Erasure::Foreign { slot: 0 }));
    }
}

#[test]
fn a_slot_with_an_erased_header_over_dirty_bytes_is_refused_and_erased() {
    let geometry = small();
    let mut image = vec![0xFF; geometry.bytes() as usize];
    image[BLOCK_BYTES as usize + 1000] = 0x00; // slot 1: an erase cut after its header went
    let flash = MemoryFlash::from_image(image);
    let scanned = bring_up(&flash, geometry, Hints::default());
    assert_eq!(scanned.table.slot(1), Slot::Erased, "the header alone reads erased");
    let mut ring = writer(&flash, &scanned, 1);
    assert_eq!(ring.append(&[b'a'; MAX_PAYLOAD], 1), Ok(at(0, 56)));
    let reads = flash.reads();
    assert_eq!(ring.append(b"x", 1), Ok(at(1, 56)));
    assert_eq!(
        flash.reads(),
        reads + 2,
        "one full read refused slot 1, one passed slot 2"
    );
    assert_eq!(slot_of(&scanned.table, 1), 2);
    assert_eq!(scanned.table.slot(1), Slot::Foreign);
    let mut eraser = eraser(&flash, &scanned.table);
    assert_eq!(eraser.erase_step(None, 0), Ok(Erasure::Foreign { slot: 1 }));
    assert!(slot_bytes(&flash.image(), 1).iter().all(|&byte| byte == 0xFF));
}

#[test]
fn the_hints_raise_the_next_sequence_but_the_scan_places_the_writer() {
    let geometry = small();
    // an empty ring: the previous boot's blocks were saved and erased, and only the hints remember their numbers
    let flash = erased(geometry);
    let cases = [
        (None, None, 0),
        (Some(at(500, 1000)), None, 501),
        (Some(at(500, 1000)), Some(at(501, 0)), 502), // a saver at a block not yet opened: a gap is harmless
        (None, Some(at(7, 0)), 8),
        (Some(at(HINT_REACH, 0)), None, HINT_REACH + 1), // as far as a hint is trusted
        (Some(at(HINT_REACH + 1, 0)), Some(at(9, 0)), 10), // corrupt: ignored, the other hint still counts
        (Some(at(u32::MAX, 0)), None, 0),                // would spend every number at once
    ];
    for (writer_hint, saver_hint, next) in cases {
        let hints = Hints {
            writer: writer_hint,
            saver: saver_hint,
        };
        assert_eq!(bring_up(&flash, geometry, hints).next_sequence, next, "{hints:?}");
    }

    let first = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &first, 1);
    for _ in 0..3 {
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap(); // blocks 0, 1, 2
    }
    drop(ring);
    // with blocks 0..2 in flash: a hint near u32::MAX (corrupt, or another build's encoding) is ignored
    for (hint, next) in [
        (at(1, 100), 3),
        (at(40, 8), 41),
        (at(u32::MAX - 1, 0), 3),
        (at(2 + HINT_REACH + 1, 0), 3),
    ] {
        let hints = Hints {
            writer: Some(hint),
            saver: None,
        };
        let rebooted = MemoryFlash::from_image(flash.image());
        let scanned = bring_up(&rebooted, geometry, hints);
        assert_eq!(
            scanned.next_sequence, next,
            "{hint:?}: the newer of the scan and the hint"
        );
        let mut ring = writer(&rebooted, &scanned, 2);
        assert_eq!(ring.append(b"x", 1), Ok(at(next, 56)));
        assert_eq!(
            slot_of(&scanned.table, next),
            3,
            "{hint:?}: the slot after the newest block"
        );
    }
}

#[test]
fn refuses_a_chunk_too_large_and_a_flash_smaller_than_the_ring() {
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let reads = flash.reads();
    assert_eq!(ring.append(&vec![0; MAX_PAYLOAD + 1], 1), Err(Error::TooLarge));
    assert_eq!((flash.programs(), flash.reads()), (0, reads), "no block opened for it");
    assert_eq!(ring.append(&vec![0; MAX_PAYLOAD], 1), Ok(at(0, 56)));
    assert_eq!(ring.position(), at(0, BLOCK_BYTES), "the largest chunk fills a block");
    let short = MemoryFlash::new(geometry.bytes() - BLOCK_BYTES);
    assert!(matches!(scan(&short, geometry, Hints::default()), Err(Error::Geometry)));
    let session = session(1);
    assert!(matches!(
        Ring::new(&short, &scanned.table, 0, 1, session, false),
        Err(Error::Geometry)
    ));
    assert!(matches!(
        Eraser::new(&short, &scanned.table, None),
        Err(Error::Geometry)
    ));
}

#[test]
fn the_eraser_never_takes_the_open_block() {
    let geometry = Geometry::new(2).unwrap();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, &scanned.table);
    ring.append(&[1; MAX_PAYLOAD], 1).unwrap(); // block 0
    assert_eq!(
        eraser.erase_step(None, 0),
        Ok(Erasure::Idle),
        "slot 1 is erased: the reserve holds"
    );
    ring.append(&[2; MAX_PAYLOAD], 1).unwrap(); // block 1 in slot 1, block 0 closed
    let Ok(Erasure::Lost(Loss {
        slot: 0,
        block_sequence: 0,
        ..
    })) = eraser.erase_step(None, 0)
    else {
        panic!("the closed block goes");
    };
    assert_eq!(
        eraser.erase_step(None, 0),
        Ok(Erasure::Idle),
        "slot 1 holds the open block"
    );
    let past = Some(at(9, 0));
    assert_eq!(eraser.erase_step(past, 0), Ok(Erasure::Idle));
    assert_eq!(
        eraser.erase_step(past, 100),
        Ok(Erasure::Idle),
        "saved or not, it is open"
    );
    ring.append(&[3; MAX_PAYLOAD], 1).unwrap(); // block 2 in slot 0, block 1 closed
    let saved = Erasure::Saved {
        slot: 1,
        block_sequence: 1,
    };
    assert_eq!(eraser.erase_step(past, 200), Ok(saved));
}

#[test]
fn after_a_flight_the_open_pad_block_is_kept_and_the_oldest_flight_block_goes() {
    // DONE: every closed block is a flight block, and the only non-flight one is the writer's open block
    let geometry = Geometry::new(4).unwrap();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, &scanned.table);
    ring.set_flight(true);
    for _ in 0..3 {
        ring.append(&[b'f'; MAX_PAYLOAD], 1).unwrap(); // flight blocks 0, 1, 2
    }
    ring.set_flight(false);
    ring.append(b"1:0:beacon", 1).unwrap(); // block 3, open, not flight: the ring is full
    let Ok(Erasure::Lost(loss)) = eraser.erase_step(None, 0) else {
        panic!("a full ring with a dead saver loses a block");
    };
    assert_eq!(
        (loss.block_sequence, loss.flight),
        (0, true),
        "never the open block, pad or not"
    );
    assert_eq!(
        scanned.table.slot(3),
        Slot::Open {
            block_sequence: 3,
            flight: false
        }
    );
}

/// What fails on [`BadSlot`]'s one slot.
#[derive(Clone, Copy, Debug)]
enum Fault {
    /// Every erase: a worn sector, or a driver error that repeats.
    Erase,
    /// Every whole-block read: the writer's check that a slot is erased.
    CheckRead,
}

/// A handle on a flash with one bad slot, where `fault` fails every time, as a driver error that repeats; every
/// other call goes to the flash. The writer or the eraser holds it in place of a plain `&MemoryFlash`.
struct BadSlot<'a> {
    /// The flash.
    flash: &'a MemoryFlash,
    /// The bad slot's offset.
    offset: u32,
    /// What fails there.
    fault: Fault,
}

impl Flash for BadSlot<'_> {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        let check = matches!(self.fault, Fault::CheckRead) && buffer.len() == BLOCK_BYTES as usize;
        if check && offset == self.offset {
            return Err(MemoryError::Failed);
        }
        self.flash.read(offset, buffer)
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), MemoryError> {
        let mut flash = self.flash;
        flash.program(offset, data)
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        if matches!(self.fault, Fault::Erase) && offset == self.offset {
            return Err(MemoryError::Failed);
        }
        let mut flash = self.flash;
        flash.erase_sector(offset)
    }

    fn size(&self) -> u32 {
        self.flash.size()
    }
}

#[test]
fn a_sector_whose_erase_always_fails_is_given_up_and_the_next_oldest_block_goes() {
    // a full ring with a dead saver, and the oldest block in a sector that no longer erases
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let table = &scanned.table;
    let mut ring = writer(&flash, &scanned, 1);
    let worn = BadSlot {
        flash: &flash,
        offset: 0,
        fault: Fault::Erase,
    };
    let mut eraser = Eraser::new(worn, table, None).unwrap();
    for _ in 0..8 {
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap(); // blocks 0..7
    }
    assert_eq!(ring.append(b"x", 1), Err(Error::WouldBlock));
    for attempt in 1..=ERASE_ATTEMPTS {
        assert_eq!(
            eraser.erase_step(None, 0),
            Err(Error::Flash(MemoryError::Failed)),
            "attempt {attempt}: the same block, as the rules choose it"
        );
        assert_eq!(table.slot(0), valid(0, false), "unchanged: still the block it was");
    }
    let figures = table.figures(None);
    assert_eq!((figures.bad, figures.lost), (1, 0), "given up, and no loss counted");
    let Ok(Erasure::Lost(loss)) = eraser.erase_step(None, 0) else {
        panic!("the next-oldest block goes instead");
    };
    assert_eq!((loss.slot, loss.block_sequence), (1, 1));
    assert_eq!(ring.append(b"x", 1), Ok(at(8, 56)), "the writer goes on");
    // the eraser keeps the ring going around the bad slot, which is neither free nor erased again
    for index in 0..40 {
        assert!(
            matches!(eraser.erase_step(None, 0), Ok(Erasure::Lost(_))),
            "block {index}"
        );
        ring.append(&[b'b'; MAX_PAYLOAD], 1)
            .unwrap_or_else(|error| panic!("block {index}: {error:?}"));
    }
    assert_eq!(table.slot(0), valid(0, false), "passed over until the next boot");
    assert_eq!(table.figures(None).bad, 1);
}

#[test]
fn an_erase_that_fails_now_and_then_never_gives_its_slot_up() {
    // a failure, then a success: the count starts again, so failures far apart never add up to a bad slot
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &scanned, 1);
    let mut eraser = eraser(&flash, &scanned.table);
    for _ in 0..8 {
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap();
    }
    for round in 0..2 * ERASE_ATTEMPTS {
        for _ in 1..ERASE_ATTEMPTS {
            flash.fail(0, 0); // the next erase fails, before it erased anything
            assert_eq!(
                eraser.erase_step(None, 0),
                Err(Error::Flash(MemoryError::Failed)),
                "round {round}"
            );
        }
        assert!(
            matches!(eraser.erase_step(None, 0), Ok(Erasure::Lost(_))),
            "round {round}: the last attempt erases it"
        );
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap();
    }
    assert_eq!(scanned.table.figures(None).bad, 0);
}

#[test]
fn a_slot_the_writer_cannot_read_goes_to_the_eraser_and_the_writer_opens_the_next() {
    let geometry = small();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let unreadable = BadSlot {
        flash: &flash,
        offset: BLOCK_BYTES,
        fault: Fault::CheckRead,
    };
    let mut ring = Ring::new(unreadable, &scanned.table, 0, 1, session(1), false).unwrap();
    assert_eq!(ring.append(&[b'a'; MAX_PAYLOAD], 1), Ok(at(0, 56)));
    assert_eq!(
        ring.append(b"x", 1),
        Ok(at(1, 56)),
        "slot 1 fails its read: the next slot opens at once"
    );
    assert_eq!(slot_of(&scanned.table, 1), 2);
    assert_eq!(
        scanned.table.slot(1),
        Slot::Foreign,
        "the eraser's now, never read again by the writer"
    );
    let mut eraser = eraser(&flash, &scanned.table);
    assert_eq!(eraser.erase_step(None, 0), Ok(Erasure::Foreign { slot: 1 }));

    // negative: when no slot can be opened because a read failed, the append says so instead of would-block
    let geometry = Geometry::new(2).unwrap();
    let flash = erased(geometry);
    let scanned = bring_up(&flash, geometry, Hints::default());
    let unreadable = BadSlot {
        flash: &flash,
        offset: BLOCK_BYTES,
        fault: Fault::CheckRead,
    };
    let mut ring = Ring::new(unreadable, &scanned.table, 0, 1, session(1), false).unwrap();
    assert_eq!(ring.append(&[b'a'; MAX_PAYLOAD], 1), Ok(at(0, 56)));
    assert_eq!(
        ring.append(b"x", 1),
        Err(Error::Flash(MemoryError::Failed)),
        "the only erased slot is unreadable"
    );
    assert_eq!(
        ring.append(b"x", 1),
        Err(Error::WouldBlock),
        "until the eraser has erased it again"
    );
}

#[test]
fn a_reboot_in_flight_writes_its_bring_up_into_a_flight_block() {
    // boot 1: two pad blocks, then BOOSTING, two flight blocks, and the power goes (a brown-out, or a panic)
    let geometry = small();
    let flash = erased(geometry);
    let first = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &first, 1);
    for flight in [false, false, true, true] {
        ring.set_flight(flight);
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap(); // blocks 0, 1 (pad), 2, 3 (flight)
    }
    drop(ring);
    let image = flash.into_image();

    // boot 2 reads a crumb in BOOSTING..LANDING: storage comes up as a flight writer, long before the warm start
    let flash = MemoryFlash::from_image(image.clone());
    let second = bring_up(&flash, geometry, Hints::default());
    let table = &second.table;
    let mut ring = Ring::new(&flash, table, second.next_sequence, 2, session(2), true).unwrap();
    assert!(ring.flight());
    let provenance = b"2:0:boot: reset_cause 4";
    assert_eq!(ring.append(provenance, 1), Ok(at(4, 56)));
    ring.set_flight(true); // the warm start resumes the flight: nothing closes
    let behind = (HEADER_BYTES + chunk_bytes(provenance.len())) as u32;
    assert_eq!(ring.append(b"2:1:warm start", 1), Ok(at(4, behind)));
    assert_eq!(
        table.slot(4),
        Slot::Open {
            block_sequence: 4,
            flight: true
        }
    );
    // the card is dead: newest wins gives up boot 1's pad first, then the flight in order, the reboot with it
    let mut eraser = eraser(&flash, table);
    let mut erasures = Vec::new();
    for _ in 0..5 {
        ring.append(&[b'f'; MAX_PAYLOAD], 1).unwrap(); // flight blocks 5..9
        erasures.push(eraser.erase_step(None, 0).unwrap());
    }
    assert_eq!(
        lost(&erasures),
        [0, 1, 2, 3],
        "the reboot's evidence outlives the pad and the flight before it"
    );
    assert_eq!(table.slot(slot_of(table, 4)), valid(4, true));

    // the gate refuses the warm start: the one block written before the decision closes, the next is a pad block
    let flash = MemoryFlash::from_image(image);
    let second = bring_up(&flash, geometry, Hints::default());
    let mut ring = Ring::new(&flash, &second.table, second.next_sequence, 2, session(2), true).unwrap();
    assert_eq!(ring.append(b"2:0:boot: reset_cause 1", 1), Ok(at(4, 56)));
    ring.set_flight(false);
    assert_eq!(ring.append(b"2:1:warm start refused", 1), Ok(at(5, 56)));
    let flags: Vec<bool> = blocks(&flash.image(), geometry)
        .iter()
        .map(|block| block.header.flight)
        .collect();
    assert_eq!(flags, [false, false, true, true, true, false]);
}

#[test]
fn a_ring_numbered_to_the_end_refuses_to_open_rather_than_give_a_number_twice() {
    // a block numbered near u32::MAX, which only another build or a corrupt hint before it can leave
    let geometry = small();
    for (newest, opens) in [(u32::MAX - 3, 2), (u32::MAX - 1, 0), (u32::MAX, 0)] {
        let mut image = vec![0xFF; geometry.bytes() as usize];
        image[..HEADER_BYTES].copy_from_slice(&header(newest, 1, false).encode());
        let flash = MemoryFlash::from_image(image);
        let scanned = bring_up(&flash, geometry, Hints::default());
        assert_eq!(scanned.next_sequence, newest.saturating_add(1), "{newest}");
        let mut ring = writer(&flash, &scanned, 2);
        for open in 0..opens {
            let expected = at(newest + 1 + open, 56);
            assert_eq!(ring.append(&[b'x'; MAX_PAYLOAD], 1), Ok(expected), "{newest}");
        }
        let (programs, reads) = (flash.programs(), flash.reads());
        for _ in 0..2 {
            assert_eq!(ring.append(&[b'x'; MAX_PAYLOAD], 1), Err(Error::Exhausted), "{newest}");
        }
        assert_eq!(
            (flash.programs(), flash.reads()),
            (programs, reads),
            "{newest}: nothing read or programmed"
        );
        let mut numbers: Vec<u32> = blocks(&flash.image(), geometry)
            .iter()
            .map(|block| block.header.block_sequence)
            .collect();
        let count = numbers.len();
        numbers.dedup();
        assert_eq!(numbers.len(), count, "{newest}: no number given twice");
        assert_eq!(count as u32, 1 + opens);
    }
}

#[test]
fn a_trusted_saver_hint_lets_the_saved_blocks_go_at_once_with_no_card_and_no_loss() {
    // boot 1 wrote blocks 0..5 and its saver put 0..2 on the card (hint (3, 0)); boot 2 has no card
    let geometry = small();
    let flash = erased(geometry);
    let first = bring_up(&flash, geometry, Hints::default());
    let mut ring = writer(&flash, &first, 1);
    for _ in 0..6 {
        ring.append(&[b'a'; MAX_PAYLOAD], 1).unwrap();
    }
    let position = ring.position();
    drop(ring);
    let image = flash.into_image();
    let hint = at(3, 0);
    let hints = Hints {
        writer: Some(position),
        saver: Some(hint),
    };
    let flash = MemoryFlash::from_image(image.clone());
    let scanned = bring_up(&flash, geometry, hints);
    let mut eraser = Eraser::new(&flash, &scanned.table, Some(hint)).unwrap();
    let erasures: Vec<Erasure> = (0..4).map(|_| eraser.erase_step(None, 0).unwrap()).collect();
    let saved = |slot, block_sequence| Erasure::Saved { slot, block_sequence };
    assert_eq!(
        erasures,
        [saved(0, 0), saved(1, 1), saved(2, 2), Erasure::Idle],
        "on the old card: erased at once, oldest first, no delay"
    );
    assert_eq!(scanned.table.figures(None).lost, 0);

    // another card (the bring-up passes None), or a hint that names no block in flash: nothing counts as saved
    for untrusted in [None, Some(at(40, 0)), Some(at(3, BLOCK_BYTES + 8))] {
        let flash = MemoryFlash::from_image(image.clone());
        let scanned = bring_up(&flash, geometry, hints);
        let mut eraser = Eraser::new(&flash, &scanned.table, untrusted).unwrap();
        assert_eq!(
            eraser.erase_step(None, 0),
            Ok(Erasure::Idle),
            "{untrusted:?}: two erased slots hold the reserve"
        );
        for _ in 0..2 {
            assert!(matches!(eraser.erase_step(None, 0), Ok(Erasure::Idle)), "{untrusted:?}");
        }
    }
}
