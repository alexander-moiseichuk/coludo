//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring's behaviour over an in-memory flash: append and read back, block rollover, wrap, reboot, spill,
//! GC rules, foreign blocks, the CONFIRM walk, and the figures. Power cuts are in power_cut.rs.

mod common;

use common::*;
use std::cell::Cell;
use std::rc::Rc;

use flash_ring::frame::{self, Confirm, Flags, HEADER_BYTES, Kind, frame_bytes};
use flash_ring::scan::Slot;
use flash_ring::{
    CONFIRM_SEARCH_BLOCKS, Erasure, Error, Figures, Flash, Geometry, MemoryFlash, Options, Position, Ring,
};

#[test]
fn round_trips_many_frames_in_order() {
    let geometry = Geometry::new(16, 4).unwrap();
    let mut ring = mount(erased(geometry), geometry, 7);
    for index in 0..400 {
        ring.append(Kind::Data, &payload(7, index), index % 7 == 0, uptime(index))
            .unwrap();
    }
    let image = ring.flash().image().to_vec();
    let found = walk(&image, geometry);
    assert_eq!(data(&found), (0..400).map(|index| (7, index)).collect::<Vec<_>>());
    let mut index = 0;
    for (number, found) in found.iter().enumerate() {
        assert_eq!(
            found.header.frame_sequence, number as u32,
            "+1 for every frame, BOOT included"
        );
        assert_eq!(found.header.boot_id, 7);
        assert_eq!(found.offset as usize % frame::ALIGNMENT, 0);
        assert_eq!(
            found.slot, found.header.block_sequence,
            "block k in slot k on a fresh ring"
        );
        if found.header.kind == Kind::Boot {
            assert_eq!(found.offset, 0, "a BOOT frame opens every block, and only there");
            assert_eq!(found.payload, boot_payload(7));
            continue;
        }
        assert_eq!(found.header.uptime_us, uptime(index));
        let flags = Flags {
            spilled: false,
            durable: index % 7 == 0,
        };
        assert_eq!(found.header.flags, flags);
        index += 1;
    }
    let last = found.last().unwrap();
    let end = last.offset + frame_bytes(last.payload.len()) as u32;
    assert_eq!(
        ring.programmed(),
        Some(Position {
            block_sequence: last.header.block_sequence,
            offset: end
        })
    );
    for (block, slot) in blocks(&image, geometry) {
        let frames = block_frames(&image, geometry, slot, block);
        let bytes = slot_bytes(&image, geometry, slot);
        let mut covered = 0;
        for found in &frames {
            assert!(
                bytes[covered..found.offset as usize].iter().all(|&byte| byte == 0xFF),
                "padding stays erased"
            );
            covered = found.offset as usize + HEADER_BYTES + found.payload.len();
        }
        assert!(
            bytes[covered..].iter().all(|&byte| byte == 0xFF),
            "block {block}: the tail stays erased"
        );
    }
}

#[test]
fn a_full_block_rolls_over_to_a_new_block_with_its_boot_frame() {
    let geometry = Geometry::new(4, 1).unwrap();
    let mut ring = mount(erased(geometry), geometry, 3);
    let boot = frame_bytes(boot_payload(3).len()) as u32;
    let largest = ring.max_payload();
    assert_eq!(largest, 4096 - boot as usize - HEADER_BYTES);
    ring.append(Kind::Data, &vec![b'x'; largest], false, 10).unwrap();
    assert_eq!(
        ring.programmed(),
        Some(Position {
            block_sequence: 0,
            offset: 4096
        }),
        "the largest payload fills a block exactly"
    );
    ring.append(Kind::Data, b"1", false, 20).unwrap();
    assert_eq!(
        ring.programmed(),
        Some(Position {
            block_sequence: 1,
            offset: boot + 48
        })
    );
    let image = ring.flash().image().to_vec();
    let second = block_frames(&image, geometry, 1, 1);
    assert_eq!(second[0].header.kind, Kind::Boot);
    assert_eq!(
        second[0].header.uptime_us, 20,
        "the BOOT frame takes the uptime of the frame it opened for"
    );
    assert_eq!(
        (second[0].header.frame_sequence, second[1].header.frame_sequence),
        (2, 3)
    );

    // A frame that does not fit the open block's rest goes to the next block, and a later small one that would
    // have fitted the rest never goes back: a closed block stays closed.
    ring.append(Kind::Data, &[b'w'; 200], false, 25).unwrap();
    ring.append(Kind::Data, &vec![b'y'; largest - 100], false, 30).unwrap();
    ring.append(Kind::Data, b"small", false, 40).unwrap();
    let image = ring.flash().image().to_vec();
    assert_eq!(
        block_frames(&image, geometry, 1, 1).len(),
        3,
        "block 1 was closed by the frame that did not fit"
    );
    let third: Vec<usize> = block_frames(&image, geometry, 2, 2)
        .iter()
        .map(|found| found.payload.len())
        .collect();
    assert_eq!(third[1..], [largest - 100, 5]);
}

#[test]
fn cutting_at_room_fills_a_block_to_its_last_byte() {
    let geometry = Geometry::new(4, 1).unwrap();
    let mut ring = mount(erased(geometry), geometry, 3);
    let largest = ring.max_payload();
    assert_eq!(ring.room(), largest, "no block is open: the next append opens one");
    ring.append(Kind::Data, &[b'a'; 100], false, 0).unwrap();
    let boot = frame_bytes(boot_payload(3).len());
    assert_eq!(ring.room(), 4096 - boot - frame_bytes(100) - HEADER_BYTES);

    let room = ring.room();
    ring.append(Kind::Data, &vec![b'b'; room], false, 1).unwrap();
    let full = Position {
        block_sequence: 0,
        offset: 4096,
    };
    assert_eq!(
        ring.programmed(),
        Some(full),
        "to the block's last byte, without opening block 1"
    );
    assert_eq!(
        ring.room(),
        largest,
        "nothing more fits block 0: the next append opens block 1"
    );

    // One byte past room() does not fit: it closes block 1 and opens block 2. There it leaves a rest of
    // exactly one header, which takes no payload, so room() counts it as none.
    ring.append(Kind::Data, b"c", false, 2).unwrap();
    let room = ring.room();
    ring.append(Kind::Data, &vec![b'd'; room + 1], false, 3).unwrap();
    let after = Position {
        block_sequence: 2,
        offset: 4096 - HEADER_BYTES as u32,
    };
    assert_eq!(ring.programmed(), Some(after));
    let image = ring.flash().image().to_vec();
    assert_eq!(
        block_frames(&image, geometry, 1, 1).len(),
        2,
        "block 1 kept only BOOT and 'c'"
    );
    assert_eq!(ring.room(), largest, "a rest of one header takes no payload");
    ring.append(Kind::Data, b"f", false, 4).unwrap();
    assert_eq!(ring.programmed().unwrap().block_sequence, 3);
}

#[test]
fn refuses_a_boot_frame_a_payload_too_large_and_a_flash_too_small() {
    let geometry = Geometry::new(4, 1).unwrap();
    let mut ring = mount(erased(geometry), geometry, 3);
    let largest = ring.max_payload();
    assert_eq!(ring.append(Kind::Boot, b"boot=9", false, 0), Err(Error::BootKind));
    assert_eq!(
        ring.append(Kind::Data, &vec![0; largest + 1], false, 0),
        Err(Error::TooLarge)
    );
    assert_eq!(
        (ring.flash().programs(), ring.programmed()),
        (0, None),
        "a refusal writes nothing"
    );

    let short = MemoryFlash::new(geometry.bytes() - 4096);
    let mounted = Ring::mount(short, geometry, OPTIONS, 1, b"boot=1");
    assert!(matches!(mounted, Err(Error::Geometry)));
    let mounted = Ring::mount(erased(geometry), geometry, OPTIONS, 1, &[b'b'; 4096 - 47]);
    assert!(
        matches!(mounted, Err(Error::TooLarge)),
        "no room left for a frame behind the BOOT"
    );
    assert!(Ring::mount(erased(geometry), geometry, OPTIONS, 1, &[b'b'; 4096 - 88]).is_ok());
}

#[test]
fn checks_a_block_is_erased_with_one_read_before_its_first_frame() {
    let geometry = small();
    let mut ring = mount(erased(geometry), geometry, 1);
    assert_eq!(ring.flash().reads(), 5, "the boot scan: one header read per block");
    ring.append(Kind::Data, &payload(1, 0), false, uptime(0)).unwrap();
    assert_eq!(ring.flash().reads(), 6, "opening block 0 reads it once");
    let mut index = 1;
    while ring.programmed().unwrap().block_sequence == 0 {
        ring.append(Kind::Data, &payload(1, index), false, uptime(index))
            .unwrap();
        index += 1;
    }
    assert_eq!(
        ring.flash().reads(),
        7,
        "frames into an open block read nothing; block 1 took one read"
    );
}

#[test]
fn wraps_around_the_slots_with_a_lagging_mover() {
    let geometry = small();
    let mut ring = mount(erased(geometry), geometry, 4);
    let mut accepted = Vec::new();
    let mut index = 0;
    while ring
        .programmed()
        .is_none_or(|at| at.block_sequence < 3 * geometry.blocks())
    {
        ring.append(Kind::Data, &payload(4, index), false, uptime(index))
            .unwrap();
        accepted.push((4, index));
        index += 1;
        let newest = ring.programmed().unwrap().block_sequence;
        ring.confirm_moved(Position {
            block_sequence: newest.saturating_sub(2),
            offset: 0,
        });
        ring.erase_step().unwrap();
        assert_eq!(ring.figures().spill_frames, 0, "GC keeps ahead of the writer");
    }
    let image = ring.flash().image().to_vec();
    for (block, slot) in blocks(&image, geometry) {
        assert_eq!(slot, block % geometry.blocks(), "block k lives in slot k % blocks");
    }
    let confirmed = ring.confirmed().block_sequence;
    let kept = data(&walk_from(&image, geometry, confirmed));
    assert!(kept.len() > 50);
    assert_eq!(
        kept[..],
        accepted[accepted.len() - kept.len()..],
        "the unconfirmed tail is whole and in order"
    );
    let open = block_frames(&image, geometry, geometry.slot(15), 15);
    assert_eq!(open[0].header.kind, Kind::Boot, "the third lap's first block");
}

#[test]
fn wraps_the_board_geometry() {
    let geometry = Geometry::BOARD;
    let boot = boot_payload(9);
    let mut ring = Ring::mount(MemoryFlash::new(24 << 20), geometry, Options::BOARD, 9, &boot).unwrap();
    assert_eq!(ring.figures().erased_ahead, 383);
    let line = vec![b'r'; 1700]; // a 50 ms frame at the flight rate (~35 KB/s)
    let mut frames = 0u32;
    let mut journal = 0;
    while ring.programmed().is_none_or(|at| at.block_sequence < 384 + 8) {
        ring.append(Kind::Data, &line, false, uptime(frames)).unwrap();
        frames += 1;
        let newest = ring.programmed().unwrap().block_sequence;
        if frames.is_multiple_of(100) && newest > 0 {
            let moved = Position {
                block_sequence: newest - 1,
                offset: 0,
            };
            journal += 1;
            let confirm = Confirm {
                moved,
                boot_id: 9,
                journal_sequence: journal,
                ..Confirm::default()
            };
            ring.confirm_moved(moved);
            ring.append(Kind::Confirm, &confirm.encode(), false, uptime(frames))
                .unwrap();
        }
        ring.erase_step().unwrap();
        assert_eq!(
            ring.figures().spill_frames,
            0,
            "one sector per frame keeps ahead at the flight rate"
        );
    }
    let newest = ring.programmed().unwrap().block_sequence;
    let confirmed = ring.confirmed();
    let image = ring.into_flash().into_image();
    assert_eq!(
        classify(&image, geometry)[geometry.slot(newest) as usize].block_sequence(),
        Some(newest)
    );

    let mut reboot = Ring::mount(MemoryFlash::from_image(image), geometry, Options::BOARD, 10, &boot).unwrap();
    let recovered = reboot
        .recover_confirm()
        .unwrap()
        .expect("a CONFIRM among the newest 16 blocks");
    assert_eq!((recovered.moved, recovered.journal_sequence), (confirmed, journal));
    reboot.append(Kind::Data, b"after the reboot\n", false, 5).unwrap();
    assert_eq!(
        reboot.programmed().unwrap().block_sequence,
        newest + 1,
        "the reboot opens the slot after the newest block"
    );
}

#[test]
fn a_reboot_opens_the_next_slot_and_never_appends_to_an_old_block() {
    let geometry = small();
    let mut first = mount(erased(geometry), geometry, 1);
    fill_to(&mut first, 1, 0, 2, 4000);
    let image = first.into_flash().into_image();

    let second = mount(MemoryFlash::from_image(image.clone()), geometry, 2);
    let valid = |block_sequence| Slot::Valid {
        block_sequence,
        boot_id: 1,
    };
    assert_eq!(
        second.slots(),
        [valid(0), valid(1), valid(2), Slot::Erased, Slot::Erased]
    );
    let next = Position {
        block_sequence: 3,
        offset: 0,
    };
    assert_eq!(
        (second.programmed(), second.flash().programs(), second.flash().erases()),
        (Some(next), 0, 0),
        "the old blocks are closed, so readable whole, and nothing of block 3 yet"
    );

    let mut second = second;
    second.append(Kind::Data, &payload(2, 0), false, uptime(0)).unwrap();
    let after = second.into_flash().into_image();
    for slot in 0..3 {
        assert_eq!(
            slot_bytes(&after, geometry, slot),
            slot_bytes(&image, geometry, slot),
            "slot {slot} untouched"
        );
    }
    let opened = block_frames(&after, geometry, 3, 3);
    assert_eq!(
        opened
            .iter()
            .map(|found| (found.header.kind, found.header.boot_id, found.header.frame_sequence))
            .collect::<Vec<_>>(),
        [(Kind::Boot, 2, 0), (Kind::Data, 2, 1)]
    );

    let mut third = mount(MemoryFlash::from_image(after.clone()), geometry, 3);
    third.append(Kind::Data, &payload(3, 0), false, uptime(0)).unwrap();
    let after_third = third.into_flash().into_image();
    assert_eq!(
        slot_bytes(&after_third, geometry, 3),
        slot_bytes(&after, geometry, 3),
        "boot 2's block is closed"
    );
    assert_eq!(blocks(&after_third, geometry).last(), Some(&(4, 4)));

    // The next block, 5, falls in slot 0 over block 0, which nobody confirmed: the frame spills.
    let mut fourth = mount(MemoryFlash::from_image(after_third.clone()), geometry, 4);
    assert_eq!(fourth.recover_confirm(), Ok(None));
    fourth.append(Kind::Data, &payload(4, 0), false, uptime(0)).unwrap();
    assert_eq!((fourth.figures().spill_frames, fourth.flash().programs()), (1, 0));
    assert_eq!(fourth.erase_step(), Ok(Erasure::Idle), "block 0 is not confirmed");
    assert_eq!(fourth.flash().image(), &after_third[..]);
}

#[test]
fn a_reboot_over_a_ring_full_of_unconfirmed_blocks_lets_the_mover_read_them_all() {
    // A long run with no card: boot 1 fills all four slots, and nothing is confirmed.
    let geometry = Geometry::new(4, 1).unwrap();
    let mut first = mount(erased(geometry), geometry, 1);
    let largest = vec![b'f'; first.max_payload()];
    for _ in 0..4 {
        first.append(Kind::Data, &largest, false, 0).unwrap();
    }
    let image = first.into_flash().into_image();

    // Boot 2 spills from its first frame, so it programs nothing, yet the mover gets its bound at once.
    let mut ring = mount(MemoryFlash::from_image(image), geometry, 2);
    for index in 0..6 {
        ring.append(Kind::Data, &payload(2, index), false, uptime(index))
            .unwrap();
    }
    let figures = ring.figures();
    assert_eq!((figures.spill_frames, figures.unconfirmed), (6, 4));
    assert_eq!(ring.erase_step(), Ok(Erasure::Idle));
    let bound = ring.programmed().expect("a bound before this boot programs anything");
    for block in 0..4 {
        let block_end = Position {
            block_sequence: block,
            offset: geometry.block_bytes(),
        };
        assert!(block_end <= bound, "block {block} is readable whole");
    }
    assert_eq!(
        bound,
        Position {
            block_sequence: 4,
            offset: 0
        },
        "and nothing of block 4"
    );

    // The card arrives: the mover copies up to the bound and confirms it, GC frees slot 0, the spill drains.
    ring.confirm_moved(bound);
    while ring.figures().spill_frames > 0 {
        assert!(matches!(ring.erase_step(), Ok(Erasure::Sector { .. })));
        ring.drain().unwrap();
    }
    let kept = walk_from(ring.flash().image(), geometry, 4);
    assert_eq!(data(&kept), (0..6).map(|index| (2, index)).collect::<Vec<_>>());
}

#[test]
fn spills_while_the_next_block_is_unconfirmed_and_drains_in_order() {
    let geometry = Geometry::new(4, 2).unwrap();
    let options = Options {
        spill_capacity: 12 << 10,
        reserve_blocks: 1,
    };
    let mut ring = Ring::mount(erased(geometry), geometry, options, 5, &boot_payload(5)).unwrap();
    let durable = |index: u32| index.is_multiple_of(3);
    // Every accepted frame's index, and whether it was spilled: a spilled frame is the queue's last.
    let mut accepted: Vec<(u32, bool)> = Vec::new();
    let mut index = 0;
    let held = loop {
        match ring.append(Kind::Data, &payload(5, index), durable(index), uptime(index)) {
            Ok(()) => accepted.push((index, ring.figures().spill_frames > 0)),
            Err(Error::SpillFull) => break index,
            Err(other) => panic!("append failed: {other:?}"),
        }
        index += 1;
    };
    let figures = ring.figures();
    assert!(
        figures.spill_frames > 20,
        "a spill holds more than one 8 KiB block: {figures:?}"
    );
    assert!(figures.spill_bytes as usize <= options.spill_capacity);
    assert_eq!(
        ring.programmed().unwrap().block_sequence,
        3,
        "blocks 0..3 filled; block 4 needs slot 0"
    );
    let programs = ring.flash().programs();

    // Back-pressure: the writer keeps the frame it cut, cuts no more, and appends it again after GC steps.
    assert_eq!(
        ring.erase_step(),
        Ok(Erasure::Idle),
        "unconfirmed data is never erased to make room"
    );
    let again = ring.append(Kind::Data, &payload(5, held), durable(held), uptime(held));
    assert_eq!(again, Err(Error::SpillFull));
    assert_eq!(
        (ring.figures(), ring.flash().programs()),
        (figures, programs),
        "a refusal stores, programs and numbers nothing"
    );
    assert_eq!(
        ring.room(),
        ring.max_payload(),
        "no block is open while frames are spilled"
    );

    ring.confirm_moved(Position {
        block_sequence: 1,
        offset: 0,
    });
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector: 1 }));
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector: 0 }));
    assert_eq!(ring.erase_step(), Ok(Erasure::Idle));
    ring.append(Kind::Data, &payload(5, held), durable(held), uptime(held))
        .unwrap(); // the spill drains into block 4 first, which makes room
    accepted.push((held, ring.figures().spill_frames > 0));
    assert_eq!(ring.programmed().unwrap().block_sequence, 4);
    assert!(
        ring.figures().spill_frames > 0,
        "the rest waits for slot 1, which holds unconfirmed block 1"
    );

    ring.confirm_moved(Position {
        block_sequence: 3,
        offset: 0,
    });
    while ring.figures().spill_frames > 0 {
        assert!(matches!(ring.erase_step(), Ok(Erasure::Sector { .. })));
        ring.drain().unwrap();
    }
    let last = held + 1;
    ring.append(Kind::Data, &payload(5, last), durable(last), uptime(last))
        .unwrap();
    accepted.push((last, ring.figures().spill_frames > 0));

    let image = ring.flash().image().to_vec();
    let kept = walk_from(&image, geometry, 3);
    let kept_data: Vec<&Found> = kept.iter().filter(|found| found.header.kind == Kind::Data).collect();
    let tail = &accepted[accepted.len() - kept_data.len()..];
    assert!(
        tail.iter().any(|&(index, _)| index == held),
        "the refused frame is stored once there is room"
    );
    for (found, &(index, spilled)) in kept_data.iter().zip(tail) {
        assert_eq!(identify(&found.payload), (5, index), "in order, nothing skipped");
        let flags = Flags {
            spilled,
            durable: durable(index),
        };
        assert_eq!(found.header.flags, flags, "frame {index}");
    }
    assert!(
        tail.iter().any(|&(index, spilled)| spilled && durable(index)),
        "a durable frame was spilled and kept its bit"
    );
    assert!(!tail[tail.len() - 1].1, "the last one went direct");
    let numbers: Vec<u32> = kept.iter().map(|found| found.header.frame_sequence).collect();
    assert!(
        numbers.windows(2).all(|pair| pair[1] == pair[0] + 1),
        "no gap: a refusal takes no number, and the frame takes one when programmed: {numbers:?}"
    );
}

#[test]
fn erases_only_confirmed_blocks_in_reverse_sector_order() {
    let geometry = small();
    let mut ring = mount(erased(geometry), geometry, 1);
    fill_to(&mut ring, 1, 0, 3, 100);
    let before = ring.flash().image().to_vec();
    assert_eq!(ring.erase_step(), Ok(Erasure::Idle), "nothing confirmed");
    ring.confirm_moved(Position {
        block_sequence: 0,
        offset: 16000,
    });
    assert_eq!(
        ring.erase_step(),
        Ok(Erasure::Idle),
        "a cursor inside block 0 does not free block 0"
    );

    ring.confirm_moved(Position {
        block_sequence: 1,
        offset: 0,
    });
    for sector in [3, 2, 1] {
        assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector }));
        assert_eq!(
            classify(ring.flash().image(), geometry)[0],
            Slot::Valid {
                block_sequence: 0,
                boot_id: 1
            },
            "the header sector goes last"
        );
    }
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector: 0 }));
    assert!(
        slot_bytes(ring.flash().image(), geometry, 0)
            .iter()
            .all(|&byte| byte == 0xFF)
    );
    assert_eq!(ring.erase_step(), Ok(Erasure::Idle), "block 1 is not before the cursor");
    for slot in 1..4 {
        assert_eq!(
            slot_bytes(ring.flash().image(), geometry, slot),
            slot_bytes(&before, geometry, slot)
        );
    }

    ring.confirm_moved(Position::default());
    assert_eq!(
        ring.confirmed(),
        Position {
            block_sequence: 1,
            offset: 0
        },
        "the cursor never goes back"
    );
}

#[test]
fn never_erases_the_newest_block_even_when_confirmed() {
    let geometry = small();
    let mut ring = mount(erased(geometry), geometry, 1);
    fill_to(&mut ring, 1, 0, 1, 100);
    ring.confirm_moved(Position {
        block_sequence: 9,
        offset: 0,
    });
    let mut erased_slots = Vec::new();
    while let Ok(Erasure::Sector { slot, .. }) = ring.erase_step() {
        erased_slots.push(slot);
    }
    assert_eq!(erased_slots, [0, 0, 0, 0], "block 0 only; open block 1 is the writer's");
    assert!(ring.slots()[1].block_sequence() == Some(1));
}

#[test]
fn does_not_skip_an_unconfirmed_block_to_erase_a_later_one() {
    let geometry = small();
    let mut first = mount(erased(geometry), geometry, 1);
    fill_to(&mut first, 1, 0, 2, 100);
    let mut image = first.into_flash().into_image();
    let start = geometry.slot_offset(1) as usize;
    image[start..start + 4096].copy_from_slice(&noise(77, 4096)); // block 1 damaged into a foreign slot

    let mut ring = mount(MemoryFlash::from_image(image), geometry, 2);
    assert_eq!(ring.slots()[1], Slot::Foreign);
    assert_eq!(ring.figures().foreign, 1);
    assert_eq!(
        ring.erase_step(),
        Ok(Erasure::Idle),
        "slot 0 (unconfirmed) stands before the foreign slot 1"
    );
    ring.confirm_moved(Position {
        block_sequence: 1,
        offset: 0,
    });
    let mut erased_slots = Vec::new();
    while let Ok(Erasure::Sector { slot, .. }) = ring.erase_step() {
        erased_slots.push(slot);
    }
    assert_eq!(
        erased_slots,
        [0, 0, 0, 0, 1, 1, 1, 1],
        "then both go, the newest block (2) stays"
    );
}

#[test]
fn the_flight_pause_holds_erases_once_enough_blocks_are_erased_ahead() {
    let geometry = Geometry::new(6, 1).unwrap();
    let options = Options {
        spill_capacity: 0,
        reserve_blocks: 2,
    };
    let mut ring = Ring::mount(erased(geometry), geometry, options, 1, &boot_payload(1)).unwrap();
    let largest = vec![b'z'; ring.max_payload()];
    for index in 0..5 {
        ring.append(Kind::Data, &largest, false, uptime(index)).unwrap(); // one block each
    }
    ring.confirm_moved(Position {
        block_sequence: 4,
        offset: 0,
    });
    ring.pause_erases(true);
    assert_eq!(ring.figures().erased_ahead, 1);
    assert_eq!(
        ring.erase_step(),
        Ok(Erasure::Sector { slot: 0, sector: 0 }),
        "1 erased ahead is below 2"
    );
    assert_eq!(
        ring.erase_step(),
        Ok(Erasure::Paused),
        "2 erased ahead: the pause holds"
    );
    ring.pause_erases(false);
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 1, sector: 0 }));
    ring.pause_erases(true);
    assert_eq!(ring.erase_step(), Ok(Erasure::Paused));
    assert_eq!(ring.figures().erased_ahead, 3);
}

#[test]
fn a_foreign_block_is_erased_without_a_confirmation() {
    let geometry = small();
    let mut first = mount(erased(geometry), geometry, 1);
    fill_to(&mut first, 1, 0, 2, 100);
    let mut image = first.into_flash().into_image();
    let foreign = geometry.slot_offset(4) as usize;
    image[foreign..foreign + 16384].copy_from_slice(&noise(5, 16384));
    let before = image.clone();

    let mut ring = mount(MemoryFlash::from_image(image), geometry, 2);
    assert_eq!(ring.slots()[4], Slot::Foreign);
    for sector in [3, 2, 1, 0] {
        assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 4, sector }));
    }
    assert_eq!(ring.erase_step(), Ok(Erasure::Idle));
    assert_eq!(ring.slots()[4], Slot::Erased);
    assert!(
        slot_bytes(ring.flash().image(), geometry, 4)
            .iter()
            .all(|&byte| byte == 0xFF)
    );
    for slot in 0..3 {
        assert_eq!(
            slot_bytes(ring.flash().image(), geometry, slot),
            slot_bytes(&before, geometry, slot)
        );
    }
}

#[test]
fn a_block_of_another_layout_version_is_never_erased() {
    let geometry = small();
    let mut first = mount(erased(geometry), geometry, 1);
    fill_to(&mut first, 1, 0, 2, 100);
    let mut image = first.into_flash().into_image();
    image[4] = 2; // block 0's BOOT, re-stamped as layout version 2 with a good header CRC: another firmware's
    let header_crc = recorder_wire::crc32(0, &image[..36]);
    image[36..40].copy_from_slice(&header_crc.to_le_bytes());
    let before = image.clone();

    let mut ring = mount(MemoryFlash::from_image(image), geometry, 2);
    assert_eq!(ring.slots()[0], Slot::OtherVersion);
    let figures = ring.figures();
    assert_eq!((figures.other_version, figures.foreign, figures.unconfirmed), (1, 0, 2));
    ring.confirm_moved(Position {
        block_sequence: 9,
        offset: 0,
    });
    assert_eq!(
        ring.erase_step(),
        Ok(Erasure::Idle),
        "slots 3 and 4 are erased, and GC stops at slot 0 whatever the cursor"
    );
    let mut index = 0;
    while ring.figures().spill_frames == 0 {
        ring.append(Kind::Data, &payload(2, index), false, uptime(index))
            .unwrap(); // blocks 3 and 4, then block 5 would need slot 0
        index += 1;
    }
    assert_eq!(ring.programmed().unwrap().block_sequence, 4);
    for _ in 0..8 {
        assert_eq!(ring.erase_step(), Ok(Erasure::Idle));
    }
    assert_eq!(
        slot_bytes(ring.flash().image(), geometry, 0),
        slot_bytes(&before, geometry, 0),
        "only the operator's discard erases it"
    );
}

#[test]
fn an_all_foreign_ring_is_formatted_ahead_of_the_writer() {
    let geometry = small();
    let mut ring = mount(
        MemoryFlash::from_image(noise(11, geometry.bytes() as usize)),
        geometry,
        1,
    );
    assert_eq!(ring.slots(), [Slot::Foreign; 5]);
    let figures = ring.figures();
    assert!(figures.foreign == 5 && figures.unconfirmed == 0, "{figures:?}");

    ring.append(Kind::Data, &payload(1, 0), false, uptime(0)).unwrap();
    assert_eq!(
        (ring.figures().spill_frames, ring.programmed()),
        (1, None),
        "spills until slot 0 is erased"
    );
    for sector in [3, 2, 1, 0] {
        assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector }));
    }
    ring.append(Kind::Data, &payload(1, 1), true, uptime(1)).unwrap();
    while let Ok(Erasure::Sector { .. }) = ring.erase_step() {}
    let figures = ring.figures();
    assert!(figures.foreign == 0, "the format is done: {figures:?}");
    assert_eq!(ring.slots()[1..], [Slot::Erased; 4]);

    let opened = block_frames(ring.flash().image(), geometry, 0, 0);
    let summary: Vec<(Kind, bool, u32)> = opened
        .iter()
        .map(|found| {
            (
                found.header.kind,
                found.header.flags.spilled,
                found.header.frame_sequence,
            )
        })
        .collect();
    assert_eq!(
        summary,
        [(Kind::Boot, false, 0), (Kind::Data, true, 1), (Kind::Data, false, 2)]
    );
}

#[test]
fn a_format_a_reboot_interrupted_still_counts_as_unformatted() {
    let geometry = Geometry::new(6, 1).unwrap();
    let mut ring = mount(
        MemoryFlash::from_image(noise(13, geometry.bytes() as usize)),
        geometry,
        1,
    );
    ring.append(Kind::Data, &payload(1, 0), false, uptime(0)).unwrap();
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector: 0 }));
    ring.drain().unwrap();
    assert_eq!(ring.programmed().unwrap().block_sequence, 0);
    assert_eq!(ring.figures().foreign, 5, "ring-unformatted");
    let image = ring.into_flash().into_image();

    let mut ring = mount(MemoryFlash::from_image(image), geometry, 2);
    let first_block = Slot::Valid {
        block_sequence: 0,
        boot_id: 1,
    };
    assert_eq!(ring.slots()[0], first_block, "the scan finds a valid block");
    assert_eq!(ring.figures().foreign, 5, "still ring-unformatted: the format goes on");
    let mut erased_slots = Vec::new();
    while let Ok(Erasure::Sector { slot, .. }) = ring.erase_step() {
        erased_slots.push(slot);
    }
    assert_eq!(
        erased_slots,
        [1, 2, 3, 4, 5],
        "ahead of the writer, then it stops at block 0"
    );
    assert_eq!(ring.figures().foreign, 0, "formatted");
}

#[test]
fn a_dirty_block_behind_an_erased_header_is_refused_then_erased() {
    let geometry = small();
    let mut image = vec![0xFF; geometry.bytes() as usize];
    image[5000] = 0x00; // a stray programmed byte in slot 0's second sector
    let mut ring = mount(MemoryFlash::from_image(image), geometry, 1);
    assert_eq!(ring.slots()[0], Slot::Erased, "the header alone reads erased");
    ring.append(Kind::Data, &payload(1, 0), false, uptime(0)).unwrap();
    assert_eq!(ring.slots()[0], Slot::Foreign, "the one full read refused it");
    assert_eq!((ring.flash().programs(), ring.figures().spill_frames), (0, 1));
    while let Ok(Erasure::Sector { slot: 0, .. }) = ring.erase_step() {}
    ring.drain().unwrap();
    assert_eq!(ring.programmed().unwrap().block_sequence, 0);
    assert_eq!(ring.flash().image()[5000], 0xFF);
    assert_eq!(data(&walk(ring.flash().image(), geometry)), [(1, 0)]);
}

/// A ring of 20 one-sector blocks where boot 1 wrote `confirms` (block, cursors) as CONFIRM frames, one block per
/// entry, then filled `filler` more blocks with one DATA frame each.
fn confirm_ring(confirms: &[&[Position]], filler: u32) -> Vec<u8> {
    let geometry = Geometry::new(20, 1).unwrap();
    let mut ring = mount(erased(geometry), geometry, 1);
    let largest = vec![b'f'; ring.max_payload()];
    for (journal, moved) in confirms.iter().enumerate() {
        for moved in moved.iter() {
            let confirm = Confirm {
                moved: *moved,
                boot_id: 1,
                journal_sequence: journal as u32,
                ..Confirm::default()
            };
            ring.append(Kind::Confirm, &confirm.encode(), false, 0).unwrap();
        }
        let room = ring.room();
        ring.append(Kind::Data, &largest[..room], false, 0).unwrap(); // fills the block to its last byte
    }
    for _ in 0..filler {
        ring.append(Kind::Data, &largest, false, 0).unwrap();
    }
    ring.into_flash().into_image()
}

#[test]
fn recovers_the_newest_confirm_within_16_blocks() {
    let geometry = Geometry::new(20, 1).unwrap();
    let at = |block_sequence, offset| Position { block_sequence, offset };
    let image = confirm_ring(&[&[at(0, 10)], &[at(0, 900), at(1, 64)]], 3);
    let mut ring = mount(MemoryFlash::from_image(image.clone()), geometry, 2);
    let found = ring.recover_confirm().unwrap().unwrap();
    assert_eq!(
        (found.moved, found.journal_sequence),
        (at(1, 64), 1),
        "the last CONFIRM of the newest block"
    );
    assert_eq!(ring.confirmed(), at(1, 64));

    // A torn last CONFIRM ends its block's walk; the one before it wins.
    let mut torn = image.clone();
    let second = block_frames(&image, geometry, 1, 1);
    let damaged = second
        .iter()
        .filter(|found| found.header.kind == Kind::Confirm)
        .nth(1)
        .unwrap();
    torn[4096 + damaged.offset as usize + HEADER_BYTES] ^= 0x01;
    let mut ring = mount(MemoryFlash::from_image(torn), geometry, 2);
    assert_eq!(ring.recover_confirm().unwrap().unwrap().moved, at(0, 900));

    // Block 0's CONFIRM, with block 15 newest, is the 16th block back: found. With block 16 newest it is not.
    let mut ring = mount(MemoryFlash::from_image(confirm_ring(&[&[at(0, 10)]], 15)), geometry, 2);
    assert_eq!(CONFIRM_SEARCH_BLOCKS, 16);
    assert_eq!(ring.recover_confirm().unwrap().unwrap().moved, at(0, 10));
    let mut ring = mount(MemoryFlash::from_image(confirm_ring(&[&[at(0, 10)]], 16)), geometry, 2);
    assert_eq!(ring.recover_confirm(), Ok(None));
    assert_eq!(
        ring.confirmed(),
        Position::default(),
        "conservative: nothing becomes erasable"
    );

    let mut fresh = mount(erased(geometry), geometry, 1);
    assert_eq!(fresh.recover_confirm(), Ok(None), "an empty ring has none");
}

#[test]
fn publishes_the_programmed_position_and_the_figures() {
    let geometry = small();
    let mut ring = mount(erased(geometry), geometry, 1);
    assert_eq!(ring.programmed(), None);
    let boot = frame_bytes(boot_payload(1).len()) as u32;
    ring.append(Kind::Data, b"0123456789", false, 1).unwrap();
    assert_eq!(
        ring.programmed(),
        Some(Position {
            block_sequence: 0,
            offset: boot + 56
        })
    );
    fill_to(&mut ring, 1, 1, 3, 100);
    ring.confirm_moved(Position {
        block_sequence: 2,
        offset: 0,
    });
    let figures = ring.figures();
    let expected = Figures {
        erased_ahead: 1,
        moved_not_erased: 2,
        foreign: 0,
        other_version: 0,
        unconfirmed: 2,
        spill_frames: 0,
        spill_bytes: 0,
    };
    assert_eq!(figures, expected);
    while let Ok(Erasure::Sector { .. }) = ring.erase_step() {}
    let figures = ring.figures();
    assert_eq!(
        (figures.erased_ahead, figures.moved_not_erased),
        (3, 0),
        "slot 4 then slots 0 and 1"
    );
}

#[test]
fn a_failed_program_closes_the_block_and_the_retry_lands_in_a_new_one() {
    /// A flash whose next program fails without writing, as a flash timeout would, once `fail_after` more
    /// programs have gone through.
    struct Failing {
        /// The flash behind it.
        memory: MemoryFlash,
        /// Programs still to let through before one fails, or None for no failure. Shared with the test, which
        /// sets it between calls.
        fail_after: Rc<Cell<Option<u32>>>,
    }
    impl Flash for Failing {
        type Error = &'static str;
        fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), &'static str> {
            self.memory.read(offset, buffer).map_err(|_| "read")
        }
        fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), &'static str> {
            match self.fail_after.get() {
                Some(0) => {
                    self.fail_after.set(None);
                    return Err("timeout");
                }
                Some(more) => self.fail_after.set(Some(more - 1)),
                None => {}
            }
            self.memory.program(offset, data).map_err(|_| "program")
        }
        fn erase_sector(&mut self, offset: u32) -> Result<(), &'static str> {
            self.memory.erase_sector(offset).map_err(|_| "erase")
        }
        fn size(&self) -> u32 {
            self.memory.size()
        }
    }
    let geometry = Geometry::new(4, 1).unwrap();
    let fail_after = Rc::new(Cell::new(None));
    let flash = Failing {
        memory: erased(geometry),
        fail_after: fail_after.clone(),
    };
    let mut ring = Ring::mount(flash, geometry, OPTIONS, 1, &boot_payload(1)).unwrap();
    let largest = vec![b'l'; ring.max_payload()];
    ring.append(Kind::Data, &largest, false, 0).unwrap(); // block 0: BOOT 0, DATA 1, full

    fail_after.set(Some(0)); // block 1's BOOT program fails
    assert_eq!(ring.append(Kind::Data, b"a", false, 1), Err(Error::Flash("timeout")));
    assert_eq!(ring.slots()[1], Slot::Foreign, "a failed BOOT leaves its slot to GC");
    assert_eq!(ring.programmed().unwrap().block_sequence, 0);
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 1, sector: 0 }));
    ring.append(Kind::Data, b"a", false, 1).unwrap(); // the same block 1, once erased: BOOT 3, DATA 4

    fail_after.set(Some(0)); // a DATA program into open block 1 fails
    assert_eq!(ring.append(Kind::Data, b"b", false, 2), Err(Error::Flash("timeout")));
    assert_eq!(ring.room(), ring.max_payload(), "the failed frame closed block 1");
    ring.append(Kind::Data, b"b", false, 2).unwrap(); // the retry opens block 2: BOOT 6, DATA 7

    let numbers = |image: &[u8], block: u32| -> Vec<(Kind, u32)> {
        block_frames(image, geometry, geometry.slot(block), block)
            .iter()
            .map(|found| (found.header.kind, found.header.frame_sequence))
            .collect()
    };
    let image = ring.flash().memory.image().to_vec();
    assert_eq!(numbers(&image, 0), [(Kind::Boot, 0), (Kind::Data, 1)]);
    assert_eq!(
        numbers(&image, 1),
        [(Kind::Boot, 3), (Kind::Data, 4)],
        "frame 2 was the failed BOOT: a visible gap"
    );
    assert_eq!(
        numbers(&image, 2),
        [(Kind::Boot, 6), (Kind::Data, 7)],
        "block 1 was closed by the failed frame 5"
    );

    // A spilled frame whose program fails in drain() stays first in the queue, and lands later, once and in
    // order, in a new block.
    let room = ring.room();
    ring.append(Kind::Data, &largest[..room], false, 3).unwrap(); // block 2 full: DATA 8
    ring.append(Kind::Data, &largest, false, 4).unwrap(); // block 3: BOOT 9, DATA 10, full
    ring.append(Kind::Data, b"s1", true, 5).unwrap(); // block 4 needs slot 0, which holds unconfirmed block 0
    ring.append(Kind::Data, b"s2", false, 6).unwrap();
    assert_eq!(ring.figures().spill_frames, 2);
    ring.confirm_moved(Position {
        block_sequence: 1,
        offset: 0,
    });
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 0, sector: 0 }));
    fail_after.set(Some(1)); // block 4's BOOT 11 goes through, then s1's program (frame 12) fails
    assert_eq!(ring.drain(), Err(Error::Flash("timeout")));
    assert_eq!(ring.figures().spill_frames, 2, "s1 is still first in the queue");
    assert_eq!(ring.drain(), Ok(()), "block 5 waits: slot 1 holds unconfirmed block 1");
    ring.confirm_moved(Position {
        block_sequence: 2,
        offset: 0,
    });
    assert_eq!(ring.erase_step(), Ok(Erasure::Sector { slot: 1, sector: 0 }));
    ring.drain().unwrap();
    assert_eq!(ring.figures().spill_frames, 0);

    let image = ring.flash().memory.image().to_vec();
    assert_eq!(
        numbers(&image, 4),
        [(Kind::Boot, 11)],
        "the failed s1 closed block 4 and left nothing in it"
    );
    let spilled = |durable| Flags { spilled: true, durable };
    let landed: Vec<(Kind, u32, Vec<u8>, Flags)> = block_frames(&image, geometry, 1, 5)
        .into_iter()
        .map(|found| {
            (
                found.header.kind,
                found.header.frame_sequence,
                found.payload,
                found.header.flags,
            )
        })
        .collect();
    assert_eq!(
        landed,
        [
            (Kind::Boot, 13, boot_payload(1), Flags::default()),
            (Kind::Data, 14, b"s1".to_vec(), spilled(true)),
            (Kind::Data, 15, b"s2".to_vec(), spilled(false)),
        ]
    );
}
