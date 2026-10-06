//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Power cuts while the ring is full (doc/specs/avionics-port.md "The ring_eraser and newest wins", "The
//! ring_writer" step 7, "Block and chunk format"). power_cut.rs cuts a boot that writes a few blocks into a large
//! ring; this one cuts a boot that wraps a 10-block ring about three times with no saver, so every cut lands
//! somewhere newest wins is at work:
//! * while the eraser erases the oldest unsaved block next to the writer's open one;
//! * while the writer is refused (`WouldBlock`), its chunk kept for the retry and the producers' records waiting
//!   in PSRAM (and then refused at push);
//! * between an erase and the open that takes the freed slot;
//! * inside the flight, which is set for a middle stretch, and inside the pad blocks after it.
//!
//! The producers outrun the eraser (it steps every third tick), so the writer is refused again and again. The
//! boot runs once whole, recording what the flash read back as after every tick; then once per flash operation it
//! made, cut there at each tear length. After every cut:
//! * each stream's surviving lines are, whole and in order, without a duplicate, at most two runs of what it
//!   queued: the run that holds the flight, and the newest records (the two may be one);
//! * against the whole run's read-back after the last tick the cut left whole, nothing is missing but the lines of
//!   at most one block, a pad block that the cut tick's eraser erased (reported as a loss) or whose erase the cut
//!   tore; and every line the cut tick added is newer than all of them;
//! * every block sequence the boot opened is in flash or reported lost by the eraser, but for the one whose erase
//!   the cut tore; the writer's open block is in flash, and no eraser step ever erased it;
//! * boot 124 then brings the ring up, opens the block after boot 123's newest, records whole, and newest wins
//!   gives up boot 123's pad blocks before its flight.

mod common;

use common::*;

use std::collections::{BTreeMap, BTreeSet};
use std::ops::Range;

use flash_ring::{BlockTable, Eraser, Erasure, Geometry, MemoryFlash, Slot};
use recorder::{Cell, Decoder, Recorder, Registry, Settings, StreamId, TICK_US, Telemetry, Writer};

/// 10 blocks, a reserve of one: the boot fills it in about 25 steps, then wraps it about three times.
fn geometry() -> Geometry {
    Geometry::new(10).unwrap()
}

/// Steps in which boot 123's producers queue records.
const PRODUCING_STEPS: u64 = 100;

/// Steps after them, the eraser stepping every tick, until the backlog is out.
const SETTLING_STEPS: u64 = 12;

/// IMU rows queued per step: 12 rows of 132 bytes, about 0.4 of a block.
const ROWS_PER_STEP: u64 = 12;

/// The eraser steps on every this many producing steps: about a block per three steps, slower than the writer.
const ERASE_EVERY: u64 = 3;

/// The steps of the flight: BOOSTING at the first, DONE at the end. About four blocks of the ten, the backlog
/// queued just before BOOSTING included.
const FLIGHT: Range<u64> = 35..45;

/// The tear lengths tried at each operation, as power_cut.rs tries them.
const TEARS: [usize; 11] = [0, 1, 7, 8, 9, 30, 55, 56, 57, 300, 5000];

/// The pad every IMU row carries, so a row is 132 bytes and the blocks fill fast.
const PAD: &str =
    "padpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadp";

/// The boot after the cut.
const NEXT_BOOT: u32 = BOOT_ID + 1;

/// A boot's recorder and its two streams.
struct Boot {
    /// The Recorder, with no attachable ring: the test thread queues through `misc`, 8 KiB.
    parts: Parts,
    /// `<session>_imu.csv`: index, pad.
    imu: StreamId,
    /// `<session>_flight_events.csv`: durable, a row every seventh step.
    events: StreamId,
}

impl Boot {
    /// Boot `boot_id`'s recorder, on the pad.
    fn new(boot_id: u32) -> Boot {
        let mut registry = Registry::new();
        let imu = registry.add("imu.csv", &["index", "pad"], -1, false).unwrap();
        let events = registry.add("flight_events.csv", &["event"], -1, true).unwrap();
        let settings = Settings {
            telemetry_ms: 0,
            stats_ms: 300,
        };
        Boot {
            parts: recorder_of(boot_id, registry, 0, 8 << 10, settings, false),
            imu,
            events,
        }
    }
}

/// The program and erase calls made on `flash` so far, a torn one included.
fn operations(flash: &MemoryFlash) -> u32 {
    flash.programs() + flash.erases()
}

/// The slot the writer has open, if any.
fn open_slot(table: &BlockTable) -> Option<u32> {
    (0..table.geometry().blocks()).find(|&slot| matches!(table.slot(slot), Slot::Open { .. }))
}

/// The writer and the eraser of one boot over one flash, stepped as their two threads would be.
struct Board<'a> {
    /// The partition.
    flash: &'a MemoryFlash,
    /// The ring_writer.
    writer: Writer<'static, 'a, &'a MemoryFlash>,
    /// The ring_eraser.
    eraser: Eraser<'a, &'a MemoryFlash>,
    /// The operation the power goes at, if any.
    cut: Option<u32>,
    /// The block sequences newest wins erased, as the eraser reported them.
    losses: Vec<u32>,
    /// Whether the cut tore an erase.
    cut_erase: bool,
}

impl Board<'_> {
    /// One writer step at `now_us`, the clock moved there, then one eraser step when `erase`. Checks that the
    /// eraser never takes the writer's open block, nor a flight block while pad blocks are left. Returns true once
    /// the power is cut: the board is dead from then on.
    fn tick(&mut self, clock: &TestClock, now_us: u64, erase: bool) -> bool {
        clock.set(now_us);
        self.writer.step(now_us, EPOCH);
        if erase {
            let open = open_slot(self.writer.ring().table());
            let (before, erases) = (operations(self.flash), self.flash.erases());
            let erased = match self.eraser.erase_step(None, (now_us / 1000) as u32) {
                Ok(Erasure::Lost(loss)) => {
                    assert!(!loss.flight, "the workload always leaves a pad block: {loss:?}");
                    self.losses.push(loss.block_sequence);
                    Some(loss.slot)
                }
                Ok(Erasure::Foreign { slot } | Erasure::Saved { slot, .. }) => Some(slot),
                Ok(Erasure::Idle) | Err(_) => None, // an error: the power went
            };
            assert!(
                erased.is_none() || erased != open,
                "the eraser took the open block, slot {open:?}"
            );
            self.cut_erase |= self.cut == Some(before) && self.flash.erases() > erases;
        }
        self.cut.is_some_and(|operation| operations(self.flash) > operation)
    }
}

/// A record a producer queued: its line's payload, and whether the stage was in flight when it was queued.
#[derive(Clone, Debug)]
struct Queued {
    /// The payload, as the decoder prints it.
    payload: String,
    /// Queued between BOOSTING and DONE.
    flight: bool,
}

/// What a boot queued, per stream, in order.
#[derive(Debug, Default)]
struct Workload {
    /// The IMU rows the ring took (a refused push is not queued).
    imu: Vec<Queued>,
    /// The durable events the ring took.
    events: Vec<Queued>,
    /// The producer's log lines the ring took.
    logs: Vec<Queued>,
}

/// Where each line of a ring image lies: the read-back of one moment.
#[derive(Clone, Debug, Default)]
struct Snapshot {
    /// Each line, by (routing, payload; None: recorder.log): its block's sequence and flight flag.
    placed: BTreeMap<(Option<String>, String), (u32, bool)>,
    /// The sequences of boot 123's blocks in flash.
    blocks: BTreeSet<u32>,
}

/// Where each line of boot 123 lies in `image`: its blocks in sequence order through one decoder, as the
/// ring_saver reads them.
fn snapshot(image: &[u8]) -> Snapshot {
    let all = blocks(image, geometry());
    let mut decoder = Decoder::new();
    let mut snapshot = Snapshot::default();
    for (index, block) in all.iter().enumerate() {
        if block.header.session.as_str() != SESSION {
            continue;
        }
        let boot_goes_on = all[index + 1..]
            .iter()
            .any(|later| later.header.boot_id == block.header.boot_id);
        let place = (block.header.block_sequence, block.header.flight);
        decoder
            .block(slot_bytes(image, block.slot), boot_goes_on, |line| {
                let body = line.body().strip_suffix(b"\n").unwrap();
                let payload = recorder_wire::verify(line.routing.map(str::as_bytes), body).unwrap();
                let key = (
                    line.routing.map(String::from),
                    String::from_utf8(payload.to_vec()).unwrap(),
                );
                assert!(snapshot.placed.insert(key, place).is_none(), "a line read twice");
            })
            .unwrap();
        snapshot.blocks.insert(block.header.block_sequence);
    }
    snapshot
}

/// What boot 123 left.
struct FirstBoot {
    /// The flash content after the cut, or after the whole run.
    image: Vec<u8>,
    /// What its producers queued.
    workload: Workload,
    /// The index of the tick the cut hit; None without a cut.
    cut_tick: Option<usize>,
    /// The losses the eraser reported.
    losses: Vec<u32>,
    /// Whether the cut tore an erase.
    cut_erase: bool,
    /// The writer's open block when the board died (or ended), as the table names it: (slot, sequence).
    open: Option<(u32, u32)>,
    /// Without a cut: the read-back after every tick.
    snapshots: Vec<Snapshot>,
    /// Program and erase calls made.
    operations: u32,
}

/// Boot 123 over an erased flash, with the power cut at operation `cut` torn at its given length (None: never).
/// Each producing step queues twelve IMU rows, a log line every third step, and every seventh step first a durable
/// event, which wakes the writer at once. The eraser steps every third step, and every tick while settling.
fn first_boot(cut: Option<(u32, usize)>) -> FirstBoot {
    let flash = MemoryFlash::new(geometry().bytes());
    if let Some((operation, torn)) = cut {
        flash.cut_power(operation, torn);
    }
    let scanned = bring_up(&flash, geometry());
    let boot = Boot::new(BOOT_ID);
    let (recorder, clock) = (boot.parts.recorder, &boot.parts.clock);
    let mut board = Board {
        flash: &flash,
        writer: Writer::new(recorder, ring(&flash, &scanned, BOOT_ID)).unwrap(),
        eraser: eraser(&flash, &scanned),
        cut: cut.map(|(operation, _)| operation),
        losses: Vec::new(),
        cut_erase: false,
    };
    let mut imu = Telemetry::new(recorder, boot.imu).unwrap();
    let mut events = Telemetry::new(recorder, boot.events).unwrap();
    let mut workload = Workload::default();
    let mut snapshots = Vec::new();
    let mut cut_tick = None;
    let mut ticks = 0;
    let mut tick = |board: &mut Board, now_us: u64, erase: bool| -> bool {
        let dead = board.tick(clock, now_us, erase);
        if dead {
            cut_tick = Some(ticks);
        } else if cut.is_none() {
            snapshots.push(snapshot(&board.flash.image()));
        }
        ticks += 1;
        dead
    };
    for step in 0..PRODUCING_STEPS + SETTLING_STEPS {
        let now = START_US + step * TICK_US;
        if step == FLIGHT.start {
            recorder.set_flight(true); // BOOSTING
        }
        if step == FLIGHT.end {
            recorder.set_flight(false); // DONE
        }
        let flight = FLIGHT.contains(&step);
        if step < PRODUCING_STEPS {
            if step % 7 == 3 {
                let uptime_us = now - 9500;
                let cells = [Cell::Text("mark")];
                if events.push(uptime_us, &cells).is_ok() {
                    let payload = row_text(uptime_us, &cells);
                    workload.events.push(Queued { payload, flight });
                    // the wake: a writer step of its own, the eraser's thread not woken
                    if tick(&mut board, uptime_us, false) {
                        break;
                    }
                }
            }
            for row in 0..ROWS_PER_STEP {
                let uptime_us = now - 8800 + row * 600;
                let index = (step * ROWS_PER_STEP + row) as i32;
                let cells = [Cell::I32(index), Cell::Text(PAD)];
                if imu.push(uptime_us, &cells).is_ok() {
                    let payload = row_text(uptime_us, &cells);
                    workload.imu.push(Queued { payload, flight });
                }
            }
            if step % 3 == 0 {
                let uptime_us = now - 5000;
                clock.set(uptime_us);
                let message = format!("step {step}");
                if recorder.log("producer", &message) {
                    let mut line = Vec::new();
                    recorder_wire::log_line(&mut line, uptime_us, "producer", &message, usize::MAX);
                    let payload = String::from_utf8(line).unwrap();
                    workload.logs.push(Queued { payload, flight });
                }
            }
        }
        let erase = step >= PRODUCING_STEPS || step % ERASE_EVERY == 0;
        if tick(&mut board, now, erase) {
            break;
        }
    }
    let table = board.writer.ring().table();
    let open = open_slot(table).map(|slot| (slot, table.slot(slot).block_sequence().unwrap()));
    let (losses, cut_erase) = (board.losses.clone(), board.cut_erase);
    drop(board);
    FirstBoot {
        operations: operations(&flash),
        image: flash.into_image(),
        workload,
        cut_tick,
        losses,
        cut_erase,
        open,
        snapshots,
    }
}

/// The streams of a workload: (routing, header line, what was queued).
fn streams(workload: &Workload) -> [(Option<String>, Option<&str>, &[Queued]); 3] {
    [
        (
            Some(format!("{SESSION}_imu.csv")),
            Some("uptime;index;pad"),
            &workload.imu,
        ),
        (
            Some(format!("{SESSION}_flight_events.csv")),
            Some("uptime;event"),
            &workload.events,
        ),
        (None, None, &workload.logs),
    ]
}

/// The queue index of each line of a stream that `read` holds: whole, in order and without a duplicate, at most
/// two runs of the queue (the one with the flight, and the newest records), under its header only when that
/// leads. Panics otherwise.
fn surviving(
    read: &ReadBack,
    routing: &Option<String>,
    header: Option<&str>,
    queued: &[Queued],
    context: &str,
) -> Vec<usize> {
    let index_of: BTreeMap<&str, usize> = queued
        .iter()
        .enumerate()
        .map(|(index, record)| (record.payload.as_str(), index))
        .collect();
    let mut lines = read.file(routing.as_deref());
    if routing.is_none() {
        lines.retain(|line| line.contains(" producer :: "));
    }
    if header.is_some() && lines.first().map(String::as_str) == header {
        lines.remove(0); // the stream's first block survived; with no repeat in 40 KiB, only it holds the header
    }
    let indexes: Vec<usize> = lines
        .iter()
        .map(|line| {
            *index_of
                .get(line.as_str())
                .unwrap_or_else(|| panic!("{context}: {line:?} is not a record queued whole"))
        })
        .collect();
    assert!(
        indexes.windows(2).all(|pair| pair[0] < pair[1]),
        "{context}: in order, no duplicate: {indexes:?}"
    );
    let runs = 1 + indexes.windows(2).filter(|pair| pair[1] != pair[0] + 1).count();
    assert!(
        indexes.is_empty() || runs <= 2,
        "{context}: {runs} runs, more than the flight's and the newest: {indexes:?}"
    );
    indexes
}

/// The whole run's own checks: the ring wrapped at least twice, the writer was refused, the eraser took only pad
/// blocks, and every record queued in flight is in flash, in a flight block.
fn check_whole(whole: &FirstBoot) {
    let read = read_back(&whole.image, geometry(), Some(SESSION));
    assert_eq!(read.lost, 0, "nothing torn without a cut");
    let opened = whole.snapshots.last().unwrap().blocks.len() + whole.losses.len();
    assert!(
        opened >= 2 * geometry().blocks() as usize,
        "wrapped at least twice: {opened} blocks"
    );
    assert!(
        read.file(None)
            .iter()
            .any(|line| line.contains("no erased block to open yet")),
        "the writer was refused for want of an erased block"
    );
    assert!(
        (whole.workload.imu.len() as u64) < PRODUCING_STEPS * ROWS_PER_STEP,
        "the producers were refused at push while the writer waited"
    );
    let last = whole.snapshots.last().unwrap();
    for (routing, header, queued) in streams(&whole.workload) {
        let context = format!("whole run, {routing:?}");
        let indexes = surviving(&read, &routing, header, queued, &context);
        if header == Some("uptime;index;pad") {
            // the IMU rows fill every block; a sparse stream's newest record may lie in a pad block given up
            assert_eq!(indexes.last(), Some(&(queued.len() - 1)), "{context}: the newest row");
        }
        for record in queued.iter().filter(|record| record.flight) {
            let place = last.placed.get(&(routing.clone(), record.payload.clone()));
            assert!(
                matches!(place, Some((_, true))),
                "{context}: {} is in a flight block: {place:?}",
                record.payload
            );
        }
        assert!(
            queued.iter().any(|record| record.flight),
            "{context}: the flight queued records"
        );
    }
}

/// Check boot 123's read-back after a cut against the whole run.
fn check_cut(whole: &FirstBoot, cut: &FirstBoot, context: &str) -> ReadBack {
    let read = read_back(&cut.image, geometry(), Some(SESSION)); // every line verifies, or this panics
    let losses = read
        .file(None)
        .iter()
        .filter(|line| line.contains(" ring :: block "))
        .count();
    assert_eq!(losses as u32, read.lost, "{context}: a loss line per torn chunk lost");
    let tick = cut.cut_tick.unwrap_or_else(|| panic!("{context}: the cut never came"));
    let before = tick
        .checked_sub(1)
        .map_or_else(Snapshot::default, |last| whole.snapshots[last].clone());
    let after = snapshot(&cut.image);
    // nothing missing but one pad block that the cut tick's eraser took, or whose erase the cut tore
    let mut gone: BTreeMap<u32, bool> = BTreeMap::new();
    for (key, &(block_sequence, flight)) in &before.placed {
        if !after.placed.contains_key(key) {
            gone.insert(block_sequence, flight);
        }
    }
    assert!(gone.len() <= 1, "{context}: lines of {gone:?} went");
    for (&block_sequence, &flight) in &gone {
        assert!(!flight, "{context}: flight block {block_sequence} went");
        assert!(
            !after.blocks.contains(&block_sequence),
            "{context}: block {block_sequence} is whole"
        );
        assert!(
            cut.losses.contains(&block_sequence) || cut.cut_erase,
            "{context}: block {block_sequence} went with no erase"
        );
    }
    for (routing, header, queued) in streams(&cut.workload) {
        let indexes = surviving(&read, &routing, header, queued, &format!("{context} {routing:?}"));
        // what the cut tick added is newer than everything before it
        let newest_before = queued
            .iter()
            .rposition(|record| before.placed.contains_key(&(routing.clone(), record.payload.clone())));
        for &index in &indexes {
            let key = (routing.clone(), queued[index].payload.clone());
            if !before.placed.contains_key(&key) {
                assert!(
                    newest_before.is_none_or(|newest| index > newest),
                    "{context} {routing:?}: record {index} came back, older than {newest_before:?}"
                );
            }
        }
    }
    // every block the boot opened is in flash or reported lost, but for the one whose erase the cut tore
    let lost: BTreeSet<u32> = cut.losses.iter().copied().collect();
    assert!(
        lost.is_disjoint(&after.blocks),
        "{context}: a block reported lost is in flash"
    );
    let accounted: BTreeSet<u32> = lost.union(&after.blocks).copied().collect();
    let unaccounted: Vec<u32> = accounted.last().map_or_else(Vec::new, |&newest| {
        (0..=newest).filter(|sequence| !accounted.contains(sequence)).collect()
    });
    assert!(
        unaccounted.is_empty() || (unaccounted.len() == 1 && cut.cut_erase),
        "{context}: blocks {unaccounted:?} neither in flash nor reported lost"
    );
    if let Some((slot, block_sequence)) = cut.open {
        assert!(
            after.blocks.contains(&block_sequence),
            "{context}: the open block {block_sequence} (slot {slot}) is not in flash"
        );
    }
    read
}

/// Boot 124 over what boot 123 left: ten steps of a short row each, the eraser stepping every tick. Checks that it
/// opens the block after boot 123's newest and reads back whole, and that newest wins gave up only boot 123's pad
/// blocks for it, never a line of the flight.
fn check_next_boot(image: Vec<u8>, before: &ReadBack, workload: &Workload, context: &str) {
    let flash = MemoryFlash::from_image(image);
    let scanned = bring_up(&flash, geometry());
    let boot = Boot::new(NEXT_BOOT);
    let (recorder, clock): (&'static Recorder, &TestClock) = (boot.parts.recorder, &boot.parts.clock);
    let mut board = Board {
        flash: &flash,
        writer: Writer::new(recorder, ring(&flash, &scanned, NEXT_BOOT)).unwrap(),
        eraser: eraser(&flash, &scanned),
        cut: None,
        losses: Vec::new(),
        cut_erase: false,
    };
    let mut imu = Telemetry::new(recorder, boot.imu).unwrap();
    let mut rows = vec![String::from("uptime;index;pad")];
    for step in 0..16 {
        let now = START_US + step * TICK_US;
        if step < 10 {
            let cells = [Cell::I32(step as i32), Cell::Text("x")];
            imu.push(now - 5000, &cells).unwrap();
            rows.push(row_text(now - 5000, &cells));
        }
        board.tick(clock, now, true);
    }
    assert_eq!(recorder.stats().telemetry.count, 0, "{context}: boot 124 settled");
    drop(board);
    let image = flash.into_image();
    let next = read_back(&image, geometry(), Some("000124"));
    assert_eq!(next.lost, 0, "{context}: boot 124 tore nothing");
    let first = before.blocks.last().map_or(0, |newest| newest + 1);
    assert_eq!(
        next.blocks.first(),
        Some(&first),
        "{context}: the block after boot 123's newest"
    );
    assert_eq!(next.file(Some("000124_imu.csv")), rows, "{context}: boot 124 whole");
    // boot 123: lines only went (whole blocks, pad first), none changed, and every flight line stayed. A loss line
    // is the decoder's, stamped with the newest uptime it read in the boot, so it is compared without its stamp:
    // once newest wins gave up the block before the torn one, an older uptime is the newest left
    let unstamped = |line: &String| -> String {
        match line.split_once(" ring :: block ") {
            Some((_, loss)) => format!("ring :: block {loss}"),
            None => line.clone(),
        }
    };
    let after = read_back(&image, geometry(), Some(SESSION));
    for (routing, lines) in &after.files {
        let earlier = before.files.get(routing).cloned().unwrap_or_default();
        let kept: BTreeSet<String> = earlier.iter().map(unstamped).collect();
        assert!(
            lines.iter().all(|line| kept.contains(&unstamped(line))),
            "{context}: boot 124 changed boot 123's {routing:?}"
        );
    }
    for (routing, _, queued) in streams(workload) {
        let earlier: BTreeSet<String> = before.file(routing.as_deref()).into_iter().collect();
        let now: BTreeSet<String> = after.file(routing.as_deref()).into_iter().collect();
        for record in queued
            .iter()
            .filter(|record| record.flight && earlier.contains(&record.payload))
        {
            assert!(
                now.contains(&record.payload),
                "{context}: boot 124 gave up a flight line of boot 123 while pad blocks were left: {}",
                record.payload
            );
        }
    }
}

#[test]
fn a_cut_while_newest_wins_erases_next_to_a_refused_writer_loses_only_what_it_must() {
    let whole = first_boot(None);
    assert_eq!(whole.cut_tick, None);
    check_whole(&whole);
    let read = read_back(&whole.image, geometry(), Some(SESSION));
    check_next_boot(whole.image.clone(), &read, &whole.workload, "whole run");
    let (mut torn_erases, mut cuts_with_losses) = (0, 0);
    for operation in 0..whole.operations {
        for torn in TEARS {
            let context = format!("cut at operation {operation}, {torn} bytes");
            let cut = first_boot(Some((operation, torn)));
            let read = check_cut(&whole, &cut, &context);
            torn_erases += u32::from(cut.cut_erase);
            cuts_with_losses += u32::from(!cut.losses.is_empty());
            check_next_boot(cut.image, &read, &cut.workload, &context);
        }
    }
    eprintln!(
        "full ring: {} operations, {} tears each; {torn_erases} cuts tore an erase, {cuts_with_losses} came after \
         newest wins erased",
        whole.operations,
        TEARS.len()
    );
    assert!(whole.operations >= 100, "{} operations", whole.operations);
    assert!(torn_erases > 0, "some cuts tore a newest-wins erase");
}
