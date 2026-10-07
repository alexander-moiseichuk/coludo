//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring_saver over an in-memory flash and a memory card (doc/specs/avionics-port.md "The ring_saver",
//! "Duplicates", "The ring_eraser and newest wins"), the writer, the eraser and the saver taking turns on the test's
//! clock:
//! * the card holds byte for byte what recorderd files from the decoder's lines of the whole ring, the global
//!   session.csv included, and so it does when the saver reads the writer's open block in many small parts;
//! * no read ever reaches past the writer's published position;
//! * batches: at most 64 blocks, at most 32 files, one boot each, and once a second when caught up;
//! * a card error keeps the batch: failed at each call of a batch, with the card powered or its power cut and back,
//!   the card ends byte for byte as without it; a file locked for 10 s holds its batch and nothing reaches the card
//!   twice; a dead card costs no flash read and no decode again;
//! * a reboot saves again from the published position: duplicates only, and never a header line again into a file
//!   that holds it, while a label reused by the next boot still shows its splice; the global session.csv gets one
//!   header a boot when a cut falls in a boot's first second;
//! * a line a cut tore is ended before the next boot's append, so it stays one damaged line of its own, and a
//!   fragment of a new file's first line is no line: the file still gets its header;
//! * the eraser erases behind the saver 100-200 ms after it passed, and newest wins never fires while it keeps up;
//! * the status line and `stats` count the backlog against the saver's position, and the measured write rate turns
//!   it into `ring_unsaved_s`.
//!
//! The host tools' verdict on the cards (recorder_flight, assemble_capture, flight_telemetry) is in saver_tools.rs.

mod common;

use common::board::{Board, Flight, Plan, Streams, flight_registry};
use common::card::{Card, CardError, Keep, contains, headers, lines, payload, unique};
use common::*;

use std::cell::Cell as Counter;
use std::collections::{BTreeMap, BTreeSet};

use flash_ring::header::Session;
use flash_ring::{BLOCK_BYTES, BlockTable, Flash, Geometry, Hints, MemoryError, MemoryFlash, Position, scan};
use recorder::{
    BATCH_BLOCKS, BATCH_FILES, BATCH_MS, Cell, Decoder, Recorder, Registry, SaveError, Saver, TICK_US, Telemetry,
    Writer,
};

/// A flight that boosts 3 s in, lands at 6 s and is done at 7 s, with the IMU at 100 Hz throughout.
const PLAN: Plan = Plan {
    boost: 300,
    land: 600,
    done: 700,
    pad_imu_every: 1,
};

/// A pad that never boosts.
const PAD: Plan = Plan {
    boost: 1 << 40,
    land: 1 << 41,
    done: 1 << 42,
    pad_imu_every: 1,
};

/// Boot `boot`'s recorder for a flight: the test thread queues through the 256 KiB `misc` ring.
fn flight_recorder(boot: u32) -> (Parts, Streams) {
    let (registry, streams) = flight_registry();
    (recorder_of(boot, registry, 0, 256 << 10, SETTINGS, false), streams)
}

/// What recorderd files from the decoder's lines of every block of this layout in `image`, by the saver's routing
/// as the spec states it: a stream line to `<session>/<routing>`, a session.csv line there and to the global
/// `session.csv`, any other line to `<session>/recorder.log`. Path to bytes.
fn routed(image: &[u8], geometry: Geometry) -> BTreeMap<String, Vec<u8>> {
    let all = blocks(image, geometry);
    let mut decoder = Decoder::new();
    let mut files: BTreeMap<String, Vec<u8>> = BTreeMap::new();
    for (index, block) in all.iter().enumerate() {
        let same_boot = |later: &Block| {
            later.header.boot_id == block.header.boot_id && later.header.session == block.header.session
        };
        let boot_goes_on = all[index + 1..].iter().any(same_boot);
        decoder
            .block(slot_bytes(image, block.slot), boot_goes_on, |line| {
                let path = format!("{}/{}", line.session, line.routing.unwrap_or("recorder.log"));
                files.entry(path).or_default().extend_from_slice(line.body());
                if line.routing == Some("session.csv") {
                    files
                        .entry("session.csv".into())
                        .or_default()
                        .extend_from_slice(line.body());
                }
            })
            .unwrap();
    }
    files
}

/// Stops the status lines, so that the writer writes nothing more once its producers are done, and the saver can
/// catch up with it for good.
fn quiet(recorder: &Recorder) {
    assert!(recorder.set_stats_ms(3_600_000));
}

/// Asserts that `found` holds `expected`'s files byte for byte.
fn same_files(found: &BTreeMap<String, Vec<u8>>, expected: &BTreeMap<String, Vec<u8>>, context: &str) {
    assert_eq!(
        found.keys().collect::<Vec<_>>(),
        expected.keys().collect::<Vec<_>>(),
        "{context}: the files"
    );
    for (path, bytes) in expected {
        assert!(
            found[path] == *bytes,
            "{context}: {path} differs:\n{}\n--- expected ---\n{}",
            String::from_utf8_lossy(&found[path]),
            String::from_utf8_lossy(bytes)
        );
    }
}

#[test]
fn the_card_holds_byte_for_byte_what_recorderd_files_from_the_decoders_lines() {
    let geometry = Geometry::new(128).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    board.erasing = false; // the image then keeps every block, for the decoder's own read-back
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    for tick in 0..900 {
        flight.fly(&mut board, tick);
    }
    quiet(parts.recorder);
    board.settle(300);
    same_files(&card.files(), &routed(&flash.image(), geometry), "one run");
    let imu = card.file("000123/000123_imu_lsm6dso32.csv");
    assert_eq!(
        lines(&imu).len() as u32,
        flight.rows[0] + 1,
        "the header, then every row once"
    );
    assert_eq!(
        payload("000123/000123_imu_lsm6dso32.csv", lines(&imu)[0]),
        Some(&b"uptime;ax;ay;az;gx;gy;gz;irq_runs"[..]),
        "it opens with its header"
    );
    assert_eq!(headers("000123/000123_imu_lsm6dso32.csv", &imu), 1);
    assert_eq!(board.card_errors, 0);
    assert!(
        board.commits.len() >= 9,
        "about one batch a second: {}",
        board.commits.len()
    );
    assert_eq!(
        parts.recorder.saver_position(),
        parts.recorder.writer_position(),
        "caught up once the writer is quiet"
    );
    assert_eq!(
        board.saver.as_ref().unwrap().position(),
        parts.recorder.saver_position().unwrap()
    );
}

/// The saver's flash handle in the read test: every read is checked against the writer's published position.
struct Spy<'a> {
    /// The flash.
    flash: &'a MemoryFlash,
    /// The table, to tell which block a slot holds.
    table: &'a BlockTable,
    /// The recorder whose writer position bounds the reads.
    recorder: &'static Recorder,
    /// Reads so far.
    reads: Counter<u32>,
}

impl Flash for &Spy<'_> {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        let writer = self
            .recorder
            .writer_position()
            .expect("no read before the writer publishes");
        let (slot, within) = (offset / BLOCK_BYTES, offset % BLOCK_BYTES);
        let block_sequence = self
            .table
            .slot(slot)
            .block_sequence()
            .expect("the saver reads blocks only");
        assert!(
            block_sequence <= writer.block_sequence,
            "block {block_sequence} is past the writer's {writer:?}"
        );
        if block_sequence == writer.block_sequence {
            assert!(
                within as usize + buffer.len() <= writer.offset as usize,
                "a read of {} bytes at {within} of the writer's block, published to {writer:?}",
                buffer.len()
            );
        }
        self.reads.set(self.reads.get() + 1);
        self.flash.read(offset, buffer)
    }

    fn program(&mut self, offset: u32, _data: &[u8]) -> Result<(), MemoryError> {
        panic!("the saver programmed offset {offset}");
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        panic!("the saver erased offset {offset}");
    }

    fn size(&self) -> u32 {
        self.flash.size()
    }
}

#[test]
fn the_saver_reads_the_open_block_in_parts_and_never_past_the_writer_position() {
    let geometry = Geometry::new(64).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let mut writer = Writer::new(parts.recorder, ring(&flash, &scanned, BOOT_ID)).unwrap();
    let spy = Spy {
        flash: &flash,
        table: &scanned.table,
        recorder: parts.recorder,
        reads: Counter::new(0),
    };
    let card = Card::new();
    let start = scanned.table.saver_start(None);
    let mut saver = Saver::new(parts.recorder, &spy, &scanned.table, card.clone(), start);
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    let mut now_us = START_US;
    for tick in 0..1200 {
        now_us += TICK_US;
        parts.clock.set(now_us);
        if tick < 900 {
            flight.queue(now_us, tick);
        } else if tick == 900 {
            quiet(parts.recorder);
        }
        writer.step(now_us, EPOCH);
        // the saver every tick: the writer's open block is read in many small parts
        while saver.step((now_us / 1000) as u32).unwrap().is_some() {}
        let (saver_at, writer_at) = (saver.position(), parts.recorder.writer_position().unwrap());
        assert!(
            saver_at <= writer_at,
            "tick {tick}: the saver {saver_at:?} past the writer {writer_at:?}"
        );
    }
    assert!(
        spy.reads.get() > 900,
        "a read a tick while the writer writes: {}",
        spy.reads.get()
    );
    same_files(&card.files(), &routed(&flash.image(), geometry), "parts");
}

#[test]
fn a_backlog_goes_in_batches_of_64_blocks_and_a_caught_up_saver_commits_once_a_second() {
    let geometry = Geometry::new(256).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, None, None);
    let pad = Plan {
        boost: 1 << 40,
        land: 1 << 41,
        done: 1 << 42,
        pad_imu_every: 1,
    };
    let mut flight = Flight::new(parts.recorder, streams, pad);
    for tick in 0..9000 {
        flight.fly(&mut board, tick);
    }
    let backlog = board.table().figures(None).unsaved;
    assert!(backlog > 2 * BATCH_BLOCKS, "a backlog of {backlog} blocks");
    assert!(board.losses.is_empty(), "no newest wins in 256 blocks");
    let card = Card::new();
    board.mount(card.clone(), None); // a new card: everything still in flash
    for tick in 9000..10000 {
        flight.fly(&mut board, tick);
    }
    let commits = &board.commits;
    assert!(
        commits[..2].iter().all(|commit| commit.saved.blocks == BATCH_BLOCKS),
        "full batches while behind: {commits:?}"
    );
    assert_eq!(
        commits[2].at_us, commits[0].at_us,
        "back to back, the rest of the backlog with them"
    );
    assert!(commits.iter().all(|commit| commit.saved.blocks <= BATCH_BLOCKS));
    let caught_up: Vec<u64> = commits
        .iter()
        .filter(|commit| commit.saved.blocks < BATCH_BLOCKS)
        .map(|commit| commit.at_us)
        .collect();
    assert!(caught_up.len() >= 5, "{caught_up:?}");
    for pair in caught_up[1..].windows(2) {
        assert!(
            pair[1] - pair[0] >= u64::from(BATCH_MS) * 1000,
            "a caught-up saver commits once a second: {caught_up:?}"
        );
    }
    let lines_saved: u32 = commits.iter().map(|commit| commit.saved.lines).sum();
    assert!(lines_saved > flight.rows[0], "every IMU row and more");
}

#[test]
fn a_batch_touches_at_most_32_files_and_the_block_that_would_pass_them_starts_the_next() {
    let geometry = Geometry::new(256).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let mut registry = Registry::new();
    let streams: Vec<_> = (0..40)
        .map(|index| {
            registry
                .add(&format!("s{index:02}.csv"), &["value"], -1, false)
                .unwrap()
        })
        .collect();
    let parts = recorder_of(BOOT_ID, registry, 0, 256 << 10, SETTINGS, false);
    let mut telemetry: Vec<Telemetry> = streams
        .iter()
        .map(|&stream| Telemetry::new(parts.recorder, stream).unwrap())
        .collect();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, None, None);
    for tick in 0..3000u64 {
        // eight streams a tick, the window moving one stream every 20 ticks: each block names about ten files
        let window = (tick / 20) as usize;
        for offset in 0..8 {
            let index = (window + offset) % streams.len();
            telemetry[index]
                .push(board.now_us + TICK_US, &[Cell::I32(tick as i32)])
                .unwrap();
        }
        board.tick();
    }
    board.mount(Card::new(), None);
    board.settle(300);
    let commits = &board.commits;
    assert!(commits.len() > 4, "{commits:?}");
    assert!(
        commits.iter().all(|commit| commit.saved.files as usize <= BATCH_FILES),
        "{commits:?}"
    );
    assert!(
        commits
            .iter()
            .filter(|commit| commit.saved.blocks < BATCH_BLOCKS)
            .count()
            > 3,
        "batches end short of 64 blocks at the file bound: {commits:?}"
    );
    let fullest = commits.iter().map(|commit| commit.saved.files).max().unwrap();
    assert!(fullest + 2 >= BATCH_FILES as u32, "and come near it: {fullest}");
}

#[test]
fn a_batch_holds_one_boot_and_a_new_card_gets_every_boot_still_in_flash() {
    let geometry = Geometry::new(64).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, None, None);
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    for tick in 0..400 {
        flight.fly(&mut board, tick);
    }
    drop(board);
    // boot 124 brings the ring up over boot 123's blocks, a new card in the slot
    let flash = MemoryFlash::from_image(flash.into_image());
    let scanned = scan(&flash, geometry, Hints::default()).unwrap();
    let (parts, streams) = flight_recorder(BOOT_ID + 1);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID + 1, Some(card.clone()), None);
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    let mut tick = 0;
    while board.commits.is_empty() {
        flight.fly(&mut board, tick);
        tick += 1;
    }
    let first: BTreeSet<String> = card.files().into_keys().collect();
    assert!(
        first
            .iter()
            .all(|path| path.starts_with("000123/") || path == "session.csv"),
        "the first batch is boot 123's alone: {first:?}"
    );
    let position = board.commits[0].saved.position;
    assert_eq!(
        position,
        Position {
            block_sequence: scanned.next_sequence,
            offset: 0
        },
        "it ends where 124 starts"
    );
    while board.commits.len() < 3 {
        flight.fly(&mut board, tick);
        tick += 1;
    }
    assert!(card.files().keys().any(|path| path.starts_with("000124/")));
    let index = card.file("session.csv");
    let boots: Vec<String> = lines(&index)
        .into_iter()
        .filter_map(|line| payload("session.csv", line))
        .map(|payload| String::from_utf8_lossy(payload).into_owned())
        .filter(|row| row.ends_with(";boot;;"))
        .collect();
    assert_eq!(boots.len(), 2, "the global index holds both boots' rows: {boots:?}");
}

/// `flight`'s `ticks` on `board`.
fn fly(flight: &mut Flight, board: &mut Board<'_>, ticks: std::ops::Range<u64>) {
    for tick in ticks {
        flight.fly(board, tick);
    }
}

/// How a card call of a batch fails: the call, what it leaves of a write, and whether the power goes (the board
/// outliving it, the card back a few steps later) or the call alone fails with the card powered.
type Fault = (u32, Keep, bool);

/// Boot 123's flight with the card in and nothing erased, `fault` striking the batch after tick 350; then on to the
/// end, quiet, until the saver has caught up. Returns the card, the flash image, the card errors the saver met and
/// the calls the faulted batch made (all of them when there was no fault).
fn faulted_run(geometry: Geometry, fault: Option<Fault>) -> (Card, Vec<u8>, u32, u32) {
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    board.erasing = false;
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    fly(&mut flight, &mut board, 0..350);
    let (operations, committed) = (card.operations(), board.commits.len());
    match fault {
        Some((at, keep, true)) => card.cut_after(at, keep),
        Some((at, keep, false)) => card.fail_after(at, keep),
        None => {}
    }
    let mut tick = 350;
    while board.commits.len() == committed {
        flight.fly(&mut board, tick);
        tick += 1;
        if card.dead() && board.card_errors >= 3 {
            card.power_on();
        }
    }
    let calls = card.operations() - operations;
    fly(&mut flight, &mut board, tick..900);
    quiet(parts.recorder);
    board.settle(300);
    let errors = board.card_errors;
    drop(board);
    (card, flash.into_image(), errors, calls)
}

#[test]
fn a_card_error_keeps_the_batch_and_appends_only_what_did_not_land() {
    let geometry = Geometry::new(128).unwrap();
    let (_, _, errors, calls) = faulted_run(geometry, None);
    assert_eq!(errors, 0);
    assert!(calls >= 10, "a batch of several files: {calls} calls");
    let keeps = [Keep::Synced, Keep::Written, Keep::Torn(10)];
    for at in 0..calls {
        for (keep, cut) in keeps.into_iter().flat_map(|keep| [(keep, false), (keep, true)]) {
            let context = format!("call {at} of {calls}, {keep:?}, power cut {cut}");
            let (card, image, errors, _) = faulted_run(geometry, Some((at, keep, cut)));
            assert!(errors >= 1, "{context}: the card failed");
            // byte for byte: the files done before the error not again, the one that failed resumed where it
            // stopped (a torn line completed, not ended), nothing duplicated and nothing left out
            same_files(&card.files(), &routed(&image, geometry), &context);
        }
    }
}

#[test]
fn a_locked_file_holds_its_batch_and_nothing_reaches_the_card_twice() {
    let geometry = Geometry::new(128).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    board.erasing = false;
    let plan = Plan {
        boost: 1500,
        land: 1800,
        done: 1900,
        pad_imu_every: 1,
    };
    let mut flight = Flight::new(parts.recorder, streams, plan);
    fly(&mut flight, &mut board, 0..350);
    // an HTTP pull holds health.csv open for 10 s: FatFs refuses the saver's write-open, again and again
    card.lock(Some("000123/000123_health.csv"));
    let mut held = None;
    for tick in 350..1350 {
        flight.fly(&mut board, tick);
        if board.card_errors > 0 {
            let (position, files) = held.get_or_insert_with(|| (parts.recorder.saver_position(), card.files()));
            assert_eq!(
                *position,
                parts.recorder.saver_position(),
                "tick {tick}: the saver stays at its batch"
            );
            assert!(
                *files == card.files(),
                "tick {tick}: nothing goes on the card again while the file is locked"
            );
        }
    }
    assert!(
        board.card_errors >= 90,
        "every step refused for 10 s: {}",
        board.card_errors
    );
    card.lock(None);
    fly(&mut flight, &mut board, 1350..2200);
    quiet(parts.recorder);
    board.settle(300);
    assert!(board.losses.is_empty());
    assert_eq!(parts.recorder.saver_position(), parts.recorder.writer_position());
    same_files(&card.files(), &routed(&flash.image(), geometry), "after the lock");
}

#[test]
fn a_dead_card_costs_no_flash_read_and_no_decode_again() {
    let geometry = Geometry::new(256).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let mut writer = Writer::new(parts.recorder, ring(&flash, &scanned, BOOT_ID)).unwrap();
    let mut flight = Flight::new(parts.recorder, streams, PAD);
    let mut now_us = START_US;
    for tick in 0..9300 {
        now_us += TICK_US;
        parts.clock.set(now_us);
        if tick < 9000 {
            flight.queue(now_us, tick);
        } else if tick == 9000 {
            quiet(parts.recorder);
        }
        writer.step(now_us, EPOCH); // the last 3 s past the merge horizon: every row queued is in flash
    }
    let spy = Spy {
        flash: &flash,
        table: &scanned.table,
        recorder: parts.recorder,
        reads: Counter::new(0),
    };
    let card = Card::new();
    card.kill();
    let start = scanned.table.saver_start(None);
    let mut saver = Saver::new(parts.recorder, &spy, &scanned.table, card.clone(), start);
    let now_ms = (now_us / 1000) as u32;
    assert!(saver.step(now_ms).is_err(), "a full batch decoded, then refused");
    let decoded = spy.reads.get();
    assert!(decoded >= BATCH_BLOCKS, "{decoded} reads");
    for step in 1..=10 {
        assert!(matches!(
            saver.step(now_ms + step * 100),
            Err(SaveError::Card(CardError::PowerCut))
        ));
        assert_eq!(spy.reads.get(), decoded, "step {step}: nothing read or decoded again");
        assert_eq!(parts.recorder.saver_position(), Some(start), "step {step}");
    }
    // the card comes back: the batch decoded at first goes on whole, and the walk after it
    card.power_on();
    let saved = saver.step(now_ms + 2000).unwrap().unwrap();
    assert_eq!(saved.blocks, BATCH_BLOCKS);
    assert_eq!(spy.reads.get(), decoded, "the batch went from memory");
    for later in [2000, 2000 + BATCH_MS] {
        while saver.step(now_ms + later).unwrap().is_some() {} // the backlog, then the last part once it is due
    }
    assert_eq!(Some(saver.position()), parts.recorder.writer_position(), "caught up");
    let path = "000123/000123_imu_lsm6dso32.csv";
    let imu = card.file(path);
    assert_eq!(
        lines(&imu).len() as u32,
        flight.rows[0] + 1,
        "its header, then every row once"
    );
    assert!(unique(path, &imu) == imu, "no line twice");
}

#[test]
fn a_reboot_resumes_at_the_hint_and_a_reused_label_still_shows_its_splice() {
    let geometry = Geometry::new(128).unwrap();
    let label = Session::new("flightday").unwrap();
    let card = Card::new();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (registry, streams) = flight_registry();
    let parts = recorder_as(labelled(41, label), registry, 0, 256 << 10, SETTINGS, false);
    let mut board = Board::new(&flash, &scanned, &parts, 41, Some(card.clone()), None);
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    fly(&mut flight, &mut board, 0..450);
    // the power goes in the middle of a batch: the card keeps what it had fsynced
    card.cut_after(5, Keep::Synced);
    let mut tick = 450;
    while !card.dead() {
        flight.fly(&mut board, tick);
        tick += 1;
    }
    let hint = parts.recorder.saver_position();
    let rows_41 = flight.rows[0];
    drop(board);
    card.power_on();
    // boot 42 reuses the label (a stale recorder.session): the saver resumes at boot 41's hint on the same card
    let flash = MemoryFlash::from_image(flash.into_image());
    let hints = Hints {
        writer: None,
        saver: hint,
    };
    let scanned = scan(&flash, geometry, hints).unwrap();
    let (registry, streams) = flight_registry();
    let parts = recorder_as(labelled(42, label), registry, 0, 256 << 10, SETTINGS, false);
    // a later power-up: its uptimes are not boot 41's, or the two boots' rows would be one line twice
    parts.clock.set(START_US + 3_000_000);
    let mut board = Board::new(&flash, &scanned, &parts, 42, Some(card.clone()), hint);
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    fly(&mut flight, &mut board, 0..900);
    quiet(parts.recorder);
    board.settle(300);
    for (path, bytes) in card.files() {
        if path.starts_with("flightday/flightday_") {
            assert_eq!(
                headers(&path, &bytes),
                2,
                "{path}: boot 41's header, then boot 42's (the splice the tools flag); none printed again"
            );
        }
    }
    let imu = card.file("flightday/flightday_imu_lsm6dso32.csv");
    let kept = unique("flightday/flightday_imu_lsm6dso32.csv", &imu);
    assert!(kept.len() < imu.len(), "the cut batch went again: duplicates");
    assert_eq!(
        lines(&kept).len() as u32,
        rows_41 + flight.rows[0] + 2, // every row queued was programmed in its tick, before the cut
        "every row of both boots once, after the duplicate filter"
    );
}

/// Boot `boot` over `image`, the flash the last boot left, with `card` in the slot: the scan takes `hints`, the saver
/// resumes at `hints.saver` (None: at the oldest block, as for a card not trusted) and the eraser trusts it. It
/// records a pad of `ticks` ticks, then goes quiet until the saver has caught up. Returns the flash as it left it.
fn next_boot(image: Vec<u8>, geometry: Geometry, card: &Card, boot: u32, hints: Hints, ticks: u64) -> Vec<u8> {
    let flash = MemoryFlash::from_image(image);
    let scanned = scan(&flash, geometry, hints).unwrap();
    let (parts, streams) = flight_recorder(boot);
    // a later power-up: uptimes of its own, as a board's are
    parts.clock.set(START_US + u64::from(boot - BOOT_ID + 1) * 30_000_000);
    let mut board = Board::new(&flash, &scanned, &parts, boot, Some(card.clone()), hints.saver);
    let mut pad = Flight::new(parts.recorder, streams, PAD);
    fly(&mut pad, &mut board, 0..ticks);
    quiet(parts.recorder);
    board.settle(300);
    assert_eq!(
        parts.recorder.saver_position(),
        parts.recorder.writer_position(),
        "boot {boot} caught up"
    );
    assert!(board.losses.is_empty());
    drop(board);
    flash.into_image()
}

/// The hints a boot leaves in the cfg NVS as its power goes: both positions as published.
fn hints_of(recorder: &Recorder) -> Hints {
    Hints {
        writer: recorder.writer_position(),
        saver: recorder.saver_position(),
    }
}

#[test]
fn a_line_a_cut_tore_ends_before_the_next_boots_append_and_stays_one_damaged_line() {
    let geometry = Geometry::new(128).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    board.erasing = false;
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    fly(&mut flight, &mut board, 0..350);
    // the next batch's first write (after the directory and the first open) lands 10 bytes, then the power goes
    card.cut_after(2, Keep::Torn(10));
    let mut tick = 350;
    while !card.dead() {
        flight.fly(&mut board, tick);
        tick += 1;
    }
    let hints = hints_of(parts.recorder);
    drop(board);
    card.power_on();
    let torn: Vec<String> = card
        .files()
        .into_iter()
        .filter(|(_, bytes)| bytes.last().is_some_and(|&byte| byte != b'\n'))
        .map(|(path, _)| path)
        .collect();
    assert_eq!(torn.len(), 1, "one torn last line: {torn:?}");
    let image = flash.into_image();
    let expected = routed(&image, geometry); // boot 123's lines are all in it
    next_boot(image, geometry, &card, BOOT_ID + 1, hints, 100);
    for (path, bytes) in card.files() {
        if !path.starts_with("000123/") {
            continue;
        }
        let damaged: Vec<&[u8]> = lines(&bytes)
            .into_iter()
            .filter(|line| payload(&path, line).is_none())
            .collect();
        if path == torn[0] {
            assert_eq!(damaged.len(), 1, "{path}: the fragment, a line of its own");
            assert_eq!(damaged[0].len(), 11, "its 10 bytes and the line end the saver gave it");
            let whole: Vec<u8> = lines(&bytes)
                .into_iter()
                .filter(|line| payload(&path, line).is_some())
                .collect::<Vec<_>>()
                .concat();
            assert!(
                unique(&path, &whole) == expected[&path],
                "{path}: every line whole besides"
            );
        } else {
            assert!(damaged.is_empty(), "{path}");
            assert!(unique(&path, &bytes) == expected[&path], "{path}");
        }
    }
}

#[test]
fn a_fragment_of_a_new_files_first_line_is_no_line_and_the_file_still_gets_its_header() {
    // boot 123's first batch, its power cut at each of its calls with 10 bytes of that write landed: a file the
    // batch created may hold only that fragment of its header line. Boot 124 resumes on the same card
    let geometry = Geometry::new(64).unwrap();
    let first_batch = |cut: Option<u32>| {
        let flash = MemoryFlash::new(geometry.bytes());
        let scanned = bring_up(&flash, geometry);
        let (parts, streams) = flight_recorder(BOOT_ID);
        let card = Card::new();
        if let Some(at) = cut {
            card.cut_after(at, Keep::Torn(10));
        }
        let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
        let mut flight = Flight::new(parts.recorder, streams, PLAN);
        let mut tick = 0;
        while board.commits.is_empty() && !card.dead() {
            flight.fly(&mut board, tick);
            tick += 1;
        }
        let hints = hints_of(parts.recorder);
        drop(board);
        (card, flash.into_image(), hints)
    };
    let (card, _, _) = first_batch(None);
    let calls = card.operations();
    assert!(calls >= 10, "{calls} calls");
    let mut fragments = 0;
    for at in 0..calls {
        let (card, image, hints) = first_batch(Some(at));
        card.power_on();
        fragments += card
            .files()
            .values()
            .filter(|bytes| !bytes.is_empty() && !bytes.contains(&b'\n'))
            .count();
        next_boot(image, geometry, &card, BOOT_ID + 1, hints, 100);
        for (path, bytes) in card.files() {
            if path.starts_with("000123/000123_") {
                assert_eq!(
                    headers(&path, &bytes),
                    1,
                    "cut at call {at}: {path}: one whole header line"
                );
            }
        }
    }
    assert!(fragments > 0, "some cut left a new file a fragment alone");
}

#[test]
fn the_global_index_gets_one_header_a_boot_when_a_cut_falls_in_a_boots_first_second() {
    let geometry = Geometry::new(128).unwrap();
    let card = Card::new();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    fly(&mut flight, &mut board, 0..400);
    let hints = hints_of(parts.recorder);
    drop(board);
    // boot 124 resumes at boot 123's hint, commits boot 123's tail, and its power goes 200 ms into its own first
    // batch: its hint names its own first block, offset 0, and nothing of boot 124 is on the card
    let flash = MemoryFlash::from_image(flash.into_image());
    let scanned = scan(&flash, geometry, hints).unwrap();
    let (parts, streams) = flight_recorder(BOOT_ID + 1);
    parts.clock.set(START_US + 30_000_000);
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID + 1, Some(card.clone()), hints.saver);
    let mut pad = Flight::new(parts.recorder, streams, PAD);
    let mut tick = 0;
    while board.commits.is_empty() {
        pad.fly(&mut board, tick);
        tick += 1;
    }
    let first = Position {
        block_sequence: scanned.next_sequence,
        offset: 0,
    };
    assert_eq!(board.commits[0].saved.position, first, "boot 123's tail");
    fly(&mut pad, &mut board, tick..tick + 20);
    assert_eq!(board.commits.len(), 1, "boot 124's first batch is still collecting");
    let hints = hints_of(parts.recorder);
    assert_eq!(hints.saver, Some(first));
    drop(board);
    next_boot(flash.into_image(), geometry, &card, BOOT_ID + 2, hints, 300);
    let index = card.file("session.csv");
    assert_eq!(
        headers("session.csv", &index),
        3,
        "one header line a boot, as recorderd wrote, none twice:\n{}",
        String::from_utf8_lossy(&index)
    );
    let boots = lines(&unique("session.csv", &index))
        .into_iter()
        .filter_map(|line| payload("session.csv", line))
        .filter(|row| row.ends_with(b";boot;;"))
        .count();
    assert_eq!(boots, 3, "each boot's boot row once");
    for session in ["000123", "000124", "000125"] {
        let path = format!("{session}/session.csv");
        assert_eq!(headers(&path, &card.file(&path)), 1, "{path}");
    }
}

#[test]
fn the_eraser_erases_behind_the_saver_100_to_200_ms_after_it_passed_and_loses_nothing() {
    let geometry = Geometry::new(32).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    let long_pad = Plan {
        boost: 5000,
        land: 5300,
        done: 5400,
        pad_imu_every: 1,
    };
    let mut flight = Flight::new(parts.recorder, streams, long_pad);
    fly(&mut flight, &mut board, 0..6000);
    quiet(parts.recorder);
    board.settle(300);
    assert!(
        board.losses.is_empty(),
        "a saver that keeps up leaves newest wins nothing: {:?}",
        board.losses
    );
    let written = parts.recorder.writer_position().unwrap().block_sequence;
    assert!(written > 4 * geometry.blocks(), "the ring wrapped {written} blocks");
    assert!(board.erased.len() as u32 > 3 * geometry.blocks());
    for &(block_sequence, erased_us) in &board.erased {
        let passed = board
            .commits
            .iter()
            .find(|commit| commit.saved.position.block_sequence > block_sequence)
            .unwrap_or_else(|| panic!("block {block_sequence} erased before the saver passed it"));
        // 100-200 ms once the eraser saw the position (its step after the saver's), then one block a step
        let after_us = erased_us - passed.at_us;
        assert!(
            (100_000..=250_000).contains(&after_us),
            "block {block_sequence} erased {after_us} us after the saver passed it"
        );
    }
    let imu = card.file("000123/000123_imu_lsm6dso32.csv");
    assert_eq!(
        lines(&imu).len() as u32,
        flight.rows[0] + 1,
        "every row on the card, once"
    );
}

#[test]
fn the_backlog_counts_against_the_saver_and_the_write_rate_makes_it_ring_unsaved_s() {
    let geometry = Geometry::new(128).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, None, None);
    let pad = Plan {
        boost: 1 << 40,
        land: 1 << 41,
        done: 1 << 42,
        pad_imu_every: 1,
    };
    let mut flight = Flight::new(parts.recorder, streams, pad);
    fly(&mut flight, &mut board, 0..1000);
    let stats = parts.recorder.stats();
    assert!(
        stats.ring.unsaved >= 15,
        "no card: every closed block unsaved: {stats:?}"
    );
    // about 100 IMU rows a second of ~80 bytes, health and the log lines: some 9 KB a second of flash
    assert!((7_000..12_000).contains(&stats.write_rate), "{stats:?}");
    let unsaved_s = stats.ring.unsaved_seconds(stats.write_rate as f32);
    let expected = stats.ring.unsaved as f32 * BLOCK_BYTES as f32 / stats.write_rate as f32;
    assert_eq!(unsaved_s, expected);
    assert!(
        (5.0..11.0).contains(&unsaved_s),
        "about the 10 s recorded so far: {unsaved_s}"
    );
    assert_eq!(parts.recorder.saver_position(), None);
    // a card goes in: the backlog empties, and the figures follow the saver
    let card = Card::new();
    board.mount(card.clone(), None);
    fly(&mut flight, &mut board, 1000..1500);
    let stats = parts.recorder.stats();
    assert!(stats.ring.unsaved <= 3, "the saver keeps up: {stats:?}");
    assert!(stats.ring.unsaved_seconds(stats.write_rate as f32) < 2.0, "{stats:?}");
    assert!(
        stats.ring.saved + stats.ring.erased_ahead >= geometry.blocks() - 5,
        "{stats:?}"
    );
    let log = card.file("000123/recorder.log");
    let status: Vec<&[u8]> = lines(&log)
        .into_iter()
        .filter_map(|line| payload("000123/recorder.log", line))
        .filter(|payload| contains(payload, b" Recorder :: "))
        .collect();
    let saver_named = |payload: &&[u8]| contains(payload, b"'saver': (");
    assert!(!saver_named(&status[0]), "no saver at first");
    assert!(
        saver_named(status.last().unwrap()),
        "the saver's position, once it runs: {}",
        String::from_utf8_lossy(status.last().unwrap())
    );
}
