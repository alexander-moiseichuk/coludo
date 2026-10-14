//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring_writer over an in-memory flash, step by step with the clock in the test's hand: the session.csv rows
//! and the status line, Telemetry's header and decimation, the log merge across rings, the chunks filling each
//! block, the flight flag, the CC tees, the wake of a durable record, back-pressure when no block is erased, a
//! failing flash, a failed program's retry and the duplicate it leaves, and the one-writer rule. Several threads
//! at once, the threaded CC tee and the host tools' verdict are in end_to_end.rs, power cuts in power_cut.rs and,
//! on a full ring under newest wins, in newest_wins_cut.rs, the header repeats and newest wins in repeats.rs.

mod common;

use common::*;

use flash_ring::chunk::{self, CHUNK_HEADER_BYTES, Walked, chunk_bytes};
use flash_ring::{BLOCK_BYTES, Geometry, MemoryFlash, Ring, Scan};
use recorder::{
    ANCHOR_AFTER_US, Cell, Decoder, MERGE_HORIZON_US, Overflow, Progress, Recorder, Registry, Settings, StreamId,
    TICK_US, Telemetry, Writer,
};

/// The test geometry: 32 blocks, 128 KiB.
fn geometry() -> Geometry {
    Geometry::new(32).unwrap()
}

/// The streams of [`registry`].
#[derive(Clone, Copy)]
struct Streams {
    /// `imu_lsm6dso32.csv`: the global decimation.
    imu: StreamId,
    /// `health.csv`: its own 1 s window.
    health: StreamId,
    /// `flight_events.csv`: never decimated, durable.
    events: StreamId,
}

/// A registry of three streams, one of each decimation kind.
fn registry() -> (Registry, Streams) {
    let mut registry = Registry::new();
    let streams = Streams {
        imu: registry
            .add("imu_lsm6dso32.csv", &["ax", "ay", "az"], 0, false)
            .unwrap(),
        health: registry.add("health.csv", &["temp", "load"], 1_000_000, false).unwrap(),
        events: registry.add("flight_events.csv", &["event"], -1, true).unwrap(),
    };
    (registry, streams)
}

/// The IMU file's routing.
const IMU: &str = "000123_imu_lsm6dso32.csv";

/// The events file's routing.
const EVENTS: &str = "000123_flight_events.csv";

/// The board's storage: an erased flash and its bring-up scan, which the writer and the eraser borrow.
struct Storage {
    /// The `log` partition.
    flash: MemoryFlash,
    /// The ring's shape.
    geometry: Geometry,
    /// The bring-up scan.
    scanned: Scan,
}

impl Storage {
    /// An erased flash of `geometry`, scanned.
    fn erased(geometry: Geometry) -> Storage {
        Storage::over(geometry, vec![0xFF; geometry.bytes() as usize])
    }

    /// A flash of `geometry` holding `image`, scanned.
    fn over(geometry: Geometry, image: Vec<u8>) -> Storage {
        let flash = MemoryFlash::from_image(image);
        let scanned = bring_up(&flash, geometry);
        Storage {
            flash,
            geometry,
            scanned,
        }
    }

    /// This boot's flash ring writer.
    fn ring(&self) -> Ring<'_, &MemoryFlash> {
        ring(&self.flash, &self.scanned, BOOT_ID)
    }

    /// This boot's flash ring writer, opened as a flight ring: the bring-up's after a reboot in flight, when the
    /// crumb names BOOSTING..LANDING.
    fn ring_in_flight(&self) -> Ring<'_, &MemoryFlash> {
        let session = session(BOOT_ID);
        let next = self.scanned.next_sequence;
        Ring::new(&self.flash, &self.scanned.table, next, BOOT_ID, session, true).unwrap()
    }

    /// The flash read back.
    fn read_back(&self) -> ReadBack {
        read_back(&self.flash.image(), self.geometry, None)
    }
}

/// A writer of `recorder` over `storage`.
fn writer<'a>(recorder: &'static Recorder, storage: &'a Storage) -> Writer<'static, 'a, &'a MemoryFlash> {
    Writer::new(recorder, storage.ring()).unwrap()
}

/// Step the writer at `now_us`, with the clock moved there too.
fn step(writer: &mut Writer<'static, '_, &MemoryFlash>, clock: &TestClock, now_us: u64) -> Progress {
    clock.set(now_us);
    writer.step(now_us, EPOCH)
}

/// Step once past the merge horizon: everything queued is then on flash.
fn settle(writer: &mut Writer<'static, '_, &MemoryFlash>, clock: &TestClock) {
    let now = clock.now();
    step(writer, clock, now + MERGE_HORIZON_US);
}

/// An IMU row of three floats.
fn imu_row(index: u32) -> [Cell<'static>; 3] {
    [Cell::F32(index as f32 * 0.5), Cell::F32(-1.0), Cell::F32(9.81)]
}

/// The session.csv header line.
const INDEX_HEADER: &str = "uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon";

#[test]
fn the_boot_row_comes_first_and_is_programmed_at_once() {
    let parts = recorder(registry().0, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    assert_eq!(
        parts.recorder.writer_position(),
        None,
        "nothing published before the first step"
    );
    let progress = step(&mut writer, &parts.clock, START_US);
    assert_eq!(
        progress,
        Progress { records: 2, chunks: 1 },
        "the header and the row, in one chunk"
    );
    assert_eq!(
        storage.read_back().file(Some("session.csv")),
        [
            INDEX_HEADER,
            "10000000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;boot;;"
        ]
    );
    let image = storage.flash.image();
    let block = blocks(&image, geometry())[0];
    let header = block.header;
    assert_eq!(
        (
            header.block_sequence,
            header.boot_id,
            header.session.as_str(),
            header.flight
        ),
        (0, BOOT_ID, SESSION, false)
    );
    let walked: Vec<Walked> = chunk::chunks(slot_bytes(&image, block.slot)).collect();
    let [Walked::Chunk(found)] = walked[..] else {
        panic!("one whole chunk: {walked:?}")
    };
    assert_eq!((found.offset, found.records), (56, 2));
    let position = parts.recorder.writer_position().unwrap();
    assert_eq!(position, writer.ring().position());
    assert_eq!(
        (position.block_sequence, position.offset as usize),
        (0, 56 + chunk_bytes(found.payload.len())),
        "the end of what is programmed"
    );
    let raw = raw_records(&image, geometry());
    assert_eq!(
        (raw[0].kind, raw[0].flags & 1, raw[1].kind),
        (1, 1, 2),
        "a durable HEADER, then the ROW"
    );
}

#[test]
fn the_boot_row_leads_even_records_queued_before_the_writer() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 0, 4096, SETTINGS);
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    imu.push(START_US - 2000, &imu_row(0)).unwrap();
    parts.clock.set(START_US - 1000);
    parts.recorder.index_session(EPOCH, None, "cc-auto", None).unwrap();
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    step(&mut writer, &parts.clock, START_US);
    settle(&mut writer, &parts.clock);
    let read = storage.read_back();
    let capture = String::from_utf8(read.capture.clone()).unwrap();
    let mut lines = capture.lines();
    assert!(lines.next().unwrap().starts_with("@session.csv@{"), "the header first");
    assert!(
        lines.next().unwrap().contains(";10000000;123;000123;"),
        "then the boot row"
    );
    let rows = read.file(Some("session.csv"));
    assert_eq!(rows.len(), 3);
    assert_eq!(
        rows[2],
        "9999000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;cc-auto;;"
    );
    assert_eq!(read.file(Some(IMU)), ["uptime;ax;ay;az", "9998000;0;-1;9.81"]);
}

#[test]
fn a_time_set_row_follows_the_boot_row_and_the_anchor_comes_a_minute_in() {
    let parts = recorder(registry().0, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    step(&mut writer, &parts.clock, START_US);
    parts.clock.set(START_US + 5_000_000);
    parts
        .recorder
        .index_session(EPOCH + 5, Some(-240), "cc-auto", Some((255_144_000, -803_918_000)))
        .unwrap();
    step(&mut writer, &parts.clock, START_US + 5_000_000);
    step(&mut writer, &parts.clock, START_US + ANCHOR_AFTER_US - 1);
    assert_eq!(
        storage.read_back().file(Some("session.csv")).len(),
        3,
        "no anchor before a minute"
    );
    step(&mut writer, &parts.clock, START_US + ANCHOR_AFTER_US);
    step(&mut writer, &parts.clock, START_US + 2 * ANCHOR_AFTER_US);
    let rows = storage.read_back().file(Some("session.csv"));
    assert_eq!(
        rows,
        [
            INDEX_HEADER,
            "10000000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;boot;;",
            "15000000;123;000123;2026-10-03T14:21:12Z;-240;taster;dev;3f2a91;cc-auto;25.5144000;-80.3918000",
            "70000000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;anchor;;",
        ],
        "one anchor per boot; its HEADER record is a repeat, which prints nothing"
    );
    let headers = raw_records(&storage.flash.image(), geometry())
        .into_iter()
        .filter(|record| record.kind == 1 && record.name == "session.csv")
        .map(|record| record.uptime_us)
        .collect::<Vec<u64>>();
    assert_eq!(
        headers,
        [START_US, START_US + ANCHOR_AFTER_US],
        "in flash, a HEADER goes before the boot row and before the anchor"
    );
}

#[test]
fn the_status_line_reports_every_stats_ms_in_the_log_merge() {
    let parts = recorder(registry().0, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    step(&mut writer, &parts.clock, START_US);
    let position = parts.recorder.writer_position().unwrap();
    assert_eq!(step(&mut writer, &parts.clock, START_US + 999_999).records, 0);
    assert_eq!(
        step(&mut writer, &parts.clock, START_US + 1_000_000).records,
        0,
        "queued for the merge"
    );
    assert_eq!(
        step(&mut writer, &parts.clock, START_US + 1_000_000 + MERGE_HORIZON_US).records,
        1
    );
    let log = storage.read_back().file(None);
    assert_eq!(
        log,
        [format!(
            "11000000 Recorder :: {{'session': '000123', 'tlm': {{'count': 0, 'max': 0, 'dropped': 0}}, 'log': \
             {{'count': 0, 'max': 0, 'dropped': 0}}, 'ring': {{'erased': 31, 'saved': 0, 'unsaved': 0, \
             'other_version': 0, 'foreign': 0, 'lost': 0, 'bad': 0, 'chunks': 1, 'flash_errors': 0, \
             'writer': (0, {}), 'saver': None, 'rate': {}}}}}",
            position.offset, position.offset
        )]
    );
    let stats = parts.recorder.stats();
    assert_eq!(
        (stats.chunks, stats.flash_errors),
        (2, 0),
        "the boot row's chunk and the status line's"
    );
    assert_eq!(
        (stats.ring.erased_ahead, stats.ring.unsaved),
        (31, 0),
        "the figures of the status line"
    );
    assert_eq!(
        stats.write_rate, position.offset,
        "the boot row's chunk, in the second since the writer started"
    );
}

#[test]
fn telemetry_writes_its_header_first_then_its_rows_in_order() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    for index in 0..5 {
        imu.push(START_US + u64::from(index) * 1000, &imu_row(index)).unwrap();
    }
    step(&mut writer, &parts.clock, START_US + 5000);
    assert_eq!(
        storage.read_back().file(Some(IMU)),
        [
            "uptime;ax;ay;az",
            "10000000;0;-1;9.81",
            "10001000;0.5;-1;9.81",
            "10002000;1;-1;9.81",
            "10003000;1.5;-1;9.81",
            "10004000;2;-1;9.81",
        ],
        "on flash in the same step: no age to wait for"
    );
}

#[test]
fn decimation_takes_the_streams_window_then_the_global_one() {
    let settings = Settings {
        telemetry_ms: 20,
        stats_ms: 1000,
    };
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 1 << 16, settings);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    let mut health = Telemetry::new(parts.recorder, streams.health).unwrap();
    let mut events = Telemetry::new(parts.recorder, streams.events).unwrap();
    let mut due = Vec::new();
    for tick in 0..=20u64 {
        let now = START_US + tick * 5000; // every 5 ms for 100 ms
        due.push(imu.due(now));
        imu.push(now, &imu_row(tick as u32)).unwrap();
        health.push(now, &[Cell::F32(41.5), Cell::I32(7)]).unwrap();
        events.push(now, &[Cell::Text("tick")]).unwrap();
    }
    step(&mut writer, &parts.clock, START_US + 100_000);
    let read = storage.read_back();
    let uptimes = |routing: &str| -> Vec<u64> { read.file(Some(routing))[1..].iter().map(|row| uptime(row)).collect() };
    let at = |ticks: &[u64]| -> Vec<u64> { ticks.iter().map(|tick| START_US + tick * 5000).collect() };
    assert_eq!(uptimes(IMU), at(&[0, 4, 8, 12, 16, 20]), "the global 20 ms");
    assert_eq!(
        uptimes("000123_health.csv"),
        at(&[0]),
        "its own 1 s, the first row always"
    );
    assert_eq!(uptimes(EVENTS), at(&(0..=20).collect::<Vec<_>>()), "never decimated");
    let expected: Vec<bool> = (0..=20).map(|tick: u32| tick.is_multiple_of(4)).collect();
    assert_eq!(due, expected, "due answers what push does");
}

#[test]
fn due_needs_a_running_writer_and_rows_queue_meanwhile() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 0, 4096, SETTINGS);
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    assert!(!imu.due(START_US), "no writer yet");
    imu.push(START_US, &imu_row(0)).unwrap(); // from an unattached thread: the misc ring
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    assert!(imu.due(START_US + 1));
    let other = Storage::erased(geometry());
    assert!(Writer::new(parts.recorder, other.ring()).is_none(), "one writer");
    step(&mut writer, &parts.clock, START_US);
    assert_eq!(
        storage.read_back().file(Some(IMU)),
        ["uptime;ax;ay;az", "10000000;0;-1;9.81"]
    );
    drop(writer);
    assert!(!imu.due(START_US + 2), "the writer is gone");
    assert!(
        Writer::new(parts.recorder, other.ring()).is_none(),
        "and is never replaced: what it counted as not yet on flash started with it"
    );
    // negative: a stream id this registry never handed out
    let (mut more, _) = self::registry();
    let foreign = more.add("extra.csv", &[], 0, false).unwrap();
    assert!(Telemetry::new(parts.recorder, foreign).is_none());
}

#[test]
fn a_header_that_failed_goes_out_with_the_next_push() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 128, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    for _ in 0..3 {
        assert!(parts.recorder.log("x", "y"), "32 bytes each");
    }
    assert!(!parts.recorder.log("x", "y"), "the ring is full");
    let mut health = Telemetry::new(parts.recorder, streams.health).unwrap();
    let row = [Cell::F32(41.5), Cell::I32(7)];
    assert_eq!(
        health.push(START_US, &row),
        Err(Overflow),
        "the 44-byte header did not fit"
    );
    step(&mut writer, &parts.clock, START_US);
    settle(&mut writer, &parts.clock); // the logs go out, and their ring space comes back
    health.push(parts.clock.now(), &row).unwrap();
    settle(&mut writer, &parts.clock);
    let rows = storage.read_back().file(Some("000123_health.csv"));
    assert_eq!(
        rows,
        ["uptime;temp;load", "10020000;41.5;7"],
        "the header, then the row"
    );
    let stats = parts.recorder.stats();
    assert_eq!((stats.telemetry.dropped, stats.log.dropped), (1, 1));
}

#[test]
fn log_lines_merge_across_rings_by_uptime() {
    let parts = recorder(registry().0, 2, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    step(&mut writer, &parts.clock, START_US);
    // each ring is in uptime order on its own; across rings they interleave
    let lines = [
        (0, 1000, "first"),
        (0, 3000, "fourth"),
        (1, 2000, "second"),
        (1, 4000, "fifth"),
    ];
    for ring in 0..2 {
        std::thread::scope(|scope| {
            scope.spawn(|| {
                let _attached = parts.recorder.attach(ring).unwrap();
                for &(_, offset, message) in lines.iter().filter(|line| line.0 == ring) {
                    parts.clock.set(START_US + offset);
                    assert!(parts.recorder.log(&format!("ring{ring}"), message));
                }
            });
        });
    }
    parts.clock.set(START_US + 2500);
    assert!(
        parts.recorder.log("main", "third"),
        "an unattached thread: the misc ring"
    );
    let progress = step(&mut writer, &parts.clock, START_US + 4000 + MERGE_HORIZON_US - 1);
    assert_eq!(progress.records, 4, "the newest is inside the horizon");
    assert_eq!(
        step(&mut writer, &parts.clock, START_US + 4000 + MERGE_HORIZON_US).records,
        1
    );
    assert_eq!(
        storage.read_back().file(None),
        [
            "10001000 ring0 :: first",
            "10002000 ring1 :: second",
            "10002500 main :: third",
            "10003000 ring0 :: fourth",
            "10004000 ring1 :: fifth",
        ]
    );
}

#[test]
fn telemetry_goes_out_at_once_while_a_log_waits_for_the_horizon() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    step(&mut writer, &parts.clock, START_US);
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    parts.clock.set(START_US + 100_000);
    parts.recorder.log("x", "held");
    imu.push(START_US + 105_000, &imu_row(0)).unwrap();
    assert_eq!(
        step(&mut writer, &parts.clock, START_US + 110_000),
        Progress { records: 2, chunks: 1 },
        "the header and the row"
    );
    assert_eq!(
        step(&mut writer, &parts.clock, START_US + 120_000),
        Progress { records: 1, chunks: 1 },
        "the log line, 20 ms old"
    );
}

#[test]
fn chunks_fill_each_block_before_the_next() {
    let geometry = Geometry::new(16).unwrap();
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 1 << 20, SETTINGS);
    let storage = Storage::erased(geometry);
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    let mut events = Telemetry::new(parts.recorder, streams.events).unwrap();
    for index in 0..600u32 {
        let now = START_US + u64::from(index) * 2000;
        imu.push(now, &imu_row(index)).unwrap();
        if index % 3 == 0 {
            // rows of several sizes, so the room a block has left rarely fits the next one exactly
            events
                .push(now, &[Cell::Text(&"e".repeat(index as usize % 200))])
                .unwrap();
        }
        if index % 7 == 0 {
            step(&mut writer, &parts.clock, now);
        }
    }
    settle(&mut writer, &parts.clock);
    let image = storage.flash.image();
    let found = blocks(&image, geometry);
    assert!(found.len() >= 6, "several blocks filled: {}", found.len());
    for pair in found.windows(2) {
        let walk = |block: &Block| -> Vec<chunk::Chunk> {
            chunk::chunks(slot_bytes(&image, block.slot))
                .map(|walked| match walked {
                    Walked::Chunk(found) => found,
                    Walked::Torn(torn) => panic!("nothing torn: {torn:?}"),
                })
                .collect()
        };
        let last = *walk(&pair[0]).last().unwrap();
        let end = last.offset as usize + chunk_bytes(last.payload.len());
        let next = walk(&pair[1])[0];
        let first_record = usize::from(u16::from_le_bytes([next.payload[0], next.payload[1]]));
        assert!(
            BLOCK_BYTES as usize - end < CHUNK_HEADER_BYTES + first_record,
            "block {} left {} bytes, the record that opened the next needed {}",
            pair[0].header.block_sequence,
            BLOCK_BYTES as usize - end,
            CHUNK_HEADER_BYTES + first_record
        );
    }
    let rows = storage.read_back().file(Some(IMU));
    assert_eq!(rows.len(), 601, "the header and every row, once");
    assert!(rows[1..].iter().map(|row| uptime(row)).is_sorted());
}

#[test]
fn the_flight_flag_marks_the_blocks_of_every_record_queued_in_flight() {
    let (registry, streams) = registry();
    // a reboot in flight: the crumb names GLIDING, so the bring-up builds the Recorder and the ring in flight, and
    // the first block opens as a flight block
    let parts = recorder_of(BOOT_ID, registry, 1, 4096, SETTINGS, true);
    let storage = Storage::erased(geometry());
    let mut writer = Writer::new(parts.recorder, storage.ring_in_flight()).unwrap();
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    let clock = &parts.clock;
    imu.push(START_US, &imu_row(0)).unwrap();
    step(&mut writer, clock, START_US);
    // the warm-start gate refused: a new session on the pad. A row and a log line queued before the fall are the
    // flight's, so the fall waits until both are programmed: the row goes in the next step, the log line only once
    // it is past the merge horizon, a step later
    imu.push(START_US + 500, &imu_row(1)).unwrap();
    clock.set(START_US + 600);
    parts.recorder.log("sequencer", "stage -> done");
    parts.recorder.set_flight(false);
    clock.set(START_US + 5000);
    parts.recorder.log("sequencer", "after the fall"); // merged after the fall applied: a pad block's
    step(&mut writer, clock, START_US + 1000);
    step(&mut writer, clock, START_US + 1000 + MERGE_HORIZON_US);
    imu.push(START_US + 30_000, &imu_row(2)).unwrap();
    step(&mut writer, clock, START_US + 30_000);
    // BOOSTING: a rise applies to the very next chunk
    parts.recorder.set_flight(true);
    imu.push(START_US + 31_000, &imu_row(3)).unwrap();
    step(&mut writer, clock, START_US + 31_000);
    imu.push(START_US + 32_000, &imu_row(4)).unwrap();
    step(&mut writer, clock, START_US + 32_000);
    let image = storage.flash.image();
    let mut placed = Vec::new();
    let mut logged = Vec::new();
    for block in blocks(&image, geometry()) {
        let mut decoder = Decoder::new();
        decoder
            .block(slot_bytes(&image, block.slot), true, |line| {
                let text = String::from_utf8_lossy(line.body()).into_owned();
                let place = || (uptime(&text[11..]), block.header.block_sequence, block.header.flight);
                if line.routing == Some(IMU) && !text.contains("uptime;") {
                    placed.push(place());
                }
                if line.routing.is_none() && text.contains(" sequencer :: ") {
                    logged.push(place());
                }
            })
            .unwrap();
    }
    assert_eq!(
        logged,
        [(START_US + 600, 0, true), (START_US + 5000, 1, false)],
        "the log line queued before the fall is in the flight block, though merged a step after the fall was seen"
    );
    assert_eq!(
        placed,
        [
            (START_US, 0, true),
            (START_US + 500, 0, true),
            (START_US + 30_000, 1, false),
            (START_US + 31_000, 2, true),
            (START_US + 32_000, 2, true),
        ],
        "each change closed the block, a fall once what came before it was in flash; no change, no new block"
    );
}

#[test]
fn a_warm_start_refused_before_the_writer_exists_holds() {
    for (refused, ring_in_flight) in [(true, true), (false, false)] {
        let (registry, streams) = registry();
        // a reboot in flight: the Recorder starts in flight, from the crumb
        let parts = recorder_of(BOOT_ID, registry, 1, 4096, SETTINGS, true);
        if refused {
            // main refuses the warm start (bring-up step 8) while the writer thread still scans (step 3)
            parts.recorder.set_flight(false);
        }
        let storage = Storage::erased(geometry());
        let ring = if ring_in_flight {
            storage.ring_in_flight()
        } else {
            storage.ring()
        };
        let mut writer = Writer::new(parts.recorder, ring).unwrap();
        let _attached = parts.recorder.attach(0).unwrap();
        let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
        for index in 0..200u32 {
            let now = START_US + u64::from(index) * TICK_US; // 2 s of rows
            imu.push(now, &imu_row(index)).unwrap();
            step(&mut writer, &parts.clock, now);
        }
        let found = blocks(&storage.flash.image(), geometry());
        assert!(found.len() >= 3, "several blocks: {}", found.len());
        let flight: Vec<bool> = found.iter().map(|block| block.header.flight).collect();
        if refused {
            assert!(
                flight.iter().all(|&flight| !flight),
                "every block of the pad session is a pad block, though the ring was opened in flight: {flight:?}"
            );
        } else {
            // negative: told nothing, the Recorder's crumb flag wins over a ring opened on the pad
            assert!(
                flight.iter().all(|&flight| flight),
                "the Recorder owns the flag: {flight:?}"
            );
        }
    }
}

#[test]
fn a_durable_record_wakes_the_writer_and_goes_in_its_step() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    step(&mut writer, &parts.clock, START_US);
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    let mut events = Telemetry::new(parts.recorder, streams.events).unwrap();
    imu.push(START_US + 1000, &imu_row(0)).unwrap();
    assert!(!parts.wakes.take(), "a plain row waits for the tick");
    events.push(START_US + 1500, &[Cell::Text("apogee")]).unwrap();
    assert!(parts.wakes.take(), "the woken writer steps now, not at the tick");
    assert_eq!(
        step(&mut writer, &parts.clock, START_US + 1500),
        Progress { records: 4, chunks: 1 },
        "both streams' headers and rows"
    );
    assert_eq!(
        storage.read_back().file(Some(EVENTS)),
        ["uptime;event", "10001500;apogee"]
    );
}

#[test]
fn the_tees_copy_records_while_their_window_is_open() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let recorder = parts.recorder;
    let _attached = recorder.attach(0).unwrap();
    assert!(recorder.drain_log_tee(1000).lines.is_empty());
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    imu.push(START_US, &imu_row(0)).unwrap();
    recorder.log("sequencer", "stage -> boosting");
    step(&mut writer, &parts.clock, START_US + MERGE_HORIZON_US);
    let logs = recorder.drain_log_tee(1000);
    assert_eq!(logs.lines, ["10000000 sequencer :: stage -> boosting"]);
    assert_eq!(logs.dropped, 0);
    assert!(
        recorder.drain_telemetry_tee(1000).lines.is_empty(),
        "its window was closed"
    );
    imu.push(START_US + 30_000, &imu_row(1)).unwrap();
    recorder.index_session(EPOCH, None, "dashboard", None).unwrap();
    step(&mut writer, &parts.clock, START_US + 30_000);
    assert_eq!(
        recorder.drain_telemetry_tee(0).lines,
        [
            "@000123_imu_lsm6dso32.csv@10030000;0.5;-1;9.81",
            "@session.csv@10020000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;dashboard;;",
        ]
    );
    // negative: a window nobody renews lapses, and its stale records go
    imu.push(START_US + 40_000, &imu_row(2)).unwrap();
    step(&mut writer, &parts.clock, START_US + 40_000);
    assert!(
        recorder.drain_telemetry_tee(1000).lines.is_empty(),
        "closed by the last drain"
    );
    imu.push(START_US + 50_000, &imu_row(3)).unwrap();
    step(&mut writer, &parts.clock, START_US + 50_000); // in the window opened at 40 ms
    imu.push(START_US + 2_000_000, &imu_row(4)).unwrap();
    step(&mut writer, &parts.clock, START_US + 2_000_000); // past it: lapsed
    assert!(
        recorder.drain_telemetry_tee(1000).lines.is_empty(),
        "the lapsed batch is gone"
    );
}

#[test]
fn with_no_erased_block_records_stay_in_their_rings_until_the_eraser_frees_one() {
    let geometry = Geometry::new(8).unwrap();
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 8192, SETTINGS);
    // an unformatted ring: every slot foreign, so nothing opens until the eraser erases one
    let storage = Storage::over(geometry, vec![0; geometry.bytes() as usize]);
    assert!(storage.scanned.unformatted);
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    let (mut accepted, mut refused) = (Vec::new(), 0);
    let mut now = START_US;
    for index in 0..400u32 {
        now = START_US + u64::from(index) * 1000;
        match imu.push(now, &imu_row(index)) {
            Ok(()) => accepted.push(now),
            Err(Overflow) => refused += 1,
        }
        assert_eq!(
            step(&mut writer, &parts.clock, now).chunks,
            0,
            "nothing can be programmed"
        );
    }
    let stats = parts.recorder.stats();
    assert!(refused > 0, "the ring filled behind the waiting chunk");
    assert_eq!(stats.telemetry.dropped, refused);
    assert!(stats.telemetry.count > 0, "records wait in their ring");
    assert_eq!(stats.telemetry.max, stats.telemetry.count);
    assert_eq!(writer.ring().table().figures(None).foreign, 8);
    // the eraser formats the slots, and everything accepted goes out, once, in order
    let mut eraser = eraser(&storage.flash, &storage.scanned);
    for index in 1..=40 {
        now += 10_000;
        eraser.erase_step(None, (now / 1000) as u32).unwrap();
        let progress = step(&mut writer, &parts.clock, now);
        assert!(
            index > 1 || progress.chunks > 0,
            "the first erased slot takes the waiting chunk at once"
        );
    }
    let stats = parts.recorder.stats();
    assert_eq!((stats.telemetry.count, stats.flash_errors), (0, 0), "{stats:?}");
    let read = storage.read_back();
    let rows = read.file(Some(IMU));
    assert_eq!(rows[0], "uptime;ax;ay;az");
    assert_eq!(rows[1..].iter().map(|row| uptime(row)).collect::<Vec<u64>>(), accepted);
    let waits: Vec<String> = read
        .file(None)
        .into_iter()
        .filter(|line| line.contains("waits"))
        .collect();
    let first_run = waits.iter().filter(|line| uptime(line) < START_US + 400_000).count();
    assert_eq!(first_run, 1, "a run of 400 refused steps logs once: {waits:?}");
    assert!(
        waits
            .iter()
            .all(|line| line.contains("no erased block to open yet; its records stay in PSRAM"))
    );
}

#[test]
fn a_dead_flash_keeps_records_queued_and_counts_its_errors() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    storage.flash.cut_power(2, 0); // the block header and the boot row's chunk, then nothing
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    step(&mut writer, &parts.clock, START_US);
    let (mut accepted, mut refused) = (0, 0);
    for index in 0..200u32 {
        let now = START_US + 1000 + u64::from(index) * 10_000;
        match imu.push(now, &imu_row(index)) {
            Ok(()) => accepted += 1,
            Err(Overflow) => refused += 1,
        }
        step(&mut writer, &parts.clock, now);
    }
    let stats = parts.recorder.stats();
    // a record leaves its ring only once its chunk is programmed: the header (48 bytes) and every accepted row
    // (48 bytes each) are still in the 4 KiB ring together
    assert!(
        48 + accepted * 48 < 4096,
        "{accepted} rows: some ring space came back unprogrammed"
    );
    assert_eq!(stats.chunks, 1, "only the boot row's chunk went out");
    assert!(stats.flash_errors > 0);
    assert!(refused > 0, "the ring filled");
    assert_eq!(
        stats.telemetry.count,
        accepted as u32 + 1,
        "every accepted row and the header still queued"
    );
    assert_eq!(stats.telemetry.dropped, refused);
}

#[test]
fn a_failed_program_goes_again_at_the_next_offset() {
    for (torn_bytes, duplicate) in [(30, false), (usize::MAX, true)] {
        let (registry, streams) = registry();
        let parts = recorder(registry, 1, 1 << 16, SETTINGS);
        let storage = Storage::erased(geometry());
        // the block header, the boot row's chunk, then the third program: it tears (30 bytes), or lands whole and
        // still fails; the flash works on
        storage.flash.fail(2, torn_bytes);
        let mut writer = writer(parts.recorder, &storage);
        let _attached = parts.recorder.attach(0).unwrap();
        let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
        let mut accepted = Vec::new();
        for index in 0..60u32 {
            let now = START_US + u64::from(index) * 10_000;
            imu.push(now, &imu_row(index)).unwrap();
            accepted.push(row_text(now, &imu_row(index)));
            step(&mut writer, &parts.clock, now);
        }
        settle(&mut writer, &parts.clock);
        let context = format!("{torn_bytes} bytes landed");
        assert_eq!(parts.recorder.stats().flash_errors, 1, "{context}");
        let image = storage.flash.image();
        let block = blocks(&image, geometry())[0];
        let walked: Vec<Walked> = chunk::chunks(slot_bytes(&image, block.slot)).collect();
        let torn: Vec<&Walked> = walked
            .iter()
            .filter(|walked| matches!(walked, Walked::Torn(_)))
            .collect();
        if duplicate {
            assert!(torn.is_empty(), "{context}: two whole copies");
        } else {
            assert!(
                matches!(torn[..], [Walked::Torn(torn)] if torn.followed),
                "{context}: the retry follows the debris: {walked:?}"
            );
        }
        let read = storage.read_back();
        assert_eq!(read.lost, 0, "{context}: a retried program is no loss");
        let rows = payloads(&read.capture, Some(IMU));
        assert_eq!(rows[0], "uptime;ax;ay;az");
        if !duplicate {
            assert_eq!(rows[1..], accepted, "{context}: every row once, whole, in order");
            continue;
        }
        // the retried chunk is in flash twice, byte for byte, the copy at the next offset
        let whole: Vec<&[u8]> = walked
            .iter()
            .filter_map(|walked| match walked {
                Walked::Chunk(found) => Some(found.payload),
                Walked::Torn(_) => None,
            })
            .collect();
        let twice: Vec<&[u8]> = whole
            .windows(2)
            .filter(|pair| pair[0] == pair[1])
            .map(|pair| pair[0])
            .collect();
        let [retried] = twice[..] else {
            panic!("{context}: one chunk in flash twice: {} found", twice.len())
        };
        let retried_rows: Vec<String> = raw_payload_uptimes(retried)
            .into_iter()
            .map(|uptime_us| accepted.iter().find(|row| uptime(row) == uptime_us).unwrap().clone())
            .collect();
        assert!(!retried_rows.is_empty(), "{context}: the retried chunk holds rows");
        assert_eq!(
            rows.len(),
            1 + accepted.len() + retried_rows.len(),
            "{context}: before the filter, the retried chunk's rows read twice"
        );
        // the tools' filter keeps the first line of each (stream, uptime, line CRC) and drops exactly the copies
        let (filtered, dropped) = drop_duplicates(&read.capture);
        assert_eq!(
            payloads(&filtered, Some(IMU))[1..],
            accepted,
            "{context}: after the filter, every row once, whole, in order"
        );
        assert_eq!(
            payloads(&dropped.concat(), Some(IMU)),
            retried_rows,
            "{context}: the lines dropped are the retried chunk's rows, and only they"
        );
        assert_eq!(
            dropped.len(),
            retried_rows.len(),
            "{context}: nothing of another file dropped"
        );
    }
}

/// The uptime of every record in a chunk payload, read by the spec's record table: a u16 length at 0, the u64
/// uptime at 8.
fn raw_payload_uptimes(payload: &[u8]) -> Vec<u64> {
    let mut uptimes = Vec::new();
    let mut rest = payload;
    while rest.len() >= 16 {
        let length = usize::from(u16::from_le_bytes([rest[0], rest[1]]));
        uptimes.push(u64::from_le_bytes(rest[8..16].try_into().unwrap()));
        rest = &rest[length..];
    }
    uptimes
}

#[test]
fn a_row_no_chunk_could_hold_raises_at_push() {
    let mut registry = Registry::new();
    let wide = registry.add("wide.csv", &["text"; 16], 0, false).unwrap();
    let parts = recorder(registry, 1, 1 << 16, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    let mut stream = Telemetry::new(parts.recorder, wide).unwrap();
    let text = "x".repeat(255);
    // 16 + 8 (wide.csv) + 15 x 257 = 3879 bytes fit a 4032-byte chunk
    stream.push(START_US, &[Cell::Text(&text); 15]).unwrap();
    // negative: one cell more is 4136, which no chunk holds: the producer learns of it now
    assert_eq!(stream.push(START_US + 1, &[Cell::Text(&text); 16]), Err(Overflow));
    step(&mut writer, &parts.clock, START_US + 1);
    let rows = storage.read_back().file(Some("000123_wide.csv"));
    assert_eq!(rows.len(), 2, "the header and the 15-cell row, whole");
    assert_eq!(rows[1].len(), "10000000;".len() + 15 * 255 + 14);
    let stats = parts.recorder.stats();
    assert_eq!(
        (stats.telemetry.dropped, stats.telemetry.count),
        (1, 0),
        "the refused push, and nothing the writer had to drop"
    );
}

#[test]
fn a_new_stats_ms_takes_effect_at_the_next_step() {
    let parts = recorder(registry().0, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    step(&mut writer, &parts.clock, START_US);
    assert!(parts.recorder.set_stats_ms(100), "1000 ms -> 100 ms");
    step(&mut writer, &parts.clock, START_US + 100_000);
    step(&mut writer, &parts.clock, START_US + 200_000);
    // negative: 0 is refused, and the period stays
    assert!(!parts.recorder.set_stats_ms(0));
    step(&mut writer, &parts.clock, START_US + 300_000);
    settle(&mut writer, &parts.clock);
    let status: Vec<u64> = storage
        .read_back()
        .file(None)
        .iter()
        .filter(|line| line.contains(" Recorder :: {'session'"))
        .map(|line| uptime(line))
        .collect();
    assert_eq!(status, [START_US + 100_000, START_US + 200_000, START_US + 300_000]);
    assert_eq!(parts.recorder.inspect().stats_ms, 100);
}

#[test]
fn stats_count_what_is_not_yet_on_flash() {
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 4096, SETTINGS);
    let storage = Storage::erased(geometry());
    let mut writer = writer(parts.recorder, &storage);
    let _attached = parts.recorder.attach(0).unwrap();
    step(&mut writer, &parts.clock, START_US);
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    for index in 0..10 {
        imu.push(START_US + 1000 + u64::from(index), &imu_row(index)).unwrap();
    }
    parts.clock.set(START_US + 1005);
    parts.recorder.log("x", "y");
    step(&mut writer, &parts.clock, START_US + 1010);
    let stats = parts.recorder.stats();
    assert_eq!(
        (stats.telemetry.count, stats.log.count),
        (0, 1),
        "the rows are programmed, the log line is held for the merge"
    );
    settle(&mut writer, &parts.clock);
    let stats = parts.recorder.stats();
    assert_eq!((stats.telemetry.count, stats.log.count, stats.log.max), (0, 0, 1));
    assert_eq!(
        parts.recorder.writer_position(),
        Some(writer.ring().position()),
        "published for the saver"
    );
}
