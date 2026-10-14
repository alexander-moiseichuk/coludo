//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The HEADER repeats and newest wins (doc/specs/avionics-port.md "Producer side": the writer copies every started
//! stream's HEADER record again about once per MiB, so a session whose first blocks newest wins erased unsaved
//! still gets its headers within the next MiB; "The ring_eraser and newest wins"):
//! * the repeats are in flash, about a MiB apart, for every stream this boot started and for no other, and a
//!   decoder that starts anywhere in the ring prints a stream's header within the next MiB, while one that reads
//!   from the start prints it once;
//! * with no saver, a run three times the ring's size keeps the newest data and the whole flight: newest wins
//!   erases the oldest pad blocks first, every erase is a loss the eraser reports, and the headers come back.

mod common;

use common::*;

use flash_ring::{Erasure, Geometry, MemoryFlash};
use recorder::{Cell, Decoder, HEADER_REPEAT_BYTES, Registry, StreamId, Telemetry, Writer};

/// Rows queued per writer step: about one block of flash.
const ROWS_PER_STEP: u32 = 16;

/// The wide pad every row carries, so a row is 232 bytes and the runs stay short.
const PAD: &str = "padpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpad\
                   padpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpad\
                   padpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpadpad";

/// The bytes one row takes in flash: 16 + `imu.csv` (7) + an I32 (5) + the pad's Text cell (2 + 200).
const ROW_BYTES: u64 = 232;

/// The slack a repeat may come late by: the chunk of the step that crossed the MiB, and the block it closed.
const REPEAT_SLACK: u64 = 2 * 4096;

/// The streams: one written all along, one written once, one registered and never written.
struct Streams {
    /// `imu.csv`: index, pad.
    imu: StreamId,
    /// `events.csv`: one durable row at the start.
    events: StreamId,
    /// `silent.csv`: registered, never pushed: it gets no file.
    silent: StreamId,
}

/// The registry of [`Streams`].
fn registry() -> (Registry, Streams) {
    let mut registry = Registry::new();
    let streams = Streams {
        imu: registry.add("imu.csv", &["index", "pad"], -1, false).unwrap(),
        events: registry.add("events.csv", &["event"], -1, true).unwrap(),
        silent: registry.add("silent.csv", &["x"], -1, false).unwrap(),
    };
    (registry, streams)
}

/// Row `index` of the IMU stream.
fn imu_row(index: u32) -> [Cell<'static>; 2] {
    [Cell::I32(index as i32), Cell::Text(&PAD[..200])]
}

/// The uptime of row `index`: one every 100 us.
fn row_uptime(index: u32) -> u64 {
    START_US + u64::from(index) * 100
}

#[test]
fn header_repeats_reach_the_decoder_within_a_mebibyte() {
    let geometry = Geometry::new(1024).unwrap(); // 4 MiB: nothing is erased
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 1 << 20, SETTINGS);
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let mut writer = Writer::new(parts.recorder, ring(&flash, &scanned, BOOT_ID)).unwrap();
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    let mut events = Telemetry::new(parts.recorder, streams.events).unwrap();
    assert!(Telemetry::new(parts.recorder, streams.silent).is_some());
    events.push(START_US, &[Cell::Text("start")]).unwrap();
    let rows = (2_700_000 / ROW_BYTES) as u32; // about 2.6 MiB
    for index in 0..rows {
        imu.push(row_uptime(index), &imu_row(index)).unwrap();
        if index % ROWS_PER_STEP == ROWS_PER_STEP - 1 {
            parts.clock.set(row_uptime(index));
            writer.step(row_uptime(index), EPOCH);
        }
    }
    writer.step(row_uptime(rows), EPOCH);
    let image = flash.image();

    // in flash: the started streams' headers about a MiB apart, the silent one's never
    let raw = raw_records(&image, geometry);
    let headers = |name: &str| -> Vec<u64> {
        raw.iter()
            .filter(|record| record.kind == 1 && record.name == name)
            .map(|record| record.flash_at)
            .collect()
    };
    let imu_headers = headers("imu.csv");
    assert!(
        imu_headers.len() >= 3,
        "the first and two repeats in 2.6 MiB: {imu_headers:?}"
    );
    for pair in imu_headers.windows(2) {
        assert!(
            pair[1] - pair[0] <= u64::from(HEADER_REPEAT_BYTES) + REPEAT_SLACK,
            "repeats about a MiB apart: {imu_headers:?}"
        );
    }
    assert_eq!(
        headers("events.csv").len(),
        imu_headers.len(),
        "written once, still repeated"
    );
    assert_eq!(
        headers("session.csv").len(),
        imu_headers.len(),
        "the boot row's header too"
    );
    assert!(
        headers("silent.csv").is_empty(),
        "a stream that never pushed gets no file"
    );

    // read from the start: one header line per file, every row once
    let read = read_back(&image, geometry, None);
    let imu_file = read.file(Some("000123_imu.csv"));
    assert_eq!(imu_file[0], "uptime;index;pad");
    assert_eq!(imu_file.len(), rows as usize + 1, "the repeats print nothing");
    assert_eq!(read.file(Some("000123_events.csv")), ["uptime;event", "10000000;start"]);
    assert!(read.file(Some("000123_silent.csv")).is_empty());

    // read from 1.2 MiB in, as after newest wins took the first blocks: the header comes within a MiB
    let all = blocks(&image, geometry);
    let skipped = (1_200_000 / 4096) as usize;
    let mut decoder = Decoder::new();
    let mut imu_lines = Vec::new();
    let mut events_lines = Vec::new();
    for block in &all[skipped..] {
        decoder
            .block(slot_bytes(&image, block.slot), true, |line| {
                let payload = String::from_utf8_lossy(line.body()).into_owned();
                match line.routing {
                    Some("000123_imu.csv") => imu_lines.push(payload),
                    Some("000123_events.csv") => events_lines.push(payload),
                    _ => {}
                }
            })
            .unwrap();
    }
    let header_at = imu_lines
        .iter()
        .position(|line| line.contains("};uptime;index;pad;<"))
        .expect("a repeat reaches a decoder that started late");
    assert!(
        header_at as u64 * ROW_BYTES <= u64::from(HEADER_REPEAT_BYTES) + REPEAT_SLACK,
        "{header_at} rows before the header"
    );
    assert_eq!(
        imu_lines.iter().filter(|line| line.contains(";uptime;")).count(),
        1,
        "and prints it once"
    );
    assert_eq!(events_lines.len(), 1, "the events file starts with its header too");
}

#[test]
fn newest_wins_keeps_the_flight_and_the_newest_rows_and_the_headers_come_back() {
    let geometry = Geometry::new(640).unwrap(); // 2.5 MiB, a reserve of 6 blocks
    let (registry, streams) = registry();
    let parts = recorder(registry, 1, 1 << 20, SETTINGS);
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let mut writer = Writer::new(parts.recorder, ring(&flash, &scanned, BOOT_ID)).unwrap();
    let mut eraser = eraser(&flash, &scanned);
    let _attached = parts.recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(parts.recorder, streams.imu).unwrap();
    // the pad (2 MiB), a flight (1.5 MiB), then the pad again (2.5 MiB): two and a half rings' worth
    let pad_before = (2 << 20) / ROW_BYTES as u32;
    let flight_end = pad_before + (3 << 19) / ROW_BYTES as u32;
    let rows = flight_end + (5 << 19) / ROW_BYTES as u32;
    let mut losses = Vec::new();
    for index in 0..rows {
        if index == pad_before {
            parts.recorder.set_flight(true); // BOOSTING
        }
        if index == flight_end {
            parts.recorder.set_flight(false); // DONE
        }
        imu.push(row_uptime(index), &imu_row(index)).unwrap();
        if index % ROWS_PER_STEP == ROWS_PER_STEP - 1 {
            let now = row_uptime(index);
            parts.clock.set(now);
            writer.step(now, EPOCH);
            if let Erasure::Lost(loss) = eraser.erase_step(None, (now / 1000) as u32).unwrap() {
                losses.push(loss);
            }
        }
    }
    writer.step(row_uptime(rows), EPOCH);
    assert_eq!(
        parts.recorder.stats().telemetry.count,
        0,
        "the writer never waited for long"
    );
    assert!(
        losses.len() > 640,
        "the ring went round more than once: {}",
        losses.len()
    );
    assert!(
        losses.iter().all(|loss| !loss.flight),
        "while pad blocks were left, no flight block went"
    );
    assert_eq!(
        writer.ring().table().figures(None).lost as usize,
        losses.len(),
        "every loss counted"
    );

    let read = read_back(&flash.image(), geometry, None);
    let imu_file = read.file(Some("000123_imu.csv"));
    let header_at = imu_file
        .iter()
        .position(|line| line == "uptime;index;pad")
        .expect("the header came back");
    assert!(
        header_at as u64 * ROW_BYTES <= u64::from(HEADER_REPEAT_BYTES) + REPEAT_SLACK,
        "within a MiB of the oldest data kept: {header_at} rows before it"
    );
    let indexes: Vec<u32> = imu_file
        .iter()
        .filter(|line| !line.starts_with("uptime"))
        .map(|row| row.split(';').nth(1).unwrap().parse().unwrap())
        .collect();
    // the kept data: one run that starts at most a step before BOOSTING and holds the whole flight, then a gap
    // (the pad after it, given up first), then the newest rows
    let gap = indexes
        .windows(2)
        .position(|pair| pair[1] != pair[0] + 1)
        .expect("the oldest pad rows after the flight went");
    let (flight, newest) = (&indexes[..=gap], &indexes[gap + 1..]);
    assert!(
        flight[0] <= pad_before && flight[0] + ROWS_PER_STEP > pad_before,
        "from just before BOOSTING: {}",
        flight[0]
    );
    assert!(*flight.last().unwrap() >= flight_end - 1, "the whole flight, to DONE");
    assert!(
        newest.windows(2).all(|pair| pair[1] == pair[0] + 1) && *newest.last().unwrap() == rows - 1,
        "then a gapless run of the newest rows"
    );
    assert_eq!(
        read.file(Some("session.csv")).first().map(String::as_str),
        Some(recorder_wire::SESSION_INDEX_HEADER),
        "the boot row's header came back too, though the boot row is gone"
    );
}
