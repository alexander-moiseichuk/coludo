//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! M4's host proof (doc/specs/avionics-port.md "M4. ring_saver, SD, HTTP pull", Proof): producers, then the writer,
//! the saver and the eraser over an in-memory flash, into an SD card that the host tools then read as the operator
//! reads a pulled session (`tools/board_pull.sh`): `recorder_flight` with 0 rejected, a board.log that is not empty
//! and a session.csv that dates the boot, and `assemble_capture` plus `flight_telemetry` strict, all good. Each card
//! is laid out as the board's `/sdcard` ([`recorder::layout`]): the tools read the session's
//! `<session>/recordings/`, and the global `sessions.csv` at the root names the session's boot. The cards:
//! * N producer threads (control, i2c0, supervisor, executor, and an unattached one through `misc`) on a real
//!   clock, the three ring tasks on one core-0 thread, the card a temporary directory (its root) through [`StdFs`];
//! * a saver cut at every call of a batch, in flight, the board dying with the card: boot 124 resumes at the
//!   published position (and, for a few cuts, at one a batch older) on the same card, saving the batch again into
//!   duplicates the tools drop, never a header twice;
//! * a hint stale across a boot boundary (written about once a second while a backlog of two boots drains), and a
//!   lost one: the next boot saves both boots again from the first, into duplicates the tools drop, never a header
//!   twice;
//! * newest wins with a dead SD: the pad blocks go first and every loss is logged, the flight stays, and a new
//!   card then gets everything still in flash, the loss lines among it;
//! * a session longer than 2^30 us whose `anchor` row is dated, as the hub's sync on connect dates it (the boot
//!   row is not: the clock was unset at boot). `recorder_flight` places a time-set row by the 2^30 wraps, so in a
//!   boot that long a lone time-set row would get two places; the anchor's one place dates the boot, and the flight
//!   after the 2^30 mark gets its UTC.
//!
//! The tools run in python3: tests/sd_tools.py judges many cards in one process, and recorder_flight's own command
//! line reads the threaded and the long session. With no python3 the tools' checks are skipped with a note (the
//! cards' own checks still run). Every card goes to a directory under the system temp dir, kept when a check fails.

mod common;

use common::board::{Board, Flight, Plan, Streams, flight_registry};
use common::card::{Card, Keep, headers, lines, payload, recorded, unique};
use common::*;

use std::collections::BTreeMap;
use std::ffi::OsStr;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant};

use flash_ring::{Eraser, Erasure, Geometry, Hints, MemoryFlash, scan};
use recorder::layout::{self, GLOBAL_INDEX, RECORDINGS};
use recorder::{BATCH_MS, Buffers, Cell, MERGE_HORIZON_US, Recorder, Saver, StdFs, StreamId, Telemetry, Writer};
use serde_json::Value;

/// The repository's tools/ directory.
fn tools() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../tools")
}

/// A fresh directory for one test's cards, under the system temp dir.
fn scratch(name: &str) -> PathBuf {
    let directory = std::env::temp_dir().join(format!("coludo-saver-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&directory);
    std::fs::create_dir_all(&directory).unwrap();
    directory
}

/// python3 with `arguments`: its stdout, or None when python3 is not installed (noted). Panics when it fails.
fn python<S: AsRef<OsStr>>(arguments: &[S], kept: &Path) -> Option<String> {
    let output = match Command::new("python3").args(arguments).output() {
        Ok(output) => output,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            eprintln!("note: python3 is not installed, so the host tools' check is skipped");
            return None;
        }
        Err(error) => panic!("python3 did not run: {error}"),
    };
    let stdout = String::from_utf8_lossy(&output.stdout).into_owned();
    assert!(
        output.status.success(),
        "python3 failed ({}): {}\n{stdout}\nfiles kept in {}",
        output.status,
        String::from_utf8_lossy(&output.stderr),
        kept.display()
    );
    Some(stdout)
}

/// The tools' verdict (tests/sd_tools.py) on session `session` of each of `cards` (each the card's root); None with
/// no python3.
fn verdicts(session: &str, cards: &[PathBuf], kept: &Path) -> Option<Vec<Value>> {
    let (driver, tools) = (Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/sd_tools.py"), tools());
    let mut arguments: Vec<&OsStr> = vec![driver.as_os_str(), tools.as_os_str(), OsStr::new(session)];
    arguments.extend(cards.iter().map(|card| card.as_os_str()));
    let stdout = python(&arguments, kept)?;
    Some(stdout.lines().map(|line| serde_json::from_str(line).unwrap()).collect())
}

/// Asserts the tools found `verdict`'s card all good: no line rejected or salvaged by recorder_flight (none proving
/// nothing elsewhere either), board.log not empty, every line of assemble_capture's capture good in
/// flight_telemetry, no file spliced, and the global index at the card's root whole, naming the session as its own
/// session.csv does (the same row sources).
fn assert_good(verdict: &Value, context: &str) {
    assert_eq!(verdict["index_damaged"], 0, "{context}: the global index: {verdict}");
    assert_eq!(
        verdict["indexed"], verdict["sources"],
        "{context}: the global index names the session as its session.csv does: {verdict}"
    );
    for (stream, counts) in verdict["tally"].as_object().unwrap() {
        assert_eq!(
            (counts[1].as_u64(), counts[2].as_u64()),
            (Some(0), Some(0)),
            "{context}: recorder_flight salvaged or rejected {stream}: {verdict}"
        );
    }
    assert_eq!(verdict["elsewhere"], 0, "{context}: {verdict}");
    assert!(
        verdict["board"].as_u64().unwrap() > 0,
        "{context}: board.log is empty: {verdict}"
    );
    let good = verdict["counts"]["good"].as_u64().unwrap();
    assert!(good > 0, "{context}: {verdict}");
    assert_eq!(
        verdict["counts"],
        serde_json::json!({"good": good, "salvaged": 0, "rejected": 0, "legacy": 0}),
        "{context}: flight_telemetry strict: {verdict}"
    );
    assert_eq!(
        verdict["spliced"],
        serde_json::json!([]),
        "{context}: no file holds two headers: {verdict}"
    );
}

/// recorder_flight's command line over the session's recordings directory `directory`, its flight cut into `out`: its
/// report, checked for 0 rejected everywhere and a board.log that is not empty. None with no python3.
fn recorder_flight(directory: &Path, out: &Path, kept: &Path) -> Option<String> {
    let tool = tools().join("recorder_flight.py");
    let arguments = [
        tool.as_os_str(),
        directory.as_os_str(),
        OsStr::new("--session"),
        OsStr::new(SESSION),
        OsStr::new("-o"),
        out.as_os_str(),
    ];
    let report = python(&arguments, kept)?;
    eprintln!("recorder_flight:\n{report}");
    let rejected: Vec<&str> = report
        .lines()
        .filter_map(|line| line.split(" rejected").next().filter(|_| line.contains(" rejected")))
        .filter_map(|head| head.rsplit(' ').next())
        .collect();
    assert!(!rejected.is_empty(), "the checked rows are reported: {report}");
    assert!(rejected.iter().all(|count| *count == "0"), "0 rejected: {report}");
    let board = report
        .lines()
        .find(|line| line.trim_start().starts_with("board.log"))
        .and_then(|line| line.split_whitespace().nth(1))
        .and_then(|count| count.parse::<u32>().ok());
    assert!(board.is_some_and(|lines| lines > 0), "board.log is not empty: {report}");
    let written = std::fs::read(out.join("recorder/board.log")).unwrap();
    assert!(!written.is_empty());
    Some(report)
}

/// Boot `boot`'s recorder for a flight on the test's thread (the 256 KiB `misc` ring).
fn flight_recorder(boot: u32) -> (Parts, Streams) {
    let (registry, streams) = flight_registry();
    (recorder_of(boot, registry, 0, 256 << 10, SETTINGS, false), streams)
}

/// The threaded run's producer threads, each with its ring: control, i2c0, supervisor, executor.
const RINGS: usize = 4;

/// IMU rows the control thread queues, one every 2 ms: 500 Hz, under the ~660 Hz that recorder_flight's ignition
/// rule assumes (it looks 200 samples ahead for its 0.3 s hold).
const IMU_ROWS: u32 = 1500;

/// The control thread's boost: rows `BOOST`, about 0.8 s of 5 g.
const BOOST: std::ops::Range<u32> = 600..1000;

/// Barometer rows the i2c0 thread queues, every 5 ms.
const BARO_ROWS: u32 = 400;

/// Health rows the supervisor queues, every 50 ms.
const HEALTH_ROWS: u32 = 45;

/// The threaded run's streams.
#[derive(Clone, Copy)]
struct ThreadStreams {
    /// The flight's: IMU, health, sequencer.
    flight: Streams,
    /// `baro_icp10111.csv`, plan B's fields.
    baro: StreamId,
}

/// The real clock's uptime: microseconds since `origin`, from [`START_US`].
fn now(origin: Instant) -> u64 {
    START_US + origin.elapsed().as_micros() as u64
}

/// A row, queued again until it fits: telemetry raises when its ring is full, and this producer waits for the writer.
fn push(telemetry: &mut Telemetry, origin: Instant, cells: &[Cell]) {
    while telemetry.push(now(origin), cells).is_err() {
        std::thread::yield_now();
    }
}

/// The core-0 thread of the threaded run: brings the ring up over a fresh 64-block flash, then steps the writer
/// (every millisecond, at once on a wake), the eraser and the saver until the producers are done; then the writer
/// past the merge horizon, quiet, and the saver until it has caught up. Returns the newest-wins losses.
fn core0(recorder: &'static Recorder, origin: Instant, producing: &AtomicBool, wakes: &AtomicU32, card: &Path) -> u32 {
    let geometry = Geometry::new(64).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let mut writer = Writer::new(recorder, ring(&flash, &scanned, BOOT_ID)).unwrap();
    let mut eraser = Eraser::new(&flash, &scanned.table, None).unwrap();
    let start = scanned.table.saver_start(None);
    let mut saver = Saver::new(recorder, &flash, &scanned.table, StdFs::new(card), start);
    let mut losses = 0;
    let mut erase = |eraser: &mut Eraser<&MemoryFlash>, now_ms: u32| {
        if let Erasure::Lost(loss) = eraser.erase_step(recorder.saver_position(), now_ms).unwrap() {
            recorder.log("ring", &loss.to_string());
            losses += 1;
        }
    };
    while producing.load(Ordering::Acquire) {
        let now_us = now(origin);
        writer.step(now_us, EPOCH);
        erase(&mut eraser, (now_us / 1000) as u32);
        while saver.step((now_us / 1000) as u32).unwrap().is_some() {}
        if wakes.swap(0, Ordering::Relaxed) == 0 {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    assert!(recorder.set_stats_ms(3_600_000), "quiet: no more status lines");
    let end = now(origin);
    for after in [MERGE_HORIZON_US, 200_000, 200_000 + MERGE_HORIZON_US] {
        writer.step(end + after, EPOCH);
    }
    let later_ms = ((end + 300_000) / 1000) as u32;
    for step in 0..4 {
        erase(&mut eraser, later_ms + step * BATCH_MS);
        while saver.step(later_ms + step * BATCH_MS).unwrap().is_some() {}
    }
    assert_eq!(
        recorder.saver_position(),
        recorder.writer_position(),
        "the saver caught up"
    );
    losses
}

#[test]
fn many_producers_record_and_the_card_reads_back_good_in_the_host_tools() {
    let origin = Instant::now();
    let (mut registry, flight) = flight_registry();
    let baro = registry
        .add(
            "baro_icp10111.csv",
            &["altitude", "temperature", "pressure", "elevation"],
            -1,
            false,
        )
        .unwrap();
    let streams = ThreadStreams { flight, baro };
    let wakes: &'static AtomicU32 = Box::leak(Box::new(AtomicU32::new(0)));
    let buffers = Buffers {
        producers: (0..RINGS).map(|_| buffer(128 << 10)).collect(),
        misc: buffer(32 << 10),
        log_tee: buffer(4096),
        telemetry_tee: buffer(4096),
    };
    let settings = recorder::Settings {
        telemetry_ms: 0,
        stats_ms: 100,
    };
    let recorder: &'static Recorder = Box::leak(Box::new(Recorder::new(
        registry,
        identity(BOOT_ID),
        settings,
        false,
        buffers,
        Box::new(move || now(origin)),
        Box::new(move || {
            wakes.fetch_add(1, Ordering::Relaxed);
        }),
    )));
    recorder.bind_config_id(CONFIG_ID);
    let root = scratch("threads");
    let card = root.join("card"); // the card's root, as /sdcard on the board
    let producing = AtomicBool::new(true);
    let unattached_logged = AtomicU32::new(0);
    let losses = std::thread::scope(|scope| {
        let core0 = scope.spawn(|| core0(recorder, origin, &producing, wakes, &card));
        let control = scope.spawn(move || {
            let _attached = recorder.attach(0).unwrap();
            let mut imu = Telemetry::new(recorder, streams.flight.imu).unwrap();
            let mut sequencer = Telemetry::new(recorder, streams.flight.sequencer).unwrap();
            for index in 0..IMU_ROWS {
                let stage = match index {
                    0 => Some(("setting", "boot")),
                    600 => Some(("boosting", "launch")),
                    1000 => Some(("gliding", "burnout")),
                    1300 => Some(("landing", "altitude")),
                    1499 => Some(("done", "landed")),
                    _ => None,
                };
                if let Some((stage, reason)) = stage {
                    recorder.set_flight((600..1499).contains(&index));
                    push(&mut sequencer, origin, &[Cell::Text(stage), Cell::Text(reason)]);
                }
                let az = if BOOST.contains(&index) { 5.0 } else { 1.0 };
                let cells = [
                    Cell::F32(0.01),
                    Cell::F32(-0.02),
                    Cell::F32(az),
                    Cell::I32(12),
                    Cell::I32(-7),
                    Cell::I32(3),
                    Cell::I32(index as i32),
                ];
                push(&mut imu, origin, &cells);
                if index.is_multiple_of(10) {
                    recorder.log("control", &format!("row {index}"));
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        });
        let i2c0 = scope.spawn(move || {
            let _attached = recorder.attach(1).unwrap();
            let mut baro = Telemetry::new(recorder, streams.baro).unwrap();
            for index in 0..BARO_ROWS {
                let altitude = index as f32 * 0.25;
                let cells = [
                    Cell::F32(altitude),
                    Cell::F32(21.5),
                    Cell::F32(101_325.0),
                    Cell::F32(altitude),
                ];
                push(&mut baro, origin, &cells);
                std::thread::sleep(Duration::from_millis(5));
            }
        });
        let supervisor = scope.spawn(move || {
            let _attached = recorder.attach(2).unwrap();
            let mut health = Telemetry::new(recorder, streams.flight.health).unwrap();
            for index in 0..HEALTH_ROWS {
                let cells = [
                    Cell::F32(41.5),
                    Cell::I32(1_900_000 - index as i32),
                    Cell::I32(23),
                    Cell::None,
                    Cell::None,
                    Cell::I32(0),
                    Cell::I32(0),
                    Cell::I32(0),
                ];
                push(&mut health, origin, &cells);
                std::thread::sleep(Duration::from_millis(50));
            }
        });
        let executor = scope.spawn(move || {
            let _attached = recorder.attach(3).unwrap();
            for set in 0..2 {
                std::thread::sleep(Duration::from_millis(200));
                recorder
                    .index_session(
                        EPOCH + 1 + set,
                        Some(-240),
                        "cc-auto",
                        Some((255_144_000, -803_918_000)),
                    )
                    .unwrap();
                recorder.log("cc", &format!("clock set {set}"));
            }
        });
        let unattached = scope.spawn(|| {
            for index in 0..50 {
                if recorder.log("vprintf", &format!("idf line {index}")) {
                    unattached_logged.fetch_add(1, Ordering::Relaxed);
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        for producer in [control, i2c0, supervisor, executor, unattached] {
            producer.join().unwrap();
        }
        producing.store(false, Ordering::Release);
        core0.join().unwrap()
    });
    assert_eq!(losses, 0, "a saver that keeps up leaves newest wins nothing");
    let stats = recorder.stats();
    assert_eq!((stats.telemetry.count, stats.log.count), (0, 0), "{stats:?}");

    // the card itself: every line verifies, every row once, one header a file
    let read = |name: &str| std::fs::read(card.join(name)).unwrap_or_default();
    for (name, rows) in [
        ("imu_lsm6dso32.csv", IMU_ROWS),
        ("baro_icp10111.csv", BARO_ROWS),
        ("health.csv", HEALTH_ROWS),
        ("sequencer.csv", 5),
    ] {
        let path = recorded(SESSION, &format!("{SESSION}_{name}"));
        let bytes = read(&path);
        assert_eq!(
            lines(&bytes).len() as u32,
            rows + 1,
            "{path}: its header, then every row once"
        );
        assert_eq!(headers(&path, &bytes), 1, "{path}");
        assert!(unique(&path, &bytes) == bytes, "{path}: no duplicate without a cut");
        assert!(
            lines(&bytes).into_iter().all(|line| payload(&path, line).is_some()),
            "{path}: every line verifies"
        );
    }
    let log = read(&recorded(SESSION, "recorder.log"));
    let vprintf = lines(&log)
        .into_iter()
        .filter_map(|line| payload("recorder.log", line))
        .filter(|payload| String::from_utf8_lossy(payload).contains(" vprintf :: "))
        .count();
    assert_eq!(vprintf as u32, unattached_logged.load(Ordering::Relaxed));
    for index in [recorded(SESSION, "session.csv"), GLOBAL_INDEX.to_string()] {
        let bytes = read(&index);
        assert_eq!(
            lines(&bytes).len(),
            4,
            "{index}: the header, the boot row, two clock sets"
        );
        assert!(
            lines(&bytes).into_iter().all(|line| payload(&index, line).is_some()),
            "{index}: every line verifies"
        );
    }
    // negative: the old layout's directories are not there, and nothing but the session and the index is
    let mut top: Vec<String> = std::fs::read_dir(&card)
        .unwrap()
        .map(|entry| entry.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    top.sort();
    assert_eq!(top, [SESSION, GLOBAL_INDEX], "the card's root");

    // the host tools, as the operator reads a pulled session
    let directory = card.join(layout::directory(SESSION, RECORDINGS));
    let Some(report) = recorder_flight(&directory, &root.join("flight"), &root) else {
        return;
    };
    assert!(report.contains(&format!("boot {BOOT_ID} (board taster")), "{report}");
    assert!(report.contains("dated by boot"), "session.csv dates the boot: {report}");
    assert!(
        report.contains("ignition at 2026-10-03T14:21:") && report.contains("Z UTC"),
        "{report}"
    );
    let verdict = &verdicts(SESSION, &[card], &root).unwrap()[0];
    assert_good(verdict, "threads");
    assert_eq!(verdict["indexed"], serde_json::json!(["boot", "cc-auto"]), "{verdict}");
    assert_eq!(verdict["rows"]["imu_lsm6dso32.csv"], IMU_ROWS, "{verdict}");
    assert_eq!(verdict["duplicates"], 0);
    std::fs::remove_dir_all(&root).unwrap();
}

/// Where the cut sweep cuts: the boot's first batch (six files: both session.csv, the IMU, health, the sequencer,
/// recorder.log), and one in flight, past the boost.
const CUT_TICKS: [u64; 2] = [0, 455];

/// The flight of the cut sweep: boost at 3 s, landing at 6 s, DONE at 7 s.
const PLAN: Plan = Plan {
    boost: 300,
    land: 600,
    done: 700,
    pad_imu_every: 1,
};

/// A pad that never boosts: boot 124's.
const PAD: Plan = Plan {
    boost: 1 << 40,
    land: 1 << 41,
    done: 1 << 42,
    pad_imu_every: 1,
};

/// One card of the cut sweep. Boot 123 flies to `from`, then on until its next batch is committed; with
/// `cut`, the card's power goes at that call of the batch, and the board dies with it (the same tick either way).
/// Boot 124 then boots over the flash as it was, on the same card: its saver resumes at boot 123's published
/// position (at the one before, when `stale`: the cfg NVS hint a batch behind), its eraser trusting the hint, and
/// it records a short pad.
///
/// Returns the card and the calls boot 123's last batch made (all of them when uncut).
fn cut_run(from: u64, cut: Option<(u32, Keep)>, stale: bool) -> (Card, u32) {
    let geometry = Geometry::new(64).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    let mut flight = Flight::new(parts.recorder, streams, PLAN);
    for tick in 0..from {
        flight.fly(&mut board, tick);
    }
    let (operations, committed) = (card.operations(), board.commits.len());
    if let Some((at, keep)) = cut {
        card.cut_after(at, keep);
    }
    let mut tick = from;
    while board.commits.len() == committed && !card.dead() {
        flight.fly(&mut board, tick);
        tick += 1;
    }
    let batch = card.operations() - operations;
    let hint = match stale {
        true => Some(board.commits[committed - 2].saved.position),
        false => parts.recorder.saver_position(),
    };
    let hints = Hints {
        writer: parts.recorder.writer_position(),
        saver: hint,
    };
    drop(board);
    card.power_on();
    let flash = MemoryFlash::from_image(flash.into_image());
    let scanned = scan(&flash, geometry, hints).unwrap();
    let (parts, streams) = flight_recorder(BOOT_ID + 1);
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID + 1, Some(card.clone()), hint);
    let mut pad = Flight::new(parts.recorder, streams, PAD);
    for tick in 0..300 {
        pad.fly(&mut board, tick);
    }
    assert!(parts.recorder.set_stats_ms(3_600_000));
    board.settle(300);
    assert_eq!(
        parts.recorder.saver_position(),
        parts.recorder.writer_position(),
        "caught up"
    );
    assert!(board.losses.is_empty());
    (card, batch)
}

#[test]
fn a_saver_cut_at_every_call_of_a_batch_saves_it_again_into_duplicates_the_tools_drop() {
    for from in CUT_TICKS {
        sweep(from);
    }
}

/// The cut sweep over boot 123's batch after tick `from`: every call of it, under both [`Keep`]s, and for a batch in
/// flight a few cuts with a hint a batch behind too.
fn sweep(from: u64) {
    let (reference, calls) = cut_run(from, None, false);
    assert!(calls >= 10, "a batch of several files: {calls} calls");
    let expected: BTreeMap<String, Vec<u8>> = reference
        .files()
        .into_iter()
        .map(|(path, bytes)| (path.clone(), unique(&path, &bytes)))
        .collect();
    assert!(
        expected
            .values()
            .zip(reference.files().values())
            .all(|(kept, bytes)| kept == bytes)
    );
    let root = scratch(&format!("cuts-{from}"));
    reference.write_to(&root.join("reference"));
    let mut cards = vec![root.join("reference")];
    let mut duplicates = 0;
    let mut cuts: Vec<(u32, Keep, bool)> = [Keep::Synced, Keep::Written]
        .into_iter()
        .flat_map(|keep| (0..calls).map(move |at| (at, keep, false)))
        .collect();
    if from > 0 {
        cuts.extend([
            (0, Keep::Synced, true),
            (calls / 2, Keep::Written, true),
            (calls - 1, Keep::Written, true),
        ]);
    }
    for (at, keep, stale) in cuts {
        let context = format!("batch after tick {from}, cut at call {at} of {calls}, {keep:?}, stale hint {stale}");
        let (card, _) = cut_run(from, Some((at, keep)), stale);
        let files = card.files();
        assert_eq!(
            files.keys().collect::<Vec<_>>(),
            expected.keys().collect::<Vec<_>>(),
            "{context}"
        );
        for (path, bytes) in &files {
            let kept = unique(path, bytes);
            assert!(
                kept == expected[path],
                "{context}: {path}, its duplicates dropped, is the uncut card's"
            );
            duplicates += lines(bytes).len() - lines(&kept).len();
            if path.ends_with(".csv") && path != GLOBAL_INDEX {
                assert_eq!(headers(path, bytes), 1, "{context}: {path}: no header printed again");
            }
        }
        let name = format!("{keep:?}-{at}-{}", if stale { "stale" } else { "hint" });
        card.write_to(&root.join(&name));
        cards.push(root.join(name));
    }
    assert!(duplicates > 0, "the cut batches went again");
    let Some(verdicts) = verdicts(SESSION, &cards, &root) else {
        return;
    };
    let reference = &verdicts[0];
    assert_good(reference, "the uncut card");
    assert_eq!(
        reference["dated"],
        serde_json::json!(["boot"]),
        "session.csv dates the boot: {reference}"
    );
    let mut dropped = 0;
    for verdict in &verdicts {
        let context = verdict["directory"].as_str().unwrap();
        assert_good(verdict, context);
        assert_eq!(verdict["rows"], reference["rows"], "{context}: every row, once");
        assert_eq!(verdict["dated"], reference["dated"], "{context}");
        assert_eq!(
            verdict["board"], reference["board"],
            "{context}: board.log, its duplicates dropped"
        );
        let tally = verdict["tally"].as_object().unwrap();
        dropped += tally.values().map(|counts| counts[3].as_u64().unwrap()).sum::<u64>();
        dropped += verdict["assembled_duplicates"].as_u64().unwrap();
    }
    assert!(dropped > 0, "the tools dropped the duplicates");
    std::fs::remove_dir_all(&root).unwrap();
}

#[test]
fn a_hint_stale_across_a_boot_boundary_or_lost_prints_no_header_twice() {
    for lost in [false, true] {
        stale_across_boots(lost);
    }
}

/// Boots 122 (a pad) and 123 (a flight) record with no card. Boot 124 finds a card and drains them, its `cfg` NVS
/// hint written about once a second, and its power goes just after boot 123's lines landed, before the next hint.
/// Boot 125 resumes on the same card at that hint, back in boot 122 (with `lost`, at the oldest block: a hint the
/// cfg lost). Either way each file holds its boot's header once, and the tools read both sessions all good.
fn stale_across_boots(lost: bool) {
    let context = if lost {
        "a lost hint"
    } else {
        "a hint stale across a boot"
    };
    let geometry = Geometry::new(256).unwrap();
    let mut image = MemoryFlash::new(geometry.bytes()).into_image();
    let mut hints = Hints::default();
    let mut imu_rows = BTreeMap::new();
    for (boot, plan) in [(122, PAD), (123, PLAN)] {
        let flash = MemoryFlash::from_image(image);
        let scanned = scan(&flash, geometry, hints).unwrap();
        let (parts, streams) = flight_recorder(boot);
        parts.clock.set(START_US + u64::from(boot - 121) * 30_000_000);
        let mut board = Board::new(&flash, &scanned, &parts, boot, None, None);
        let mut flight = Flight::new(parts.recorder, streams, plan);
        for tick in 0..900 {
            flight.fly(&mut board, tick);
        }
        assert!(parts.recorder.set_stats_ms(3_600_000));
        board.settle(300);
        imu_rows.insert(boot, flight.rows[0]);
        hints = Hints {
            writer: parts.recorder.writer_position(),
            saver: None,
        };
        drop(board);
        image = flash.into_image();
    }
    let card = Card::new();
    let flash = MemoryFlash::from_image(image);
    let scanned = scan(&flash, geometry, hints).unwrap();
    let (parts, streams) = flight_recorder(124);
    parts.clock.set(START_US + 4 * 30_000_000);
    let mut board = Board::new(&flash, &scanned, &parts, 124, Some(card.clone()), None);
    let mut nvs = parts.recorder.saver_position();
    let mut pad = Flight::new(parts.recorder, streams, PAD);
    let mut tick = 0;
    while !card.files().keys().any(|path| path.starts_with("000123/recordings/")) {
        pad.fly(&mut board, tick);
        tick += 1;
        if tick.is_multiple_of(100) {
            nvs = parts.recorder.saver_position(); // the hint, about once a second
        }
    }
    assert!(tick < 100, "boot 123's lines landed before the next hint: tick {tick}");
    let hints = Hints {
        writer: parts.recorder.writer_position(),
        saver: if lost { None } else { nvs },
    };
    drop(board);
    let flash = MemoryFlash::from_image(flash.into_image());
    let scanned = scan(&flash, geometry, hints).unwrap();
    assert_eq!(
        scanned.table.saver_start(hints.saver).block_sequence,
        0,
        "{context}: boot 125 starts back in boot 122"
    );
    let (parts, streams) = flight_recorder(125);
    parts.clock.set(START_US + 5 * 30_000_000);
    let mut board = Board::new(&flash, &scanned, &parts, 125, Some(card.clone()), hints.saver);
    let mut pad = Flight::new(parts.recorder, streams, PAD);
    for tick in 0..300 {
        pad.fly(&mut board, tick);
    }
    assert!(parts.recorder.set_stats_ms(3_600_000));
    board.settle(300);
    assert_eq!(parts.recorder.saver_position(), parts.recorder.writer_position());
    drop(board);
    let mut duplicates = 0;
    for (path, bytes) in card.files() {
        duplicates += lines(&bytes).len() - lines(&unique(&path, &bytes)).len();
        if path.ends_with(".csv") && path != GLOBAL_INDEX {
            assert_eq!(headers(&path, &bytes), 1, "{context}: {path}: no header printed again");
        }
    }
    assert!(duplicates > 0, "{context}: boots 122 and 123 went again");
    let root = scratch(if lost { "lost-hint" } else { "stale-hint" });
    card.write_to(&root);
    for (boot, session) in [(122, "000122"), (123, "000123")] {
        let Some(verdicts) = verdicts(session, std::slice::from_ref(&root), &root) else {
            return;
        };
        assert_good(&verdicts[0], &format!("{context}: {session}"));
        assert_eq!(
            verdicts[0]["rows"]["imu_lsm6dso32.csv"], imu_rows[&boot],
            "{context}: {session}: every row once"
        );
    }
    std::fs::remove_dir_all(&root).unwrap();
}

#[test]
fn with_a_dead_card_newest_wins_gives_up_the_pad_first_keeps_the_flight_and_logs_every_loss() {
    let geometry = Geometry::new(64).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let dead = Card::new();
    dead.kill();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(dead), None);
    // a pad long enough to wrap the ring about four times, past the 1 MiB header repeat; then the flight
    let plan = Plan {
        boost: 13_500,
        land: 13_800,
        done: 13_900,
        pad_imu_every: 1,
    };
    let mut flight = Flight::new(parts.recorder, streams, plan);
    for tick in 0..14_200 {
        flight.fly(&mut board, tick);
    }
    assert!(board.card_errors > 100, "the dead card fails every batch");
    let losses = board.losses.clone();
    assert!(losses.len() > 150, "newest wins took {} blocks", losses.len());
    assert!(
        losses.iter().all(|loss| !loss.flight),
        "pad blocks first, flight blocks kept"
    );
    assert!(
        losses
            .windows(2)
            .all(|pair| pair[0].block_sequence < pair[1].block_sequence),
        "oldest first"
    );
    assert_eq!(board.table().figures(None).lost as usize, losses.len(), "ring.lost");
    // a working card in the slot: everything still in flash goes to it, the loss lines among it
    let card = Card::new();
    board.mount(card.clone(), None);
    assert!(parts.recorder.set_stats_ms(3_600_000));
    board.settle(300);
    assert_eq!(board.losses.len(), losses.len(), "nothing lost once the card works");
    let imu_path = recorded(SESSION, &format!("{SESSION}_imu_lsm6dso32.csv"));
    let imu = card.file(&imu_path);
    let ticks: Vec<u64> = lines(&imu)
        .into_iter()
        .filter_map(|line| payload(&imu_path, line))
        .filter(|payload| payload.first().is_some_and(u8::is_ascii_digit))
        .map(|payload| {
            String::from_utf8_lossy(payload)
                .rsplit(';')
                .next()
                .unwrap()
                .parse()
                .unwrap()
        })
        .collect();
    assert!(
        (plan.boost..plan.done).all(|tick| ticks.contains(&tick)),
        "every flight row is on the card"
    );
    assert!(!ticks.contains(&0), "the oldest pad rows went");
    assert_eq!(
        headers(&imu_path, &imu),
        1,
        "the header repeat gave the file its header"
    );
    let log_path = recorded(SESSION, "recorder.log");
    let log = card.file(&log_path);
    let loss_lines: Vec<String> = lines(&log)
        .into_iter()
        .filter_map(|line| payload(&log_path, line))
        .map(|payload| String::from_utf8_lossy(payload).into_owned())
        .filter(|line| line.contains(" ring :: ") && line.ends_with(" erased unsaved"))
        .collect();
    // every loss was logged (the board asserts each line queued); a line is on the card unless newest wins took the
    // block it was written into, a later loss of its own
    let lost: std::collections::BTreeSet<u32> = losses.iter().map(|loss| loss.block_sequence).collect();
    let mut on_card = loss_lines.iter().peekable();
    let mut missing = 0;
    for (loss, &block) in losses.iter().zip(&board.loss_blocks) {
        if on_card.peek().is_some_and(|line| line.ends_with(&loss.to_string())) {
            on_card.next();
        } else {
            missing += 1;
            assert!(
                lost.contains(&block) || lost.contains(&(block + 1)),
                "the line of {loss:?}, queued in block {block}, is neither on the card nor in a block lost since"
            );
        }
    }
    assert_eq!(
        on_card.next(),
        None,
        "every loss line on the card names a loss, in order"
    );
    assert!(
        missing > 0 && loss_lines.len() > 30,
        "{missing} lines went with later losses, {} kept",
        loss_lines.len()
    );
    assert!(loss_lines[0].contains(&format!("(boot {BOOT_ID}, session {SESSION}, flight 0)")));
    let root = scratch("dead-card");
    card.write_to(&root);
    let Some(verdicts) = verdicts(SESSION, std::slice::from_ref(&root), &root) else {
        return;
    };
    assert_good(&verdicts[0], "a new card after a dead one");
    let out = root.join("flight");
    recorder_flight(&root.join(layout::directory(SESSION, RECORDINGS)), &out, &root).unwrap();
    std::fs::remove_dir_all(&root).unwrap();
}

#[test]
fn a_session_longer_than_2_30_us_is_dated_by_its_anchor_row() {
    let geometry = Geometry::new(256).unwrap();
    let flash = MemoryFlash::new(geometry.bytes());
    let scanned = bring_up(&flash, geometry);
    let (parts, streams) = flight_recorder(BOOT_ID);
    let card = Card::new();
    let mut board = Board::new(&flash, &scanned, &parts, BOOT_ID, Some(card.clone()), None);
    board.epoch = 0; // the clock is unset at boot: the boot row dates nothing
    // the pad until 1080 s, the IMU at 1 Hz there: the flight lies past the 2^30 us mark (1073.7 s)
    let plan = Plan {
        boost: 108_000,
        land: 108_300,
        done: 108_400,
        pad_imu_every: 100,
    };
    let mut flight = Flight::new(parts.recorder, streams, plan);
    let set_tick = 3_000;
    for tick in 0..108_600 {
        if tick == set_tick {
            // the hub's sync on connect, 30 s in: a dated cc-auto row
            parts
                .recorder
                .index_session(EPOCH, Some(-240), "cc-auto", Some((255_144_000, -803_918_000)))
                .unwrap();
        }
        if tick >= set_tick {
            board.epoch = EPOCH + ((tick - set_tick) / 100) as i64;
        }
        flight.fly(&mut board, tick);
    }
    assert!(parts.recorder.set_stats_ms(3_600_000));
    board.settle(300);
    assert!(board.now_us > 1 << 30, "a boot longer than 2^30 us");
    assert!(board.losses.is_empty());
    let index_path = recorded(SESSION, "session.csv");
    let index = card.file(&index_path);
    let rows: Vec<String> = lines(&index)
        .into_iter()
        .filter_map(|line| payload(&index_path, line))
        .map(|payload| String::from_utf8_lossy(payload).into_owned())
        .collect();
    assert_eq!(rows.len(), 4, "header, boot, cc-auto, anchor: {rows:?}");
    assert!(
        rows[1].ends_with(";;;taster;dev;3f2a91;boot;;"),
        "undated boot row: {}",
        rows[1]
    );
    assert!(
        rows[3].contains(";2026-10-03T14:21:37Z;;taster;dev;3f2a91;anchor;;"),
        "dated anchor: {}",
        rows[3]
    );
    let root = scratch("long");
    card.write_to(&root);
    let directory = root.join(layout::directory(SESSION, RECORDINGS));
    let Some(report) = recorder_flight(&directory, &root.join("flight"), &root) else {
        return;
    };
    assert!(report.contains("dated by anchor"), "{report}");
    assert!(
        report.contains("ignition at 2026-10-03T14:38:") && report.contains("Z UTC, from the anchor row"),
        "the flight past the 2^30 us mark gets its UTC: {report}"
    );
    let verdict = &verdicts(SESSION, std::slice::from_ref(&root), &root).unwrap()[0];
    assert_good(verdict, "a long session");
    assert_eq!(verdict["dated"], serde_json::json!(["cc-auto", "anchor"]), "{verdict}");
    assert_eq!(
        verdict["indexed"],
        serde_json::json!(["anchor", "boot", "cc-auto"]),
        "the global index holds every row: {verdict}"
    );
    std::fs::remove_dir_all(&root).unwrap();
}
