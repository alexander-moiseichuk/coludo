//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder end to end, as on the board: four producer threads push rows and log lines into their own rings,
//! an unattached thread logs through the shared one, an executor thread writes session.csv time-set rows, and the
//! writer runs on its own thread against a real clock, into a host flash ring brought up as the firmware does.
//! The ring is then read back as the ring_saver and `GET /ring` read it: every block through the decoder, which
//! gives a capture of wire lines. Every line must verify under its own routing, recorderd's demux of the capture
//! must file the same lines, every stream must open with its header and hold every row in order, and each
//! thread's log lines must keep their order.
//!
//! The host tools then read the ring image itself, as the operator does when the card failed: tools/ring_split.py
//! splits it into session directories, which must hold byte for byte what recorderd's demux of the decoder's
//! capture files, and its `--boot --raw-capture` must write that very capture. So ring_split's binary decoder (block
//! header, chunk walk, records, cells, F32 text) reads Rust-written bytes. Then tools/recorder_wire.py must call
//! every line of its capture GOOD, tools/flight_telemetry.py must read it strictly (all good, nothing salvaged,
//! rejected or legacy, no splice), and every line of the files it split must verify. The image and ring_split's
//! output go to a directory under the system temp dir, kept when the check fails. The check is skipped, with a
//! note, when python3 is not installed.
//!
//! The CC tee, threaded: a producer and the writer run on their own threads while an executor drains the
//! telemetry tee as the CC polls it, and every row programmed inside the window must come out of the drains, or be
//! counted in their `dropped`.

mod common;

use common::*;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant};

use flash_ring::{Geometry, MemoryFlash};
use recorder::{Buffers, Cell, MERGE_HORIZON_US, Recorder, Registry, Settings, StreamId, Telemetry, Writer};

/// Producer threads, each with its own ring.
const THREADS: usize = 4;

/// Rows per stream.
const ROWS: u32 = 1500;

/// A log line every this many rows.
const LOG_EVERY: u32 = 10;

/// Time-set rows the executor writes.
const TIME_SETS: u32 = 3;

/// 256 blocks, 1 MiB: room for the whole run with no saver and no eraser.
fn geometry() -> Geometry {
    Geometry::new(256).unwrap()
}

/// A thread's two streams: a fast one of mixed cells, and a durable event stream.
#[derive(Clone, Copy)]
struct Streams {
    /// `t<k>_fast.csv`: index, value, flag, angle, wide, word, nothing.
    fast: StreamId,
    /// `t<k>_events.csv`: index and a word, durable, every 100th row.
    events: StreamId,
}

/// The fast stream's cells for row `index` of thread `thread`.
fn fast_row(thread: usize, index: u32) -> [Cell<'static>; 7] {
    const WORDS: [&str; 4] = ["setting", "boosting", "gliding", "landing"];
    [
        Cell::I32(index as i32),
        Cell::F32(index as f32 * 0.1 + thread as f32),
        Cell::Bool(index.is_multiple_of(2)),
        Cell::E7(-803_917_950 + index as i32),
        Cell::I64(i64::from(index) << 33),
        Cell::Text(WORDS[(index as usize + thread) % 4]),
        Cell::None,
    ]
}

/// The registry: two streams per thread.
fn registry() -> (Registry, Vec<Streams>) {
    let mut registry = Registry::new();
    let streams = (0..THREADS)
        .map(|thread| Streams {
            fast: registry
                .add(
                    &format!("t{thread}_fast.csv"),
                    &["index", "value", "flag", "angle", "wide", "word", "nothing"],
                    -1,
                    false,
                )
                .unwrap(),
            events: registry
                .add(&format!("t{thread}_events.csv"), &["index", "event"], -1, true)
                .unwrap(),
        })
        .collect();
    (registry, streams)
}

/// The real clock's uptime: microseconds since `origin`, from [`START_US`].
fn now(origin: Instant) -> u64 {
    START_US + origin.elapsed().as_micros() as u64
}

/// A Recorder with the real clock, and a count of the writer wakes.
fn recorder(registry: Registry, origin: Instant, wakes: &'static AtomicU32) -> &'static Recorder {
    let buffers = Buffers {
        producers: (0..THREADS + 1).map(|_| buffer(64 << 10)).collect(), // the last ring: the executor's
        misc: buffer(16 << 10),
        log_tee: buffer(64 << 10),
        telemetry_tee: buffer(64 << 10),
    };
    let settings = Settings {
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
    recorder
}

/// Push until it is queued: telemetry raises when its ring is full, and this producer waits for the writer.
fn push(telemetry: &mut Telemetry, origin: Instant, cells: &[Cell], retries: &AtomicU32) {
    while telemetry.push(now(origin), cells).is_err() {
        retries.fetch_add(1, Ordering::Relaxed);
        std::thread::yield_now();
    }
}

/// The writer thread's life: bring the ring up over a fresh flash, step every millisecond (at once on a wake)
/// until the producers are done, then step past the merge horizon and the next status line, and hand back the
/// flash image.
fn write(recorder: &'static Recorder, origin: Instant, producing: &AtomicBool, wakes: &AtomicU32) -> Vec<u8> {
    let flash = MemoryFlash::new(geometry().bytes());
    let scanned = bring_up(&flash, geometry());
    let mut writer = Writer::new(recorder, ring(&flash, &scanned, BOOT_ID)).unwrap();
    while producing.load(Ordering::Acquire) {
        writer.step(now(origin), EPOCH);
        if wakes.swap(0, Ordering::Relaxed) == 0 {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    let end = now(origin);
    for after in [MERGE_HORIZON_US, 200_000, 200_000 + MERGE_HORIZON_US] {
        writer.step(end + after, EPOCH);
    }
    drop(writer);
    flash.into_image()
}

#[test]
fn many_threads_record_into_the_ring_and_every_line_reads_back_good() {
    let origin = Instant::now();
    let (registry, streams) = registry();
    let wakes: &'static AtomicU32 = Box::leak(Box::new(AtomicU32::new(0)));
    let recorder = recorder(registry, origin, wakes);
    let producing = AtomicBool::new(true);
    let retries = AtomicU32::new(0);
    let logged: Vec<AtomicU32> = (0..THREADS).map(|_| AtomicU32::new(0)).collect();
    let misc_logged = AtomicU32::new(0);
    let image = std::thread::scope(|scope| {
        let writer_thread = scope.spawn(|| write(recorder, origin, &producing, wakes));
        let producers: Vec<_> = (0..THREADS)
            .map(|thread| {
                let (streams, retries, logged) = (streams[thread], &retries, &logged[thread]);
                scope.spawn(move || {
                    let _attached = recorder.attach(thread).unwrap();
                    let mut fast = Telemetry::new(recorder, streams.fast).unwrap();
                    let mut events = Telemetry::new(recorder, streams.events).unwrap();
                    for index in 0..ROWS {
                        push(&mut fast, origin, &fast_row(thread, index), retries);
                        if index.is_multiple_of(100) {
                            push(
                                &mut events,
                                origin,
                                &[Cell::I32(index as i32), Cell::Text("mark")],
                                retries,
                            );
                        }
                        if index.is_multiple_of(LOG_EVERY)
                            && recorder.log(&format!("thread{thread}"), &format!("row {index}"))
                        {
                            logged.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                })
            })
            .collect();
        let executor = scope.spawn(|| {
            let _attached = recorder.attach(THREADS).unwrap();
            for set in 0..TIME_SETS {
                std::thread::sleep(Duration::from_millis(5));
                recorder
                    .index_session(
                        EPOCH + i64::from(set),
                        Some(-240),
                        "cc-auto",
                        Some((255_144_000, -803_918_000)),
                    )
                    .unwrap();
            }
        });
        let unattached = scope.spawn(|| {
            for index in 0..50 {
                if recorder.log("vprintf", &format!("idf line {index}")) {
                    misc_logged.fetch_add(1, Ordering::Relaxed);
                }
                std::thread::sleep(Duration::from_micros(200));
            }
        });
        for producer in producers {
            producer.join().unwrap();
        }
        executor.join().unwrap();
        unattached.join().unwrap();
        producing.store(false, Ordering::Release);
        writer_thread.join().unwrap()
    });

    let stats = recorder.stats();
    assert_eq!(
        (stats.telemetry.count, stats.log.count),
        (0, 0),
        "everything is on flash: {stats:?}"
    );
    assert_eq!(
        stats.telemetry.dropped,
        retries.load(Ordering::Relaxed),
        "each refused push counted"
    );
    assert_eq!(stats.flash_errors, 0);

    let read = read_back(&image, geometry(), None); // every line verifies under its own routing, or this panics
    assert_eq!(read.lost, 0);
    let files = demux(&read.capture);
    assert_eq!(
        files.keys().collect::<Vec<_>>(),
        read.files.keys().collect::<Vec<_>>(),
        "recorderd files the decoder's lines where the decoder routed them"
    );
    for (routing, lines) in &files {
        assert_eq!(lines.len(), read.files[routing].len(), "{routing:?}");
    }

    for thread in 0..THREADS {
        let fast = read.file(Some(&format!("{SESSION}_t{thread}_fast.csv")));
        assert_eq!(
            fast[0], "uptime;index;value;flag;angle;wide;word;nothing",
            "the header first"
        );
        assert_eq!(fast.len(), ROWS as usize + 1, "thread {thread}: every row, once");
        let mut last = 0;
        for (index, row) in fast[1..].iter().enumerate() {
            let uptime = uptime(row);
            assert!(uptime >= last, "thread {thread} row {index}: uptime went back");
            last = uptime;
            assert_eq!(
                *row,
                row_text(uptime, &fast_row(thread, index as u32)),
                "thread {thread} row {index}"
            );
        }
        let events = read.file(Some(&format!("{SESSION}_t{thread}_events.csv")));
        assert_eq!(events[0], "uptime;index;event");
        assert_eq!(events.len(), (ROWS as usize).div_ceil(100) + 1);
    }
    let session = read.file(Some("session.csv"));
    assert_eq!(session[0], recorder_wire::SESSION_INDEX_HEADER);
    assert!(session[1].ends_with(";boot;;"), "{}", session[1]);
    assert_eq!(
        session.len(),
        2 + TIME_SETS as usize,
        "header, boot, time sets: {session:?}"
    );
    assert!(
        session[2..]
            .iter()
            .all(|row| row.contains(";-240;taster;dev;3f2a91;cc-auto;25.5144000;-80.3918000"))
    );

    // recorder.log: each thread's lines in order, all of those it queued; the shared ring's too
    let mut per_source: BTreeMap<String, Vec<u64>> = BTreeMap::new();
    for line in read.file(None) {
        let source = line.split(' ').nth(1).unwrap().to_string();
        per_source.entry(source).or_default().push(uptime(&line));
    }
    for (thread, logged) in logged.iter().enumerate() {
        let uptimes = &per_source[&format!("thread{thread}")];
        assert_eq!(uptimes.len() as u32, logged.load(Ordering::Relaxed));
        assert!(uptimes.is_sorted(), "thread {thread}'s lines keep their order");
    }
    assert_eq!(per_source["vprintf"].len() as u32, misc_logged.load(Ordering::Relaxed));
    assert!(
        !per_source["Recorder"].is_empty(),
        "status lines, the last one at the end"
    );
    host_tools(&image, &read.capture, &files);
}

#[test]
fn the_tee_loses_no_row_while_the_executor_drains() {
    const TEE_ROWS: u32 = 20_000;
    let origin = Instant::now();
    let mut registry = Registry::new();
    let imu = registry.add("imu.csv", &["index"], -1, false).unwrap();
    let buffers = Buffers {
        producers: vec![buffer(1 << 20)],
        misc: buffer(4096),
        log_tee: buffer(4096),
        telemetry_tee: buffer(1 << 20), // never full at this pace: every row must come out as a line
    };
    let settings = Settings {
        telemetry_ms: 0,
        stats_ms: 1_000_000,
    };
    let recorder: &'static Recorder = Box::leak(Box::new(Recorder::new(
        registry,
        identity(BOOT_ID),
        settings,
        false,
        buffers,
        Box::new(move || now(origin)),
        Box::new(|| {}),
    )));
    recorder.bind_config_id(CONFIG_ID);
    let producing = AtomicBool::new(true);
    let (lines, dropped) = std::thread::scope(|scope| {
        let writer_thread = scope.spawn(|| {
            let flash = MemoryFlash::new(geometry().bytes());
            let scanned = bring_up(&flash, geometry());
            let mut writer = Writer::new(recorder, ring(&flash, &scanned, BOOT_ID)).unwrap();
            while producing.load(Ordering::Acquire) {
                writer.step(now(origin), EPOCH);
                std::thread::yield_now();
            }
            writer.step(now(origin), EPOCH); // the last rows
        });
        // the CC opens the window before the first row, and renews it at every poll: a 10 s window, polled far
        // more often than a real CC does, so drains overlap the writer's steps as often as they can
        recorder.drain_telemetry_tee(10_000);
        let producer = scope.spawn(|| {
            let _attached = recorder.attach(0).unwrap();
            let mut telemetry = Telemetry::new(recorder, imu).unwrap();
            for index in 0..TEE_ROWS {
                while telemetry.push(now(origin), &[Cell::I32(index as i32)]).is_err() {
                    std::thread::yield_now();
                }
                if index % 64 == 0 {
                    std::thread::sleep(Duration::from_micros(50));
                }
            }
        });
        let (mut lines, mut dropped) = (Vec::new(), 0);
        while !producer.is_finished() {
            let batch = recorder.drain_telemetry_tee(10_000);
            dropped += batch.dropped;
            lines.extend(batch.lines);
            std::thread::sleep(Duration::from_millis(2));
        }
        producer.join().unwrap();
        while recorder.stats().telemetry.count > 0 {
            std::thread::yield_now();
        }
        producing.store(false, Ordering::Release);
        writer_thread.join().unwrap();
        let batch = recorder.drain_telemetry_tee(0);
        dropped += batch.dropped;
        lines.extend(batch.lines);
        (lines, dropped)
    });
    let routing = format!("@{SESSION}_imu.csv@");
    let header = format!("{routing}uptime;index");
    let indexes: Vec<u32> = lines
        .iter()
        .filter(|line| line.starts_with(&routing) && **line != header)
        .map(|line| line.rsplit(';').next().unwrap().parse().unwrap())
        .collect();
    assert_eq!(
        indexes.len() as u32 + dropped,
        TEE_ROWS,
        "every row programmed inside the window was teed, or counted dropped"
    );
    assert_eq!(dropped, 0, "a tee this large never fills at this pace");
    assert!(indexes.iter().copied().eq(0..TEE_ROWS), "each row once, in order");
    assert_eq!(
        lines.iter().filter(|line| **line == header).count(),
        1,
        "the header, once"
    );
}

/// The repository's tools/ directory.
fn tools() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../tools")
}

/// The host tools' check, run by python3 over ring_split's capture and session directory: prints one JSON summary.
const CHECK: &str = r#"
import json, os, sys
tools, capture, directory = sys.argv[1], sys.argv[2], sys.argv[3]
sys.path.insert(0, tools)
import flight_telemetry, recorder_wire
with open(capture, encoding='utf-8', errors='surrogateescape') as handle:
    text = handle.read()
lines = text.splitlines()
statuses = {}
for line in lines:
    status = recorder_wire.split(line)[0]
    statuses[status] = statuses.get(status, 0) + 1
streams, logs = flight_telemetry.parse(text)
files = {}
for name in sorted(os.listdir(directory)):
    with open(os.path.join(directory, name), encoding='utf-8', errors='surrogateescape') as handle:
        routing = None if name == 'recorder.log' else name
        rows = handle.read().splitlines()
        files[name] = sum(recorder_wire.verify(routing, row) is not None for row in rows), len(rows)
print(json.dumps({'lines': len(lines), 'statuses': statuses, 'counts': flight_telemetry.line_counts(),
                  'spliced': flight_telemetry.spliced(streams), 'rows': {name: len(stream.rows) for name, stream
                  in streams.items()}, 'logs': len(logs), 'files': files}))
"#;

/// Every file under `root`, by its path relative to `root`, with its bytes.
fn tree(root: &Path) -> BTreeMap<PathBuf, Vec<u8>> {
    let mut found = BTreeMap::new();
    let mut directories = vec![root.to_path_buf()];
    while let Some(directory) = directories.pop() {
        for entry in std::fs::read_dir(&directory).unwrap() {
            let path = entry.unwrap().path();
            if path.is_dir() {
                directories.push(path);
            } else {
                let relative = path.strip_prefix(root).unwrap().to_path_buf();
                found.insert(relative, std::fs::read(&path).unwrap());
            }
        }
    }
    found
}

/// Split the ring `image` with tools/ring_split.py and check what it wrote against the decoder: its session
/// directory against recorderd's demux of the decoder's capture (`files`), byte for byte, and its raw capture
/// against that capture. Then run [`CHECK`] over ring_split's output and assert its verdict.
fn host_tools(image: &[u8], capture: &[u8], files: &BTreeMap<Option<String>, Vec<Vec<u8>>>) {
    let directory = std::env::temp_dir().join(format!("coludo-recorder-end-to-end-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&directory); // ring_split refuses to replace a file a stale run left
    std::fs::create_dir_all(&directory).unwrap();
    std::fs::write(directory.join("image.bin"), image).unwrap();
    let (split, raw) = (directory.join("split"), directory.join("capture.txt"));
    let output = std::process::Command::new("python3")
        .arg(tools().join("ring_split.py"))
        .arg(directory.join("image.bin"))
        .arg(&split)
        .args(["--boot", &BOOT_ID.to_string(), "--raw-capture"])
        .arg(&raw)
        .output();
    let output = match output {
        Ok(output) => output,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            eprintln!("note: python3 is not installed, so the host tools' check is skipped");
            std::fs::remove_dir_all(&directory).unwrap();
            return;
        }
        Err(error) => panic!("python3 did not run: {error}"),
    };
    let kept = || format!("files kept in {}", directory.display());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "ring_split failed ({}): {}\n{stdout}\n{}",
        output.status,
        String::from_utf8_lossy(&output.stderr),
        kept()
    );
    eprintln!("ring_split: {stdout}");
    assert!(
        stdout.contains(&format!("boot {BOOT_ID} session {SESSION}: "))
            && stdout.contains("lost chunks 0, damaged chunks 0, duplicates 0"),
        "{stdout}\n{}",
        kept()
    );
    assert!(
        std::fs::read(&raw).unwrap() == capture,
        "ring_split's raw capture is the decoder's, byte for byte: {}",
        kept()
    );
    let mut expected = BTreeMap::new();
    for (routing, lines) in files {
        let name = routing.as_deref().unwrap_or("recorder.log");
        expected.insert(Path::new(SESSION).join(name), lines.concat());
        if name == recorder_wire::SESSION_INDEX {
            expected.insert(PathBuf::from(name), lines.concat()); // the global index too
        }
    }
    let found = tree(&split);
    assert_eq!(
        found.keys().collect::<Vec<_>>(),
        expected.keys().collect::<Vec<_>>(),
        "ring_split writes the files recorderd's demux does: {}",
        kept()
    );
    for (path, bytes) in &expected {
        assert!(found[path] == *bytes, "{}: byte for byte; {}", path.display(), kept());
    }
    let output = std::process::Command::new("python3")
        .arg("-c")
        .arg(CHECK)
        .arg(tools())
        .arg(&raw)
        .arg(split.join(SESSION))
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        output.status.success(),
        "the host check failed ({}): {}\n{stdout}\nfiles kept in {}",
        output.status,
        String::from_utf8_lossy(&output.stderr),
        directory.display()
    );
    let verdict: serde_json::Value = serde_json::from_str(stdout.trim()).unwrap();
    eprintln!("host tools: {verdict}");
    let lines = verdict["lines"].as_u64().unwrap();
    let kept = || format!("{verdict}\nfiles kept in {}", directory.display());
    assert_eq!(
        verdict["statuses"],
        serde_json::json!({"good": lines}),
        "recorder_wire.split: {}",
        kept()
    );
    assert_eq!(
        verdict["counts"],
        serde_json::json!({"good": lines, "salvaged": 0, "rejected": 0, "legacy": 0}),
        "flight_telemetry strict: {}",
        kept()
    );
    assert_eq!(
        verdict["spliced"],
        serde_json::json!([]),
        "no file holds two headers: {}",
        kept()
    );
    for thread in 0..THREADS {
        assert_eq!(verdict["rows"][format!("t{thread}_fast.csv")], ROWS, "{}", kept());
    }
    for (name, counts) in verdict["files"].as_object().unwrap() {
        assert_eq!(
            counts[0],
            counts[1],
            "{name}: every line of the demuxed file verifies; {}",
            kept()
        );
    }
    assert_eq!(verdict["files"].as_object().unwrap().len(), files.len());
    std::fs::remove_dir_all(&directory).unwrap();
}
