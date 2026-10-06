//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder end to end, as on the board: four producer threads push rows and log lines into their own rings,
//! an unattached thread logs through the shared one, an executor thread writes session.csv time-set rows, and
//! the writer runs on its own thread against a real clock, into an in-memory flash. The ring is then read back
//! as a dump would be: the boot's DATA payloads in block order make a capture, recorderd's demux files it, and
//! every line must verify under its own routing, every stream must open with its header and hold every row in
//! order, and each thread's log lines must keep their order.
//!
//! The host tools then judge the same capture: tools/recorder_wire.py must call every line GOOD, and
//! tools/flight_telemetry.py must read it strictly (all good, nothing salvaged, rejected or legacy, no splice).
//! The capture and the demuxed files go to a directory under the system temp dir, kept when the check fails. The
//! check is skipped, with a note, when python3 is not installed.

mod common;

use common::*;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant};

use flash_ring::Geometry;
use recorder::{
    Buffers, CUT_AGE_US, Cell, MERGE_HORIZON_US, Recorder, Registry, Settings, StreamId, Telemetry, Writer,
};

/// Producer threads, each with its own ring.
const THREADS: usize = 4;

/// Rows per stream.
const ROWS: u32 = 1500;

/// A log line every this many rows.
const LOG_EVERY: u32 = 10;

/// Time-set rows the executor writes.
const TIME_SETS: u32 = 3;

/// 64 blocks of 64 KiB: room for the whole run with no mover.
fn geometry() -> Geometry {
    Geometry::new(64, 16).unwrap()
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
    let mut registry = Registry::new(SESSION).unwrap();
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

/// A Recorder with a real clock (microseconds since `origin`, from [`START_US`]).
fn recorder(registry: Registry, origin: Instant) -> &'static Recorder {
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
    let clock = move || START_US + origin.elapsed().as_micros() as u64;
    Box::leak(Box::new(Recorder::new(
        registry,
        identity(),
        settings,
        buffers,
        Box::new(clock),
    )))
}

/// The real clock's uptime.
fn now(origin: Instant) -> u64 {
    START_US + origin.elapsed().as_micros() as u64
}

/// Push until it is queued: telemetry raises when its ring is full, and this producer waits for the writer.
fn push(telemetry: &mut Telemetry, origin: Instant, cells: &[Cell], retries: &AtomicU32) {
    while telemetry.push(now(origin), cells).is_err() {
        retries.fetch_add(1, Ordering::Relaxed);
        std::thread::yield_now();
    }
}

#[test]
fn many_threads_record_into_the_ring_and_every_line_reads_back_good() {
    let origin = Instant::now();
    let (registry, streams) = registry();
    let recorder = recorder(registry, origin);
    let ring = ring(recorder, geometry(), OPTIONS);
    let mut writer = Writer::new(recorder, ring).unwrap();
    let producing = AtomicBool::new(true);
    let retries = AtomicU32::new(0);
    let logged: Vec<AtomicU32> = (0..THREADS).map(|_| AtomicU32::new(0)).collect();
    let misc_logged = AtomicU32::new(0);
    let ring = std::thread::scope(|scope| {
        let writer_thread = scope.spawn(|| {
            while producing.load(Ordering::Acquire) {
                writer.step(now(origin), EPOCH);
                std::thread::sleep(Duration::from_millis(1));
            }
            // everything queued is out once the horizon and the cut age have passed; a second later the status
            // line is due once more, and goes out the same way
            let end = now(origin);
            for after in [
                MERGE_HORIZON_US,
                1_000_000,
                1_000_000 + MERGE_HORIZON_US,
                1_000_000 + 2 * CUT_AGE_US,
            ] {
                writer.step(end + after, EPOCH);
            }
            writer.into_ring()
        });
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

    let found = frames(ring.flash().image(), geometry());
    let capture = capture(&found);
    let files = demux(&capture);
    let payloads = verified(&files); // every line verifies under its own routing, or this panics

    for thread in 0..THREADS {
        let fast = &payloads[&Some(format!("{SESSION}_t{thread}_fast.csv"))];
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
            let mut expected = Vec::new();
            recorder_wire::row(&mut expected, uptime, &fast_row(thread, index as u32));
            assert_eq!(row.as_bytes(), expected, "thread {thread} row {index}");
        }
        let events = &payloads[&Some(format!("{SESSION}_t{thread}_events.csv"))];
        assert_eq!(events[0], "uptime;index;event");
        assert_eq!(events.len(), (ROWS as usize).div_ceil(100) + 1);
    }
    let session = &payloads[&Some("session.csv".to_string())];
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
    let log = &payloads[&None];
    let mut per_source: BTreeMap<String, Vec<u64>> = BTreeMap::new();
    for line in log {
        let source = line.split(' ').nth(1).unwrap().to_string();
        per_source.entry(source).or_default().push(uptime(line));
    }
    for thread in 0..THREADS {
        let uptimes = &per_source[&format!("thread{thread}")];
        assert_eq!(uptimes.len() as u32, logged[thread].load(Ordering::Relaxed));
        assert!(uptimes.is_sorted(), "thread {thread}'s lines keep their order");
    }
    assert_eq!(per_source["vprintf"].len() as u32, misc_logged.load(Ordering::Relaxed));
    assert!(
        !per_source["Recorder"].is_empty(),
        "status lines, the last one at the end"
    );

    host_tools(&capture, &files);
}

/// The repository's tools/ directory.
fn tools() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../tools")
}

/// The host tools' check, run by python3 over the files this test wrote: prints one JSON summary.
const CHECK: &str = r#"
import json, os, sys
tools, directory = sys.argv[1], sys.argv[2]
sys.path.insert(0, tools)
import flight_telemetry, recorder_wire
with open(os.path.join(directory, 'capture.txt'), encoding='utf-8', errors='surrogateescape') as handle:
    text = handle.read()
lines = text.splitlines()
statuses = {}
for line in lines:
    status = recorder_wire.split(line)[0]
    statuses[status] = statuses.get(status, 0) + 1
streams, logs = flight_telemetry.parse(text)
files = {}
for name in sorted(os.listdir(os.path.join(directory, 'files'))):
    with open(os.path.join(directory, 'files', name), encoding='utf-8', errors='surrogateescape') as handle:
        routing = None if name == 'recorder.log' else name
        rows = handle.read().splitlines()
        files[name] = sum(recorder_wire.verify(routing, row) is not None for row in rows), len(rows)
print(json.dumps({'lines': len(lines), 'statuses': statuses, 'counts': flight_telemetry.line_counts(),
                  'spliced': flight_telemetry.spliced(streams), 'rows': {name: len(stream.rows) for name, stream
                  in streams.items()}, 'logs': len(logs), 'files': files}))
"#;

/// Write the capture and the demuxed files, then run [`CHECK`] and assert its verdict.
fn host_tools(capture: &[u8], files: &BTreeMap<Option<String>, Vec<Vec<u8>>>) {
    let directory = std::env::temp_dir().join(format!("coludo-recorder-end-to-end-{}", std::process::id()));
    std::fs::create_dir_all(directory.join("files")).unwrap();
    std::fs::write(directory.join("capture.txt"), capture).unwrap();
    for (routing, lines) in files {
        let name = routing.as_deref().unwrap_or("recorder.log");
        std::fs::write(directory.join("files").join(name), lines.concat()).unwrap();
    }
    let output = std::process::Command::new("python3")
        .arg("-c")
        .arg(CHECK)
        .arg(tools())
        .arg(&directory)
        .output();
    let output = match output {
        Ok(output) => output,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            eprintln!("note: python3 is not installed, so the host tools' check is skipped");
            return;
        }
        Err(error) => panic!("python3 did not run: {error}"),
    };
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
    assert_eq!(verdict["spliced"], serde_json::json!([]), "{}", kept());
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
