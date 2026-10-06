//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Power cuts at the recorder level (doc/specs/avionics-port.md "The flash writer", its power-cut bound, and
//! "SD mover", the torn frame). Boot 123 records rows, durable events and log lines into a flash ring. It runs
//! once whole, then once per flash operation it made, with the power cut there and the operation torn at several
//! lengths. Boot 124 then mounts what is left and records again. After every cut:
//! * boot 123's lines read back, as the mover files them (its whole frames, then the lines of the torn frame
//!   whose wrapper verifies), as a gapless and duplicate-free prefix of what it queued, each stream under its
//!   header;
//! * every row queued more than `CUT_AGE_US` and one step before the cut is among them (a log line: plus the
//!   merge horizon and one more step), and so is every durable event whose step ended with nothing spilled;
//! * boot 124 opens the block after boot 123's newest, leaves boot 123's lines as they were, and every line it
//!   records reads back whole, its session.csv rows included.
//!
//! Both over an erased ring and over a ring of foreign bytes that GC formats while boot 123 writes, spilling
//! until the first block is erased.

mod common;

use std::collections::BTreeMap;

use common::{BOOT_ID, EPOCH, OPTIONS, START_US, TestClock, buffer};
use flash_ring::frame::{self, HEADER_BYTES, Header, Kind, frame_bytes};
use flash_ring::{Geometry, MemoryFlash, Ring};
use recorder::{
    Buffers, CUT_AGE_US, Cell, Identity, MERGE_HORIZON_US, Recorder, Registry, Settings, StreamId, Telemetry, Writer,
};

/// 16 blocks of one 4 KiB sector: small enough to cut the power at every operation, big enough for several
/// blocks per boot.
fn geometry() -> Geometry {
    Geometry::new(16, 1).unwrap()
}

/// The writer's step period, as the firmware's loop runs it at most.
const STEP_US: u64 = 10_000;

/// Steps in which boot 123's producers queue records: enough for ten blocks.
const PRODUCING_STEPS: u64 = 150;

/// Steps after them, until everything queued is out.
const SETTLING_STEPS: u64 = 12;

/// The tear lengths tried at each operation: nothing, inside a frame header, one byte short of it, the whole
/// header, one byte more, inside the payload (or an erased prefix of a sector), and a call that completed but
/// never returned.
const TEARS: [usize; 8] = [0, 1, 39, 40, 41, 100, 1000, 100_000];

/// The boot after the cut.
const NEXT_BOOT: u32 = BOOT_ID + 1;

/// A boot's recorder: the shared state, the clock its log lines read, and its two streams.
struct Boot {
    /// The Recorder, with no attachable ring: the test thread queues through `misc`.
    recorder: Recorder,
    /// The uptime the test sets.
    clock: TestClock,
    /// `<session>_imu.csv`: three rows a step.
    imu: StreamId,
    /// `<session>_flight_events.csv`: durable, a row every seventh step.
    events: StreamId,
}

impl Boot {
    /// Boot `boot_id`'s recorder, with small buffers: a sweep makes a thousand of them, and each leaks its own.
    fn new(boot_id: u32) -> Boot {
        let mut registry = Registry::new(&format!("{boot_id:06}")).unwrap();
        let imu = registry.add("imu.csv", &["index", "value"], -1, false).unwrap();
        let events = registry.add("flight_events.csv", &["event"], -1, true).unwrap();
        let clock = TestClock::default();
        let reading = clock.clone();
        let buffers = Buffers {
            producers: Vec::new(),
            misc: buffer(8 << 10),
            log_tee: buffer(64),
            telemetry_tee: buffer(64),
        };
        let identity = Identity {
            boot_id: Some(boot_id),
            board: "taster".into(),
            firmware: "dev".into(),
            config_id: "3f2a91".into(),
        };
        let settings = Settings {
            telemetry_ms: 0,
            stats_ms: 200,
        };
        Boot {
            recorder: Recorder::new(registry, identity, settings, buffers, Box::new(move || reading.now())),
            clock,
            imu,
            events,
        }
    }

    /// The flash ring over `flash`, mounted for this boot.
    fn ring(&self, flash: MemoryFlash, boot_id: u32) -> Ring<MemoryFlash> {
        Ring::mount(
            flash,
            geometry(),
            OPTIONS,
            boot_id,
            &self.recorder.boot_payload("poweron", None),
        )
        .unwrap()
    }
}

/// A record a producer queued: its uptime, and the payload it must read back as.
#[derive(Clone, Debug)]
struct Queued {
    /// Its uptime.
    uptime_us: u64,
    /// Its line's payload, as the writer prints it.
    payload: String,
}

/// What a boot queued, per file, in order.
#[derive(Debug, Default)]
struct Workload {
    /// The IMU rows.
    imu: Vec<Queued>,
    /// The durable events.
    events: Vec<Queued>,
    /// The producer's log lines.
    logs: Vec<Queued>,
    /// The events whose step ended with nothing spilled: programmed, so on flash whatever comes after.
    events_programmed: usize,
}

/// A row's payload, as the writer prints it.
fn row_text(uptime_us: u64, cells: &[Cell]) -> String {
    let mut line = Vec::new();
    recorder_wire::row(&mut line, uptime_us, cells);
    String::from_utf8(line).unwrap()
}

/// The program and erase calls `writer` made so far, a torn one included.
fn operations(writer: &Writer<'_, MemoryFlash>) -> u32 {
    writer.ring().flash().programs() + writer.ring().flash().erases()
}

/// What boot 123 left: the flash image, what it queued, the uptime of the step the cut hit (None: no cut), and
/// the flash operations it made.
struct FirstBoot {
    /// The flash content after the cut, or after the whole run.
    image: Vec<u8>,
    /// What its producers queued.
    workload: Workload,
    /// The uptime of the step the cut hit.
    cut_at: Option<u64>,
    /// Program and erase calls.
    operations: u32,
}

/// Boot 123 over `image`, with the power cut at operation `cut` torn at its given length (None: never). Its
/// producers queue three IMU rows a step, a log line every third and an event every seventh; then it settles.
/// It stops at the step the cut hit: the board is dead.
fn first_boot(image: &[u8], cut: Option<(u32, usize)>) -> FirstBoot {
    let boot = Boot::new(BOOT_ID);
    let mut flash = MemoryFlash::from_image(image.to_vec());
    if let Some((operation, torn)) = cut {
        flash.cut_power(operation, torn);
    }
    let mut writer = Writer::new(&boot.recorder, boot.ring(flash, BOOT_ID)).unwrap();
    let mut imu = Telemetry::new(&boot.recorder, boot.imu).unwrap();
    let mut events = Telemetry::new(&boot.recorder, boot.events).unwrap();
    let mut workload = Workload::default();
    let mut cut_at = None;
    for step in 0..PRODUCING_STEPS + SETTLING_STEPS {
        let now = START_US + step * STEP_US;
        if step < PRODUCING_STEPS {
            for (back, index) in [9000, 6000, 3000].into_iter().zip(0..) {
                let uptime_us = now - back;
                let index = (step * 3 + index) as i32;
                let cells = [Cell::I32(index), Cell::F32(index as f32 * 0.25)];
                imu.push(uptime_us, &cells).unwrap();
                let payload = row_text(uptime_us, &cells);
                workload.imu.push(Queued { uptime_us, payload });
            }
            if step % 3 == 0 {
                let uptime_us = now - 5000;
                boot.clock.set(uptime_us);
                let message = format!("step {step}");
                assert!(boot.recorder.log("producer", &message));
                let mut line = Vec::new();
                recorder_wire::log_line(&mut line, uptime_us, "producer", &message, usize::MAX);
                let payload = String::from_utf8(line).unwrap();
                workload.logs.push(Queued { uptime_us, payload });
            }
            if step % 7 == 3 {
                let uptime_us = now - 1000;
                let cells = [Cell::Text("mark")];
                events.push(uptime_us, &cells).unwrap();
                let payload = row_text(uptime_us, &cells);
                workload.events.push(Queued { uptime_us, payload });
            }
        }
        boot.clock.set(now);
        writer.step(now, EPOCH);
        if let Some((operation, _)) = cut
            && operations(&writer) > operation
        {
            cut_at = Some(now);
            break;
        }
        if writer.ring().figures().spill_frames == 0 {
            workload.events_programmed = workload.events.len();
        }
    }
    let operations = operations(&writer);
    FirstBoot {
        image: writer.into_ring().into_flash().into_image(),
        workload,
        cut_at,
        operations,
    }
}

/// A boot's lines per file (None: recorder.log), as payloads.
type Files = BTreeMap<Option<String>, Vec<String>>;

/// File each line of `bytes` (whole wire lines) under its routing. Every line must verify under it.
fn file_lines(files: &mut Files, bytes: &[u8]) {
    let tail = recorder_wire::split(bytes, |routing, line| {
        let body = line.strip_suffix(b"\n").expect("a split line keeps its line end");
        let payload = recorder_wire::verify(routing, body)
            .unwrap_or_else(|| panic!("a whole line fails its checks: {}", String::from_utf8_lossy(line)));
        let routing = routing.map(|routing| String::from_utf8(routing.to_vec()).unwrap());
        files
            .entry(routing)
            .or_default()
            .push(String::from_utf8(payload.to_vec()).unwrap());
    });
    assert!(tail.is_empty(), "an unterminated line in whole lines");
}

/// What a boot left on a flash image, read as the SD mover reads it.
struct ReadBack {
    /// Its lines, per file.
    files: Files,
    /// Its blocks' sequence numbers, oldest first.
    blocks: Vec<u32>,
    /// The whole lines salvaged from a torn DATA frame, None when no frame was torn.
    torn: Option<usize>,
}

/// Boot `boot`'s lines on `image`: each of its blocks (those whose BOOT header names it) in sequence order, every
/// whole DATA frame, then the whole lines of a torn DATA frame behind them, whose unterminated tail is lost.
fn read_boot(image: &[u8], boot: u32) -> ReadBack {
    let block_bytes = geometry().block_bytes() as usize;
    let mut blocks: Vec<(u32, &[u8])> = image
        .chunks(block_bytes)
        .filter_map(|block| {
            let header = Header::decode(block).ok()?;
            (header.kind == Kind::Boot && header.boot_id == boot).then_some((header.block_sequence, block))
        })
        .collect();
    blocks.sort_by_key(|(sequence, _)| *sequence);
    let mut read_back = ReadBack {
        files: Files::new(),
        blocks: blocks.iter().map(|(sequence, _)| *sequence).collect(),
        torn: None,
    };
    for (sequence, block) in blocks {
        let mut end = 0;
        for found in frame::frames(block).take_while(|found| found.header.block_sequence == sequence) {
            assert_eq!(found.header.boot_id, boot, "a boot writes only into blocks it opened");
            if found.header.kind == Kind::Data {
                file_lines(&mut read_back.files, found.payload);
            }
            end = found.offset as usize + frame_bytes(found.payload.len());
        }
        if let Some(rest) = block.get(end..)
            && let Ok(header) = Header::decode(rest)
            && header.kind == Kind::Data
        {
            assert_eq!((header.boot_id, header.block_sequence), (boot, sequence));
            let programmed = &rest[HEADER_BYTES..rest.len().min(HEADER_BYTES + header.payload_length as usize)];
            let whole = programmed
                .iter()
                .rposition(|&byte| byte == b'\n')
                .map_or(0, |last| last + 1);
            file_lines(&mut read_back.files, &programmed[..whole]);
            assert!(read_back.torn.is_none(), "one torn frame at most");
            read_back.torn = Some(programmed[..whole].iter().filter(|&&byte| byte == b'\n').count());
        }
    }
    read_back
}

/// Check one file's read-back against what was queued for it: its header first (when it has one, and anything
/// at all came back), then a gapless, duplicate-free prefix of the queue, at least `required` long.
///
/// Returns how long ago before `cut_at` the oldest missing record was queued (0 when none is missing).
fn check_prefix(
    found: &[String],
    queued: &[Queued],
    header: Option<&str>,
    required: usize,
    cut_at: u64,
    context: &str,
) -> u64 {
    let rows = match header {
        Some(header) if !found.is_empty() => {
            assert_eq!(found[0], header, "{context}: the header first");
            &found[1..]
        }
        _ => found,
    };
    assert!(rows.len() <= queued.len(), "{context}: more than was queued");
    for (index, (row, queued)) in rows.iter().zip(queued).enumerate() {
        assert_eq!(
            *row, queued.payload,
            "{context}: record {index} is not the one queued there"
        );
    }
    assert!(
        rows.len() >= required,
        "{context}: {} records back, {required} were old enough to be on flash",
        rows.len()
    );
    queued
        .get(rows.len())
        .map_or(0, |missing| cut_at.saturating_sub(missing.uptime_us))
}

/// Of `queued`, how many were queued more than `slack` before `cut_at` (all of them without a cut).
fn old_enough(queued: &[Queued], cut_at: Option<u64>, slack: u64) -> usize {
    cut_at.map_or(queued.len(), |cut_at| {
        queued
            .iter()
            .take_while(|record| record.uptime_us + slack < cut_at)
            .count()
    })
}

/// A boot's session.csv rows: the header, and its `boot` row at the first step.
fn session_rows(boot: u32) -> Vec<String> {
    vec![
        recorder_wire::SESSION_INDEX_HEADER.to_string(),
        format!("{START_US};{boot};{boot:06};2026-10-03T14:21:07Z;;taster;dev;3f2a91;boot;;"),
    ]
}

/// The oldest losses a cut caused, in us before the cut: of a row (or event), and of a log line.
#[derive(Clone, Copy, Debug, Default)]
struct Loss {
    /// The oldest row or event missing.
    row_us: u64,
    /// The oldest log line missing.
    log_us: u64,
}

/// Check boot 123's read-back against its workload and the cut.
fn check_first_boot(read_back: &ReadBack, workload: &Workload, cut_at: Option<u64>, context: &str) -> Loss {
    let empty = Vec::new();
    let file = |routing: Option<&str>| read_back.files.get(&routing.map(String::from)).unwrap_or(&empty);
    let cut = cut_at.unwrap_or(u64::MAX);
    let row_slack = CUT_AGE_US + STEP_US;
    let imu = check_prefix(
        file(Some("000123_imu.csv")),
        &workload.imu,
        Some("uptime;index;value"),
        old_enough(&workload.imu, cut_at, row_slack),
        cut,
        &format!("{context} imu"),
    );
    let required = old_enough(&workload.events, cut_at, row_slack).max(workload.events_programmed);
    let events = check_prefix(
        file(Some("000123_flight_events.csv")),
        &workload.events,
        Some("uptime;event"),
        required,
        cut,
        &format!("{context} events"),
    );
    let producer: Vec<String> = file(None)
        .iter()
        .filter(|line| line.contains(" producer :: "))
        .cloned()
        .collect();
    let logs = check_prefix(
        &producer,
        &workload.logs,
        None,
        old_enough(&workload.logs, cut_at, MERGE_HORIZON_US + CUT_AGE_US + 2 * STEP_US),
        cut,
        &format!("{context} log"),
    );
    let index = session_rows(BOOT_ID);
    let session = file(Some("session.csv"));
    assert_eq!(*session, index[..session.len()], "{context}: session.csv");
    if START_US + row_slack < cut {
        assert_eq!(session.len(), 2, "{context}: the boot row is old enough");
    }
    Loss {
        row_us: imu.max(events),
        log_us: logs,
    }
}

/// Boot 124 over what boot 123 left: it records twenty steps of rows and an event, then settles. Checks that it
/// opens the block after boot 123's newest, leaves boot 123's lines as they were, and reads back whole.
fn check_next_boot(image: Vec<u8>, before: &ReadBack, context: &str) {
    let boot = Boot::new(NEXT_BOOT);
    let mut writer = Writer::new(&boot.recorder, boot.ring(MemoryFlash::from_image(image), NEXT_BOOT)).unwrap();
    assert_eq!(
        boot.recorder.programmed(),
        None,
        "{context}: None until the writer's first step"
    );
    let mut imu = Telemetry::new(&boot.recorder, boot.imu).unwrap();
    let mut events = Telemetry::new(&boot.recorder, boot.events).unwrap();
    let mut workload = Workload::default();
    for step in 0..60 {
        let now = START_US + step * STEP_US;
        if step < 20 {
            let cells = [Cell::I32(step as i32), Cell::F32(0.5)];
            imu.push(now - 5000, &cells).unwrap();
            workload.imu.push(Queued {
                uptime_us: now - 5000,
                payload: row_text(now - 5000, &cells),
            });
        }
        if step == 10 {
            let cells = [Cell::Text("next")];
            events.push(now - 1000, &cells).unwrap();
            workload.events.push(Queued {
                uptime_us: now - 1000,
                payload: row_text(now - 1000, &cells),
            });
        }
        boot.clock.set(now);
        writer.step(now, EPOCH);
    }
    assert_eq!(writer.ring().figures().spill_frames, 0, "{context}: boot 124 settled");
    assert!(
        boot.recorder.programmed().is_some(),
        "{context}: published for the mover"
    );
    let image = writer.into_ring().into_flash().into_image();

    let after = read_boot(&image, BOOT_ID);
    assert_eq!(
        after.files, before.files,
        "{context}: boot 124 changed boot 123's lines"
    );
    let next = read_boot(&image, NEXT_BOOT);
    assert_eq!(next.torn, None, "{context}: boot 124 tore nothing");
    let first = before.blocks.last().map_or(0, |newest| newest + 1);
    assert_eq!(
        next.blocks.first(),
        Some(&first),
        "{context}: the block after boot 123's newest"
    );
    let file = |routing: &str| next.files.get(&Some(routing.to_string())).cloned().unwrap_or_default();
    let rows = |queued: &[Queued], header: &str| -> Vec<String> {
        std::iter::once(header.to_string())
            .chain(queued.iter().map(|record| record.payload.clone()))
            .collect()
    };
    assert_eq!(
        file("000124_imu.csv"),
        rows(&workload.imu, "uptime;index;value"),
        "{context}"
    );
    assert_eq!(
        file("000124_flight_events.csv"),
        rows(&workload.events, "uptime;event"),
        "{context}"
    );
    assert_eq!(file("session.csv"), session_rows(NEXT_BOOT), "{context}");
}

/// What a sweep saw.
#[derive(Debug, Default)]
struct Sweep {
    /// The flash operations of the whole run, each cut in turn.
    operations: u32,
    /// The oldest losses over every cut.
    worst: Loss,
    /// The cuts that tore a DATA frame after some of its whole lines, which the salvage then kept.
    salvaged: u32,
}

/// Run boot 123 whole over `image`, then cut it at every operation and every tear, each followed by boot 124.
fn sweep(image: &[u8], name: &str) -> Sweep {
    let whole = first_boot(image, None);
    assert_eq!(whole.cut_at, None);
    let read_back = read_boot(&whole.image, BOOT_ID);
    assert_eq!(read_back.torn, None, "{name}: nothing torn without a cut");
    check_first_boot(&read_back, &whole.workload, None, name);
    check_next_boot(whole.image.clone(), &read_back, name);
    let mut sweep = Sweep {
        operations: whole.operations,
        ..Sweep::default()
    };
    for operation in 0..whole.operations {
        for torn in TEARS {
            let context = format!("{name}: cut at operation {operation}, {torn} bytes");
            let cut = first_boot(image, Some((operation, torn)));
            let cut_at = cut.cut_at.unwrap_or_else(|| panic!("{context}: the cut never came"));
            let read_back = read_boot(&cut.image, BOOT_ID);
            let loss = check_first_boot(&read_back, &cut.workload, Some(cut_at), &context);
            sweep.worst.row_us = sweep.worst.row_us.max(loss.row_us);
            sweep.worst.log_us = sweep.worst.log_us.max(loss.log_us);
            sweep.salvaged += u32::from(read_back.torn.is_some_and(|lines| lines > 0));
            check_next_boot(cut.image, &read_back, &context);
        }
    }
    eprintln!("{name} ring: {} tears per operation, {sweep:?}", TEARS.len());
    sweep
}

/// The sweep's own checks: it cut many operations, salvaged torn frames, and stayed inside the spec's bounds.
fn check_sweep(sweep: &Sweep, fewest_operations: u32) {
    assert!(sweep.operations >= fewest_operations, "{sweep:?}");
    assert!(sweep.salvaged > 0, "some cut tore a frame after whole lines: {sweep:?}");
    assert!(sweep.worst.row_us <= CUT_AGE_US + STEP_US, "{sweep:?}");
    assert!(
        sweep.worst.log_us <= MERGE_HORIZON_US + CUT_AGE_US + 2 * STEP_US,
        "{sweep:?}"
    );
}

#[test]
fn a_cut_over_an_erased_ring_loses_only_the_newest_records() {
    let sweep = sweep(&vec![0xFF; geometry().bytes() as usize], "erased");
    check_sweep(&sweep, 50);
}

#[test]
fn a_cut_while_gc_formats_a_foreign_ring_loses_only_the_newest_records() {
    let sweep = sweep(&vec![0; geometry().bytes() as usize], "foreign");
    check_sweep(&sweep, 60);
}
