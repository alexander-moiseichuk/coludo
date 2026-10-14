//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Power cuts at the recorder level (doc/specs/avionics-port.md "The ring_writer", its power-cut bound; "Block and
//! chunk format", the torn chunk). Boot 123 records rows, durable events and log lines into a flash ring, the
//! eraser stepping beside the writer as its thread would. A durable event wakes the writer, which steps at once.
//! The boot runs once whole, then once per flash operation it made (programs and erases), with the power cut
//! there and the operation torn at several lengths. Boot 124 then brings the ring up again and records. After
//! every cut:
//! * boot 123's lines read back through the decoder, as the ring_saver reads them, as a gapless and duplicate-free
//!   prefix of what it queued, each stream under its header, and with a loss line for every torn chunk the
//!   decoder calls a loss;
//! * every record that a step had programmed before the cut is there, and the oldest record missing is younger
//!   than the bound: a row one tick (the writer's step), a durable event nothing (its wake carried it), a log
//!   line one tick and the merge horizon; over a foreign ring, whose first block waits for the eraser, one tick
//!   more. The spec's bound adds what the host flash does not take: an erase and an NVS page erase holding the
//!   flash, which gives at most ~45 ms for rows, ~35 ms for a durable record and ~65 ms for log lines;
//! * boot 124 opens the block after boot 123's newest, leaves boot 123's lines as they were, and every line it
//!   records reads back whole.
//!
//! Both over an erased ring and over a ring of foreign bytes that the eraser formats while boot 123 writes. A ring
//! that fills, with newest wins erasing next to a refused writer, is cut in newest_wins_cut.rs.

mod common;

use common::*;

use flash_ring::{Eraser, Geometry, MemoryFlash};
use recorder::{Cell, MERGE_HORIZON_US, Registry, Settings, StreamId, TICK_US, Telemetry, Writer};

/// 24 blocks: small enough to cut the power at every operation, big enough for several blocks per boot.
fn geometry() -> Geometry {
    Geometry::new(24).unwrap()
}

/// Steps in which boot 123's producers queue records: about five blocks.
const PRODUCING_STEPS: u64 = 100;

/// Steps after them, until everything queued is out.
const SETTLING_STEPS: u64 = 4;

/// The tear lengths tried at each operation: nothing, inside a chunk header, one short of it, the whole of it,
/// one more, inside a record, around the 56-byte block header, inside a payload or an erased prefix of a sector,
/// and a call that completed but never returned.
const TEARS: [usize; 11] = [0, 1, 7, 8, 9, 30, 55, 56, 57, 300, 5000];

/// The boot after the cut.
const NEXT_BOOT: u32 = BOOT_ID + 1;

/// The spec's power-cut bound for rows, which the host's own bound must stay inside.
const SPEC_ROWS_US: u64 = 45_000;

/// The spec's bound for a durable record.
const SPEC_DURABLE_US: u64 = 35_000;

/// The spec's bound for log lines.
const SPEC_LOGS_US: u64 = 65_000;

/// A boot's recorder and its two streams.
struct Boot {
    /// The Recorder, with no attachable ring: the test thread queues through `misc`.
    parts: Parts,
    /// `<session>_imu.csv`: three rows a step.
    imu: StreamId,
    /// `<session>_flight_events.csv`: durable, a row every seventh step.
    events: StreamId,
}

impl Boot {
    /// Boot `boot_id`'s recorder, with small buffers: a sweep makes thousands of them, and each leaks its own.
    fn new(boot_id: u32) -> Boot {
        let mut registry = Registry::new();
        let imu = registry.add("imu.csv", &["index", "value"], -1, false).unwrap();
        let events = registry.add("flight_events.csv", &["event"], -1, true).unwrap();
        let settings = Settings {
            telemetry_ms: 0,
            stats_ms: 200,
        };
        Boot {
            parts: recorder_of(boot_id, registry, 0, 16 << 10, settings, false),
            imu,
            events,
        }
    }
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
}

impl Board<'_> {
    /// One writer step and one eraser step at `now_us`, the clock moved there. Returns true once the power is
    /// cut: the board is dead from then on.
    fn tick(&mut self, clock: &TestClock, now_us: u64) -> bool {
        clock.set(now_us);
        self.writer.step(now_us, EPOCH);
        let _ = self.eraser.erase_step(None, (now_us / 1000) as u32); // a failed erase: the board is dead
        self.cut.is_some_and(|operation| operations(self.flash) > operation)
    }
}

/// The program and erase calls made on `flash` so far, a torn one included.
fn operations(flash: &MemoryFlash) -> u32 {
    flash.programs() + flash.erases()
}

/// A record a producer queued: its uptime, and the payload it must read back as.
#[derive(Clone, Debug)]
struct Queued {
    /// Its uptime.
    uptime_us: u64,
    /// Its line's payload, as the decoder prints it.
    payload: String,
}

/// What a boot queued, per file, in order, and how much of it a completed step had programmed.
#[derive(Debug, Default)]
struct Workload {
    /// The IMU rows.
    imu: Vec<Queued>,
    /// The durable events.
    events: Vec<Queued>,
    /// The producer's log lines.
    logs: Vec<Queued>,
    /// The IMU rows, events and log lines a completed step left with nothing of their class unprogrammed: on
    /// flash, whatever came after.
    programmed: [usize; 3],
}

/// What boot 123 left.
struct FirstBoot {
    /// The flash content after the cut, or after the whole run.
    image: Vec<u8>,
    /// What its producers queued.
    workload: Workload,
    /// The uptime of the step the cut hit; None without a cut.
    cut_at: Option<u64>,
    /// Program and erase calls made.
    operations: u32,
}

/// Boot 123 over `image`, with the power cut at operation `cut` torn at its given length (None: never). Its
/// producers queue three IMU rows a step and a log line every third; every seventh step a durable event first,
/// which wakes the writer at once. It stops at the step the cut hit: the board is dead.
fn first_boot(image: &[u8], cut: Option<(u32, usize)>) -> FirstBoot {
    let flash = MemoryFlash::from_image(image.to_vec());
    if let Some((operation, torn)) = cut {
        flash.cut_power(operation, torn);
    }
    let scanned = bring_up(&flash, geometry());
    let boot = Boot::new(BOOT_ID);
    let (recorder, clock, wakes) = (boot.parts.recorder, &boot.parts.clock, &boot.parts.wakes);
    let mut board = Board {
        flash: &flash,
        writer: Writer::new(recorder, ring(&flash, &scanned, BOOT_ID)).unwrap(),
        eraser: eraser(&flash, &scanned),
        cut: cut.map(|(operation, _)| operation),
    };
    let mut imu = Telemetry::new(recorder, boot.imu).unwrap();
    let mut events = Telemetry::new(recorder, boot.events).unwrap();
    let mut workload = Workload::default();
    let mut cut_at = None;
    'steps: for step in 0..PRODUCING_STEPS + SETTLING_STEPS {
        let now = START_US + step * TICK_US;
        if step < PRODUCING_STEPS {
            if step % 7 == 3 {
                let uptime_us = now - 9500;
                let cells = [Cell::Text("mark")];
                events.push(uptime_us, &cells).unwrap();
                workload.events.push(Queued {
                    uptime_us,
                    payload: row_text(uptime_us, &cells),
                });
                assert!(wakes.take(), "a durable record wakes the writer");
                if board.tick(clock, uptime_us) {
                    cut_at = Some(uptime_us);
                    break 'steps;
                }
                mark_programmed(recorder, &mut workload);
            }
            for (back, index) in [9000, 6000, 3000].into_iter().zip(0..) {
                let uptime_us = now - back;
                let index = (step * 3 + index) as i32;
                let cells = [Cell::I32(index), Cell::F32(index as f32 * 0.25)];
                imu.push(uptime_us, &cells).unwrap();
                workload.imu.push(Queued {
                    uptime_us,
                    payload: row_text(uptime_us, &cells),
                });
            }
            if step % 3 == 0 {
                let uptime_us = now - 5000;
                clock.set(uptime_us);
                let message = format!("step {step}");
                assert!(recorder.log("producer", &message));
                let mut line = Vec::new();
                recorder_wire::log_line(&mut line, uptime_us, "producer", &message, usize::MAX);
                workload.logs.push(Queued {
                    uptime_us,
                    payload: String::from_utf8(line).unwrap(),
                });
            }
        }
        if board.tick(clock, now) {
            cut_at = Some(now);
            break;
        }
        mark_programmed(recorder, &mut workload);
    }
    drop(board);
    FirstBoot {
        operations: operations(&flash),
        image: flash.into_image(),
        workload,
        cut_at,
    }
}

/// After a completed step: a class with nothing left unprogrammed has all it queued so far on flash.
fn mark_programmed(recorder: &recorder::Recorder, workload: &mut Workload) {
    let stats = recorder.stats();
    if stats.telemetry.count == 0 {
        workload.programmed[0] = workload.imu.len();
        workload.programmed[1] = workload.events.len();
    }
    if stats.log.count == 0 {
        workload.programmed[2] = workload.logs.len();
    }
}

/// Check one file's read-back against what was queued for it: its header first (when it has one, and anything
/// at all came back), then a gapless, duplicate-free prefix of the queue, at least `required` long.
///
/// Returns how long before `cut_at` the oldest missing record was queued (0 when none is missing).
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
        "{context}: {} records back, {required} were programmed by a completed step",
        rows.len()
    );
    queued
        .get(rows.len())
        .map_or(0, |missing| cut_at.saturating_sub(missing.uptime_us))
}

/// A boot's session.csv rows: the header, and its `boot` row at the first step.
fn session_rows(boot: u32) -> Vec<String> {
    vec![
        recorder_wire::SESSION_INDEX_HEADER.to_string(),
        format!("{START_US};{boot};{boot:06};2026-10-03T14:21:07Z;;taster;dev;3f2a91;boot;;"),
    ]
}

/// The oldest losses a cut caused, in us before the cut.
#[derive(Clone, Copy, Debug, Default)]
struct Loss {
    /// The oldest IMU row missing.
    row_us: u64,
    /// The oldest durable event missing.
    durable_us: u64,
    /// The oldest log line missing.
    log_us: u64,
}

/// Check boot 123's read-back against its workload and the cut.
fn check_first_boot(read_back: &ReadBack, workload: &Workload, cut_at: Option<u64>, context: &str) -> Loss {
    let cut = cut_at.unwrap_or(u64::MAX);
    let row_us = check_prefix(
        &read_back.file(Some("000123_imu.csv")),
        &workload.imu,
        Some("uptime;index;value"),
        workload.programmed[0],
        cut,
        &format!("{context} imu"),
    );
    let durable_us = check_prefix(
        &read_back.file(Some("000123_flight_events.csv")),
        &workload.events,
        Some("uptime;event"),
        workload.programmed[1],
        cut,
        &format!("{context} events"),
    );
    let log = read_back.file(None);
    let producer: Vec<String> = log
        .iter()
        .filter(|line| line.contains(" producer :: "))
        .cloned()
        .collect();
    let log_us = check_prefix(
        &producer,
        &workload.logs,
        None,
        workload.programmed[2],
        cut,
        &format!("{context} log"),
    );
    let losses = log.iter().filter(|line| line.contains(" ring :: block ")).count();
    assert_eq!(
        losses as u32, read_back.lost,
        "{context}: a loss line per torn chunk lost"
    );
    let index = session_rows(BOOT_ID);
    let session = read_back.file(Some("session.csv"));
    assert_eq!(*session, index[..session.len()], "{context}: session.csv");
    if cut_at.is_none() {
        assert_eq!(session.len(), 2, "{context}: the boot row");
        assert_eq!(
            (row_us, durable_us, log_us),
            (0, 0, 0),
            "{context}: everything, with no cut"
        );
    }
    Loss {
        row_us,
        durable_us,
        log_us,
    }
}

/// Boot 124 over what boot 123 left: it records twenty steps of rows and an event, then settles. Checks that it
/// opens the block after boot 123's newest, leaves boot 123's lines as they were, and reads back whole.
fn check_next_boot(image: Vec<u8>, before: &ReadBack, context: &str) {
    let flash = MemoryFlash::from_image(image);
    let scanned = bring_up(&flash, geometry());
    let boot = Boot::new(NEXT_BOOT);
    let (recorder, clock) = (boot.parts.recorder, &boot.parts.clock);
    let mut board = Board {
        flash: &flash,
        writer: Writer::new(recorder, ring(&flash, &scanned, NEXT_BOOT)).unwrap(),
        eraser: eraser(&flash, &scanned),
        cut: None,
    };
    let mut imu = Telemetry::new(recorder, boot.imu).unwrap();
    let mut events = Telemetry::new(recorder, boot.events).unwrap();
    let mut workload = Workload::default();
    for step in 0..40 {
        let now = START_US + step * TICK_US;
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
        board.tick(clock, now);
    }
    assert_eq!(recorder.stats().telemetry.count, 0, "{context}: boot 124 settled");
    assert!(
        recorder.writer_position().is_some(),
        "{context}: published for the saver"
    );
    drop(board);
    let image = flash.into_image();

    let after = read_back(&image, geometry(), Some("000123"));
    assert_eq!(
        after.files, before.files,
        "{context}: boot 124 changed boot 123's lines"
    );
    let next = read_back(&image, geometry(), Some("000124"));
    assert_eq!(next.lost, 0, "{context}: boot 124 tore nothing");
    let first = before.blocks.last().map_or(0, |newest| newest + 1);
    assert_eq!(
        next.blocks.first(),
        Some(&first),
        "{context}: the block after boot 123's newest"
    );
    let rows = |queued: &[Queued], header: &str| -> Vec<String> {
        std::iter::once(header.to_string())
            .chain(queued.iter().map(|record| record.payload.clone()))
            .collect()
    };
    assert_eq!(
        next.file(Some("000124_imu.csv")),
        rows(&workload.imu, "uptime;index;value"),
        "{context}"
    );
    assert_eq!(
        next.file(Some("000124_flight_events.csv")),
        rows(&workload.events, "uptime;event"),
        "{context}"
    );
    assert_eq!(next.file(Some("session.csv")), session_rows(NEXT_BOOT), "{context}");
}

/// What a sweep saw.
#[derive(Debug, Default)]
struct Sweep {
    /// The flash operations of the whole run, each cut in turn.
    operations: u32,
    /// The oldest losses over every cut.
    worst: Loss,
    /// The cuts after which the decoder logged a torn chunk as a loss.
    torn: u32,
}

/// Run boot 123 whole over `image`, then cut it at every operation and every tear, each followed by boot 124.
fn sweep(image: &[u8], name: &str) -> Sweep {
    let whole = first_boot(image, None);
    assert_eq!(whole.cut_at, None);
    let read = read_back(&whole.image, geometry(), Some("000123"));
    assert_eq!(read.lost, 0, "{name}: nothing torn without a cut");
    check_first_boot(&read, &whole.workload, None, name);
    check_next_boot(whole.image.clone(), &read, name);
    let mut sweep = Sweep {
        operations: whole.operations,
        ..Sweep::default()
    };
    for operation in 0..whole.operations {
        for torn in TEARS {
            let context = format!("{name}: cut at operation {operation}, {torn} bytes");
            let cut = first_boot(image, Some((operation, torn)));
            let cut_at = cut.cut_at.unwrap_or_else(|| panic!("{context}: the cut never came"));
            let read = read_back(&cut.image, geometry(), Some("000123"));
            let loss = check_first_boot(&read, &cut.workload, Some(cut_at), &context);
            sweep.worst.row_us = sweep.worst.row_us.max(loss.row_us);
            sweep.worst.durable_us = sweep.worst.durable_us.max(loss.durable_us);
            sweep.worst.log_us = sweep.worst.log_us.max(loss.log_us);
            sweep.torn += u32::from(read.lost > 0);
            check_next_boot(cut.image, &read, &context);
        }
    }
    eprintln!("{name} ring: {} tears per operation, {sweep:?}", TEARS.len());
    sweep
}

/// The sweep's own checks: it cut many operations, left torn chunks the decoder logged, and stayed inside the
/// host's bound, which is inside the spec's. `waits` is the ticks the first block may wait for the eraser.
fn check_sweep(sweep: &Sweep, fewest_operations: u32, waits: u64) {
    assert!(sweep.operations >= fewest_operations, "{sweep:?}");
    assert!(
        sweep.torn > 0,
        "some cut tore a chunk the decoder logged as lost: {sweep:?}"
    );
    assert!(sweep.worst.row_us <= (1 + waits) * TICK_US, "{sweep:?}");
    assert!(sweep.worst.durable_us <= waits * TICK_US, "{sweep:?}");
    assert!(
        sweep.worst.log_us <= (1 + waits) * TICK_US + MERGE_HORIZON_US,
        "{sweep:?}"
    );
    assert!(
        sweep.worst.row_us <= SPEC_ROWS_US
            && sweep.worst.durable_us <= SPEC_DURABLE_US
            && sweep.worst.log_us <= SPEC_LOGS_US,
        "{sweep:?}"
    );
}

#[test]
fn a_cut_over_an_erased_ring_loses_only_the_newest_records() {
    let sweep = sweep(&vec![0xFF; geometry().bytes() as usize], "erased");
    check_sweep(&sweep, 100, 0);
}

#[test]
fn a_cut_while_the_eraser_formats_a_foreign_ring_loses_only_the_newest_records() {
    let sweep = sweep(&vec![0; geometry().bytes() as usize], "foreign");
    check_sweep(&sweep, 120, 1);
}
