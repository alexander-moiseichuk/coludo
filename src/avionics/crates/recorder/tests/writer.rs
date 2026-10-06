//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder over an in-memory flash, step by step with the clock in the test's hand: the session.csv rows
//! and the status line, Telemetry's header and decimation, the log merge across rings, the frame cuts, the CC
//! tees, back-pressure when the flash ring cannot take frames, a failed program's repeat, and the one-writer
//! rule. Several threads at once, and the host tools' verdict, are in end_to_end.rs; power cuts in
//! power_cut.rs.

mod common;

use common::*;

use std::cell::Cell as SharedCell;
use std::rc::Rc;

use flash_ring::frame::{self, HEADER_BYTES, Header, Kind, frame_bytes};
use flash_ring::{Flash, Geometry, MemoryError, MemoryFlash, Options, Ring};
use recorder::{
    ANCHOR_AFTER_US, CUT_AGE_US, Cell, FRAME_BYTES, MERGE_HORIZON_US, Overflow, Recorder, Registry, Settings, StreamId,
    Telemetry, Writer,
};

/// The test geometry: 32 blocks of 4 sectors (16 KiB each, 512 KiB).
fn geometry() -> Geometry {
    Geometry::new(32, 4).unwrap()
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
    let mut registry = Registry::new(SESSION).unwrap();
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

/// A Recorder of `rings` rings of `ring_bytes`, and its writer over a fresh flash ring of `geometry`.
fn setup(
    rings: usize,
    ring_bytes: usize,
    settings: Settings,
) -> (&'static Recorder, TestClock, Streams, Writer<'static, MemoryFlash>) {
    let (registry, streams) = registry();
    let (recorder, clock) = recorder(registry, rings, ring_bytes, settings);
    let writer = Writer::new(recorder, ring(recorder, geometry(), OPTIONS)).unwrap();
    (recorder, clock, streams, writer)
}

/// Step the writer at `now_us`, with the clock moved there too.
fn step<F: Flash>(writer: &mut Writer<'static, F>, clock: &TestClock, now_us: u64) -> recorder::Progress {
    clock.set(now_us);
    writer.step(now_us, EPOCH)
}

/// Step until everything queued is out: past the merge horizon and the cut age.
fn settle<F: Flash>(writer: &mut Writer<'static, F>, clock: &TestClock) {
    let now = clock.now();
    step(writer, clock, now + MERGE_HORIZON_US);
    step(writer, clock, now + MERGE_HORIZON_US + CUT_AGE_US);
}

/// A file's verified payloads (empty when the file does not exist).
fn file(writer: &Writer<'static, MemoryFlash>, routing: Option<&str>) -> Vec<String> {
    read_back(writer.ring())
        .remove(&routing.map(String::from))
        .unwrap_or_default()
}

/// An IMU row of three floats.
fn imu_row(index: u32) -> [Cell<'static>; 3] {
    [Cell::F32(index as f32 * 0.5), Cell::F32(-1.0), Cell::F32(9.81)]
}

#[test]
fn the_boot_row_comes_first_and_cuts_a_frame() {
    let (_recorder, clock, _streams, mut writer) = setup(1, 4096, SETTINGS);
    let progress = step(&mut writer, &clock, START_US);
    assert_eq!(progress.frames, 1, "a session.csv row is durable");
    assert_eq!(
        file(&writer, Some("session.csv")),
        [
            "uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon",
            "10000000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;boot;;",
        ]
    );
    let found = frames(writer.ring().flash().image(), writer.ring().geometry());
    assert_eq!(found[0].header.kind, Kind::Boot);
    assert_eq!(found[1].header.kind, Kind::Data);
    assert!(found[1].header.flags.durable);
    assert_eq!(found[1].header.uptime_us, START_US);
}

#[test]
fn the_boot_row_leads_even_records_queued_before_the_writer() {
    let (registry, streams) = registry();
    let (recorder, clock) = recorder(registry, 0, 4096, SETTINGS);
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    imu.push(START_US - 2000, &imu_row(0)).unwrap();
    clock.set(START_US - 1000);
    recorder.index_session(EPOCH, None, "cc-auto", None).unwrap();
    let mut writer = Writer::new(recorder, ring(recorder, geometry(), OPTIONS)).unwrap();
    step(&mut writer, &clock, START_US);
    settle(&mut writer, &clock);
    let found = frames(writer.ring().flash().image(), writer.ring().geometry());
    let capture = String::from_utf8(capture(&found)).unwrap();
    let mut lines = capture.lines();
    assert!(lines.next().unwrap().starts_with("@session.csv@{"), "the header first");
    assert!(
        lines.next().unwrap().contains(";10000000;123;000123;"),
        "then the boot row"
    );
    let rows = file(&writer, Some("session.csv"));
    assert_eq!(rows.len(), 3);
    assert!(
        rows[2].starts_with("9999000;123;000123;") && rows[2].ends_with(";cc-auto;;"),
        "{rows:?}"
    );
}

#[test]
fn a_time_set_row_follows_the_boot_row_and_the_anchor_comes_a_minute_in() {
    let (recorder, clock, _streams, mut writer) = setup(1, 4096, SETTINGS);
    step(&mut writer, &clock, START_US);
    clock.set(START_US + 5_000_000);
    recorder
        .index_session(EPOCH + 5, Some(-240), "cc-auto", Some((255_144_000, -803_918_000)))
        .unwrap();
    step(&mut writer, &clock, START_US + 5_000_000);
    step(&mut writer, &clock, START_US + ANCHOR_AFTER_US - 1);
    assert_eq!(file(&writer, Some("session.csv")).len(), 3, "no anchor before a minute");
    step(&mut writer, &clock, START_US + ANCHOR_AFTER_US);
    step(&mut writer, &clock, START_US + 2 * ANCHOR_AFTER_US);
    let rows = file(&writer, Some("session.csv"));
    assert_eq!(
        rows[2],
        "15000000;123;000123;2026-10-03T14:21:12Z;-240;taster;dev;3f2a91;cc-auto;25.5144000;-80.3918000"
    );
    assert_eq!(rows[3], rows[0], "the header again before the anchor");
    assert_eq!(
        rows[4],
        "70000000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;anchor;;"
    );
    assert_eq!(rows.len(), 5, "one anchor per boot");
}

#[test]
fn the_status_line_reports_every_stats_ms_in_the_log_merge() {
    let (recorder, clock, _streams, mut writer) = setup(1, 4096, SETTINGS);
    step(&mut writer, &clock, START_US);
    assert_eq!(step(&mut writer, &clock, START_US + 999_999).lines, 0);
    assert_eq!(
        step(&mut writer, &clock, START_US + 1_000_000).lines,
        0,
        "queued for the merge"
    );
    assert_eq!(
        step(&mut writer, &clock, START_US + 1_000_000 + MERGE_HORIZON_US).lines,
        1
    );
    settle(&mut writer, &clock);
    let log = file(&writer, None);
    assert_eq!(log.len(), 1);
    assert_eq!(
        log[0],
        "11000000 Recorder :: {'session': '000123', 'tlm': {'count': 0, 'max': 0, 'dropped': 0}, 'log': \
         {'count': 0, 'max': 0, 'dropped': 0}, 'ring': {'erased_ahead': 31, 'moved_not_erased': 0, 'foreign': 0, \
         'other_version': 0, 'unconfirmed': 1, 'spill_frames': 0, 'spill_bytes': 0, 'frames': 1, 'flash_errors': 0}}"
    );
    assert_eq!(recorder.stats().frames, 2, "the boot row's frame and the status line's");
}

#[test]
fn telemetry_writes_its_header_first_then_its_rows_in_order() {
    let (recorder, clock, streams, mut writer) = setup(1, 4096, SETTINGS);
    let _attached = recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    for index in 0..5 {
        imu.push(START_US + u64::from(index) * 1000, &imu_row(index)).unwrap();
    }
    step(&mut writer, &clock, START_US + 5000);
    settle(&mut writer, &clock);
    assert_eq!(
        file(&writer, Some(IMU)),
        [
            "uptime;ax;ay;az",
            "10000000;0;-1;9.81",
            "10001000;0.5;-1;9.81",
            "10002000;1;-1;9.81",
            "10003000;1.5;-1;9.81",
            "10004000;2;-1;9.81",
        ]
    );
}

#[test]
fn decimation_takes_the_streams_window_then_the_global_one() {
    let settings = Settings {
        telemetry_ms: 20,
        stats_ms: 1000,
    };
    let (recorder, clock, streams, mut writer) = setup(1, 1 << 16, settings);
    let _attached = recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    let mut health = Telemetry::new(recorder, streams.health).unwrap();
    let mut events = Telemetry::new(recorder, streams.events).unwrap();
    let mut due = Vec::new();
    for tick in 0..=20u64 {
        let now = START_US + tick * 5000; // every 5 ms for 100 ms
        due.push(imu.due(now));
        imu.push(now, &imu_row(tick as u32)).unwrap();
        health.push(now, &[Cell::F32(41.5), Cell::I32(7)]).unwrap();
        events.push(now, &[Cell::Text("tick")]).unwrap();
    }
    step(&mut writer, &clock, START_US + 100_000);
    settle(&mut writer, &clock);
    let uptimes = |routing: &str| -> Vec<u64> {
        file(&writer, Some(routing))[1..]
            .iter()
            .map(|row| uptime(row))
            .collect()
    };
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
    let (recorder, clock) = recorder(registry, 0, 4096, SETTINGS);
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    assert!(!imu.due(START_US), "no writer yet");
    imu.push(START_US, &imu_row(0)).unwrap(); // from an unattached thread: the misc ring
    let mut writer = Writer::new(recorder, ring(recorder, geometry(), OPTIONS)).unwrap();
    assert!(imu.due(START_US + 1));
    assert!(
        Writer::new(recorder, ring(recorder, geometry(), OPTIONS)).is_none(),
        "one writer"
    );
    step(&mut writer, &clock, START_US);
    settle(&mut writer, &clock);
    assert_eq!(file(&writer, Some(IMU)), ["uptime;ax;ay;az", "10000000;0;-1;9.81"]);
    drop(writer);
    assert!(!imu.due(START_US + 2), "the writer is gone");
    assert!(
        Writer::new(recorder, ring(recorder, geometry(), OPTIONS)).is_none(),
        "and is never replaced: what it counted as not yet on flash started with it"
    );
    // negative: a stream id this registry never handed out
    let (mut other, _) = self::registry();
    let foreign = other.add("extra.csv", &[], 0, false).unwrap();
    assert!(Telemetry::new(recorder, foreign).is_none());
}

#[test]
fn a_header_that_failed_goes_out_with_the_next_push() {
    let (recorder, clock, streams, mut writer) = setup(1, 64, SETTINGS);
    let _attached = recorder.attach(0).unwrap();
    for _ in 0..3 {
        assert!(recorder.log("x", "y"), "20 bytes each");
    }
    assert!(!recorder.log("x", "y"), "the ring is full");
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    assert_eq!(imu.push(START_US, &imu_row(0)), Err(Overflow), "the header did not fit");
    step(&mut writer, &clock, START_US);
    settle(&mut writer, &clock); // the logs go out, and their ring space comes back
    imu.push(clock.now(), &imu_row(1)).unwrap();
    settle(&mut writer, &clock);
    let rows = file(&writer, Some(IMU));
    assert_eq!(rows.len(), 2, "{rows:?}");
    assert_eq!(rows[0], "uptime;ax;ay;az");
    let stats = recorder.stats();
    assert_eq!((stats.telemetry.dropped, stats.log.dropped), (1, 1));
}

#[test]
fn log_lines_merge_across_rings_by_uptime() {
    let (recorder, clock, _streams, mut writer) = setup(2, 4096, SETTINGS);
    step(&mut writer, &clock, START_US);
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
                let _attached = recorder.attach(ring).unwrap();
                for &(_, offset, message) in lines.iter().filter(|line| line.0 == ring) {
                    clock.set(START_US + offset);
                    assert!(recorder.log(&format!("ring{ring}"), message));
                }
            });
        });
    }
    clock.set(START_US + 2500);
    assert!(recorder.log("main", "third"), "an unattached thread: the misc ring");
    let progress = step(&mut writer, &clock, START_US + 4000 + MERGE_HORIZON_US - 1);
    assert_eq!(progress.lines, 4, "the newest is inside the horizon");
    assert_eq!(step(&mut writer, &clock, START_US + 4000 + MERGE_HORIZON_US).lines, 1);
    settle(&mut writer, &clock);
    assert_eq!(
        file(&writer, None),
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
    let (recorder, clock, streams, mut writer) = setup(1, 4096, SETTINGS);
    let _attached = recorder.attach(0).unwrap();
    step(&mut writer, &clock, START_US);
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    clock.set(START_US + 100_000);
    recorder.log("x", "held");
    imu.push(START_US + 105_000, &imu_row(0)).unwrap();
    assert_eq!(
        step(&mut writer, &clock, START_US + 110_000).lines,
        2,
        "the header and the row"
    );
    assert_eq!(
        step(&mut writer, &clock, START_US + 120_000).lines,
        1,
        "the log line, 20 ms old"
    );
}

#[test]
fn a_frame_is_cut_at_its_age() {
    let (recorder, clock, streams, mut writer) = setup(1, 4096, SETTINGS);
    let _attached = recorder.attach(0).unwrap();
    step(&mut writer, &clock, START_US);
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    imu.push(START_US + 1000, &imu_row(0)).unwrap();
    assert_eq!(step(&mut writer, &clock, START_US + 1000).frames, 0);
    assert_eq!(step(&mut writer, &clock, START_US + 1000 + CUT_AGE_US - 1).frames, 0);
    assert_eq!(step(&mut writer, &clock, START_US + 1000 + CUT_AGE_US).frames, 1);
    let found = frames(writer.ring().flash().image(), writer.ring().geometry());
    let last = found.last().unwrap();
    assert_eq!(
        last.header.uptime_us,
        START_US + 1000,
        "the frame carries its first line's uptime"
    );
    assert!(!last.header.flags.durable);
}

#[test]
fn a_busy_step_cuts_16_kib_frames_and_a_durable_record_cuts_at_once() {
    let (recorder, clock, streams, mut writer) = setup(1, 1 << 20, SETTINGS);
    let _attached = recorder.attach(0).unwrap();
    step(&mut writer, &clock, START_US);
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    let mut events = Telemetry::new(recorder, streams.events).unwrap();
    for index in 0..1000 {
        imu.push(START_US + u64::from(index), &imu_row(index)).unwrap();
    }
    events.push(START_US + 1000, &[Cell::Text("apogee")]).unwrap();
    let progress = step(&mut writer, &clock, START_US + 1000);
    assert!(progress.frames >= 3, "~50 KB of rows: {progress:?}");
    let found = frames(writer.ring().flash().image(), writer.ring().geometry());
    let data: Vec<_> = found.iter().filter(|frame| frame.header.kind == Kind::Data).collect();
    assert!(data.iter().all(|frame| frame.payload.len() <= FRAME_BYTES));
    let last = data.last().unwrap();
    assert!(last.header.flags.durable, "the event cut its frame at once");
    let tail = String::from_utf8_lossy(&last.payload);
    assert!(
        tail.trim_end().ends_with('>') && tail.lines().last().unwrap().contains("apogee"),
        "{tail}"
    );
    assert_eq!(file(&writer, Some(IMU)).len(), 1001, "the header and every row, whole");
}

#[test]
fn frames_fill_each_block_before_the_next() {
    let geometry = Geometry::new(16, 1).unwrap(); // 4 KiB blocks
    let (registry, streams) = registry();
    let (recorder, clock) = recorder(registry, 1, 1 << 20, SETTINGS);
    let mut writer = Writer::new(recorder, ring(recorder, geometry, OPTIONS)).unwrap();
    let _attached = recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    for index in 0..400u32 {
        let now = START_US + u64::from(index) * 2000;
        imu.push(now, &imu_row(index)).unwrap();
        step(&mut writer, &clock, now);
    }
    settle(&mut writer, &clock);
    let image = writer.ring().flash().image();
    let block_bytes = geometry.block_bytes() as usize;
    let mut checked = 0;
    let found = frames(image, geometry);
    for pair in found.windows(2) {
        let (this, next) = (&pair[0], &pair[1]);
        if next.header.kind != Kind::Boot || this.header.kind != Kind::Data {
            continue;
        }
        // `this` closed its block: the next frame's first line must not have fitted behind it
        let after = found_end(image, block_bytes, this);
        let first_line = next_data_first_line(&found, next);
        assert!(
            block_bytes - after < 40 + first_line,
            "block left {} bytes, the next line needed {}",
            block_bytes - after,
            40 + first_line
        );
        checked += 1;
    }
    assert!(checked >= 5, "several blocks filled: {checked}");
    assert_eq!(file(&writer, Some(IMU)).len(), 401);
}

/// The 8-aligned end of a frame inside its block, found by locating its payload in the image.
fn found_end(image: &[u8], block_bytes: usize, frame: &Found) -> usize {
    let start = image
        .windows(frame.payload.len())
        .position(|window| window == frame.payload)
        .expect("the payload is in the image");
    let end = start + frame.payload.len();
    (end % block_bytes).next_multiple_of(8)
}

/// The wrapped length (with its line end) of the first line of the DATA frame after `boot`.
fn next_data_first_line(found: &[Found], boot: &Found) -> usize {
    let index = found.iter().position(|frame| std::ptr::eq(frame, boot)).unwrap();
    let data = &found[index + 1];
    data.payload.iter().position(|&byte| byte == b'\n').unwrap() + 1
}

#[test]
fn the_tees_copy_bare_lines_while_their_window_is_open() {
    let (recorder, clock, streams, mut writer) = setup(1, 4096, SETTINGS);
    let _attached = recorder.attach(0).unwrap();
    assert!(recorder.drain_log_tee(1000).lines.is_empty());
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    imu.push(START_US, &imu_row(0)).unwrap();
    recorder.log("sequencer", "stage -> boosting");
    step(&mut writer, &clock, START_US + MERGE_HORIZON_US);
    let logs = recorder.drain_log_tee(1000);
    assert_eq!(logs.lines, ["10000000 sequencer :: stage -> boosting"]);
    assert_eq!(logs.dropped, 0);
    assert!(
        recorder.drain_telemetry_tee(1000).lines.is_empty(),
        "its window was closed"
    );
    imu.push(START_US + 30_000, &imu_row(1)).unwrap();
    recorder.index_session(EPOCH, None, "dashboard", None).unwrap();
    step(&mut writer, &clock, START_US + 30_000);
    assert_eq!(
        recorder.drain_telemetry_tee(0).lines,
        [
            "@000123_imu_lsm6dso32.csv@10030000;0.5;-1;9.81",
            "@session.csv@10020000;123;000123;2026-10-03T14:21:07Z;;taster;dev;3f2a91;dashboard;;",
        ]
    );
    // negative: a window nobody renews lapses, and its stale lines go
    imu.push(START_US + 40_000, &imu_row(2)).unwrap();
    step(&mut writer, &clock, START_US + 40_000);
    assert!(
        recorder.drain_telemetry_tee(1000).lines.is_empty(),
        "closed by the last drain"
    );
    imu.push(START_US + 50_000, &imu_row(3)).unwrap();
    step(&mut writer, &clock, START_US + 50_000); // in the window opened at 40 ms
    imu.push(START_US + 2_000_000, &imu_row(4)).unwrap();
    step(&mut writer, &clock, START_US + 2_000_000); // past it: lapsed
    assert!(
        recorder.drain_telemetry_tee(1000).lines.is_empty(),
        "the lapsed batch is gone"
    );
}

#[test]
fn a_full_spill_holds_records_in_their_rings_until_gc_frees_blocks() {
    let geometry = Geometry::new(8, 1).unwrap();
    let options = Options {
        spill_capacity: 4096,
        reserve_blocks: 0,
    };
    let (registry, streams) = registry();
    let (recorder, clock) = recorder(registry, 1, 8192, SETTINGS);
    // an unformatted ring: every slot foreign, so nothing is writable until GC erases one
    let flash = MemoryFlash::from_image(vec![0; geometry.bytes() as usize]);
    let ring = Ring::mount(
        flash,
        geometry,
        options,
        BOOT_ID,
        &recorder.boot_payload("poweron", None),
    )
    .unwrap();
    let mut writer = Writer::new(recorder, ring).unwrap();
    let _attached = recorder.attach(0).unwrap();
    recorder.pause_erases(true); // and GC paused, with no reserve to keep
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    let (mut accepted, mut refused) = (Vec::new(), 0);
    let mut now = START_US;
    for index in 0..600u32 {
        now = START_US + u64::from(index) * 1000;
        match imu.push(now, &imu_row(index)) {
            Ok(()) => accepted.push(now),
            Err(Overflow) => refused += 1,
        }
        step(&mut writer, &clock, now);
    }
    let stats = recorder.stats();
    assert!(refused > 0, "the rings filled behind the full spill");
    assert_eq!(stats.telemetry.dropped, refused);
    assert!(stats.ring.spill_frames > 0 && stats.ring.foreign == 8);
    assert!(stats.telemetry.count > 0, "records wait in their ring");
    recorder.pause_erases(false);
    for index in 1..=200 {
        step(&mut writer, &clock, now + index * 10_000);
    }
    let stats = recorder.stats();
    assert_eq!((stats.telemetry.count, stats.ring.spill_frames), (0, 0), "{stats:?}");
    let rows = file(&writer, Some(IMU));
    assert_eq!(rows[0], "uptime;ax;ay;az");
    let uptimes: Vec<u64> = rows[1..].iter().map(|row| uptime(row)).collect();
    assert_eq!(uptimes, accepted, "every accepted row, once, in order");
    let found = frames(writer.ring().flash().image(), geometry);
    assert!(found.iter().any(|frame| frame.header.flags.spilled));
}

#[test]
fn a_failing_flash_keeps_records_queued_and_counts_its_errors() {
    let (registry, streams) = registry();
    let (recorder, clock) = recorder(registry, 1, 4096, SETTINGS);
    let mut flash = MemoryFlash::new(geometry().bytes());
    flash.cut_power(2, 0); // the BOOT frame and the boot row's frame, then nothing
    let ring = Ring::mount(
        flash,
        geometry(),
        OPTIONS,
        BOOT_ID,
        &recorder.boot_payload("poweron", None),
    )
    .unwrap();
    let mut writer = Writer::new(recorder, ring).unwrap();
    let _attached = recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    step(&mut writer, &clock, START_US);
    let mut accepted = 0;
    let mut refused = 0;
    for index in 0..200u32 {
        let now = START_US + 1000 + u64::from(index) * 10_000;
        match imu.push(now, &imu_row(index)) {
            Ok(()) => accepted += 1,
            Err(Overflow) => refused += 1,
        }
        step(&mut writer, &clock, now);
    }
    let stats = recorder.stats();
    // a record leaves its ring only once its frame is programmed: the header (16 bytes) and every accepted row
    // (32 bytes each) are still in the 4 KiB ring together, the refused frame's too
    assert!(
        16 + accepted as usize * 32 < 4096,
        "{accepted} rows: some ring space came back unprogrammed"
    );
    assert_eq!(stats.frames, 1, "only the boot row's frame went out");
    assert!(stats.flash_errors > 100, "each retry counts: {}", stats.flash_errors);
    assert!(refused > 0, "the ring filled");
    assert_eq!(
        stats.telemetry.count,
        accepted + 1,
        "every accepted row and the header still queued"
    );
    assert_eq!(stats.telemetry.dropped, refused);
}

/// The board's block size, 64 KiB: a block holds more than a 16 KiB frame, as on the board.
fn board_geometry() -> Geometry {
    Geometry::new(8, 16).unwrap()
}

#[test]
fn a_row_no_frame_could_hold_raises_at_push() {
    let mut registry = Registry::new(SESSION).unwrap();
    let wide = registry.add("wide.csv", &["text"; 64], 0, false).unwrap();
    let (recorder, clock) = recorder(registry, 1, 1 << 17, SETTINGS);
    let mut writer = Writer::new(recorder, ring(recorder, board_geometry(), Options::BOARD)).unwrap();
    let _attached = recorder.attach(0).unwrap();
    let mut stream = Telemetry::new(recorder, wide).unwrap();
    let text = "x".repeat(255);
    // at worst `000123_wide.csv` (15) + 2, a 20-digit uptime and `;`, 63 x 255 bytes and 62 `;`, the wrapper
    // (22) and the line end: 16 188 bytes, in a 16 KiB frame
    stream.push(START_US, &[Cell::Text(&text); 63]).unwrap();
    // negative: one cell more is 16 444, which no frame holds: the producer learns of it now
    assert_eq!(stream.push(START_US + 1, &[Cell::Text(&text); 64]), Err(Overflow));
    step(&mut writer, &clock, START_US + 1);
    settle(&mut writer, &clock);
    let rows = file(&writer, Some("000123_wide.csv"));
    assert_eq!(rows.len(), 2, "the header and the 63-cell row, whole");
    assert_eq!(rows[1].len(), "10000000;".len() + 63 * 255 + 62);
    let stats = recorder.stats();
    assert_eq!(
        (stats.telemetry.dropped, stats.telemetry.count),
        (1, 0),
        "the refused push, and nothing the writer had to drop"
    );
}

#[test]
fn a_line_no_block_of_a_small_ring_holds_is_dropped_and_counted() {
    // the producer checks a row against a 16 KiB frame; a test ring whose block is smaller than that relies on
    // the writer's own guard
    let geometry = Geometry::new(4, 1).unwrap(); // a frame payload below 4 KiB
    let mut registry = Registry::new(SESSION).unwrap();
    let wide = registry.add("wide.csv", &["text"; 30], 0, false).unwrap();
    let (recorder, clock) = recorder(registry, 1, 1 << 16, SETTINGS);
    let mut writer = Writer::new(recorder, ring(recorder, geometry, OPTIONS)).unwrap();
    let _attached = recorder.attach(0).unwrap();
    let text = "x".repeat(255);
    let mut stream = Telemetry::new(recorder, wide).unwrap();
    stream.push(START_US, &[Cell::Text(&text); 30]).unwrap(); // a ~7.7 KB line
    step(&mut writer, &clock, START_US);
    settle(&mut writer, &clock);
    assert_eq!(file(&writer, Some("000123_wide.csv")).len(), 1, "the header only");
    let stats = recorder.stats();
    assert_eq!((stats.telemetry.dropped, stats.telemetry.count), (1, 0));
    // negative: a row larger than a ring record never queues
    let huge = [Cell::Text(&text); 255];
    assert_eq!(stream.push(START_US + 1, &huge), Err(Overflow));
    assert_eq!(recorder.stats().telemetry.dropped, 2);
}

#[test]
fn frames_fill_each_block_while_the_spill_drains() {
    let geometry = Geometry::new(16, 1).unwrap(); // 4 KiB blocks
    let options = Options {
        spill_capacity: 64 << 10,
        reserve_blocks: 0,
    };
    let (registry, streams) = registry();
    let (recorder, clock) = recorder(registry, 1, 1 << 20, SETTINGS);
    // an unformatted ring with GC paused: every frame spills, the boot row's first, until erases resume
    let flash = MemoryFlash::from_image(vec![0; geometry.bytes() as usize]);
    let ring = Ring::mount(
        flash,
        geometry,
        options,
        BOOT_ID,
        &recorder.boot_payload("poweron", None),
    )
    .unwrap();
    let mut writer = Writer::new(recorder, ring).unwrap();
    let _attached = recorder.attach(0).unwrap();
    recorder.pause_erases(true);
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    let mut most_spilled = 0;
    for index in 0..500u32 {
        let now = START_US + u64::from(index) * 2000;
        imu.push(now, &imu_row(index)).unwrap();
        if index == 150 {
            recorder.pause_erases(false);
        }
        step(&mut writer, &clock, now);
        most_spilled = most_spilled.max(writer.ring().figures().spill_frames);
    }
    settle(&mut writer, &clock);
    assert!(most_spilled >= 5, "frames waited in the spill: {most_spilled}");
    assert_eq!(writer.ring().figures().spill_frames, 0, "and all went out");
    let image = writer.ring().flash().image();
    let block_bytes = geometry.block_bytes() as usize;
    let found = frames(image, geometry);
    let mut checked = 0;
    for pair in found.windows(2) {
        let (this, next) = (&pair[0], &pair[1]);
        if next.header.kind != Kind::Boot || this.header.kind != Kind::Data {
            continue;
        }
        let after = found_end(image, block_bytes, this);
        let first_line = next_data_first_line(&found, next);
        assert!(
            block_bytes - after < 40 + first_line,
            "block {} left {} bytes, the next line needed {}",
            this.header.block_sequence,
            block_bytes - after,
            40 + first_line
        );
        checked += 1;
    }
    assert!(checked >= 5, "several blocks filled: {checked}");
    assert!(found.iter().any(|frame| frame.header.flags.spilled));
    let rows = file(&writer, Some(IMU));
    assert_eq!(rows.len(), 501, "the header and every row");
    assert!(rows[1..].iter().map(|row| uptime(row)).is_sorted());
}

/// A flash whose one chosen program call fails after programming only a prefix, as a flash error that is not a
/// power cut may: every other call works.
struct Flaky {
    /// The flash behind it.
    memory: MemoryFlash,
    /// Program calls so far.
    programs: u32,
    /// The program call to fail, counted from 0, and how many of its leading bytes take effect.
    failing: Option<(u32, usize)>,
    /// Set once the chosen call failed.
    failed: Rc<SharedCell<bool>>,
}

impl Flash for Flaky {
    type Error = MemoryError;

    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), MemoryError> {
        self.memory.read(offset, buffer)
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), MemoryError> {
        let call = self.programs;
        self.programs += 1;
        match self.failing {
            Some((failing, prefix)) if failing == call => {
                self.memory.program(offset, &data[..prefix.min(data.len())])?;
                self.failed.set(true);
                Err(MemoryError::OutOfRange)
            }
            _ => self.memory.program(offset, data),
        }
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), MemoryError> {
        self.memory.erase_sector(offset)
    }

    fn size(&self) -> u32 {
        self.memory.size()
    }
}

#[test]
fn a_failed_program_is_repeated_whole_as_the_boots_next_data_frame() {
    let (registry, streams) = registry();
    let (recorder, clock) = recorder(registry, 1, 1 << 16, SETTINGS);
    let failed = Rc::new(SharedCell::new(false));
    let flash = Flaky {
        memory: MemoryFlash::new(geometry().bytes()),
        programs: 0,
        // BOOT, the boot row's frame, then DATA frames: the fourth program tears 400 bytes into its frame
        failing: Some((3, HEADER_BYTES + 400)),
        failed: failed.clone(),
    };
    let ring = Ring::mount(
        flash,
        geometry(),
        OPTIONS,
        BOOT_ID,
        &recorder.boot_payload("poweron", None),
    )
    .unwrap();
    let mut writer = Writer::new(recorder, ring).unwrap();
    let _attached = recorder.attach(0).unwrap();
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    let mut accepted = Vec::new();
    for index in 0..60u32 {
        let now = START_US + u64::from(index) * 10_000;
        imu.push(now, &imu_row(index)).unwrap();
        accepted.push(now);
        step(&mut writer, &clock, now);
    }
    settle(&mut writer, &clock);
    assert!(failed.get(), "the chosen program failed");
    assert_eq!(recorder.stats().flash_errors, 1);

    // the torn frame: the first frame of its block that does not decode, its header whole
    let image = writer.ring().flash().memory.image().to_vec();
    let geometry = geometry();
    let block_bytes = geometry.block_bytes() as usize;
    let mut torn = None;
    for block in image.chunks(block_bytes) {
        let end = frame::frames(block)
            .last()
            .map_or(0, |found| found.offset as usize + frame_bytes(found.payload.len()));
        if let Some(rest) = block.get(end..)
            && let Ok(header) = Header::decode(rest)
            && header.kind == Kind::Data
        {
            torn = Some((header, rest[HEADER_BYTES..HEADER_BYTES + 400].to_vec()));
        }
    }
    let (torn, surviving) = torn.expect("a torn DATA frame with a whole header");
    let found = frames(&image, geometry);
    let repeat = found
        .iter()
        .find(|frame| frame.header.kind == Kind::Data && frame.header.frame_sequence > torn.frame_sequence)
        .expect("the boot's next DATA frame");
    assert_eq!(
        (repeat.header.uptime_us, repeat.header.payload_length),
        (torn.uptime_us, torn.payload_length),
        "the repeat names the torn frame"
    );
    assert!(
        repeat.payload.starts_with(&surviving),
        "and holds its surviving bytes first"
    );
    // why the mover must not salvage it as well: the torn frame's whole lines would land twice
    let salvaged = surviving.iter().filter(|&&byte| byte == b'\n').count();
    assert!(salvaged > 0, "the torn frame held whole lines");
    // filed by the rule, every row is there once, in order
    let rows = verified(&demux(&capture(&found)))
        .remove(&Some(IMU.to_string()))
        .unwrap();
    assert_eq!(rows[0], "uptime;ax;ay;az");
    assert_eq!(rows[1..].iter().map(|row| uptime(row)).collect::<Vec<_>>(), accepted);
}

#[test]
fn a_new_stats_ms_takes_effect_at_the_next_step() {
    let (recorder, clock, _streams, mut writer) = setup(1, 4096, SETTINGS);
    step(&mut writer, &clock, START_US);
    assert!(recorder.set_stats_ms(100), "1000 ms -> 100 ms");
    step(&mut writer, &clock, START_US + 100_000);
    step(&mut writer, &clock, START_US + 200_000);
    // negative: 0 is refused, and the period stays
    assert!(!recorder.set_stats_ms(0));
    step(&mut writer, &clock, START_US + 300_000);
    settle(&mut writer, &clock);
    let status: Vec<u64> = file(&writer, None)
        .iter()
        .filter(|line| line.contains(" Recorder :: {'session'"))
        .map(|line| uptime(line))
        .collect();
    assert_eq!(status, [START_US + 100_000, START_US + 200_000, START_US + 300_000]);
    assert_eq!(recorder.inspect().stats_ms, 100);
}

#[test]
fn stats_count_what_is_not_yet_on_flash() {
    let (recorder, clock, streams, mut writer) = setup(1, 4096, SETTINGS);
    let _attached = recorder.attach(0).unwrap();
    step(&mut writer, &clock, START_US);
    let mut imu = Telemetry::new(recorder, streams.imu).unwrap();
    for index in 0..10 {
        imu.push(START_US + 1000 + u64::from(index), &imu_row(index)).unwrap();
    }
    recorder.log("x", "y");
    step(&mut writer, &clock, START_US + 1010);
    let stats = recorder.stats();
    assert_eq!(
        (stats.telemetry.count, stats.log.count),
        (11, 1),
        "in the open frame, and held"
    );
    settle(&mut writer, &clock);
    let stats = recorder.stats();
    assert_eq!((stats.telemetry.count, stats.telemetry.max), (0, 11));
    assert_eq!((stats.log.count, stats.log.max), (0, 1));
    assert_eq!(
        recorder.programmed(),
        writer.ring().programmed(),
        "published for the mover"
    );
    assert!(recorder.programmed().is_some());
}
