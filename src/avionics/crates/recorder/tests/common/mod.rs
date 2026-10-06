//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Helpers shared by the recorder's integration tests: a Recorder with leaked buffers and a clock the test sets,
//! a flash ring over an in-memory flash, and the read-back a ring dump gets on the host (`ring_split.py`'s job):
//! the boot's DATA payloads in block order, then recorderd's demux, with every line verified.

// Each test binary uses its own subset of these helpers.
#![allow(dead_code)]

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use flash_ring::frame::{self, Header, Kind};
use flash_ring::{Geometry, MemoryFlash, Options, Ring};
use recorder::{Buffers, Identity, Recorder, Registry, Settings};

/// The boot id the tests record under.
pub const BOOT_ID: u32 = 123;

/// The session prefix of [`BOOT_ID`].
pub const SESSION: &str = "000123";

/// 2026-10-03T14:21:07Z: a set clock.
pub const EPOCH: i64 = 1_791_037_267;

/// The uptime the tests start at: a board some seconds after boot.
pub const START_US: u64 = 10_000_000;

/// A clock the test moves by hand; the Recorder reads it for its log lines.
#[derive(Clone, Default)]
pub struct TestClock(Arc<Mutex<u64>>);

impl TestClock {
    /// Set the uptime.
    pub fn set(&self, uptime_us: u64) {
        *self.0.lock().unwrap() = uptime_us;
    }

    /// The uptime.
    pub fn now(&self) -> u64 {
        *self.0.lock().unwrap()
    }
}

/// A leaked, zeroed buffer of `bytes`: the PSRAM the firmware hands over for good.
pub fn buffer(bytes: usize) -> &'static mut [u8] {
    Vec::leak(vec![0; bytes])
}

/// The boot's identity.
pub fn identity() -> Identity {
    Identity {
        boot_id: Some(BOOT_ID),
        board: "taster".into(),
        firmware: "dev".into(),
        config_id: "3f2a91".into(),
    }
}

/// A Recorder of `rings` producer rings of `ring_bytes` each over `registry`, leaked to `'static` as on the
/// board, with its clock at [`START_US`].
pub fn recorder(
    registry: Registry,
    rings: usize,
    ring_bytes: usize,
    settings: Settings,
) -> (&'static Recorder, TestClock) {
    let clock = TestClock::default();
    clock.set(START_US);
    let reading = clock.clone();
    let buffers = Buffers {
        producers: (0..rings).map(|_| buffer(ring_bytes)).collect(),
        misc: buffer(ring_bytes),
        log_tee: buffer(64 << 10),
        telemetry_tee: buffer(64 << 10),
    };
    let recorder = Recorder::new(registry, identity(), settings, buffers, Box::new(move || reading.now()));
    (Box::leak(Box::new(recorder)), clock)
}

/// The tests' recorder settings: no global decimation, a status line every second.
pub const SETTINGS: Settings = Settings {
    telemetry_ms: 0,
    stats_ms: 1000,
};

/// The tests' flash ring options: a 64 KiB spill, one block kept erased ahead in flight.
pub const OPTIONS: Options = Options {
    spill_capacity: 64 << 10,
    reserve_blocks: 1,
};

/// A flash ring of `geometry` over an erased in-memory flash, with this boot's BOOT payload.
pub fn ring(recorder: &Recorder, geometry: Geometry, options: Options) -> Ring<MemoryFlash> {
    let flash = MemoryFlash::new(geometry.bytes());
    Ring::mount(
        flash,
        geometry,
        options,
        BOOT_ID,
        &recorder.boot_payload("poweron", None),
    )
    .unwrap()
}

/// One frame read back from a flash image.
#[derive(Clone, Debug)]
pub struct Found {
    /// Its header.
    pub header: Header,
    /// Its payload.
    pub payload: Vec<u8>,
}

/// Every whole frame of a flash image, block by block in block-sequence order, each block's frames in address
/// order (a block ends at its first frame that does not decode).
pub fn frames(image: &[u8], geometry: Geometry) -> Vec<Found> {
    let block_bytes = geometry.block_bytes() as usize;
    let mut blocks: Vec<(u32, &[u8])> = image
        .chunks(block_bytes)
        .filter_map(|block| {
            let (header, _) = frame::decode(block).ok()?;
            (header.kind == Kind::Boot).then_some((header.block_sequence, block))
        })
        .collect();
    blocks.sort_by_key(|(sequence, _)| *sequence);
    let mut found = Vec::new();
    for (sequence, block) in blocks {
        for frame in frame::frames(block).take_while(|frame| frame.header.block_sequence == sequence) {
            found.push(Found {
                header: frame.header,
                payload: frame.payload.to_vec(),
            });
        }
    }
    found
}

/// The boot's capture: its DATA payloads concatenated, which is the old UART stream's format.
pub fn capture(found: &[Found]) -> Vec<u8> {
    found
        .iter()
        .filter(|frame| frame.header.kind == Kind::Data)
        .flat_map(|frame| frame.payload.iter().copied())
        .collect()
}

/// recorderd's demux of a capture: per file (`None` for recorder.log), the wire lines as the file holds them,
/// in order. Panics on an unterminated tail, which a whole frame never leaves.
pub fn demux(capture: &[u8]) -> BTreeMap<Option<String>, Vec<Vec<u8>>> {
    let mut files: BTreeMap<Option<String>, Vec<Vec<u8>>> = BTreeMap::new();
    let tail = recorder_wire::split(capture, |routing, payload| {
        let routing = routing.map(|routing| String::from_utf8(routing.to_vec()).expect("a routing is text"));
        files.entry(routing).or_default().push(payload.to_vec());
    });
    assert!(tail.is_empty(), "an unterminated line: {tail:?}");
    files
}

/// Each file's lines verified under the file's own routing, as their payloads; panics on any line that fails.
pub fn verified(files: &BTreeMap<Option<String>, Vec<Vec<u8>>>) -> BTreeMap<Option<String>, Vec<String>> {
    files
        .iter()
        .map(|(routing, lines)| {
            let payloads = lines
                .iter()
                .map(|line| {
                    let body = line.strip_suffix(b"\n").expect("a demuxed line keeps its line end");
                    let payload = recorder_wire::verify(routing.as_deref().map(str::as_bytes), body)
                        .unwrap_or_else(|| panic!("{routing:?}: line fails its checks: {line:?}"));
                    String::from_utf8(payload.to_vec()).expect("a payload is text")
                })
                .collect();
            (routing.clone(), payloads)
        })
        .collect()
}

/// The verified payloads of a writer's flash: frames read back, demuxed, verified.
pub fn read_back(ring: &Ring<MemoryFlash>) -> BTreeMap<Option<String>, Vec<String>> {
    let found = frames(ring.flash().image(), ring.geometry());
    verified(&demux(&capture(&found)))
}

/// The leading uptime of a row or log payload.
pub fn uptime(payload: &str) -> u64 {
    let digits: String = payload.chars().take_while(char::is_ascii_digit).collect();
    digits.parse().unwrap_or_else(|_| panic!("no uptime: {payload:?}"))
}
