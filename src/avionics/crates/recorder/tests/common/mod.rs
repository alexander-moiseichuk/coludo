//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Helpers shared by the recorder's integration tests: a Recorder with leaked buffers, a clock the test moves and
//! a count of its writer wakes; the flash ring over an in-memory flash, brought up as the firmware does (the scan,
//! then the writer and the eraser over their own handles); and the read-back a ring image gets on the host
//! (`ring_split.py`'s job): every block of this layout in sequence order through the [`Decoder`], each line
//! verified under its own routing.

// Each test binary uses its own subset of these helpers.
#![allow(dead_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use flash_ring::header::{BlockHeader, Session, VERSION};
use flash_ring::{BLOCK_BYTES, Eraser, Geometry, Hints, MemoryFlash, Ring, Scan, scan};
use recorder::{Buffers, Decoder, Identity, Recorder, Registry, Settings};

/// The boot id the tests record under.
pub const BOOT_ID: u32 = 123;

/// The session prefix of [`BOOT_ID`].
pub const SESSION: &str = "000123";

/// 2026-10-03T14:21:07Z: a set clock.
pub const EPOCH: i64 = 1_791_037_267;

/// The uptime the tests start at: a board some seconds after boot.
pub const START_US: u64 = 10_000_000;

/// The tests' recorder settings: no global decimation, a status line every second.
pub const SETTINGS: Settings = Settings {
    telemetry_ms: 0,
    stats_ms: 1000,
};

/// A clock the test moves by hand; the Recorder reads it for its log lines and time-set rows.
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

/// How often producers woke the writer, as the firmware's notify would count it.
#[derive(Clone, Default)]
pub struct Wakes(Arc<AtomicU32>);

impl Wakes {
    /// Whether a wake came since the last call, which it clears: the writer thread's `ulTaskNotifyTake`.
    pub fn take(&self) -> bool {
        self.0.swap(0, Ordering::Relaxed) > 0
    }
}

/// A leaked, zeroed buffer of `bytes`: the PSRAM the firmware hands over for good.
pub fn buffer(bytes: usize) -> &'static mut [u8] {
    Vec::leak(vec![0; bytes])
}

/// Boot `boot`'s session label, `%06u` as the board prints it.
pub fn session(boot: u32) -> Session {
    Session::new(&format!("{boot:06}")).unwrap()
}

/// Boot `boot`'s identity.
pub fn identity(boot: u32) -> Identity {
    Identity {
        boot_id: Some(boot),
        session: session(boot),
        board: "taster".into(),
        firmware: "dev".into(),
        config_id: "3f2a91".into(),
    }
}

/// The parts of a test Recorder the test keeps.
pub struct Parts {
    /// The Recorder, leaked to `'static` as on the board.
    pub recorder: &'static Recorder,
    /// Its clock, at [`START_US`].
    pub clock: TestClock,
    /// Its writer wakes.
    pub wakes: Wakes,
}

/// Boot `boot`'s Recorder of `rings` producer rings of `ring_bytes` each over `registry`, with 64 KiB tees,
/// starting in flight when `flight` says so (the crumb of a reboot in flight).
pub fn recorder_of(
    boot: u32,
    registry: Registry,
    rings: usize,
    ring_bytes: usize,
    settings: Settings,
    flight: bool,
) -> Parts {
    let clock = TestClock::default();
    clock.set(START_US);
    let reading = clock.clone();
    let wakes = Wakes::default();
    let counting = wakes.0.clone();
    let buffers = Buffers {
        producers: (0..rings).map(|_| buffer(ring_bytes)).collect(),
        misc: buffer(ring_bytes),
        log_tee: buffer(64 << 10),
        telemetry_tee: buffer(64 << 10),
    };
    let recorder = Recorder::new(
        registry,
        identity(boot),
        settings,
        flight,
        buffers,
        Box::new(move || reading.now()),
        Box::new(move || {
            counting.fetch_add(1, Ordering::Relaxed);
        }),
    );
    Parts {
        recorder: Box::leak(Box::new(recorder)),
        clock,
        wakes,
    }
}

/// Boot [`BOOT_ID`]'s Recorder, on the pad: see [`recorder_of`].
pub fn recorder(registry: Registry, rings: usize, ring_bytes: usize, settings: Settings) -> Parts {
    recorder_of(BOOT_ID, registry, rings, ring_bytes, settings, false)
}

/// The bring-up scan of `flash`, with no hints (a first boot).
pub fn bring_up(flash: &MemoryFlash, geometry: Geometry) -> Scan {
    scan(flash, geometry, Hints::default()).unwrap()
}

/// Boot `boot`'s flash ring writer over `flash`, after `scanned`, on the pad.
pub fn ring<'a>(flash: &'a MemoryFlash, scanned: &'a Scan, boot: u32) -> Ring<'a, &'a MemoryFlash> {
    Ring::new(flash, &scanned.table, scanned.next_sequence, boot, session(boot), false).unwrap()
}

/// The eraser over `flash`, with no trusted saver hint (M3: no saver, so newest wins alone frees blocks).
pub fn eraser<'a>(flash: &'a MemoryFlash, scanned: &'a Scan) -> Eraser<'a, &'a MemoryFlash> {
    Eraser::new(flash, &scanned.table, None).unwrap()
}

/// A block of this layout in an image: where it is and whose it is.
#[derive(Clone, Copy, Debug)]
pub struct Block {
    /// Its slot.
    pub slot: u32,
    /// Its header.
    pub header: BlockHeader,
}

/// The blocks of this layout in `image`, oldest first.
pub fn blocks(image: &[u8], geometry: Geometry) -> Vec<Block> {
    let mut blocks: Vec<Block> = (0..geometry.blocks())
        .filter_map(|slot| {
            let header = BlockHeader::decode(slot_bytes(image, slot)).ok()?;
            (header.version == VERSION).then_some(Block { slot, header })
        })
        .collect();
    blocks.sort_by_key(|block| block.header.block_sequence);
    blocks
}

/// One slot's bytes in `image`.
pub fn slot_bytes(image: &[u8], slot: u32) -> &[u8] {
    let start = (slot * BLOCK_BYTES) as usize;
    &image[start..start + BLOCK_BYTES as usize]
}

/// What a ring image reads back as.
#[derive(Debug, Default)]
pub struct ReadBack {
    /// Every wire line, in block order: a capture, as `GET /ring` streams it.
    pub capture: Vec<u8>,
    /// Each file's verified payloads (None: recorder.log), in order.
    pub files: BTreeMap<Option<String>, Vec<String>>,
    /// The sequences of the blocks decoded, oldest first.
    pub blocks: Vec<u32>,
    /// The torn chunks the decoder called losses.
    pub lost: u32,
}

impl ReadBack {
    /// A file's payloads (empty when the file has none).
    pub fn file(&self, routing: Option<&str>) -> Vec<String> {
        self.files.get(&routing.map(String::from)).cloned().unwrap_or_default()
    }
}

/// `image` read as the ring_saver reads it: every block of this layout (of `session` only, when given) in
/// sequence order through one [`Decoder`], each told whether a later block of its boot is in flash. Every line
/// must verify under its own routing, or this panics.
pub fn read_back(image: &[u8], geometry: Geometry, session: Option<&str>) -> ReadBack {
    let all = blocks(image, geometry);
    let mut decoder = Decoder::new();
    let mut read_back = ReadBack::default();
    for (index, block) in all.iter().enumerate() {
        if session.is_some_and(|session| block.header.session.as_str() != session) {
            continue;
        }
        let same_boot = |later: &Block| {
            later.header.boot_id == block.header.boot_id && later.header.session == block.header.session
        };
        let boot_goes_on = all[index + 1..].iter().any(same_boot);
        let summary = decoder
            .block(slot_bytes(image, block.slot), boot_goes_on, |line| {
                read_back.capture.extend_from_slice(line.wire);
                let body = line.body().strip_suffix(b"\n").expect("a line ends with its line end");
                let payload = recorder_wire::verify(line.routing.map(str::as_bytes), body)
                    .unwrap_or_else(|| panic!("a line fails its checks: {}", String::from_utf8_lossy(line.wire)));
                read_back
                    .files
                    .entry(line.routing.map(String::from))
                    .or_default()
                    .push(String::from_utf8(payload.to_vec()).expect("a payload is text"));
            })
            .expect("a block of this layout decodes");
        assert_eq!(
            summary.damaged, 0,
            "block {}: no writer makes a damaged chunk",
            block.header.block_sequence
        );
        read_back.lost += summary.lost;
        read_back.blocks.push(block.header.block_sequence);
    }
    read_back
}

/// recorderd's demux of a capture: per file (None: recorder.log), the lines as the file holds them, in order.
/// Panics on an unterminated tail, which the decoder never leaves.
pub fn demux(capture: &[u8]) -> BTreeMap<Option<String>, Vec<Vec<u8>>> {
    let mut files: BTreeMap<Option<String>, Vec<Vec<u8>>> = BTreeMap::new();
    let tail = recorder_wire::split(capture, |routing, payload| {
        let routing = routing.map(|routing| String::from_utf8(routing.to_vec()).expect("a routing is text"));
        files.entry(routing).or_default().push(payload.to_vec());
    });
    assert!(tail.is_empty(), "an unterminated line: {tail:?}");
    files
}

/// The verified payloads that recorderd's demux of `capture` files under `routing` (None: recorder.log), in order.
/// Panics on a line that does not verify, which the decoder never makes.
pub fn payloads(capture: &[u8], routing: Option<&str>) -> Vec<String> {
    let files = demux(capture);
    let Some(lines) = files.get(&routing.map(String::from)) else {
        return Vec::new();
    };
    lines
        .iter()
        .map(|body| {
            let body = body.strip_suffix(b"\n").expect("a line ends with its line end");
            let payload = recorder_wire::verify(routing.map(str::as_bytes), body)
                .unwrap_or_else(|| panic!("a line fails its checks: {}", String::from_utf8_lossy(body)));
            String::from_utf8(payload.to_vec()).expect("a payload is text")
        })
        .collect()
}

/// The tools' duplicate filter (doc/specs/avionics-port.md "Duplicates"), which tools/recorder_wire.py is to run
/// the same way: the first line of each (stream, uptime, line CRC) key stays and every later one goes. The stream
/// is the line's routing (None: recorder.log), the uptime its payload's leading number, and the line CRC its OPEN
/// tag, the CRC-32 over routing and payload. Two different lines with one uptime differ in their CRC, so both stay.
///
/// Returns the capture without the duplicates, and the lines dropped, whole and in order. Panics on a line that
/// is not wrapped, which the decoder never makes.
pub fn drop_duplicates(capture: &[u8]) -> (Vec<u8>, Vec<Vec<u8>>) {
    let mut seen: BTreeSet<(Option<Vec<u8>>, u64, u32)> = BTreeSet::new();
    let (mut kept, mut dropped) = (Vec::new(), Vec::new());
    let tail = recorder_wire::split(capture, |routing, body| {
        let mut line = Vec::new();
        if let Some(routing) = routing {
            line.extend_from_slice(b"@");
            line.extend_from_slice(routing);
            line.extend_from_slice(b"@");
        }
        line.extend_from_slice(body);
        let wrapper = recorder_wire::unwrap(body.strip_suffix(b"\n").unwrap_or(body))
            .unwrap_or_else(|| panic!("an unwrapped line: {}", String::from_utf8_lossy(&line)));
        let digits = wrapper.payload.iter().take_while(|byte| byte.is_ascii_digit()).count();
        let uptime_us = std::str::from_utf8(&wrapper.payload[..digits])
            .unwrap()
            .parse()
            .unwrap_or(0);
        if seen.insert((routing.map(<[u8]>::to_vec), uptime_us, wrapper.open)) {
            kept.extend_from_slice(&line);
        } else {
            dropped.push(line);
        }
    });
    assert!(tail.is_empty(), "an unterminated line: {tail:?}");
    (kept, dropped)
}

/// A record as the test's oracle reads it out of a chunk, by the spec's table rather than the crate's codec: for
/// what the decoder does not print, such as a repeated HEADER record.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawRecord {
    /// The block it is in.
    pub block_sequence: u32,
    /// Its place in the ring's history: the block's index among the blocks read, times 4 KiB, plus its chunk's
    /// offset. Differences between two of them approximate the flash written between the two.
    pub flash_at: u64,
    /// Its kind byte: 1 HEADER, 2 ROW, 3 LOG.
    pub kind: u8,
    /// Its flags byte.
    pub flags: u8,
    /// Its destination file name.
    pub name: String,
    /// Its uptime.
    pub uptime_us: u64,
}

/// Every record of every whole chunk of the blocks of this layout in `image`, oldest block first.
pub fn raw_records(image: &[u8], geometry: Geometry) -> Vec<RawRecord> {
    let mut found = Vec::new();
    for (index, block) in blocks(image, geometry).into_iter().enumerate() {
        for walked in flash_ring::chunk::chunks(slot_bytes(image, block.slot)) {
            let flash_ring::chunk::Walked::Chunk(chunk) = walked else {
                continue;
            };
            let mut rest = chunk.payload;
            while rest.len() >= 16 {
                let length = usize::from(u16::from_le_bytes([rest[0], rest[1]]));
                let name_length = usize::from(rest[4]);
                assert!(length >= 16 + name_length && length <= rest.len(), "a whole record");
                found.push(RawRecord {
                    block_sequence: block.header.block_sequence,
                    flash_at: index as u64 * u64::from(BLOCK_BYTES) + u64::from(chunk.offset),
                    kind: rest[2],
                    flags: rest[3],
                    name: String::from_utf8(rest[16..16 + name_length].to_vec()).unwrap(),
                    uptime_us: u64::from_le_bytes(rest[8..16].try_into().unwrap()),
                });
                rest = &rest[length..];
            }
            assert!(rest.is_empty(), "a payload is whole records");
        }
    }
    found
}

/// The leading uptime of a row or log payload.
pub fn uptime(payload: &str) -> u64 {
    let digits: String = payload.chars().take_while(char::is_ascii_digit).collect();
    digits.parse().unwrap_or_else(|_| panic!("no uptime: {payload:?}"))
}

/// A row's payload, as the decoder prints it.
pub fn row_text(uptime_us: u64, cells: &[recorder::Cell]) -> String {
    let mut line = Vec::new();
    recorder_wire::row(&mut line, uptime_us, cells);
    String::from_utf8(line).unwrap()
}
