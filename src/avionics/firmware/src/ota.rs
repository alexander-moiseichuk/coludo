//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Firmware OTA (M15, doc/specs/avionics-port.md): a new image into the app slot that is not running, on the pad. The
//! data comes by either of two transports, the decision over CC:
//! - **The console** (the operator, 2026-10-11: more reliable than Wi-Fi for big transfers): `ota begin size=<n>
//!   sha256=<hex>`, then `ota write offset=<n> crc=<crc32 hex> data=<base64url, no padding>` lines in order, then
//!   `ota end` (`ota abort` drops it). tools/ota.py raises the console to 2 Mbaud for it first (`console baud`). A line
//!   the UART damaged (at 2 Mbaud a late interrupt drops bytes) fails its CRC or offset and writes nothing; the
//!   transfer stays, and the sender sends that line again.
//! - **Wi-Fi:** `POST /ota?sha256=<hex>` on the file server (crate::files), the whole image in one request. It
//!   starves the CC link while 3 MB cross into flash (the hub dropped the board for a few seconds).
//!
//! Either way only in SETTING and disarmed, one at a time: the image (esptool elf2image's app.bin, as firmware/flash.sh
//! makes it) goes into the next slot, its SHA-256 is checked against the sender's and its header and app description
//! read back: the bootloader never re-hashes the app (CONFIG_BOOTLOADER_SKIP_VALIDATE_ALWAYS), so these are its checks. `ota status` names the running, the boot and the next slot, the running build (the first
//! 8 bytes of its ELF's SHA-256, as esptool stamps it) and what was received; `ota commit confirm` (SETTING, disarmed)
//! makes the received slot the one otadata boots, and the next `reboot` runs it.
//!
//! **No app rollback, ever** (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=n): a reset always boots the slot otadata selects.
//! `arm` refuses while the running slot is not the selected one ([`problem`]), so a committed build never waits for
//! a reset in flight.
//!
//! **No flash mapping:** the image goes in by SPI1 erases and writes (esp_partition, as the ring writer writes the
//! telemetry ring), checked by our own SHA-256 and the image's header and app description (plain reads), and the slot
//! is selected by writing otadata the way IDF's esp_rewrite_ota_data does (an alternating sector, the sequence number,
//! its CRC). esp_ota_end and esp_ota_set_boot_partition verify the image by mapping it, which turns the cache off and
//! parks the flight core: from a PSRAM stack it asserted, and from an internal one the chip's watchdog reset the board
//! in the verify, in both directions. The firmware maps flash nowhere at runtime.

use std::ffi::CStr;
use std::sync::Mutex;
use std::sync::atomic::{AtomicPtr, Ordering};

use base64::Engine as _;
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use esp_idf_svc::io::Write;
use esp_idf_svc::sys;
use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};
use sequencer::Stage;
use sha2::{Digest, Sha256};

use crate::status::FLIGHT;

/// What one read of the POST takes: a flash page's worth.
const CHUNK: usize = 4096;

/// What a slot can hold (partitions.csv: two app slots of 4 MB).
const SLOT_BYTES: usize = 0x40_0000;

/// One image received and checked, not yet committed.
struct Received {
    /// The slot it is in.
    slot: *const sys::esp_partition_t,
    /// Its size.
    bytes: usize,
    /// Its build: the first 8 bytes of its ELF's SHA-256.
    build: String,
}

// SAFETY: the slot points into the partition table, which lives as long as the board and never changes.
unsafe impl Send for Received {}

/// The image received and not yet committed, or why the last one failed.
static RECEIVED: Mutex<Option<Result<Received, String>>> = Mutex::new(None);

/// The transfer running, if one is.
static UPLOAD: Mutex<Option<Upload>> = Mutex::new(None);

/// The slot this boot runs, read once at boot ([`register_commands`]).
static RUNNING: AtomicPtr<sys::esp_partition_t> = AtomicPtr::new(std::ptr::null_mut());

/// The slot otadata boots next: read at boot, moved by `ota commit`.
static BOOT: AtomicPtr<sys::esp_partition_t> = AtomicPtr::new(std::ptr::null_mut());

/// A slot's label, `ota_0`.
fn label(slot: *const sys::esp_partition_t) -> String {
    if slot.is_null() {
        return "none".into();
    }
    // SAFETY: a partition-table entry, valid for the board's life; its label is NUL-terminated.
    unsafe { CStr::from_ptr((*slot).label.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}

/// A build's id: the first 8 bytes of the ELF's SHA-256 its app description carries, in hex.
fn build(description: &sys::esp_app_desc_t) -> String {
    description.app_elf_sha256[..8]
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// The erase unit ahead of the writes: a block, erased just before the first write into it.
const ERASE_BLOCK: usize = 0x1_0000;

/// One otadata sector (IDF's layout: two of them, the newer one wins).
const OTADATA_SECTOR: usize = 0x1000;

/// An esp_image's first byte.
const IMAGE_MAGIC: u8 = 0xE9;

/// Where the app description sits in an image: past the image header (24 bytes) and the first segment's (8).
const APP_DESC_OFFSET: usize = 32;

/// The app slot of OTA index `index` (ota_0, ota_1).
fn slot_of(index: u32) -> *const sys::esp_partition_t {
    // SAFETY: a plain lookup in the partition table.
    unsafe {
        sys::esp_partition_find_first(
            sys::esp_partition_type_t_ESP_PARTITION_TYPE_APP,
            sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_OTA_MIN + index,
            std::ptr::null(),
        )
    }
}

/// A slot's OTA index (ota_1: 1).
fn index_of(slot: *const sys::esp_partition_t) -> u32 {
    // SAFETY: a partition-table entry, valid for the board's life.
    unsafe { (*slot).subtype - sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_APP_OTA_MIN }
}

/// The otadata partition.
fn otadata() -> Result<*const sys::esp_partition_t, String> {
    // SAFETY: a plain lookup in the partition table.
    let partition = unsafe {
        sys::esp_partition_find_first(
            sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
            sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_DATA_OTA,
            std::ptr::null(),
        )
    };
    (!partition.is_null())
        .then_some(partition)
        .ok_or("no otadata partition".into())
}

/// An otadata entry's CRC, as the bootloader checks it: over its sequence number alone.
fn entry_crc(entry: &sys::esp_ota_select_entry_t) -> u32 {
    // SAFETY: four live bytes.
    unsafe { sys::esp_rom_crc32_le(u32::MAX, (&entry.ota_seq as *const u32).cast(), 4) }
}

/// The two otadata entries, each Some when valid (a sequence number set, its CRC right).
fn entries() -> Result<[Option<sys::esp_ota_select_entry_t>; 2], String> {
    let partition = otadata()?;
    let mut found = [None, None];
    for (sector, slot) in found.iter_mut().enumerate() {
        let mut entry = sys::esp_ota_select_entry_t::default();
        // SAFETY: the otadata partition and a live entry to fill (an SPI1 read, no mapping).
        sys::esp!(unsafe {
            sys::esp_partition_read(
                partition,
                sector * OTADATA_SECTOR,
                (&mut entry as *mut sys::esp_ota_select_entry_t).cast(),
                std::mem::size_of::<sys::esp_ota_select_entry_t>(),
            )
        })
        .map_err(|error| format!("otadata read: {error}"))?;
        if entry.ota_seq != u32::MAX && entry.crc == entry_crc(&entry) {
            *slot = Some(entry);
        }
    }
    Ok(found)
}

/// The slot otadata boots: the newest valid entry's, ota_0 when there is none (erased otadata, as flash.sh leaves it).
fn boot_slot() -> *const sys::esp_partition_t {
    let newest = entries()
        .ok()
        .and_then(|found| found.into_iter().flatten().map(|entry| entry.ota_seq).max());
    slot_of(newest.map_or(0, |sequence| (sequence - 1) % 2))
}

/// Makes `slot` the one otadata boots, as IDF's esp_rewrite_ota_data does: the next sequence number that names it,
/// into the sector that does not hold the newest entry (erased first). No rollback: the state stays undefined.
fn select(slot: *const sys::esp_partition_t) -> Result<(), String> {
    let partition = otadata()?;
    let found = entries()?;
    let index = index_of(slot);
    let newest = found
        .iter()
        .enumerate()
        .filter_map(|(sector, entry)| entry.map(|entry| (entry.ota_seq, sector)))
        .max();
    let (sequence, sector) = match newest {
        Some((current, sector)) => {
            let base = (index + 1) % 2;
            let mut step = 0;
            while current > base + step * 2 {
                step += 1;
            }
            (base + step * 2, 1 - sector)
        }
        None => (index + 1, 0),
    };
    let mut entry = sys::esp_ota_select_entry_t {
        ota_seq: sequence,
        seq_label: [0xFF; 20],
        ota_state: sys::esp_ota_img_states_t_ESP_OTA_IMG_UNDEFINED,
        crc: 0,
    };
    entry.crc = entry_crc(&entry);
    // SAFETY: the otadata partition, one of its two sectors; SPI1 erase and write, no mapping.
    sys::esp!(unsafe { sys::esp_partition_erase_range(partition, sector * OTADATA_SECTOR, OTADATA_SECTOR) })
        .map_err(|error| format!("otadata erase: {error}"))?;
    // SAFETY: as above; a live entry.
    sys::esp!(unsafe {
        sys::esp_partition_write(
            partition,
            sector * OTADATA_SECTOR,
            (&entry as *const sys::esp_ota_select_entry_t).cast(),
            std::mem::size_of::<sys::esp_ota_select_entry_t>(),
        )
    })
    .map_err(|error| format!("otadata write: {error}"))?;
    Ok(())
}

/// A transfer running: what it was promised and what it got so far.
struct Upload {
    /// The slot it writes.
    slot: *const sys::esp_partition_t,
    /// The size promised.
    length: usize,
    /// The bytes written so far.
    written: usize,
    /// How far the slot is erased.
    erased: usize,
    /// The SHA-256 promised, hex.
    expected: String,
    /// Over the bytes written so far.
    hasher: Sha256,
}

// SAFETY: the slot points into the partition table, which lives as long as the board and never changes.
unsafe impl Send for Upload {}

/// Why a transfer may not start now: off the pad, armed, or bad terms.
fn refused(length: usize, expected: &str) -> Option<String> {
    let flight = FLIGHT.load();
    if flight.stage != Stage::Setting || flight.armed {
        return Some("OTA only in setting and disarmed".into());
    }
    if expected.len() != 64 || !expected.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Some("sha256 must be 64 hex digits".into());
    }
    (length == 0 || length > SLOT_BYTES).then(|| format!("size must be 1..{SLOT_BYTES} bytes"))
}

/// Starts a transfer of `length` bytes whose SHA-256 is `expected`; the slot it goes to.
fn begin(length: usize, expected: &str) -> Result<String, String> {
    if let Some(refusal) = refused(length, expected) {
        return Err(refusal);
    }
    let mut upload = UPLOAD
        .lock()
        .map_err(|_| "the transfer state is poisoned".to_string())?;
    if upload.is_some() {
        return Err("a transfer is already running (ota abort drops it)".into());
    }
    // SAFETY: the partition table only: the app slot after the running one.
    let slot = unsafe { sys::esp_ota_get_next_update_partition(std::ptr::null()) };
    if slot.is_null() {
        return Err("no slot to write".into());
    }
    *upload = Some(Upload {
        slot,
        length,
        written: 0,
        erased: 0,
        expected: expected.to_ascii_lowercase(),
        hasher: Sha256::new(),
    });
    Ok(label(slot))
}

/// Writes the transfer's next bytes, which must start at `offset` (None: wherever it stands, the POST's); how many it
/// holds now. A line at the wrong offset writes nothing and keeps the transfer (the sender sends it again); a failed
/// erase or write drops it. The slot is erased a block ahead of the writes.
fn write(bytes: Vec<u8>, offset: Option<usize>) -> Result<usize, String> {
    let mut guard = UPLOAD
        .lock()
        .map_err(|_| "the transfer state is poisoned".to_string())?;
    let upload = guard.as_mut().ok_or("no transfer running (ota begin first)")?;
    if let Some(offset) = offset.filter(|offset| *offset != upload.written) {
        return Err(format!(
            "offset {offset}, the transfer stands at {}: nothing written",
            upload.written
        ));
    }
    let end = upload.written + bytes.len();
    if end > upload.length {
        *guard = None;
        return Err("more bytes than promised: the transfer is dropped".into());
    }
    let mut failure = None;
    while upload.erased < end && failure.is_none() {
        // SAFETY: the slot from the partition table; an SPI1 erase of one block within it.
        failure = sys::esp!(unsafe { sys::esp_partition_erase_range(upload.slot, upload.erased, ERASE_BLOCK) })
            .err()
            .map(|error| format!("erase at {}: {error}", upload.erased));
        upload.erased += ERASE_BLOCK;
    }
    if failure.is_none() {
        // SAFETY: the slot and live bytes; an SPI1 write within the erased reach.
        failure = sys::esp!(unsafe {
            sys::esp_partition_write(upload.slot, upload.written, bytes.as_ptr().cast(), bytes.len())
        })
        .err()
        .map(|error| format!("write at {}: {error}", upload.written));
    }
    if let Some(error) = failure {
        *guard = None;
        return Err(error);
    }
    upload.hasher.update(&bytes);
    // progress in the log every tenth, which CC's log view streams (the operator: to see it move)
    let tenth = |bytes: usize| bytes * 10 / upload.length;
    if tenth(end) > tenth(upload.written) {
        log::info!(
            "ota :: {end} of {} bytes ({}%)",
            upload.length,
            end * 100 / upload.length
        );
    }
    upload.written = end;
    Ok(end)
}

/// The image's checks by plain reads: an esp_image of this chip, and its app description; its build.
fn check_image(slot: *const sys::esp_partition_t) -> Result<String, String> {
    let mut header = [0u8; APP_DESC_OFFSET];
    // SAFETY: the slot and a live buffer (an SPI1 read).
    sys::esp!(unsafe { sys::esp_partition_read(slot, 0, header.as_mut_ptr().cast(), header.len()) })
        .map_err(|error| format!("header read: {error}"))?;
    let chip = u16::from_le_bytes([header[12], header[13]]);
    if header[0] != IMAGE_MAGIC || chip != sys::esp_chip_id_t_ESP_CHIP_ID_ESP32P4 {
        return Err(format!("no ESP32-P4 image (magic {:#04x}, chip {chip})", header[0]));
    }
    let mut description = sys::esp_app_desc_t::default();
    // SAFETY: the slot and a live description to fill (an SPI1 read).
    sys::esp!(unsafe {
        sys::esp_partition_read(
            slot,
            APP_DESC_OFFSET,
            (&mut description as *mut sys::esp_app_desc_t).cast(),
            std::mem::size_of::<sys::esp_app_desc_t>(),
        )
    })
    .map_err(|error| format!("app description read: {error}"))?;
    if description.magic_word != sys::ESP_APP_DESC_MAGIC_WORD {
        return Err("no app description in the image".into());
    }
    Ok(build(&description))
}

/// Ends the transfer: every byte promised, the SHA-256 the sender's, the image this chip's. The outcome becomes `ota
/// status`'s `received`.
fn end() -> Result<String, String> {
    let upload = UPLOAD
        .lock()
        .map_err(|_| "the transfer state is poisoned".to_string())?
        .take()
        .ok_or("no transfer running (ota begin first)")?;
    let digest: String = upload
        .hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    let outcome = if upload.written != upload.length {
        Err(format!(
            "{} of {} bytes: the transfer is dropped",
            upload.written, upload.length
        ))
    } else if digest != upload.expected {
        Err(format!(
            "sha256 {digest}, the sender's {}: not this image",
            upload.expected
        ))
    } else {
        check_image(upload.slot).map(|build| Received {
            slot: upload.slot,
            bytes: upload.written,
            build,
        })
    };
    let text = match &outcome {
        Ok(received) => {
            let text = format!(
                "{} {} bytes, build {}",
                label(received.slot),
                received.bytes,
                received.build
            );
            log::info!("ota :: received {text}: ota commit confirm, then reboot");
            Ok(text)
        }
        Err(error) => {
            log::error!("ota :: {error}");
            Err(error.clone())
        }
    };
    if let Ok(mut received) = RECEIVED.lock() {
        *received = Some(outcome);
    }
    text
}

/// Drops the running transfer, if any: the slot stays unselected.
fn abort() -> bool {
    UPLOAD.lock().map(|mut upload| upload.take().is_some()).unwrap_or(false)
}

/// Why `arm` refuses: the running slot is not the one otadata boots (a committed build waits for a reboot on the
/// ground, never for a reset in flight); None when they are the same.
pub fn problem() -> Option<String> {
    let (running, boot) = (RUNNING.load(Ordering::Acquire), BOOT.load(Ordering::Acquire));
    (running != boot).then(|| {
        format!(
            "the board runs {} but boots {} next: reboot on the ground first",
            label(running),
            label(boot)
        )
    })
}

/// `POST /ota?sha256=<hex>`: the whole image in one request (see the module notes). Answers 200 with the slot, size
/// and build; 409 refused (off the pad, armed, busy, bad terms); 500 a write or a check that failed (the slot is left
/// unselected).
pub fn receive(mut request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    let length = request
        .header("Content-Length")
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(0);
    let expected = request
        .uri()
        .split_once("sha256=")
        .map(|(_, value)| value.split('&').next().unwrap_or("").to_string())
        .unwrap_or_default();
    if let Err(error) = begin(length, &expected) {
        request
            .into_status_response(409)?
            .write_all(format!("refused: {error}\n").as_bytes())?;
        return Ok(());
    }
    let mut failure = None;
    let mut taken = 0usize;
    while taken < length {
        let mut buffer = vec![0u8; CHUNK.min(length - taken)];
        match request.read(&mut buffer) {
            Ok(0) => {
                failure = Some(format!("the sender stopped after {taken} of {length} bytes"));
                break;
            }
            Ok(count) => {
                buffer.truncate(count);
                taken += count;
                if let Err(error) = write(buffer, None) {
                    failure = Some(error);
                    break;
                }
            }
            Err(error) => {
                failure = Some(format!("read at {taken}: {error}"));
                break;
            }
        }
    }
    let outcome = match failure {
        None => end(),
        Some(error) => {
            abort();
            Err(error)
        }
    };
    let (status, text) = match outcome {
        Ok(text) => (200, format!("ok ota {text}: ota commit confirm, then reboot\n")),
        Err(error) => (500, format!("err ota {error}\n")),
    };
    request.into_status_response(status)?.write_all(text.as_bytes())?;
    Ok(())
}

/// The running build: the first 8 bytes of its ELF's SHA-256.
fn running_build() -> String {
    // SAFETY: the running app's description, static for the board's life.
    build(unsafe { &*sys::esp_app_get_description() })
}

/// An answer line: `ok ota <text>` or `err ota <why>`.
fn answer(outcome: Result<String, String>) -> String {
    match outcome {
        Ok(text) => cc_protocol::build("ok", &["ota", &text], &[]),
        Err(error) => cc_protocol::build("err", &["ota", &error], &[]),
    }
}

/// `ota commit confirm`: the received image becomes the one otadata boots.
fn commit() -> Result<String, String> {
    if FLIGHT.load().armed {
        return Err("armed: OTA only while disarmed".into());
    }
    let mut received = RECEIVED
        .lock()
        .map_err(|_| "the received state is poisoned".to_string())?;
    let Some(Ok(image)) = received.as_ref() else {
        return Err("no verified image received".into());
    };
    select(image.slot).map_err(|error| format!("{}: {error}", label(image.slot)))?;
    BOOT.store(image.slot.cast_mut(), Ordering::Release);
    let text = format!(
        "committed {} build {}: reboot to run it",
        label(image.slot),
        image.build
    );
    log::warn!("ota :: {text}");
    *received = None;
    Ok(text)
}

/// Reads the slots (on the main task: its stack is internal) and registers the OTA's console commands (crate::gcl).
pub fn register_commands() {
    // SAFETY: plain reads of the partition table; otadata by SPI1 reads (boot_slot).
    unsafe {
        RUNNING.store(sys::esp_ota_get_running_partition().cast_mut(), Ordering::Release);
        BOOT.store(boot_slot().cast_mut(), Ordering::Release);
    }
    gcl::register(vec![
        status_command(),
        begin_command(),
        write_command(),
        end_command(),
        abort_command(),
        commit_command(),
    ]);
}

/// `ota status` (the Glider Command Language).
fn status_command() -> Command {
    Command {
        owner: "ota",
        words: "ota status",
        about: "the app slots (running, boot, next), the running build, the transfer running, the image received",
        gate: Gate::Open,
        reach: Reach::Everywhere,
        parameters: &[],
        bare_words: &[],
        run: Box::new(|_| {
            let (running, boot) = (RUNNING.load(Ordering::Acquire), BOOT.load(Ordering::Acquire));
            // SAFETY: the partition table only: the app slot after the running one.
            let next = unsafe { sys::esp_ota_get_next_update_partition(std::ptr::null()) };
            let transfer = match UPLOAD.lock().as_deref() {
                Ok(Some(upload)) => format!("{}/{}", upload.written, upload.length),
                _ => "none".into(),
            };
            let received = match RECEIVED.lock().as_deref() {
                Ok(Some(Ok(received))) => format!("{}:{}", label(received.slot), received.build),
                Ok(Some(Err(error))) => format!("failed:{error}"),
                _ => "none".into(),
            };
            let named = [
                ("running", label(running)),
                ("boot", label(boot)),
                ("next", label(next)),
                ("build", running_build()),
                ("transfer", transfer),
                ("received", received),
            ];
            let named: Vec<(&str, &str)> = named.iter().map(|(key, value)| (*key, value.as_str())).collect();
            cc_protocol::build("ok", &["ota"], &named)
        }),
    }
}

/// `ota begin` (the Glider Command Language).
fn begin_command() -> Command {
    Command {
        owner: "ota",
        words: "ota begin",
        about: "start receiving an image into the next slot (the console's transfer: tools/ota.py --console)",
        gate: Gate::Setting,
        reach: Reach::Everywhere,
        parameters: &[
            Parameter {
                name: "size",
                kind: Kind::Integer {
                    low: 1,
                    high: SLOT_BYTES as i64,
                },
                default: None,
                about: "the image's bytes",
            },
            Parameter {
                name: "sha256",
                kind: Kind::Text,
                default: None,
                about: "the image's SHA-256, 64 hex digits",
            },
        ],
        bare_words: &[],
        run: Box::new(|arguments| {
            let opened = begin(arguments.integer("size") as usize, arguments.text("sha256"));
            answer(opened.map(|slot| format!("into {slot}")))
        }),
    }
}

/// `ota write` (the Glider Command Language).
fn write_command() -> Command {
    Command {
        owner: "ota",
        words: "ota write",
        about: "the image's next bytes: a damaged line (CRC, offset) writes nothing, send it again",
        gate: Gate::Setting,
        reach: Reach::Everywhere,
        parameters: &[
            Parameter {
                name: "offset",
                kind: Kind::Integer {
                    low: 0,
                    high: SLOT_BYTES as i64,
                },
                default: None,
                about: "where these bytes start in the image",
            },
            Parameter {
                name: "crc",
                kind: Kind::Text,
                default: None,
                about: "the bytes' CRC-32 (zlib's), hex",
            },
            Parameter {
                name: "data",
                kind: Kind::Text,
                default: None,
                about: "the bytes, base64url with no padding (a plain token)",
            },
        ],
        bare_words: &[],
        run: Box::new(|arguments| {
            let outcome = URL_SAFE_NO_PAD
                .decode(arguments.text("data"))
                .map_err(|error| format!("data is no base64url ({error}): nothing written"))
                .and_then(|bytes| {
                    let crc = format!("{:08x}", recorder_wire::crc32(0, &bytes));
                    if crc == arguments.text("crc").to_ascii_lowercase() {
                        Ok(bytes)
                    } else {
                        Err(format!(
                            "crc {crc}, the line's {}: nothing written",
                            arguments.text("crc")
                        ))
                    }
                })
                .and_then(|bytes| write(bytes, Some(arguments.integer("offset") as usize)))
                .map(|written| written.to_string());
            answer(outcome)
        }),
    }
}

/// `ota end` (the Glider Command Language).
fn end_command() -> Command {
    Command {
        owner: "ota",
        words: "ota end",
        about: "close the transfer: every byte, the SHA-256 and the image checked",
        gate: Gate::Setting,
        reach: Reach::Everywhere,
        parameters: &[],
        bare_words: &[],
        run: Box::new(|_| answer(end())),
    }
}

/// `ota abort` (the Glider Command Language).
fn abort_command() -> Command {
    Command {
        owner: "ota",
        words: "ota abort",
        about: "drop the transfer running; the slot stays unselected",
        gate: Gate::Open,
        reach: Reach::Everywhere,
        parameters: &[],
        bare_words: &[],
        run: Box::new(|_| {
            answer(if abort() {
                Ok("aborted".into())
            } else {
                Err("no transfer running".into())
            })
        }),
    }
}

/// `ota commit` (the Glider Command Language).
fn commit_command() -> Command {
    Command {
        owner: "ota",
        words: "ota commit",
        about: "make the received image the one the next boot runs (then reboot); disarmed only",
        gate: Gate::Setting,
        reach: Reach::Everywhere,
        parameters: &[],
        bare_words: &[gcl::CONFIRM],
        run: Box::new(|_| answer(commit())),
    }
}
