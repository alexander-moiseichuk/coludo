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
//! makes it) goes into the next slot (erased only as far as it reaches), its SHA-256 is checked against the sender's,
//! and the image verified (esp_ota_end): the bootloader never re-hashes the app (CONFIG_BOOTLOADER_SKIP_VALIDATE_ALWAYS),
//! so this is its one check. `ota status` names the running, the boot and the next slot, the running build (the first
//! 8 bytes of its ELF's SHA-256, as esptool stamps it) and what was received; `ota commit confirm` (SETTING, disarmed)
//! makes the received slot the one otadata boots, and the next `reboot` runs it.
//!
//! **No app rollback, ever** (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=n): a reset always boots the slot otadata selects.
//! `arm` refuses while the running slot is not the selected one ([`problem`]), so a committed build never waits for
//! a reset in flight.
//!
//! **Off PSRAM stacks:** the OTA calls map flash (otadata, the image's verify), which turns the cache off, and a task
//! whose stack is in PSRAM must not do that (`ota status` on the executor, and an upload on httpd, asserted in
//! spi_flash_disable_interrupts_caches_and_other_cpu). So the slots are read once at boot on the main task and kept,
//! and every esp_ota call runs on one writer thread with an internal stack, alive for a transfer only ([`Writer`]),
//! or on a short-lived one for the commit ([`on_internal_stack`]).

use std::ffi::CStr;
use std::sync::Mutex;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};

use base64::Engine as _;
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use esp_idf_svc::io::Write;
use esp_idf_svc::sys;
use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};
use sequencer::Stage;
use sha2::{Digest, Sha256};

use crate::status::FLIGHT;

/// What one read of the POST takes: a flash page's worth.
const CHUNK: usize = 4096;

/// What a slot can hold (partitions.csv: two app slots of 3 MB).
const SLOT_BYTES: usize = 0x30_0000;

/// The stack of the threads that map flash: an image's verify (SHA-256 over it) and otadata's sectors.
const INTERNAL_STACK: usize = 8 << 10;

/// One image received and verified, not yet committed.
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

/// Spawns `job` on a thread whose stack is internal RAM (see the module notes).
///
/// Errors: the thread would not spawn (no internal RAM for its stack).
fn spawn_internal<T: Send + 'static>(
    job: impl FnOnce() -> T + Send + 'static,
) -> anyhow::Result<std::thread::JoinHandle<T>> {
    ThreadSpawnConfiguration {
        name: Some(c"ota"),
        stack_size: INTERNAL_STACK,
        priority: 3,
        pin_to_core: Some(Core::Core0),
        stack_alloc_caps: MallocCap::Internal | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(INTERNAL_STACK).spawn(job);
    ThreadSpawnConfiguration::default().set()?;
    Ok(spawned?)
}

/// Runs `job` on a thread whose stack is internal RAM, and waits for it.
fn on_internal_stack<T: Send + 'static>(job: impl FnOnce() -> T + Send + 'static) -> anyhow::Result<T> {
    spawn_internal(job)?
        .join()
        .map_err(|_| anyhow::anyhow!("the internal-stack thread panicked"))
}

/// What the writer thread is asked.
enum Job {
    /// Write these bytes next.
    Write(Vec<u8>, SyncSender<Result<(), String>>),
    /// Close and verify the image; its build.
    End(SyncSender<Result<String, String>>),
}

/// The writer thread: every esp_ota call of a transfer, on an internal stack.
struct Writer;

impl Writer {
    /// Opens the next slot for `length` bytes on a new writer thread: the slot and the thread's mailbox.
    fn open(length: usize) -> Result<(usize, SyncSender<Job>), String> {
        let (jobs, inbox) = sync_channel::<Job>(1);
        let (opened, opening) = sync_channel::<Result<usize, String>>(1);
        spawn_internal(move || Writer::run(length, &opened, &inbox))
            .map_err(|error| format!("no writer thread: {error}"))?;
        let slot = opening
            .recv()
            .map_err(|_| "the writer thread ended before the slot opened".to_string())??;
        Ok((slot, jobs))
    }

    /// The thread: begin, then each job until the end, or an abort (the mailbox dropped).
    fn run(length: usize, opened: &SyncSender<Result<usize, String>>, inbox: &Receiver<Job>) {
        // SAFETY: the next app slot from the partition table (NULL: after the running one).
        let slot = unsafe { sys::esp_ota_get_next_update_partition(std::ptr::null()) };
        if slot.is_null() {
            let _ = opened.send(Err("no slot to write".into()));
            return;
        }
        let mut handle: sys::esp_ota_handle_t = 0;
        // SAFETY: a valid slot; esp_ota_begin erases `length` bytes of it (the image's reach, not the whole slot).
        if let Err(error) = sys::esp!(unsafe { sys::esp_ota_begin(slot, length, &mut handle) }) {
            let _ = opened.send(Err(format!("begin on {}: {error}", label(slot))));
            return;
        }
        let _ = opened.send(Ok(slot as usize));
        loop {
            match inbox.recv() {
                Ok(Job::Write(bytes, reply)) => {
                    // SAFETY: an open OTA handle and live bytes.
                    let written = sys::esp!(unsafe { sys::esp_ota_write(handle, bytes.as_ptr().cast(), bytes.len()) })
                        .map_err(|error| format!("write: {error}"));
                    let _ = reply.send(written);
                }
                Ok(Job::End(reply)) => {
                    // SAFETY: an open OTA handle; esp_ota_end verifies the image it holds.
                    let ended = sys::esp!(unsafe { sys::esp_ota_end(handle) })
                        .map_err(|error| format!("the image in {} did not verify: {error}", label(slot)))
                        .and_then(|()| {
                            let mut description = sys::esp_app_desc_t::default();
                            // SAFETY: a verified app slot and a live description to fill.
                            sys::esp!(unsafe { sys::esp_ota_get_partition_description(slot, &mut description) })
                                .map(|()| build(&description))
                                .map_err(|error| format!("no app description in {}: {error}", label(slot)))
                        });
                    let _ = reply.send(ended);
                    return;
                }
                Err(_) => {
                    // SAFETY: an open OTA handle, abandoned; the slot stays unselected.
                    unsafe { sys::esp_ota_abort(handle) };
                    return;
                }
            }
        }
    }
}

/// A transfer running: what it was promised and what it got so far.
struct Upload {
    /// The writer thread's mailbox (dropping it aborts the transfer).
    jobs: SyncSender<Job>,
    /// The slot it writes.
    slot: usize,
    /// The size promised.
    length: usize,
    /// The bytes written so far.
    written: usize,
    /// The SHA-256 promised, hex.
    expected: String,
    /// Over the bytes written so far.
    hasher: Sha256,
}

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
    let (slot, jobs) = Writer::open(length)?;
    *upload = Some(Upload {
        jobs,
        slot,
        length,
        written: 0,
        expected: expected.to_ascii_lowercase(),
        hasher: Sha256::new(),
    });
    Ok(label(slot as *const sys::esp_partition_t))
}

/// Writes the transfer's next bytes, which must start at `offset` (None: wherever it stands, the POST's); how many it
/// holds now. A line at the wrong offset writes nothing and keeps the transfer (the sender sends it again); a failed
/// write drops it.
fn write(bytes: Vec<u8>, offset: Option<usize>) -> Result<usize, String> {
    let mut guard = UPLOAD
        .lock()
        .map_err(|_| "the transfer state is poisoned".to_string())?;
    let upload = guard.as_mut().ok_or("no transfer running (ota begin first)")?;
    if let Some(offset) = offset.filter(|offset| *offset != upload.written) {
        return Err(format!("offset {offset}, the transfer stands at {}: nothing written", upload.written));
    }
    if upload.written + bytes.len() > upload.length {
        *guard = None;
        return Err("more bytes than promised: the transfer is dropped".into());
    }
    upload.hasher.update(&bytes);
    let count = bytes.len();
    let (reply, answer) = sync_channel(1);
    let outcome = upload
        .jobs
        .send(Job::Write(bytes, reply))
        .map_err(|_| "the writer thread is gone".to_string())
        .and_then(|()| answer.recv().map_err(|_| "the writer thread is gone".to_string())?);
    match outcome {
        Ok(()) => {
            upload.written += count;
            Ok(upload.written)
        }
        Err(error) => {
            *guard = None;
            Err(error)
        }
    }
}

/// Ends the transfer: every byte promised, the SHA-256 the sender's, the image verified. The outcome becomes `ota
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
        let (reply, answer) = sync_channel(1);
        upload
            .jobs
            .send(Job::End(reply))
            .map_err(|_| "the writer thread is gone".to_string())
            .and_then(|()| answer.recv().map_err(|_| "the writer thread is gone".to_string())?)
            .map(|build| Received {
                slot: upload.slot as *const sys::esp_partition_t,
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

/// Drops the running transfer, if any: the writer thread aborts, the slot stays unselected.
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
    let slot = image.slot as usize;
    // otadata is mapped: an internal stack
    let set = on_internal_stack(move || {
        // SAFETY: a verified app slot from the partition table.
        sys::esp!(unsafe { sys::esp_ota_set_boot_partition(slot as *const sys::esp_partition_t) })
            .map_err(|error| error.to_string())
    })
    .map_err(|error| format!("the commit did not run: {error}"))?;
    set.map_err(|error| format!("{}: {error}", label(image.slot)))?;
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
    // SAFETY: plain reads of the partition table and otadata, on the main task's internal stack.
    unsafe {
        RUNNING.store(sys::esp_ota_get_running_partition().cast_mut(), Ordering::Release);
        BOOT.store(sys::esp_ota_get_boot_partition().cast_mut(), Ordering::Release);
    }
    gcl::register(vec![
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
        },
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
        },
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
                            Err(format!("crc {crc}, the line's {}: nothing written", arguments.text("crc")))
                        }
                    })
                    .and_then(|bytes| write(bytes, Some(arguments.integer("offset") as usize)))
                    .map(|written| written.to_string());
                answer(outcome)
            }),
        },
        Command {
            owner: "ota",
            words: "ota end",
            about: "close the transfer: every byte, the SHA-256 and the image checked",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[],
            run: Box::new(|_| answer(end())),
        },
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
        },
        Command {
            owner: "ota",
            words: "ota commit",
            about: "make the received image the one the next boot runs (then reboot); disarmed only",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[gcl::CONFIRM],
            run: Box::new(|_| answer(commit())),
        },
    ]);
}
