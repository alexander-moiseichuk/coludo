//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The remote flight recorder (M20, doc/specs/avionics-port.md M19/M20) on an ESP32-C6 SuperMini: the P4's
//! uart_saver sends its recorder's wire lines, `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`, and this board files them
//! on its own SD card, so the data survives the P4 and the P4's card. It replaces the Luckfox's recorderd.
//!
//! - The link: UART1 RX on GPIO17 (the board's RX pin) at 921600 baud, 8N1; TX + GND only from the P4.
//! - The card: SPI2, SCK GPIO19, MOSI GPIO20, MISO GPIO14, CS GPIO18, FAT at `/sd`.
//! - The lines are split as recorderd split them (crates/recorder_wire `split`), and filed where the P4's ring_saver
//!   files them on its own card (spec §3.10), so the same tools read both cards and the files match byte for byte: a
//!   stream line to `<session>/recordings/<routing>`, a session.csv line to the session's `session.csv` and the
//!   card's `sessions.csv`, a log line to the session's `recorder.log`, each line's session named by its routing (the
//!   P4's tee, crates/recorder saver `tee`). The P4's rules for a file's end hold too: a header line printed again
//!   after a reboot stays out of a file that holds one, and a torn last line gets its `\n` first. Each line is filed
//!   as it came, the payload after its routing, once: a line the P4 sends again after a reboot (no newer than its
//!   file's last) stays out. Its wrapper is checked (recorder_wire `verify`) and counted; a damaged line goes raw to
//!   the session's `damaged.log`, so the data files hold what the P4's hold.
//! - Lines gather in memory and go to the card once a second: per file, open for append, write, close (FatFs syncs
//!   at close), so a power cut costs at most the last second.
//! - Every 5 s one status line on the USB console: bytes and lines in, good and damaged, sessions, card errors. The
//!   console takes `files [dir]` (every file with its size and CRC-32, to compare with the P4's own card) and `read
//!   <path> <offset> <length>` (a file's bytes, to find where two copies part).
//! - A card that does not answer is no stop: the link is read and counted all the same (lines then go unfiled,
//!   counted), and the mount is tried again every 10 s. The first try traces the card's init on the console, each SPI
//!   command and the card's answer.
//! - A mounted card is read back before anything is written: its name, size and SPI clock, the volume's free space
//!   and the root's entries. A card that cannot be read is not written to (the mount counts as failed).
//! - The remote beacon (crate::remote): the glider's beacon.csv report sets an access point `<BOARD>_beacon` and its
//!   page, the glider's own; the last report on the card is replayed after a restart.

mod remote;

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{CStr, CString};
use std::fs::OpenOptions;
use std::io::{Read as _, Seek as _, SeekFrom, Write as _};
use std::time::{Duration, Instant};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::fs::fatfs::Fatfs;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::spi::SdSpiHostDriver;
use esp_idf_svc::hal::sd::{SdCardConfiguration, SdCardDriver};
use esp_idf_svc::hal::spi::SPI2;
use esp_idf_svc::hal::spi::config::DriverConfig;
use esp_idf_svc::hal::spi::{Dma, SpiDriver};
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::hal::uart::{UartConfig, UartRxDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::io::vfs::MountedFatfs;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::{
    esp, esp_log_level_set, esp_log_level_t, esp_log_level_t_ESP_LOG_DEBUG, esp_log_level_t_ESP_LOG_INFO,
    esp_log_level_t_ESP_LOG_VERBOSE, esp_vfs_fat_info,
};

/// The card's mount point.
const ROOT: &str = "/sd";

/// The P4's uart_saver's rate (plan B's recorder link).
const BAUD: u32 = 921_600;

/// The UART's receive buffer, bytes: ~0.35 s of a saturated link, so a card write never loses a byte.
const RX_BYTES: usize = 32 << 10;

/// How long a read waits for bytes.
const READ_WAIT: Duration = Duration::from_millis(20);

/// How often the gathered lines go to the card.
const FLUSH_EVERY: Duration = Duration::from_secs(1);

/// How often the console gets a status line.
const STATUS_EVERY: Duration = Duration::from_secs(5);

/// When the first status line comes after boot: soon, but after the host's console has come back.
const FIRST_STATUS: Duration = Duration::from_secs(2);

/// The longest unterminated line kept: a longer run of bytes with no `\n` is noise, dropped and counted.
const LINE_LIMIT: usize = 8 << 10;

/// How often an unmounted card is tried again.
const MOUNT_EVERY: Duration = Duration::from_secs(10);

/// The card's SPI pins: SCK, MOSI, MISO, CS (the SuperMini's right header).
const CARD_PINS: [i32; 4] = [19, 20, 14, 18];

/// The card-init log tags the first mount raises (ESP-IDF's names), so the console shows each SPI command and the
/// card's answer: the commands with their responses, the host's set-up and clock, an R1 error, the failed step.
const TRACE_TAGS: [(&CStr, esp_log_level_t); 5] = [
    (c"sdmmc_cmd", esp_log_level_t_ESP_LOG_VERBOSE),
    (c"sdspi_host", esp_log_level_t_ESP_LOG_DEBUG),
    (c"sdspi_transaction", esp_log_level_t_ESP_LOG_DEBUG),
    (c"sdmmc_init", esp_log_level_t_ESP_LOG_DEBUG),
    (c"sdmmc_common", esp_log_level_t_ESP_LOG_DEBUG),
];

/// The root's entries the read-back lists by name.
const LISTED: usize = 16;

/// How often the console thread looks for typed bytes.
const CONSOLE_POLL: Duration = Duration::from_millis(100);

/// The console thread's stack: a command and its directory walk.
const CONSOLE_STACK: usize = 8 << 10;

/// The link's loop's priority (the main task's, raised from IDF's 1): above the console's, so a `files` walk runs
/// only while the loop waits for bytes. With the console above it (pthread's default, 5), walking 5 MB starved the
/// UART reader: its buffer overflowed, lines were lost and damaged (2026-10-11).
const LINK_PRIORITY: u32 = 3;

/// The console thread's priority: below the link's loop.
const CONSOLE_PRIORITY: u8 = 2;

/// The routing of the session index's lines, and their file in a session's recordings.
const SESSION_INDEX: &str = "session.csv";

/// A session's log file: the lines with no routing of their own.
const LOG_FILE: &str = "recorder.log";

/// A session's damaged lines, raw: out of the data files.
const DAMAGED_FILE: &str = "damaged.log";

/// The card's global index at its root: every session.csv line.
const GLOBAL_INDEX: &str = "sessions.csv";

/// How much of a file's end a first look reads: more than any header or session.csv line.
const TAIL_BYTES: usize = 1024;

/// What the receiver counted.
#[derive(Default)]
struct Counts {
    /// Bytes read from the link.
    bytes: u64,
    /// Lines whose wrapper checks out.
    good: u64,
    /// Lines with no wrapper or a damaged one (filed all the same).
    damaged: u64,
    /// Runs of bytes dropped for want of a line end.
    overlong: u32,
    /// Card calls that failed.
    card_errors: u32,
    /// Lines the P4 sent again after a reboot, kept out.
    repeated: u64,
    /// Lines read while no card was mounted, so filed nowhere.
    unfiled: u64,
}

/// What a file on the card holds, as a first look found it and the lines since kept it.
#[derive(Default)]
struct Known {
    /// It holds a whole line (a fragment a cut left alone is none): a header line printed again stays out.
    held: bool,
    /// Its last line has no `\n` (a cut tore it): a `\n` goes before the next append.
    torn: bool,
    /// Its last line's uptime (us) and bytes: a line no newer is one the P4 sent again.
    last: Option<(u64, Vec<u8>)>,
    /// For the global index: the session its last row names.
    last_session: Option<String>,
}

/// The lines gathered since the last flush, by file, and what the receiver counted.
struct Filer {
    /// The bytes to append, by path.
    pending: BTreeMap<String, Vec<u8>>,
    /// The files looked at this boot.
    known: BTreeMap<String, Known>,
    /// The directories made this boot.
    made: BTreeSet<String>,
    /// A card is mounted: each line is weighed against what its file holds, and filed.
    mounted: bool,
    /// The session the last line named: the session of a damaged line, and of a line from a P4 whose tee named none.
    session: String,
    /// The sessions seen this boot.
    sessions: BTreeSet<String>,
    /// The glider's newest beacon report filed since the loop last took it (crate::remote).
    report: Option<beacon::Report>,
    /// The counts.
    counts: Counts,
}

impl Filer {
    /// Nothing gathered.
    fn new() -> Filer {
        Filer {
            pending: BTreeMap::new(),
            known: BTreeMap::new(),
            made: BTreeSet::new(),
            mounted: false,
            session: "unknown".to_string(),
            sessions: BTreeSet::new(),
            report: None,
            counts: Counts::default(),
        }
    }

    /// One demuxed line: `routing` and `payload` (its `\n` included), filed where the P4's ring_saver files it.
    ///
    /// The P4's tee names the session of every line (crates/recorder saver `tee`): `<session>_<stream>.csv`,
    /// `<session>_recorder.log` for a log line, `<session>_session.csv` for a session.csv line. The last two carry the
    /// body as the P4 files it, its checks made under the line's own routing (none, `session.csv`). A line with no
    /// session named (a P4 from before that tee) goes under the session the last line named.
    ///
    /// The P4's card holds each line once, its saver filing the lines of a batch only once it commits it; its tee
    /// sends them before, so after a reboot the next boot's saver sends again what the last commit had not filed.
    /// Every file's lines rise in uptime (measured: no file of a session ever steps back), so a line no newer than
    /// its file's last is one sent again, and stays out. A header line stays out of a file that holds one (the
    /// global index: one per session), as the P4 saver keeps it out (crates/recorder saver `holds_header`).
    ///
    /// A damaged line goes to the session's `damaged.log`, raw, its routing kept: the data files hold only lines
    /// whose wrapper checks out, as the P4's do (a reboot cuts the line on the wire: its routing names no file).
    fn line(&mut self, routing: Option<&[u8]>, payload: &[u8]) {
        let body = payload.strip_suffix(b"\n").unwrap_or(payload);
        let text = routing.map(String::from_utf8_lossy);
        let (session, name) = match text.as_deref() {
            None => (None, LOG_FILE),
            Some(routing) => match recorder_wire::routing_session(routing) {
                Some((session, rest @ (LOG_FILE | SESSION_INDEX))) => (Some(session), rest),
                Some((session, _)) => (Some(session), routing),
                None => (None, routing),
            },
        };
        let checked = (name != LOG_FILE).then_some(name.as_bytes());
        let Some(verified) = recorder_wire::verify(checked, body) else {
            self.counts.damaged += 1;
            let path = format!("{ROOT}/{}/recordings/{DAMAGED_FILE}", self.session);
            let pending = self.pending.entry(path).or_default();
            if let Some(routing) = routing {
                pending.push(b'@');
                pending.extend_from_slice(routing);
                pending.push(b'@');
            }
            pending.extend_from_slice(payload);
            return;
        };
        self.counts.good += 1;
        if let Some(session) = session
            && session != self.session
        {
            self.session = session.to_string();
            self.sessions.insert(self.session.clone());
        }
        let name = name.replace(['/', '\\'], "_");
        let path = format!("{ROOT}/{}/recordings/{name}", self.session);
        // a row's or a log line's uptime; a header line, a column list, has none
        let digits = verified.iter().take_while(|byte| byte.is_ascii_digit()).count();
        let uptime = std::str::from_utf8(&verified[..digits])
            .ok()
            .and_then(|digits| digits.parse().ok());
        if self.mounted && !self.fresh(&path, uptime, body) {
            self.counts.repeated += 1;
            return;
        }
        self.pending.entry(path).or_default().extend_from_slice(payload);
        if session.is_some_and(|session| {
            name.strip_prefix(session).and_then(|rest| rest.strip_prefix('_')) == Some(beacon::FILE)
        }) && let Some(report) = beacon::Report::parse(verified)
        {
            // the root's beacon.csv keeps every report as its wire line, routing and all: a restart replays the last
            // (crate::remote `last_report`), whichever session wrote it
            let replay = self.pending.entry(format!("{ROOT}/{}", beacon::FILE)).or_default();
            replay.push(b'@');
            replay.extend_from_slice(name.as_bytes());
            replay.push(b'@');
            replay.extend_from_slice(payload);
            self.report = Some(report);
        }
        if name == SESSION_INDEX {
            let global = format!("{ROOT}/{GLOBAL_INDEX}");
            if self.mounted {
                let session = self.session.clone();
                let known = self.known(&global);
                if uptime.is_none() && known.last_session.as_deref() == Some(session.as_str()) {
                    return; // the header again: the global index holds one per session
                }
                if uptime.is_some() {
                    known.last_session = Some(session);
                }
            }
            self.pending.entry(global).or_default().extend_from_slice(payload);
        }
    }

    /// Whether the line `body`, its `uptime` (None for a header), is new to `path`: newer than the file's last line
    /// (the same uptime counts when the bytes differ), or a header the file holds none of. Kept as the file's last.
    fn fresh(&mut self, path: &str, uptime: Option<u64>, body: &[u8]) -> bool {
        let known = self.known(path);
        let fresh = match (uptime, &known.last) {
            (None, _) => !known.held,
            (Some(uptime), Some((last, bytes))) => uptime > *last || (uptime == *last && bytes.as_slice() != body),
            (Some(_), None) => true,
        };
        if fresh {
            known.held = true;
            if let Some(uptime) = uptime {
                known.last = Some((uptime, body.to_vec()));
            }
        }
        fresh
    }

    /// What `path` holds, looked at on the card the first time (a card that fails the look counts as holding
    /// nothing: the append will tell).
    fn known(&mut self, path: &str) -> &mut Known {
        if !self.known.contains_key(path) {
            let known = look(path).unwrap_or_else(|error| {
                self.counts.card_errors += 1;
                log::warn!("card :: {path}: {error}");
                Known::default()
            });
            self.known.insert(path.to_string(), known);
        }
        self.known.get_mut(path).expect("inserted above")
    }

    /// Every gathered file to the card: its directory made once, a `\n` first after a torn last line, then open for
    /// append, write, close.
    fn flush(&mut self) {
        for (path, mut bytes) in std::mem::take(&mut self.pending) {
            if let Some((directory, _)) = path.rsplit_once('/')
                && !self.made.contains(directory)
            {
                match std::fs::create_dir_all(directory) {
                    Ok(()) => {
                        self.made.insert(directory.to_string());
                    }
                    Err(error) => {
                        self.card_error(&path, &error);
                        continue;
                    }
                }
            }
            let torn = self.known.get(&path).is_some_and(|known| known.torn);
            if torn {
                bytes.insert(0, b'\n');
            }
            let written = OpenOptions::new()
                .create(true)
                .append(true)
                .open(&path)
                .and_then(|mut file| file.write_all(&bytes));
            match written {
                Ok(()) => {
                    if let Some(known) = self.known.get_mut(&path) {
                        known.torn = false;
                    }
                }
                Err(error) => {
                    // looked at again before its next line: how much of this landed is unknown
                    self.known.remove(&path);
                    self.card_error(&path, &error);
                }
            }
        }
    }

    /// One failed card call, counted and logged.
    fn card_error(&mut self, path: &str, error: &std::io::Error) {
        self.counts.card_errors += 1;
        log::warn!("card :: {path}: {error}");
    }
}

/// What `path` holds: its end read ([`TAIL_BYTES`]); nothing for a file not there yet.
fn look(path: &str) -> std::io::Result<Known> {
    let mut file = match std::fs::File::open(path) {
        Ok(file) => file,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(Known::default()),
        Err(error) => return Err(error),
    };
    let size = file.metadata()?.len();
    file.seek(SeekFrom::Start(size.saturating_sub(TAIL_BYTES as u64)))?;
    let mut tail = Vec::with_capacity(TAIL_BYTES);
    file.read_to_end(&mut tail)?;
    // the last whole line: the tail up to its last `\n`, from the `\n` before
    let last = tail.iter().rposition(|&byte| byte == b'\n').map(|end| {
        let start = tail[..end]
            .iter()
            .rposition(|&byte| byte == b'\n')
            .map_or(0, |at| at + 1);
        &tail[start..end]
    });
    let uptime = last.and_then(recorder_wire::unwrap).and_then(|wrapper| {
        let digits = wrapper.payload.iter().take_while(|byte| byte.is_ascii_digit()).count();
        std::str::from_utf8(&wrapper.payload[..digits]).ok()?.parse().ok()
    });
    Ok(Known {
        held: last.is_some(),
        torn: tail.last().is_some_and(|&byte| byte != b'\n'),
        last: uptime.zip(last.map(<[u8]>::to_vec)),
        last_session: last.and_then(row_session),
    })
}

/// The session a session.csv row names (`{OPEN};<uptime>;<boot>;<session>;...`), None for the header or a line
/// that does not verify.
fn row_session(line: &[u8]) -> Option<String> {
    let payload = recorder_wire::verify(Some(SESSION_INDEX.as_bytes()), line)?;
    let mut cells = payload.split(|&byte| byte == b';');
    if !cells.next()?.first().is_some_and(u8::is_ascii_digit) {
        return None; // the header: its first cell is a column name
    }
    let session = cells.nth(1)?;
    Some(String::from_utf8_lossy(session).into_owned())
}

/// The card mounted at [`ROOT`] and read back: the mount, kept alive (dropping it unmounts), or why it failed. Each
/// try takes the SPI host and its pins afresh, so a failed one can be tried again; `trace` logs the card's init.
fn mount(trace: bool) -> anyhow::Result<Box<dyn std::any::Any>> {
    let [sck, mosi, miso, cs] = CARD_PINS;
    // SAFETY: SPI2 and these GPIOs are the card's alone; a failed try dropped what it had taken.
    let (spi2, sck, mosi, miso, cs) = unsafe {
        (
            SPI2::steal(),
            AnyIOPin::steal(sck as u8),
            AnyIOPin::steal(mosi as u8),
            AnyIOPin::steal(miso as u8),
            AnyIOPin::steal(cs as u8),
        )
    };
    let spi = SpiDriver::new(
        spi2,
        sck,
        mosi,
        Some(miso),
        &DriverConfig::default().dma(Dma::Auto(4096)),
    )
    .map_err(|error| anyhow::anyhow!("the SPI host: {error}"))?;
    let levels = |raised: bool| {
        for (tag, level) in TRACE_TAGS {
            // SAFETY: a static NUL-terminated tag; INFO is the console's default level (sdkconfig.defaults)
            unsafe { esp_log_level_set(tag.as_ptr(), if raised { level } else { esp_log_level_t_ESP_LOG_INFO }) };
        }
    };
    if trace {
        log::info!("receiver :: tracing the card's init on SPI2");
        levels(true);
    }
    let card = SdSpiHostDriver::new(
        spi,
        Some(cs),
        AnyIOPin::none(),
        AnyIOPin::none(),
        AnyIOPin::none(),
        None,
    )
    .map_err(|error| anyhow::anyhow!("the SD host on SPI: {error}"))
    .and_then(|host| {
        SdCardDriver::new_spi(host, &SdCardConfiguration::new())
            .map_err(|error| anyhow::anyhow!("the card's init (no answer: wiring, power, pull-ups, a card?): {error}"))
    });
    if trace {
        levels(false);
    }
    let card = card?;
    let info = card.card();
    // SAFETY: an SD card's init decodes its CID into `cid` (the union's other side is an MMC's raw CID)
    let name = unsafe { info.__bindgen_anon_1.cid.name };
    let name: String = name
        .iter()
        .take_while(|&&byte| byte != 0)
        .map(|&byte| char::from(byte))
        .collect();
    let bytes = info.csd.capacity as u64 * info.csd.sector_size as u64;
    log::info!(
        "card :: {name}, {:.1} GiB ({} sectors of {} B), SPI at {} kHz, OCR {:#010x}",
        bytes as f64 / f64::from(1u32 << 30),
        info.csd.capacity,
        info.csd.sector_size,
        info.real_freq_khz,
        info.ocr
    );
    let fatfs = Fatfs::new_sdcard(0, card).map_err(|error| anyhow::anyhow!("FAT on the card: {error}"))?;
    let mounted = MountedFatfs::mount(fatfs, ROOT, 8).map_err(|error| anyhow::anyhow!("the mount: {error}"))?;
    read_back().map_err(|error| anyhow::anyhow!("reading the card back: {error}"))?;
    Ok(Box::new(mounted))
}

/// The mounted card read back before any write: the volume's size and free space, and the root's entries.
fn read_back() -> anyhow::Result<()> {
    let (mut total, mut free) = (0u64, 0u64);
    let root = CString::new(ROOT)?;
    // SAFETY: a NUL-terminated mount point and two u64s to fill
    esp!(unsafe { esp_vfs_fat_info(root.as_ptr(), &mut total, &mut free) })?;
    let mut listed = Vec::new();
    let mut count = 0usize;
    for entry in std::fs::read_dir(ROOT)? {
        let entry = entry?;
        count += 1;
        if listed.len() < LISTED {
            let metadata = entry.metadata()?;
            let name = entry.file_name().to_string_lossy().into_owned();
            listed.push(if metadata.is_dir() {
                format!("{name}/")
            } else {
                format!("{name} ({} B)", metadata.len())
            });
        }
    }
    log::info!(
        "card :: read back: {} of {} MiB free; the root's {count} entries: {}{}",
        free >> 20,
        total >> 20,
        listed.join(", "),
        if count > listed.len() { ", ..." } else { "" }
    );
    Ok(())
}

/// The USB console's commands, on their own thread so the link is read meanwhile; the console has no driver, so its
/// reads never wait and the thread looks again every [`CONSOLE_POLL`].
///
/// - `files [dir]`: every file under `/sd` (or `/sd/<dir>`), one `file <path>;<size>;<crc32>` line each (zlib's
///   CRC-32 in hex, as Python's `binascii.crc32`), then `files :: <count> files, <bytes> bytes`. A file being filed
///   is read up to where it ends at that moment, so its line describes that prefix.
fn spawn_console() -> anyhow::Result<()> {
    // below the link's loop ([`LINK_PRIORITY`]): a walk over a session's files must never hold the UART reader
    ThreadSpawnConfiguration {
        name: Some(c"console"),
        priority: CONSOLE_PRIORITY,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(CONSOLE_STACK).spawn(|| {
        let mut held = Vec::new();
        let mut chunk = [0u8; 64];
        loop {
            match std::io::stdin().read(&mut chunk) {
                Ok(count) if count > 0 => held.extend_from_slice(&chunk[..count]),
                _ => std::thread::sleep(CONSOLE_POLL),
            }
            while let Some(end) = held.iter().position(|&byte| byte == b'\n') {
                let line: Vec<u8> = held.drain(..=end).collect();
                let line = String::from_utf8_lossy(&line);
                let mut words = line.split_whitespace();
                let command = words.next();
                let arguments: Vec<&str> = words.collect();
                match (command, arguments.as_slice()) {
                    (Some("files"), []) => files(None),
                    (Some("files"), [directory]) => files(Some(directory)),
                    (Some("read"), [path, offset, length]) => read(path, offset, length),
                    (None, _) => {}
                    (Some(other), _) => {
                        println!("err {other}: the commands are `files [dir]` and `read <path> <offset> <length>`")
                    }
                }
            }
        }
    });
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}

/// The console's `files [dir]` (see [`spawn_console`]).
fn files(directory: Option<&str>) {
    let start = match directory {
        Some(directory) => format!("{ROOT}/{}", directory.trim_matches('/')),
        None => ROOT.to_string(),
    };
    let mut pending = vec![std::path::PathBuf::from(&start)];
    let (mut count, mut total) = (0u64, 0u64);
    let mut buffer = vec![0u8; 4096];
    while let Some(directory) = pending.pop() {
        let entries = match std::fs::read_dir(&directory) {
            Ok(entries) => entries,
            Err(error) => {
                println!("err {}: {error}", directory.display());
                continue;
            }
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if entry.file_type().is_ok_and(|kind| kind.is_dir()) {
                pending.push(path);
                continue;
            }
            let summed = std::fs::File::open(&path).and_then(|mut file| {
                let (mut size, mut crc) = (0u64, 0u32);
                loop {
                    let read = file.read(&mut buffer)?;
                    if read == 0 {
                        return Ok((size, crc));
                    }
                    size += read as u64;
                    crc = recorder_wire::crc32(crc, &buffer[..read]);
                }
            });
            let shown = path.strip_prefix(ROOT).unwrap_or(&path).display().to_string();
            match summed {
                Ok((size, crc)) => {
                    println!("file {shown};{size};{crc:08x}");
                    count += 1;
                    total += size;
                }
                Err(error) => println!("err {shown}: {error}"),
            }
        }
    }
    println!("files :: {count} files, {total} bytes");
}

/// The console's `read <path> <offset> <length>`: that part of a file under `/sd` (at most 4 KB), one line
/// `read <offset> <length> <hex>`: the console's text is not byte-exact (it turns `\n` into `\r\n`), hex is, so a
/// comparison with the P4's card finds the line where two copies part.
fn read(path: &str, offset: &str, length: &str) {
    let (Ok(offset), Ok(length)) = (offset.parse::<u64>(), length.parse::<usize>()) else {
        println!("err read: <offset> and <length> are numbers");
        return;
    };
    let path = format!("{ROOT}/{}", path.trim_start_matches('/'));
    let mut bytes = vec![0u8; length.min(4096)];
    let got = std::fs::File::open(&path).and_then(|mut file| {
        file.seek(SeekFrom::Start(offset))?;
        let mut got = 0;
        while got < bytes.len() {
            let count = file.read(&mut bytes[got..])?;
            if count == 0 {
                break;
            }
            got += count;
        }
        Ok(got)
    });
    match got {
        Ok(got) => {
            let hex: String = bytes[..got].iter().map(|byte| format!("{byte:02x}")).collect();
            println!("read {offset} {got} {hex}");
        }
        Err(error) => println!("err {path}: {error}"),
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    // SAFETY: the running task's own priority.
    unsafe { esp_idf_svc::sys::vTaskPrioritySet(std::ptr::null_mut(), LINK_PRIORITY) };
    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;
    let mut card = None;
    let mut mounted_at: Option<Instant> = None;
    let try_mount = |card: &mut Option<Box<dyn std::any::Any>>, mounted_at: &mut Option<Instant>| {
        let first = mounted_at.is_none();
        *mounted_at = Some(Instant::now());
        match mount(first) {
            Ok(mount) => {
                log::info!("receiver :: card mounted at {ROOT}");
                *card = Some(mount);
            }
            Err(error) => log::warn!(
                "receiver :: no card: {error}; trying again in {} s",
                MOUNT_EVERY.as_secs()
            ),
        }
    };
    try_mount(&mut card, &mut mounted_at);
    // the remote beacon (crate::remote): off until a report turns it on; the last one on the card first
    let mut remote = remote::RemoteBeacon::new(
        peripherals.modem,
        EspSystemEventLoop::take()?,
        EspDefaultNvsPartition::take().ok(),
    );
    if card.is_some()
        && let Some(report) = remote::last_report(ROOT)
    {
        log::info!("beacon :: the glider's last report read back from the card");
        remote.report(report, None);
    }
    let link = UartRxDriver::new(
        peripherals.uart1,
        pins.gpio17,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &UartConfig::default().baudrate(Hertz(BAUD)).rx_fifo_size(RX_BYTES),
    )?;
    log::info!("receiver :: the P4's lines on GPIO17 at {BAUD} baud");
    spawn_console()?;
    let mut filer = Filer::new();
    filer.mounted = card.is_some();
    let mut held: Vec<u8> = Vec::new();
    let mut buffer = vec![0u8; 4096];
    let wait = TickType::from(READ_WAIT).ticks();
    // the first status line FIRST_STATUS after boot: what came back from the card shows before the link's first rows
    let mut flushed = Instant::now();
    let mut reported = flushed.checked_sub(STATUS_EVERY - FIRST_STATUS).unwrap_or(flushed);
    loop {
        let count = link.read(&mut buffer, wait).unwrap_or(0);
        filer.counts.bytes += count as u64;
        held.extend_from_slice(&buffer[..count]);
        let rest = recorder_wire::split(&held, |routing, payload| filer.line(routing, payload)).len();
        held.drain(..held.len() - rest);
        if let Some(report) = filer.report.take() {
            remote.report(report, Some(Instant::now()));
        }
        remote.step();
        if held.len() > LINE_LIMIT {
            held.clear();
            filer.counts.overlong += 1;
        }
        if flushed.elapsed() >= FLUSH_EVERY {
            flushed = Instant::now();
            if card.is_some() {
                filer.flush();
            } else {
                filer.counts.unfiled += filer
                    .pending
                    .values()
                    .map(|bytes| bytes.iter().filter(|&&byte| byte == b'\n').count() as u64)
                    .sum::<u64>();
                filer.pending.clear();
                if mounted_at.is_none_or(|at| at.elapsed() >= MOUNT_EVERY) {
                    try_mount(&mut card, &mut mounted_at);
                    filer.mounted = card.is_some();
                }
            }
        }
        if reported.elapsed() >= STATUS_EVERY {
            reported = Instant::now();
            let counts = &filer.counts;
            log::info!(
                "receiver :: {} bytes, {} lines good, {} damaged, {} overlong, {} repeated, card {} (errors {}, unfiled {}), \
                 session {} ({} seen), beacon {}",
                counts.bytes,
                counts.good,
                counts.damaged,
                counts.overlong,
                counts.repeated,
                if card.is_some() { "mounted" } else { "NONE" },
                counts.card_errors,
                counts.unfiled,
                filer.session,
                filer.sessions.len(),
                remote.state()
            );
        }
    }
}
