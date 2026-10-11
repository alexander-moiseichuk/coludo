//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The remote flight recorder (M20, doc/specs/avionics-port.md M19/M20) on an ESP32-C6 SuperMini: the P4's
//! uart_saver sends its recorder's wire lines, `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`, and this board files them
//! on its own SD card, so the data survives the P4 and the P4's card. It replaces the Luckfox's recorderd.
//!
//! - The link: UART1 RX on GPIO17 (the board's RX pin) at 921600 baud, 8N1; TX + GND only from the P4.
//! - The card: SPI2, SCK GPIO19, MOSI GPIO20, MISO GPIO14, CS GPIO18, FAT at `/sd`.
//! - The lines are split as recorderd split them (crates/recorder_wire `split`), and filed in the P4 card's own
//!   layout (spec §3.10), so the same tools read both cards: a routed line to `<session>/recordings/<routing>` (the
//!   session is the routing up to its first `_`), a `session.csv` line to the session's `session.csv` and the card's
//!   `sessions.csv`, any other line to the session's `recorder.log` (the session the last routed line named). Each
//!   line is filed as it came, the payload after its routing; its wrapper is checked (recorder_wire `verify`) and
//!   counted, a damaged line filed all the same for the tools to judge, as recorderd did.
//! - Lines gather in memory and go to the card once a second: per file, open for append, write, close (FatFs syncs
//!   at close), so a power cut costs at most the last second.
//! - Every 5 s one status line on the USB console: bytes and lines in, good and damaged, sessions, card errors.
//! - A card that does not answer is no stop: the link is read and counted all the same (lines then go unfiled,
//!   counted), and the mount is tried again every 10 s. The first try traces the card's init on the console, each SPI
//!   command and the card's answer.
//! - A mounted card is read back before anything is written: its name, size and SPI clock, the volume's free space
//!   and the root's entries. A card that cannot be read is not written to (the mount counts as failed).

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::{CStr, CString};
use std::fs::OpenOptions;
use std::io::Write as _;
use std::time::{Duration, Instant};

use esp_idf_svc::fs::fatfs::Fatfs;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::sd::spi::SdSpiHostDriver;
use esp_idf_svc::hal::sd::{SdCardConfiguration, SdCardDriver};
use esp_idf_svc::hal::spi::SPI2;
use esp_idf_svc::hal::spi::config::DriverConfig;
use esp_idf_svc::hal::spi::{Dma, SpiDriver};
use esp_idf_svc::hal::uart::{UartConfig, UartRxDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::io::vfs::MountedFatfs;
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

/// The routing of the session index's lines.
const SESSION_INDEX: &[u8] = b"session.csv";

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
    /// Card writes that failed.
    card_errors: u32,
    /// Lines read while no card was mounted, so filed nowhere.
    unfiled: u64,
}

/// The lines gathered since the last flush, by file, and what the receiver counted.
struct Filer {
    /// The bytes to append, by path.
    pending: BTreeMap<String, Vec<u8>>,
    /// The directories made this boot.
    made: BTreeSet<String>,
    /// The session the last routed line named: the unrouted lines' session.
    session: String,
    /// The sessions seen this boot.
    sessions: BTreeSet<String>,
    /// The counts.
    counts: Counts,
}

impl Filer {
    /// Nothing gathered.
    fn new() -> Filer {
        Filer {
            pending: BTreeMap::new(),
            made: BTreeSet::new(),
            session: "unknown".to_string(),
            sessions: BTreeSet::new(),
            counts: Counts::default(),
        }
    }

    /// One demuxed line: `routing` (None for recorder.log) and `payload` (its `\n` included), filed by session.
    fn line(&mut self, routing: Option<&[u8]>, payload: &[u8]) {
        let body = payload.strip_suffix(b"\n").unwrap_or(payload);
        if recorder_wire::verify(routing, body).is_some() {
            self.counts.good += 1;
        } else {
            self.counts.damaged += 1;
        }
        match routing {
            Some(SESSION_INDEX) => {
                let path = format!("{ROOT}/{}/recordings/session.csv", self.session);
                self.append(path, payload);
                self.append(format!("{ROOT}/sessions.csv"), payload);
            }
            Some(routing) => {
                let name = String::from_utf8_lossy(routing).replace(['/', '\\'], "_");
                if let Some((session, _)) = name.split_once('_')
                    && !session.is_empty()
                    && session != self.session
                {
                    self.session = session.to_string();
                    self.sessions.insert(self.session.clone());
                }
                let path = format!("{ROOT}/{}/recordings/{name}", self.session);
                self.append(path, payload);
            }
            None => {
                let path = format!("{ROOT}/{}/recordings/recorder.log", self.session);
                self.append(path, payload);
            }
        }
    }

    /// `bytes` gathered for `path`.
    fn append(&mut self, path: String, bytes: &[u8]) {
        self.pending.entry(path).or_default().extend_from_slice(bytes);
    }

    /// Every gathered file to the card: its directory made once, then open for append, write, close.
    fn flush(&mut self) {
        for (path, bytes) in std::mem::take(&mut self.pending) {
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
            let written = OpenOptions::new()
                .create(true)
                .append(true)
                .open(&path)
                .and_then(|mut file| file.write_all(&bytes));
            if let Err(error) = written {
                self.card_error(&path, &error);
            }
        }
    }

    /// One failed card call, counted and logged.
    fn card_error(&mut self, path: &str, error: &std::io::Error) {
        self.counts.card_errors += 1;
        log::warn!("card :: {path}: {error}");
    }
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

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
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
    let link = UartRxDriver::new(
        peripherals.uart1,
        pins.gpio17,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &UartConfig::default().baudrate(Hertz(BAUD)).rx_fifo_size(RX_BYTES),
    )?;
    log::info!("receiver :: the P4's lines on GPIO17 at {BAUD} baud");
    let mut filer = Filer::new();
    let mut held: Vec<u8> = Vec::new();
    let mut buffer = vec![0u8; 4096];
    let wait = TickType::from(READ_WAIT).ticks();
    let (mut flushed, mut reported) = (Instant::now(), Instant::now());
    loop {
        let count = link.read(&mut buffer, wait).unwrap_or(0);
        filer.counts.bytes += count as u64;
        held.extend_from_slice(&buffer[..count]);
        let rest = recorder_wire::split(&held, |routing, payload| filer.line(routing, payload)).len();
        held.drain(..held.len() - rest);
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
                }
            }
        }
        if reported.elapsed() >= STATUS_EVERY {
            reported = Instant::now();
            let counts = &filer.counts;
            log::info!(
                "receiver :: {} bytes, {} lines good, {} damaged, {} overlong, card {} (errors {}, unfiled {}), session {} \
                 ({} seen)",
                counts.bytes,
                counts.good,
                counts.damaged,
                counts.overlong,
                if card.is_some() { "mounted" } else { "NONE" },
                counts.card_errors,
                counts.unfiled,
                filer.session,
                filer.sessions.len()
            );
        }
    }
}
