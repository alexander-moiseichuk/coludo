//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The GNSS receiver on its UART (M11; plan B gnss.py, its logic in crates/gnss). One thread (core 0, low priority,
//! PSRAM stack) owns the UART: it writes the module's configuration (the driver's command bytes with plan B's gaps),
//! then reads lines forever, hands each to the engine ([`gnss::Gnss::line`]) and publishes what comes back:
//! `position` (1e-7 degree), `altitude`, `elevation`, `speed` and `course` on the databoard (the GNSS's `elevation`
//! is the stage machine's own GNSS slot, by its name), and the four streams `<name>.csv`, `<name>_gga.csv`,
//! `<name>_sky.csv`, `<name>_events.csv`. Once a second the engine's tick runs: the outage sweep, the sky window for
//! the stage now (the diagnostics on the pad, off from BOOSTING to DONE: the switch frame written whole), and the sky
//! row when a burst has ended.
//!
//! The executor sees it through atomics (`inspect`, `probe`, `health`), so the thread holds no lock and makes its
//! text only for its own event rows and log lines.
//!
//! The NEO-M8N (`neom8n`, M16) delivers UBX instead of NMEA (gnss::drivers::neom8n): the thread finds its frames in
//! the byte stream (gnss::ubx `Frames`) and hands each to the engine ([`gnss::Gnss::message`]), which fills the same
//! update a line does. Its bring-up ([`bring_up_m8`]) finds the module's rate by listening (it keeps its port
//! settings over a P4 reboot), names it (UBX-MON-VER), raises the port to the driver's baud, and falls back to UBX at
//! the power-on rate, then to NMEA, when the module does not follow; each frame's ACK or NAK is logged, a refusal and a
//! fallback are events.
//!
//! The pad drift calibration (plan B tasks/gnss_calib.py, [`Calibration`]) is the supervisor's duty: it reads the
//! fused speed and course, whoever provides them, and publishes the frozen drift through atomics for `inspect` and
//! the flight loop (M13).

use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::time::{Duration, Instant};

use control::drift::DriftCalibrator;
use control::navigation::Position;
use databoard::{Parameter, Writer};
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::uart::{Uart, UartConfig, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use gnss::{Antenna, Gnss, SkySwitch, Update};
use recorder::{Cell, Telemetry};
use sequencer::Stage;

use crate::devices::{GnssCalibMember, GnssMember, Topology};
use crate::status::FLIGHT;
use firmware::threads::{self, Spec, Stack};
use firmware::time::uptime_us;

/// The module on the UART, as the entry's driver names it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Module {
    /// The NEO-6M (`neo6mv2`): UBX configuration, NMEA output.
    Neo6,
    /// The ATGM336H (`atgm336h`): CASIC (PCAS) configuration.
    Atgm,
    /// The NEO-M8N (`neom8n`, M16): UBX configuration, UBX output.
    M8,
}

impl Module {
    /// The module the entry's implementation names, None for one that is no GNSS module.
    pub fn of(implementation: &str) -> Option<Module> {
        match implementation {
            "neo6mv2" => Some(Module::Neo6),
            "atgm336h" => Some(Module::Atgm),
            "neom8n" => Some(Module::M8),
            _ => None,
        }
    }

    /// What the module delivers, for the probe's words.
    fn delivery(self) -> &'static str {
        match self {
            Module::M8 => "ubx",
            Module::Neo6 | Module::Atgm => "nmea",
        }
    }
}

/// The thread's stack (PSRAM): the UART calls, the engine's parse and the event texts.
const STACK_SIZE: usize = 8 << 10;

/// The thread's priority on core 0: below the ring tasks and the video writer.
const PRIORITY: u8 = 3;

/// How long one UART read waits for bytes, ms: short enough for the 1 s tick to keep its beat.
const READ_WAIT_MS: u64 = 50;

/// The longest NMEA line kept: a sentence is at most 82 characters; a longer run without a newline is noise, dropped.
const LINE_BYTES: usize = 128;

/// The engine's tick period.
const TICK: Duration = Duration::from_millis(1000);

/// The module's fix rate at DONE (M11b): the beacon shows one fix, and a slow module draws less.
const DONE_HZ: i32 = 1;

/// NMEA lines (UBX frames, on the NEO-M8N) read since boot (plan B's `_lines`: the probe's liveness counter).
static LINES: AtomicU32 = AtomicU32::new(0);
/// Whether the last RMC had a valid fix.
static FIX: AtomicBool = AtomicBool::new(false);
/// The GGA's fix quality, satellites used and HDOP in hundredths.
static QUALITY: [AtomicI32; 3] = [const { AtomicI32::new(0) }; 3];
/// The latest sky row's eight columns (in view, used, mode, the four C/N0, the antenna code).
static SKY: [AtomicI32; 8] = [const { AtomicI32::new(0) }; 8];
/// The GNSS's entry name, its UART's id and its module, once the thread runs.
static RUNNING: OnceLock<(&'static str, u8, Module)> = OnceLock::new();

/// Starts the GNSS thread on `uart` (UART2) for `member`, its quantities found in `topology`.
///
/// Errors: the UART would not open, or the thread would not spawn.
pub fn start(member: GnssMember, uart: impl Uart + 'static, topology: &'static Topology) -> anyhow::Result<()> {
    let config = UartConfig::default().baudrate(Hertz(member.uart.baud));
    // SAFETY: the board config gives these GPIOs to the GNSS UART alone; the driver lives for the program.
    let (tx, rx) = unsafe { (AnyIOPin::steal(member.uart.tx), AnyIOPin::steal(member.uart.rx)) };
    let driver = UartDriver::new(
        uart,
        tx,
        rx,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &config,
    )?;
    threads::spawn(
        Spec {
            name: c"gnss",
            stack_size: STACK_SIZE,
            stack: Stack::Psram,
            priority: PRIORITY,
            core: Core::Core0,
        },
        move || run(member, driver, topology),
    )?;
    Ok(())
}

/// The channels the GNSS publishes, as the topology holds them.
struct Channels {
    /// `position`, 1e-7 degree.
    position: Option<Writer<'static, [i32; 2]>>,
    /// `altitude`, m MSL.
    altitude: Option<Writer<'static, f32>>,
    /// `elevation`, m above the GNSS ground zero.
    elevation: Option<Writer<'static, f32>>,
    /// `speed`, m/s.
    speed: Option<Writer<'static, f32>>,
    /// `course`, degrees.
    course: Option<Writer<'static, f32>>,
}

/// The four streams, with a recorder.
struct Streams {
    /// `<name>.csv`: the fix.
    fix: Option<Telemetry<'static>>,
    /// `<name>_gga.csv`.
    gga: Option<Telemetry<'static>>,
    /// `<name>_sky.csv`.
    sky: Option<Telemetry<'static>>,
    /// `<name>_events.csv`, durable.
    events: Option<Telemetry<'static>>,
}

/// The thread: the configuration, then the lines and the tick forever.
fn run(member: GnssMember, driver: UartDriver<'static>, topology: &'static Topology) {
    let name = member.name;
    let _ = RUNNING.set((name, member.uart.id, member.module));
    let scalar = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
    let mut channels = Channels {
        position: topology
            .position("position")
            .and_then(|parameter| parameter.writer(name)),
        altitude: scalar("altitude"),
        elevation: scalar("elevation"),
        speed: scalar("speed"),
        course: scalar("course"),
    };
    let stream = |key: String| crate::recorder::telemetry(&key);
    let mut streams = Streams {
        fix: stream(name.to_string()),
        gga: stream(format!("{name}_gga")),
        sky: stream(format!("{name}_sky")),
        events: stream(format!("{name}_events")),
    };
    let (mut switch, delivery) = configure(&driver, &member, member.hz, &mut streams.events);
    // DONE (M11b): the module at 1 Hz, the beacon's cadence, until a reset
    let mut slowed = false;
    log::info!(
        "{name} :: {:?} configured at {} Hz on uart:{} ({} baud, {})",
        member.module,
        member.hz,
        member.uart.id,
        delivery.baud(member.uart.baud),
        delivery.name()
    );
    let mut engine = Gnss::new();
    let mut reader = Reader::new(delivery);
    // the first bytes read and their count, for the silence check below
    let mut heard = Heard::default();
    let configured = Instant::now();
    let mut buffer = [0u8; 256];
    let wait = TickType::new_millis(READ_WAIT_MS).ticks();
    let mut next_tick = Instant::now() + TICK;
    loop {
        let count = driver.read(&mut buffer, wait).unwrap_or(0);
        heard.note(&buffer[..count]);
        for &byte in &buffer[..count] {
            if let Some((update, answer)) = reader.feed(byte, &mut engine) {
                publish(name, &update, &mut channels, &mut streams);
                if let Some(answer) = answer {
                    acknowledge(name, answer, &mut streams.events);
                }
            }
        }
        publish_state(&engine);
        let now = Instant::now();
        if now >= next_tick {
            heard.check(name, &engine, &reader, configured.elapsed(), &mut streams.events);
            next_tick = now + TICK;
            let stage = FLIGHT.load().stage;
            if !slowed && stage == Stage::Done && member.hz > DONE_HZ {
                let delivery;
                (switch, delivery) = configure(&driver, &member, DONE_HZ, &mut streams.events);
                reader = Reader::new(delivery);
                slowed = true;
                log::info!("{name} :: DONE: the module at {DONE_HZ} Hz");
            }
            let tick = engine.tick(uptime_us(), stage);
            if let Some(frame) = tick.write {
                let _ = driver.write(switch.frame(frame));
            }
            if let Some(row) = tick.sky_row {
                push(&mut streams.sky, &row.columns().map(Cell::I32));
            }
            // the sky as plan B's inspect computes it, live: the bursts' sentences so far
            for (published, value) in SKY.iter().zip(engine.sky_row().columns()) {
                published.store(value, Ordering::Relaxed);
            }
            for event in &tick.events {
                note(name, &mut streams.events, &event.to_string());
            }
        }
    }
}

/// How the thread reads its module: NMEA lines, or UBX frames (the NEO-M8N).
enum Reader {
    /// The line so far.
    Lines(Vec<u8>),
    /// The frame parser.
    Frames(gnss::ubx::Frames),
}

impl Reader {
    /// The reader for `delivery`.
    fn new(delivery: Delivery) -> Reader {
        match delivery {
            Delivery::Ubx(_) => Reader::Frames(gnss::ubx::Frames::new()),
            Delivery::Nmea => Reader::Lines(Vec::with_capacity(LINE_BYTES)),
        }
    }

    /// One byte read: what the line or the frame it completes gave the engine, and a frame's ACK.
    fn feed(&mut self, byte: u8, engine: &mut Gnss) -> Option<(Update, Option<gnss::ubx::Answer>)> {
        match self {
            Reader::Lines(line) => {
                if line.len() == LINE_BYTES {
                    line.clear(); // noise: no newline within a sentence's length
                }
                line.push(byte);
                // the line with its newline, as plan B's readline() gave it: a bare newline is a line too
                if byte != b'\n' {
                    return None;
                }
                let update = engine.line(line, uptime_us());
                line.clear();
                Some((update, None))
            }
            Reader::Frames(frames) => {
                let message = frames.push(byte)?;
                Some(engine.message(&message, uptime_us()))
            }
        }
    }
}

/// What the UART delivered before the first line or frame: the bytes' count and the first of them, so a module that
/// answers at another rate, or not at all, says so once (the NEO-M8N's port switch is the one place it can).
#[derive(Default)]
struct Heard {
    /// Bytes read so far.
    bytes: u32,
    /// The first bytes read.
    first: Vec<u8>,
    /// The silence was reported.
    reported: bool,
}

impl Heard {
    /// The longest sample kept.
    const SAMPLE: usize = 24;
    /// How long after the configuration the first line or frame may take.
    const GRACE: Duration = Duration::from_secs(3);

    /// Bytes read.
    fn note(&mut self, bytes: &[u8]) {
        self.bytes = self.bytes.saturating_add(bytes.len() as u32);
        let room = Self::SAMPLE.saturating_sub(self.first.len());
        self.first.extend_from_slice(&bytes[..bytes.len().min(room)]);
    }

    /// Once the grace is over with nothing decoded: one event, with the bytes' count, the damaged frames and the
    /// first bytes in hex (nothing at all: the wire; text at a UBX module's rate: the port switch did not take).
    fn check(
        &mut self,
        name: &str,
        engine: &Gnss,
        reader: &Reader,
        since: Duration,
        events: &mut Option<Telemetry<'static>>,
    ) {
        if self.reported || engine.lines() > 0 || since < Self::GRACE {
            return;
        }
        self.reported = true;
        let damaged = match reader {
            Reader::Frames(frames) => frames.rejected(),
            Reader::Lines(_) => 0,
        };
        let sample: Vec<String> = self.first.iter().map(|byte| format!("{byte:02x}")).collect();
        note(
            name,
            events,
            &format!(
                "nothing decoded {} s after the configuration: {} bytes read, {damaged} damaged frames, first {}",
                since.as_secs(),
                self.bytes,
                sample.join(" ")
            ),
        );
    }
}

/// What the module delivers once configured.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Delivery {
    /// NMEA lines at the config's baud.
    Nmea,
    /// UBX frames at this baud: the NEO-M8N driver's raised one, or the power-on one when the switch did not take.
    Ubx(u32),
}

impl Delivery {
    /// The UART's rate for it: the config's `baud`, or the UBX one.
    fn baud(self, baud: u32) -> u32 {
        match self {
            Delivery::Nmea => baud,
            Delivery::Ubx(rate) => rate,
        }
    }

    /// Its word, for the log.
    fn name(self) -> &'static str {
        match self {
            Delivery::Nmea => "nmea",
            Delivery::Ubx(_) => "ubx",
        }
    }
}

/// Writes `member`'s module configuration at `hz` (plan B `_configure`): each command, then the driver's gap. The
/// NEO-M8N's bring-up is its own ([`bring_up_m8`]). Returns the sky switch, the two frames written whole at the flight
/// window's edges, and what the module delivers.
fn configure(
    driver: &UartDriver<'static>,
    member: &GnssMember,
    hz: i32,
    events: &mut Option<Telemetry<'static>>,
) -> (SkySwitch, Delivery) {
    let (commands, gap_ms, switch): (Vec<Vec<u8>>, u32, SkySwitch) = match member.module {
        Module::Neo6 => (
            gnss::drivers::neo6mv2::commands(hz).to_vec(),
            gnss::drivers::neo6mv2::SEND_GAP_MS,
            gnss::drivers::neo6mv2::sky_switch(hz),
        ),
        Module::Atgm => (
            gnss::drivers::atgm336h::commands(hz).to_vec(),
            gnss::drivers::atgm336h::SEND_GAP_MS,
            gnss::drivers::atgm336h::sky_switch(hz),
        ),
        Module::M8 => return bring_up_m8(driver, member, hz, events),
    };
    send(driver, &commands, gap_ms);
    (switch, Delivery::Nmea)
}

/// Each of `commands`, then `gap_ms`.
fn send(driver: &UartDriver<'static>, commands: &[Vec<u8>], gap_ms: u32) {
    for command in commands {
        let _ = driver.write(command);
        std::thread::sleep(Duration::from_millis(u64::from(gap_ms)));
    }
}

/// How long the bring-up listens for the module's answers.
const ANSWER_WAIT: Duration = Duration::from_millis(1200);

/// The fastest rate UBX runs at the power-on 9600 baud: a NAV-PVT is 100 bytes, 960 bytes a second the most.
const POWER_ON_UBX_HZ: i32 = 5;

/// The NEO-M8N's bring-up (M16). A u-blox ignores the valid frames that follow, for a second or so, a frame sent at
/// a rate it is not at (the taster's, 2026-10-12), so the rate is found by LISTENING first, never by sending at a
/// guess:
/// 1. at the raised rate, then at the power-on `baud`: whatever decodes there (UBX frames; NMEA lines whose checksum
///    holds) is the module's rate. A module an earlier boot raised streams NAV-PVT at the raised rate; a module just
///    powered talks NMEA at 9600;
/// 2. at that rate, a UBX-MON-VER poll: its reply proves the module hears the board and names it;
/// 3. a module at the power-on rate gets the port raised, then the setup and a poll at the raised rate; one already
///    raised gets the setup there. Any UBX frame heard after it is the binary delivery;
/// 4. else the port set to UBX at the power-on rate, the setup at most [`POWER_ON_UBX_HZ`]: binary at 9600;
/// 5. else NMEA at the power-on rate with the NEO-6M's u-blox configuration (an M8 takes it), an event saying so: the
///    GNSS keeps serving.
fn bring_up_m8(
    driver: &UartDriver<'static>,
    member: &GnssMember,
    hz: i32,
    events: &mut Option<Telemetry<'static>>,
) -> (SkySwitch, Delivery) {
    let name = member.name;
    let baud = member.uart.baud;
    let raised = gnss::drivers::neom8n::BAUD;
    let poll = gnss::ubx::frame(gnss::ubx::MONITOR_CLASS, gnss::ubx::VERSION, &[]);
    let found = [raised, baud].into_iter().find(|rate| {
        let _ = driver.change_baudrate(Hertz(*rate));
        let _ = driver.clear_rx();
        let heard = listen(driver, ANSWER_WAIT);
        heard.frames > 0 || heard.sentences > 0
    });
    let at = found.unwrap_or(baud);
    match found {
        Some(rate) => log::info!("{name} :: the module talks at {rate} baud"),
        None => log::warn!("{name} :: the module is silent at {raised} and {baud} baud: configured at {baud} blind"),
    }
    let _ = driver.change_baudrate(Hertz(at));
    let _ = driver.clear_rx();
    let _ = driver.write(&poll);
    let heard = listen(driver, ANSWER_WAIT);
    answers(name, &heard, events);
    if !heard.identified {
        log::warn!("{name} :: no MON-VER answer at {at} baud: does the module hear the board?");
    }
    if at != raised {
        raise_port(driver, at);
    }
    if setup(driver, name, hz, &poll, events) {
        return (gnss::drivers::neom8n::sky_switch(hz), Delivery::Ubx(raised));
    }
    log::warn!("{name} :: nothing at {raised} baud: the port did not switch rate; UBX at {baud} baud next");
    let _ = driver.change_baudrate(Hertz(baud));
    let _ = driver.clear_rx();
    let _ = driver.write(&gnss::drivers::neom8n::port(baud));
    std::thread::sleep(Duration::from_millis(PORT_SETTLE_MS));
    let slow = hz.min(POWER_ON_UBX_HZ);
    if setup(driver, name, slow, &poll, events) {
        note(
            name,
            events,
            &format!("UBX at {baud} baud: the module took UBX but not the rate switch; at most {slow} Hz there"),
        );
        return (gnss::drivers::neom8n::sky_switch(slow), Delivery::Ubx(baud));
    }
    let _ = driver.clear_rx();
    note(
        name,
        events,
        &format!(
            "the module did not take the switch to UBX: NMEA at {baud} baud kept, the NEO-6M's configuration sent"
        ),
    );
    send(
        driver,
        &gnss::drivers::neo6mv2::commands(hz),
        gnss::drivers::neo6mv2::SEND_GAP_MS,
    );
    (gnss::drivers::neo6mv2::sky_switch(hz), Delivery::Nmea)
}

/// The setup frames at `hz` and a MON-VER `poll` at the UART's rate now, and the answers heard (logged): whether any
/// UBX frame came, which is the module delivering UBX at this rate.
fn setup(
    driver: &UartDriver<'static>,
    name: &str,
    hz: i32,
    poll: &[u8],
    events: &mut Option<Telemetry<'static>>,
) -> bool {
    send(
        driver,
        &gnss::drivers::neom8n::commands(hz),
        gnss::drivers::neom8n::SEND_GAP_MS,
    );
    let _ = driver.write(poll);
    let heard = listen(driver, ANSWER_WAIT);
    answers(name, &heard, events);
    heard.frames > 0
}

/// What the bring-up heard in one listen.
#[derive(Default)]
struct Listened {
    /// Bytes read.
    bytes: u32,
    /// NMEA sentences among them whose checksum holds.
    sentences: u32,
    /// UBX frames found.
    frames: u32,
    /// Their answers: ACKs and NAKs.
    answers: Vec<gnss::ubx::Answer>,
    /// A MON-VER among them, decoded.
    version: Option<gnss::ubx::Version>,
    /// Whether a MON-VER came.
    identified: bool,
}

/// Everything the UART delivers for `wait`, its UBX frames found.
fn listen(driver: &UartDriver<'static>, wait: Duration) -> Listened {
    let mut heard = Listened::default();
    let mut frames = gnss::ubx::Frames::new();
    let mut line: Vec<u8> = Vec::with_capacity(LINE_BYTES);
    let mut buffer = [0u8; 256];
    let tick = TickType::new_millis(READ_WAIT_MS).ticks();
    let until = Instant::now() + wait;
    while Instant::now() < until {
        let count = driver.read(&mut buffer, tick).unwrap_or(0);
        heard.bytes = heard.bytes.saturating_add(count as u32);
        for &byte in &buffer[..count] {
            if line.len() == LINE_BYTES {
                line.clear();
            }
            line.push(byte);
            if byte == b'\n' {
                let sentence = std::str::from_utf8(&line).map(str::trim).unwrap_or_default();
                heard.sentences += u32::from(sentence.starts_with('$') && gnss::checksum_ok(sentence));
                line.clear();
            }
            let Some(message) = frames.push(byte) else {
                continue;
            };
            heard.frames += 1;
            match (message.class, message.id) {
                (gnss::ubx::ACKNOWLEDGEMENT_CLASS, id) => {
                    heard.answers.extend(gnss::ubx::Answer::decode(id, &message.payload));
                }
                (gnss::ubx::MONITOR_CLASS, gnss::ubx::VERSION) => {
                    heard.identified = true;
                    heard.version = gnss::ubx::Version::decode(&message.payload);
                }
                _ => {}
            }
        }
    }
    heard
}

/// Logs what a listen heard: each configuration frame's answer, and the module's MON-VER.
fn answers(name: &str, heard: &Listened, events: &mut Option<Telemetry<'static>>) {
    for answer in &heard.answers {
        acknowledge(name, *answer, events);
    }
    if let Some(version) = &heard.version {
        log::info!(
            "{name} :: MON-VER: software {}, hardware {}, {}",
            version.software,
            version.hardware,
            version.extensions.join(", ")
        );
    }
}

/// The NEO-M8N's port raised to the driver's rate from `rate`, the one it talks at: its port frame written at `rate`
/// and drained, [`PORT_SETTLE_MS`] for the module to re-open its port, then the UART at the raised rate.
fn raise_port(driver: &UartDriver<'static>, rate: u32) {
    let raised = gnss::drivers::neom8n::BAUD;
    let _ = driver.change_baudrate(Hertz(rate));
    let _ = driver.write(&gnss::drivers::neom8n::port(raised));
    let _ = driver.wait_tx_done(TickType::new_millis(PORT_DRAIN_MS).ticks());
    std::thread::sleep(Duration::from_millis(PORT_SETTLE_MS));
    let _ = driver.change_baudrate(Hertz(raised));
    let _ = driver.clear_rx();
}

/// How long a module takes to re-open its port after a port frame, ms.
const PORT_SETTLE_MS: u64 = 100;

/// The longest wait for a port frame to leave the UART, ms (28 bytes at 9600 baud take ~30 ms).
const PORT_DRAIN_MS: u64 = 200;

/// A configuration frame's answer: an ACK logged, a NAK logged and kept as an event (the configuration did not take).
fn acknowledge(name: &str, answer: gnss::ubx::Answer, events: &mut Option<Telemetry<'static>>) {
    match answer {
        gnss::ubx::Answer::Acknowledged(gnss::ubx::CONFIGURATION_CLASS, id) => {
            log::info!("{name} :: {} acknowledged", gnss::ubx::configuration_name(id))
        }
        gnss::ubx::Answer::Refused(gnss::ubx::CONFIGURATION_CLASS, id) => note(
            name,
            events,
            &format!(
                "the module refused {}: its configuration did not take",
                gnss::ubx::configuration_name(id)
            ),
        ),
        gnss::ubx::Answer::Acknowledged(class, id) => log::info!("{name} :: {class:#04x}/{id:#04x} acknowledged"),
        gnss::ubx::Answer::Refused(class, id) => {
            note(name, events, &format!("the module refused {class:#04x}/{id:#04x}"))
        }
    }
}

/// Publishes one line's update: the channels in plan B's order, the rows, the events.
fn publish(name: &str, update: &Update, channels: &mut Channels, streams: &mut Streams) {
    let now_us = uptime_us();
    if let (Some(position), Some(writer)) = (update.position, &mut channels.position) {
        writer.push([position.latitude_e7, position.longitude_e7], now_us);
    }
    for (value, writer) in [
        (update.speed_ms, &mut channels.speed),
        (update.course_deg, &mut channels.course),
        (update.altitude_m, &mut channels.altitude),
        (update.elevation_m, &mut channels.elevation),
    ] {
        if let (Some(value), Some(writer)) = (value, writer) {
            writer.push(value, now_us);
        }
    }
    if let Some(row) = update.fix_row {
        push(
            &mut streams.fix,
            &[
                Cell::E7(row.position.latitude_e7),
                Cell::E7(row.position.longitude_e7),
                Cell::F32(row.speed_knots),
                Cell::F32(row.course_deg),
            ],
        );
    }
    if let Some(row) = update.gga_row {
        push(
            &mut streams.gga,
            &[
                Cell::F32(row.altitude_m),
                Cell::F32(row.elevation_m),
                Cell::I32(row.quality),
                Cell::I32(row.satellites),
                Cell::I32(row.hdop_centi),
            ],
        );
    }
    for event in &update.events {
        note(name, &mut streams.events, &event.to_string());
    }
}

/// The engine's state for the executor.
fn publish_state(engine: &Gnss) {
    LINES.store(engine.lines(), Ordering::Relaxed);
    FIX.store(engine.fix(), Ordering::Relaxed);
    let quality = [
        engine.fix_quality(),
        engine.satellites(),
        (engine.hdop() * 100.0).round() as i32,
    ];
    for (published, value) in QUALITY.iter().zip(quality) {
        published.store(value, Ordering::Relaxed);
    }
}

/// One row to a stream, a refusal dropped (the rows are the record, not the flight).
fn push(stream: &mut Option<Telemetry<'static>>, row: &[Cell]) {
    if let Some(stream) = stream {
        let _ = stream.push(uptime_us(), row);
    }
}

/// One event: its durable row and its log line (plan B task.event).
fn note(name: &str, events: &mut Option<Telemetry<'static>>, text: &str) {
    log::warn!("{name} :: {text}");
    push(events, &[Cell::Text(text)]);
}

/// The fused `position` now (plan B `data['position'].value()`): the live fix `health` shows and the mission's launch
/// point falls back to; None while no source of it is fresh.
pub fn position(topology: &Topology) -> Option<Position> {
    let [latitude_e7, longitude_e7] = topology.position("position")?.read(uptime_us())?.value;
    Some(Position {
        latitude_e7,
        longitude_e7,
    })
}

/// What `inspect` shows of the GNSS beyond its channels (plan B gnss.py inspect).
pub struct Inspection {
    /// Whether the last RMC had a valid fix.
    pub fix: bool,
    /// The GGA's fix quality: 0 none, 1 GPS, 2 DGPS.
    pub fix_quality: i32,
    /// Satellites used in the fix (GGA).
    pub satellites: i32,
    /// The horizontal dilution of precision.
    pub hdop: f32,
    /// NMEA lines read since boot.
    pub lines: u32,
    /// The sky: in view, used, the GSA mode, the four strongest C/N0 (dB-Hz).
    pub sky: [i32; 7],
    /// The antenna's word (`unknown` on the NEO-6M).
    pub antenna: &'static str,
}

/// The GNSS's state for `inspect`; None before the thread runs.
pub fn inspection() -> Option<Inspection> {
    RUNNING.get()?;
    let sky = SKY.each_ref().map(|value| value.load(Ordering::Relaxed));
    let [fix_quality, satellites, hdop_centi] = QUALITY.each_ref().map(|value| value.load(Ordering::Relaxed));
    let antenna = [Antenna::Unknown, Antenna::Ok, Antenna::Open, Antenna::Short]
        .into_iter()
        .find(|antenna| *antenna as i32 == sky[7])
        .unwrap_or_default();
    Some(Inspection {
        fix: FIX.load(Ordering::Relaxed),
        fix_quality,
        satellites,
        hdop: hdop_centi as f32 / 100.0,
        lines: LINES.load(Ordering::Relaxed),
        sky: [sky[0], sky[1], sky[2], sky[3], sky[4], sky[5], sky[6]],
        antenna: antenna.name(),
    })
}

/// The GNSS's probe (plan B gnss.py probe), on the executor: the lines read advance within 1.5 s, logged in plan B's
/// shape. Its name and verdict (None healthy, else why); None when no GNSS runs or `only` names another object.
pub fn probe(only: Option<&str>) -> Option<(&'static str, Option<String>)> {
    let (name, uart, module) = *RUNNING.get()?;
    if only.is_some_and(|only| only != name) {
        return None;
    }
    let link = module.delivery();
    log::info!("{name} :: probe: {link} link ...");
    let before = LINES.load(Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(1500));
    let after = LINES.load(Ordering::Relaxed);
    let verdict = if after == before {
        let message = format!("{link} link: no {} on uart:{uart} in 1.5s", link.to_uppercase());
        log::warn!("{name} :: probe FAILED: {message}");
        Some(message)
    } else {
        let [_, satellites, hdop_centi] = QUALITY.each_ref().map(|value| value.load(Ordering::Relaxed));
        log::info!(
            "{name} :: probe: {link} link ok (+{} lines, fix={}, sats={satellites}, hdop={:.1})",
            after - before,
            FIX.load(Ordering::Relaxed),
            hdop_centi as f32 / 100.0
        );
        None
    };
    Some((name, verdict))
}

/// The pad drift calibration once its entry came up: the supervisor passes it every `period_ms`.
pub struct Calibration {
    /// The entry's name: its log line's.
    name: &'static str,
    /// Its pass period, ms (`period_ms`).
    period_ms: u32,
    /// The averaging and the freeze (crates/control drift).
    calibrator: DriftCalibrator,
    /// The fused ground speed, m/s.
    speed: Option<&'static Parameter<f32>>,
    /// The fused ground course, degrees.
    course: Option<&'static Parameter<f32>>,
}

/// The frozen drift, east and north (f32 bits, m/s): zero until the freeze.
static DRIFT: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
/// The calibration's samples since its restart.
static DRIFT_SAMPLES: AtomicU32 = AtomicU32::new(0);
/// Whether the drift froze.
static DRIFT_FROZEN: AtomicBool = AtomicBool::new(false);
/// Whether a calibration runs.
static CALIBRATING: AtomicBool = AtomicBool::new(false);

impl Calibration {
    /// The calibration of `member`, its quantities found in `topology`.
    pub fn new(member: GnssCalibMember, topology: &'static Topology) -> Calibration {
        CALIBRATING.store(true, Ordering::Relaxed);
        Calibration {
            name: member.name,
            period_ms: member.period_ms,
            calibrator: DriftCalibrator::new(member.min_samples),
            speed: topology.scalar("speed"),
            course: topology.scalar("course"),
        }
    }

    /// The entry's name.
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// Its pass period, ms.
    pub fn period_ms(&self) -> u32 {
        self.period_ms
    }

    /// One pass at `now_us` (plan B's run loop body): the flight word's stage and arming, the fused speed and course
    /// when both are fresh (read, never extrapolated: a stale fix must not enter the mean), and the freeze's log line.
    pub fn pass(&mut self, now_us: u64) {
        let flight = FLIGHT.load();
        let fresh = |parameter: Option<&'static Parameter<f32>>| {
            parameter
                .and_then(|parameter| parameter.read(now_us))
                .map(|reading| reading.value)
        };
        let ground = fresh(self.speed)
            .zip(fresh(self.course))
            .map(|(speed, course)| [speed, course]);
        if let Some(([east, north], count)) = self.calibrator.pass(flight.stage, flight.armed, ground) {
            log::info!("{} :: drift {east:.3}, {north:.3} m/s ({count} samples)", self.name);
        }
        for (published, value) in DRIFT.iter().zip(self.calibrator.drift()) {
            published.store(value.to_bits(), Ordering::Relaxed);
        }
        DRIFT_SAMPLES.store(self.calibrator.samples(), Ordering::Relaxed);
        DRIFT_FROZEN.store(self.calibrator.frozen(), Ordering::Relaxed);
    }
}

/// The frozen pad drift, east and north, m/s: zero until the freeze, and without a calibration (plan B
/// `GnssCalib.drift()`, the flight loop's wind de-bias, M13).
pub fn drift() -> [f32; 2] {
    DRIFT
        .each_ref()
        .map(|value| f32::from_bits(value.load(Ordering::Relaxed)))
}

/// The calibration's state for `inspect` (plan B gnss_calib.py inspect): the drift, its samples, whether it froze.
/// None without a calibration.
pub fn calibration_inspection() -> Option<([f32; 2], u32, bool)> {
    CALIBRATING.load(Ordering::Relaxed).then(|| {
        (
            drift(),
            DRIFT_SAMPLES.load(Ordering::Relaxed),
            DRIFT_FROZEN.load(Ordering::Relaxed),
        )
    })
}
