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
//! text only for its own event rows and log lines. The UBX NAV-PVT path (the NEO-M8N) comes with M16.
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
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::hal::uart::{Uart, UartConfig, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys;
use gnss::{Antenna, Gnss, SkySwitch, Update};
use recorder::{Cell, Telemetry};
use sequencer::Stage;

use crate::devices::{GnssCalibMember, GnssMember, Topology};
use crate::status::FLIGHT;

/// The module on the UART, as the entry's driver names it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Module {
    /// The NEO-6M (`neo6mv2`): UBX configuration, NMEA output.
    Neo6,
    /// The ATGM336H (`atgm336h`): CASIC (PCAS) configuration.
    Atgm,
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

/// NMEA lines read since boot (plan B's `_lines`: the probe's liveness counter).
static LINES: AtomicU32 = AtomicU32::new(0);
/// Whether the last RMC had a valid fix.
static FIX: AtomicBool = AtomicBool::new(false);
/// The GGA's fix quality, satellites used and HDOP in hundredths.
static QUALITY: [AtomicI32; 3] = [const { AtomicI32::new(0) }; 3];
/// The latest sky row's eight columns (in view, used, mode, the four C/N0, the antenna code).
static SKY: [AtomicI32; 8] = [const { AtomicI32::new(0) }; 8];
/// The GNSS's entry name and its UART's id, once the thread runs.
static RUNNING: OnceLock<(&'static str, u8)> = OnceLock::new();

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
    ThreadSpawnConfiguration {
        name: Some(c"gnss"),
        stack_size: STACK_SIZE,
        priority: PRIORITY,
        pin_to_core: Some(Core::Core0),
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new()
        .stack_size(STACK_SIZE)
        .spawn(move || run(member, driver, topology));
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
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
    let _ = RUNNING.set((name, member.uart.id));
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
    let stream = |key: String| {
        crate::recorder::get()
            .zip(crate::recorder::device_stream(&key))
            .and_then(|(recorder, stream)| Telemetry::new(recorder, stream))
    };
    let mut streams = Streams {
        fix: stream(name.to_string()),
        gga: stream(format!("{name}_gga")),
        sky: stream(format!("{name}_sky")),
        events: stream(format!("{name}_events")),
    };
    let mut switch = configure(&driver, member.module, member.hz);
    // DONE (M11b): the module at 1 Hz, the beacon's cadence, until a reset
    let mut slowed = false;
    log::info!(
        "{name} :: {:?} configured at {} Hz on uart:{} ({} baud)",
        member.module,
        member.hz,
        member.uart.id,
        member.uart.baud
    );
    let mut engine = Gnss::new();
    let mut line: Vec<u8> = Vec::with_capacity(LINE_BYTES);
    let mut buffer = [0u8; 256];
    let wait = TickType::new_millis(READ_WAIT_MS).ticks();
    let mut next_tick = Instant::now() + TICK;
    loop {
        let count = driver.read(&mut buffer, wait).unwrap_or(0);
        for &byte in &buffer[..count] {
            if line.len() == LINE_BYTES {
                line.clear(); // noise: no newline within a sentence's length
            }
            line.push(byte);
            // the line with its newline, as plan B's readline() gave it: a bare newline is a line too
            if byte == b'\n' {
                let update = engine.line(&line, uptime_us());
                publish(name, &update, &mut channels, &mut streams);
                line.clear();
            }
        }
        publish_state(&engine);
        let now = Instant::now();
        if now >= next_tick {
            next_tick = now + TICK;
            let stage = FLIGHT.load().stage;
            if !slowed && stage == Stage::Done && member.hz > DONE_HZ {
                switch = configure(&driver, member.module, DONE_HZ);
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

/// Writes the module's configuration (plan B `_configure`): each command, then the driver's gap. Returns its sky
/// switch, the two frames written whole at the flight window's edges.
fn configure(driver: &UartDriver<'static>, module: Module, hz: i32) -> SkySwitch {
    let (commands, gap_ms, switch): (Vec<Vec<u8>>, u32, SkySwitch) = match module {
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
    };
    for command in commands {
        let _ = driver.write(&command);
        std::thread::sleep(Duration::from_millis(u64::from(gap_ms)));
    }
    switch
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

/// The uptime in microseconds.
fn uptime_us() -> u64 {
    // SAFETY: a plain read of the monotonic timer.
    unsafe { sys::esp_timer_get_time() as u64 }
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
    let (name, uart) = *RUNNING.get()?;
    if only.is_some_and(|only| only != name) {
        return None;
    }
    log::info!("{name} :: probe: nmea link ...");
    let before = LINES.load(Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(1500));
    let after = LINES.load(Ordering::Relaxed);
    let verdict = if after == before {
        let message = format!("nmea link: no NMEA on uart:{uart} in 1.5s");
        log::warn!("{name} :: probe FAILED: {message}");
        Some(message)
    } else {
        let [_, satellites, hdop_centi] = QUALITY.each_ref().map(|value| value.load(Ordering::Relaxed));
        log::info!(
            "{name} :: probe: nmea link ok (+{} lines, fix={}, sats={satellites}, hdop={:.1})",
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
