//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Avionics bring-up (doc/specs/avionics-port.md "Bring-up order"): start the clock, count the boot and load the
//! board config, which name this boot's records; open the recorder before anything else logs, so every later line is
//! kept, in flight when NVS holds a crumb in BOOSTING..LANDING; mount the card (one FAT volume, or two: the recordings
//! and the videos); create the I2C ports and decide the layout by their scan, which rewrites the config and so names
//! its id; start the ring tasks (the writer, the eraser, and the saver that copies the flash to the card); then the
//! board, the devices in config order on the lent ports, and the warm start's restore, which may commit a flight
//! through the flight word's writer and rebase the barometers while main still holds both; then each port goes back to
//! its owner with the parts that came up, and the control thread takes the writer; the supervisor (health and the
//! checkpoint and the watchdog), the CC executor and the UART console, and finally join Wi-Fi through the C6 and serve
//! the CC hub forever. A panic anywhere prints its backtrace and reboots (sdkconfig: PANIC_PRINT_REBOOT).

mod board;
mod clock;
mod commands;
mod config;
mod console;
mod control;
mod devices;
mod diagnostics;
mod files;
mod link;
mod logging;
mod owner;
mod panics_logger;
mod recorder;
mod sensors;
mod status;
mod supervisor;
mod warm;

use std::sync::Arc;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use firmware::thermal::Thermometer;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    logging::install();
    clock::start();

    let peripherals = Peripherals::take()?;
    let event_loop = EspSystemEventLoop::take()?;
    let partition = EspDefaultNvsPartition::take()?;
    // the boot count and the board config first: they name this boot's records (plan B main: the boot id before
    // bring-up). Their own lines wait for the recorder below, so the capture has them too.
    let boot_id = board::count_boot(partition.clone())?;
    let settings = config::partition()?;
    let store = config::Store::open(settings.clone())?;
    let mut loaded = board_config::load(store.board().as_deref(), config::defaults());
    // the saved config's id: what `set-config board` answered for it. The boot runs under the post-layout one below
    let saved_config_id = board_config::config_id(&loaded.document);
    let session = recorder::session_prefix(&loaded.document, boot_id);
    // a crumb in BOOSTING..LANDING: a reboot in flight, whose records go into flight blocks from the first one
    let in_flight = warm::peek(&partition);
    // storage first: from here on every log line and row is kept, queued in PSRAM until the ring_writer runs
    if let Err(error) = recorder::open(&loaded.document, boot_id, &session, in_flight) {
        log::error!("Recorder :: not recording this boot: {error}");
    }
    log::info!("config :: {saved_config_id} {}", loaded.source);
    for error in &loaded.errors {
        log::warn!("config :: {error}");
    }
    // the last boot's panic into the panic log, before anything here can panic again; then this boot's capture
    panics_logger::start(boot_id.unwrap_or(0) as u32, &session, &saved_config_id);
    let thermometer = Thermometer::new(peripherals.temp_sensor)
        .inspect_err(|error| log::warn!("thermometer :: {error}"))
        .ok();
    let sd_mounted = firmware::sd::mount();
    // the card by partition 1's volume serial: the ring_saver's position hint is trusted only for the card it was
    // saved to
    let card = sd_mounted.then(|| recorder::Card {
        serial: firmware::sd::volume_serial(),
    });
    if let (Some(card), Some(layout)) = (card, firmware::sd::layout()) {
        log_card(card, layout);
    } else {
        log::warn!("sd :: no usable microSD -- running without it");
    }
    // the camera's encoder before anything else takes internal RAM: its reference buffer is one ~138 KB internal
    // block, which the ring threads' stacks and buffers would split. No camera in the board config or no card: no
    // recording, so no reservation, and the block stays the heap's
    let camera = firmware::video::Camera::from_config(&loaded.document);
    match camera {
        Some(camera) => log::info!("video :: camera ov5647, oscillator {} Hz", camera.clock_hz),
        // loud with a card: a config made from a template without the entry would fly without video
        None if sd_mounted => {
            log::warn!("video :: no camera in the board config (an 'ov5647' component): no video this boot")
        }
        None => log::info!("video :: no camera in the board config: no video"),
    }
    if sd_mounted
        && camera.is_some()
        && let Err(error) = firmware::video::reserve()
    {
        log::warn!("video :: encoder not reserved, `video start` will try again: {error}");
    }
    // the oops thread before the bring-up, which records a general call as an oops (its control block is internal,
    // so after the encoder's reservation)
    if let Err(error) = panics_logger::start_oops() {
        log::error!("panic :: the oops thread did not start, oopses are dropped: {error}");
    }
    // the I2C ports, each created by its owner thread on core 1 and lent here; port 0 always (the camera's SCCB is
    // wired to it), port 1 when the config defines it
    let mut lent = Vec::new();
    for id in 0..2u8 {
        let Some(bus) = board_config::i2c_bus(&loaded.document, id).or((id == 0).then_some(PORT0_STOCK)) else {
            continue;
        };
        match owner::spawn(id, bus.sda, bus.scl, bus.clock_hz) {
            Ok(loan) => lent.push(loan),
            Err(error) => log::error!("i2c:{id} :: no bus: {error}"),
        }
    }
    // which board this is, by the scan of the lent ports, before anything reads the config for its devices (plan B
    // main.py:91-104): the config is rewritten in place, and from here on it is the one this boot runs
    // a bus spec the view cannot read (a pin past 255) votes nothing, as plan B's Pin() raised on it, even though
    // port 0 runs on the stock pins then
    let readable = [0, 1].map(|bus| board_config::i2c_bus(&loaded.document, bus).is_some());
    let resolved = layout::resolve(&mut loaded.document, |bus, _| {
        let Some((_, port)) = lent
            .iter_mut()
            .find(|(_, port)| port.id() == bus && readable[usize::from(bus)])
        else {
            return Err(());
        };
        port.scan()
            .map_err(|error| log::warn!("layout :: i2c:{bus} scan failed ({error}): no votes from this bus"))
    });
    let undecided = matches!(resolved, layout::Resolved::Undecided { .. });
    for line in resolved.log_lines() {
        if undecided {
            log::warn!("{line}");
        } else {
            log::info!("{line}");
        }
    }
    // the ports at the rewritten config's clocks
    for (_, port) in &mut lent {
        let id = port.id();
        let clock_hz = board_config::i2c_bus(&loaded.document, id).map_or(PORT0_STOCK.clock_hz, |bus| bus.clock_hz);
        if let Err(error) = port.configure(clock_hz) {
            log::warn!("i2c:{id} :: not at the config's {clock_hz} Hz: {error}");
        }
    }
    // the id this boot runs under: the rewritten config's (plan B main.py:120-124), in its records from here on
    let config_id = board_config::config_id(&loaded.document);
    panics_logger::name_config(&config_id);
    // the ring_writer scans the `log` partition in its own thread, then starts the ring_eraser and, with a card, the
    // ring_saver: all three after the encoder's reservation. Boot goes on, and the producers queue meanwhile
    if let Err(error) = recorder::start(settings.clone(), card, &config_id) {
        log::error!("Recorder :: the ring_writer did not start: {error}");
    }
    let board = Arc::new(board::Board::new(
        board_config::board_id(&loaded.document),
        boot_id,
        session,
        thermometer,
        sd_mounted,
    ));
    // provenance (plan B main): the build and the config this capture comes from, stamped into the capture itself;
    // the saved config's id last, the one `set-config board` answered
    log::info!(
        "main :: boot: board {} | firmware {} | config {config_id} {} | reset_cause {} | boot {} | session {} | \
         saved_config {saved_config_id}",
        board.id,
        board::FIRMWARE_VERSION,
        loaded.source,
        board::reset_reason(),
        board
            .boot_id
            .map_or_else(|| "None".to_string(), |boot_id| boot_id.to_string()),
        board.session,
    );
    // the config is frozen for the run from here; the devices come up in its order on the lent ports, with nothing
    // else running (plan B's Controller.setup), and each port goes back to its owner with the parts that came up
    let loaded: &'static board_config::Loaded = Box::leak(Box::new(loaded));
    let devices::Devices {
        table,
        topology,
        mut members,
        spi,
        spi_bus,
        separation,
        sequencer,
        checkpoint,
        watchdog,
    } = devices::bring_up(&loaded.document, &mut lent, recorder::get().is_some());
    // the warm start, while the parts and the flight word's one writer are still main's and nothing else runs: a reset
    // in flight resumes the crumb's stage, its barometers rebased to the pad and the mission's launch point and zone
    // restored (doc/specs/avionics-port.md "Bring-up order"; crate::warm)
    let mut writer = status::FLIGHT
        .writer()
        .expect("main takes the flight word's writer, once");
    let range = board_config::max_range_m(&loaded.document).unwrap_or(mission::DEFAULT_MAX_RANGE_M);
    let mut mission = mission::Mission::load(store.launch().as_deref(), range);
    let separated = separation.as_ref().is_some_and(|member| member.part.separated());
    let seed = warm::restore(
        &partition,
        &loaded.document,
        separated,
        &mut mission,
        &mut members,
        &mut writer,
        in_flight,
    );
    // the zone and the launch point the checkpoint freezes at BOOSTING entry; the executor republishes them from here
    warm::publish_mission(&mission);
    // the magnetometer's hard/soft-iron calibration, as plan B's setup restored it
    sensors::restore_calibrations(&partition, &mut members);
    for (parked, port) in lent {
        let id = port.id();
        let parts = std::mem::take(&mut members[usize::from(id)]);
        if let Err(error) = owner::start(parked, port, parts, topology, table) {
            log::error!("i2c:{id} :: {error}");
        }
    }
    // the checkpoint's pad latch stops at half the altitude launch's rise: a launch the accel trigger missed stays in
    // SETTING through the altitude launch's dwell, and must not latch its pad up the climb
    let pad_ceiling_m = sequencer.as_ref().map(|member| member.settings.launch_alt_m / 2.0);
    // the flight tick on core 1, which wakes the owners too (§4.2, §4.3), steps the stage machine and owns the flight
    // word's one writer from here
    let schedule_hz = board_config::schedule_hz(&loaded.document);
    if let Err(error) = control::start(
        schedule_hz,
        spi_bus,
        spi,
        separation,
        sequencer,
        topology,
        table,
        writer,
    ) {
        log::error!("control :: the control thread did not start: {error}");
    }
    // the checkpoint keeps no crumb without its NVS namespace: its entry is crashed then, so `arm` refuses. A warm
    // start's identity seeds it, so the restored flight's crumbs carry the pad and the launch it launched with
    let keeping = checkpoint.and_then(|member| {
        warm::Keeping::open(member, &partition, topology, pad_ceiling_m, seed)
            .inspect_err(|error| {
                log::error!("{} :: {error}", member.name);
                devices::crash(table, [Some(member.name)], error);
            })
            .ok()
    });
    // the stall limit is at least two control ticks
    let tick_ms = 1000.0 / schedule_hz.max(1) as f32;
    if let Err(error) = supervisor::start(board.clone(), table, keeping, watchdog, tick_ms) {
        log::error!("supervisor :: did not start: {error}");
    }
    let wifi = board_config::wifi(&loaded.document);
    let credentials = config::Credentials::new(settings);
    // the Glider Command Language: each component registers its own commands as it activates, so `help` lists
    // exactly what this board runs (firmware::gcl)
    firmware::gcl::set_cc_names(commands::names());
    register_system_commands();
    owner::register_commands();
    control::register_commands();
    panics_logger::register_commands();
    diagnostics::register_commands();
    credentials.register_commands();
    link::register_commands();
    if sd_mounted {
        firmware::sd::register_commands();
        // the camera's one owner: the flight recording by the flight word, and `video start`/`stop` through its
        // mailbox
        if let Some(camera) = camera
            && let Err(error) = firmware::video::start(
                board.session.clone(),
                board.boot_id.unwrap_or(0) as u32,
                camera,
                &status::FLIGHT,
            )
        {
            log::error!("video :: the video thread did not start, no video this boot: {error}");
        }
    }
    let executor = commands::Executor::start(commands::Context {
        board: board.clone(),
        store,
        nvs: partition.clone(),
        mission,
        config: loaded,
        config_id,
        layout: resolved.label(),
        devices: table,
        topology,
    })?;
    // the console drains the UART before taking it over, so the lines above survive
    console::start(
        peripherals.uart0,
        peripherals.pins.gpio37,
        peripherals.pins.gpio38,
        executor.clone(),
    )?;
    log::info!("{}", board.banner());
    link::run(peripherals.modem, event_loop, partition, wifi, credentials, &executor)
}

/// Port 0 when the config defines no i2c:0: the WaveShare board's own wiring (the camera's SCCB and the codec are on
/// SDA 7 / SCL 8), at plan B's default clock.
const PORT0_STOCK: board_config::I2cSpecification = board_config::I2cSpecification {
    id: 0,
    sda: 7,
    scl: 8,
    clock_hz: 400_000,
};

/// Logs the mounted card: partition 1's serial (the name the session-start record keeps), how its volumes are
/// mounted (one partition or two, doc/specs/avionics-port.md "SD and flash file layout"), the videos partition's
/// serial, and each volume's size and free space; and, loudly, a videos partition that would not mount.
fn log_card(card: recorder::Card, layout: firmware::sd::Layout) {
    let start = std::time::Instant::now();
    let volumes = firmware::sd::measure_free();
    let videos = match layout {
        firmware::sd::Layout::TwoPartitions => format!(
            " (videos {})",
            firmware::sd::videos_serial().map_or_else(|| "unknown".to_string(), firmware::sd::serial_name)
        ),
        firmware::sd::Layout::OnePartition => " (videos/ a directory beside the sessions)".to_string(),
        firmware::sd::Layout::VideosFailed => String::new(),
    };
    log::info!(
        "sd :: mounted card {}, {}{videos}: {} (measured in {} ms)",
        card.name(),
        layout.name(),
        firmware::sd::describe(&volumes),
        start.elapsed().as_millis()
    );
    if layout == firmware::sd::Layout::VideosFailed {
        log::error!(
            "sd :: partition 2 is not mounted (it would not mount, or the table is unsound: sd_card's line above says \
             which): no video this boot"
        );
    }
}

/// The kinds of a deliberate panic (`panic kind=... confirm`).
const PANIC_KINDS: &[&str] = &["rust", "fault", "abort", "oops", "twdt", "stall"];

/// The system's own console command (the Glider Command Language): `panic kind=... confirm`, which proves a panic
/// reboots the board, warm-starting a flight in the air (crate::warm), and lands in the panic log
/// (crate::panics_logger); and that the supervisor's watchdog resets a board whose supervisor stops feeding the task
/// watchdog (`twdt`) or whose control thread stalls (`stall`). Open in every stage, for the console alone: it is the
/// one hands-free reset in GLIDING and at DONE (`reboot` runs only in SETTING), and never one the hub can send. `twdt`
/// and `stall` are refused when nothing would reset the board: a stalled control thread could not even be disarmed.
fn register_system_commands() {
    firmware::gcl::register(vec![firmware::gcl::Command {
        owner: "system",
        words: "panic",
        about: "panic on purpose and see it in `panic log`: rust (a Rust panic), fault (a store to address 0), abort \
                (C's abort()) reboot the board; oops records a non-fatal panic and goes on; twdt stops the \
                supervisor feeding the task watchdog, which panics the board wdt_timeout_ms later (refused unless the \
                task watchdog is armed); stall stops the control thread, and the supervisor's stall check restarts \
                the board (refused unless it controls a flight and the check would restart it)",
        gate: firmware::gcl::Gate::Open,
        reach: firmware::gcl::Reach::Console,
        parameters: &[firmware::gcl::Parameter {
            name: "kind",
            kind: firmware::gcl::Kind::Choice(PANIC_KINDS),
            default: Some("rust"),
            about: "rust, fault, abort, twdt, stall (each resets) or oops (does not)",
        }],
        bare_words: &[firmware::gcl::CONFIRM],
        run: Box::new(|arguments| match arguments.text("kind") {
            "fault" => {
                // SAFETY: deliberately not: a store to address 0 is the access fault this kind exists to raise.
                unsafe { core::arch::asm!("sw zero, 0(zero)") };
                unreachable!("a store to address 0 faults")
            }
            // SAFETY: C's abort() never returns; it is the point of this kind.
            "abort" => unsafe { esp_idf_svc::sys::abort() },
            "oops" => {
                panics_logger::oops(
                    "test-oops",
                    "a deliberate oops from a console command: nothing is broken",
                );
                cc_protocol::build("ok", &["panic", "oops"], &[])
            }
            "twdt" => match supervisor::starve() {
                Ok(()) => cc_protocol::build("ok", &["panic", "twdt"], &[]),
                Err(refusal) => cc_protocol::build("err", &["refused", refusal], &[]),
            },
            "stall" => match supervisor::stall_refused() {
                None => {
                    control::stall();
                    cc_protocol::build("ok", &["panic", "stall"], &[])
                }
                Some(refusal) => cc_protocol::build("err", &["refused", refusal], &[]),
            },
            _ => panic!("deliberate panic from a console command"),
        }),
    }]);
}
