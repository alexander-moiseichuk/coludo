//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Avionics bring-up (doc/specs/avionics-port.md "Bring-up order"): start the clock, count the boot and load the
//! board config, which name this boot's records; open the recorder before anything else logs, so every later line is
//! kept; mount the card (one FAT volume, or two: the recordings and the videos) and start the ring tasks (the writer,
//! the eraser, and the saver that copies the flash to the card); then the board, the supervisor, the mission, the CC
//! executor and the UART console, and finally join Wi-Fi through the C6 and serve the CC hub forever. A panic
//! anywhere prints its backtrace and reboots (sdkconfig: PANIC_PRINT_REBOOT).

mod board;
mod clock;
mod commands;
mod config;
mod console;
mod diagnostics;
mod files;
mod link;
mod logging;
mod owner;
mod panics_logger;
mod recorder;
mod status;
mod supervisor;

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
    let loaded = board_config::load(store.board().as_deref(), config::defaults());
    let config_id = board_config::config_id(&loaded.document);
    let session = recorder::session_prefix(&loaded.document, boot_id);
    // storage first: from here on every log line and row is kept, queued in PSRAM until the ring_writer runs
    if let Err(error) = recorder::open(&loaded.document, boot_id, &session, &config_id) {
        log::error!("Recorder :: not recording this boot: {error}");
    }
    log::info!("config :: {config_id} {}", loaded.source);
    for error in &loaded.errors {
        log::warn!("config :: {error}");
    }
    // the last boot's panic into the panic log, before anything here can panic again; then this boot's capture
    panics_logger::start(boot_id.unwrap_or(0) as u32, &session, &config_id);
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
    // the I2C ports, each created by its owner thread on core 1 and lent here; port 0 always (the camera's SCCB is
    // wired to it), port 1 when the config defines it. Back to their owners at once until layout and setup use them
    for id in 0..2u8 {
        let Some((sda, scl, clock_hz)) = i2c_pins(&loaded.document, id).or((id == 0).then_some((7, 8, 400_000))) else {
            continue;
        };
        match owner::spawn(id, sda, scl, clock_hz).and_then(|(parked, port)| owner::start(parked, port)) {
            Ok(()) => {}
            Err(error) => log::error!("i2c:{id} :: no bus: {error}"),
        }
    }
    // the ring_writer scans the `log` partition in its own thread, then starts the ring_eraser and, with a card, the
    // ring_saver: all three after the encoder's reservation. Boot goes on, and the producers queue meanwhile
    if let Err(error) = recorder::start(settings.clone(), card) {
        log::error!("Recorder :: the ring_writer did not start: {error}");
    }
    let board = Arc::new(board::Board::new(
        board_config::board_id(&loaded.document),
        boot_id,
        session,
        thermometer,
        sd_mounted,
    ));
    // provenance (plan B main): the build and the config this capture comes from, stamped into the capture itself
    log::info!(
        "main :: boot: board {} | firmware {} | config {config_id} {} | reset_cause {} | boot {} | session {}",
        board.id,
        board::FIRMWARE_VERSION,
        loaded.source,
        board::reset_reason(),
        board
            .boot_id
            .map_or_else(|| "None".to_string(), |boot_id| boot_id.to_string()),
        board.session,
    );
    if let Err(error) = supervisor::start(board.clone()) {
        log::error!("supervisor :: {error}");
    }
    let range = board_config::max_range_m(&loaded.document).unwrap_or(mission::DEFAULT_MAX_RANGE_M);
    let mission = mission::Mission::load(store.launch().as_deref(), range);
    let wifi = board_config::wifi(&loaded.document);
    let credentials = config::Credentials::new(settings);
    // the Glider Command Language: each component registers its own commands as it activates, so `help` lists
    // exactly what this board runs (firmware::gcl)
    firmware::gcl::set_cc_names(commands::names());
    register_system_commands();
    owner::register_commands();
    if let Err(error) = panics_logger::start_oops() {
        log::error!("panic :: the oops thread did not start, oopses are dropped: {error}");
    }
    panics_logger::register_commands();
    diagnostics::register_commands();
    credentials.register_commands();
    link::register_commands();
    if sd_mounted {
        firmware::sd::register_commands();
        if let Some(camera) = camera {
            firmware::video::register_commands(board.session.clone(), board.boot_id.unwrap_or(0) as u32, camera);
        }
    }
    let executor = commands::Executor::start(commands::Context {
        board: board.clone(),
        store,
        mission,
        config: loaded,
        config_id,
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

/// Port `id`'s pins and speed from the board config's `buses.i2c.<id>` (`sda`, `scl`, `freq` default 400 kHz, plan B
/// i2cbus.py:91-96); None when the config does not define it.
fn i2c_pins(document: &serde_json::Value, id: u8) -> Option<(u8, u8, u32)> {
    let bus = document.get("buses")?.get("i2c")?.get(id.to_string())?;
    let pin = |key: &str| {
        bus.get(key)
            .and_then(serde_json::Value::as_u64)
            .and_then(|pin| u8::try_from(pin).ok())
    };
    let clock_hz = bus
        .get("freq")
        .and_then(serde_json::Value::as_u64)
        .map_or(400_000, |hz| hz as u32);
    Some((pin("sda")?, pin("scl")?, clock_hz))
}

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
const PANIC_KINDS: &[&str] = &["rust", "fault", "abort", "oops"];

/// The system's own console command (the Glider Command Language): `panic kind=... confirm`, which proves a panic
/// reboots the board (and warm-starts it, once that is ported) and lands in the panic log (crate::panics_logger).
fn register_system_commands() {
    firmware::gcl::register(vec![firmware::gcl::Command {
        owner: "system",
        words: "panic",
        about: "panic on purpose and see it in `panic log`: rust (a Rust panic), fault (a store to address 0), abort \
                (C's abort()) reboot the board; oops records a non-fatal panic and goes on",
        gate: firmware::gcl::Gate::Setting,
        reach: firmware::gcl::Reach::Everywhere,
        parameters: &[firmware::gcl::Parameter {
            name: "kind",
            kind: firmware::gcl::Kind::Choice(PANIC_KINDS),
            default: Some("rust"),
            about: "rust, fault, abort (each reboots) or oops (does not)",
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
            _ => panic!("deliberate panic from a console command"),
        }),
    }]);
}
