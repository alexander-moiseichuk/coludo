//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Avionics bring-up (doc/specs/avionics-port.md "Bring-up order"): start the clock, count the boot and load the
//! board config, which name this boot's records; open the recorder before anything else logs, so every later line is
//! kept; mount the card and start the ring_writer; then the board, the supervisor, the mission, the CC executor and
//! the UART console, and finally join Wi-Fi through the C6 and serve the CC hub forever. A panic anywhere prints its
//! backtrace and reboots (sdkconfig: PANIC_PRINT_REBOOT).

mod board;
mod clock;
mod commands;
mod config;
mod console;
mod diagnostics;
mod files;
mod link;
mod logging;
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
    let thermometer = Thermometer::new(peripherals.temp_sensor)
        .inspect_err(|error| log::warn!("thermometer :: {error}"))
        .ok();
    let sd_mounted = firmware::sd::mount();
    if sd_mounted {
        let start = std::time::Instant::now();
        let free = firmware::sd::measure_free();
        log::info!(
            "sd :: mounted, {free:?} MB free (measured in {} ms)",
            start.elapsed().as_millis()
        );
    } else {
        log::warn!("sd :: no usable microSD -- running without it");
    }
    // the camera's encoder before anything else takes internal RAM: its reference buffer is one ~138 KB internal
    // block, which the ring threads' stacks and buffers would split (no card, no recording, no reservation)
    if sd_mounted && let Err(error) = firmware::video::reserve() {
        log::warn!("video :: encoder not reserved, `video start` will try again: {error}");
    }
    // the ring_writer scans the `log` partition in its own thread; boot goes on, and the producers queue meanwhile
    if let Err(error) = recorder::start(settings.clone(), sd_mounted) {
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
    diagnostics::register_commands();
    credentials.register_commands();
    link::register_commands();
    if sd_mounted {
        firmware::sd::register_commands();
        firmware::video::register_commands(board.session.clone(), board.boot_id.unwrap_or(0) as u32);
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

/// The system's own console command (the Glider Command Language): `panic confirm`, which proves a panic reboots the
/// board (and warm-starts it, once that is ported).
fn register_system_commands() {
    firmware::gcl::register(vec![firmware::gcl::Command {
        owner: "system",
        words: "panic",
        about: "panic on purpose: proves a panic reboots the board (and warm-starts it once that is ported)",
        gate: firmware::gcl::Gate::Setting,
        reach: firmware::gcl::Reach::Everywhere,
        parameters: &[],
        bare_words: &[firmware::gcl::CONFIRM],
        run: Box::new(|_| panic!("deliberate panic from a console command")),
    }]);
}
