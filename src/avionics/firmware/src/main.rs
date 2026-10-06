//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Avionics bring-up: start the clock, load the board config, count the boot, load the mission, start the CC
//! executor, open the UART console, then join Wi-Fi through the C6 and serve the CC hub forever. A panic anywhere prints its backtrace
//! and reboots (sdkconfig: PANIC_PRINT_REBOOT).

mod board;
mod clock;
mod commands;
mod config;
mod console;
mod diagnostics;
mod files;
mod link;
mod logging;
mod status;

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
    // the board config first: it names the board, its networks and its mission range (plan B config.load)
    let settings = config::partition()?;
    let store = config::Store::open(settings.clone())?;
    let loaded = board_config::load(store.board().as_deref(), config::defaults());
    let config_id = board_config::config_id(&loaded.document);
    log::info!("config :: {config_id} {}", loaded.source);
    for error in &loaded.errors {
        log::warn!("config :: {error}");
    }
    let board = Arc::new(board::Board::open(
        board_config::board_id(&loaded.document),
        partition.clone(),
        thermometer,
        sd_mounted,
    )?);
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
        let session = board.session().unwrap_or_else(|| "uncounted".into());
        firmware::video::register_commands(session, board.boot_id.unwrap_or(0) as u32);
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
