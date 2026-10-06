//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Avionics bring-up: start the clock, count the boot, load the mission, start the CC executor, open the UART
//! console, then join Wi-Fi through the C6 and serve the CC hub forever. A panic anywhere prints its backtrace
//! and reboots (sdkconfig: PANIC_PRINT_REBOOT).

mod board;
mod clock;
mod commands;
mod config;
mod console;
mod diagnostics;
mod files;
mod link;
mod status;
mod tee;

use std::sync::Arc;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use firmware::thermal::Thermometer;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    tee::install();
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
    let board = Arc::new(board::Board::open(partition.clone(), thermometer, sd_mounted)?);
    let store = config::Store::open()?;
    // the zone range gate is plan B's default until the board config (its `field` component) is read (M2)
    let mission = mission::Mission::load(store.launch().as_deref(), mission::DEFAULT_MAX_RANGE_M);
    let executor = commands::Executor::start(commands::Context {
        board: board.clone(),
        store,
        mission,
    })?;
    // the console drains the UART before taking it over, so the lines above survive
    console::start(
        peripherals.uart0,
        peripherals.pins.gpio37,
        peripherals.pins.gpio38,
        board.clone(),
        executor.clone(),
    )?;
    log::info!("{}", board.banner());
    link::run(peripherals.modem, event_loop, partition, &board, &executor)
}
