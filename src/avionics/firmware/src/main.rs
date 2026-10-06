//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Avionics bring-up: count the boot, open the UART console, then join Wi-Fi through the C6 and serve the
//! CC hub forever. A panic anywhere prints its backtrace and reboots (sdkconfig: PANIC_PRINT_REBOOT).

mod board;
mod commands;
mod console;
mod diagnostics;
mod files;
mod link;

use std::sync::Arc;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use firmware::thermal::Thermometer;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    // SAFETY: rewrites one function pointer in esp_hosted's OS table; nothing has failed over yet.
    unsafe { esp_idf_svc::sys::hosted_no_reboot_install() };

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
    // the console drains the UART before taking it over, so the lines above survive
    console::start(
        peripherals.uart0,
        peripherals.pins.gpio37,
        peripherals.pins.gpio38,
        board.clone(),
    )?;
    log::info!("{}", board.banner());
    link::run(peripherals.modem, event_loop, partition, &board)
}
