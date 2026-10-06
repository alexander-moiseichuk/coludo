//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Thermal soak: how hot does the die get with both HP cores at full load, and how fast? 10 min of f32 FMA
//! on both cores (the peak.rs kernel), then 3 min idle, the die temperature printed every 5 s as
//! `soak;<seconds>;<celsius>;<load|idle>`, and a summary. Run it bare and with a heatsink to compare; the
//! camera heat test (V8) repeats it with the video pipeline running.
//!
//! The burners sleep 1 ms every ~100 ms, so the idle tasks still run and the task watchdog stays quiet
//! (~1% duty lost).

use std::hint::black_box;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use firmware::thermal::Thermometer;

/// Seconds at full load.
const LOAD_SECONDS: u64 = 600;
/// Seconds idle afterwards.
const COOL_SECONDS: u64 = 180;
/// Seconds between readings.
const PERIOD_SECONDS: u64 = 5;

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    let thermometer = Thermometer::new(Peripherals::take()?.temp_sensor)?;
    std::thread::sleep(Duration::from_secs(2));
    let running = Arc::new(AtomicBool::new(true));
    let burners: Vec<_> = [Core::Core0, Core::Core1]
        .into_iter()
        .map(|core| {
            ThreadSpawnConfiguration {
                pin_to_core: Some(core),
                priority: 5,
                ..Default::default()
            }
            .set()
            .unwrap();
            let running = running.clone();
            std::thread::spawn(move || burn(&running))
        })
        .collect();
    ThreadSpawnConfiguration::default().set()?;
    let start = Instant::now();
    let mut readings: Vec<(u64, f32)> = Vec::new();
    let mut seconds = 0;
    while seconds <= LOAD_SECONDS + COOL_SECONDS {
        if seconds == LOAD_SECONDS {
            running.store(false, Ordering::Relaxed);
        }
        let celsius = thermometer.celsius().unwrap_or(f32::NAN);
        println!(
            "soak;{seconds};{celsius:.1};{}",
            if seconds < LOAD_SECONDS { "load" } else { "idle" }
        );
        readings.push((seconds, celsius));
        seconds += PERIOD_SECONDS;
        let next = Duration::from_secs(seconds);
        std::thread::sleep(next.saturating_sub(start.elapsed()));
    }
    for burner in burners {
        burner.join().unwrap();
    }
    let at = |second: u64| {
        readings
            .iter()
            .find(|(time, _)| *time >= second)
            .map(|(_, celsius)| *celsius)
    };
    let loaded: Vec<f32> = readings
        .iter()
        .filter(|(time, _)| *time < LOAD_SECONDS)
        .map(|(_, celsius)| *celsius)
        .collect();
    let peak = loaded.iter().cloned().fold(f32::MIN, f32::max);
    let plateau = loaded.iter().rev().take(12).sum::<f32>() / 12.0; // the last minute under load
    let settled = readings
        .iter()
        .find(|(time, celsius)| *time < LOAD_SECONDS && (celsius - plateau).abs() <= 0.5);
    println!("soak;summary;idle_start;{:.1};C", at(0).unwrap_or(f32::NAN));
    println!("soak;summary;load_peak;{peak:.1};C");
    println!("soak;summary;load_plateau;{plateau:.1};C");
    println!(
        "soak;summary;seconds_to_plateau;{};s",
        settled.map(|(time, _)| *time).unwrap_or(LOAD_SECONDS)
    );
    println!(
        "soak;summary;idle_end;{:.1};C",
        readings.last().map(|(_, celsius)| *celsius).unwrap_or(f32::NAN)
    );
    println!("soak;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

/// f32 FMA on 16 independent chains until told to stop.
fn burn(running: &AtomicBool) {
    let mut chains: [f32; 16] = core::array::from_fn(|index| 1.0 + index as f32 * 0.1);
    let (multiplier, addend) = (black_box(0.999_999f32), black_box(1e-7f32));
    while running.load(Ordering::Relaxed) {
        for _ in 0..1_000_000 {
            for chain in chains.iter_mut() {
                *chain = chain.mul_add(multiplier, addend);
            }
        }
        black_box(&chains);
        std::thread::sleep(Duration::from_millis(1)); // let IDLE run: the task watchdog stays fed
    }
}
