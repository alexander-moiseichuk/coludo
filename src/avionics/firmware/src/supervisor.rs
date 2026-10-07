//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The supervisor thread (doc/specs/avionics-port.md "Threads, cores, priorities": core 0, priority 9). For now it
//! writes health.csv at 1 Hz, plan B's board_health columns unchanged (spec "Row, log and index text"), so
//! recorder_flight, flight_report and flight_video read the Rust board's vitals as they read plan B's. The watchdog,
//! the checkpoint, gnss_calib and the field logic join it as they are ported (M8, M11).
//!
//! Its neighbours in the record need nothing from it: the ring_writer logs the `'session': '<s>'` status line every
//! `recorder.stats_ms` itself, and writes session.csv's `boot` and `anchor` rows; a clock set writes its own row
//! (commands/inspection.rs).

use std::sync::Arc;
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::sys;
use recorder::{Cell, Telemetry};

use crate::board::{self, Board};
use crate::recorder::Producer;

/// The health stream's file: `<session>_health.csv` on the SD.
pub const HEALTH_FILE: &str = "health.csv";

/// The health stream's columns after `uptime` (plan B tasks/board_health.py, doc/telemetry.md):
/// * `temp` - the die temperature in °C, to 0.1;
/// * `mem_free` - free 8-bit heap, internal and PSRAM together, bytes;
/// * `load` - the busiest core's CPU load, whole percent;
/// * `oom_s`, `land_s`, `leak_kbps`, `rescues`, `rescue_ms` - plan B's memory forecast and rescue, which Rust has no
///   use for (no GC), and the landing forecast that comes with the elevation slope: `None` and 0, as the spec says.
pub const HEALTH_FIELDS: [&str; 8] = [
    "temp",
    "mem_free",
    "load",
    "oom_s",
    "land_s",
    "leak_kbps",
    "rescues",
    "rescue_ms",
];

/// The supervisor's period: health.csv at 1 Hz, as plan B's board_health.
const PERIOD: Duration = Duration::from_secs(1);

/// Its priority on core 0 (spec "Threads, cores, priorities").
const PRIORITY: u8 = 9;

/// Its stack: one row of eight cells at a time.
const STACK_SIZE: usize = 6 << 10;

/// Spawns the supervisor (core 0, priority 9). It needs the recorder: health.csv goes nowhere without one.
///
/// Args:
/// * `board` - the vitals it samples (the die temperature).
///
/// Errors: no recorder, or the thread would not spawn.
pub fn start(board: Arc<Board>) -> anyhow::Result<()> {
    let (Some(recorder), Some(health)) = (crate::recorder::get(), crate::recorder::health_stream()) else {
        anyhow::bail!("no recorder: health.csv is not written");
    };
    ThreadSpawnConfiguration {
        name: Some(c"supervisor"),
        stack_size: STACK_SIZE,
        priority: PRIORITY,
        pin_to_core: Some(Core::Core0),
        // stack in PSRAM (avionics-port.md §4.7, internal RAM budget): a slow core-0 thread that never calls
        // esp_mmu_map/unmap, the one runtime path that turns the cache off
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(STACK_SIZE).spawn(move || {
        let _attachment = recorder
            .attach(Producer::Supervisor as usize)
            .inspect_err(|error| log::warn!("supervisor :: health.csv shares the misc ring: {error:?}"))
            .ok();
        let Some(mut telemetry) = Telemetry::new(recorder, health) else {
            log::error!("supervisor :: the registry has no health.csv");
            return;
        };
        let mut load = Load::new();
        let mut overflowing = false;
        loop {
            std::thread::sleep(PERIOD);
            let now_us = uptime_us();
            let row = [
                board.celsius().map_or(Cell::None, Cell::F32),
                Cell::I64(board::heap_free(sys::MALLOC_CAP_8BIT) as i64),
                Cell::I32(load.busiest_percent(now_us)),
                Cell::None,   // oom_s
                Cell::None,   // land_s
                Cell::I32(0), // leak_kbps
                Cell::I32(0), // rescues
                Cell::I32(0), // rescue_ms
            ];
            // telemetry raises (the error policy): the recorder counts the row in tlm.dropped, and the first of a run
            // of refusals is logged
            match telemetry.push(now_us, &row) {
                Ok(()) => overflowing = false,
                Err(overflow) if !overflowing => {
                    log::warn!("supervisor :: health.csv row: {overflow}");
                    overflowing = true;
                }
                Err(_) => {}
            }
        }
    });
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}

/// The busiest core's CPU load between two samples, from each core's idle task run time (FreeRTOS run-time stats
/// on esp_timer, so the counters are µs, in 64 bits: CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64).
struct Load {
    /// Each core's idle run time at the last sample.
    idle: [u64; 2],
    /// When the last sample was taken, µs.
    sampled_us: u64,
}

impl Load {
    /// Counting from now.
    fn new() -> Load {
        Load {
            idle: idle_times(),
            sampled_us: uptime_us(),
        }
    }

    /// The busiest core's load since the last call, in whole percent (0 for a core whose idle task ran throughout).
    fn busiest_percent(&mut self, now_us: u64) -> i32 {
        let idle = idle_times();
        let elapsed_us = now_us.saturating_sub(self.sampled_us).max(1);
        let busiest = (0..2)
            .map(|core| {
                let idle_us = idle[core].saturating_sub(self.idle[core]).min(elapsed_us);
                100 - (idle_us * 100 / elapsed_us) as i32
            })
            .max()
            .unwrap_or(0);
        self.idle = idle;
        self.sampled_us = now_us;
        busiest
    }
}

/// Each core's idle task run time so far, µs.
fn idle_times() -> [u64; 2] {
    [0, 1].map(|core| {
        let mut status = sys::TaskStatus_t::default();
        // SAFETY: the idle task of each of the two cores always exists; vTaskGetInfo fills the struct it is handed
        // and keeps no reference to it (no stack scan, the state left for it to look up).
        unsafe {
            let idle = sys::xTaskGetIdleTaskHandleForCore(core);
            sys::vTaskGetInfo(idle, &mut status, 0, sys::eTaskState_eInvalid);
        }
        status.ulRunTimeCounter
    })
}

/// The uptime in µs (esp_timer, unwrapped): the row's stamp.
fn uptime_us() -> u64 {
    // SAFETY: a plain read of the monotonic system timer, never negative.
    unsafe { sys::esp_timer_get_time() as u64 }
}
