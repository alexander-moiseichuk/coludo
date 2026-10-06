//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! B4: can the on-board flash take telemetry in flight without stalling the control loop? Runs on the `log`
//! partition (partitions.csv) and prints `flash;<group>;<name>;<value>;<unit>` lines:
//!  * latency of a 4 KB sector erase, a 64 KB block erase and a 256 B page program, and the sustained program
//!    rate into erased flash;
//!  * the stall test: a 1 kHz loop on core 1 (esp_timer -> task notify, priority 20, executing ordinary code)
//!    records how late each tick wakes while core 0 erases and programs continuously -- and the same loop with
//!    the flash idle, as the reference;
//!  * the time to erase the whole partition (the pad's pre-erase).
//!
//! The build's sdkconfig decides which variant is measured (code in flash by default; XIP from PSRAM with
//! sdkconfig.xip): the `meta;xip_from_psram` line says which.

use std::hint::black_box;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;

/// The stall test's loop period.
const PERIOD_MICROS: u64 = 1000;
/// Ticks per stall phase: 10 s.
const TICKS: usize = 10_000;

fn main() {
    esp_idf_svc::sys::link_patches();
    std::thread::sleep(Duration::from_millis(300));
    report(
        "meta",
        "xip_from_psram",
        cfg!(esp_idf_spiram_xip_from_psram) as u8 as f64,
        "bool",
    );
    let partition = log_partition();
    // SAFETY: a found partition descriptor lives for the program.
    let size = unsafe { (*partition.0).size } as usize;
    report("meta", "partition_size", size as f64 / 1024.0, "KB");
    latencies(partition);
    stall(partition, false);
    stall(partition, true);
    let start = micros();
    // SAFETY: erases inside the bench partition only.
    unsafe { sys::esp!(sys::esp_partition_erase_range(partition.0, 0, size)) }.expect("erase");
    report(
        "throughput",
        "erase_whole_partition",
        (micros() - start) as f64 / 1e6,
        "s",
    );
    println!("flash;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

fn report(group: &str, name: &str, value: f64, unit: &str) {
    println!("flash;{group};{name};{value:.3};{unit}");
}

fn micros() -> i64 {
    // SAFETY: a plain read of the system timer.
    unsafe { sys::esp_timer_get_time() }
}

/// The `log` data partition, as a shareable handle.
#[derive(Clone, Copy)]
struct Partition(*const sys::esp_partition_t);
unsafe impl Send for Partition {}

impl Partition {
    /// The descriptor (a method, so a closure captures the whole Send handle, not the raw field).
    fn raw(self) -> *const sys::esp_partition_t {
        self.0
    }
}

fn log_partition() -> Partition {
    // SAFETY: a lookup by type/subtype/label; the result is checked.
    let partition = unsafe {
        sys::esp_partition_find_first(
            sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
            sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
            c"log".as_ptr(),
        )
    };
    assert!(!partition.is_null(), "no `log` partition -- flash with partitions.csv");
    Partition(partition)
}

/// Erase, program and read latencies, and the sustained program rate.
fn latencies(partition: Partition) {
    let page = vec![0x5au8; 4096];
    let mut timings = Vec::new();
    // SAFETY (whole block): every call stays inside the bench partition with live buffers.
    unsafe {
        for sector in 0..16 {
            let start = micros();
            sys::esp!(sys::esp_partition_erase_range(partition.0, sector * 4096, 4096)).expect("erase");
            timings.push((micros() - start) as f64 / 1000.0);
        }
        report_spread("latency", "erase_4k", &mut timings, "ms");
        timings.clear();
        for block in 1..9 {
            let start = micros();
            sys::esp!(sys::esp_partition_erase_range(partition.0, block * 65536, 65536)).expect("erase");
            timings.push((micros() - start) as f64 / 1000.0);
        }
        report_spread("latency", "erase_64k", &mut timings, "ms");
        timings.clear();
        for index in 0..64 {
            let start = micros();
            sys::esp!(sys::esp_partition_write_raw(
                partition.0,
                65536 + index * 256,
                page.as_ptr().cast(),
                256
            ))
            .expect("program");
            timings.push((micros() - start) as f64);
        }
        report_spread("latency", "program_256b", &mut timings, "us");
        // sustained: 512 KB of erased flash in 4 KB programs
        let start = micros();
        for index in 0..128 {
            sys::esp!(sys::esp_partition_write_raw(
                partition.0,
                65536 * 2 + index * 4096,
                page.as_ptr().cast(),
                4096
            ))
            .expect("program");
        }
        report(
            "throughput",
            "program_sustained",
            512.0 * 1e6 / 1024.0 / (micros() - start) as f64,
            "MB/s",
        );
        let mut readback = vec![0u8; 65536];
        let start = micros();
        for _ in 0..16 {
            sys::esp!(sys::esp_partition_read(
                partition.0,
                65536 * 2,
                readback.as_mut_ptr().cast(),
                65536
            ))
            .expect("read");
        }
        report("throughput", "read", 1.0 * 1e6 / (micros() - start) as f64, "MB/s");
        report(
            "integrity",
            "readback_matches",
            readback.iter().take(4096).all(|byte| *byte == 0x5a) as u8 as f64,
            "bool",
        );
    }
}

/// Sort `values` and report min / p50 / max.
fn report_spread(group: &str, name: &str, values: &mut [f64], unit: &str) {
    values.sort_by(f64::total_cmp);
    report(group, &format!("{name}_min"), values[0], unit);
    report(group, &format!("{name}_p50"), values[values.len() / 2], unit);
    report(group, &format!("{name}_max"), values[values.len() - 1], unit);
}

fn current_task() -> usize {
    // SAFETY: a plain scheduler query.
    unsafe { sys::xTaskGetCurrentTaskHandle() as usize }
}

unsafe extern "C" fn tick(argument: *mut core::ffi::c_void) {
    // SAFETY: the argument is the paced task's handle, alive while the timer runs.
    unsafe {
        sys::xTaskGenericNotify(
            argument as sys::TaskHandle_t,
            0,
            0,
            sys::eNotifyAction_eIncrement,
            core::ptr::null_mut(),
        )
    };
}

/// The stall test: a 1 kHz paced loop on core 1, with or without core 0 writing flash meanwhile.
fn stall(partition: Partition, writing: bool) {
    let label = if writing { "flash_busy" } else { "flash_idle" };
    let running = Arc::new(AtomicBool::new(true));
    let operations = Arc::new(AtomicU32::new(0));
    let writer = writing.then(|| {
        let (running, operations) = (running.clone(), operations.clone());
        ThreadSpawnConfiguration {
            pin_to_core: Some(Core::Core0),
            priority: 5,
            ..Default::default()
        }
        .set()
        .unwrap();
        std::thread::spawn(move || {
            let page = vec![0xa5u8; 4096];
            let mut block = 16usize; // past the region latencies() used
            while running.load(Ordering::Relaxed) {
                // SAFETY: inside the bench partition (blocks 16..127 of 64 KB), with a live buffer.
                unsafe {
                    sys::esp!(sys::esp_partition_erase_range(partition.raw(), block * 65536, 65536)).expect("erase");
                    for index in 0..16 {
                        sys::esp!(sys::esp_partition_write_raw(
                            partition.raw(),
                            block * 65536 + index * 4096,
                            page.as_ptr().cast(),
                            4096
                        ))
                        .expect("program");
                    }
                }
                operations.fetch_add(1, Ordering::Relaxed);
                block = if block >= 127 { 16 } else { block + 1 };
            }
        })
    });
    ThreadSpawnConfiguration {
        pin_to_core: Some(Core::Core1),
        priority: 20,
        ..Default::default()
    }
    .set()
    .unwrap();
    let paced = std::thread::spawn(|| {
        let arguments = sys::esp_timer_create_args_t {
            callback: Some(tick),
            arg: current_task() as *mut core::ffi::c_void,
            dispatch_method: sys::esp_timer_dispatch_t_ESP_TIMER_TASK,
            name: c"stall".as_ptr(),
            skip_unhandled_events: false,
        };
        let mut timer: sys::esp_timer_handle_t = core::ptr::null_mut();
        let mut lateness = Vec::with_capacity(TICKS);
        let mut work = 0u32;
        // SAFETY: a timer owned by this function, notifying this task.
        unsafe {
            sys::esp!(sys::esp_timer_create(&arguments, &mut timer)).expect("timer");
            let start = micros();
            sys::esp!(sys::esp_timer_start_periodic(timer, PERIOD_MICROS)).expect("timer");
            let mut tick = 0i64;
            while lateness.len() < TICKS {
                // a burst of wake-ups after a stall counts as one late tick each
                let pending = sys::ulTaskGenericNotifyTake(0, 1, u32::MAX) as i64;
                for _ in 0..pending {
                    tick += 1;
                    lateness.push((micros() - start - tick * PERIOD_MICROS as i64).max(0) as u32);
                }
                for step in 0..200u32 {
                    work = black_box(work.wrapping_mul(31).wrapping_add(step)); // ordinary code, from flash or PSRAM
                }
            }
            sys::esp_timer_stop(timer);
            sys::esp_timer_delete(timer);
        }
        black_box(work);
        lateness
    });
    let mut lateness = paced.join().unwrap();
    ThreadSpawnConfiguration::default().set().unwrap();
    running.store(false, Ordering::Relaxed);
    if let Some(writer) = writer {
        writer.join().unwrap();
    }
    lateness.sort_unstable();
    let at = |rank: usize| lateness[(lateness.len() - 1) * rank / 1000] as f64;
    report("stall", &format!("{label}_lateness_p50"), at(500), "us");
    report("stall", &format!("{label}_lateness_p99"), at(990), "us");
    report("stall", &format!("{label}_lateness_p999"), at(999), "us");
    report("stall", &format!("{label}_lateness_max"), at(1000), "us");
    let missed = lateness.iter().filter(|value| **value as u64 >= PERIOD_MICROS).count();
    report("stall", &format!("{label}_missed_ticks"), missed as f64, "ticks");
    report(
        "stall",
        &format!("{label}_blocks_written"),
        operations.load(Ordering::Relaxed) as f64,
        "64KB",
    );
}
