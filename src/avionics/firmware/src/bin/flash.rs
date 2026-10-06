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
//! The build's sdkconfig decides which variant is measured (XIP from PSRAM, the default since B4; code in flash
//! with CONFIG_SPIRAM_XIP_FROM_PSRAM=n): the `meta;xip_from_psram` line says which.

use std::hint::black_box;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;
use firmware::thermal::Thermometer;

/// The stall test's loop period.
const PERIOD_MICROS: u64 = 1000;
/// Ticks per stall phase: 10 s.
const TICKS: usize = 10_000;

fn main() {
    esp_idf_svc::sys::link_patches();
    let thermometer = esp_idf_svc::hal::peripherals::Peripherals::take()
        .ok()
        .and_then(|peripherals| Thermometer::new(peripherals.temp_sensor).ok());
    let die = |when: &str| {
        let celsius = thermometer.as_ref().and_then(Thermometer::celsius).unwrap_or(f32::NAN);
        println!("flash;thermal;die_{when};{celsius:.1};C");
    };
    std::thread::sleep(Duration::from_millis(300));
    die("start");
    report(
        "meta",
        "xip_from_psram",
        cfg!(esp_idf_spiram_xip_from_psram) as u8 as f32,
        "bool",
    );
    let partition = log_partition();
    // SAFETY: a found partition descriptor lives for the program.
    let size = unsafe { (*partition.0).size } as usize;
    report("meta", "partition_size", size as f32 / 1024.0, "KB");
    latencies(partition);
    above_16_megabytes(partition);
    stall(partition, false);
    stall(partition, true);
    let start = micros();
    // SAFETY: erases inside the bench partition only.
    unsafe { sys::esp!(sys::esp_partition_erase_range(partition.0, 0, size)) }.expect("erase");
    report(
        "throughput",
        "erase_whole_partition",
        (micros() - start) as f32 / 1e6,
        "s",
    );
    die("end");
    println!("flash;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

fn report(group: &str, name: &str, value: f32, unit: &str) {
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
            timings.push((micros() - start) as f32 / 1000.0);
        }
        report_spread("latency", "erase_4k", &mut timings, "ms");
        timings.clear();
        for block in 1..9 {
            let start = micros();
            sys::esp!(sys::esp_partition_erase_range(partition.0, block * 65536, 65536)).expect("erase");
            timings.push((micros() - start) as f32 / 1000.0);
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
            timings.push((micros() - start) as f32);
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
            512.0 * 1e6 / 1024.0 / (micros() - start) as f32,
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
        report("throughput", "read", 1.0 * 1e6 / (micros() - start) as f32, "MB/s");
        report(
            "integrity",
            "readback_matches",
            readback.iter().take(4096).all(|byte| *byte == 0x5a) as u8 as f32,
            "bool",
        );
    }
}

/// Above 16 MB the flash needs 4-byte addressing; with 3-byte commands a write there would wrap 16 MB lower --
/// onto the app. Write different 4 KB patterns exactly 16 MB apart and read both back.
fn above_16_megabytes(partition: Partition) {
    // SAFETY: a found partition descriptor lives for the program.
    let (base, size) = unsafe { ((*partition.0).address as usize, (*partition.0).size as usize) };
    let low = 0x10_0000; // partition offset 1 MB
    let high = low + 0x100_0000; // 16 MB further on
    if high + 4096 > size {
        report("boundary", "skipped_partition_too_small", size as f32 / 1048576.0, "MB");
        return;
    }
    let (pattern_low, pattern_high) = (vec![0x3cu8; 4096], vec![0xc3u8; 4096]);
    let mut back = vec![0u8; 4096];
    // SAFETY: inside the bench partition, live buffers.
    let identical = unsafe {
        sys::esp!(sys::esp_partition_erase_range(partition.0, low, 4096)).expect("erase");
        sys::esp!(sys::esp_partition_erase_range(partition.0, high, 4096)).expect("erase");
        sys::esp!(sys::esp_partition_write_raw(partition.0, low, pattern_low.as_ptr().cast(), 4096)).expect("write");
        sys::esp!(sys::esp_partition_write_raw(partition.0, high, pattern_high.as_ptr().cast(), 4096)).expect("write");
        sys::esp!(sys::esp_partition_read(partition.0, low, back.as_mut_ptr().cast(), 4096)).expect("read");
        let low_intact = back == pattern_low;
        sys::esp!(sys::esp_partition_read(partition.0, high, back.as_mut_ptr().cast(), 4096)).expect("read");
        (low_intact, back == pattern_high)
    };
    report("boundary", "high_absolute_address", (base + high) as f32 / 1048576.0, "MB");
    report("boundary", "low_kept_its_pattern", identical.0 as u8 as f32, "bool");
    report("boundary", "high_kept_its_pattern", identical.1 as u8 as f32, "bool");
}

/// Sort `values` and report min / p50 / max.
fn report_spread(group: &str, name: &str, values: &mut [f32], unit: &str) {
    values.sort_by(f32::total_cmp);
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
            // above 16 MB absolute when the partition reaches there (4-byte addressing under load), else past
            // the region latencies() used
            // SAFETY: a found partition descriptor lives for the program.
            let (base, size) = unsafe { ((*partition.raw()).address as usize, (*partition.raw()).size as usize) };
            let first = if base + size > 0x100_0000 + 128 * 65536 { (0x100_0000 - base.min(0x100_0000)) / 65536 } else { 16 };
            let last = first + 111;
            let mut block = first;
            while running.load(Ordering::Relaxed) {
                // SAFETY: inside the bench partition (blocks first..=last of 64 KB), with a live buffer.
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
                block = if block >= last { first } else { block + 1 };
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
    let at = |rank: usize| lateness[(lateness.len() - 1) * rank / 1000] as f32;
    report("stall", &format!("{label}_lateness_p50"), at(500), "us");
    report("stall", &format!("{label}_lateness_p99"), at(990), "us");
    report("stall", &format!("{label}_lateness_p999"), at(999), "us");
    report("stall", &format!("{label}_lateness_max"), at(1000), "us");
    let missed = lateness.iter().filter(|value| **value as u64 >= PERIOD_MICROS).count();
    report("stall", &format!("{label}_missed_ticks"), missed as f32, "ticks");
    report(
        "stall",
        &format!("{label}_blocks_written"),
        operations.load(Ordering::Relaxed) as f32,
        "64KB",
    );
}
