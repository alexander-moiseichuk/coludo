//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Phase 6 on-board benchmarks, the software-only part: B1 CPU, B2 FPU, B3 memory, B5 threads and pacing
//! (scratchpad plan, section 2.1). Every result is one console line,
//! `bench;<group>;<name>;<value>;<unit>`, which `tools/avionics_console.py --reset --wait 60` collects.
//!
//! Short intervals are timed with the RISC-V `cycle` counter (the HP core clock, 360 MHz on rev v1.x; it
//! wraps every ~12 s, so nothing timed with it runs that long); longer ones with esp_timer (1 us).

use std::hint::black_box;
use std::sync::mpsc;
use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;
use firmware::thermal::Thermometer;

fn main() {
    esp_idf_svc::sys::link_patches();
    let thermometer = esp_idf_svc::hal::peripherals::Peripherals::take()
        .ok()
        .and_then(|peripherals| Thermometer::new(peripherals.temp_sensor).ok());
    let die = |when: &str| {
        let celsius = thermometer.as_ref().and_then(Thermometer::celsius).unwrap_or(f32::NAN);
        println!("bench;thermal;die_{when};{celsius:.1};C");
    };
    std::thread::sleep(Duration::from_millis(500)); // let the boot log drain first
    die("start");
    let hertz = cycle_hertz();
    report("meta", "cpu_clock", hertz as f32 / 1e6, "MHz");
    cpu(hertz);
    fpu(hertz);
    memory(hertz);
    threads(hertz);
    die("end");
    println!("bench;done");
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

/// Print one result line.
fn report(group: &str, name: &str, value: f32, unit: &str) {
    println!("bench;{group};{name};{value:.3};{unit}");
}

/// The core's cycle counter.
#[inline(always)]
fn cycles() -> u32 {
    let count: u32;
    // SAFETY: reads the user-visible cycle CSR; no side effects.
    unsafe { core::arch::asm!("rdcycle {0}", out(reg) count) };
    count
}

/// Microseconds since boot.
fn micros() -> i64 {
    // SAFETY: a plain read of the monotonic system timer.
    unsafe { sys::esp_timer_get_time() }
}

/// Calibrate the cycle counter against esp_timer over 100 ms: cycles per second.
fn cycle_hertz() -> u64 {
    let (start_cycles, start_micros) = (cycles(), micros());
    while micros() - start_micros < 100_000 {}
    let elapsed_cycles = cycles().wrapping_sub(start_cycles) as u64;
    elapsed_cycles * 1_000_000 / (micros() - start_micros) as u64
}

/// Nanoseconds per call of `body`, the best of 5 runs of `count` calls each (the best run is the one the
/// fewest interrupts landed in).
fn per_call(hertz: u64, count: u32, mut body: impl FnMut()) -> f32 {
    let mut best = u32::MAX;
    for _ in 0..5 {
        let start = cycles();
        for _ in 0..count {
            body();
        }
        best = best.min(cycles().wrapping_sub(start));
    }
    best as f32 * 1e9 / hertz as f32 / count as f32
}

/// The IEEE CRC32 table (the polynomial of zlib / Python binascii.crc32, the recorder wire's CRC).
fn crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    for (index, entry) in table.iter_mut().enumerate() {
        let mut value = index as u32;
        for _ in 0..8 {
            value = if value & 1 != 0 {
                0xEDB8_8320 ^ (value >> 1)
            } else {
                value >> 1
            };
        }
        *entry = value;
    }
    table
}

/// Table-driven CRC32 in software.
fn crc32(table: &[u32; 256], data: &[u8]) -> u32 {
    !data.iter().fold(!0u32, |crc, byte| {
        table[((crc ^ *byte as u32) & 0xff) as usize] ^ (crc >> 8)
    })
}

/// B1: what pure compute costs -- CRC32, a recorder row, a JSON reply.
fn cpu(hertz: u64) {
    let table = crc_table();
    let data: Vec<u8> = (0..65536u32).map(|index| (index * 31 + 7) as u8).collect();
    let software = per_call(hertz, 4, || {
        black_box(crc32(&table, black_box(&data)));
    });
    report("cpu", "crc32_64k_software", software / 1000.0, "us");
    // SAFETY: the ROM routine reads `len` bytes of a live buffer.
    let rom = per_call(hertz, 4, || {
        black_box(unsafe { sys::esp_rom_crc32_le(0, black_box(data.as_ptr()), data.len() as u32) });
    });
    report("cpu", "crc32_64k_rom", rom / 1000.0, "us");
    // SAFETY: as above.
    let same = crc32(&table, &data) == unsafe { sys::esp_rom_crc32_le(0, data.as_ptr(), data.len() as u32) };
    report("cpu", "crc32_rom_matches_zlib", same as u8 as f32, "bool");

    // a 20-field telemetry row, formatted and wrapped the way the recorder wire does it
    let mut row = String::with_capacity(256);
    let row_cost = per_call(hertz, 1000, || {
        use std::fmt::Write;
        row.clear();
        let uptime = black_box(123_456_789u32);
        let _ = write!(
            row,
            "{uptime};{};{};{};{};{}",
            1013.25f32,
            -12.5f32,
            4_500i32,
            1,
            black_box("boosting")
        );
        for field in 0..14 {
            let _ = write!(row, ";{:.2}", black_box(field as f32 * 1.37));
        }
        let open = crc32(&table, row.as_bytes());
        let close = !open ^ uptime;
        black_box((open, close));
    });
    report("cpu", "row_20_fields_format_and_crc", row_cost / 1000.0, "us");

    let json_cost = per_call(hertz, 200, || {
        let info = serde_json::json!({
            "temp": null, "mem_free": black_box(234_567), "uptime": black_box(98_765_432),
            "stage": "setting", "layout": "v1.1", "degraded": ["attitude-backup"],
        });
        black_box(info.to_string());
    });
    report("cpu", "health_json", json_cost / 1000.0, "us");
}

/// B2: f32 (hardware) vs f64 (software) vs i32 fixed-point, per operation. The f64 here exists only to price the
/// rule "no f64 anywhere in the firmware" -- the core has no double-precision unit.
#[expect(
    clippy::too_many_lines,
    reason = "a benchmark: one measurement after another, each printed as it is taken"
)]
fn fpu(hertz: u64) {
    const COUNT: u32 = 10_000;
    let (mut a32, b32) = (1.000_1f32, 0.999_9f32);
    let (mut a64, b64) = (1.000_1f64, 0.999_9f64);
    let (mut a_fixed, b_fixed) = (10_001i32, 9_999i32);
    report(
        "fpu",
        "f32_add",
        per_call(hertz, COUNT, || a32 = black_box(a32 + b32)),
        "ns",
    );
    report(
        "fpu",
        "f32_mul",
        per_call(hertz, COUNT, || a32 = black_box(a32 * b32)),
        "ns",
    );
    report(
        "fpu",
        "f32_div",
        per_call(hertz, COUNT, || a32 = black_box(1.0 / black_box(a32 + 1.5))),
        "ns",
    );
    report(
        "fpu",
        "f32_sqrt",
        per_call(hertz, COUNT, || a32 = black_box(black_box(a32 + 2.0).sqrt())),
        "ns",
    );
    report(
        "fpu",
        "f32_sin",
        per_call(hertz, COUNT, || a32 = black_box(black_box(a32).sin())),
        "ns",
    );
    report(
        "fpu",
        "f32_atan2",
        per_call(hertz, COUNT, || a32 = black_box(black_box(a32).atan2(b32))),
        "ns",
    );
    report(
        "fpu",
        "f64_add",
        per_call(hertz, COUNT, || a64 = black_box(a64 + b64)),
        "ns",
    );
    report(
        "fpu",
        "f64_mul",
        per_call(hertz, COUNT, || a64 = black_box(a64 * b64)),
        "ns",
    );
    report(
        "fpu",
        "f64_div",
        per_call(hertz, COUNT, || a64 = black_box(1.0 / black_box(a64 + 1.5))),
        "ns",
    );
    report(
        "fpu",
        "f64_sqrt",
        per_call(hertz, COUNT, || a64 = black_box(black_box(a64 + 2.0).sqrt())),
        "ns",
    );
    report(
        "fpu",
        "f64_sin",
        per_call(hertz, COUNT, || a64 = black_box(black_box(a64).sin())),
        "ns",
    );
    report(
        "fpu",
        "f64_atan2",
        per_call(hertz, COUNT, || a64 = black_box(black_box(a64).atan2(b64))),
        "ns",
    );
    report(
        "fpu",
        "i32_mul_scale",
        per_call(hertz, COUNT, || a_fixed = black_box(a_fixed * b_fixed / 10_000)),
        "ns",
    );
    report(
        "fpu",
        "i32_div",
        per_call(hertz, COUNT, || a_fixed = black_box(1_000_000 / black_box(a_fixed | 1))),
        "ns",
    );

    // one control step as flight does it: a PID on roll and pitch, the fin mixer, a complementary attitude
    // update -- in f32, to put a number on "the whole step"
    let mut state = [0.0f32; 8];
    let step = per_call(hertz, 1000, || {
        let (gyro_x, gyro_y, accel_x, accel_y, accel_z) = black_box((0.01f32, -0.02f32, 0.1f32, 0.2f32, 9.7f32));
        let dt = 0.0025f32;
        let roll_accel = accel_y.atan2(accel_z);
        let pitch_accel = (-accel_x).atan2((accel_y * accel_y + accel_z * accel_z).sqrt());
        state[0] = 0.98 * (state[0] + gyro_x * dt) + 0.02 * roll_accel;
        state[1] = 0.98 * (state[1] + gyro_y * dt) + 0.02 * pitch_accel;
        for axis in 0..2 {
            let error = black_box(0.05f32) - state[axis];
            state[2 + axis] = (state[2 + axis] + error * dt).clamp(-0.3, 0.3);
            state[4 + axis] = (2.0 * error + 0.5 * state[2 + axis] - 0.1 * gyro_x).clamp(-0.35, 0.35);
        }
        state[6] = (state[4] + state[5]).clamp(-0.35, 0.35);
        state[7] = (state[4] - state[5]).clamp(-0.35, 0.35);
        black_box(&state);
    });
    report("fpu", "control_step_f32", step / 1000.0, "us");
}

/// A heap buffer with the given capabilities (internal RAM or PSRAM), freed on drop.
struct Buffer {
    pointer: *mut u8,
}

impl Buffer {
    fn new(length: usize, capabilities: u32) -> Option<Self> {
        // SAFETY: a plain allocation; null is checked.
        let pointer = unsafe { sys::heap_caps_aligned_alloc(64, length, capabilities) } as *mut u8;
        (!pointer.is_null()).then_some(Self { pointer })
    }
}

impl Drop for Buffer {
    fn drop(&mut self) {
        // SAFETY: allocated by heap_caps_aligned_alloc above, freed once.
        unsafe { sys::heap_caps_free(self.pointer.cast()) };
    }
}

/// B3: copy and fill bandwidth between internal RAM and PSRAM, and the cost of a cache miss.
#[expect(
    clippy::too_many_lines,
    reason = "a benchmark: one measurement after another, each printed as it is taken"
)]
fn memory(hertz: u64) {
    let internal = sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT;
    let psram = sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT;
    // SAFETY: plain heap queries.
    unsafe {
        report(
            "memory",
            "internal_free",
            sys::heap_caps_get_free_size(internal) as f32 / 1024.0,
            "KB",
        );
        report(
            "memory",
            "internal_largest",
            sys::heap_caps_get_largest_free_block(internal) as f32 / 1024.0,
            "KB",
        );
        report(
            "memory",
            "psram_free",
            sys::heap_caps_get_free_size(psram) as f32 / 1024.0,
            "KB",
        );
    }
    for (size, label) in [(4096usize, "4k"), (65536, "64k"), (1 << 20, "1m")] {
        for (source_caps, target_caps, route) in [
            (internal, internal, "internal_to_internal"),
            (psram, psram, "psram_to_psram"),
            (psram, internal, "psram_to_internal"),
            (internal, psram, "internal_to_psram"),
        ] {
            let (Some(source), Some(target)) = (Buffer::new(size, source_caps), Buffer::new(size, target_caps)) else {
                continue; // 1 MB of internal RAM does not exist; that row simply has no number
            };
            let repeats = ((4 << 20) / size) as u32; // move 4 MB per run
            let nanoseconds = per_call(hertz, repeats, || {
                // SAFETY: two live, disjoint buffers of `size` bytes.
                unsafe { core::ptr::copy_nonoverlapping(source.pointer, target.pointer, size) };
                black_box(target.pointer);
            });
            report(
                "memory",
                &format!("copy_{route}_{label}"),
                size as f32 * 1e3 / nanoseconds,
                "MB/s",
            );
        }
        if let Some(target) = Buffer::new(size, psram) {
            let repeats = ((4 << 20) / size) as u32;
            // SAFETY: a live buffer of `size` bytes.
            let nanoseconds = per_call(hertz, repeats, || unsafe {
                core::ptr::write_bytes(target.pointer, 0x5a, size);
                black_box(target.pointer);
            });
            report(
                "memory",
                &format!("fill_psram_{label}"),
                size as f32 * 1e3 / nanoseconds,
                "MB/s",
            );
        }
    }
    // dependent loads over a random cycle, one per 64-byte line: the latency of a miss
    for (caps, label, bytes) in [
        (internal, "internal_128k", 128usize << 10),
        (psram, "psram_4m", 4 << 20),
    ] {
        let lines = bytes / 64;
        let Some(buffer) = Buffer::new(bytes, caps) else {
            continue;
        };
        let slots = buffer.pointer.cast::<u32>();
        let mut order: Vec<u32> = (0..lines as u32).collect();
        let mut seed = 0x1234_5678u32;
        for index in (1..lines).rev() {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            order.swap(index, seed as usize % (index + 1));
        }
        for window in 0..lines {
            // SAFETY: every line index is inside the buffer; slot = line * 16 u32s.
            unsafe { *slots.add(order[window] as usize * 16) = order[(window + 1) % lines] * 16 };
        }
        let mut cursor = 0u32;
        let nanoseconds = per_call(hertz, 100_000, || {
            // SAFETY: cursor always holds a slot index written above.
            cursor = unsafe { *slots.add(cursor as usize) };
        });
        black_box(cursor);
        report("memory", &format!("miss_latency_{label}"), nanoseconds, "ns");
    }
}

/// Percentile (0..=1000 per mille) of an unsorted sample, in place.
fn per_mille(samples: &mut [u32], rank: usize) -> u32 {
    samples.sort_unstable();
    samples[((samples.len() - 1) * rank / 1000).min(samples.len() - 1)]
}

/// Report a latency sample (cycles) as p50 / p99 / p99.9 / max microseconds.
fn report_latency(group: &str, name: &str, hertz: u64, samples: &mut [u32]) {
    let micros = |value: u32| value as f32 * 1e6 / hertz as f32;
    report(group, &format!("{name}_p50"), micros(per_mille(samples, 500)), "us");
    report(group, &format!("{name}_p99"), micros(per_mille(samples, 990)), "us");
    report(group, &format!("{name}_p999"), micros(per_mille(samples, 999)), "us");
    report(group, &format!("{name}_max"), micros(per_mille(samples, 1000)), "us");
}

/// Spawn the next std thread pinned to `core` at FreeRTOS `priority`.
fn pin_next_thread(core: Core, priority: u8) {
    ThreadSpawnConfiguration {
        pin_to_core: Some(core),
        priority,
        ..Default::default()
    }
    .set()
    .expect("thread config");
}

/// B5: thread round trips (same core, across cores) and loop pacing at 400 Hz.
fn threads(hertz: u64) {
    const ROUND_TRIPS: usize = 5000;
    for (far_core, label) in [(Core::Core0, "same_core"), (Core::Core1, "cross_core")] {
        pin_next_thread(far_core, 10);
        let (to_far, from_near) = mpsc::sync_channel::<u32>(1);
        let (to_near, from_far) = mpsc::sync_channel::<u32>(1);
        let echo = std::thread::spawn(move || {
            while let Ok(value) = from_near.recv() {
                if to_near.send(value).is_err() {
                    break;
                }
            }
        });
        pin_next_thread(Core::Core0, 10);
        let measure = std::thread::spawn(move || {
            let mut samples = Vec::with_capacity(ROUND_TRIPS);
            for round in 0..ROUND_TRIPS as u32 {
                let start = cycles();
                to_far.send(round).unwrap();
                from_far.recv().unwrap();
                samples.push(cycles().wrapping_sub(start));
            }
            drop(to_far);
            samples
        });
        let mut samples = measure.join().unwrap();
        echo.join().unwrap();
        report_latency("threads", &format!("channel_round_trip_{label}"), hertz, &mut samples);
    }

    // 400 Hz pacing on core 1 at a flight-like priority: lateness of each wake against its deadline
    const PERIOD_MICROS: i64 = 2500;
    const TICKS: usize = 4000; // 10 s
    pin_next_thread(Core::Core1, 20);
    let sleeper = std::thread::spawn(|| {
        let mut lateness = Vec::with_capacity(TICKS);
        let mut deadline = micros() + PERIOD_MICROS;
        for _ in 0..TICKS {
            let wait = deadline - micros();
            if wait > 0 {
                std::thread::sleep(Duration::from_micros(wait as u64));
            }
            lateness.push((micros() - deadline).max(0) as u32);
            deadline += PERIOD_MICROS;
        }
        lateness
    });
    let mut lateness = sleeper.join().unwrap();
    // lateness is in microseconds here; report_latency takes cycles, so scale by the clock
    let scale = (hertz / 1_000_000) as u32;
    lateness.iter_mut().for_each(|value| *value *= scale);
    report_latency("pacing", "sleep_400hz_lateness", hertz, &mut lateness);

    for (far_core, label) in [(Core::Core0, "same_core"), (Core::Core1, "cross_core")] {
        let mut samples = notify_round_trips(far_core, ROUND_TRIPS);
        report_latency("threads", &format!("notify_round_trip_{label}"), hertz, &mut samples);
    }

    pin_next_thread(Core::Core1, 20);
    let paced = std::thread::spawn(|| timer_paced(PERIOD_MICROS as u64, TICKS));
    let mut lateness = paced.join().unwrap();
    lateness.iter_mut().for_each(|value| *value *= scale);
    report_latency("pacing", "esp_timer_notify_400hz_lateness", hertz, &mut lateness);
    ThreadSpawnConfiguration::default().set().expect("thread config");
}

/// This task's FreeRTOS handle, as a plain number so it can cross threads.
fn current_task() -> usize {
    // SAFETY: a plain query of the scheduler.
    unsafe { sys::xTaskGetCurrentTaskHandle() as usize }
}

/// Give `task` one notification (the lightest FreeRTOS wake-up: no queue, no semaphore object).
fn notify(task: usize) {
    // SAFETY: `task` is a live task handle for as long as the test runs.
    unsafe {
        sys::xTaskGenericNotify(
            task as sys::TaskHandle_t,
            0,
            0,
            sys::eNotifyAction_eIncrement,
            core::ptr::null_mut(),
        )
    };
}

/// Block until this task is notified.
fn wait_notified() {
    // SAFETY: waits on the calling task's own notification slot 0.
    unsafe { sys::ulTaskGenericNotifyTake(0, 1, u32::MAX) };
}

/// Raw task-notification ping-pong with an echo task on `far_core`: cycles per round trip.
fn notify_round_trips(far_core: Core, count: usize) -> Vec<u32> {
    let (echo_out, echo_in) = mpsc::channel::<usize>(); // the echo's handle, to the measuring task
    let (near_out, near_in) = mpsc::channel::<usize>(); // the measuring task's handle, to the echo
    pin_next_thread(far_core, 10);
    let echo = std::thread::spawn(move || {
        echo_out.send(current_task()).unwrap();
        let near = near_in.recv().unwrap();
        for _ in 0..count {
            wait_notified();
            notify(near);
        }
    });
    pin_next_thread(Core::Core0, 10);
    let measure = std::thread::spawn(move || {
        let echo_task = echo_in.recv().unwrap();
        near_out.send(current_task()).unwrap();
        std::thread::sleep(Duration::from_millis(10)); // let the echo block on its notification first
        let mut samples = Vec::with_capacity(count);
        for _ in 0..count {
            let start = cycles();
            notify(echo_task);
            wait_notified();
            samples.push(cycles().wrapping_sub(start));
        }
        samples
    });
    let samples = measure.join().unwrap();
    echo.join().unwrap();
    samples
}

/// An esp_timer callback: notify the task whose handle is `argument`.
unsafe extern "C" fn timer_tick(argument: *mut core::ffi::c_void) {
    notify(argument as usize);
}

/// Pace `ticks` wake-ups at `period` us with a periodic esp_timer notifying this task: lateness in us.
fn timer_paced(period: u64, ticks: usize) -> Vec<u32> {
    let arguments = sys::esp_timer_create_args_t {
        callback: Some(timer_tick),
        arg: current_task() as *mut core::ffi::c_void,
        dispatch_method: sys::esp_timer_dispatch_t_ESP_TIMER_TASK,
        name: c"pace".as_ptr(),
        skip_unhandled_events: false,
    };
    let mut timer: sys::esp_timer_handle_t = core::ptr::null_mut();
    let mut lateness = Vec::with_capacity(ticks);
    // SAFETY: create/start/stop/delete a timer owned by this function; the task it notifies is this one.
    unsafe {
        sys::esp!(sys::esp_timer_create(&arguments, &mut timer)).expect("timer");
        let start = micros();
        sys::esp!(sys::esp_timer_start_periodic(timer, period)).expect("timer start");
        for tick in 1..=ticks as i64 {
            wait_notified();
            lateness.push((micros() - start - tick * period as i64).max(0) as u32);
        }
        sys::esp_timer_stop(timer);
        sys::esp_timer_delete(timer);
    }
    lateness
}
