//! Operator diagnostics on the console: `load [seconds]` -- per-core CPU load and the busiest tasks over a window;
//! `pace [seconds] [gptimer]` -- a 1 kHz loop on core 1 (priority 20, the flight thread's place) woken by an
//! esp_timer (its task lives on core 0) or by a gptimer interrupt allocated on core 1, and how late each tick
//! wakes. Run them while the camera records to price video against the flight loop.

use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;

/// One task's run time at a moment.
struct Sample {
    name: String,
    core: i32,
    runtime: u64,
}

/// Every task's accumulated run time (µs) and the total.
fn snapshot() -> (Vec<Sample>, u64) {
    // SAFETY: sizes the array from the scheduler's own count (+ slack for tasks created meanwhile).
    let capacity = unsafe { sys::uxTaskGetNumberOfTasks() } as usize + 8;
    let mut status: Vec<sys::TaskStatus_t> = Vec::with_capacity(capacity);
    let mut total = 0;
    // SAFETY: the array has `capacity` slots; the call fills at most that many and returns the count.
    let count = unsafe { sys::uxTaskGetSystemState(status.as_mut_ptr(), capacity as u32, &mut total) } as usize;
    // SAFETY: `count` entries were written.
    unsafe { status.set_len(count) };
    let samples = status
        .iter()
        .map(|task| Sample {
            // SAFETY: the scheduler's task name is a NUL-terminated string.
            name: unsafe { std::ffi::CStr::from_ptr(task.pcTaskName) }
                .to_string_lossy()
                .into_owned(),
            core: task.xCoreID as i32,
            runtime: task.ulRunTimeCounter as u64,
        })
        .collect();
    (samples, total as u64)
}

/// Per-core load over `seconds` and the busiest tasks, as one CC line.
pub fn load(seconds: u64) -> String {
    let (before, _) = snapshot();
    let start = unsafe { sys::esp_timer_get_time() };
    std::thread::sleep(Duration::from_secs(seconds));
    let (after, _) = snapshot();
    let elapsed = (unsafe { sys::esp_timer_get_time() } - start) as u64;
    let mut deltas: Vec<(String, i32, u64)> = after
        .iter()
        .map(|task| {
            let earlier = before
                .iter()
                .find(|old| old.name == task.name)
                .map(|old| old.runtime)
                .unwrap_or(0);
            (task.name.clone(), task.core, task.runtime.saturating_sub(earlier))
        })
        .collect();
    let idle = |core: &str| {
        deltas
            .iter()
            .find(|(name, _, _)| name == core)
            .map(|(_, _, delta)| *delta)
            .unwrap_or(0)
    };
    let busy = |idle_time: u64| 100.0 - idle_time as f32 * 100.0 / elapsed as f32;
    let (core0, core1) = (busy(idle("IDLE0")), busy(idle("IDLE1")));
    deltas.retain(|(name, _, _)| !name.starts_with("IDLE"));
    deltas.sort_by(|left, right| right.2.cmp(&left.2));
    let top: Vec<String> = deltas
        .iter()
        .take(6)
        .map(|(name, core, delta)| format!("{name}@{core}:{:.1}", *delta as f32 * 100.0 / elapsed as f32))
        .collect();
    cc_protocol::build(
        "ok",
        &["load"],
        &[
            ("core0_pct", &format!("{core0:.1}")),
            ("core1_pct", &format!("{core1:.1}")),
            ("top", &top.join(",")),
        ],
    )
}

/// What wakes the paced loop.
#[derive(Clone, Copy)]
pub enum Source {
    /// An esp_timer periodic callback (dispatched by the esp_timer task, on core 0).
    EspTimer,
    /// A gptimer alarm interrupt, allocated on core 1 next to the loop.
    Gptimer,
}

/// gptimer alarm ISR: notify the paced task. Integer only (a float in an ISR aborts).
unsafe extern "C" fn alarm(
    _timer: sys::gptimer_handle_t,
    _event: *const sys::gptimer_alarm_event_data_t,
    task: *mut core::ffi::c_void,
) -> bool {
    let mut woken: sys::BaseType_t = 0;
    // SAFETY: `task` is the paced thread's handle, alive while the timer runs.
    unsafe {
        sys::xTaskGenericNotifyFromISR(
            task as sys::TaskHandle_t,
            0,
            0,
            sys::eNotifyAction_eIncrement,
            core::ptr::null_mut(),
            &mut woken,
        )
    };
    woken != 0
}

unsafe extern "C" fn tick(task: *mut core::ffi::c_void) {
    // SAFETY: `task` is the paced thread's handle, alive while the timer runs.
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

/// A 1 kHz loop on core 1 for `seconds`, woken by `source`: how late each tick woke, as one CC line.
pub fn pace(seconds: u64, source: Source) -> String {
    const PERIOD_MICROSECONDS: u64 = 1000;
    let ticks = (seconds * 1_000_000 / PERIOD_MICROSECONDS) as usize;
    ThreadSpawnConfiguration {
        pin_to_core: Some(Core::Core1),
        priority: 20,
        ..Default::default()
    }
    .set()
    .ok();
    let paced = std::thread::spawn(move || {
        let task = unsafe { sys::xTaskGetCurrentTaskHandle() } as *mut core::ffi::c_void;
        let mut lateness = Vec::with_capacity(ticks);
        // SAFETY: a timer owned by this thread, notifying this thread, torn down before returning; created here,
        // on core 1, so a gptimer's interrupt is allocated on core 1.
        unsafe {
            let mut esp_timer: sys::esp_timer_handle_t = core::ptr::null_mut();
            let mut gptimer: sys::gptimer_handle_t = core::ptr::null_mut();
            match source {
                Source::EspTimer => {
                    let arguments = sys::esp_timer_create_args_t {
                        callback: Some(tick),
                        arg: task,
                        dispatch_method: sys::esp_timer_dispatch_t_ESP_TIMER_TASK,
                        name: c"pace".as_ptr(),
                        skip_unhandled_events: false,
                    };
                    if sys::esp_timer_create(&arguments, &mut esp_timer) != sys::ESP_OK {
                        return lateness;
                    }
                    sys::esp_timer_start_periodic(esp_timer, PERIOD_MICROSECONDS);
                }
                Source::Gptimer => {
                    let config = sys::gptimer_config_t {
                        clk_src: sys::soc_periph_gptimer_clk_src_t_GPTIMER_CLK_SRC_DEFAULT,
                        direction: sys::gptimer_count_direction_t_GPTIMER_COUNT_UP,
                        resolution_hz: 1_000_000,
                        ..Default::default()
                    };
                    let callbacks = sys::gptimer_event_callbacks_t { on_alarm: Some(alarm) };
                    let mut alarm_config = sys::gptimer_alarm_config_t {
                        alarm_count: PERIOD_MICROSECONDS,
                        ..Default::default()
                    };
                    alarm_config.flags.set_auto_reload_on_alarm(1);
                    if sys::gptimer_new_timer(&config, &mut gptimer) != sys::ESP_OK
                        || sys::gptimer_register_event_callbacks(gptimer, &callbacks, task) != sys::ESP_OK
                        || sys::gptimer_set_alarm_action(gptimer, &alarm_config) != sys::ESP_OK
                        || sys::gptimer_enable(gptimer) != sys::ESP_OK
                        || sys::gptimer_start(gptimer) != sys::ESP_OK
                    {
                        return lateness;
                    }
                }
            }
            // lateness against an ideal grid anchored at the first tick (both sources share the crystal)
            let mut first = 0i64;
            let mut tick = 0i64;
            while lateness.len() < ticks {
                let pending = sys::ulTaskGenericNotifyTake(0, 1, u32::MAX) as i64;
                let now = sys::esp_timer_get_time();
                for _ in 0..pending {
                    if tick == 0 {
                        first = now;
                    }
                    let late = now - first - tick * PERIOD_MICROSECONDS as i64;
                    lateness.push(late.max(0) as u32);
                    tick += 1;
                }
            }
            match source {
                Source::EspTimer => {
                    sys::esp_timer_stop(esp_timer);
                    sys::esp_timer_delete(esp_timer);
                }
                Source::Gptimer => {
                    sys::gptimer_stop(gptimer);
                    sys::gptimer_disable(gptimer);
                    sys::gptimer_del_timer(gptimer);
                }
            }
        }
        lateness
    });
    ThreadSpawnConfiguration::default().set().ok();
    let mut lateness = paced.join().unwrap_or_default();
    if lateness.is_empty() {
        return cc_protocol::build("err", &["pace", "no timer"], &[]);
    }
    lateness.sort_unstable();
    let at = |per_mille: usize| lateness[(lateness.len() - 1) * per_mille / 1000].to_string();
    let missed = lateness
        .iter()
        .filter(|late| **late as u64 >= PERIOD_MICROSECONDS)
        .count()
        .to_string();
    let label = match source {
        Source::EspTimer => "esp_timer",
        Source::Gptimer => "gptimer",
    };
    cc_protocol::build(
        "ok",
        &["pace", label],
        &[
            ("ticks", &lateness.len().to_string()),
            ("p50_us", &at(500)),
            ("p99_us", &at(990)),
            ("p999_us", &at(999)),
            ("max_us", &at(1000)),
            ("missed", &missed),
        ],
    )
}

/// Write the flash `log` partition flat out for `seconds` on a background thread (core 0, priority 5): erase a
/// 64 KB block, program it in 4 KB pages, next block, from 16 MB up (4-byte addressing). Returns at once; the
/// thread logs the blocks written when it ends. The telemetry ring's worst case, to run beside `pace` and `video`.
pub fn flash_write(seconds: u64) -> String {
    // SAFETY: a lookup by label; the descriptor lives for the program.
    let partition = unsafe {
        sys::esp_partition_find_first(
            sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
            sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
            c"log".as_ptr(),
        )
    };
    if partition.is_null() {
        return cc_protocol::build("err", &["flashwrite", "no log partition"], &[]);
    }
    let partition = partition as usize;
    ThreadSpawnConfiguration {
        pin_to_core: Some(Core::Core0),
        priority: 5,
        ..Default::default()
    }
    .set()
    .ok();
    let spawned = std::thread::Builder::new().stack_size(8192).spawn(move || {
        let partition = partition as *const sys::esp_partition_t;
        let page = vec![0xa5u8; 4096];
        // SAFETY: the descriptor lives for the program.
        let (base, size) = unsafe { ((*partition).address as usize, (*partition).size as usize) };
        let first = 0x100_0000usize.saturating_sub(base) / 65536; // from 16 MB absolute
        let last = size / 65536 - 1;
        let deadline = unsafe { sys::esp_timer_get_time() } + seconds as i64 * 1_000_000;
        let (mut block, mut written) = (first, 0u32);
        while unsafe { sys::esp_timer_get_time() } < deadline {
            // SAFETY: inside the log partition, with a live buffer.
            unsafe {
                if sys::esp_partition_erase_range(partition, block * 65536, 65536) != sys::ESP_OK {
                    break;
                }
                for index in 0..16 {
                    sys::esp_partition_write_raw(partition, block * 65536 + index * 4096, page.as_ptr().cast(), 4096);
                }
            }
            written += 1;
            block = if block >= last { first } else { block + 1 };
        }
        log::info!(
            "flashwrite :: {written} blocks of 64 KB in {seconds} s ({} KB/s)",
            written as u64 * 64 / seconds
        );
    });
    ThreadSpawnConfiguration::default().set().ok();
    match spawned {
        Ok(_) => cc_protocol::build("ok", &["flashwrite", "started"], &[("seconds", &seconds.to_string())]),
        Err(error) => cc_protocol::build("err", &["flashwrite", &error.to_string()], &[]),
    }
}
