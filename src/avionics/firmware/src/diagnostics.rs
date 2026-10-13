//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Operator diagnostics, registered as GCL commands (crate::gcl) at boot:
//! - `load seconds=5` -- per-core CPU load and the busiest tasks over a window;
//! - `pace seconds=10 source=gptimer` -- a 1 kHz loop on core 1 (priority 20, the flight thread's place) woken by a
//!   gptimer interrupt allocated on core 1 (the flight design) or by an esp_timer (its task lives on core 0), and how
//!   late each tick wakes;
//! - `heap [stacks|tasks]` -- where the RAM is: internal, DMA-capable, RTC, SPM and PSRAM heaps; each task's stack
//!   (where it lives, how much of it was never touched); and (in an investigation build) each task's heap share.
//!
//! Run them while the camera records to price video against the flight loop. `load` and `pace` are SETTING-only:
//! `pace` runs a priority-20 loop on the flight core, and neither belongs anywhere near a flight.
//!
//! (The `flashwrite` load generator is gone: it wrote raw patterns into the `log` partition, which is the live
//! telemetry ring now. src/bin/flash.rs keeps that measurement.)

use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;
use firmware::gcl::{Command, Gate, Kind, Parameter, Reach};
use firmware::time::uptime_us;

/// One task's run time at a moment.
struct Sample {
    name: String,
    core: i32,
    runtime: u64,
}

/// The scheduler's state of every task, and the total run time (µs).
fn task_states() -> (Vec<sys::TaskStatus_t>, u64) {
    // SAFETY: sizes the array from the scheduler's own count (+ slack for tasks created meanwhile).
    let capacity = unsafe { sys::uxTaskGetNumberOfTasks() } as usize + 8;
    let mut status: Vec<sys::TaskStatus_t> = Vec::with_capacity(capacity);
    let mut total = 0;
    // SAFETY: the array has `capacity` slots; the call fills at most that many and returns the count.
    let count = unsafe { sys::uxTaskGetSystemState(status.as_mut_ptr(), capacity as u32, &mut total) } as usize;
    // SAFETY: `count` entries were written.
    unsafe { status.set_len(count) };
    (status, total)
}

/// A task's name.
fn task_name(task: &sys::TaskStatus_t) -> String {
    // SAFETY: the scheduler's task name is a NUL-terminated string.
    unsafe { std::ffi::CStr::from_ptr(task.pcTaskName) }
        .to_string_lossy()
        .into_owned()
}

/// Every task's accumulated run time (µs) and the total.
fn snapshot() -> (Vec<Sample>, u64) {
    let (status, total) = task_states();
    let samples = status
        .iter()
        .map(|task| Sample {
            name: task_name(task),
            core: task.xCoreID,
            runtime: task.ulRunTimeCounter,
        })
        .collect();
    (samples, total)
}

/// One heap class's figures for `heap`: free bytes, the largest free block, and the lowest free since boot.
fn heap_class(caps: u32) -> [String; 3] {
    let mut info = sys::multi_heap_info_t::default();
    // SAFETY: fills a plain struct with the heap allocator's own counters.
    unsafe { sys::heap_caps_get_info(&mut info, caps) };
    [
        info.total_free_bytes.to_string(),
        info.largest_free_block.to_string(),
        info.minimum_free_bytes.to_string(),
    ]
}

/// `heap`: where the RAM is, as one CC line. Internal RAM is the scarce kind on the P4: the camera's encoder holds one
/// ~138 KB block of it, esp_hosted copies every Wi-Fi frame into internal DMA memory and asserts when it gets none,
/// and the SD driver bounces every transfer whose buffer is not DMA-ready through internal DMA memory. So: free bytes,
/// the largest free block (what one allocation can get) and the lowest free since boot, for internal RAM, its
/// DMA-capable part, and PSRAM.
///
/// The RTC (LP SRAM) and SPM heaps are internal but not DMA-capable, and IDF hands them to plain requests last, so
/// they show what a cold, non-DMA object could still use.
pub fn heap() -> String {
    let internal = heap_class(sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT);
    let dma = heap_class(sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_DMA);
    let rtc = heap_class(sys::MALLOC_CAP_RTCRAM);
    let spm = heap_class(sys::MALLOC_CAP_SPM);
    let psram = heap_class(sys::MALLOC_CAP_SPIRAM);
    // SAFETY: a plain read of a counter hosted_no_reboot keeps.
    let hosted_internal = unsafe { sys::hosted_internal_fallbacks() }.to_string();
    cc_protocol::build(
        "ok",
        &["heap"],
        &[
            ("internal_free", &internal[0]),
            ("internal_largest", &internal[1]),
            ("internal_min", &internal[2]),
            ("dma_free", &dma[0]),
            ("dma_largest", &dma[1]),
            ("dma_min", &dma[2]),
            ("rtc_free", &rtc[0]),
            ("rtc_largest", &rtc[1]),
            ("spm_free", &spm[0]),
            ("spm_largest", &spm[1]),
            ("psram_free", &psram[0]),
            ("psram_largest", &psram[1]),
            ("psram_min", &psram[2]),
            // esp_hosted buffers that found PSRAM full and took internal DMA memory (components/hosted_no_reboot)
            ("hosted_internal", &hosted_internal),
        ],
    )
}

/// Where a task's stack lives, by its lowest address: `i` the internal L2 memory (DMA-capable), `p` PSRAM, `r` the RTC
/// (LP) SRAM, `s` the SPM, `?` anywhere else.
fn stack_region(base: usize) -> char {
    let within = |low: u32, high: u32| (low as usize..high as usize).contains(&base);
    if within(sys::SOC_DRAM_LOW, sys::SOC_DRAM_HIGH) {
        'i'
    } else if within(sys::SOC_EXTRAM_LOW, sys::SOC_EXTRAM_HIGH) {
        'p'
    } else if within(sys::SOC_RTC_DRAM_LOW, sys::SOC_RTC_DRAM_HIGH) {
        'r'
    } else if within(sys::SOC_SPM_LOW, sys::SOC_SPM_HIGH) {
        's'
    } else {
        '?'
    }
}

/// `heap stacks`: each task's stack as `<task>=<never-used bytes>/<region>:<core>` (region as [`stack_region`]; core
/// `any` when unpinned), internal stacks first, then by name: the protocol's plain characters, so a value reads as is.
/// The never-used figure is FreeRTOS's high-water mark: the least free the stack has had since the task started, in
/// bytes.
pub fn heap_stacks() -> String {
    let (status, _) = task_states();
    let mut rows: Vec<(char, String, u32, String)> = status
        .iter()
        .map(|task| {
            let core = match task.xCoreID {
                core @ (0 | 1) => core.to_string(),
                _ => "any".to_string(), // tskNO_AFFINITY
            };
            (
                stack_region(task.pxStackBase as usize),
                task_name(task).replace(' ', "_"),
                task.usStackHighWaterMark,
                core,
            )
        })
        .collect();
    rows.sort_by(|left, right| (left.0 != 'i', &left.1).cmp(&(right.0 != 'i', &right.1)));
    let values: Vec<(String, String)> = rows
        .iter()
        .map(|(region, name, free, core)| (name.clone(), format!("{free}/{region}:{core}")))
        .collect();
    let named: Vec<(&str, &str)> = values
        .iter()
        .map(|(key, value)| (key.as_str(), value.as_str()))
        .collect();
    cc_protocol::build("ok", &["heap", "stacks"], &named)
}

/// `heap tasks`: each task's current heap use, internal and PSRAM, as `<task>=<internal>/<psram>` bytes, the largest
/// internal users first. Only in an investigation build with CONFIG_HEAP_TASK_TRACKING: that option crashes on any
/// allocation made while the scheduler is suspended, so it never ships. Task handles are matched against the live
/// task list (a deleted task's handle is never dereferenced); allocations of tasks that are gone show as `gone`.
#[cfg(esp_idf_heap_task_tracking)]
pub fn heap_tasks() -> String {
    const MAXIMUM_TASKS: usize = 64;
    // SAFETY: plain C structs, valid all-zero.
    let mut totals: Vec<sys::heap_task_totals_t> = vec![unsafe { std::mem::zeroed() }; MAXIMUM_TASKS];
    let mut count: usize = 0;
    let mut parameters = sys::heap_task_info_params_t {
        caps: [sys::MALLOC_CAP_INTERNAL as i32, sys::MALLOC_CAP_SPIRAM as i32, 0, 0],
        mask: [sys::MALLOC_CAP_INTERNAL as i32, sys::MALLOC_CAP_SPIRAM as i32, 0, 0],
        tasks: std::ptr::null_mut(),
        num_tasks: 0,
        totals: totals.as_mut_ptr(),
        num_totals: &mut count,
        max_totals: MAXIMUM_TASKS,
        blocks: std::ptr::null_mut(),
        max_blocks: 0,
    };
    // SAFETY: the totals array has MAXIMUM_TASKS slots and `count` says how many the call filled.
    unsafe { sys::heap_caps_get_per_task_info(&mut parameters) };
    // the live tasks' handles and names, to name the totals without touching a deleted task's handle
    let (status, _) = task_states();
    let name_of = |handle: sys::TaskHandle_t| -> String {
        if handle.is_null() {
            return "boot".to_string();
        }
        status
            .iter()
            .find(|task| task.xHandle == handle)
            .map(task_name)
            .unwrap_or_else(|| "gone".to_string())
    };
    let mut rows: Vec<(String, usize, usize)> = totals[..count.min(MAXIMUM_TASKS)]
        .iter()
        .map(|entry| (name_of(entry.task), entry.size[0], entry.size[1]))
        .collect();
    rows.sort_by(|a, b| b.1.cmp(&a.1));
    let values: Vec<(String, String)> = rows
        .iter()
        .map(|(name, internal, psram)| (name.replace(' ', "_"), format!("{internal}/{psram}")))
        .collect();
    let named: Vec<(&str, &str)> = values
        .iter()
        .map(|(key, value)| (key.as_str(), value.as_str()))
        .collect();
    cc_protocol::build("ok", &["heap", "tasks"], &named)
}

/// `heap tasks` in a build without CONFIG_HEAP_TASK_TRACKING: why there is no per-task figure.
#[cfg(not(esp_idf_heap_task_tracking))]
pub fn heap_tasks() -> String {
    cc_protocol::build(
        "err",
        &[
            "heap",
            "per-task figures need an investigation build with CONFIG_HEAP_TASK_TRACKING",
        ],
        &[],
    )
}

/// Per-core load over `seconds` and the busiest tasks, as one CC line.
pub fn load(seconds: u64) -> String {
    let (before, _) = snapshot();
    let start = uptime_us();
    std::thread::sleep(Duration::from_secs(seconds));
    let (after, _) = snapshot();
    let elapsed = uptime_us() - start;
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
    deltas.sort_by_key(|(_, _, delta)| std::cmp::Reverse(*delta));
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

/// The paced loop's period, us: 1 kHz.
const PERIOD_MICROSECONDS: u64 = 1000;

/// A started timer pacing the loop: it notifies the paced thread every [`PERIOD_MICROSECONDS`].
enum Pacer {
    /// An esp_timer, periodic.
    EspTimer(sys::esp_timer_handle_t),
    /// A gptimer, its alarm auto-reloading.
    Gptimer(sys::gptimer_handle_t),
}

impl Pacer {
    /// Starts `source` notifying `task`; None when the driver refuses it. Called on core 1, so a gptimer's interrupt
    /// is allocated there.
    ///
    /// SAFETY: `task` is the calling thread's handle, and the thread stops the timer ([`Pacer::stop`]) before it ends.
    unsafe fn start(source: Source, task: *mut core::ffi::c_void) -> Option<Self> {
        // SAFETY: the caller's (see above); the handles are the drivers' own until stop.
        unsafe {
            match source {
                Source::EspTimer => {
                    let mut esp_timer: sys::esp_timer_handle_t = core::ptr::null_mut();
                    let arguments = sys::esp_timer_create_args_t {
                        callback: Some(tick),
                        arg: task,
                        dispatch_method: sys::esp_timer_dispatch_t_ESP_TIMER_TASK,
                        name: c"pace".as_ptr(),
                        skip_unhandled_events: false,
                    };
                    if sys::esp_timer_create(&arguments, &mut esp_timer) != sys::ESP_OK {
                        return None;
                    }
                    sys::esp_timer_start_periodic(esp_timer, PERIOD_MICROSECONDS);
                    Some(Self::EspTimer(esp_timer))
                }
                Source::Gptimer => {
                    let mut gptimer: sys::gptimer_handle_t = core::ptr::null_mut();
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
                        return None;
                    }
                    Some(Self::Gptimer(gptimer))
                }
            }
        }
    }

    /// Stops the timer and hands it back to its driver.
    fn stop(self) {
        // SAFETY: a timer [`Pacer::start`] started, stopped once.
        unsafe {
            match self {
                Self::EspTimer(esp_timer) => {
                    sys::esp_timer_stop(esp_timer);
                    sys::esp_timer_delete(esp_timer);
                }
                Self::Gptimer(gptimer) => {
                    sys::gptimer_stop(gptimer);
                    sys::gptimer_disable(gptimer);
                    sys::gptimer_del_timer(gptimer);
                }
            }
        }
    }
}

/// The paced thread: `ticks` ticks woken by `source`, each one's lateness (us) against an ideal grid anchored at the
/// first tick (both sources share the crystal); empty when the timer would not start.
fn paced(ticks: usize, source: Source) -> Vec<u32> {
    let task = unsafe { sys::xTaskGetCurrentTaskHandle() } as *mut core::ffi::c_void;
    let mut lateness = Vec::with_capacity(ticks);
    // SAFETY: this thread's own handle; the timer is stopped below, before the thread returns.
    let Some(pacer) = (unsafe { Pacer::start(source, task) }) else {
        return lateness;
    };
    let mut first = 0i64;
    let mut tick = 0i64;
    while lateness.len() < ticks {
        // SAFETY: takes this thread's own notifications.
        let pending = unsafe { sys::ulTaskGenericNotifyTake(0, 1, u32::MAX) } as i64;
        let now = uptime_us() as i64;
        for _ in 0..pending {
            if tick == 0 {
                first = now;
            }
            let late = now - first - tick * PERIOD_MICROSECONDS as i64;
            lateness.push(late.max(0) as u32);
            tick += 1;
        }
    }
    pacer.stop();
    lateness
}

/// A 1 kHz loop on core 1 for `seconds`, woken by `source`: how late each tick woke, as one CC line.
pub fn pace(seconds: u64, source: Source) -> String {
    let ticks = (seconds * 1_000_000 / PERIOD_MICROSECONDS) as usize;
    ThreadSpawnConfiguration {
        pin_to_core: Some(Core::Core1),
        priority: 20,
        ..Default::default()
    }
    .set()
    .ok();
    // a stack the heap cannot give is an answer, not a panic (std::thread::spawn's): under a heavy load internal RAM
    // runs short, and a bench diagnostic must not reboot the board
    let paced = std::thread::Builder::new().spawn(move || paced(ticks, source));
    ThreadSpawnConfiguration::default().set().ok();
    let paced = match paced {
        Ok(paced) => paced,
        Err(error) => return cc_protocol::build("err", &["pace", "no thread", &error.to_string()], &[]),
    };
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

/// The diagnostics' console commands (the Glider Command Language), registered at boot.
pub fn register_commands() {
    firmware::gcl::register(vec![
        interrupts_command(),
        heap_command(),
        load_command(),
        pace_command(),
    ]);
}

/// `interrupts` (the Glider Command Language).
fn interrupts_command() -> Command {
    Command {
        owner: "diagnostics",
        words: "interrupts",
        about: "IDF's interrupt table, printed on the USB console: each allocated interrupt's core, level, source \
            and whether it is IRAM-safe (esp_intr_dump): where the tick's, the GPIO's and the buses' ISRs run",
        gate: Gate::Open,
        reach: Reach::Console,
        parameters: &[],
        bare_words: &[],
        run: Box::new(|_| {
            // SAFETY: a null stream is stdout (intr_alloc.c:1017-1019), the USB console's UART.
            match unsafe { sys::esp_intr_dump(core::ptr::null_mut()) } {
                sys::ESP_OK => cc_protocol::build("ok", &["interrupts", "printed on the console"], &[]),
                code => cc_protocol::build("err", &["interrupts", &format!("esp_intr_dump {code:#x}")], &[]),
            }
        }),
    }
}

/// `heap` (the Glider Command Language).
fn heap_command() -> Command {
    Command {
        owner: "diagnostics",
        words: "heap",
        about: "where the RAM is: free, largest block and lowest-ever free of internal, DMA-capable, RTC, SPM and \
            PSRAM heaps",
        gate: Gate::Open,
        reach: Reach::Everywhere,
        parameters: &[],
        bare_words: &[
            firmware::gcl::Word {
                name: "stacks",
                about: "each task's stack: never-used bytes / where it lives (i internal, p PSRAM, r RTC, s SPM) : \
                    core",
            },
            firmware::gcl::Word {
                name: "tasks",
                about: "each task's internal/PSRAM heap use (an investigation build with CONFIG_HEAP_TASK_TRACKING \
                    only)",
            },
        ],
        run: Box::new(|arguments| {
            if arguments.word("stacks") {
                heap_stacks()
            } else if arguments.word("tasks") {
                heap_tasks()
            } else {
                heap()
            }
        }),
    }
}

/// `load` (the Glider Command Language).
fn load_command() -> Command {
    Command {
        owner: "diagnostics",
        words: "load",
        about: "each core's CPU load and the busiest tasks over a window",
        gate: Gate::Setting,
        reach: Reach::Everywhere,
        parameters: &[Parameter {
            name: "seconds",
            kind: Kind::Integer { low: 1, high: 600 },
            default: Some("5"),
            about: "the window (through the hub keep it under ~25 s: its exchange times out at 30)",
        }],
        bare_words: &[],
        run: Box::new(|arguments| load(arguments.integer("seconds") as u64)),
    }
}

/// `pace` (the Glider Command Language).
fn pace_command() -> Command {
    Command {
        owner: "diagnostics",
        words: "pace",
        about: "a 1 kHz loop on core 1 at the flight thread's priority, and how late each tick wakes",
        gate: Gate::Setting,
        reach: Reach::Everywhere,
        parameters: &[
            Parameter {
                name: "seconds",
                kind: Kind::Integer { low: 1, high: 600 },
                default: Some("10"),
                about: "how long (through the hub keep it under ~25 s: its exchange times out at 30)",
            },
            Parameter {
                name: "source",
                kind: Kind::Choice(&["gptimer", "esptimer"]),
                default: Some("gptimer"),
                about: "what wakes the loop: a core-1 gptimer interrupt (the flight design) or an esp_timer",
            },
        ],
        bare_words: &[],
        run: Box::new(|arguments| {
            let source = if arguments.text("source") == "esptimer" {
                Source::EspTimer
            } else {
                Source::Gptimer
            };
            pace(arguments.integer("seconds") as u64, source)
        }),
    }
}
