//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The control thread (doc/specs/avionics-port.md §4.2, §4.3; M6): the flight tick on core 1 at priority 20, paced by
//! a gptimer alarm at the config's `schedule_hz` (board_config::schedule_hz). The timer is created on this thread, so
//! its interrupt is allocated on core 1, and its ISR only notifies: this thread first, then each I2C owner
//! (crate::owner), which run their own tables on the same tick. No float, no lock and no allocation in the ISR.
//!
//! Each tick takes the notifications that piled up: one is the tick, any more are ticks this thread missed (an
//! overrun). Its lateness is measured against an ideal grid anchored at the first tick (the gptimer and esp_timer share
//! the crystal). The figures are atomics the console reads (`control`), never a lock on this thread's path.
//!
//! The tick's work (§4.3), as the milestones bring it: the primary IMU, the LSM6DSO32 on SPI2 (M6) -- its INT1 edges
//! counted by a core-1 GPIO interrupt (`irq_runs`, plan B's evidence of a live line), one 12-byte burst read, `accel`
//! and `rate` pushed, its row written on this thread's own producer ring.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use databoard::Writer;
use drivers::lsm6dso32::{Lsm6dso32, OverSpi};
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;
use firmware::buses::spi::SpiHandle;
use firmware::gcl::{self, Command, Gate, Reach};
use recorder::{Cell, Telemetry};

use crate::devices::{SpiMember, Topology};
use crate::recorder::Producer;

/// The control thread's priority: above the bus owners (18, 17), below esp_hosted's (§4.2).
const PRIORITY: u8 = 20;

/// The control thread's stack (internal: a flight thread); grown as the tick takes on the IMU, the attitude filter
/// and the flight law, each step measured with `heap stacks`.
const STACK_SIZE: usize = 6 << 10;

/// The lateness budget a tick must keep, microseconds (the M6 proof: max <= 200 us under FHD@25 and bus load).
pub const LATENESS_BUDGET_US: u32 = 200;

/// The gptimer's counting rate: one count a microsecond.
const TIMER_RESOLUTION_HZ: u32 = 1_000_000;

/// The tick rate the thread runs at, Hz (0 until it runs).
static SCHEDULE_HZ: AtomicU32 = AtomicU32::new(0);
/// Ticks since the first, missed ones included.
static TICKS: AtomicU32 = AtomicU32::new(0);
/// Ticks the thread woke too late to take on their own: notifications that piled up beyond one.
static OVERRUNS: AtomicU32 = AtomicU32::new(0);
/// The latest tick's lateness, microseconds.
static LATE_LAST_US: AtomicU32 = AtomicU32::new(0);
/// The largest lateness so far, microseconds.
static LATE_MAX_US: AtomicU32 = AtomicU32::new(0);
/// Ticks later than [`LATENESS_BUDGET_US`].
static LATE_OVER_BUDGET: AtomicU32 = AtomicU32::new(0);
/// INT1 edges since the tick last took them: the GPIO interrupt counts, the tick swaps the count out.
static INTERRUPT_EDGES: AtomicU32 = AtomicU32::new(0);
/// IMU samples read.
static IMU_READS: AtomicU32 = AtomicU32::new(0);
/// IMU reads that failed.
static IMU_FAILURES: AtomicU32 = AtomicU32::new(0);
/// The latest tick's `irq_runs`.
static IRQ_RUNS_LAST: AtomicU32 = AtomicU32::new(0);
/// Whether INT1 is silent (the driver's verdict: three wakes in a row with no edge).
static INTERRUPT_SILENT: AtomicBool = AtomicBool::new(false);

/// The primary IMU as the control thread drives it: the part, its bus handle, its databoard writers and its stream.
struct Imu {
    /// The entry's name: the databoard source and the stream's.
    name: &'static str,
    /// Its chip select on SPI2.
    handle: SpiHandle,
    /// The driver.
    part: Lsm6dso32<OverSpi>,
    /// The `accel` channel's writer, when the quantity is on the databoard.
    accel: Option<Writer<'static, [f32; 3]>>,
    /// The `rate` channel's writer.
    rate: Option<Writer<'static, [f32; 3]>>,
    /// Its telemetry stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// A read failed and has not succeeded since: one log line per run of failures (plan B's `note`).
    failing: bool,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Imu {
    /// The IMU of `member`, its writers claimed from `topology` and its stream opened.
    fn new(member: SpiMember, topology: &'static Topology) -> Imu {
        let writer = |quantity: &str| {
            topology
                .vector(quantity)
                .and_then(|parameter| parameter.writer(member.name))
        };
        let telemetry = crate::recorder::get()
            .zip(crate::recorder::device_stream(member.name))
            .and_then(|(recorder, stream)| Telemetry::new(recorder, stream));
        Imu {
            name: member.name,
            accel: writer("accel"),
            rate: writer("rate"),
            telemetry,
            handle: member.handle,
            part: member.part,
            failing: false,
            overflowing: false,
        }
    }

    /// One tick: take the INT1 edges, read a sample, push it and write its row.
    fn tick(&mut self, now_us: u64, period_ms: u32) {
        let runs = INTERRUPT_EDGES.swap(0, Ordering::Relaxed);
        IRQ_RUNS_LAST.store(runs, Ordering::Relaxed);
        if self.part.wake(runs) {
            log::warn!("lsm6dso32 :: INT1 silent -- sampling on the {period_ms} ms fallback");
        }
        INTERRUPT_SILENT.store(self.part.interrupt_silent(), Ordering::Relaxed);
        let sample = match self.part.read(&mut self.handle) {
            Ok(sample) => sample,
            Err(fault) => {
                IMU_FAILURES.fetch_add(1, Ordering::Relaxed);
                if !self.failing {
                    log::warn!("lsm6dso32 :: read {fault}");
                    self.failing = true;
                }
                return;
            }
        };
        self.failing = false;
        IMU_READS.fetch_add(1, Ordering::Relaxed);
        let accel = sample.accel_g();
        if let Some(writer) = &mut self.accel {
            writer.push(accel, now_us);
        }
        if let Some(writer) = &mut self.rate {
            writer.push(sample.rate_degrees_per_second(), now_us);
        }
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        if !telemetry.due(now_us) {
            return;
        }
        let [gx, gy, gz] = sample.rate_centidegrees_per_second();
        let row = [
            Cell::F32(accel[0]),
            Cell::F32(accel[1]),
            Cell::F32(accel[2]),
            Cell::I32(gx),
            Cell::I32(gy),
            Cell::I32(gz),
            Cell::I32(runs as i32),
        ];
        match telemetry.push(now_us, &row) {
            Ok(()) => self.overflowing = false,
            Err(overflow) => {
                if !self.overflowing {
                    log::warn!("{} :: telemetry {overflow}", self.name);
                    self.overflowing = true;
                }
            }
        }
    }
}

/// The INT1 data-ready interrupt: count the edge. Integer only.
unsafe extern "C" fn interrupt_edge(_: *mut core::ffi::c_void) {
    INTERRUPT_EDGES.fetch_add(1, Ordering::Relaxed);
}

/// Arms a rising-edge interrupt on GPIO `gpio` that counts into [`INTERRUPT_EDGES`]; called on the control thread, so
/// the GPIO interrupt service, installed here first, runs on core 1. No pull: plan B's `Pin(gpio, Pin.IN)`.
fn arm_interrupt(gpio: u8) -> Result<(), sys::EspError> {
    let config = sys::gpio_config_t {
        pin_bit_mask: 1u64 << gpio,
        mode: sys::gpio_mode_t_GPIO_MODE_INPUT,
        pull_up_en: sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
        pull_down_en: sys::gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
        intr_type: sys::gpio_int_type_t_GPIO_INTR_POSEDGE,
        ..Default::default()
    };
    // SAFETY: plain GPIO configuration of the pin the board config gives INT1; the handler is a static function.
    unsafe {
        sys::esp!(sys::gpio_config(&config))?;
        match sys::gpio_install_isr_service(0) {
            sys::ESP_OK | sys::ESP_ERR_INVALID_STATE => {}
            code => sys::esp!(code)?,
        }
        sys::esp!(sys::gpio_isr_handler_add(
            i32::from(gpio),
            Some(interrupt_edge),
            core::ptr::null_mut()
        ))?;
    }
    Ok(())
}

/// The tasks the alarm notifies: the control thread, then each running I2C owner. Leaked: the timer runs for the
/// program.
struct Wakers {
    /// The task handles, `count` of them used.
    tasks: [sys::TaskHandle_t; 3],
    /// How many are used.
    count: usize,
}

/// The gptimer alarm ISR: notify every waker (eIncrement, so a pile-up counts the missed ticks). Integer only.
unsafe extern "C" fn alarm(
    _timer: sys::gptimer_handle_t,
    _event: *const sys::gptimer_alarm_event_data_t,
    context: *mut core::ffi::c_void,
) -> bool {
    // SAFETY: `context` is the leaked Wakers, alive for the program.
    let wakers = unsafe { &*(context as *const Wakers) };
    let mut woken: sys::BaseType_t = 0;
    for &task in &wakers.tasks[..wakers.count] {
        // SAFETY: each handle is a task that lives for the program (the control thread and the owners never end).
        unsafe {
            sys::xTaskGenericNotifyFromISR(
                task,
                0,
                0,
                sys::eNotifyAction_eIncrement,
                core::ptr::null_mut(),
                &mut woken,
            )
        };
    }
    woken != 0
}

/// Starts the control thread (core 1, priority 20) ticking at `schedule_hz`, waking the I2C owners that run on each
/// tick too, with the SPI parts that came up (`spi`: the primary IMU) and the databoard's `topology`. Call it after
/// the owners started (their task handles are taken here).
///
/// Errors: the thread would not spawn.
pub fn start(schedule_hz: u32, spi: Vec<SpiMember>, topology: &'static Topology) -> anyhow::Result<()> {
    ThreadSpawnConfiguration {
        name: Some(c"control"),
        stack_size: STACK_SIZE,
        priority: PRIORITY,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new()
        .stack_size(STACK_SIZE)
        .spawn(move || run(schedule_hz, spi, topology));
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}

/// The control thread: the IMU, the timer, then the tick forever.
fn run(schedule_hz: u32, spi: Vec<SpiMember>, topology: &'static Topology) {
    let period_us = (TIMER_RESOLUTION_HZ / schedule_hz.max(1)) as i64;
    // this thread's own producer ring: its rows and log lines never contend with another thread's
    let _attachment = crate::recorder::get().and_then(|recorder| recorder.attach(Producer::Control as usize).ok());
    let mut spi = spi.into_iter();
    let mut imu = spi.next().map(|member| {
        if let Some(gpio) = member.interrupt
            && let Err(error) = arm_interrupt(gpio)
        {
            log::warn!(
                "{} :: INT1 GPIO{gpio} not armed ({error}): sampling on the tick alone",
                member.name
            );
        }
        Imu::new(member, topology)
    });
    for extra in spi {
        log::warn!("control :: {} not driven: one SPI IMU only", extra.name);
    }
    let mut tasks = [core::ptr::null_mut(); 3];
    // SAFETY: a plain read of this task's own handle.
    tasks[0] = unsafe { sys::xTaskGetCurrentTaskHandle() };
    let mut count = 1;
    for task in (0..2).filter_map(crate::owner::task) {
        tasks[count] = task;
        count += 1;
    }
    let wakers: &'static Wakers = Box::leak(Box::new(Wakers { tasks, count }));
    if let Err(error) = start_timer(period_us as u64, wakers) {
        log::error!("control :: no tick ({error}): the control thread stops");
        return;
    }
    SCHEDULE_HZ.store(schedule_hz, Ordering::Relaxed);
    log::info!(
        "control :: ticking at {schedule_hz} Hz on core 1 (gptimer), waking {} bus owner(s)",
        count - 1
    );
    let mut first_us = None;
    let mut ticks: i64 = 0;
    loop {
        // SAFETY: this task's own notification value, index 0, cleared on take.
        let pending = unsafe { sys::ulTaskGenericNotifyTake(0, 1, u32::MAX) };
        // SAFETY: a plain read of the monotonic timer.
        let now_us = unsafe { sys::esp_timer_get_time() };
        if pending == 0 {
            continue;
        }
        if pending > 1 {
            OVERRUNS.fetch_add(pending - 1, Ordering::Relaxed);
        }
        let first = *first_us.get_or_insert(now_us);
        ticks += i64::from(pending);
        TICKS.store(ticks as u32, Ordering::Relaxed);
        let late = (now_us - first - (ticks - 1) * period_us).clamp(0, i64::from(u32::MAX)) as u32;
        LATE_LAST_US.store(late, Ordering::Relaxed);
        LATE_MAX_US.fetch_max(late, Ordering::Relaxed);
        if late > LATENESS_BUDGET_US {
            LATE_OVER_BUDGET.fetch_add(1, Ordering::Relaxed);
        }
        if let Some(imu) = &mut imu {
            imu.tick(now_us as u64, (period_us / 1000) as u32);
        }
    }
}

/// The tick's gptimer: an auto-reloading alarm every `period_us`, its ISR notifying `wakers`; created on this thread
/// (core 1), so its interrupt is allocated on core 1.
fn start_timer(period_us: u64, wakers: &'static Wakers) -> Result<(), sys::EspError> {
    let config = sys::gptimer_config_t {
        clk_src: sys::soc_periph_gptimer_clk_src_t_GPTIMER_CLK_SRC_DEFAULT,
        direction: sys::gptimer_count_direction_t_GPTIMER_COUNT_UP,
        resolution_hz: TIMER_RESOLUTION_HZ,
        ..Default::default()
    };
    let callbacks = sys::gptimer_event_callbacks_t { on_alarm: Some(alarm) };
    let mut alarm_config = sys::gptimer_alarm_config_t {
        alarm_count: period_us,
        ..Default::default()
    };
    alarm_config.flags.set_auto_reload_on_alarm(1);
    let mut timer: sys::gptimer_handle_t = core::ptr::null_mut();
    let context = wakers as *const Wakers as *mut core::ffi::c_void;
    // SAFETY: a filled config and a live output; the timer runs for the program, with a context that does too.
    unsafe {
        sys::esp!(sys::gptimer_new_timer(&config, &mut timer))?;
        sys::esp!(sys::gptimer_register_event_callbacks(timer, &callbacks, context))?;
        sys::esp!(sys::gptimer_set_alarm_action(timer, &alarm_config))?;
        sys::esp!(sys::gptimer_enable(timer))?;
        sys::esp!(sys::gptimer_start(timer))?;
    }
    Ok(())
}

/// The control thread's console command (the Glider Command Language): `control`, the tick's figures.
pub fn register_commands() {
    gcl::register(vec![Command {
        owner: "control",
        words: "control",
        about: "the control tick: its rate, ticks, overruns (ticks the thread missed) and lateness (the latest, the \
                largest, and how many ticks passed the 200 us budget); the IMU's reads, failures, INT1 edges at the \
                latest tick and whether INT1 is silent",
        gate: Gate::Open,
        reach: Reach::Everywhere,
        parameters: &[],
        bare_words: &[],
        run: Box::new(|_| {
            cc_protocol::build(
                "ok",
                &["control"],
                &[
                    ("schedule_hz", &SCHEDULE_HZ.load(Ordering::Relaxed).to_string()),
                    ("ticks", &TICKS.load(Ordering::Relaxed).to_string()),
                    ("overruns", &OVERRUNS.load(Ordering::Relaxed).to_string()),
                    ("late_last_us", &LATE_LAST_US.load(Ordering::Relaxed).to_string()),
                    ("late_max_us", &LATE_MAX_US.load(Ordering::Relaxed).to_string()),
                    (
                        "late_over_budget",
                        &LATE_OVER_BUDGET.load(Ordering::Relaxed).to_string(),
                    ),
                    ("imu_reads", &IMU_READS.load(Ordering::Relaxed).to_string()),
                    ("imu_failures", &IMU_FAILURES.load(Ordering::Relaxed).to_string()),
                    ("irq_runs_last", &IRQ_RUNS_LAST.load(Ordering::Relaxed).to_string()),
                    ("int1_silent", &INTERRUPT_SILENT.load(Ordering::Relaxed).to_string()),
                ],
            )
        }),
    }]);
}
