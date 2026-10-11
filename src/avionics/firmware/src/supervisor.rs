//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The supervisor thread (doc/specs/avionics-port.md "Threads, cores, priorities": core 0, priority 9), on a 100 ms
//! base tick that never ends:
//!  * the task watchdog, when the board config enables a `watchdog` (plan B's tasks/watchdog.py): on its first loop
//!    the supervisor reconfigures IDF's task watchdog to panic after `wdt_timeout_ms` with no idle task watched (the
//!    built sdkconfig only warns, and plan B's `machine.WDT` reset), subscribes itself, and feeds it every base tick.
//!    A supervisor that stops looping, wedged or starved, panics the board, a reset the warm start recovers from;
//!  * the control thread's stall check (controller::Stall), every `period_ms` on the base tick's grid: while the board
//!    controls a flight (armed, launched, in an airborne stage), a control tick older than `stall_ms` (and two tick
//!    periods) restarts the board. The line `<watchdog> :: control loop stalled (stage=X) -> reset` is kept in this
//!    session's recorder.log first, and Wi-Fi's stop is taken off the restart's way: through the C6 it is a remote
//!    call that a parked link makes wait seconds, past the task watchdog. A boot that is a stall restart's own warm
//!    start never restarts for a stall again (crate::warm::stall_restart_suppressed), so a stall that comes back every
//!    boot leaves the board stalled rather than cycling it;
//!  * health.csv at 1 Hz, plan B's board_health columns unchanged (spec "Row, log and index text"), so
//!    recorder_flight, flight_report and flight_video read the Rust board's vitals as they read plan B's;
//!  * the checkpoint (crate::warm::Keeping) every `poll_ms`, when the board config enables one: the crumb a reset in
//!    flight warm-starts from, and checkpoint.csv;
//!  * the GNSS pad drift calibration (crate::gnss::Calibration) every `period_ms`, when the board config enables one.
//!
//! It runs with or without a recorder: without one, its rows go nowhere and the rest goes on; and once subscribed to
//! the task watchdog it never leaves its loop. The field logic joins it as it is ported (M11). The
//! NVS and flash locks its checkpoint takes are core 0's: the flight tick never waits on them. Its console lines go
//! through the logger's printer, as the flight core's do (crate::logging): a console reply holds the UART for
//! seconds, and a line printed in place would wait that out with the task watchdog unfed.
//!
//! The port, against plan B: the task watchdog watches the supervisor alone, and the control thread through the stall
//! check, where plan B's watched its whole event loop (a hung I2C bus included: here a bus transfer times out in 50
//! ms, so a hung bus shows as failing reads on a live owner). It is a task entry, so a timeout's backtrace names the
//! starved task. The stall check runs while the board controls a flight, a passive one included, where plan B's ran
//! only while its flight task controlled the fins. A refused key fails the entry at bring-up, where plan B's arm
//! refused the task its run loop crashed on.
//!
//! Its neighbours in the record need nothing from it: the ring_writer logs the `'session': '<s>'` status line every
//! `recorder.stats_ms` itself, and writes session.csv's `boot` and `anchor` rows; a clock set writes its own row
//! (commands/inspection.rs).

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::Duration;

use controller::{Stall, Table};
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::sys;
use recorder::{Cell, Telemetry};
use sequencer::{Flight, Stage};
use serde::Serialize;

use crate::board::{self, Board};
use crate::devices::WatchdogMember;
use crate::gnss::Calibration;
use crate::recorder::Producer;
use crate::status::FLIGHT;
use crate::warm::Keeping;

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

/// The base tick in ms: every duty runs on a whole number of them, its period rounded up.
const BASE_TICK_MS: u64 = 100;

/// The base tick in µs, the grid's step.
const BASE_TICK_US: u64 = BASE_TICK_MS * 1000;

/// health.csv every this many base ticks: 1 Hz, as plan B's board_health.
const HEALTH_TICKS: u64 = 10;

/// Its priority on core 0 (spec "Threads, cores, priorities").
const PRIORITY: u8 = 9;

/// Its stack (PSRAM): a health row, the checkpoint's crumb text and NVS writes, and the task watchdog's calls.
const STACK_SIZE: usize = 8 << 10;

/// How long a stall restart waits for the printer to put its line on the console.
const PRINTER_LIMIT: Duration = Duration::from_millis(100);

/// Why the supervisor's entries are crashed when its thread does not start (`verify`, `arm`).
const NOT_STARTED: &str = "the supervisor did not start";

/// The watchdog's settings, once the board runs one (`inspect`).
static SETTINGS: OnceLock<board_config::Watchdog> = OnceLock::new();

/// Whether the task watchdog panics the board and watches the supervisor: reconfigured and subscribed.
static TASK_WATCHDOG_ARMED: AtomicBool = AtomicBool::new(false);

/// Whether the stall check runs: the board runs a watchdog, and the supervisor is in its loop.
static STALL_CHECKING: AtomicBool = AtomicBool::new(false);

/// Set by `panic kind=twdt`, never cleared: the supervisor feeds the task watchdog no more, which then panics the
/// board.
static STARVING: AtomicBool = AtomicBool::new(false);

/// The longest gap between two feeds of the task watchdog, ms: its margin, against `wdt_timeout_ms`.
static FEED_MAX_MS: AtomicU32 = AtomicU32::new(0);

/// The longest gap between two stall checks, ms.
static CHECK_MAX_MS: AtomicU32 = AtomicU32::new(0);

/// Spawns the supervisor (core 0, priority 9).
///
/// Args:
/// * `board` - the vitals it samples (the die temperature).
/// * `table` - the device table: the watchdog's entry is crashed when the task watchdog will not arm, and every
///   supervised entry when the thread does not start.
/// * `keeping` - the checkpoint, when its entry came up and its NVS opened.
/// * `watchdog` - the watchdog, when its entry came up.
/// * `calibration` - the GNSS pad drift calibration, when its entry came up.
/// * `tick_ms` - the control tick's period, ms: the stall limit is at least two of them.
///
/// Errors: the thread would not spawn; the checkpoint's, the watchdog's and the calibration's entries are crashed
/// then.
pub fn start(
    board: Arc<Board>,
    table: &'static Table<'static>,
    keeping: Option<Keeping>,
    watchdog: Option<WatchdogMember>,
    calibration: Option<Calibration>,
    tick_ms: f32,
) -> anyhow::Result<()> {
    let supervised = [
        keeping.as_ref().map(Keeping::name),
        watchdog.as_ref().map(|member| member.name),
        calibration.as_ref().map(Calibration::name),
    ];
    if let Some(member) = &watchdog {
        let _ = SETTINGS.set(member.settings);
    }
    let spawned = ThreadSpawnConfiguration {
        name: Some(c"supervisor"),
        stack_size: STACK_SIZE,
        priority: PRIORITY,
        pin_to_core: Some(Core::Core0),
        // stack in PSRAM (avionics-port.md §4.7, internal RAM budget): a slow core-0 thread that never calls
        // esp_mmu_map/unmap, the one runtime path that turns the cache off (NVS writes do not, the flash running
        // with the cache on under XIP from PSRAM)
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()
    .map_err(anyhow::Error::from)
    .and_then(|()| {
        std::thread::Builder::new()
            .stack_size(STACK_SIZE)
            .spawn(move || run(board, table, keeping, watchdog, calibration, tick_ms))
            .map_err(anyhow::Error::from)
    });
    // the default back for the threads spawned after this one
    let reset = ThreadSpawnConfiguration::default().set();
    if spawned.is_err() {
        crate::devices::crash(table, supervised, NOT_STARTED);
    }
    spawned?;
    reset?;
    Ok(())
}

/// The supervisor: the task watchdog armed on its first loop, then every base tick the duties due on it, forever.
fn run(
    board: Arc<Board>,
    table: &'static Table<'static>,
    mut keeping: Option<Keeping>,
    watchdog: Option<WatchdogMember>,
    mut calibration: Option<Calibration>,
    tick_ms: f32,
) {
    let _attachment = crate::recorder::get().and_then(|recorder| {
        recorder
            .attach(Producer::Supervisor as usize)
            .inspect_err(|error| log::warn!("supervisor :: its rows and lines share the misc ring: {error:?}"))
            .ok()
    });
    let mut watch = watchdog.map(|member| Watch::new(&member, table, tick_ms));
    let mut health = Health::new(board);
    let keeping_ticks = keeping.as_ref().map_or(1, |keeping| ticks(keeping.poll_ms()));
    let calibration_ticks = calibration
        .as_ref()
        .map_or(1, |calibration| ticks(u64::from(calibration.period_ms())));
    // the grid, uptime µs: a base tick falls due one step after the last
    let mut due_us = uptime_us();
    // base ticks since the start: 100 ms each, so the u64 never wraps
    let mut base_tick: u64 = 0;
    loop {
        due_us += BASE_TICK_US;
        let now_us = uptime_us();
        // a tick that came late (a long NVS write, a busy core) starts the grid again from now
        if due_us < now_us {
            due_us = now_us;
        }
        std::thread::sleep(Duration::from_micros(due_us - now_us));
        // the feed and the stall check first: neither waits behind a health row or an NVS write
        if let Some(watch) = &mut watch {
            watch.tick(base_tick);
        }
        if base_tick.is_multiple_of(HEALTH_TICKS) {
            health.row(uptime_us());
        }
        if let Some(keeping) = &mut keeping
            && base_tick.is_multiple_of(keeping_ticks)
        {
            // the schedule runs on the grid's time, so two 500 ms polls are exactly a 1 s checkpoint apart, as plan
            // B's sleeps between polls are never less: measured at each wake, two polls come a few µs short of 1 s
            // about half the time, and those checkpoints slip to the third poll
            keeping.pass(due_us);
        }
        if let Some(calibration) = &mut calibration
            && base_tick.is_multiple_of(calibration_ticks)
        {
            calibration.pass(uptime_us());
        }
        base_tick += 1;
    }
}

/// A period in ms as a whole number of base ticks, rounded up, at least one.
fn ticks(period_ms: u64) -> u64 {
    period_ms.div_ceil(BASE_TICK_MS).max(1)
}

/// The supervisor's watchdog (plan B's watchdog task): the task watchdog it feeds, and the control thread's stall
/// check.
struct Watch {
    /// The entry's name: its lines'.
    name: &'static str,
    /// Whether the task watchdog took the supervisor: it is fed only then.
    armed: bool,
    /// The stall rule, its limit from `stall_ms` and the control tick.
    stall: Stall,
    /// A stall check every this many base ticks: `period_ms` rounded up to the base tick.
    check_ticks: u64,
    /// This boot is a stall restart's own warm start: a stall is logged, never restarted for.
    suppressed: bool,
    /// A suppressed stall was logged: once a boot.
    stall_logged: bool,
    /// When the task watchdog was last fed, uptime ms; None before the first feed.
    fed_ms: Option<u64>,
    /// When the stall was last checked, uptime ms; None before the first check.
    checked_ms: Option<u64>,
}

impl Watch {
    /// The watchdog of `member` on the calling thread, the supervisor: the task watchdog reconfigured to panic after
    /// `wdt_timeout_ms` and this thread subscribed, else the entry crashed in `table` with why (IDF's warn-only
    /// default stays); the stall rule on a control tick of `tick_ms`.
    fn new(member: &WatchdogMember, table: &Table, tick_ms: f32) -> Watch {
        let settings = member.settings;
        let armed = match arm_task_watchdog(settings.timeout_ms) {
            Ok(()) => {
                log::info!(
                    "{} :: task watchdog armed: a panic after {} ms without a feed; stall check every {} ms, limit \
                     {} ms",
                    member.name,
                    settings.timeout_ms,
                    settings.period_ms,
                    settings.stall_ms
                );
                true
            }
            Err(error) => {
                let reason = format!("the task watchdog did not arm: {error}");
                log::error!("{} :: {reason}", member.name);
                crate::devices::crash(table, [Some(member.name)], &reason);
                false
            }
        };
        TASK_WATCHDOG_ARMED.store(armed, Ordering::Relaxed);
        STALL_CHECKING.store(true, Ordering::Relaxed);
        let suppressed = crate::warm::stall_restart_suppressed();
        if suppressed {
            log::warn!(
                "{} :: this boot is a stall restart's warm start: a stall restarts it no more",
                member.name
            );
        }
        Watch {
            name: member.name,
            armed,
            stall: Stall::new(settings.stall_ms, tick_ms),
            check_ticks: ticks(u64::from(settings.period_ms)),
            suppressed,
            stall_logged: false,
            fed_ms: None,
            checked_ms: None,
        }
    }

    /// One base tick: the feed, unless `panic kind=twdt` starves it; the stall check when due.
    fn tick(&mut self, base_tick: u64) {
        if self.armed && !STARVING.load(Ordering::Relaxed) {
            // SAFETY: the calling task subscribed itself (arm_task_watchdog) and never unsubscribes.
            unsafe { sys::esp_task_wdt_reset() };
            record_gap(&mut self.fed_ms, &FEED_MAX_MS);
        }
        if base_tick.is_multiple_of(self.check_ticks) {
            self.check();
        }
    }

    /// One stall check (controller::Stall's order: the clock, then `ticked`, then the heartbeat). A stall restarts
    /// the board, unless this boot is a stall restart's warm start: then it is logged, once.
    fn check(&mut self) {
        record_gap(&mut self.checked_ms, &CHECK_MAX_MS);
        let now_ms = (uptime_us() / 1000) as u32;
        let flight = FLIGHT.load();
        let ticked = crate::control::ticked();
        let last_tick_ms = crate::control::last_tick_ms();
        if !self.stall.step(now_ms, last_tick_ms, ticked, controlling(flight)) {
            return;
        }
        if !self.suppressed {
            restart(self.name, flight.stage);
        }
        if !self.stall_logged {
            log::warn!(
                "{} :: control loop stalled (stage={}) -> no reset: this boot is a stall restart's warm start",
                self.name,
                flight.stage.name()
            );
            self.stall_logged = true;
        }
    }
}

/// Records the gap since `last_ms` (the uptime in ms) in `longest`, and now as the last.
fn record_gap(last_ms: &mut Option<u64>, longest: &AtomicU32) {
    let now_ms = uptime_us() / 1000;
    if let Some(last_ms) = *last_ms {
        longest.fetch_max(
            u32::try_from(now_ms.saturating_sub(last_ms)).unwrap_or(u32::MAX),
            Ordering::Relaxed,
        );
    }
    *last_ms = Some(now_ms);
}

/// Whether the board controls a flight, the stall check's condition: armed, launched (by its own stage machine or a
/// warm start: a bench hold is not) and in an airborne stage.
fn controlling(flight: Flight) -> bool {
    flight.armed && flight.launched && flight.stage.airborne()
}

/// Reconfigures IDF's task watchdog to panic after `timeout_ms`, watching no idle task, and subscribes the calling
/// task (the supervisor).
fn arm_task_watchdog(timeout_ms: u32) -> Result<(), sys::EspError> {
    let config = sys::esp_task_wdt_config_t {
        timeout_ms,
        idle_core_mask: 0,
        trigger_panic: true,
    };
    // SAFETY: IDF reads the config during the call; the supervisor is its only caller, once.
    sys::esp!(unsafe { sys::esp_task_wdt_reconfigure(&config) })?;
    // SAFETY: null subscribes the calling task, which lives for the program.
    sys::esp!(unsafe { sys::esp_task_wdt_add(core::ptr::null_mut()) })
}

/// Restarts the board for a stalled control thread (plan B's `machine.reset()`): the line first, into this session's
/// recorder.log (waiting at most 100 ms for the ring_writer) and to the console (at most 100 ms more for the printer,
/// which a console reply may hold up: the line then goes unprinted, never the restart late); Wi-Fi's stop taken off
/// the restart's shutdown handlers (esp_wifi registers it; through the C6 it is a remote call a parked link makes
/// wait seconds, past the task watchdog); then a software reset, which the warm start recovers from. Nothing on the
/// way waits unbounded, so the task watchdog, unfed meanwhile, never fires first and the reset stays a software one,
/// which the stall restart's guard knows.
fn restart(name: &str, stage: Stage) -> ! {
    log::warn!("{name} :: control loop stalled (stage={}) -> reset", stage.name());
    let stop: unsafe extern "C" fn() -> sys::esp_err_t = sys::esp_wifi_stop;
    // SAFETY: IDF registers esp_wifi_stop cast to a shutdown handler, `void (*)(void)` (esp_wifi's wifi_default.c);
    // the unregister only compares the address, it never calls through the cast.
    let handler =
        unsafe { core::mem::transmute::<unsafe extern "C" fn() -> sys::esp_err_t, unsafe extern "C" fn()>(stop) };
    // SAFETY: a plain removal from IDF's handler list; ESP_ERR_INVALID_STATE when Wi-Fi never registered it.
    unsafe { sys::esp_unregister_shutdown_handler(Some(handler)) };
    crate::recorder::catch_up();
    crate::logging::wait_for_printer(PRINTER_LIMIT);
    // SAFETY: the restart never returns.
    unsafe { sys::esp_restart() }
}

/// Stops feeding the task watchdog (`panic kind=twdt`), which panics the board `wdt_timeout_ms` later; the supervisor
/// runs on meanwhile.
///
/// Errors: the task watchdog is not armed, so nothing would reset the board.
pub fn starve() -> Result<(), &'static str> {
    if !TASK_WATCHDOG_ARMED.load(Ordering::Relaxed) {
        return Err("the task watchdog is not armed (no enabled watchdog entry, or it did not arm)");
    }
    STARVING.store(true, Ordering::Relaxed);
    Ok(())
}

/// Why `panic kind=stall` would not restart the board, None when it would: the stall check must run (an enabled
/// watchdog), its restart must not be suppressed, and the board must control a flight with its control thread
/// ticking. Without them the control thread would stop for good, and the board with it: `disarm` needs its tick.
pub fn stall_refused() -> Option<&'static str> {
    if !STALL_CHECKING.load(Ordering::Relaxed) {
        return Some("no stall check runs (no enabled watchdog entry)");
    }
    if crate::warm::stall_restart_suppressed() {
        return Some("this boot is a stall restart's warm start: a stall restarts it no more");
    }
    if !(controlling(FLIGHT.load()) && crate::control::ticked()) {
        return Some("the board controls no flight (armed, launched by its stage machine, airborne)");
    }
    None
}

/// What `inspect` shows of the watchdog beyond its entry.
#[derive(Serialize)]
pub struct Inspection {
    /// Whether the task watchdog panics the board and watches the supervisor.
    armed: bool,
    /// `wdt_timeout_ms`.
    timeout_ms: Option<u32>,
    /// `period_ms`: the stall check's.
    period_ms: Option<u32>,
    /// `stall_ms`.
    stall_ms: Option<f32>,
    /// Whether the board controls a flight now, its control thread ticking: what the stall check judges.
    controlling: bool,
    /// The control thread's latest tick's age, ms; null before its first.
    tick_age_ms: Option<i32>,
    /// The longest gap between two feeds, ms.
    feed_max_ms: u32,
    /// The longest gap between two stall checks, ms.
    check_max_ms: u32,
    /// Whether this boot's stall restart is suppressed (a stall restart's own warm start).
    restart_suppressed: bool,
}

/// The watchdog's [`Inspection`].
pub fn inspect() -> Inspection {
    let settings = SETTINGS.get();
    let ticked = crate::control::ticked();
    let now_ms = (uptime_us() / 1000) as u32;
    Inspection {
        armed: TASK_WATCHDOG_ARMED.load(Ordering::Relaxed),
        timeout_ms: settings.map(|settings| settings.timeout_ms),
        period_ms: settings.map(|settings| settings.period_ms),
        stall_ms: settings.map(|settings| settings.stall_ms),
        controlling: controlling(FLIGHT.load()) && ticked,
        tick_age_ms: ticked.then(|| now_ms.wrapping_sub(crate::control::last_tick_ms()) as i32),
        feed_max_ms: FEED_MAX_MS.load(Ordering::Relaxed),
        check_max_ms: CHECK_MAX_MS.load(Ordering::Relaxed),
        restart_suppressed: crate::warm::stall_restart_suppressed(),
    }
}

/// health.csv: the vitals it samples and its stream.
struct Health {
    /// The board's vitals (the die temperature).
    board: Arc<Board>,
    /// The CPU load since the last row.
    load: Load,
    /// health.csv, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// A row was refused and none written since: one line per run of refusals.
    overflowing: bool,
}

impl Health {
    /// The stream of `board`'s vitals, counting the load from now.
    fn new(board: Arc<Board>) -> Health {
        Health {
            board,
            load: Load::new(),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::health_stream())
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            overflowing: false,
        }
    }

    /// One row at `now_us`, with a recorder.
    fn row(&mut self, now_us: u64) {
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        let row = [
            self.board.celsius().map_or(Cell::None, Cell::F32),
            Cell::I64(board::heap_free(sys::MALLOC_CAP_8BIT) as i64),
            Cell::I32(self.load.busiest_percent(now_us)),
            Cell::None,   // oom_s
            Cell::None,   // land_s
            Cell::I32(0), // leak_kbps
            Cell::I32(0), // rescues
            Cell::I32(0), // rescue_ms
        ];
        // telemetry raises (the error policy): the recorder counts the row in tlm.dropped, and the first of a run of
        // refusals is logged
        match telemetry.push(now_us, &row) {
            Ok(()) => self.overflowing = false,
            Err(overflow) if !self.overflowing => {
                log::warn!("supervisor :: health.csv row: {overflow}");
                self.overflowing = true;
            }
            Err(_) => {}
        }
    }
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
