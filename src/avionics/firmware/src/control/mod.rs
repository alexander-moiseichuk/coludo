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
//! counted by a core-1 GPIO interrupt (`irq_runs`, plan B's evidence of a live line), one 16-byte burst from its status
//! register, and only a new conversion published: `accel` on XLDA, `rate` on GDA, the row on either, written on this
//! thread's own producer ring; [`STALE_LIMIT`] reads in a row with neither make the part unhealthy, with one line (a
//! part slower than the tick reads stale now and then: that is its ODR, not a fault); the separation switch (M6) --
//! a both-edge
//! GPIO interrupt on core 1 records the contact's episode (its first and latest edge, esp_timer us) into a stamp pair;
//! each tick reads the pin, offers the read (observed, or polled when the arm failed), takes the pair once no edge came
//! for `debounce_ms` and concludes it by that read -- separated, nested or a blip, each recorded with its moment and
//! wobble in the durable separation.csv. In BOOSTING a separation counts only once the thrust gate is open (M7,
//! drivers::separation): the glider's own +x specific force, read from a fresh `accel` each tick, showed the thrust and
//! then its end; before that it is pending, and counts when the gate opens or by the backstop (the pin held open
//! 100 ms). A counted one moves the stage to GLIDING.
//!
//! The stage machine (M7, sequencer::StageMachine), built from the first enabled `sequencer` entry: stepped every
//! `period_ms` on the uptime clock (every tick when the tick is slower), after the requests, with `accel`, `elevation`
//! and `agl` read fresh at that moment. Its own advance commits to the flight word, then logs `controller :: stage ->
//! X` to the console only, then records `<entry> :: stage -> X (reason)` and the `<entry>.csv` row `stage;reason`; a
//! move made from outside (the separation, an operator) writes its row only, before any advance's, as plan B's `_tick`
//! and `_advance` do. The reason text is made here, on core 1, only for the machine's own transition (six or fewer a
//! flight): with the separation's event line, the one exception to "text only off the flight path" (§3.1).
//!
//! When the thread or its tick does not start, each entry it drives (the IMU, the separation switch, the stage
//! machine) is crashed with "the control thread did not start": nothing reads or steps them this boot, so `arm`
//! refuses.
//!
//! Each tick stores its heartbeat, the uptime in ms as a wrapping u32 (the P4 has no 64-bit atomics), which the
//! supervisor's stall check reads (crate::supervisor): one Relaxed store, and one Relaxed load of the flag
//! `panic kind=stall` sets, on which the thread stops for good, so the check is proved on the board.
//!
//! By subsystem (R3 of doc/specs/avionics-refactor.md): this module is the thread itself -- its timer, its wakers,
//! its start and its tick, the figures `control` shows; the filter, the stage machine and the flight law are the
//! control tick's duties (crates/controller `ControlCore`, the SITL's too) behind the board's hooks ([`duties`]);
//! the SPI IMU ([`imu`]), the separation switch ([`separation`]), the GPIO interrupts ([`interrupts`]), the lapse
//! watches ([`watch`]) and the requests ([`requests`]) each have their own file.
//!
//! The flight word's one writer lives here (M7, crate::status): this thread commits every change of the stage, the
//! hold, the arming and the launch, between two ticks' work, so no change races a detector. The separation's GLIDING
//! commits through it, and so do the operator's requests: the executor posts each one to its request slot, which the
//! tick serves first among its requests (§4.3, step 4), and logs it from the answer, so this thread takes no lock and
//! makes no kernel call for a request. The mailbox after the slot carries the SETTING-only `probe` and `bustune`, each
//! checked against the stage again where it is served: a launch detected during a probe sweep never runs a probe. The
//! recorder's flight flag follows each commit: raised before a transition's rows, lowered after them. `control` shows
//! the tick's work time (from the wake to the end of its work), whether the flight word holds the machine's launch,
//! and what the thrust gate has seen.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering};
use std::sync::OnceLock;
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel};

use serde_json::Value;
use std::time::Duration;

use control::attitude::Attitude;
use controller::{ControlCore, DeviceEntry, FilterSetup, FlightSetup, SequencerSetup, Table};
use databoard::{Parameter, Writer};
use drivers::axes::Axes;
use drivers::debounce::StampPair;
use drivers::lsm6dso32::{Lsm6dso32, OverSpi, STALE_LIMIT};
use drivers::separation::{Kind, Thrust, ThrustGate};
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::sys;
use firmware::buses::spi::{SpiBus, SpiHandle};
use firmware::gcl::{self, Command, Gate, Reach};
use recorder::{Cell, Rows, Telemetry};
use sequencer::{ChangeRefused, Commit, FlightFlag, FlightWriter, RequestSlot, Stage};

use crate::devices::{AttitudeMember, SeparationMember, SequencerMember, SpiMember, Topology};
use crate::logging::CONSOLE_ONLY;
use crate::recorder::Producer;
use crate::status::FLIGHT;
use firmware::threads::{self, Spec, Stack};
use firmware::time::uptime_us;

mod duties;
mod imu;
mod interrupts;
mod requests;
mod separation;
mod watch;

pub use self::duties::*;
pub use self::imu::*;
pub use self::interrupts::*;
pub use self::requests::*;
pub use self::separation::*;
pub use self::watch::*;

/// The control thread's priority: above the bus owners (18, 17), below esp_hosted's (§4.2).
const PRIORITY: u8 = 20;

/// The control thread's stack (internal: a flight thread); grown as the tick takes on the IMU, the attitude filter
/// and the flight law, each step measured with `heap stacks`. 6 KB kept 996 bytes never used with the flight loop on
/// the pad, and an IMU probe's float formatting overflowed it (M13).
const STACK_SIZE: usize = 8 << 10;

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
/// The latest tick's work, microseconds: from the wake to the end of its work.
static WORK_LAST_US: AtomicU32 = AtomicU32::new(0);
/// The longest tick's work so far, microseconds.
static WORK_MAX_US: AtomicU32 = AtomicU32::new(0);

/// Set as the thread enters its tick loop, never cleared: until then, and for good when the thread or its timer did
/// not start, nothing serves the request slot, and nothing commits to the flight word.
static RUNNING: AtomicBool = AtomicBool::new(false);

/// The heartbeat the supervisor's stall check reads (controller::Stall): the uptime in ms, as a wrapping u32, of the
/// latest tick. One Relaxed store a tick.
static LAST_TICK_MS: AtomicU32 = AtomicU32::new(0);

/// Set (Release) once the first tick has stored [`LAST_TICK_MS`], never cleared: before it the heartbeat means
/// nothing, and a thread that never ticked is a failed bring-up, not a stall.
static TICKED: AtomicBool = AtomicBool::new(false);

/// Set by `panic kind=stall`, never cleared: at its next tick the thread stops for good, and the supervisor's stall
/// check restarts the board. One Relaxed load a tick.
static STALL: AtomicBool = AtomicBool::new(false);

/// The device name the GNSS publishes `elevation` under (plan B tasks/sequencer.py `_GNSS_SOURCE`): the stage machine
/// binds it to its own slot, which never starts a flight (sequencer::GNSS_SOURCE).
pub const GNSS_SOURCE: &str = "gnss";

/// Why an entry this thread drives is crashed when the thread or its tick did not start (`verify`, `arm`).
const NOT_STARTED: &str = "the control thread did not start";

/// Whether the control thread runs: its tick, and the GPIO interrupt service it installed first.
pub fn running() -> bool {
    RUNNING.load(Ordering::Acquire)
}

/// The tasks the alarm notifies: the control thread, then each running I2C owner. Leaked: the timer runs for the
/// program.
struct Wakers {
    /// The task handles, `count` of them used.
    tasks: [sys::TaskHandle_t; 3],
    /// How many are used.
    count: usize,
    /// The alarms so far: the even ones wake the control thread, the odd ones the owners.
    alarms: AtomicU32,
}

/// The gptimer alarm ISR, at twice the tick rate (D20 of the M9 plan): an even alarm wakes the control thread, an odd
/// one the I2C owners, half a tick after it -- so a backup's push lands half a tick before the control thread reads it
/// (about 5 ms old there, not 10-20), measured needed under the flight's load (the BMI323's lapses passed 1 a minute
/// with the video and the ring load running). eIncrement, so a pile-up counts the missed wakes. Integer only.
unsafe extern "C" fn alarm(
    _timer: sys::gptimer_handle_t,
    _event: *const sys::gptimer_alarm_event_data_t,
    context: *mut core::ffi::c_void,
) -> bool {
    // SAFETY: `context` is the leaked Wakers, alive for the program.
    let wakers = unsafe { &*(context as *const Wakers) };
    let mut woken: sys::BaseType_t = 0;
    let owners = wakers.alarms.fetch_add(1, Ordering::Relaxed) % 2 == 1;
    let tasks = if owners {
        &wakers.tasks[1..wakers.count]
    } else {
        &wakers.tasks[..1]
    };
    for &task in tasks {
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

/// Starts the control thread (core 1, priority 20) ticking at `schedule_hz`, waking the I2C owners half a tick after
/// each tick (D20 of the M9 plan), with the SPI parts that came up (`spi`: the primary IMU), the separation switch, the stage machine, the
/// databoard's `topology`, the device `table` whose entries' health the tick writes, and the flight word's one
/// `writer`, which the thread owns from here on. Call it after the owners started (their task handles are taken here).
///
/// Errors: the thread would not spawn (the writer is gone with it, and the flight word stays as it is); the entries it
/// would drive are crashed then.
#[expect(
    clippy::too_many_arguments,
    reason = "what the bring-up hands the flight thread, each taken once"
)]
pub fn start(
    schedule_hz: u32,
    spi_bus: Option<SpiBus>,
    spi: Vec<SpiMember>,
    separation: Option<SeparationMember>,
    sequencer: Option<SequencerMember>,
    attitude: Option<AttitudeMember>,
    fins: Option<crate::fins::Fins>,
    centre: Option<usize>,
    flight: Option<crate::devices::FlightMember>,
    topology: &'static Topology,
    table: &'static Table<'static>,
    writer: FlightWriter<'static>,
) -> anyhow::Result<()> {
    let driven = [
        spi.first().map(|member| member.name),
        separation.as_ref().map(|member| member.name),
        sequencer.as_ref().map(|member| member.name),
        attitude.as_ref().map(|member| member.name),
    ];
    let spawned = threads::spawn(
        Spec {
            name: c"control",
            stack_size: STACK_SIZE,
            stack: Stack::Internal,
            priority: PRIORITY,
            core: Core::Core1,
        },
        move || {
            run(
                schedule_hz,
                spi_bus,
                spi,
                separation,
                sequencer,
                attitude,
                fins,
                centre,
                flight,
                topology,
                table,
                writer,
            )
        },
    );
    if spawned.is_err() {
        crate::devices::crash(table, driven, NOT_STARTED);
    }
    spawned?;
    Ok(())
}

/// The control thread: the IMU, the separation switch, the stage machine, the timer, then the tick forever.
#[expect(clippy::too_many_arguments, reason = "start's arguments, moved into the thread")]
fn run(
    schedule_hz: u32,
    spi_bus: Option<SpiBus>,
    spi: Vec<SpiMember>,
    separation: Option<SeparationMember>,
    sequencer: Option<SequencerMember>,
    attitude: Option<AttitudeMember>,
    mut fins: Option<crate::fins::Fins>,
    centre: Option<usize>,
    flight: Option<crate::devices::FlightMember>,
    topology: &'static Topology,
    table: &'static Table<'static>,
    mut writer: FlightWriter<'static>,
) {
    let period_us = (TIMER_RESOLUTION_HZ / schedule_hz.max(1)) as i64;
    // this thread's own producer ring: its rows and log lines never contend with another thread's
    let _attachment = crate::recorder::get().and_then(|recorder| recorder.attach(Producer::Control as usize).ok());
    if let Err(error) = install_gpio_service() {
        log::warn!("control :: no GPIO interrupt service ({error}): INT1 and the separation pin go unarmed");
    }
    let mut spi = spi.into_iter();
    let mut imu = spi.next().map(|member| {
        if let Some(gpio) = member.interrupt
            && let Err(error) = arm_interrupt(gpio, interrupt_edge)
        {
            log::warn!(
                "{} :: INT1 GPIO{gpio} not armed ({error}): sampling on the tick alone",
                member.name
            );
        }
        INTERRUPT_WIRED.store(member.interrupt.is_some(), Ordering::Relaxed);
        SPI_CLOCK_HZ.store(spi_bus.as_ref().map_or(0, SpiBus::clock_hz), Ordering::Relaxed);
        let _ = IMU_NAME.set(member.name);
        if u64::from(member.period_ms) * 1000 < period_us as u64 {
            log::warn!(
                "control :: {} wants a sample every {} ms, the tick comes every {} ms: read every tick",
                member.name,
                member.period_ms,
                period_us / 1000
            );
        }
        Imu::new(member, topology, table)
    });
    let watches = Watch::every(topology, table);
    let (mailbox, requests) = sync_channel(REQUESTS_PER_TICK);
    let _ = MAILBOX.set(mailbox);
    for extra in spi {
        log::warn!("control :: {} not driven: one SPI IMU only", extra.name);
    }
    // the gate takes the stage machine's launch numbers: the entry's, else plan B's defaults
    let (launch, origin) = sequencer.as_ref().map_or_else(
        || (sequencer::Settings::default(), "plan B's defaults"),
        |member| (member.settings, member.name),
    );
    let mut switch = separation.map(|member| Switch::new(member, &launch, origin, topology));
    let flight_name = flight.as_ref().map_or("flight", |member| member.name);
    let mut core = control_core(sequencer, attitude, flight, topology, period_us as u64);
    let mut tasks = [core::ptr::null_mut(); 3];
    // SAFETY: a plain read of this task's own handle.
    tasks[0] = unsafe { sys::xTaskGetCurrentTaskHandle() };
    let mut count = 1;
    for task in (0..2).filter_map(crate::owner::task) {
        tasks[count] = task;
        count += 1;
    }
    let wakers: &'static Wakers = Box::leak(Box::new(Wakers {
        tasks,
        count,
        alarms: AtomicU32::new(0),
    }));
    // the alarm at half the tick: the control thread on the even alarms, the owners on the odd ones
    if let Err(error) = start_timer(period_us as u64 / 2, wakers) {
        log::error!("control :: no tick ({error}): the control thread stops");
        // the switch's name is in SWITCH once Switch::new ran
        let driven = [
            imu.as_ref().map(|imu| imu.name),
            SWITCH.get().map(|(name, _)| *name),
            core.sequencer(),
            core.filter(),
        ];
        crate::devices::crash(table, driven, NOT_STARTED);
        return;
    }
    SCHEDULE_HZ.store(schedule_hz, Ordering::Relaxed);
    log::info!(
        "control :: ticking at {schedule_hz} Hz on core 1 (gptimer), waking {} bus owner(s)",
        count - 1
    );
    let mut first_us = None;
    let mut ticks: i64 = 0;
    // the fins' probe sweep under way, and the INA226's power it samples
    let mut sweeping: Sweeping = None;
    // DONE's fin rest (M11b): when DONE began, and whether the fins rest
    let mut done_since_us: Option<u64> = None;
    let mut rested = false;
    // a cold boot's fin centring, stepped below: it never holds this thread's start (crate::fins)
    if let (Some(fins), Some(concurrency)) = (fins.as_mut(), centre) {
        fins.centre(concurrency);
    }
    let power = topology.scalar("power");
    // the flight loop's surfaces bound to their servos (M13): the law actuates through them, every tick
    let bound = core
        .law()
        .map(|law| crate::flight::bind(flight_name, law, fins.as_ref()))
        .unwrap_or_default();
    RUNNING.store(true, Ordering::Release);
    loop {
        // SAFETY: this task's own notification value, index 0, cleared on take.
        let pending = unsafe { sys::ulTaskGenericNotifyTake(0, 1, u32::MAX) };
        let now_us = uptime_us() as i64;
        if pending == 0 {
            continue;
        }
        if STALL.load(Ordering::Relaxed) {
            stop();
        }
        if pending > 1 {
            OVERRUNS.fetch_add(pending - 1, Ordering::Relaxed);
        }
        let first_tick = first_us.is_none();
        let first = *first_us.get_or_insert(now_us);
        ticks += i64::from(pending);
        TICKS.store(ticks as u32, Ordering::Relaxed);
        // the heartbeat, then (once) the word that vouches for it: the supervisor loads them in the other order
        LAST_TICK_MS.store((now_us / 1000) as u32, Ordering::Relaxed);
        if first_tick {
            TICKED.store(true, Ordering::Release);
        }
        let late = (now_us - first - (ticks - 1) * period_us).clamp(0, i64::from(u32::MAX)) as u32;
        LATE_LAST_US.store(late, Ordering::Relaxed);
        LATE_MAX_US.fetch_max(late, Ordering::Relaxed);
        if late > LATENESS_BUDGET_US {
            LATE_OVER_BUDGET.fetch_add(1, Ordering::Relaxed);
        }
        // DONE (M11b): the IMU and the attitude filter stop, the fins rest a second in (after the law centred them)
        let done = writer.flight().stage == Stage::Done;
        if let Some(imu) = &mut imu
            && !done
        {
            imu.tick(now_us as u64, period_us as u64 / 2);
        }
        if !done {
            done_since_us = None;
        } else if let Some(fins) = fins.as_mut() {
            let since = *done_since_us.get_or_insert(now_us as u64);
            if !rested && now_us as u64 >= since + REST_AFTER_US {
                fins.rest();
                rested = true;
            }
        }
        rested &= done;
        if let Some(switch) = &mut switch {
            // the switch reads its own clock after the level: the IMU's SPI work runs between the wake and here
            switch.tick(&mut writer);
        }
        // the operator's request first: lock-free, and its answer is the executor's to log
        if let Some(Ok(commit)) = EXECUTOR_SLOT.serve(&mut writer) {
            follow_flight(&commit);
        }
        if let Some(fins) = fins.as_mut().filter(|fins| fins.centring()) {
            let centring_us = uptime_us();
            fins.step_centring(centring_us, writer.flight().stage == Stage::Setting);
        }
        serve(
            &requests,
            imu.as_mut(),
            spi_bus.as_ref(),
            fins.as_mut(),
            &mut sweeping,
            writer.flight().stage,
        );
        if let (Some(fins), Some((sweep, _))) = (fins.as_mut(), sweeping.as_mut()) {
            let flight = writer.flight();
            let may_move = flight.stage == Stage::Setting && !flight.armed && !flight.launched;
            let sweep_us = uptime_us();
            let power = power
                .and_then(|parameter| parameter.read(sweep_us))
                .map(|reading| reading.value);
            if let Some(swept) = fins.step_sweep(sweep, sweep_us, power, may_move)
                && let Some((_, reply)) = sweeping.take()
            {
                let _ = reply.send(Ok(swept));
            }
        }
        // the consumers' view of the IMU channels, where the stage machine reads them
        let read_us = uptime_us();
        for watch in &watches {
            watch.look(read_us);
        }
        // the filter, the stage machine and the law (crates/controller ControlCore, the SITL's too)
        let mut board = Board {
            fins: fins.as_mut(),
            bound: &bound,
        };
        core.tick(now_us as u64, period_us as u64 / 2, done, &mut writer, &mut board);
        let work = (uptime_us() as i64 - now_us).clamp(0, i64::from(u32::MAX)) as u32;
        WORK_LAST_US.store(work, Ordering::Relaxed);
        WORK_MAX_US.fetch_max(work, Ordering::Relaxed);
    }
}

/// Stops the control thread for good (`panic kind=stall`), holding nothing: no lock is taken on this path, and the
/// flight word stays as last committed. The timer goes on waking the bus owners.
fn stop() -> ! {
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

/// The uptime in ms, as a wrapping u32, of the latest tick ([`ticked`] says whether there was one).
pub fn last_tick_ms() -> u32 {
    LAST_TICK_MS.load(Ordering::Relaxed)
}

/// Whether a tick has stored the heartbeat this boot. Acquire: load it before [`last_tick_ms`], so a check that
/// straddles the first tick never pairs `true` with the boot's 0 (controller::Stall).
pub fn ticked() -> bool {
    TICKED.load(Ordering::Acquire)
}

/// Stops the control thread at its next tick (`panic kind=stall`): the supervisor's stall check is what restarts the
/// board. The caller checks first that it will (crate::supervisor::stall_refused).
pub fn stall() {
    STALL.store(true, Ordering::Relaxed);
}

/// Moves the recorder's flight flag as `commit` moves it (sequencer::Commit::flight_flag), at once: a commit whose
/// rows are not the stage machine's (an operator's request, which writes none; the separation, which never changes
/// `airborne()`). It follows `airborne()` of each committed stage, an operator hold's too.
fn follow_flight(commit: &Commit) {
    if let Some(flag) = commit.flight_flag() {
        set_flight(flag == FlightFlag::Raise);
    }
}

/// Raises (`flight`) or lowers the recorder's flight flag, with a recorder.
fn set_flight(flight: bool) {
    if let Some(recorder) = crate::recorder::get() {
        recorder.set_flight(flight);
    }
}

/// How long into DONE the fins keep their pulse, µs: the law's leaving-control centring travels first (M11b).
const REST_AFTER_US: u64 = 1_000_000;

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
    gcl::register(vec![
        mute_command(),
        Command {
            owner: "control",
            words: "control",
            about: "the control tick: its rate, ticks, overruns (ticks the thread missed), lateness (the latest, the \
                largest, and how many ticks passed the 200 us budget) and work (the latest and the largest, from the \
                wake to the end of the tick's work); the Rust allocations made on core 1 since boot (the flight core: \
                they should stop at the bring-up's); whether the flight word holds the stage machine's launch; the \
                IMU's reads, failures, reads with no new conversion, halves pushed raced and flagged over identical \
                bytes (accel/rate), each watched channel's lapses (source.quantity:count), INT1 edges at the latest tick and whether INT1 is \
                silent; the IMU's SPI clock; the separation latch, whether its pin is polled, the events it \
                recorded (blips among them), whether a separation is pending, and what the thrust gate has seen since \
                boosting began (none, seen, ended); flight-core log lines the console queue dropped",
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
                        ("work_last_us", &WORK_LAST_US.load(Ordering::Relaxed).to_string()),
                        ("work_max_us", &WORK_MAX_US.load(Ordering::Relaxed).to_string()),
                        (
                            "flight_core_allocations",
                            &firmware::allocator::flight_core_allocations().to_string(),
                        ),
                        ("launched", &FLIGHT.load().launched.to_string()),
                        ("imu_reads", &IMU_READS.load(Ordering::Relaxed).to_string()),
                        ("imu_failures", &IMU_FAILURES.load(Ordering::Relaxed).to_string()),
                        ("imu_stale", &IMU_STALE.load(Ordering::Relaxed).to_string()),
                        ("imu_raced", &pair(&IMU_RACED)),
                        ("imu_identical", &pair(&IMU_IDENTICAL)),
                        ("lapses", &lapses()),
                        ("imu_muted", &IMU_MUTED.load(Ordering::Relaxed).to_string()),
                        ("irq_runs_last", &IRQ_RUNS_LAST.load(Ordering::Relaxed).to_string()),
                        ("int1_silent", &INTERRUPT_SILENT.load(Ordering::Relaxed).to_string()),
                        ("separated", &SEPARATED.load(Ordering::Relaxed).to_string()),
                        (
                            "separation_polled",
                            &SEPARATION_POLLED.load(Ordering::Relaxed).to_string(),
                        ),
                        (
                            "separation_events",
                            &SEPARATION_EVENTS.load(Ordering::Relaxed).to_string(),
                        ),
                        (
                            "separation_blips",
                            &SEPARATION_BLIPS.load(Ordering::Relaxed).to_string(),
                        ),
                        ("pending", &SEPARATION_PENDING.load(Ordering::Relaxed).to_string()),
                        ("thrust", THRUSTS[usize::from(THRUST.load(Ordering::Relaxed))].name()),
                        ("console_dropped", &crate::logging::console_dropped().to_string()),
                        ("spi_clock_hz", &SPI_CLOCK_HZ.load(Ordering::Relaxed).to_string()),
                    ],
                )
            }),
        },
    ]);
}

/// A per-half counter pair as `accel/rate`.
fn pair(counters: &[AtomicU32; 2]) -> String {
    format!(
        "{}/{}",
        counters[0].load(Ordering::Relaxed),
        counters[1].load(Ordering::Relaxed)
    )
}
