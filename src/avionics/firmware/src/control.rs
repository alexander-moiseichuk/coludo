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
//! The flight word's one writer lives here (M7, crate::status): this thread commits every change of the stage, the
//! hold, the arming and the launch, between two ticks' work, so no change races a detector. The separation's GLIDING
//! commits through it, and so do the operator's requests: the executor posts each one to its request slot, which the
//! tick serves first among its requests (§4.3, step 4), and logs it from the answer, so this thread takes no lock and
//! makes no kernel call for a request. The mailbox after the slot carries the SETTING-only `probe` and `bustune`, each
//! checked against the stage again where it is served: a launch detected during a probe sweep never runs a probe. The
//! recorder's flight flag follows each commit: raised before a transition's rows, lowered after them. `control` shows
//! the tick's work time (from the wake to the end of its work), whether the flight word holds the machine's launch,
//! and what the thrust gate has seen.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use std::sync::OnceLock;
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel};
use std::time::Duration;

use controller::{DeviceEntry, Table};
use databoard::{Parameter, Writer};
use drivers::axes::Axes;
use drivers::debounce::StampPair;
use drivers::lsm6dso32::{Lsm6dso32, OverSpi, STALE_LIMIT};
use drivers::separation::{Kind, Thrust, ThrustGate};
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;
use firmware::buses::spi::{SpiBus, SpiHandle};
use firmware::gcl::{self, Command, Gate, Reach};
use recorder::{Cell, Telemetry};
use sequencer::{
    Binding, ChangeRefused, Commit, Decider, FlightFlag, FlightWriter, RequestSlot, Stage, StageMachine, Transition,
    VetoEdge,
};

use crate::devices::{SeparationMember, SequencerMember, SpiMember, Topology};
use crate::logging::CONSOLE_ONLY;
use crate::recorder::Producer;
use crate::status::FLIGHT;

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
/// The second IMU's INT1 edges since its owner last took them ([`take_second_edges`]): the BMI323's data-ready line,
/// evidence only, its reads paced by the owner's tick (M9).
static SECOND_EDGES: AtomicU32 = AtomicU32::new(0);
/// IMU reads `imu mute` skipped (M9).
static IMU_MUTED: AtomicU32 = AtomicU32::new(0);
/// IMU samples read.
static IMU_READS: AtomicU32 = AtomicU32::new(0);
/// IMU reads that failed.
static IMU_FAILURES: AtomicU32 = AtomicU32::new(0);
/// IMU reads that found no new conversion: about 1 - ODR/100 of the ticks on a part slower than the tick.
static IMU_STALE: AtomicU32 = AtomicU32::new(0);
/// IMU halves pushed by the driver's correction (a), by half (accel, rate): the data-ready flag clear but the bytes
/// changed, a conversion that landed between the status byte and its data (M9).
static IMU_RACED: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
/// IMU halves whose data-ready flag was set over bytes identical to the last read's, by half (accel, rate): a raced
/// conversion's duplicate (skipped, correction (b)) or a bit-identical new conversion (pushed). What D1's residual is
/// measured by (M9).
static IMU_IDENTICAL: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
/// The IMU channels the consumer side watches ([`Watch`]): an IMU's `accel` and `rate`, two IMUs.
const WATCHED: usize = 4;
/// Each watched channel's lapses, by [`WATCH_NAMES`]' order: ticks at which the channel was not fresh in its
/// quantity's shared window while its part was healthy -- what a consumer (the stage machine, the attitude filter) saw
/// as a gap (M9).
static LAPSES: [AtomicU32; WATCHED] = [const { AtomicU32::new(0) }; WATCHED];
/// The watched channels' names, `<source>.<quantity>`, once the control thread starts.
static WATCH_NAMES: OnceLock<Vec<String>> = OnceLock::new();
/// The latest tick's `irq_runs`.
static IRQ_RUNS_LAST: AtomicU32 = AtomicU32::new(0);
/// Whether INT1 is silent (the driver's verdict: three wakes in a row with no edge).
static INTERRUPT_SILENT: AtomicBool = AtomicBool::new(false);
/// The IMU's entry name, once the control thread drives one.
static IMU_NAME: OnceLock<&'static str> = OnceLock::new();
/// Whether the IMU's INT1 is wired (its `int_pin` names a GPIO).
static INTERRUPT_WIRED: AtomicBool = AtomicBool::new(false);
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

/// The executor's request slot (§4.3, step 4): its `stage`, `arm` and `disarm` reach the flight word's writer
/// through it.
/// The executor's alone: another requesting thread gets a slot of its own.
static EXECUTOR_SLOT: RequestSlot = RequestSlot::new();

/// How long the executor waits for the tick to take its request before withdrawing it (§4.4): two hundred ticks at
/// the default 100 Hz, and two at the slowest rate a config may set (1 Hz), so a thread that took none has stalled.
const REQUEST_DEADLINE: Duration = Duration::from_secs(2);

/// How often the executor looks for its answer: one FreeRTOS tick (CONFIG_FREERTOS_HZ is 1000).
const REQUEST_POLL: Duration = Duration::from_millis(1);

/// What the control thread's mailbox asks between its tick's work (§4.3, step 4), each answer going back through its
/// sender: the outcome, or the stage's refusal outside SETTING (a value: the executor makes its text).
enum Request {
    /// Probe the IMU: None healthy, else why.
    ProbeImu(SyncSender<Result<Option<String>, ChangeRefused>>),
    /// Run the IMU's SPI handle at `clock_hz` and probe it (a bustune rung): the verdict and the clock it runs at.
    BustuneSpi {
        clock_hz: u32,
        reply: SyncSender<Result<(Option<String>, u32), ChangeRefused>>,
    },
    /// Skip the IMU's reads for `duration_ms` (`imu mute`, M9): whether an IMU took it.
    Mute {
        duration_ms: u32,
        reply: SyncSender<Result<bool, ChangeRefused>>,
    },
}

/// The control thread's mailbox, once it runs.
static MAILBOX: OnceLock<SyncSender<Request>> = OnceLock::new();

/// The fastest SPI clock a rung may ask for: the LSM6DSO32's SPI maximum, 10 MHz.
const MAXIMUM_SPI_HZ: u32 = 10_000_000;

/// The clock the IMU's SPI handle runs at, Hz (0 without one).
static SPI_CLOCK_HZ: AtomicU32 = AtomicU32::new(0);

/// How many mailbox requests a tick serves at most (§4.3: two), after the request slot's one.
const REQUESTS_PER_TICK: usize = 2;

/// How long a probe waits for the control thread's answer: a tick and a frame, with a wide margin.
const PROBE_DEADLINE: Duration = Duration::from_secs(2);

/// The separation pin's open episode, its first and latest edge (esp_timer's us truncated to u32): the GPIO interrupt
/// records into it, the tick takes it once quiet (drivers::debounce).
static SEPARATION: StampPair = StampPair::new();
/// Events the separation switch recorded since boot, separation.csv's rows (`control`): an episode concluded, or
/// under the thrust gate a pending separation counted or cleared.
static SEPARATION_EVENTS: AtomicU32 = AtomicU32::new(0);
/// Of those, the blips: the contact moved and came back (`control`).
static SEPARATION_BLIPS: AtomicU32 = AtomicU32::new(0);
/// A separation concluded in BOOSTING waits for the thrust gate or the backstop (`control`'s `pending`).
static SEPARATION_PENDING: AtomicBool = AtomicBool::new(false);
/// What the thrust gate has seen since BOOSTING entry, as its index in [`THRUSTS`] (`control`'s `thrust`).
static THRUST: AtomicU8 = AtomicU8::new(0);
/// Arming the separation pin's edge interrupt failed: the tick polls it (`control`).
static SEPARATION_POLLED: AtomicBool = AtomicBool::new(false);
/// The separation switch's debounced latch, for the console and (M8) the warm-start gate: true = separated.
static SEPARATED: AtomicBool = AtomicBool::new(false);

/// The separation switch's entry name and GPIO, once the control thread drives it (its probe reads the pin).
static SWITCH: OnceLock<(&'static str, u8)> = OnceLock::new();

/// The thrust gate's verdicts, in the order [`THRUST`] indexes them.
const THRUSTS: [Thrust; 3] = [Thrust::Unseen, Thrust::Seen, Thrust::Ended];

/// The body frame's +x, the nose, in an `accel` vector (x, y, z): the axis the thrust gate reads (the body frame is
/// +x nose, +y left, +z up; each airframe's mount is checked nose up, where +x reads about +1 g).
const AXIAL: usize = 0;

/// The device name the GNSS publishes `elevation` under (plan B tasks/sequencer.py `_GNSS_SOURCE`): the stage machine
/// binds it to its own slot, which never starts a flight (sequencer::GNSS_SOURCE).
pub const GNSS_SOURCE: &str = "gnss";

/// Why an entry this thread drives is crashed when the thread or its tick did not start (`verify`, `arm`).
const NOT_STARTED: &str = "the control thread did not start";

/// Publishes the separation latch: bring-up seeds it from the level it read, the tick keeps it.
pub fn publish_separated(separated: bool) {
    SEPARATED.store(separated, Ordering::Relaxed);
}

/// The separation switch's name and pin level (true = HIGH = nested), for its probe; None without one.
pub fn switch_level() -> Option<(&'static str, bool)> {
    let (name, gpio) = *SWITCH.get()?;
    // SAFETY: a plain read of the input bring-up configured.
    Some((name, unsafe { sys::gpio_get_level(i32::from(gpio)) } != 0))
}

/// The separation switch as the control thread drives it: its pin, its debounce, its thrust gate and its stream.
struct Switch {
    /// Its GPIO.
    gpio: u8,
    /// The previous read's level while the pin is polled (arming the edge interrupt failed); None while armed.
    polled: Option<bool>,
    /// The latch.
    part: drivers::separation::Separation,
    /// The thrust gate: in BOOSTING a separation counts only once it is open, or by the backstop.
    gate: ThrustGate,
    /// The `accel` quantity the gate reads its a_x from, when a provider of it came up.
    accel: Option<&'static Parameter<[f32; 3]>>,
    /// separation.csv, with a recorder.
    telemetry: Option<Telemetry<'static>>,
}

impl Switch {
    /// The switch of `member`, its edge interrupt armed on this thread (core 1), its thrust gate on `launch`'s
    /// `launch_g` and `launch_ms` (the stage machine's; `origin` says whose) reading `accel` from `topology`.
    fn new(
        member: SeparationMember,
        launch: &sequencer::Settings,
        origin: &str,
        topology: &'static Topology,
    ) -> Switch {
        let accel = topology.vector("accel");
        match accel {
            Some(_) => log::info!(
                "separation :: thrust gate on +x: thrust above {} g for {} ms, from {origin}",
                launch.launch_g,
                launch.launch_ms
            ),
            None => log::warn!("separation :: no accel on the databoard: in boosting only the backstop counts"),
        }
        let polled = match arm_edges(member.gpio) {
            Ok(()) => None,
            Err(error) => {
                log::warn!(
                    "separation :: GPIO{} edge interrupt not armed ({error}): the pin is polled every tick, and a \
                     change and its undoing between two reads go unseen",
                    member.gpio
                );
                // the first read is compared with the level bring-up read
                Some(member.part.latched())
            }
        };
        SEPARATION_POLLED.store(polled.is_some(), Ordering::Relaxed);
        // a change between bring-up's read and the arming is the tick's own read's to see (StampPair::observe)
        SEPARATED.store(member.part.separated(), Ordering::Relaxed);
        let _ = SWITCH.set((member.name, member.gpio));
        Switch {
            gpio: member.gpio,
            polled,
            part: member.part,
            gate: ThrustGate::new(launch),
            accel,
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(member.name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
        }
    }

    /// One tick: the pin's level, then the clock, offered to the stamp pair (observed, or polled when the arm failed);
    /// the thrust gate stepped with the glider's a_x from a fresh `accel`; then the switch's tick under the gate
    /// (drivers::separation `Separation::tick`): a quiet episode concluded by that same level, or a pending separation
    /// counted or cleared. A separation that advances BOOSTING commits GLIDING through `writer` and logs the
    /// controller's console line, then the row is written with the moment and the wobble, and the event is logged
    /// (plan B's order, drivers/separation.py `_apply`).
    fn tick(&mut self, writer: &mut FlightWriter<'static>) {
        // the level first, then the clock: the level decides the episode this tick concludes
        // SAFETY: a plain read of the input bring-up configured.
        let high = unsafe { sys::gpio_get_level(i32::from(self.gpio)) } != 0;
        // SAFETY: a plain read of the monotonic timer.
        let now_us = unsafe { sys::esp_timer_get_time() } as u64;
        let read_us = now_us as u32;
        match &mut self.polled {
            Some(previous) => {
                SEPARATION.poll(read_us, high, *previous, self.part.latched());
                *previous = high;
            }
            None => SEPARATION.observe(read_us, high, self.part.latched()),
        }
        let stage = writer.flight().stage;
        // read at the clock taken after the IMU's push this tick: the databoard refuses a stamp from the future
        let axial_g = self
            .accel
            .and_then(|accel| accel.read(now_us))
            .map(|reading| reading.value[AXIAL]);
        // the uptime's ms as a wrapping u32, the stage machine's clock (ThrustGate::step)
        self.gate.step((now_us / 1000) as u32, stage, axial_g);
        let thrust = self.gate.thrust();
        let index = THRUSTS.iter().position(|seen| *seen == thrust).unwrap_or_default();
        THRUST.store(index as u8, Ordering::Relaxed);
        let event = self.part.tick(&SEPARATION, high, read_us, stage, self.gate.open());
        SEPARATION_PENDING.store(self.part.pending(), Ordering::Relaxed);
        let Some(event) = event else {
            return;
        };
        if event.advanced
            && let Some(commit) = writer.separate()
        {
            follow_flight(&commit);
            log::info!(target: CONSOLE_ONLY, "controller :: stage -> {}", commit.after.stage.name());
        }
        SEPARATED.store(self.part.separated(), Ordering::Relaxed);
        SEPARATION_EVENTS.fetch_add(1, Ordering::Relaxed);
        if event.kind == Kind::Blip {
            SEPARATION_BLIPS.fetch_add(1, Ordering::Relaxed);
        }
        // the moment on the uptime's clock: the read's uptime less the moment's age on the u32 clock (wrap-safe)
        let moment_us = now_us.saturating_sub(u64::from(read_us.wrapping_sub(event.moment_us)));
        if let Some(telemetry) = &mut self.telemetry {
            let [name, stage] = event.row();
            let row = [
                Cell::Text(name),
                Cell::Text(stage),
                Cell::I64(moment_us as i64),
                Cell::I64(i64::from(event.duration_us)),
            ];
            // plan B catches an overflow here and goes on: the stage has moved either way
            if let Err(overflow) = telemetry.push(now_us, &row) {
                log::warn!("separation :: record {overflow}");
            }
        }
        log::info!(
            "separation :: {} -- started {moment_us} us, finished +{} us, {}",
            event.name(),
            event.duration_us,
            u8::from(high)
        );
    }
}

/// A separation-pin edge, recorded into [`SEPARATION`] at esp_timer's us. Integer only, wait-free.
unsafe extern "C" fn separation_edge(_: *mut core::ffi::c_void) {
    // SAFETY: a plain read of the monotonic timer, ISR-safe.
    SEPARATION.record(unsafe { sys::esp_timer_get_time() } as u32);
}

/// Arms a both-edge interrupt on the separation pin (already an input with its pull-down) that records into
/// [`SEPARATION`], through the service [`install_gpio_service`] installed.
fn arm_edges(gpio: u8) -> Result<(), sys::EspError> {
    // SAFETY: the pin bring-up configured; the handler is a static function.
    unsafe {
        sys::esp!(sys::gpio_set_intr_type(
            i32::from(gpio),
            sys::gpio_int_type_t_GPIO_INTR_ANYEDGE
        ))?;
        sys::esp!(sys::gpio_isr_handler_add(
            i32::from(gpio),
            Some(separation_edge),
            core::ptr::null_mut()
        ))?;
        sys::esp!(sys::gpio_intr_enable(i32::from(gpio)))?;
    }
    Ok(())
}

/// The primary IMU as the control thread drives it: the part, its bus handle, its databoard writers and its stream.
struct Imu {
    /// The entry's name: the databoard source and the stream's.
    name: &'static str,
    /// Its handle on SPI2; None only when a bustune rung's re-add and the fall-back to the configured clock both
    /// failed.
    handle: Option<SpiHandle>,
    /// Its chip select.
    chip_select: u8,
    /// The driver.
    part: Lsm6dso32<OverSpi>,
    /// Its table entry, whose `healthy` this thread writes (plan B's `Task._healthy`).
    entry: Option<&'static DeviceEntry<'static>>,
    /// Reads in a row with no new conversion.
    stale_run: u32,
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
    /// Its sampling period, microseconds (`period_ms`, plan B's 10).
    period_us: u64,
    /// When the next read is due, microseconds.
    next_due_us: u64,
    /// How its axes map onto the airframe's (`axes`, M9): applied to every converted sample before the push and the
    /// row.
    axes: Axes,
    /// Until when `imu mute` skips its reads, microseconds; 0 when not muted (M9).
    muted_until_us: u64,
}

impl Imu {
    /// The IMU of `member`, its writers claimed from `topology`, its entry found in `table`, its stream opened.
    fn new(member: SpiMember, topology: &'static Topology, table: &'static Table<'static>) -> Imu {
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
            chip_select: member.handle.chip_select(),
            handle: Some(member.handle),
            part: member.part,
            entry: table.find(member.name),
            stale_run: 0,
            failing: false,
            overflowing: false,
            period_us: u64::from(member.period_ms) * 1000,
            next_due_us: 0,
            axes: member.axes,
            muted_until_us: 0,
        }
    }

    /// `imu mute`: skips the reads for `duration_ms` from `now_us`, the part unhealthy from the first skipped read, so
    /// the databoard hands its channels to the next fresh source; the reads resume by themselves at expiry, at `arm`,
    /// or when the stage leaves SETTING.
    fn mute(&mut self, now_us: u64, duration_ms: u32) {
        self.muted_until_us = now_us + u64::from(duration_ms) * 1000;
        log::warn!("{} :: muted for {duration_ms} ms (imu mute)", self.name);
    }

    /// Plan B's probe (lsm6dso32.py:248-278): WHO_AM_I and one sample, logged in plan B's shape. None when healthy,
    /// else why.
    fn probe(&mut self) -> Option<String> {
        let step = "who_am_i + sample";
        log::info!("{} :: probe: {step} ...", self.name);
        let Some(handle) = self.handle.as_mut() else {
            return Some("no SPI handle (a bustune rung lost it)".to_string());
        };
        match self.part.probe(handle) {
            Ok(sample) => {
                let [x, y, z] = sample.accel_g();
                let [roll, pitch, yaw] = sample.rate_degrees_per_second();
                log::info!(
                    "{} :: probe: {step} ok {:.2}g ({roll:.1},{pitch:.1},{yaw:.1}) dps",
                    self.name,
                    (x * x + y * y + z * z).sqrt()
                );
                None
            }
            Err(fault) => {
                let message = format!("{step}: {fault}");
                log::warn!("{} :: probe FAILED: {message}", self.name);
                Some(message)
            }
        }
    }

    /// A bustune rung (plan B's spibus retune, D21): the handle dropped first (a second device on the chip select
    /// would take the pin over), re-added at `clock_hz` (the first-use discard runs again), then probed. A clock the
    /// driver refuses falls back to the bus's configured one; when that fails too, the IMU has no handle and is
    /// unhealthy. Returns the verdict (None healthy, else why) and the clock the handle runs at (0: none).
    fn retune(&mut self, bus: &SpiBus, clock_hz: u32) -> (Option<String>, u32) {
        self.handle = None;
        // a re-added handle reads afresh: the next read is judged by its flags alone
        self.part.forget();
        match bus.handle_at(self.chip_select, clock_hz) {
            Ok(handle) => {
                self.handle = Some(handle);
                SPI_CLOCK_HZ.store(clock_hz, Ordering::Relaxed);
                (self.probe(), clock_hz)
            }
            Err(error) => {
                let configured = bus.clock_hz();
                log::warn!(
                    "{} :: SPI at {clock_hz} Hz refused ({error}): back to {configured} Hz",
                    self.name
                );
                match bus.handle_at(self.chip_select, configured) {
                    Ok(handle) => {
                        self.handle = Some(handle);
                        SPI_CLOCK_HZ.store(configured, Ordering::Relaxed);
                        (Some(format!("{clock_hz} Hz refused: {error}")), configured)
                    }
                    Err(fallback) => {
                        log::error!("{} :: no SPI handle at {configured} Hz either ({fallback})", self.name);
                        self.set_healthy(false);
                        SPI_CLOCK_HZ.store(0, Ordering::Relaxed);
                        (Some(format!("no SPI handle: {fallback}")), 0)
                    }
                }
            }
        }
    }

    /// Records the latest read's outcome in the table entry: true when that changed it.
    fn set_healthy(&self, healthy: bool) -> bool {
        self.entry.is_some_and(|entry| entry.set_healthy(healthy))
    }

    /// One tick: when its period is due (within half a tick, `slack_us`), take the INT1 edges since the last read,
    /// read the status burst, and publish what is new (see the module notes). A tick faster than the period skips
    /// reads; edges accumulate.
    fn tick(&mut self, now_us: u64, slack_us: u64) {
        if now_us + slack_us < self.next_due_us {
            return;
        }
        // the next read a period on, or a period from now when the reads fell behind
        self.next_due_us = (self.next_due_us + self.period_us).max(now_us + self.period_us - slack_us);
        if self.muted_until_us != 0 {
            // a mute is a bench diagnostic: arming or leaving SETTING ends it at once
            let flight = FLIGHT.load();
            let ended_by_flight = flight.armed || flight.stage != Stage::Setting;
            if now_us < self.muted_until_us && !ended_by_flight {
                // a declared outage: unhealthy from its first read (no lapse is counted against a part known out),
                // the INT1 edges taken but not judged (a muted part's latched line goes quiet by design)
                IMU_MUTED.fetch_add(1, Ordering::Relaxed);
                INTERRUPT_EDGES.swap(0, Ordering::Relaxed);
                self.stale_run = STALE_LIMIT;
                if self.set_healthy(false) {
                    log::warn!("lsm6dso32 :: muted -- unhealthy");
                }
                return;
            }
            self.muted_until_us = 0;
            // the first read after the mute is judged by its flags alone
            self.part.forget();
            if ended_by_flight {
                log::warn!("{} :: mute ended by arm or a stage change: reading again", self.name);
            } else {
                log::info!("{} :: mute over: reading again", self.name);
            }
        }
        let runs = INTERRUPT_EDGES.swap(0, Ordering::Relaxed);
        IRQ_RUNS_LAST.store(runs, Ordering::Relaxed);
        if self.part.wake(runs) {
            log::warn!("lsm6dso32 :: INT1 silent -- sampling on the tick, the line unproven");
        }
        INTERRUPT_SILENT.store(self.part.interrupt_silent(), Ordering::Relaxed);
        let Some(handle) = self.handle.as_mut() else {
            return;
        };
        let (fresh, sample) = match self.part.read_fresh(handle) {
            Ok(read) => read,
            Err(fault) => {
                IMU_FAILURES.fetch_add(1, Ordering::Relaxed);
                self.set_healthy(false);
                if !self.failing {
                    log::warn!("lsm6dso32 :: read {fault}");
                    self.failing = true;
                }
                return;
            }
        };
        self.failing = false;
        IMU_READS.fetch_add(1, Ordering::Relaxed);
        for (half, raced, identical) in [
            (0, fresh.accel_raced, fresh.accel_identical),
            (1, fresh.rate_raced, fresh.rate_identical),
        ] {
            if raced {
                IMU_RACED[half].fetch_add(1, Ordering::Relaxed);
            }
            if identical {
                IMU_IDENTICAL[half].fetch_add(1, Ordering::Relaxed);
            }
        }
        if !fresh.accel && !fresh.rate {
            IMU_STALE.fetch_add(1, Ordering::Relaxed);
            self.stale_run = self.stale_run.saturating_add(1);
            if self.stale_run >= STALE_LIMIT && self.set_healthy(false) {
                log::warn!("lsm6dso32 :: no new conversion -- stale");
            }
            return;
        }
        self.stale_run = 0;
        self.set_healthy(true);
        // the airframe's axes, on converted values (drivers::axes): the push and the row alike
        let accel = self.axes.apply(sample.accel_g());
        if let (true, Some(writer)) = (fresh.accel, &mut self.accel) {
            writer.push(accel, now_us);
        }
        if let (true, Some(writer)) = (fresh.rate, &mut self.rate) {
            writer.push(self.axes.apply(sample.rate_degrees_per_second()), now_us);
        }
        // measured -> reported (the operator, M9): a row for every fresh sample, never decimated
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        let [gx, gy, gz] = self.axes.apply(sample.rate_centidegrees_per_second());
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

/// One IMU channel the consumer side watches for lapses (M9): a tick at which the channel is not fresh in its
/// quantity's shared window while its part is healthy, as the stage machine and the attitude filter read it. Counted
/// from the channel's first push, so a boot's bring-up is no lapse.
struct Watch {
    /// The source's channel.
    channel: &'static databoard::Channel<[f32; 3]>,
    /// The quantity's shared freshness window, microseconds.
    window_us: u32,
    /// The source's table entry, whose health gates the count.
    entry: &'static DeviceEntry<'static>,
    /// Its counter in [`LAPSES`].
    lapses: &'static AtomicU32,
}

impl Watch {
    /// Every source's `accel` and `rate` channel, by quantity in rank order, at most [`WATCHED`]: the SPI IMU's on
    /// this thread and the BMI323's on its bus owner alike. Their names published for `inspect`.
    fn every(topology: &'static Topology, table: &'static Table<'static>) -> Vec<Watch> {
        let mut watches = Vec::new();
        let mut names = Vec::new();
        for quantity in ["accel", "rate"] {
            let Some(parameter) = topology.vector(quantity) else {
                continue;
            };
            for source in parameter.sources() {
                let (Some(channel), Some(entry), Some(lapses)) =
                    (parameter.channel(source), table.find(source), LAPSES.get(watches.len()))
                else {
                    continue;
                };
                names.push(format!("{source}.{quantity}"));
                watches.push(Watch {
                    channel,
                    window_us: parameter.window_us(),
                    entry,
                    lapses,
                });
            }
        }
        let _ = WATCH_NAMES.set(names);
        watches
    }

    /// One look at `read_us`: a lapse when the part is healthy and the channel, pushed before, is not fresh.
    fn look(&self, read_us: u64) {
        if self.entry.healthy()
            && self.channel.pushed_at().is_some()
            && self.channel.recent(self.window_us, read_us).is_none()
        {
            self.lapses.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// The stage machine as the control thread steps it (plan B's Sequencer task): the machine, the quantities it reads,
/// its stream and its pace.
struct Sequencer {
    /// The entry's name: the descriptor of its recorded line, and its stream's.
    name: &'static str,
    /// The machine, boxed on the internal heap at the thread's start (~2.4 KB the control stack does not carry).
    machine: Box<StageMachine>,
    /// Which slot each source holds, bound once at the thread's start from the databoard's sources in rank order (M9).
    binding: Binding,
    /// `vetoes.csv`, the veto episodes' begin and end rows, with a recorder (M9).
    vetoes: Option<Telemetry<'static>>,
    /// A veto row was refused and logged: once.
    veto_refusal_logged: bool,
    /// The `accel` quantity (g), when a provider of it came up.
    accel: Option<&'static Parameter<[f32; 3]>>,
    /// The `elevation` quantity (m above the pad).
    elevation: Option<&'static Parameter<f32>>,
    /// The `agl` quantity (the laser's m above the ground).
    height_above_ground: Option<&'static Parameter<f32>>,
    /// `<name>.csv`, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// A row was refused and logged: once, never again (plan B's `note`, which this task never re-arms).
    refusal_logged: bool,
    /// Its step period, microseconds (`period_ms`).
    period_us: u64,
    /// When the next step is due, microseconds.
    next_due_us: u64,
}

impl Sequencer {
    /// The machine of `member`, its quantities found in `topology`, each source bound to its slot (one line per class,
    /// and one per source past its class's slots), its streams opened.
    fn new(member: SequencerMember, topology: &'static Topology) -> Sequencer {
        let accel = topology.vector("accel");
        let elevation = topology.scalar("elevation");
        let height_above_ground = topology.scalar("agl");
        let binding = Binding::new(
            accel.into_iter().flat_map(|parameter| parameter.sources()),
            elevation.into_iter().flat_map(|parameter| parameter.sources()),
            height_above_ground
                .into_iter()
                .flat_map(|parameter| parameter.sources()),
        );
        let slots = |decider: fn(usize) -> Decider, count: usize| {
            (0..count)
                .filter_map(|slot| binding.source(decider(slot)))
                .collect::<Vec<_>>()
                .join(", ")
        };
        let text = format!(
            "accel [{}], barometers [{}], gnss [{}]",
            slots(Decider::Accelerometer, sequencer::ACCELEROMETER_SLOTS),
            slots(Decider::Barometer, sequencer::BAROMETER_SLOTS),
            binding.source(Decider::Gnss).unwrap_or("")
        );
        log::info!("{} :: sources: {text}", member.name);
        for ignored in binding.ignored() {
            log::warn!("{} :: {ignored}", member.name);
        }
        let _ = BINDING.set((text, binding.ignored().iter().map(ToString::to_string).collect()));
        let open = |stream: Option<recorder::StreamId>| {
            crate::recorder::get()
                .zip(stream)
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream))
        };
        Sequencer {
            name: member.name,
            machine: Box::new(StageMachine::new(member.settings)),
            binding,
            vetoes: open(crate::recorder::device_stream(crate::recorder::VETOES)),
            veto_refusal_logged: false,
            accel,
            elevation,
            height_above_ground,
            telemetry: open(crate::recorder::device_stream(member.name)),
            refusal_logged: false,
            period_us: u64::from(member.settings.period_ms) * 1000,
            next_due_us: 0,
        }
    }

    /// One tick: when its period is due (within half a tick, `slack_us`, of `now_us`, the wake), one step of the
    /// machine on the flight as `writer` last committed it, with the quantities read fresh now (plan B `_tick`). A move
    /// made from outside since the last step writes its row only; the machine's own advance commits through `writer`,
    /// then logs `controller :: stage -> X` to the console, then records `<name> :: stage -> X (reason)` and the row
    /// (plan B `_advance`). The recorder's flight flag rises before those rows and falls after them.
    fn tick(&mut self, now_us: u64, slack_us: u64, writer: &mut FlightWriter<'static>) {
        if now_us + slack_us < self.next_due_us {
            return;
        }
        // the next step a period on, or a period from now when the steps fell behind
        self.next_due_us = (self.next_due_us + self.period_us).max((now_us + self.period_us).saturating_sub(slack_us));
        // after the IMU's push and the separation's commit: the databoard refuses a stamp from the future
        // SAFETY: a plain read of the monotonic timer.
        let read_us = unsafe { sys::esp_timer_get_time() } as u64;
        // each bound source's own channel, fresh in its quantity's shared window (M9: the multi-source machine)
        let readings = self.binding.readings(
            |source| own_reading(self.accel, source, read_us),
            |source| own_reading(self.elevation, source, read_us),
            |source| own_reading(self.height_above_ground, source, read_us),
        );
        let flight = writer.flight();
        let step = self
            .machine
            .step((read_us / 1000) as u32, flight.stage, readings, flight.manual);
        if let Some(external) = step.external {
            // a constant text: core 1 formats nothing for a move it did not make
            self.record(read_us, external.to, "external");
        }
        for edge in step.vetoes() {
            self.veto(read_us, edge);
        }
        if let Some(transition) = step.advance {
            self.advance(read_us, transition, writer);
        }
    }

    /// One `vetoes.csv` row at `now_us` for a veto episode's `edge`: the detector, the edge, the deciding source, the
    /// vetoes that held it, the vote and the decider's value, every cell a constant or a number (no text made on core
    /// 1). A refused row is logged once.
    fn veto(&mut self, now_us: u64, edge: VetoEdge) {
        VETO_EDGES.fetch_add(1, Ordering::Relaxed);
        let Some(telemetry) = &mut self.vetoes else {
            return;
        };
        let veto = edge.veto();
        let row = [
            Cell::Text(veto.detector.name()),
            Cell::Text(edge.name()),
            Cell::Text(self.binding.source(veto.decider).unwrap_or("")),
            Cell::Text(veto.kinds()),
            Cell::I32(i32::from(veto.contrary)),
            Cell::I32(i32::from(veto.live)),
            Cell::F32(veto.value),
        ];
        if let Err(overflow) = telemetry.push(now_us, &row)
            && !self.veto_refusal_logged
        {
            log::warn!("{} :: vetoes {overflow}", self.name);
            self.veto_refusal_logged = true;
        }
    }

    /// The machine's own `transition` at `now_us`: the commit, the controller's console line, the recorded line and
    /// the row, in plan B's order, the flight flag around them.
    fn advance(&mut self, now_us: u64, transition: Transition, writer: &mut FlightWriter<'static>) {
        let flag = writer.advance(transition).flight_flag();
        if flag == Some(FlightFlag::Raise) {
            set_flight(true);
        }
        let stage = transition.to.name();
        log::info!(target: CONSOLE_ONLY, "controller :: stage -> {stage}");
        // the reason's text, made on core 1: a machine transition is the one exception (see the module notes)
        let reason = transition.reason.to_string();
        log::info!("{} :: stage -> {stage} ({reason})", self.name);
        self.record(now_us, transition.to, &reason);
        if flag == Some(FlightFlag::Lower) {
            set_flight(false);
        }
    }

    /// One `<name>.csv` row at `now_us`: `stage` and `reason`. A refused row is logged once and never stops the
    /// machine: the stage has moved either way (plan B `_record`).
    fn record(&mut self, now_us: u64, stage: Stage, reason: &str) {
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        if let Err(overflow) = telemetry.push(now_us, &[Cell::Text(stage.name()), Cell::Text(reason)])
            && !self.refusal_logged
        {
            log::warn!("{} :: record {overflow}", self.name);
            self.refusal_logged = true;
        }
    }
}

/// One source's own reading of `parameter` at `now_us`, as the multi-source machine takes it: its channel's latest
/// value when fresh in the quantity's shared window (plan B's `read()` freshness, health not consulted), else None.
fn own_reading<T: databoard::Sample>(parameter: Option<&Parameter<T>>, source: &str, now_us: u64) -> Option<T> {
    let parameter = parameter?;
    parameter.channel(source)?.recent(parameter.window_us(), now_us)
}

/// The stage machine's binding as `inspect sequencer` shows it: the slots' sources, and each source past its class's
/// slots; set once at the control thread's start.
static BINDING: OnceLock<(String, Vec<String>)> = OnceLock::new();
/// Veto episode edges the stage machine reported (begins and ends).
static VETO_EDGES: AtomicU32 = AtomicU32::new(0);

/// What `inspect` shows of the stage machine beyond the table (M9): its slots' sources, the ignored ones, and the veto
/// edges it reported. None before the control thread binds it.
pub fn sequencer_inspection() -> Option<(String, Vec<String>, u32)> {
    let (slots, ignored) = BINDING.get()?;
    Some((slots.clone(), ignored.clone(), VETO_EDGES.load(Ordering::Relaxed)))
}

/// The INT1 data-ready interrupt: count the edge. Integer only.
unsafe extern "C" fn interrupt_edge(_: *mut core::ffi::c_void) {
    INTERRUPT_EDGES.fetch_add(1, Ordering::Relaxed);
}

/// The second IMU's INT1 data-ready interrupt: count the edge. Integer only.
unsafe extern "C" fn second_edge(_: *mut core::ffi::c_void) {
    SECOND_EDGES.fetch_add(1, Ordering::Relaxed);
}

/// Arms the second IMU's INT1 (the BMI323's, M9) on GPIO `gpio`, counting into [`take_second_edges`]. Only once the
/// control thread runs ([`running`]): it installed the GPIO interrupt service on core 1 before it ran.
///
/// Errors: the pin would not configure, or its handler would not attach.
pub fn arm_second_interrupt(gpio: u8) -> Result<(), sys::EspError> {
    arm_interrupt(gpio, second_edge)
}

/// The second IMU's INT1 edges since the last take.
pub fn take_second_edges() -> u32 {
    SECOND_EDGES.swap(0, Ordering::Relaxed)
}

/// Whether the control thread runs: its tick, and the GPIO interrupt service it installed first.
pub fn running() -> bool {
    RUNNING.load(Ordering::Acquire)
}

/// Arms a rising-edge interrupt on GPIO `gpio` that counts with `handler`, through the service
/// [`install_gpio_service`] installed. The internal pull-down holds a broken wire at 0,
/// where plan B's `Pin(gpio, Pin.IN)` left it floating.
fn arm_interrupt(gpio: u8, handler: unsafe extern "C" fn(*mut core::ffi::c_void)) -> Result<(), sys::EspError> {
    let config = sys::gpio_config_t {
        pin_bit_mask: 1u64 << gpio,
        mode: sys::gpio_mode_t_GPIO_MODE_INPUT,
        pull_up_en: sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
        pull_down_en: sys::gpio_pulldown_t_GPIO_PULLDOWN_ENABLE,
        intr_type: sys::gpio_int_type_t_GPIO_INTR_POSEDGE,
        ..Default::default()
    };
    // SAFETY: plain GPIO configuration of the pin the board config gives INT1; the handler is a static function.
    unsafe {
        sys::esp!(sys::gpio_config(&config))?;
        sys::esp!(sys::gpio_isr_handler_add(
            i32::from(gpio),
            Some(handler),
            core::ptr::null_mut()
        ))?;
    }
    Ok(())
}

/// Installs the GPIO interrupt service, once, before any GPIO interrupt is armed: on this thread, so every GPIO
/// interrupt it dispatches runs on core 1.
fn install_gpio_service() -> Result<(), sys::EspError> {
    // SAFETY: no GPIO interrupt is armed yet; ESP_INTR_FLAG 0, a level-1 interrupt allocated on this core.
    match unsafe { sys::gpio_install_isr_service(0) } {
        // installed already, by a component before this thread: its interrupts land where it installed them
        sys::ESP_OK | sys::ESP_ERR_INVALID_STATE => Ok(()),
        code => sys::esp!(code),
    }
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
/// tick too, with the SPI parts that came up (`spi`: the primary IMU), the separation switch, the stage machine, the
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
    topology: &'static Topology,
    table: &'static Table<'static>,
    writer: FlightWriter<'static>,
) -> anyhow::Result<()> {
    let driven = [
        spi.first().map(|member| member.name),
        separation.as_ref().map(|member| member.name),
        sequencer.as_ref().map(|member| member.name),
    ];
    let spawned = ThreadSpawnConfiguration {
        name: Some(c"control"),
        stack_size: STACK_SIZE,
        priority: PRIORITY,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set()
    .map_err(anyhow::Error::from)
    .and_then(|()| {
        std::thread::Builder::new()
            .stack_size(STACK_SIZE)
            .spawn(move || {
                run(
                    schedule_hz,
                    spi_bus,
                    spi,
                    separation,
                    sequencer,
                    topology,
                    table,
                    writer,
                )
            })
            .map_err(anyhow::Error::from)
    });
    // the default back for the threads spawned after this one
    let reset = ThreadSpawnConfiguration::default().set();
    if spawned.is_err() {
        crate::devices::crash(table, driven, NOT_STARTED);
    }
    spawned?;
    reset?;
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
    let mut machine = sequencer.map(|member| {
        if u64::from(member.settings.period_ms) * 1000 < period_us as u64 {
            log::warn!(
                "control :: {} wants a step every {} ms, the tick comes every {} ms: a step every tick",
                member.name,
                member.settings.period_ms,
                period_us / 1000
            );
        }
        Sequencer::new(member, topology)
    });
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
        // the switch's name is in SWITCH once Switch::new ran
        let driven = [
            imu.as_ref().map(|imu| imu.name),
            SWITCH.get().map(|(name, _)| *name),
            machine.as_ref().map(|machine| machine.name),
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
    RUNNING.store(true, Ordering::Release);
    loop {
        // SAFETY: this task's own notification value, index 0, cleared on take.
        let pending = unsafe { sys::ulTaskGenericNotifyTake(0, 1, u32::MAX) };
        // SAFETY: a plain read of the monotonic timer.
        let now_us = unsafe { sys::esp_timer_get_time() };
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
        if let Some(imu) = &mut imu {
            imu.tick(now_us as u64, period_us as u64 / 2);
        }
        if let Some(switch) = &mut switch {
            // the switch reads its own clock after the level: the IMU's SPI work runs between the wake and here
            switch.tick(&mut writer);
        }
        // the operator's request first: lock-free, and its answer is the executor's to log
        if let Some(Ok(commit)) = EXECUTOR_SLOT.serve(&mut writer) {
            follow_flight(&commit);
        }
        serve(&requests, imu.as_mut(), spi_bus.as_ref(), writer.flight().stage);
        // the consumers' view of the IMU channels, where the stage machine reads them
        // SAFETY: a plain read of the monotonic timer.
        let read_us = unsafe { sys::esp_timer_get_time() } as u64;
        for watch in &watches {
            watch.look(read_us);
        }
        if let Some(machine) = &mut machine {
            machine.tick(now_us as u64, period_us as u64 / 2, &mut writer);
        }
        // SAFETY: a plain read of the monotonic timer.
        let work = (unsafe { sys::esp_timer_get_time() } - now_us).clamp(0, i64::from(u32::MAX)) as u32;
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

/// Serves at most [`REQUESTS_PER_TICK`] requests waiting in the mailbox, each refused unless `stage` is SETTING: the
/// executor checked its gate once, and a probe sweep then takes seconds, during which a launch may be detected. A
/// refusal is the stage as a value, so off the pad this thread makes no text (§3.1: text only off the flight path).
fn serve(requests: &Receiver<Request>, mut imu: Option<&mut Imu>, bus: Option<&SpiBus>, stage: Stage) {
    for _ in 0..REQUESTS_PER_TICK {
        let request = match requests.try_recv() {
            Ok(request) => request,
            Err(TryRecvError::Empty | TryRecvError::Disconnected) => return,
        };
        let refusal = stage.change_refused(false);
        match request {
            Request::ProbeImu(reply) => {
                let answer = match refusal {
                    Some(refusal) => Err(refusal),
                    None => Ok(imu.as_deref_mut().and_then(Imu::probe)),
                };
                let _ = reply.send(answer);
            }
            Request::BustuneSpi { clock_hz, reply } => {
                let answer = match (refusal, imu.as_deref_mut(), bus) {
                    (Some(refusal), ..) => Err(refusal),
                    (None, Some(imu), Some(bus)) => Ok(imu.retune(bus, clock_hz)),
                    (None, ..) => Ok((Some("no SPI IMU".to_string()), 0)),
                };
                let _ = reply.send(answer);
            }
            Request::Mute { duration_ms, reply } => {
                let answer = match (refusal, imu.as_deref_mut()) {
                    (Some(refusal), _) => Err(refusal),
                    (None, Some(imu)) => {
                        // SAFETY: a plain read of the monotonic timer.
                        imu.mute(unsafe { sys::esp_timer_get_time() } as u64, duration_ms);
                        Ok(true)
                    }
                    (None, None) => Ok(false),
                };
                let _ = reply.send(answer);
            }
        }
    }
}

/// Applies an operator `request` (the executor's `stage`, `arm`, `disarm`) to the flight word through its one writer, this
/// thread: posted to the executor's slot, served at the next tick, withdrawn when not taken in [`REQUEST_DEADLINE`].
/// While the thread is not running nothing commits, so a request that would change nothing (a disarm of a disarmed
/// board) is answered with the flight as it stands, at once, and any other is not applied. The executor's alone.
///
/// Returns: the answer: the flight committed, or why an arm was refused.
///
/// Errors: why there is no answer, saying whether the request was applied.
pub fn request(request: sequencer::Request) -> Result<sequencer::Answer, String> {
    if !RUNNING.load(Ordering::Acquire) {
        let flight = FLIGHT.load();
        return match flight.apply(request) {
            Ok(after) if after == flight => Ok(Ok(flight)),
            _ => Err("the control thread is not running: not applied".to_string()),
        };
    }
    EXECUTOR_SLOT
        .request(request, REQUEST_DEADLINE, REQUEST_POLL)
        .map_err(|unanswered| unanswered.to_string())
}

/// A `bustune spi` rung's outcome: the IMU's name, its verdict at the asked clock (None healthy, else why), and the
/// clock its handle runs at afterwards (the configured one when the driver refused the asked one, 0 when neither came
/// back).
pub struct SpiRung {
    pub name: &'static str,
    pub verdict: Option<String>,
    pub running_hz: u32,
}

/// A `bustune spi` rung on the control thread (SPI2's owner); None when the thread drives no IMU. A clock above
/// [`MAXIMUM_SPI_HZ`] is the IMU failing the rung, answered here and never sent: the handle is not touched, and the
/// CC's ladder (5, 8, 10, 16, 20 MHz) still finds the part's ceiling.
///
/// Errors: the control thread is busy, did not answer in time, or refused the rung outside SETTING.
pub fn bustune_spi(clock_hz: u32) -> Option<Result<SpiRung, String>> {
    let name = *IMU_NAME.get()?;
    if clock_hz > MAXIMUM_SPI_HZ {
        return Some(Ok(SpiRung {
            name,
            verdict: Some(format!(
                "{clock_hz} Hz is above the part's {MAXIMUM_SPI_HZ} Hz SPI maximum, not tried"
            )),
            running_hz: SPI_CLOCK_HZ.load(Ordering::Relaxed),
        }));
    }
    let mailbox = MAILBOX.get()?;
    let (reply, answer) = sync_channel(1);
    if mailbox.try_send(Request::BustuneSpi { clock_hz, reply }).is_err() {
        return Some(Err("the control thread is busy".to_string()));
    }
    Some(match answer.recv_timeout(PROBE_DEADLINE) {
        Ok(rung) => rung
            .map(|(verdict, running_hz)| SpiRung {
                name,
                verdict,
                running_hz,
            })
            .map_err(|refusal| refusal.to_string()),
        Err(_) => Err("the control thread did not answer".to_string()),
    })
}

/// The IMU's probe on the control thread (CC `probe`): its name and verdict (None healthy, else why); None when the
/// thread drives no IMU.
///
/// Errors: the control thread is busy, did not answer in time, or refused the probe outside SETTING.
pub fn probe_imu() -> Option<Result<(&'static str, Option<String>), String>> {
    let name = *IMU_NAME.get()?;
    let mailbox = MAILBOX.get()?;
    let (reply, answer) = sync_channel(1);
    if mailbox.try_send(Request::ProbeImu(reply)).is_err() {
        return Some(Err("the control thread is busy".to_string()));
    }
    Some(match answer.recv_timeout(PROBE_DEADLINE) {
        Ok(verdict) => verdict
            .map(|verdict| (name, verdict))
            .map_err(|refusal| refusal.to_string()),
        Err(_) => Err("the control thread did not answer".to_string()),
    })
}

/// What `inspect` shows of the IMU the control thread drives (plan B lsm6dso32.py:300-308, beyond the channels and
/// the table's `healthy`): its name, whether INT1 is wired, whether INT1 is silent. None without one.
pub fn imu() -> Option<(&'static str, bool, bool)> {
    Some((
        *IMU_NAME.get()?,
        INTERRUPT_WIRED.load(Ordering::Relaxed),
        INTERRUPT_SILENT.load(Ordering::Relaxed),
    ))
}

/// The separation switch's debounced latch (plan B `separated()`): true = separated.
pub fn separated() -> bool {
    SEPARATED.load(Ordering::Relaxed)
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
    gcl::register(vec![
        mute_command(),
        Command {
            owner: "control",
            words: "control",
            about: "the control tick: its rate, ticks, overruns (ticks the thread missed), lateness (the latest, the \
                largest, and how many ticks passed the 200 us budget) and work (the latest and the largest, from the \
                wake to the end of the tick's work); whether the flight word holds the stage machine's launch; the \
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

/// `source`'s watched channels' lapses, by quantity (`inspect` of an IMU); empty before the thread starts.
pub fn lapses_of(source: &str) -> Vec<(String, u32)> {
    WATCH_NAMES
        .get()
        .map(|names| {
            names
                .iter()
                .zip(&LAPSES)
                .filter_map(|(name, count)| {
                    let (owner, quantity) = name.split_once('.')?;
                    (owner == source).then(|| (quantity.to_string(), count.load(Ordering::Relaxed)))
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Each watched channel's lapses as `source.quantity:count`, comma-separated; empty before the thread starts.
fn lapses() -> String {
    WATCH_NAMES
        .get()
        .map(|names| {
            names
                .iter()
                .zip(&LAPSES)
                .map(|(name, count)| format!("{name}:{}", count.load(Ordering::Relaxed)))
                .collect::<Vec<_>>()
                .join(",")
        })
        .unwrap_or_default()
}

/// GCL `imu mute ms=<1..10000>`: the control thread skips the SPI IMU's reads for that long, so the part goes stale,
/// then unhealthy, and the databoard hands `accel` and `rate` to the next fresh source; the reads resume by themselves
/// at expiry, judged by their flags alone. The failover's forced proof (M9): `bustune spi` cannot silence the part.
/// Rust-only, like every bus diagnostic only in SETTING while disarmed.
fn mute_command() -> Command {
    Command {
        owner: "control",
        words: "imu mute",
        about: "skip the SPI IMU's reads for ms: it goes stale, then unhealthy, and accel and rate fail over to the \
                next fresh source; the reads resume by themselves",
        gate: Gate::Disarmed,
        reach: Reach::Everywhere,
        parameters: &[gcl::Parameter {
            name: "ms",
            kind: gcl::Kind::Integer { low: 1, high: 10_000 },
            default: None,
            about: "how long, milliseconds",
        }],
        bare_words: &[],
        run: Box::new(|arguments| {
            let duration_ms = arguments.integer("ms") as u32;
            let Some(mailbox) = MAILBOX.get() else {
                return cc_protocol::build("err", &["imu", "the control thread is not running"], &[]);
            };
            let (reply, answer) = sync_channel(1);
            if mailbox.try_send(Request::Mute { duration_ms, reply }).is_err() {
                return cc_protocol::build("err", &["imu", "the control thread is busy"], &[]);
            }
            match answer.recv_timeout(PROBE_DEADLINE) {
                Ok(Ok(true)) => cc_protocol::build("ok", &["imu", "mute"], &[("ms", &duration_ms.to_string())]),
                Ok(Ok(false)) => cc_protocol::build("err", &["imu", "no SPI IMU on the control thread"], &[]),
                Ok(Err(refusal)) => cc_protocol::build("err", &["unsafe", &refusal.to_string()], &[]),
                Err(_) => cc_protocol::build("err", &["imu", "the control thread did not answer"], &[]),
            }
        }),
    }
}
