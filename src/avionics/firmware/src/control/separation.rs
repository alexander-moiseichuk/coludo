//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The separation switch (M6, M7): its edges, its debounced episodes and the thrust gate that counts one.

use super::*;

/// The separation pin's open episode, its first and latest edge (esp_timer's us truncated to u32): the GPIO interrupt
/// records into it, the tick takes it once quiet (drivers::debounce).
pub(super) static SEPARATION: StampPair = StampPair::new();
/// Events the separation switch recorded since boot, separation.csv's rows (`control`): an episode concluded, or
/// under the thrust gate a pending separation counted or cleared.
pub(super) static SEPARATION_EVENTS: AtomicU32 = AtomicU32::new(0);
/// Of those, the blips: the contact moved and came back (`control`).
pub(super) static SEPARATION_BLIPS: AtomicU32 = AtomicU32::new(0);
/// A separation concluded in BOOSTING waits for the thrust gate or the backstop (`control`'s `pending`).
pub(super) static SEPARATION_PENDING: AtomicBool = AtomicBool::new(false);
/// What the thrust gate has seen since BOOSTING entry, as its index in [`THRUSTS`] (`control`'s `thrust`).
pub(super) static THRUST: AtomicU8 = AtomicU8::new(0);
/// Arming the separation pin's edge interrupt failed: the tick polls it (`control`).
pub(super) static SEPARATION_POLLED: AtomicBool = AtomicBool::new(false);
/// The separation switch's debounced latch, for the console and (M8) the warm-start gate: true = separated.
pub(super) static SEPARATED: AtomicBool = AtomicBool::new(false);

/// The separation switch's entry name and GPIO, once the control thread drives it (its probe reads the pin).
pub(super) static SWITCH: OnceLock<(&'static str, u8)> = OnceLock::new();

/// The thrust gate's verdicts, in the order [`THRUST`] indexes them.
pub(super) const THRUSTS: [Thrust; 3] = [Thrust::Unseen, Thrust::Seen, Thrust::Ended];

/// The body frame's +x, the nose, in an `accel` vector (x, y, z): the axis the thrust gate reads (the body frame is
/// +x nose, +y left, +z up; each airframe's mount is checked nose up, where +x reads about +1 g).
pub(super) const AXIAL: usize = 0;

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
pub(super) struct Switch {
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
    pub(super) fn new(
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
            telemetry: crate::recorder::telemetry(member.name),
        }
    }

    /// One tick: the pin's level, then the clock, offered to the stamp pair (observed, or polled when the arm failed);
    /// the thrust gate stepped with the glider's a_x from a fresh `accel`; then the switch's tick under the gate
    /// (drivers::separation `Separation::tick`): a quiet episode concluded by that same level, or a pending separation
    /// counted or cleared. A separation that advances BOOSTING commits GLIDING through `writer` and logs the
    /// controller's console line, then the row is written with the moment and the wobble, and the event is logged
    /// (plan B's order, drivers/separation.py `_apply`).
    pub(super) fn tick(&mut self, writer: &mut FlightWriter<'static>) {
        // the level first, then the clock: the level decides the episode this tick concludes
        // SAFETY: a plain read of the input bring-up configured.
        let high = unsafe { sys::gpio_get_level(i32::from(self.gpio)) } != 0;
        let now_us = uptime_us();
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
pub(super) unsafe extern "C" fn separation_edge(_: *mut core::ffi::c_void) {
    SEPARATION.record(uptime_us() as u32);
}

/// Arms a both-edge interrupt on the separation pin (already an input with its pull-down) that records into
/// [`SEPARATION`], through the service [`install_gpio_service`] installed.
pub(super) fn arm_edges(gpio: u8) -> Result<(), sys::EspError> {
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

/// The separation switch's debounced latch (plan B `separated()`): true = separated.
pub fn separated() -> bool {
    SEPARATED.load(Ordering::Relaxed)
}
