//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The primary IMU on SPI2 (M6): its read on each tick, its INT1 edges, its probe, its clock rung and its mute.

use super::*;

/// INT1 edges since the tick last took them: the GPIO interrupt counts, the tick swaps the count out.
pub(super) static INTERRUPT_EDGES: AtomicU32 = AtomicU32::new(0);
/// IMU reads `imu mute` skipped (M9).
pub(super) static IMU_MUTED: AtomicU32 = AtomicU32::new(0);
/// IMU samples read.
pub(super) static IMU_READS: AtomicU32 = AtomicU32::new(0);
/// IMU reads that failed.
pub(super) static IMU_FAILURES: AtomicU32 = AtomicU32::new(0);
/// IMU reads that found no new conversion: about 1 - ODR/100 of the ticks on a part slower than the tick.
pub(super) static IMU_STALE: AtomicU32 = AtomicU32::new(0);
/// IMU halves pushed by the driver's correction (a), by half (accel, rate): the data-ready flag clear but the bytes
/// changed, a conversion that landed between the status byte and its data (M9).
pub(super) static IMU_RACED: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
/// IMU halves whose data-ready flag was set over bytes identical to the last read's, by half (accel, rate): a raced
/// conversion's duplicate (skipped, correction (b)) or a bit-identical new conversion (pushed). What D1's residual is
/// measured by (M9).
pub(super) static IMU_IDENTICAL: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
/// The latest tick's `irq_runs`.
pub(super) static IRQ_RUNS_LAST: AtomicU32 = AtomicU32::new(0);
/// Whether INT1 is silent (the driver's verdict: three wakes in a row with no edge).
pub(super) static INTERRUPT_SILENT: AtomicBool = AtomicBool::new(false);
/// The IMU's entry name, once the control thread drives one.
pub(super) static IMU_NAME: OnceLock<&'static str> = OnceLock::new();
/// Whether the IMU's INT1 is wired (its `int_pin` names a GPIO).
pub(super) static INTERRUPT_WIRED: AtomicBool = AtomicBool::new(false);
/// The fastest SPI clock a rung may ask for: the LSM6DSO32's SPI maximum, 10 MHz.
pub(super) const MAXIMUM_SPI_HZ: u32 = 10_000_000;

/// The clock the IMU's SPI handle runs at, Hz (0 without one).
pub(super) static SPI_CLOCK_HZ: AtomicU32 = AtomicU32::new(0);

/// The primary IMU as the control thread drives it: the part, its bus handle, its databoard writers and its stream.
pub(super) struct Imu {
    /// The entry's name: the databoard source and the stream's.
    pub(super) name: &'static str,
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
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// A read failed and has not succeeded since: one log line per run of failures (plan B's `note`).
    failing: bool,
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
    pub(super) fn new(member: SpiMember, topology: &'static Topology, table: &'static Table<'static>) -> Imu {
        let writer = |quantity: &str| {
            topology
                .vector(quantity)
                .and_then(|parameter| parameter.writer(member.name))
        };
        Imu {
            name: member.name,
            accel: writer("accel"),
            rate: writer("rate"),
            rows: crate::recorder::rows(member.name),
            chip_select: member.handle.chip_select(),
            handle: Some(member.handle),
            part: member.part,
            entry: table.find(member.name),
            stale_run: 0,
            failing: false,
            period_us: u64::from(member.period_ms) * 1000,
            next_due_us: 0,
            axes: member.axes,
            muted_until_us: 0,
        }
    }

    /// `imu mute`: skips the reads for `duration_ms` from `now_us`, the part unhealthy from the first skipped read, so
    /// the databoard hands its channels to the next fresh source; the reads resume by themselves at expiry, at `arm`,
    /// or when the stage leaves SETTING.
    pub(super) fn mute(&mut self, now_us: u64, duration_ms: u32) {
        self.muted_until_us = now_us + u64::from(duration_ms) * 1000;
        log::warn!("{} :: muted for {duration_ms} ms (imu mute)", self.name);
    }

    /// Plan B's probe (lsm6dso32.py:248-278): WHO_AM_I and one sample, logged in plan B's shape. None when healthy,
    /// else why.
    pub(super) fn probe(&mut self) -> Option<String> {
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
    pub(super) fn retune(&mut self, bus: &SpiBus, clock_hz: u32) -> (Option<String>, u32) {
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
    pub(super) fn set_healthy(&self, healthy: bool) -> bool {
        self.entry.is_some_and(|entry| entry.set_healthy(healthy))
    }

    /// One tick: when its period is due (within half a tick, `slack_us`), take the INT1 edges since the last read,
    /// read the status burst, and publish what is new (see the module notes). A tick faster than the period skips
    /// reads; edges accumulate.
    pub(super) fn tick(&mut self, now_us: u64, slack_us: u64) {
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
        if !self.rows.recording() {
            return;
        }
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
        self.rows.push(now_us, &row);
    }
}

/// The INT1 data-ready interrupt: count the edge. Integer only.
pub(super) unsafe extern "C" fn interrupt_edge(_: *mut core::ffi::c_void) {
    INTERRUPT_EDGES.fetch_add(1, Ordering::Relaxed);
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

/// GCL `imu mute ms=<1..10000>`: the control thread skips the SPI IMU's reads for that long, so the part goes stale,
/// then unhealthy, and the databoard hands `accel` and `rate` to the next fresh source; the reads resume by themselves
/// at expiry, judged by their flags alone. The failover's forced proof (M9): `bustune spi` cannot silence the part.
/// Rust-only, like every bus diagnostic only in SETTING while disarmed.
pub(super) fn mute_command() -> Command {
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
                return cc_protocol::build("err", &["internal", "the control thread is not running"], &[]);
            };
            let (reply, answer) = sync_channel(1);
            if mailbox.try_send(Request::Mute { duration_ms, reply }).is_err() {
                return cc_protocol::build("err", &["internal", "the control thread is busy"], &[]);
            }
            match answer.recv_timeout(PROBE_DEADLINE) {
                Ok(Ok(true)) => cc_protocol::build("ok", &["imu", "mute"], &[("ms", &duration_ms.to_string())]),
                Ok(Ok(false)) => cc_protocol::build("err", &["internal", "no SPI IMU on the control thread"], &[]),
                Ok(Err(refusal)) => cc_protocol::build("err", &["unsafe", &refusal.to_string()], &[]),
                Err(_) => cc_protocol::build("err", &["internal", "the control thread did not answer"], &[]),
            }
        }),
    }
}
