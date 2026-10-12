//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board's side of the control tick's duties (crates/controller `ControlCore`, R3): the hooks, and the state the
//! executor reads of the filter and the stage machine.

use super::*;

/// The attitude filter's latest row ([`Attitude::row`]: heading, roll, pitch in centidegrees, free, mag_known, the
/// magnetic offset), published after each step for the executor (M9): integers, so the flight core makes no text.
pub(super) static ATTITUDE_ROW: [AtomicI32; 6] = [const { AtomicI32::new(0) }; 6];
/// Whether a source that outranks the filter has ever been mirrored (plan B's `_seeded`).
pub(super) static ATTITUDE_SEEDED: AtomicBool = AtomicBool::new(false);
/// Whether the filter is blind (no fresh gyro for its blind window).
pub(super) static ATTITUDE_BLIND: AtomicBool = AtomicBool::new(false);
/// The filter's entry name and the `rate` quantity it reads, once it runs (its probe's gyro check).
pub(super) static ATTITUDE_NAME: OnceLock<(&'static str, Option<&'static Parameter<[f32; 3]>>)> = OnceLock::new();

/// The board's side of the control tick (crates/controller `ControlCore`, R3): the uptime read again after the IMU's
/// push, the mission's view and the GNSS drift, the recorder's flight flag, the console's stage line, the state the
/// executor reads, and the fins through their bound servos.
pub(super) struct Board<'a> {
    /// The fin servos, when they came up.
    pub(super) fins: Option<&'a mut crate::fins::Fins>,
    /// Each mixer surface's fin index (crate::flight::bind), None for a surface with no servo.
    pub(super) bound: &'a [Option<usize>],
}

impl controller::Hooks for Board<'_> {
    fn now_us(&mut self) -> u64 {
        uptime_us()
    }

    fn mission(
        &mut self,
    ) -> (
        Option<control::guidance::Landing>,
        Option<control::navigation::Position>,
    ) {
        crate::flight::mission_view()
    }

    fn drift(&mut self) -> [f32; 2] {
        crate::gnss::drift()
    }

    fn flight_flag(&mut self, raised: bool) {
        set_flight(raised);
    }

    fn moved(&mut self, stage: Stage) {
        log::info!(target: CONSOLE_ONLY, "controller :: stage -> {}", stage.name());
    }

    fn veto_edge(&mut self) {
        VETO_EDGES.fetch_add(1, Ordering::Relaxed);
    }

    fn attitude(&mut self, row: [i32; 6], seeded: bool, blind: bool) {
        for (published, value) in ATTITUDE_ROW.iter().zip(row) {
            published.store(value, Ordering::Relaxed);
        }
        ATTITUDE_SEEDED.store(seeded, Ordering::Relaxed);
        ATTITUDE_BLIND.store(blind, Ordering::Relaxed);
    }

    fn fins(&mut self, angles: &[i32]) {
        let Some(fins) = self.fins.as_deref_mut() else {
            return;
        };
        for (&angle, index) in angles.iter().zip(self.bound) {
            if let Some(index) = *index {
                fins.set_angle(index, f64::from(angle));
            }
        }
    }

    fn vitals(&mut self, law: &control::flight::FlightLaw) {
        crate::flight::publish_vitals(law);
    }
}

/// The control tick's duties (crates/controller `ControlCore`) for the entries that came up, on a tick of `tick_us`:
/// the stage machine (a step period shorter than the tick logged), the attitude filter and the flight loop, each
/// recording into its stream; the binding `inspect sequencer` shows, and the filter's name and gyro for its probe,
/// kept for the executor.
pub(super) fn control_core(
    sequencer: Option<SequencerMember>,
    attitude: Option<AttitudeMember>,
    flight: Option<crate::devices::FlightMember>,
    topology: &'static Topology,
    tick_us: u64,
) -> ControlCore {
    if let Some(member) = &sequencer
        && u64::from(member.settings.period_ms) * 1000 < tick_us
    {
        log::warn!(
            "control :: {} wants a step every {} ms, the tick comes every {} ms: a step every tick",
            member.name,
            member.settings.period_ms,
            tick_us / 1000
        );
    }
    let core = ControlCore::new(
        topology,
        tick_us,
        uptime_us(),
        attitude.map(|member| FilterSetup {
            name: member.name,
            settings: member.settings,
            rows: crate::recorder::rows(member.name),
        }),
        sequencer.map(|member| SequencerSetup {
            name: member.name,
            settings: member.settings,
            rows: Rows::new(member.name, "record", crate::recorder::telemetry(member.name)).logged_once(),
            vetoes: Rows::new(
                member.name,
                "vetoes",
                crate::recorder::telemetry(crate::recorder::VETOES),
            )
            .logged_once(),
        }),
        flight.map(|member| FlightSetup {
            entry: member.entry,
            fins: member.fins,
            rows: crate::recorder::rows(member.name),
            seed_airspeed: crate::flight::warm_airspeed(),
        }),
    );
    if let Some(binding) = core.binding() {
        let _ = BINDING.set(binding);
    }
    if let Some(name) = core.filter() {
        let _ = ATTITUDE_NAME.set((name, topology.vector("rate")));
    }
    core
}

/// What `inspect` shows of the attitude filter (plan B tasks/attitude.py `inspect`): its flags, angles and magnetic
/// reference as the filter last published them; None when no filter runs.
pub fn attitude_inspection() -> Option<serde_json::Value> {
    ATTITUDE_NAME.get()?;
    Some(control::attitude::inspection(
        ATTITUDE_ROW.each_ref().map(|value| value.load(Ordering::Relaxed)),
        ATTITUDE_SEEDED.load(Ordering::Relaxed),
        ATTITUDE_BLIND.load(Ordering::Relaxed),
    ))
}

/// The attitude filter's probe (plan B tasks/attitude.py `probe`), on the executor: its name and verdict, the gyro
/// `rate` fresh now (None healthy, else plan B's text), logged in plan B's shape; None when no filter runs or
/// `only` names another object.
pub fn probe_attitude(only: Option<&str>) -> Option<(&'static str, Option<String>)> {
    let (name, rate) = *ATTITUDE_NAME.get()?;
    if only.is_some_and(|only| only != name) {
        return None;
    }
    log::info!("{name} :: probe: gyro rate ...");
    let now_us = uptime_us();
    let fresh = rate.is_some_and(|rate| rate.read(now_us).is_some());
    let verdict = Attitude::probe(fresh).err().map(str::to_string);
    match &verdict {
        None => log::info!("{name} :: probe: gyro rate ok"),
        Some(why) => log::warn!("{name} :: probe FAILED: {why}"),
    }
    Some((name, verdict))
}

/// Whether the attitude filter is blind (`attitude-blind`, plan B cc_client): no fresh gyro for its blind window.
pub fn attitude_blind() -> bool {
    ATTITUDE_BLIND.load(Ordering::Relaxed)
}

/// The stage machine's binding as `inspect sequencer` shows it: the slots' sources, and each source past its class's
/// slots; set once at the control thread's start.
pub(super) static BINDING: OnceLock<(String, Vec<String>)> = OnceLock::new();
/// Veto episode edges the stage machine reported (begins and ends).
pub(super) static VETO_EDGES: AtomicU32 = AtomicU32::new(0);

/// What `inspect` shows of the stage machine beyond the table (M9): its slots' sources, the ignored ones, and the veto
/// edges it reported. None before the control thread binds it.
pub fn sequencer_inspection() -> Option<(String, Vec<String>, u32)> {
    let (slots, ignored) = BINDING.get()?;
    Some((slots.clone(), ignored.clone(), VETO_EDGES.load(Ordering::Relaxed)))
}
