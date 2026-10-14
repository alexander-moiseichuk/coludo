//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! [`ControlCore`]: the control tick's pure duties (doc/specs/avionics-refactor.md R3), one code path for the board's
//! control thread and the SITL, so the SITL flies the board's code and not a copy of it (plan B's sim-mirrors trap):
//!
//! 1. the attitude filter (plan B tasks/attitude.py, M9): paced on its `period_ms`, reading the fused `rate`, `accel`
//!    and `mag` through the marks map, its attitude pushed and recorded on every publish; stopped at DONE;
//! 2. the stage machine (plan B's Sequencer task, M7, M9): stepped every `period_ms` within half a tick of the wake,
//!    on each bound source's channel; a move made from outside writes its row, a veto episode's edges theirs, and its
//!    own advance commits, then the console's line, then the recorded line and the row, the recorder's flight flag
//!    raised before those rows and lowered after them (plan B `_tick`, `_advance`);
//! 3. the flight law (plan B tasks/flight.py, M13): every tick, on the fresh quantities and the mission, its fins
//!    handed on when it actuates, and every 100 ms (a second at DONE) the flight.csv row and the vitals.
//!
//! What a board has and the SITL has not comes in through [`Hooks`]: the clock read after the IMU's push (the
//! databoard refuses a stamp from the future), the mission and the GNSS drift, the recorder's flight flag, the
//! console, the state published for the executor, and the fins. The board's control thread keeps the hardware: the
//! gptimer, the SPI IMU, the separation switch, the GPIO interrupts and the lapse watches. Nothing here allocates or
//! formats text on a tick but the machine's own transition (six or fewer a flight).

use control::attitude::{Attitude, Pace, Quantities, assemble};
use control::flight::FlightLaw;
use control::guidance::Landing;
use control::navigation::Position;
use databoard::{Parameter, Writer};
use recorder::{Cell, Rows};
use sequencer::{Decider, FlightFlag, FlightWriter, Stage, StageMachine, Transition, VetoEdge};
use serde_json::Value;

use crate::inputs::{FlightQuantities, StageSources};
use crate::topology::Topology;

/// The flight.csv sample period, µs (plan B `telemetry_ms` 100: the control state SAMPLED, not traced).
const SAMPLE_US: u64 = 100_000;
/// At DONE, µs (M11b).
const DONE_SAMPLE_US: u64 = 1_000_000;

/// What the tick needs of its host beyond the databoard: each call is cheap, never blocks and never allocates.
pub trait Hooks {
    /// The clock now, µs: the board's uptime read again after the IMU's push, the SITL's tick time.
    fn now_us(&mut self) -> u64;
    /// The mission's landing zone and its set launch point, as the law reads them now.
    fn mission(&mut self) -> (Option<Landing>, Option<Position>);
    /// The pad drift's estimate (east, north, m/s), the wind's de-bias (M11).
    fn drift(&mut self) -> [f32; 2];
    /// The recorder's flight flag: raised before a transition's rows, lowered after them.
    fn flight_flag(&mut self, raised: bool);
    /// The stage machine moved itself to `stage`: the console's line, before the recorded one.
    fn moved(&mut self, stage: Stage);
    /// A veto episode's edge, to count.
    fn veto_edge(&mut self);
    /// The attitude filter's state after a step: its row, whether it mirrored a source that outranks it, whether it
    /// is blind.
    fn attitude(&mut self, row: [i32; 6], seeded: bool, blind: bool);
    /// The law actuated: its whole-degree fin angles, by the mixer's surfaces.
    fn fins(&mut self, angles: &[i32]);
    /// A flight.csv sample was taken: the law's vitals.
    fn vitals(&mut self, law: &FlightLaw);
}

/// The stage machine's setup.
pub struct SequencerSetup {
    /// The entry's name: its recorded line's and its stream's.
    pub name: &'static str,
    /// Its settings (`sequencer::Settings::from_config`).
    pub settings: sequencer::Settings,
    /// `<name>.csv`.
    pub rows: Rows<'static>,
    /// `vetoes.csv`.
    pub vetoes: Rows<'static>,
}

/// The attitude filter's setup.
pub struct FilterSetup {
    /// The entry's name: the source of `attitude`, its log lines' and its stream's.
    pub name: &'static str,
    /// Its settings (`control::attitude::Settings::from_json`).
    pub settings: control::attitude::Settings,
    /// attitude.csv.
    pub rows: Rows<'static>,
}

/// The flight loop's setup.
pub struct FlightSetup {
    /// The `flight` entry's fields.
    pub entry: Value,
    /// The config's `fins` section.
    pub fins: Value,
    /// flight.csv.
    pub rows: Rows<'static>,
    /// A warm start's airspeed, the law's seed.
    pub seed_airspeed: Option<f32>,
}

/// The stage machine as the tick steps it: the machine, the sources it reads, its streams and its pace.
struct SequencerDuty {
    /// The entry's name.
    name: &'static str,
    /// The machine, boxed (~2.4 KB the control stack does not carry).
    machine: Box<StageMachine>,
    /// Which slot each source holds, bound once from the databoard's sources in rank order (M9).
    sources: StageSources<'static>,
    /// `vetoes.csv`, the veto episodes' begin and end rows (M9).
    vetoes: Rows<'static>,
    /// `<name>.csv`.
    rows: Rows<'static>,
    /// Its step period, µs (`period_ms`).
    period_us: u64,
    /// When the next step is due, µs.
    next_due_us: u64,
}

impl SequencerDuty {
    /// The machine of `setup`, each source bound to its slot (one line per class, and one per source past its
    /// class's slots).
    fn new(setup: SequencerSetup, topology: &'static Topology) -> SequencerDuty {
        let duty = SequencerDuty {
            name: setup.name,
            machine: Box::new(StageMachine::new(setup.settings)),
            sources: StageSources::new(topology),
            vetoes: setup.vetoes,
            rows: setup.rows,
            period_us: u64::from(setup.settings.period_ms) * 1000,
            next_due_us: 0,
        };
        let (text, ignored) = duty.binding();
        log::info!("{} :: sources: {text}", duty.name);
        for line in ignored {
            log::warn!("{} :: {line}", duty.name);
        }
        duty
    }

    /// The bound sources as one line (`accel [..], barometers [..], gnss [..]`), and the sources past their
    /// class's slots.
    fn binding(&self) -> (String, Vec<String>) {
        let binding = self.sources.binding();
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
        (text, binding.ignored().iter().map(ToString::to_string).collect())
    }

    /// One tick that woke at `now_us`: when its period is due (within `slack_us`), one step of the machine on the
    /// flight as `writer` last committed it, with the quantities read fresh now (plan B `_tick`).
    fn tick(&mut self, now_us: u64, slack_us: u64, writer: &mut FlightWriter<'_>, hooks: &mut dyn Hooks) {
        if now_us + slack_us < self.next_due_us {
            return;
        }
        // the next step a period on, or a period from now when the steps fell behind
        self.next_due_us = (self.next_due_us + self.period_us).max((now_us + self.period_us).saturating_sub(slack_us));
        let read_us = hooks.now_us();
        // each bound source's own channel, fresh in its quantity's shared window (M9: the multi-source machine)
        let readings = self.sources.readings(read_us);
        let flight = writer.flight();
        let step = self
            .machine
            .step((read_us / 1000) as u32, flight.stage, readings, flight.manual);
        if let Some(external) = step.external {
            // a constant text: the tick formats nothing for a move it did not make
            self.record(read_us, external.to, "external");
        }
        for edge in step.vetoes() {
            self.veto(read_us, edge, hooks);
        }
        if let Some(transition) = step.advance {
            self.advance(read_us, transition, writer, hooks);
        }
    }

    /// One `vetoes.csv` row at `now_us` for a veto episode's `edge`: the detector, the edge, the deciding source, the
    /// vetoes that held it, the vote and the decider's value, every cell a constant or a number.
    fn veto(&mut self, now_us: u64, edge: VetoEdge, hooks: &mut dyn Hooks) {
        hooks.veto_edge();
        if !self.vetoes.recording() {
            return;
        }
        let veto = edge.veto();
        let row = [
            Cell::Text(veto.detector.name()),
            Cell::Text(edge.name()),
            Cell::Text(self.sources.binding().source(veto.decider).unwrap_or("")),
            Cell::Text(veto.kinds()),
            Cell::I32(i32::from(veto.contrary)),
            Cell::I32(i32::from(veto.live)),
            Cell::F32(veto.value),
        ];
        self.vetoes.push(now_us, &row);
    }

    /// The machine's own `transition` at `now_us`: the commit, the console's line, the recorded line and the row, in
    /// plan B's order, the flight flag around them.
    fn advance(&mut self, now_us: u64, transition: Transition, writer: &mut FlightWriter<'_>, hooks: &mut dyn Hooks) {
        let flag = writer.advance(transition).flight_flag();
        if flag == Some(FlightFlag::Raise) {
            hooks.flight_flag(true);
        }
        hooks.moved(transition.to);
        // the reason's text: a machine transition is the one exception to "no text on the tick"
        let reason = transition.reason.to_string();
        log::info!("{} :: stage -> {} ({reason})", self.name, transition.to.name());
        self.record(now_us, transition.to, &reason);
        if flag == Some(FlightFlag::Lower) {
            hooks.flight_flag(false);
        }
    }

    /// One `<name>.csv` row at `now_us`: `stage` and `reason`. A refused row never stops the machine: the stage has
    /// moved either way (plan B `_record`).
    fn record(&mut self, now_us: u64, stage: Stage, reason: &str) {
        self.rows.push(now_us, &[Cell::Text(stage.name()), Cell::Text(reason)]);
    }
}

/// The attitude filter as the tick steps it.
struct FilterDuty {
    /// The entry's name.
    name: &'static str,
    /// The filter, boxed.
    attitude: Box<Attitude>,
    /// The quantities it reads and publishes into.
    quantities: Quantities<'static>,
    /// Which ticks step it.
    pace: Pace,
    /// The rank it publishes at (its `provides.attitude.priority`).
    rank: i32,
    /// The `attitude` writer.
    writer: Option<Writer<'static, [f32; 3]>>,
    /// attitude.csv.
    rows: Rows<'static>,
}

impl FilterDuty {
    /// The filter of `setup` on a tick of `tick_us`, started at `now_us`; None (logged) when no `attitude` quantity
    /// came up to publish into.
    fn new(setup: FilterSetup, topology: &'static Topology, tick_us: u64, now_us: u64) -> Option<FilterDuty> {
        let Some(attitude) = topology.vector("attitude") else {
            log::warn!(
                "{} :: no attitude quantity on the databoard: the filter does not run",
                setup.name
            );
            return None;
        };
        log::info!(
            "{} :: every {} ms at rank {}, reading rate, accel, mag, course and speed",
            setup.name,
            setup.settings.period_ms,
            setup.settings.rank
        );
        Some(FilterDuty {
            name: setup.name,
            attitude: Box::new(Attitude::new(setup.settings, now_us)),
            quantities: Quantities {
                rate: topology.vector("rate"),
                acceleration: topology.vector("accel"),
                magnetic: topology.vector("mag"),
                course: topology.scalar("course"),
                speed: topology.scalar("speed"),
                attitude,
            },
            pace: Pace::new(setup.settings.period_ms, tick_us),
            rank: setup.settings.rank,
            writer: attitude.writer(setup.name),
            rows: setup.rows,
        })
    }

    /// One tick that woke at `now_us`: when its pace is due, one filter step on the readings fresh now, its
    /// transition line logged (a constant), its state handed on, and what it publishes pushed and recorded.
    fn tick(&mut self, now_us: u64, hooks: &mut dyn Hooks) {
        if !self.pace.due(now_us) {
            return;
        }
        let read_us = hooks.now_us();
        let readings = assemble(&self.quantities, self.rank, read_us);
        let step = self.attitude.tick(read_us, &readings);
        if let Some(line) = step.log {
            log::warn!("{} :: {line}", self.name);
        }
        let row = self.attitude.row();
        hooks.attitude(row, self.attitude.seeded(), self.attitude.blind());
        let Some(orientation) = step.published else {
            return;
        };
        if let Some(writer) = &mut self.writer {
            writer.push(orientation.to_sample(), read_us);
        }
        self.rows.push(read_us, &row.map(Cell::I32));
    }
}

/// The flight loop as the tick steps it.
struct FlightDuty {
    /// The law.
    law: FlightLaw,
    /// What it reads.
    quantities: FlightQuantities<'static>,
    /// flight.csv.
    rows: Rows<'static>,
    /// The next flight.csv sample, µs.
    next_sample_us: u64,
}

impl FlightDuty {
    /// The loop of `setup` on `topology`.
    fn new(setup: FlightSetup, topology: &'static Topology) -> FlightDuty {
        // the guidance's fix freshness: the GNSS channels' own window (plan B tasks/flight.py), 500 ms with no GNSS
        let window_us = [
            topology.position("position").map(Parameter::window_us),
            topology.scalar("speed").map(Parameter::window_us),
        ]
        .into_iter()
        .flatten()
        .max();
        let mut law = FlightLaw::new(
            &setup.entry,
            &setup.fins,
            window_us.map_or(500, |window_us| window_us / 1000),
        );
        if let Some(seed) = setup.seed_airspeed.filter(|seed| *seed > 0.0) {
            law.seed_airspeed(seed);
        }
        FlightDuty {
            law,
            quantities: FlightQuantities::new(topology),
            rows: setup.rows,
            next_sample_us: 0,
        }
    }

    /// One tick: the law's step on the fresh quantities, the fins it actuated handed on, and every 100 ms the
    /// flight.csv row and the vitals.
    fn tick(&mut self, writer: &FlightWriter<'_>, hooks: &mut dyn Hooks) {
        let now_us = hooks.now_us();
        let flight = writer.flight();
        let (landing, set_launch) = hooks.mission();
        let step = self
            .quantities
            .step(now_us, flight.stage, flight.armed, landing, set_launch, hooks.drift());
        if self.law.step(&step) {
            hooks.fins(self.law.angles());
        }
        if now_us >= self.next_sample_us {
            // DONE (M11b): the recording a trickle, a row a second
            self.next_sample_us = now_us
                + if flight.stage == Stage::Done {
                    DONE_SAMPLE_US
                } else {
                    SAMPLE_US
                };
            self.sample(now_us, flight.stage);
            hooks.vitals(&self.law);
        }
    }

    /// The flight.csv row (plan B's columns: centi-units where plan B wrote fixnums).
    fn sample(&mut self, now_us: u64, stage: Stage) {
        let law = &self.law;
        let guidance = law.guidance();
        let [roll, pitch, yaw] = law.commands();
        let wind = law.wind();
        let row = [
            Cell::I32(i32::from(stage as u8)),
            Cell::I32(i32::from(law.active())),
            Cell::I32((law.airspeed() * 100.0) as i32),
            Cell::I32(law.cap()),
            Cell::I32((guidance.roll_setpoint() * 100.0).round() as i32),
            Cell::I32((guidance.pitch_setpoint() * 100.0).round() as i32),
            Cell::I32(guidance.heading_error()),
            Cell::I32(roll),
            Cell::I32(pitch),
            Cell::I32(yaw),
            Cell::I32((wind.speed() * 100.0) as i32),
            Cell::I32(wind.direction() as i32),
            Cell::I32(i32::from(guidance.reckoning())),
        ];
        self.rows.push(now_us, &row);
    }
}

/// The control tick's pure duties, each present when its entry came up.
pub struct ControlCore {
    /// The attitude filter.
    filter: Option<FilterDuty>,
    /// The stage machine.
    sequencer: Option<SequencerDuty>,
    /// The flight loop.
    flight: Option<FlightDuty>,
}

impl ControlCore {
    /// The duties set up on `topology` for a tick of `tick_us`, started at `now_us`; a filter with no `attitude`
    /// quantity does not run (logged).
    pub fn new(
        topology: &'static Topology,
        tick_us: u64,
        now_us: u64,
        filter: Option<FilterSetup>,
        sequencer: Option<SequencerSetup>,
        flight: Option<FlightSetup>,
    ) -> ControlCore {
        // the board's boot order: the stage machine's binding lines before the filter's
        let sequencer = sequencer.map(|setup| SequencerDuty::new(setup, topology));
        ControlCore {
            filter: filter.and_then(|setup| FilterDuty::new(setup, topology, tick_us, now_us)),
            sequencer,
            flight: flight.map(|setup| FlightDuty::new(setup, topology)),
        }
    }

    /// The stage machine's bound sources as one line, and the sources past their class's slots; None without one.
    pub fn binding(&self) -> Option<(String, Vec<String>)> {
        self.sequencer.as_ref().map(SequencerDuty::binding)
    }

    /// The filter's entry name, when it runs.
    pub fn filter(&self) -> Option<&'static str> {
        self.filter.as_ref().map(|filter| filter.name)
    }

    /// The stage machine's entry name, when it runs.
    pub fn sequencer(&self) -> Option<&'static str> {
        self.sequencer.as_ref().map(|sequencer| sequencer.name)
    }

    /// The flight law, when it runs: its mixer's surfaces, its fin angles.
    pub fn law(&self) -> Option<&FlightLaw> {
        self.flight.as_ref().map(|flight| &flight.law)
    }

    /// One control tick that woke at `now_us`, `slack_us` being half a tick, `done` whether the flight was at DONE
    /// when it woke: the filter (not at DONE), then the stage machine, then the law, in the board's order (D11 of the
    /// M9 plan: the attitude before the stage machine).
    pub fn tick(
        &mut self,
        now_us: u64,
        slack_us: u64,
        done: bool,
        writer: &mut FlightWriter<'_>,
        hooks: &mut dyn Hooks,
    ) {
        if let Some(filter) = &mut self.filter
            && !done
        {
            filter.tick(now_us, hooks);
        }
        if let Some(sequencer) = &mut self.sequencer {
            sequencer.tick(now_us, slack_us, writer, hooks);
        }
        if let Some(flight) = &mut self.flight {
            flight.tick(writer, hooks);
        }
    }
}

#[cfg(test)]
mod tests {
    use recorder::Rows;
    use sequencer::FlightWord;
    use serde_json::{Value, json};

    use super::*;
    use crate::{Table, candidate, topology};

    /// A host that counts what the tick asked of it.
    #[derive(Default)]
    struct Probe {
        /// The clock it answers.
        now_us: u64,
        /// The clock's reads.
        reads: u32,
        /// The filter's state handed on.
        attitudes: u32,
    }

    impl Hooks for Probe {
        fn now_us(&mut self) -> u64 {
            self.reads += 1;
            self.now_us
        }

        fn mission(&mut self) -> (Option<Landing>, Option<Position>) {
            (None, None)
        }

        fn drift(&mut self) -> [f32; 2] {
            [0.0; 2]
        }

        fn flight_flag(&mut self, _raised: bool) {}

        fn moved(&mut self, _stage: Stage) {}

        fn veto_edge(&mut self) {}

        fn attitude(&mut self, _row: [i32; 6], _seeded: bool, _blind: bool) {
            self.attitudes += 1;
        }

        fn fins(&mut self, _angles: &[i32]) {}

        fn vitals(&mut self, _law: &FlightLaw) {}
    }

    /// The topology of `document`, every entry up.
    fn topology_of(document: Value) -> &'static Topology {
        let document: &'static Value = Box::leak(Box::new(document));
        let entries = board_config::directory(document);
        let table = Table::new(
            entries
                .iter()
                .map(|entry| (candidate(document, entry), Ok(())))
                .collect(),
        );
        Box::leak(Box::new(topology(&entries, &table).0))
    }

    /// The ticks (ms from the first) at which `core` read the clock more than `per_tick` times, over `ticks` 10 ms
    /// ticks, at DONE when `done`.
    fn stepped(core: &mut ControlCore, ticks: u64, done: bool) -> (Vec<u64>, Probe) {
        let word = FlightWord::default();
        let mut writer = word.writer().expect("the word's writer");
        let mut probe = Probe::default();
        let mut steps = Vec::new();
        for tick in 0..ticks {
            probe.now_us = 10_000_000 + tick * 10_000;
            let before = probe.reads;
            core.tick(probe.now_us, 5_000, done, &mut writer, &mut probe);
            if probe.reads > before {
                steps.push(tick * 10);
            }
        }
        (steps, probe)
    }

    #[test]
    fn the_stage_machine_steps_on_the_board_s_grid() {
        // a 50 ms step on a 10 ms tick, half a tick of slack: the board's control thread steps at 0, 40, 90, 140 ms
        // (the SITL's own pacing once stepped at 0, 30, 80: a phase no board ran)
        let topology = topology_of(json!({"sensors": [], "components": []}));
        let settings = sequencer::Settings {
            period_ms: 50,
            ..sequencer::Settings::default()
        };
        let mut core = ControlCore::new(
            topology,
            10_000,
            10_000_000,
            None,
            Some(SequencerSetup {
                name: "sequencer",
                settings,
                rows: Rows::new("sequencer", "record", None),
                vetoes: Rows::new("sequencer", "vetoes", None),
            }),
            None,
        );
        let (steps, _) = stepped(&mut core, 20, false);
        assert_eq!(steps, [0, 40, 90, 140, 190]);
    }

    #[test]
    fn the_filter_steps_on_its_pace_and_never_at_done() {
        let document = json!({
            "sensors": [{"name": "attitude", "activity": "attitude", "enabled": true, "period_ms": 20,
                         "provides": {"attitude": {"priority": 1, "timeout_ms": 40}}}],
            "components": [],
        });
        let topology = topology_of(document);
        let setup = || FilterSetup {
            name: "attitude",
            settings: control::attitude::Settings::from_json(&json!({"period_ms": 20})),
            rows: Rows::new("attitude", "row", None),
        };
        let mut core = ControlCore::new(topology, 10_000, 10_000_000, Some(setup()), None, None);
        assert_eq!(core.filter(), Some("attitude"));
        let (_, probe) = stepped(&mut core, 10, false);
        assert_eq!(probe.attitudes, 5, "every other 10 ms tick");
        // NEGATIVE: at DONE the filter is stopped, as the board's IMU is
        let mut core = ControlCore::new(topology, 10_000, 10_000_000, Some(setup()), None, None);
        let (_, probe) = stepped(&mut core, 10, true);
        assert_eq!(probe.attitudes, 0);
    }

    #[test]
    fn a_filter_with_no_attitude_quantity_does_not_run() {
        // NEGATIVE: nothing provides `attitude`, so there is nothing to publish into
        let topology = topology_of(json!({"sensors": [], "components": []}));
        let core = ControlCore::new(
            topology,
            10_000,
            10_000_000,
            Some(FilterSetup {
                name: "attitude",
                settings: control::attitude::Settings::from_json(&json!({})),
                rows: Rows::new("attitude", "row", None),
            }),
            None,
            None,
        );
        assert_eq!(core.filter(), None);
        assert!(core.binding().is_none() && core.law().is_none());
    }
}
