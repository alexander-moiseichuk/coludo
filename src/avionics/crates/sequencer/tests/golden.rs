//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage machine answers as plan B does, against two fixtures golden/generate.py builds from src/glider on
//! CPython:
//! - commons.json, sections `dwell_step` and `apogee_step` (golden/build_commons.py; the other two sections are
//!   crates/control's): the detector primitives case by case;
//! - sequencer_trace.json (golden/build_sequencer_trace.py): plan B's `Sequencer._tick` driven tick by tick,
//!   where every stage and every sequencer.csv row (stage and reason text) must match EXACTLY, from settings
//!   `Settings::from_config` reads out of each scenario's config as the firmware reads the board config. Plan B's
//!   fused read is one source per class: the accel in the first accelerometer slot, the elevation in the first
//!   barometer slot (the GNSS slot when the GNSS vouches for it), the agl in the laser slot. The port's own rules are
//!   pinned in the fixture, each step they change carrying the `divergence` the port owes instead and its `cause`:
//!   the altitude launch's rate gate removes the altitude launches whose elevation never rose `launch_alt_m` within
//!   `launch_alt_window_ms` above a level it held for `launch_ms` (`rate_gate`); and the fusion rule's physics vetoes
//!   hold the altitude launch at rest and the apogee under thrust (`physics_veto`, only in the scenarios made for
//!   them). Its per-source trackers move no plan B event (`source_history` would pin one): the GNSS standing in for a
//!   stale barometer at the top joins the flight afresh and calls plan B's apogee on plan B's step.

use golden::{Compare, Fixture};
use sequencer::commons::{self, Apogee, Dwell};
use sequencer::{Detector, Physics, Reason, Settings, SourceReadings, Stage, StageMachine, Veto, VetoEdge};
use serde_json::{Value, json};

/// plan B's commons.py primitives (four sections; this crate ports two).
const COMMONS: &str = include_str!("../../../golden/commons.json");

/// plan B's `Sequencer._tick` over scripted and seeded scenarios.
const SEQUENCER_TRACE: &str = include_str!("../../../golden/sequencer_trace.json");

/// A section of commons.json as golden fixture text: `{compare, note, cases}`.
fn section(name: &str) -> String {
    Fixture::parse(COMMONS).get(name).to_string()
}

/// A fixture clock or duration in ms: a non-negative integer that fits the u32 clock.
fn milliseconds(value: &Value) -> u32 {
    value
        .as_u64()
        .and_then(|milliseconds| u32::try_from(milliseconds).ok())
        .unwrap_or_else(|| panic!("a millisecond value is a u32: {value}"))
}

/// A fixture metre or g value, as the f32 the port computes in.
fn float(value: &Value) -> f32 {
    value.as_f64().unwrap_or_else(|| panic!("a number: {value}")) as f32
}

/// A fixture value that may be null.
fn optional<T>(value: &Value, read: impl Fn(&Value) -> T) -> Option<T> {
    (!value.is_null()).then(|| read(value))
}

/// An f32 answer as the fixture writes a float (an f32 widens to f64 exactly).
fn answer(value: Option<f32>) -> Value {
    value.map_or(Value::Null, |value| json!(f64::from(value)))
}

#[test]
fn the_fixtures_say_where_they_came_from() {
    for (text, generator, source) in [
        (COMMONS, "commons", "src/glider/commons.py"),
        (SEQUENCER_TRACE, "sequencer_trace", "src/glider/tasks/sequencer.py"),
    ] {
        let fixture = Fixture::parse(text);
        let meta = fixture.get("meta");
        assert!(
            meta["tag"].as_str().unwrap().starts_with("micropython_plan_B"),
            "{meta}"
        );
        assert_eq!(meta["generator"], generator);
        assert!(meta["source"].as_str().unwrap().contains(source), "{meta}");
    }
}

/// The port's dwell_step for one commons.json case.
fn dwell(input: &Value) -> Value {
    let state = Dwell {
        credit_ms: milliseconds(&input["credit_ms"]),
        last_ms: optional(&input["last_ms"], milliseconds),
    };
    let active = input["active"].as_bool().expect("active is a flag");
    let (state, fired) = commons::dwell_step(
        active,
        milliseconds(&input["now_ms"]),
        state,
        milliseconds(&input["threshold_ms"]),
    );
    json!({"credit_ms": state.credit_ms, "last_ms": state.last_ms, "fired": fired})
}

/// The port's apogee_step for one commons.json case.
fn apogee(input: &Value) -> Value {
    let state = Apogee {
        peak: optional(&input["peak"], float),
        since_ms: optional(&input["since_ms"], milliseconds),
        smooth: optional(&input["smooth"], float),
    };
    let (state, fired) = commons::apogee_step(
        optional(&input["elevation"], float),
        milliseconds(&input["now_ms"]),
        state,
        float(&input["drop_m"]),
        milliseconds(&input["dwell_ms"]),
    );
    json!({"peak": answer(state.peak), "since_ms": state.since_ms, "smooth": answer(state.smooth), "fired": fired})
}

#[test]
fn dwell_step_matches_plan_b() {
    golden::run(&section("dwell_step"), dwell);
}

#[test]
fn apogee_step_matches_plan_b() {
    golden::run(&section("apogee_step"), apogee);
}

#[test]
#[should_panic(expected = "golden cases failed")]
fn a_dwell_that_resets_instead_of_draining_fails() {
    // the contiguous dwell plan B replaced: any miss wipes the credit
    golden::run(&section("dwell_step"), |input| {
        if input["active"] == false {
            json!({"credit_ms": 0, "last_ms": null, "fired": false})
        } else {
            dwell(input)
        }
    });
}

#[test]
#[should_panic(expected = "golden cases failed")]
fn an_unsmoothed_peak_fails() {
    // the running maximum plan B replaced: the raw elevation is the peak
    golden::run(&section("apogee_step"), |input| {
        let mut answer = apogee(input);
        if let Some(elevation) = input["elevation"].as_f64()
            && input["peak"].as_f64().is_none_or(|peak| elevation > peak)
        {
            answer["peak"] = json!(elevation);
        }
        answer
    });
}

/// A scenario's settings, read from its config as the firmware reads the sequencer's board-config block. Every key
/// must be one the port reads: `from_config` ignores the others, as plan B does, so a key plan B's shipped entry
/// gains later would otherwise reach the fixture and go unported unnoticed.
fn settings(config: &Value) -> Settings {
    let read = settings_json(&Settings::default());
    for key in config.as_object().expect("a config is an object").keys() {
        assert!(
            read.get(key).is_some(),
            "a sequencer config key the port does not read: {key}"
        );
    }
    Settings::from_config(config).unwrap_or_else(|error| panic!("a fixture config plan B flies is refused: {error}"))
}

#[test]
#[should_panic(expected = "a sequencer config key the port does not read: apogee_vote_ms")]
fn a_config_key_the_port_does_not_read_fails() {
    settings(&json!({"launch_g": 2.5, "apogee_vote_ms": 250}));
}

/// Settings as the fixture's init.settings writes them.
fn settings_json(settings: &Settings) -> Value {
    json!({
        "period_ms": settings.period_ms,
        "launch_g": f64::from(settings.launch_g), "launch_ms": settings.launch_ms,
        "launch_alt_m": f64::from(settings.launch_alt_m), "boost_timeout_ms": settings.boost_timeout_ms,
        "apogee_drop_m": f64::from(settings.apogee_drop_m), "apogee_arm_ms": settings.apogee_arm_ms,
        "flight_timeout_ms": settings.flight_timeout_ms, "land_agl_m": f64::from(settings.land_agl_m),
        "land_ms": settings.land_ms, "still_g": f64::from(settings.still_g), "ground_ms": settings.ground_ms,
        "land_timeout_ms": settings.land_timeout_ms,
    })
}

/// A scripted read's fresh value and its source's device name: None for null (no reading at all) and for a stale
/// read (null source: what plan B's read() extrapolates and the Rust databoard never produces).
fn reading<T>(read: &Value, value: impl Fn(&Value) -> T) -> Option<(T, &str)> {
    if read.is_null() || read["source"].is_null() {
        return None;
    }
    let source = read["source"].as_str().expect("a source is a device name");
    Some((value(&read["value"]), source))
}

/// A step's three reads as the port's slots take plan B's fused read: one source per class, the GNSS's elevation in
/// the GNSS slot.
fn readings(reads: &Value) -> SourceReadings {
    let vector = |value: &Value| {
        let axes = value.as_array().expect("an accel value is [x, y, z]");
        [float(&axes[0]), float(&axes[1]), float(&axes[2])]
    };
    let mut slots = SourceReadings::default();
    slots.accelerations[0] = reading(&reads["accel"], vector).map(|(value, _)| value);
    match reading(&reads["elevation"], float) {
        Some((metres, sequencer::GNSS_SOURCE)) => slots.gnss_elevation = Some(metres),
        Some((metres, _)) => slots.elevations[0] = Some(metres),
        None => {}
    }
    slots.height_above_ground = reading(&reads["agl"], float).map(|(value, _)| value);
    slots
}

/// A stage by its fixture name.
fn stage(value: &Value) -> Stage {
    Stage::from_name(value.as_str().expect("a stage is a name")).unwrap_or_else(|| panic!("no stage {value}"))
}

/// One scenario step as the port ran it.
struct Answer {
    /// The scenario, the step's index and its clock, for a failure message.
    name: String,
    /// Plan B's out.
    plan_b: Value,
    /// The out the port owes instead, on a step its own rules change; None on every other.
    divergence: Option<Value>,
    /// What changes the step: `rate_gate` or `physics_veto` (or `source_history`, which the fixture pins nowhere);
    /// None where nothing does.
    cause: Option<String>,
    /// The port's out.
    port: Value,
    /// The veto episodes open after the step, by detector.
    vetoes: Vec<Veto>,
}

impl Answer {
    /// The out the port owes: the divergence's, else plan B's.
    fn owed(&self) -> &Value {
        self.divergence.as_ref().unwrap_or(&self.plan_b)
    }
}

/// Every scenario step, run on the port with `adapt` making its slots from the step's reads. Each step's stage and
/// manual flag come from the fixture -- the Controller's, after the step's events -- and the machine's advance is
/// reported as the stage after the step.
fn trace_answers_through(adapt: impl Fn(&Value) -> SourceReadings) -> Vec<Answer> {
    let fixture = Fixture::parse(SEQUENCER_TRACE);
    let mut answers = Vec::new();
    for scenario in fixture.get("scenarios").as_array().expect("scenarios is a list") {
        let name = scenario["name"].as_str().expect("a scenario has a name");
        let mut machine = StageMachine::new(settings(&scenario["init"]["config"]));
        let mut open: Vec<Veto> = Vec::new();
        for (index, step) in scenario["steps"]
            .as_array()
            .expect("steps is a list")
            .iter()
            .enumerate()
        {
            let input = &step["in"];
            let now_ms = milliseconds(&input["now_ms"]);
            let current = stage(&input["stage"]);
            let manual = input["manual"].as_bool().expect("manual is a flag");
            let result = machine.step(now_ms, current, adapt(&input["reads"]), manual);
            let after = result.advance.map_or(current, |advance| advance.to);
            let records: Vec<Value> = result
                .transitions()
                .map(|transition| json!([transition.to.name(), transition.reason.to_string()]))
                .collect();
            for edge in result.vetoes() {
                open.retain(|veto| veto.detector != edge.veto().detector);
                if let VetoEdge::Begin(veto) = edge {
                    open.push(veto);
                }
            }
            answers.push(Answer {
                name: format!("{name} step {index} (now_ms {now_ms})"),
                plan_b: step["out"].clone(),
                divergence: step.get("divergence").map(|divergence| divergence["out"].clone()),
                cause: step
                    .get("divergence")
                    .map(|divergence| divergence["cause"].as_str().expect("a cause").to_string()),
                port: json!({"stage": after.name(), "records": records}),
                vetoes: open.clone(),
            });
        }
    }
    answers
}

/// Every scenario step, run on the port through the fused-read adapter ([`readings`]).
fn trace_answers() -> Vec<Answer> {
    trace_answers_through(readings)
}

#[test]
fn the_settings_and_their_defaults_are_plan_b_s() {
    // f32 thresholds widen to f64 near plan B's decimal (0.3 is 0.30000001); integers compare exactly
    let compare = Compare::Abs(1e-6);
    for scenario in Fixture::parse(SEQUENCER_TRACE).get("scenarios").as_array().unwrap() {
        let init = &scenario["init"];
        let answer = settings_json(&settings(&init["config"]));
        assert_eq!(
            compare.check(scenario["name"].as_str().unwrap(), &init["settings"], &answer),
            Ok(())
        );
    }
}

#[test]
fn the_trace_matches_plan_b_exactly_but_for_the_pinned_divergences() {
    let fixture = Fixture::parse(SEQUENCER_TRACE);
    let compare = fixture.compare();
    assert_eq!(compare, Compare::Exact, "stages and reasons compare exactly");
    let answers = trace_answers();
    assert!(answers.len() > 100, "a trace that compared nothing must not pass");
    let failures: Vec<String> = answers
        .iter()
        .filter_map(|answer| compare.check(&answer.name, answer.owed(), &answer.port).err())
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} sequencer steps differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
    // every pinned divergence is one: the port answers plan B's out there, and only there, differently
    let differing: Vec<&str> = answers
        .iter()
        .filter(|answer| answer.port != answer.plan_b)
        .map(|answer| answer.name.as_str())
        .collect();
    let pinned: Vec<&str> = answers
        .iter()
        .filter(|answer| answer.divergence.is_some())
        .map(|answer| answer.name.as_str())
        .collect();
    assert_eq!(differing, pinned);
}

/// How many of an out's rows have a reason starting with `prefix`.
fn rows_starting(out: &Value, prefix: &str) -> usize {
    out["records"]
        .as_array()
        .expect("records is a list")
        .iter()
        .filter(|record| record[1].as_str().is_some_and(|reason| reason.starts_with(prefix)))
        .count()
}

/// An out's rows without its altitude launch.
fn without_the_altitude_launch(out: &Value) -> Vec<Value> {
    let records = out["records"].as_array().expect("records is a list");
    records
        .iter()
        .filter(|record| {
            !record[1]
                .as_str()
                .is_some_and(|reason| reason.starts_with("launch alt="))
        })
        .cloned()
        .collect()
}

/// Whether a scenario's fresh barometer elevation, over its steps up to and including `last`, ever rose more than
/// `rise_m` within `window_ms` above a level it held for `held_ms`: the rate gate's question asked of the fixture's
/// own reads (every fresh reading that is not the GNSS's, whatever the stage, each at its own clock), not of the
/// machine. A reading confirms the highest of the readings from the last one at least `held_ms` older on, itself
/// included; with none that old it confirms no level.
fn rose_within(steps: &[Value], last: usize, rise_m: f32, window_ms: u32, held_ms: u32) -> bool {
    let readings: Vec<(u32, f32)> = steps[..=last]
        .iter()
        .filter_map(|step| {
            let (metres, source) = reading(&step["in"]["reads"]["elevation"], float)?;
            (source != sequencer::GNSS_SOURCE).then(|| (milliseconds(&step["in"]["now_ms"]), metres))
        })
        .collect();
    let age_ms = |now_ms: u32, then_ms: u32| i64::from(commons::elapsed_ms(now_ms, then_ms));
    let levels: Vec<Option<f32>> = (0..readings.len())
        .map(|confirming| {
            let now_ms = readings[confirming].0;
            let held_from = readings[..=confirming]
                .iter()
                .rposition(|&(then_ms, _)| age_ms(now_ms, then_ms) >= i64::from(held_ms))?;
            let held = readings[held_from..=confirming].iter().map(|&(_, metres)| metres);
            Some(held.fold(f32::MIN, f32::max))
        })
        .collect();
    readings.iter().enumerate().any(|(later, &(now_ms, metres))| {
        readings[..later].iter().zip(&levels).any(|(&(then_ms, _), level)| {
            let within = (0..=i64::from(window_ms)).contains(&age_ms(now_ms, then_ms));
            within && level.is_some_and(|level_m| metres - level_m > rise_m)
        })
    })
}

#[test]
fn the_rate_gate_removes_only_altitude_launches_of_a_slow_rise() {
    let fixture = Fixture::parse(SEQUENCER_TRACE);
    let (mut made, mut removed, mut carried) = (0, 0, 0);
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let settings = settings(&scenario["init"]["config"]);
        let (line_m, window_ms) = (settings.launch_alt_m, settings.launch_alt_window_ms);
        let steps = scenario["steps"].as_array().unwrap();
        for (index, step) in steps.iter().enumerate() {
            let name = format!("{} step {index}", scenario["name"]);
            let fast = || rose_within(steps, index, line_m, window_ms, settings.launch_ms);
            let Some(divergence) = step.get("divergence") else {
                // an altitude launch the port makes as plan B does: the same question answers that it rose
                if rows_starting(&step["out"], "launch alt=") > 0 {
                    assert!(
                        fast(),
                        "{name}: an altitude launch that never rose launch_alt_m within the window"
                    );
                    made += 1;
                }
                continue;
            };
            if divergence["cause"] != "rate_gate" {
                continue; // the fusion rule's: the_fusion_rule_changes_only_the_events_it_pins
            }
            let (out, owed) = (&step["out"], &divergence["out"]);
            if rows_starting(out, "launch alt=") > 0 {
                // an altitude launch removed: the rise was slow, and the tick records nothing else differently
                assert!(!fast(), "{name}: the elevation rose launch_alt_m within the window");
                assert_eq!(rows_starting(owed, "launch alt="), 0, "{name}");
                let accel_launch = rows_starting(owed, "launch |a|=");
                assert_eq!(
                    without_the_altitude_launch(owed).len(),
                    without_the_altitude_launch(out).len() + accel_launch,
                    "{name}"
                );
                assert!(
                    without_the_altitude_launch(owed).starts_with(&without_the_altitude_launch(out)),
                    "{name}"
                );
                let stage = if accel_launch > 0 {
                    &out["stage"]
                } else {
                    &step["in"]["stage"]
                };
                assert_eq!(&owed["stage"], stage, "{name}");
                removed += 1;
                continue;
            }
            // the tick after a launch the port did not make: plan B's BOOSTING reaches it from outside
            let before = &steps[index - 1];
            assert!(
                rows_starting(&before["out"], "launch alt=") > 0 && before["divergence"]["out"]["stage"] == "setting",
                "{name}: a divergence that follows no removed launch"
            );
            let mut records = vec![json!([step["in"]["stage"], "external"])];
            records.extend(out["records"].as_array().unwrap().iter().cloned());
            assert_eq!(owed, &json!({"stage": out["stage"], "records": records}), "{name}");
            carried += 1;
        }
    }
    assert_eq!(
        (made, removed, carried),
        (1, 9, 3),
        "the fast climb kept; six launches over the line from the first fresh baro reading, two from one low reading, \
         one slow climb removed; three flights carried on"
    );
    // the gate only removes: every launch the port makes, plan B makes on the same tick
    for answer in trace_answers() {
        if rows_starting(&answer.port, "launch ") > 0 {
            assert!(
                rows_starting(&answer.plan_b, "launch ") > 0,
                "{}: a launch plan B did not make",
                answer.name
            );
        }
    }
}

/// The scenarios made for the fusion rule's physics vetoes: the only ones they change.
const PHYSICS_SCENARIOS: [&str; 3] = [
    "a_drifted_pad_at_rest_then_a_fast_climb",
    "a_dropout_on_a_pad_at_rest_then_a_fast_climb",
    "a_sustained_boost_with_the_elevation_falling",
];

/// The steps of `scenario` that carry a divergence of `cause`, by name and index.
fn pinned(answers: &[Answer], cause: &str) -> Vec<String> {
    answers
        .iter()
        .filter(|answer| answer.cause.as_deref() == Some(cause))
        .map(|answer| answer.name.clone())
        .collect()
}

#[test]
fn the_fusion_rule_changes_only_the_events_it_pins() {
    let answers = trace_answers();
    let scenarios = Fixture::parse(SEQUENCER_TRACE)
        .get("scenarios")
        .as_array()
        .unwrap()
        .len();
    assert_eq!(scenarios, 49, "45 scenarios from before the fusion rule, 4 made for it");
    // the physics vetoes: no plan B event changes in the scenarios made before them; in each made for them, its one
    // event is removed and the next step carries plan B's stage on
    let physics = pinned(&answers, "physics_veto");
    assert_eq!(
        physics,
        [
            "a_drifted_pad_at_rest_then_a_fast_climb step 33 (now_ms 1650)",
            "a_drifted_pad_at_rest_then_a_fast_climb step 34 (now_ms 1700)",
            "a_dropout_on_a_pad_at_rest_then_a_fast_climb step 33 (now_ms 1650)",
            "a_dropout_on_a_pad_at_rest_then_a_fast_climb step 34 (now_ms 1700)",
            "a_sustained_boost_with_the_elevation_falling step 4 (now_ms 200)",
            "a_sustained_boost_with_the_elevation_falling step 5 (now_ms 250)",
        ]
    );
    for name in &physics {
        assert!(
            PHYSICS_SCENARIOS.iter().any(|scenario| name.starts_with(scenario)),
            "{name}"
        );
    }
    // where the event is removed, the port holds the detector with that physics veto open
    let held = |name: &str| {
        let answer = answers.iter().find(|answer| answer.name == name).expect("the step");
        answer
            .vetoes
            .iter()
            .map(|veto| (veto.detector, veto.physics))
            .collect::<Vec<_>>()
    };
    assert_eq!(held(&physics[0]), [(Detector::LaunchAltitude, Some(Physics::AtRest))]);
    assert_eq!(held(&physics[2]), [(Detector::LaunchAltitude, Some(Physics::AtRest))]);
    assert_eq!(held(&physics[4]), [(Detector::Apogee, Some(Physics::UnderThrust))]);
    // the per-source trackers move no apogee: where the GNSS stands in for a stale barometer at the top, after a gap
    // the barometer was read through, it joins the flight afresh and calls plan B's apogee on plan B's step
    assert_eq!(pinned(&answers, "source_history"), Vec::<String>::new());
    let top = answers
        .iter()
        .find(|answer| answer.name == "flight_dropouts_gnss_rocking step 104 (now_ms 6496)")
        .expect("the step");
    assert_eq!(rows_starting(&top.port, "apogee "), 1, "{}", top.port);
    let scenario = Fixture::parse(SEQUENCER_TRACE)
        .get("scenarios")
        .as_array()
        .unwrap()
        .iter()
        .find(|scenario| scenario["name"] == "flight_dropouts_gnss_rocking")
        .cloned()
        .expect("the scenario");
    let sources: Vec<&str> = scenario["steps"].as_array().unwrap()[..=104]
        .iter()
        .filter(|step| step["in"]["stage"] == "boosting")
        .filter_map(|step| reading(&step["in"]["reads"]["elevation"], float).map(|(_, source)| source))
        .collect();
    assert_eq!(
        sources.last(),
        Some(&"gnss"),
        "the GNSS decides the step plan B calls apogee on"
    );
    let handovers = sources.windows(2).filter(|pair| pair[0] != pair[1]).count();
    assert_eq!(
        handovers, 3,
        "the GNSS stood in twice in BOOSTING, the barometer fresh between"
    );
}

#[test]
fn a_gnss_never_read_fails_where_it_stands_in() {
    // NEGATIVE: the GNSS slot is where plan B's GNSS stand-ins reach the port. Left empty, the apogee and the landing
    // the GNSS decides alone go uncalled, and so does the apogee it calls standing in at the top
    let blind = |reads: &Value| SourceReadings {
        gnss_elevation: None,
        ..readings(reads)
    };
    let failures: Vec<String> = trace_answers_through(blind)
        .iter()
        .filter(|answer| answer.port != *answer.owed())
        .map(|answer| answer.name.clone())
        .collect();
    assert_eq!(
        failures,
        [
            "gnss_elevation_in_flight step 7 (now_ms 350)",
            "gnss_elevation_in_flight step 8 (now_ms 400)",
            "gnss_elevation_in_flight step 11 (now_ms 550)",
            "flight_dropouts_gnss_rocking step 104 (now_ms 6496)",
            "flight_dropouts_gnss_rocking step 105 (now_ms 6551)",
        ]
    );
}

#[test]
fn an_accelerometer_read_off_its_axes_fails_the_trace() {
    // NEGATIVE: the thrust gate reads the body frame's +x; the same |accel| read on z never sees the boost, so the
    // apogee under thrust is called, and the trace fails exactly there
    let off_axis = |reads: &Value| {
        let mut slots = readings(reads);
        slots.accelerations[0] = slots.accelerations[0].map(|[x, y, z]| [z, y, x]);
        slots
    };
    let failures: Vec<String> = trace_answers_through(off_axis)
        .iter()
        .filter(|answer| answer.port != *answer.owed())
        .map(|answer| answer.name.clone())
        .collect();
    assert_eq!(
        failures,
        [
            "a_sustained_boost_with_the_elevation_falling step 4 (now_ms 200)",
            "a_sustained_boost_with_the_elevation_falling step 5 (now_ms 250)",
        ]
    );
}

#[test]
fn the_trace_covers_every_reason_and_a_two_row_step() {
    let reasons = [
        Reason::External,
        Reason::LaunchAltitude {
            elevation_m: 0.0,
            dwell_ms: 0,
        },
        Reason::LaunchAcceleration {
            magnitude_g: 0.0,
            dwell_ms: 0,
        },
        Reason::Apogee { peak_m: 0.0 },
        Reason::BurnoutTimeout,
        Reason::HeightAboveGround { height_m: 0.0 },
        Reason::LandingTimeout { timeout_ms: 0 },
        Reason::Stationary { magnitude_g: 0.0 },
        Reason::FlightTimeout,
    ];
    let answers = trace_answers();
    let texts: Vec<&str> = answers
        .iter()
        .flat_map(|answer| answer.owed()["records"].as_array().unwrap())
        .map(|record| record[1].as_str().unwrap())
        .collect();
    for reason in reasons {
        // the text up to the first number names the reason
        let text = reason.to_string();
        let stem = text.split(|character: char| character.is_ascii_digit()).next().unwrap();
        assert!(
            texts.iter().any(|seen| seen.starts_with(stem)),
            "no '{stem}' row in the trace"
        );
    }
    assert!(
        answers
            .iter()
            .any(|answer| answer.owed()["records"].as_array().unwrap().len() == 2),
        "no step with two rows"
    );
    for stage in Stage::ALL {
        assert!(
            answers.iter().any(|answer| answer.owed()["stage"] == stage.name()),
            "the trace never reaches {}",
            stage.name()
        );
    }
}

#[test]
fn the_trace_covers_the_backstop_from_boosting_and_a_held_separation() {
    let fixture = Fixture::parse(SEQUENCER_TRACE);
    let steps: Vec<&Value> = fixture
        .get("scenarios")
        .as_array()
        .unwrap()
        .iter()
        .flat_map(|scenario| scenario["steps"].as_array().unwrap())
        .collect();
    let backstop_from_boosting = steps.iter().any(|step| {
        step["in"]["stage"] == "boosting"
            && step["out"]["records"]
                .as_array()
                .unwrap()
                .iter()
                .any(|record| record[1] == "flight timeout")
    });
    assert!(backstop_from_boosting, "no flight timeout fired from BOOSTING");
    let held_separation = steps.iter().any(|step| {
        let input = &step["in"];
        input["manual"] == true
            && input["stage"] == "gliding"
            && input["events"]
                .as_array()
                .is_some_and(|events| events.iter().any(|event| event == "separation"))
    });
    assert!(
        held_separation,
        "no separation moved a held BOOSTING (and kept the hold)"
    );
}

#[test]
fn a_stale_reading_taken_as_fresh_fails_the_trace() {
    // the bug the freshness gate exists for: a stale channel's extrapolated value used as a reading
    let fixture = Fixture::parse(SEQUENCER_TRACE);
    let compare = fixture.compare();
    let scenario = fixture
        .get("scenarios")
        .as_array()
        .unwrap()
        .iter()
        .find(|scenario| scenario["name"] == "stale_agl_at_apogee")
        .expect("the stale agl scenario");
    let mut machine = StageMachine::new(settings(&scenario["init"]["config"]));
    let mut failed = false;
    for step in scenario["steps"].as_array().unwrap() {
        let input = &step["in"];
        let mut reads = input["reads"].clone();
        for channel in ["accel", "elevation", "agl"] {
            if !reads[channel].is_null() && reads[channel]["source"].is_null() {
                reads[channel]["source"] = json!("extrapolated");
            }
        }
        let current = stage(&input["stage"]);
        let result = machine.step(
            milliseconds(&input["now_ms"]),
            current,
            readings(&reads),
            input["manual"].as_bool().unwrap(),
        );
        let records: Vec<Value> = result
            .transitions()
            .map(|transition| json!([transition.to.name(), transition.reason.to_string()]))
            .collect();
        let answer = json!({"stage": result.advance.map_or(current, |advance| advance.to).name(), "records": records});
        failed |= compare.check("stale", &step["out"], &answer).is_err();
    }
    assert!(
        failed,
        "an extrapolated agl of -9.6 m must land the glider at 274 m and fail the trace"
    );
}
