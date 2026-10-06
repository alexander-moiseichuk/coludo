//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage machine answers as plan B does, against two fixtures golden/generate.py builds from src/glider on
//! CPython:
//! - commons.json, sections `dwell_step` and `apogee_step` (golden/build_commons.py; the other two sections are
//!   crates/control's): the detector primitives case by case;
//! - sequencer_trace.json (golden/build_sequencer_trace.py): plan B's `Sequencer._tick` driven tick by tick,
//!   where every stage and every sequencer.csv row (stage and reason text) must match EXACTLY.

use golden::{Compare, Fixture};
use sequencer::commons::{self, Apogee, Dwell};
use sequencer::{Reading, Readings, Reason, Settings, Source, Stage, StageMachine};
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

/// A scenario's settings: Rust's defaults with the config's keys over them -- as the firmware builds them from
/// the board config.
fn settings(config: &Value) -> Settings {
    let mut settings = Settings::default();
    for (key, value) in config.as_object().expect("a config is an object") {
        match key.as_str() {
            "launch_g" => settings.launch_g = float(value),
            "launch_ms" => settings.launch_ms = milliseconds(value),
            "launch_alt_m" => settings.launch_alt_m = float(value),
            "boost_timeout_ms" => settings.boost_timeout_ms = milliseconds(value),
            "apogee_drop_m" => settings.apogee_drop_m = float(value),
            "apogee_arm_ms" => settings.apogee_arm_ms = milliseconds(value),
            "flight_timeout_ms" => settings.flight_timeout_ms = milliseconds(value),
            "land_agl_m" => settings.land_agl_m = float(value),
            "land_ms" => settings.land_ms = milliseconds(value),
            "still_g" => settings.still_g = float(value),
            "ground_ms" => settings.ground_ms = milliseconds(value),
            "land_timeout_ms" => settings.land_timeout_ms = milliseconds(value),
            other => panic!("a sequencer config key the port does not know: {other}"),
        }
    }
    settings
}

/// Settings as the fixture's init.settings writes them.
fn settings_json(settings: &Settings) -> Value {
    json!({
        "launch_g": f64::from(settings.launch_g), "launch_ms": settings.launch_ms,
        "launch_alt_m": f64::from(settings.launch_alt_m), "boost_timeout_ms": settings.boost_timeout_ms,
        "apogee_drop_m": f64::from(settings.apogee_drop_m), "apogee_arm_ms": settings.apogee_arm_ms,
        "flight_timeout_ms": settings.flight_timeout_ms, "land_agl_m": f64::from(settings.land_agl_m),
        "land_ms": settings.land_ms, "still_g": f64::from(settings.still_g), "ground_ms": settings.ground_ms,
        "land_timeout_ms": settings.land_timeout_ms,
    })
}

/// A scripted read: null (no reading at all), a stale one (null source: what plan B's read() extrapolates and
/// the Rust databoard never produces) or a fresh one.
fn reading<T>(read: &Value, value: impl Fn(&Value) -> T) -> Option<Reading<T>> {
    if read.is_null() || read["source"].is_null() {
        return None;
    }
    let source = match read["source"].as_str().expect("a source is a device name") {
        "gnss" => Source::Gnss,
        _ => Source::Other,
    };
    Some(Reading {
        value: value(&read["value"]),
        source,
    })
}

/// A step's three reads.
fn readings(reads: &Value) -> Readings {
    let vector = |value: &Value| {
        let axes = value.as_array().expect("an accel value is [x, y, z]");
        [float(&axes[0]), float(&axes[1]), float(&axes[2])]
    };
    Readings {
        acceleration: reading(&reads["accel"], vector),
        elevation: reading(&reads["elevation"], float),
        height_above_ground: reading(&reads["agl"], float),
    }
}

/// A stage by its fixture name.
fn stage(value: &Value) -> Stage {
    Stage::from_name(value.as_str().expect("a stage is a name")).unwrap_or_else(|| panic!("no stage {value}"))
}

/// Every scenario step, run on the port: (name, plan B's out, the Rust out). Each step's stage and manual flag
/// come from the fixture -- the Controller's, after the step's events -- and the machine's advance is reported
/// as the stage after the step.
fn trace_answers() -> Vec<(String, Value, Value)> {
    let fixture = Fixture::parse(SEQUENCER_TRACE);
    let mut answers = Vec::new();
    for scenario in fixture.get("scenarios").as_array().expect("scenarios is a list") {
        let name = scenario["name"].as_str().expect("a scenario has a name");
        let mut machine = StageMachine::new(settings(&scenario["init"]["config"]));
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
            let result = machine.step(now_ms, current, &readings(&input["reads"]), manual);
            let after = result.advance.map_or(current, |advance| advance.to);
            let records: Vec<Value> = result
                .transitions()
                .map(|transition| json!([transition.to.name(), transition.reason.to_string()]))
                .collect();
            answers.push((
                format!("{name} step {index} (now_ms {now_ms})"),
                step["out"].clone(),
                json!({"stage": after.name(), "records": records}),
            ));
        }
    }
    answers
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
fn the_trace_matches_plan_b_exactly() {
    let fixture = Fixture::parse(SEQUENCER_TRACE);
    let compare = fixture.compare();
    assert_eq!(compare, Compare::Exact, "stages and reasons compare exactly");
    let answers = trace_answers();
    assert!(answers.len() > 100, "a trace that compared nothing must not pass");
    let failures: Vec<String> = answers
        .iter()
        .filter_map(|(name, owed, answer)| compare.check(name, owed, answer).err())
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} sequencer steps differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
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
        .flat_map(|(_, owed, _)| owed["records"].as_array().unwrap())
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
            .any(|(_, owed, _)| owed["records"].as_array().unwrap().len() == 2),
        "no step with two rows"
    );
    for stage in Stage::ALL {
        assert!(
            answers.iter().any(|(_, owed, _)| owed["stage"] == stage.name()),
            "the trace never reaches {}",
            stage.name()
        );
    }
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
            &readings(&reads),
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
