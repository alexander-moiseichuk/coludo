//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The GNSS receiver answers as plan B does: src/avionics/golden/gnss.json, which golden/build_gnss.py builds from
//! src/glider/gnss.py and the drivers on CPython, over test_gnss.py's captured bursts and seeded sweeps. Six
//! sections: checksum, degrees, nmea, ubx and commands are golden fixtures of their own; trace replays scenarios
//! step by step and compares each step's answer in four parts (degrees, speed, values, exact), each under its own
//! comparison. Where the fixture marks a divergence (hdop_cd from the decimal text, a ddmm field that is not
//! digits), the Rust answer it carries is owed instead of plan B's.

use control::navigation::{Position, degrees_from_e7};
use gnss::drivers::{atgm336h, neo6mv2};
use gnss::{Event, Gnss, SkyFrame, SkyRow, SkySwitch, Update, checksum_ok, degrees_e7, nmea};
use golden::{Compare, Fixture, hex};
use sequencer::Stage;
use serde_json::{Map, Value, json};

/// plan B's receiver over the checksum, degrees, nmea, ubx, commands and trace cases.
const GNSS: &str = include_str!("../../../golden/gnss.json");

/// The bytes a hex string names.
fn bytes(text: &str) -> Vec<u8> {
    (0..text.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&text[index..index + 2], 16).expect("hex"))
        .collect()
}

/// The answer a case owes: the Rust answer a divergence carries (its members but the note), else plan B's.
fn owed(case: &Value) -> Value {
    match case.get("divergence") {
        Some(Value::Object(divergence)) => {
            assert!(
                divergence["note"].as_str().is_some_and(|note| !note.is_empty()),
                "{case}"
            );
            let answer: Map<String, Value> = divergence
                .iter()
                .filter(|(key, _value)| key.as_str() != "note")
                .map(|(key, value)| (key.clone(), value.clone()))
                .collect();
            Value::Object(answer)
        }
        Some(other) => panic!("a divergence is an object: {other}"),
        None => case["output"].clone(),
    }
}

/// Run a section's cases through `port`, each against the answer it owes, under the section's comparison.
fn check(name: &str, mut port: impl FnMut(&Value) -> Value) {
    let fixture = Fixture::parse(GNSS);
    let section = fixture.get(name);
    let compare = Compare::from_json(&section["compare"]);
    let cases = section["cases"].as_array().expect("cases");
    assert!(!cases.is_empty(), "{name}: no cases");
    let failures: Vec<String> = cases
        .iter()
        .filter_map(|case| {
            let label = format!("{name} {}", case["name"].as_str().expect("a case name"));
            compare.check(&label, &owed(case), &port(&case["input"])).err()
        })
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} cases failed:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
}

/// A position as `[latitude, longitude]` in decimal degrees.
fn degrees(position: Position) -> Value {
    json!(position.degrees())
}

/// An optional f32 as a JSON number (widened exactly) or null.
fn number(value: Option<f32>) -> Value {
    value.map_or(Value::Null, |value| json!(f64::from(value)))
}

/// The sky switch of a scenario's driver at its rate.
fn sky_switch(init: &Value) -> SkySwitch {
    let hz = i32::try_from(init["hz"].as_i64().expect("hz")).expect("an i32 rate");
    match init["driver"].as_str() {
        Some("atgm336h") => atgm336h::sky_switch(hz),
        Some("neo6mv2") => neo6mv2::sky_switch(hz),
        other => panic!("unknown driver {other:?}"),
    }
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(GNSS);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG, "{meta}");
    assert_eq!(meta["generator"], "gnss");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/gnss.py"));
}

#[test]
fn checksums_match_plan_b() {
    check("checksum", |input| json!(checksum_ok(input.as_str().expect("a line"))));
}

#[test]
fn degrees_match_plan_b() {
    check("degrees", |input| {
        let value = input["value"].as_str().expect("value");
        let hemisphere = input["hemisphere"].as_str().expect("hemisphere");
        match degrees_e7(value, hemisphere) {
            Ok(units) => json!({"degrees": units.map(degrees_from_e7)}),
            Err(_malformed) => json!({"malformed": true}),
        }
    });
}

#[test]
fn degrees_round_to_the_nearest_unit() {
    // the comparison allows 1e-6 degree; the units must also be plan B's float rounded to the nearest 1e-7
    let fixture = Fixture::parse(GNSS);
    for case in fixture.get("degrees")["cases"].as_array().unwrap() {
        let (Some(expected), None) = (case["output"]["degrees"].as_f64(), case.get("divergence")) else {
            continue;
        };
        let units = expected * 1e7;
        if (units - units.trunc()).abs() > 0.499_999 && (units - units.trunc()).abs() < 0.500_001 {
            continue; // a tie in plan B's float could round either way
        }
        let input = &case["input"];
        let answer = degrees_e7(input["value"].as_str().unwrap(), input["hemisphere"].as_str().unwrap());
        assert_eq!(answer, Ok(Some(units.round() as i32)), "{}", case["name"]);
    }
}

#[test]
fn command_sentences_match_plan_b() {
    check("nmea", |input| json!(hex(&nmea(input.as_str().expect("a body")))));
}

#[test]
fn ubx_frames_match_plan_b() {
    check("ubx", |input| {
        let field = |name: &str| u8::try_from(input[name].as_u64().expect(name)).expect("a byte");
        let payload = bytes(input["payload"].as_str().expect("payload"));
        json!(hex(&neo6mv2::ubx(field("class"), field("id"), &payload)))
    });
}

#[test]
fn driver_inits_match_plan_b() {
    check("commands", |input| {
        let hz = i32::try_from(input["hz"].as_i64().expect("hz")).expect("an i32 rate");
        let (frames, switch) = match input["driver"].as_str() {
            Some("atgm336h") => (atgm336h::commands(hz).to_vec(), atgm336h::sky_switch(hz)),
            Some("neo6mv2") => (neo6mv2::commands(hz).to_vec(), neo6mv2::sky_switch(hz)),
            other => panic!("unknown driver {other:?}"),
        };
        let frames: Vec<String> = frames.iter().map(|frame| hex(frame)).collect();
        json!({"frames": frames, "sky_off": hex(&switch.off), "sky_on": hex(&switch.on)})
    });
}

/// What one trace step produced: the line's update, the sky row and the frame written, beside the events.
#[derive(Default)]
struct Answer {
    /// The channels and rows a line produced (and its events).
    update: Update,
    /// The sky row a settle or a tick pushed.
    sky_row: Option<SkyRow>,
    /// The frame a window or a tick wrote.
    write: Option<SkyFrame>,
}

/// Run one step's input on the receiver.
fn step(gnss: &mut Gnss, input: &Value) -> Answer {
    let time = |name: &str| input[name].as_u64().unwrap_or_else(|| panic!("{name} in {input}"));
    let stage = || {
        let id = u8::try_from(input["stage"].as_u64().expect("stage")).expect("a stage id");
        Stage::from_id(id).expect("a flight stage")
    };
    let mut answer = Answer::default();
    match input["op"].as_str().expect("op") {
        "line" => {
            let raw = format!("{}\r\n", input["text"].as_str().expect("text"));
            answer.update = gnss.line(raw.as_bytes(), time("now_us"));
        }
        "raw" => answer.update = gnss.line(&bytes(input["hex"].as_str().expect("hex")), time("now_us")),
        "parse" => answer.update = gnss.parse(input["text"].as_str().expect("text"), time("now_us")),
        "settle" => {
            if let Some((row, first)) = gnss.settle(time("at_us")) {
                answer.sky_row = Some(row);
                answer.update.events.extend(first);
            }
        }
        "sweep" => answer.update.events = gnss.sweep(time("at_us")),
        "window" => {
            if let Some((frame, event)) = gnss.sky_window(stage()) {
                answer.write = Some(frame);
                answer.update.events.push(event);
            }
        }
        "tick" => {
            let tick = gnss.tick(time("now_us"), stage());
            answer.update.events = tick.events;
            answer.sky_row = tick.sky_row;
            answer.write = tick.write;
        }
        other => panic!("unknown step {other}"),
    }
    answer
}

/// The parts of a trace step's answer, each under its own comparison (see the fixture's trace note): the
/// positions, the speeds (relative, tight enough to tell the knots constant), the other values, the exact rest.
const PARTS: [&str; 4] = ["degrees", "speed", "values", "exact"];

/// A step's answer in the fixture's four parts.
fn out(gnss: &Gnss, answer: &Answer, switch: &SkySwitch) -> Value {
    let update = &answer.update;
    let pushed: Vec<&str> = [
        ("position", update.position.is_some()),
        ("speed", update.speed_ms.is_some()),
        ("course", update.course_deg.is_some()),
        ("altitude", update.altitude_m.is_some()),
        ("elevation", update.elevation_m.is_some()),
    ]
    .into_iter()
    .filter_map(|(name, pushed)| pushed.then_some(name))
    .collect();
    let sky = gnss.sky_row();
    let events: Vec<String> = update.events.iter().map(Event::to_string).collect();
    json!({
        "degrees": {
            "position": update.position.map(degrees),
            "fix_row": update.fix_row.map(|row| degrees(row.position)),
        },
        "speed": {
            "speed_ms": number(update.speed_ms),
            "fix_row_knots": number(update.fix_row.map(|row| row.speed_knots)),
        },
        "values": {
            "course_deg": number(update.course_deg),
            "altitude_m": number(update.altitude_m), "elevation_m": number(update.elevation_m),
            "fix_row_course": number(update.fix_row.map(|row| row.course_deg)),
            "gga_row": update.gga_row.map(|row| [f64::from(row.altitude_m), f64::from(row.elevation_m)]),
            "hdop": f64::from(gnss.hdop()), "cn0_mean": f64::from(gnss.cn0_mean()),
        },
        "exact": {
            "pushed": pushed,
            "gga_row": update.gga_row.map(|row| [row.quality, row.satellites, row.hdop_centi]),
            "sky_row": answer.sky_row.map(|row| row.columns()),
            "events": events,
            "write": answer.write.map(|frame| hex(switch.frame(frame))),
            "fix": gnss.fix(), "fix_quality": gnss.fix_quality(), "satellites": gnss.satellites(),
            "in_view": sky.in_view, "used": sky.used, "mode": sky.mode, "cn0": sky.cn0,
            "antenna": sky.antenna.name(), "lines": gnss.lines(), "diagnosing": gnss.diagnosing(),
        },
    })
}

/// The answer a step owes: plan B's, with the members a divergence overrides in each part.
fn owed_step(step: &Value) -> Value {
    let mut owed = step["out"].clone();
    if let Some(divergence) = step.get("divergence") {
        assert!(
            divergence["note"].as_str().is_some_and(|note| !note.is_empty()),
            "{step}"
        );
        for part in PARTS {
            if let Some(Value::Object(overrides)) = divergence.get(part) {
                for (member, value) in overrides {
                    owed[part][member] = value.clone();
                }
            }
        }
    }
    owed
}

/// Replay every trace scenario on a fresh receiver; every step's three parts against what it owes.
fn replay(mut adjust: impl FnMut(&mut Value)) -> Vec<String> {
    let fixture = Fixture::parse(GNSS);
    let trace = fixture.get("trace");
    let compares: Vec<(&str, Compare)> = PARTS
        .into_iter()
        .map(|part| (part, Compare::from_json(&trace["compare"][part])))
        .collect();
    let mut failures = Vec::new();
    let mut steps = 0;
    for scenario in trace["scenarios"].as_array().expect("scenarios") {
        let name = scenario["name"].as_str().expect("a scenario name");
        let switch = sky_switch(&scenario["init"]);
        let mut gnss = Gnss::new();
        for (index, recorded) in scenario["steps"].as_array().expect("steps").iter().enumerate() {
            steps += 1;
            let answer = step(&mut gnss, &recorded["in"]);
            let mut answer = out(&gnss, &answer, &switch);
            adjust(&mut answer);
            let owed = owed_step(recorded);
            for (part, compare) in &compares {
                let label = format!("{name} step {index} {part}");
                if let Err(failure) = compare.check(&label, &owed[part], &answer[part]) {
                    failures.push(failure);
                }
            }
        }
    }
    assert!(steps > 0, "the trace has no steps");
    failures
}

#[test]
fn the_receiver_replays_plan_b_s_trace() {
    let failures = replay(|_answer| {});
    assert!(
        failures.is_empty(),
        "{} trace parts failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

#[test]
fn a_position_one_unit_short_still_passes_and_a_minute_digit_does_not() {
    // the degrees tolerance is 1e-6: half a unit of rounding passes, a lost 4th minute decimal (1.7e-6) fails
    let nudge = |offset: f64| {
        move |answer: &mut Value| {
            if let Some(position) = answer["degrees"]["position"].as_array_mut() {
                position[0] = json!(position[0].as_f64().unwrap() + offset);
            }
        }
    };
    assert!(replay(nudge(1e-7)).is_empty());
    assert!(!replay(nudge(1.7e-6)).is_empty());
}

#[test]
fn the_speed_part_tells_plan_b_s_knots_constant_from_the_exact_one() {
    // an answer scaled by `ratio`, as a port with another knots constant would push it
    let scaled = |ratio: f64| {
        move |answer: &mut Value| {
            if let Some(speed) = answer["speed"]["speed_ms"].as_f64() {
                answer["speed"]["speed_ms"] = json!(speed * ratio);
            }
        }
    };
    // f32 rounding of the same constant passes (1e-7 is about two of them); the exact 1852/3600 does not
    assert!(replay(scaled(1.0 + 1e-7)).is_empty());
    let exact = replay(scaled(1852.0 / 3600.0 / 0.514_444));
    assert!(!exact.is_empty(), "the trace carries non-zero speeds");
    assert!(exact.iter().all(|failure| failure.contains(" speed")), "{exact:?}");
}

#[test]
fn every_divergence_is_one() {
    let fixture = Fixture::parse(GNSS);
    let mut marked = 0;
    for case in fixture.get("degrees")["cases"].as_array().unwrap() {
        if case.get("divergence").is_some() {
            marked += 1;
            assert_ne!(owed(case), case["output"], "{}", case["name"]);
        }
    }
    for scenario in fixture.get("trace")["scenarios"].as_array().unwrap() {
        for recorded in scenario["steps"].as_array().unwrap() {
            if recorded.get("divergence").is_some() {
                marked += 1;
                assert_ne!(owed_step(recorded), recorded["out"], "{recorded}");
            }
        }
    }
    assert!(marked > 0, "hdop_cd and the ddmm digits diverge somewhere");
}

#[test]
fn a_wrong_count_or_event_fails_the_exact_part() {
    let lines = replay(|answer| answer["exact"]["lines"] = json!(999));
    assert!(!lines.is_empty() && lines.iter().all(|failure| failure.contains("exact")));
    let events = replay(|answer| {
        if let Some(events) = answer["exact"]["events"].as_array_mut() {
            events
                .iter_mut()
                .for_each(|event| *event = json!(format!("{} ", event.as_str().unwrap())));
        }
    });
    assert!(
        !events.is_empty(),
        "the trace carries events, and their text is compared"
    );
}
