//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight law answers as plan B's flight task does: src/avionics/golden/control_trace.json, which
//! golden/build_control_trace.py builds by replaying a closed-loop tools/virtual_flight.py flight's per-tick inputs
//! through plan B's own tasks/flight.py, open-loop. The same inputs go through [`FlightLaw::step`] here, on the same
//! clock (the shims' 10 s boot uptime, one 10 ms slice per tick).
//!
//! Compared on every tick: the fins and the fin cap within 1 degree, the airspeed within 0.1 m/s. On every 10th tick
//! (the flight.csv state): active and reckoning exactly, the setpoints, the heading error and the per-axis demands
//! within 1 degree, the wind speed within 0.1 m/s and its direction within 2 degrees once it blows over 0.5 m/s.
//! Plan B ran the law in centidegree integers and doubles, the port in f32 degrees, so a rare whole-degree flip at a
//! rounding edge is the tolerance's reason, not a law difference.

use control::flight::{FlightLaw, Step};
use control::governor::Readings;
use control::guidance::{Fix, Heading, Inputs, Landing};
use control::navigation::Position;
use sequencer::Stage;
use serde_json::Value;

/// Plan B's flight task over two closed-loop flights' inputs.
const TRACE: &str = include_str!("../../../golden/control_trace.json");

/// The shims' uptime when a builder starts (golden/shims.py `_BOOT_UPTIME_US`).
const BOOT_UPTIME_US: u64 = 10_000_000;

/// The GNSS channels' window the builder's databoard gave the guidance: the larger of position's 400 ms and speed's
/// 500 ms (golden/build_control_trace.py, plan B tasks/flight.py).
const GNSS_WINDOW_MS: u32 = 500;

/// One packed field: None when empty, else its comma-separated numbers.
fn numbers(field: &str) -> Option<Vec<f64>> {
    (!field.is_empty()).then(|| {
        field
            .split(',')
            .map(|number| number.parse::<f64>().expect("a number"))
            .collect()
    })
}

/// The one number of a packed field.
fn scalar(field: &str) -> Option<f32> {
    numbers(field).map(|values| values[0] as f32)
}

/// The three numbers of a packed field.
fn vector(field: &str) -> Option<[f32; 3]> {
    numbers(field).map(|values| [values[0] as f32, values[1] as f32, values[2] as f32])
}

/// A degree pair as a position.
fn position(value: &Value) -> Position {
    Position::from_degrees(
        value[0].as_f64().expect("a latitude"),
        value[1].as_f64().expect("a longitude"),
    )
}

/// The mismatches of one case, as text (empty when it answers as plan B).
fn replay(case: &Value) -> Vec<String> {
    let name = case["name"].as_str().expect("a name");
    let input = &case["input"];
    let mut law = FlightLaw::new(&input["flight"], &input["fins"], GNSS_WINDOW_MS);
    let zone = &input["zone"];
    let landing = Landing::new(
        position(&zone[0]),
        position(&zone[1]),
        Heading::resolve(input["endgame_heading"].as_str().expect("a heading")),
    );
    let period_us = 1_000_000 / input["hz"].as_u64().expect("hz");
    let ticks = input["ticks"].as_array().expect("ticks");
    let outputs = case["output"]["ticks"].as_array().expect("output ticks");
    let samples = case["output"]["samples"].as_array().expect("samples");
    let mut mismatches = Vec::new();
    let mut course: Option<f32> = None;
    let mut ground_stamp: Option<u64> = None;
    let mut sample = samples.iter().peekable();
    for (index, tick) in ticks.iter().enumerate() {
        let fields: Vec<&str> = tick.as_str().expect("a packed tick").split(';').collect();
        let speed = scalar(fields[4]);
        if let Some(sampled) = scalar(fields[6]) {
            course = Some(sampled);
            ground_stamp = Some(ground_stamp.map_or(1, |stamp| stamp + 1));
        }
        let fix = numbers(fields[5]).map(|degrees| Fix {
            position: Position::from_degrees(degrees[0], degrees[1]),
            age_ms: 0,
        });
        let step = Step {
            stage: Stage::from_id(fields[0].parse().expect("a stage id")).expect("a stage"),
            armed: true,
            now_us: BOOT_UPTIME_US + (index as u64 + 1) * period_us,
            attitude: vector(fields[2]),
            rate: vector(fields[3]),
            readings: Readings {
                acceleration: scalar(fields[1]).map(|accel| [0.0, 0.0, accel]),
                gnss_speed: speed,
                pitot: scalar(fields[7]),
            },
            guidance: Inputs {
                landing: Some(landing),
                launch: None,
                position: fix,
                agl: scalar(fields[8]),
                elevation: scalar(fields[9]),
                ..Inputs::default()
            },
            ground: course.zip(speed).map(|(course, speed)| [course, speed]),
            ground_stamp,
            drift: [0.0; 2],
        };
        // the fixture lists the fins sorted by surface name; take the law's in that order, whatever the mixer's
        law.step(&step);
        let mut named: Vec<(String, i32)> = law
            .angles()
            .to_vec()
            .into_iter()
            .zip(law.mixer().surfaces())
            .map(|(angle, surface)| (surface.name.clone(), angle))
            .collect();
        named.sort();
        let angles: Vec<i32> = named.into_iter().map(|(_, angle)| angle).collect();
        let expected: Vec<&str> = outputs[index].as_str().expect("a packed output").split(';').collect();
        let fins: Vec<i32> = expected[0]
            .split(',')
            .map(|angle| angle.parse().expect("an angle"))
            .collect();
        let cap: i32 = expected[1].parse().expect("a cap");
        let airspeed: f32 = expected[2].parse().expect("an airspeed");
        let mut differ = |what: &str, got: String, want: String| {
            mismatches.push(format!("{name} tick {index}: {what} {got}, plan B {want}"));
        };
        if angles.iter().zip(&fins).any(|(got, want)| (got - want).abs() > 1) {
            differ("fins", format!("{angles:?}"), format!("{fins:?}"));
        }
        if (law.cap() - cap).abs() > 1 {
            differ("cap", law.cap().to_string(), cap.to_string());
        }
        if (law.airspeed() - airspeed).abs() > 0.1 {
            differ("airspeed", law.airspeed().to_string(), airspeed.to_string());
        }
        if sample.peek().is_some_and(|packed| {
            packed
                .as_str()
                .expect("a packed sample")
                .starts_with(&format!("{index};"))
        }) {
            let packed = sample.next().expect("the sample").as_str().expect("a packed sample");
            let fields: Vec<&str> = packed.split(';').collect();
            let guidance = law.guidance();
            let active = fields[1] == "1";
            let roll_setpoint: f32 = fields[2].parse().expect("roll_sp");
            let pitch_setpoint: f32 = fields[3].parse().expect("pitch_sp");
            let heading_error: i32 = fields[4].parse().expect("heading_err");
            let commands: Vec<i32> = fields[5]
                .split(',')
                .map(|command| command.parse().expect("a command"))
                .collect();
            let wind: Vec<f32> = fields[6]
                .split(',')
                .map(|value| value.parse().expect("a wind"))
                .collect();
            let reckoning = fields[7] == "1";
            if law.active() != active {
                differ("active", law.active().to_string(), active.to_string());
            }
            if (guidance.roll_setpoint() - roll_setpoint).abs() > 1.0
                || (guidance.pitch_setpoint() - pitch_setpoint).abs() > 1.0
            {
                differ(
                    "setpoints",
                    format!("{} {}", guidance.roll_setpoint(), guidance.pitch_setpoint()),
                    format!("{roll_setpoint} {pitch_setpoint}"),
                );
            }
            if (guidance.heading_error() - heading_error).abs() > 1 {
                differ(
                    "heading_err",
                    guidance.heading_error().to_string(),
                    heading_error.to_string(),
                );
            }
            if law
                .commands()
                .iter()
                .zip(&commands)
                .any(|(got, want)| (got - want).abs() > 1)
            {
                differ("commands", format!("{:?}", law.commands()), format!("{commands:?}"));
            }
            let (speed, from) = (law.wind().speed(), law.wind().direction());
            let turned = (from - wind[1] + 540.0).rem_euclid(360.0) - 180.0;
            if (speed - wind[0]).abs() > 0.1 || (wind[0] > 0.5 && turned.abs() > 2.0) {
                differ(
                    "wind",
                    format!("{speed} from {from}"),
                    format!("{} from {}", wind[0], wind[1]),
                );
            }
            if guidance.reckoning() != reckoning {
                differ("reckoning", guidance.reckoning().to_string(), reckoning.to_string());
            }
        }
    }
    mismatches
}

#[test]
fn the_flight_law_answers_as_plan_b() {
    let fixture: Value = serde_json::from_str(TRACE).expect("the fixture parses");
    let cases = fixture["cases"].as_array().expect("cases");
    assert!(!cases.is_empty());
    let mismatches: Vec<String> = cases.iter().flat_map(replay).collect();
    assert!(
        mismatches.is_empty(),
        "{} mismatching ticks; the first:\n{}",
        mismatches.len(),
        mismatches.iter().take(25).cloned().collect::<Vec<_>>().join("\n")
    );
}

#[test]
fn the_law_actuates_nothing_on_the_pad_until_it_engages() {
    let fixture: Value = serde_json::from_str(TRACE).expect("the fixture parses");
    let input = &fixture["cases"][0]["input"];
    let mut law = FlightLaw::new(&input["flight"], &input["fins"], GNSS_WINDOW_MS);
    let step = |stage: Stage, armed: bool, now_us: u64| Step {
        stage,
        armed,
        now_us,
        attitude: Some([30.0, 5.0, -6.0]),
        rate: Some([0.0; 3]),
        readings: Readings::default(),
        guidance: Inputs::default(),
        ground: None,
        ground_stamp: None,
        drift: [0.0; 2],
    };
    let neutral = law.angles().to_vec();
    // on the pad, armed or not, a never-engaged law writes no fin: an operator's command stands
    assert!(!law.step(&step(Stage::Setting, false, 10_000)));
    assert!(!law.step(&step(Stage::Setting, true, 20_000)));
    // gliding and armed it controls; disarmed in the glide it centres once, then holds off
    assert!(law.step(&step(Stage::Gliding, true, 30_000)));
    assert!(law.active());
    assert!(
        law.step(&step(Stage::Gliding, false, 40_000)),
        "leaving control centres the fins"
    );
    assert_eq!(law.angles(), neutral.as_slice());
    assert!(!law.active());
    assert!(!law.step(&step(Stage::Gliding, false, 50_000)), "then nothing");
    // engaged without a fresh attitude: neutral, every tick
    assert!(law.step(&step(Stage::Gliding, true, 60_000)));
    let blind = Step {
        attitude: None,
        ..step(Stage::Gliding, true, 70_000)
    };
    assert!(law.step(&blind));
    assert_eq!(law.angles(), neutral.as_slice());
}
