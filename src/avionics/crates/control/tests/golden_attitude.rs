//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The attitude filter answers as plan B's does: src/avionics/golden/attitude.json, which golden/build_attitude.py
//! builds from src/glider/tasks/attitude.py on CPython. Each scenario sets a filter up from its config at time 0 and
//! replays its ticks in order; every tick's flags (free, blind, seeded, mag_known, the logged line) compare EXACTLY
//! and its angles (the state, the magnetic offset, the published attitude) within the plan's 0.1 degree, the short
//! way around (see tests/grouped). Inputs are plan B's integer units, read here exactly as the builder states:
//! rate_cd_s / 100 degrees per second, accel_cg / 100 g, course_cd / 100 degrees, the fused value / 100 degrees.
//!
//! Except the angles of the [`KINEMATIC`] scenarios: the port's kinematics part from plan B's by design (2026-10-14),
//! and there only the flags compare.

mod grouped;

use control::attitude::{Attitude, Fused, Orientation, Readings, Settings, Tick};
use grouped::Grouped;
use serde_json::{Value, json};

/// plan B's filter over test_attitude.py's blocks, the magnetic yaw, the porting corners, config_default and
/// seeded flights.
const ATTITUDE: &str = include_str!("../../../golden/attitude.json");

/// The scenarios whose ANGLES part from plan B by design (crates/control attitude `rotation`, 2026-10-14): plan B added
/// the gyro's body rates to the Euler angles and blended roll and pitch apart and unwrapped, where the port turns a
/// quaternion and tilts the shortest way. The two agree to the fixture's 0.1 degree for a turn about one axis from
/// level and a pull on one angle; these scenarios turn about several axes at once (the seeded flights, the long free
/// run), turn about a banked yaw axis, pull roll and pitch together, or pull across +-180 (169 degrees apart on the
/// long free run, 4.4 on a seeded flight). Their flags, blind and seeded states, magnetic latch and log lines still
/// compare exactly, every tick; their angles are the unit tests' (attitude/rotation.rs and attitude/tests.rs: a
/// banked turn, a spin at a vertical nose, the short way round, the long single-axis run).
const KINEMATIC: [&str; 13] = [
    "a_long_free_run_is_exact",
    "seeded_flight_00",
    "seeded_flight_01",
    "seeded_flight_02",
    "seeded_flight_03",
    "seeded_flight_04",
    "seeded_flight_05",
    "free_run_integrates_the_gyro",
    "a_silent_primary_hands_over_to_the_gyro",
    "the_gravity_band_edges",
    "the_accel_pull_goes_the_long_way_round",
    "the_turn_gate_holds_the_bank",
    "the_accel_correction_is_throttled",
];

/// The fixture, loaded.
fn fixture() -> Grouped {
    Grouped::parse(ATTITUDE)
}

/// A fixture vector of plan B integers, divided into the port's unit (`scale` units per unit); None for null.
fn vector(value: &Value, scale: f32) -> Option<[f32; 3]> {
    let items = value.as_array()?;
    assert_eq!(items.len(), 3, "a vector has three axes: {value}");
    Some([0, 1, 2].map(|axis| items[axis].as_f64().expect("a vector of numbers") as f32 / scale))
}

/// The readings a step's `in` describes.
fn readings(input: &Value) -> Readings {
    let fused = (!input["fused"].is_null()).then(|| {
        let [heading, roll, pitch] = vector(&input["fused"]["value"], 100.0).expect("a fused value");
        let rank = input["fused"]["rank"].as_i64().expect("a fused rank");
        Fused {
            orientation: Orientation { heading, roll, pitch },
            rank: i32::try_from(rank).expect("a rank within i32"),
        }
    });
    Readings {
        rate: vector(&input["rate_cd_s"], 100.0),
        acceleration: vector(&input["accel_cg"], 100.0),
        course: input["course_cd"].as_f64().map(|course| course as f32 / 100.0),
        ground_speed: input["speed"].as_f64().map(|speed| speed as f32),
        magnetic: vector(&input["mag"], 1.0),
        fused,
        backup_fresh: input["backup"].as_bool().expect("backup is a flag"),
        ballistic: false,
        resting: false,
        airspeed: None,
    }
}

/// A step's time in microseconds after setup.
fn now_us(input: &Value) -> u64 {
    input["now_ms"].as_u64().expect("now_ms") * 1000
}

/// The filter's answers after a tick, grouped as the fixture writes them.
fn answers(filter: &Attitude, tick: &Tick) -> Value {
    let state = filter.orientation();
    let published = tick.published.map_or(Value::Null, |orientation| {
        json!([orientation.heading, orientation.roll, orientation.pitch])
    });
    json!({
        "exact": {
            "free": filter.free(),
            "blind": filter.blind(),
            "seeded": filter.seeded(),
            "mag_known": filter.magnetic().known(),
            "log": tick.log,
        },
        "degrees": {
            "state": [state.heading, state.roll, state.pitch],
            "mag_offset": filter.magnetic().offset(),
            "published": published,
        },
    })
}

/// The largest angle difference, in degrees the short way around, between plan B's `degrees` group and the port's
/// (0 where either side is not a number: the structure is the comparison's business).
fn worst(owed: &Value, answered: &Value) -> f64 {
    match (owed, answered) {
        (Value::Array(owed), Value::Array(answered)) => owed
            .iter()
            .zip(answered)
            .map(|(owed, answered)| worst(owed, answered))
            .fold(0.0, f64::max),
        (Value::Object(owed), Value::Object(answered)) => owed
            .iter()
            .map(|(key, owed)| answered.get(key).map_or(0.0, |answered| worst(owed, answered)))
            .fold(0.0, f64::max),
        (Value::Number(owed), Value::Number(answered)) => around(owed.as_f64().unwrap(), answered.as_f64().unwrap()),
        _ => 0.0,
    }
}

/// Replay one scenario on a filter built from `config`: every mismatch, as "<case> step <n>: ...", the worst
/// angle difference from plan B in degrees, and the filter as the last step left it.
fn replay_measured(fixture: &Grouped, case: &Value, config: &Value) -> (Vec<String>, f64, Attitude) {
    replay_adjusted(fixture, case, config, |readings| readings)
}

/// [`replay_measured`] with every tick's readings passed through `adjust` first: a negative test's way of feeding
/// the filter something slightly off what plan B was fed.
fn replay_adjusted(
    fixture: &Grouped,
    case: &Value,
    config: &Value,
    adjust: impl Fn(Readings) -> Readings,
) -> (Vec<String>, f64, Attitude) {
    let name = case["name"].as_str().expect("a scenario has a name");
    let mut filter = Attitude::new(Settings::from_json(config), 0);
    let steps = case["steps"].as_array().expect("a scenario has steps");
    assert!(!steps.is_empty(), "{name}: no steps");
    let mut largest = 0.0_f64;
    let failures = steps
        .iter()
        .enumerate()
        .filter_map(|(index, step)| {
            let input = &step["in"];
            let tick = filter.tick(now_us(input), &adjust(readings(input)));
            let answer = answers(&filter, &tick);
            if KINEMATIC.contains(&name) {
                let flags = |out: &Value| json!({"exact": out["exact"]});
                return fixture
                    .check(&format!("{name} step {index}"), &flags(&step["out"]), &flags(&answer))
                    .err();
            }
            largest = largest.max(worst(&step["out"]["degrees"], &answer["degrees"]));
            fixture
                .check(&format!("{name} step {index}"), &step["out"], &answer)
                .err()
        })
        .collect();
    (failures, largest, filter)
}

/// Replay one scenario on a filter built from `config`; every mismatch, and the filter as the last step left it.
fn replay(fixture: &Grouped, case: &Value, config: &Value) -> (Vec<String>, Attitude) {
    let (failures, _, filter) = replay_measured(fixture, case, config);
    (failures, filter)
}

/// Every scenario's failures, each replayed on its own config.
fn every_failure(fixture: &Grouped) -> Vec<String> {
    fixture
        .cases("scenarios")
        .iter()
        .flat_map(|case| replay(fixture, case, &case["init"]["config"]).0)
        .collect()
}

/// The scenario of this name.
fn scenario<'fixture>(fixture: &'fixture Grouped, name: &str) -> &'fixture Value {
    fixture
        .cases("scenarios")
        .iter()
        .find(|case| case["name"] == name)
        .unwrap_or_else(|| panic!("no scenario {name}"))
}

/// An inspect angle, text to the hundredth as plan B prints it, as degrees.
fn inspected(value: &Value) -> f64 {
    let text = value.as_str().expect("an inspect angle is text");
    let (whole, fraction) = text.split_once('.').expect("a decimal point");
    assert_eq!(fraction.len(), 2, "two decimals: {text}");
    assert!(
        whole.trim_start_matches('-').bytes().all(|byte| byte.is_ascii_digit()),
        "{text}"
    );
    text.parse().expect("a decimal")
}

/// How far apart two angles are in degrees, the short way around.
fn around(first: f64, second: f64) -> f64 {
    let difference = (first - second).rem_euclid(360.0);
    difference.min(360.0 - difference)
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = fixture();
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "attitude");
    assert!(
        meta["source"]
            .as_str()
            .unwrap()
            .contains("src/glider/tasks/attitude.py")
    );
    assert_eq!(
        fixture.get("compare")["degrees"],
        json!({"kind": "angle_deg", "tol": 0.1})
    );
    assert_eq!(fixture.get("compare")["exact"], json!({"kind": "exact"}));
}

#[test]
fn the_settings_resolve_as_plan_b_setup() {
    let fixture = fixture();
    for case in fixture.cases("scenarios") {
        let settings = Settings::from_json(&case["init"]["config"]);
        let owed = &case["init"]["settings"];
        let centi = |value: f32| (value * 100.0).round() as i64;
        let answered = json!({
            "period_ms": settings.period_ms,
            "accel_period_us": settings.acceleration_period_ms * 1000,
            "corr_shift": settings.correction_shift,
            "turn_gate_cd_s": centi(settings.turn_gate),
            "course_gate_mps": owed["course_gate_mps"].as_f64().filter(|gate| *gate as f32 == settings.course_gate),
            "mag_shift": settings.magnetic_shift,
            "mag_offset_shift": settings.magnetic_offset_shift,
            "mag_level_cd": centi(settings.magnetic_level),
            "course_shift": settings.course_shift,
            "grav_low_sq_cg2": centi(settings.gravity_low).pow(2),
            "grav_high_sq_cg2": centi(settings.gravity_high).pow(2),
            "rank": settings.rank,
            "telemetry_decimate_us": settings.telemetry_ms * 1000,
        });
        assert_eq!(&answered, owed, "{}", case["name"]);
    }
}

#[test]
fn every_tick_matches_plan_b() {
    let fixture = fixture();
    let mut failures = Vec::new();
    let mut largest = 0.0_f64;
    for case in fixture.cases("scenarios") {
        let (missed, worst, _) = replay_measured(&fixture, case, &case["init"]["config"]);
        failures.extend(missed);
        largest = largest.max(worst);
    }
    let steps: usize = fixture
        .cases("scenarios")
        .iter()
        .map(|case| case["steps"].as_array().unwrap().len())
        .sum();
    assert!(steps > 1000, "only {steps} ticks compared");
    assert!(
        failures.is_empty(),
        "{} of {steps} attitude ticks differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
    eprintln!("attitude: worst |port - plan B| {largest:.5} degrees over {steps} ticks");
}

#[test]
fn every_scenario_ends_inspecting_as_plan_b() {
    let fixture = fixture();
    for case in fixture.cases("scenarios") {
        let name = case["name"].as_str().unwrap();
        let (_, filter) = replay(&fixture, case, &case["init"]["config"]);
        let owed = case["inspect"].as_object().expect("an inspect object");
        let answered = filter.inspect();
        let answered = answered.as_object().unwrap();
        let mut owed_keys: Vec<&String> = owed.keys().collect();
        let mut answered_keys: Vec<&String> = answered.keys().collect();
        owed_keys.sort();
        answered_keys.sort();
        assert_eq!(owed_keys, answered_keys, "{name}");
        let kinematic = KINEMATIC.contains(&name);
        for (key, value) in owed {
            if value.is_boolean() {
                assert_eq!(&answered[key], value, "{name} {key}");
            } else if !kinematic {
                let error = around(inspected(value), inspected(&answered[key]));
                assert!(error <= 0.1, "{name} {key}: plan B {value}, the port {}", answered[key]);
            }
        }
        // the attitude.csv row is the same state in integer centidegrees and 0/1 flags
        let row = filter.row();
        assert_eq!(row[3], i32::from(owed["free_running"] == true), "{name}");
        assert_eq!(row[4], i32::from(owed["mag_known"] == true), "{name}");
        if kinematic {
            continue;
        }
        let centidegrees = |key: &str| inspected(&owed[key]) * 100.0;
        assert!(
            around(f64::from(row[0]) / 100.0, centidegrees("heading") / 100.0) <= 0.1,
            "{name} {row:?}"
        );
        assert!(
            (f64::from(row[1]) - centidegrees("roll")).abs() <= 10.0,
            "{name} {row:?}"
        );
        assert!(
            (f64::from(row[2]) - centidegrees("pitch")).abs() <= 10.0,
            "{name} {row:?}"
        );
        assert!(
            around(f64::from(row[5]) / 100.0, centidegrees("mag_offset") / 100.0) <= 0.1,
            "{name} {row:?}"
        );
    }
}

#[test]
fn a_wrong_pull_fails() {
    // NEGATIVE: the 0.1 degree tolerance is not a blanket -- corr_shift 3 for 2 misses the gravity pull at once
    let fixture = fixture();
    let case = scenario(&fixture, "the_accel_pulls_toward_gravity");
    let mut config = case["init"]["config"].clone();
    config["corr_shift"] = json!(3);
    let (failures, _) = replay(&fixture, case, &config);
    assert!(failures.len() > 30, "{} failures", failures.len());
    // (a turn gate too loose, which pulls a coordinated turn flat: attitude/tests.rs the_turn_gate_holds_the_bank,
    // its scenario's angles being kinematic here)
}

#[test]
fn a_flag_or_a_line_one_tick_off_fails() {
    // NEGATIVE: the flags are exact -- a blind window of four or six ticks would fail a scripted scenario
    let fixture = fixture();
    let case = scenario(&fixture, "blind_without_a_backup_keeps_publishing");
    let blind_steps: Vec<usize> = case["steps"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
        .filter(|(_, step)| step["out"]["exact"]["log"] == "gyro lost -- attitude filter blind")
        .map(|(index, _)| index)
        .collect();
    assert_eq!(blind_steps, [5, 13], "blind on the fifth gyro-less tick, each time");
    let mut text = ATTITUDE.to_string();
    let marker = "\"log\": \"gyro lost -- attitude filter blind\"";
    let at = text.find(marker).expect("a logged blind transition");
    text.replace_range(at..at + marker.len(), "\"log\": null");
    let corrupted = Grouped::parse(&text);
    assert!(!every_failure(&corrupted).is_empty());
}

#[test]
fn the_withhold_hands_over_and_never_mirrors_a_backup() {
    // the scripted withhold: published, then withheld from the fifth gyro-less tick, back once the backup is stale
    let fixture = fixture();
    let case = scenario(&fixture, "blind_with_a_fresh_backup_withholds");
    let published: Vec<bool> = case["steps"]
        .as_array()
        .unwrap()
        .iter()
        .map(|step| !step["out"]["degrees"]["published"].is_null())
        .collect();
    assert_eq!(
        published,
        [true, true, true, true, true, false, false, false, true, true, true]
    );
    let case = scenario(&fixture, "never_mirror_a_backup_or_a_peer");
    for step in case["steps"].as_array().unwrap() {
        assert_eq!(step["out"]["exact"]["seeded"], false, "a backup or a peer was mirrored");
    }
}

#[test]
fn the_fixture_pins_the_orders_plan_b_keeps() {
    let fixture = fixture();
    let outs = |name: &str| -> Vec<Value> {
        scenario(&fixture, name)["steps"]
            .as_array()
            .unwrap()
            .iter()
            .map(|step| step["out"].clone())
            .collect()
    };
    // the level gate reads the roll BEFORE the gyro step: the tick from 14.5 to 15.5 learns, the next refuses
    let gate = outs("the_level_gate_reads_the_roll_before_the_gyro_step");
    assert_eq!(gate[1]["exact"]["mag_known"], true);
    assert_eq!(gate[1]["degrees"]["state"][1], 15.5);
    assert_eq!(gate[2]["degrees"]["mag_offset"], gate[1]["degrees"]["mag_offset"]);
    // the track needs the course AND the speed: six ticks with one of them alone hold the heading and learn nothing
    let track = outs("the_track_needs_the_course_and_the_speed");
    for out in &track[1..7] {
        assert_eq!(out["degrees"]["state"][0], 45.0);
        assert_eq!(out["exact"]["mag_known"], false);
    }
    assert_eq!(track[7]["exact"]["mag_known"], true);
    // plan B's accelerometer pull went the long way round: from roll 170 toward gravity at -170 it landed near 85 (the
    // port's takes the short way: attitude/tests.rs the_accel_pull_takes_the_short_way_round)
    let long_way = outs("the_accel_pull_goes_the_long_way_round");
    let roll = long_way[1]["degrees"]["published"][1].as_f64().unwrap();
    assert!((roll - 85.0).abs() < 0.1, "{roll}");
}

#[test]
fn the_kinematic_scenarios_are_the_ones_that_turn_or_pull_off_one_axis() {
    // NEGATIVE: every other scenario still holds the plan's 0.1 degree: the list is the deviation, no wider
    let fixture = fixture();
    let names: Vec<&str> = fixture
        .cases("scenarios")
        .iter()
        .map(|case| case["name"].as_str().unwrap())
        .collect();
    for kinematic in KINEMATIC {
        assert!(names.contains(&kinematic), "{kinematic}: no such scenario");
    }
    // the other 17 of the 30 (mirroring, blind, withheld, gaps, the track and the magnetometer, the level gate, a
    // single-axis pull, config_default's pad and slow roll) compare every angle, every tick
    assert_eq!(names.len() - KINEMATIC.len(), 17, "{names:?}");
}
