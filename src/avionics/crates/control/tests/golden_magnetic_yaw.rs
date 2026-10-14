//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The magnetic yaw answers as plan B's does: src/avionics/golden/magnetic_yaw.json, which
//! golden/build_magnetic_yaw.py builds from src/glider/tasks/attitude.py (`_magnetic_heading`, `_magnetic_yaw`) on
//! CPython, driving the two methods on their own as test_magnetic_yaw.py does. Each scenario builds a MagneticYaw from
//! its knobs and carries a yaw from `init.yaw_cd` through its calls; `known` compares EXACTLY and the angles within
//! the plan's 0.1 degree, the short way around (see tests/grouped). Inputs are plan B's units: the field in counts,
//! roll_cd, pitch_cd and course_cd / 100 degrees.

mod grouped;

use control::attitude::{MagneticYaw, Settings};
use grouped::Grouped;
use serde_json::{Value, json};

/// plan B's magnetic yaw over test_magnetic_yaw.py's tests and seeded runs.
const MAGNETIC_YAW: &str = include_str!("../../../golden/magnetic_yaw.json");

/// The fixture, loaded.
fn fixture() -> Grouped {
    Grouped::parse(MAGNETIC_YAW)
}

/// A centidegree member of a step, as degrees.
fn degrees(input: &Value, member: &str) -> f32 {
    input[member]
        .as_i64()
        .unwrap_or_else(|| panic!("{member} is a whole number of centidegrees")) as f32
        / 100.0
}

/// The field a step reads, or None for no fresh field.
fn field(input: &Value) -> Option<[f32; 3]> {
    let axes = input["mag"].as_array()?;
    Some([0, 1, 2].map(|axis| axes[axis].as_f64().expect("a field of numbers") as f32))
}

/// Replay one scenario, with the field transformed by `bend` (the identity for the real comparison); every
/// mismatch, as "<case> step <n>: ...".
fn replay(fixture: &Grouped, case: &Value, bend: impl Fn([f32; 3]) -> [f32; 3]) -> Vec<String> {
    let name = case["name"].as_str().expect("a scenario has a name");
    let settings = Settings::from_json(&case["init"]["config"]);
    let mut magnetic = MagneticYaw::new(
        settings.magnetic_shift,
        settings.magnetic_offset_shift,
        settings.magnetic_level,
    );
    let mut yaw = degrees(&case["init"], "yaw_cd");
    let steps = case["steps"].as_array().expect("a scenario has steps");
    assert!(!steps.is_empty(), "{name}: no steps");
    steps
        .iter()
        .enumerate()
        .filter_map(|(index, step)| {
            let input = &step["in"];
            let reading = field(input).map(&bend);
            let (roll, pitch) = (degrees(input, "roll_cd"), degrees(input, "pitch_cd"));
            let answer = match input["action"].as_str() {
                Some("heading") => json!({"degrees": {"heading": magnetic.heading(reading, roll, pitch)}}),
                Some("yaw") => {
                    let tracking = input["tracking"]
                        .as_bool()
                        .expect("tracking is a flag")
                        .then(|| degrees(input, "course_cd"));
                    yaw = magnetic.step(reading, roll, pitch, tracking, yaw);
                    json!({"exact": {"known": magnetic.known()},
                           "degrees": {"yaw": yaw, "offset": magnetic.offset()}})
                }
                other => panic!("unknown action {other:?}"),
            };
            fixture
                .check(&format!("{name} step {index}"), &step["out"], &answer)
                .err()
        })
        .collect()
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = fixture();
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "magnetic_yaw");
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
}

#[test]
fn the_knobs_resolve_as_plan_b_setup() {
    let fixture = fixture();
    for case in fixture.cases("scenarios") {
        let settings = Settings::from_json(&case["init"]["config"]);
        let owed = &case["init"]["settings"];
        assert_eq!(owed["mag_shift"], settings.magnetic_shift, "{}", case["name"]);
        assert_eq!(owed["mag_offset_shift"], settings.magnetic_offset_shift);
        assert_eq!(owed["mag_level_cd"], (settings.magnetic_level * 100.0).round() as i64);
    }
}

#[test]
fn every_call_matches_plan_b() {
    let fixture = fixture();
    let cases = fixture.cases("scenarios");
    let failures: Vec<String> = cases
        .iter()
        .flat_map(|case| replay(&fixture, case, |axes| axes))
        .collect();
    let steps: usize = cases.iter().map(|case| case["steps"].as_array().unwrap().len()).sum();
    assert!(steps > 500, "only {steps} calls compared");
    assert!(
        failures.is_empty(),
        "{} of {steps} magnetic yaw calls differ:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

#[test]
fn a_mirrored_field_fails() {
    // NEGATIVE: atan2(my, mx) for plan B's atan2(-my, mx) -- the sign the magnetic frame turns on -- misses at once
    let fixture = fixture();
    let case = fixture
        .cases("scenarios")
        .iter()
        .find(|case| case["name"] == "heading_comes_from_the_horizontal_axes")
        .unwrap();
    let failures = replay(&fixture, case, |[x, y, z]| [x, -y, z]);
    assert_eq!(
        failures.len(),
        1,
        "only the field along +y reads differently: {failures:?}"
    );
    let seeded: usize = fixture
        .cases("scenarios")
        .iter()
        .filter(|case| case["name"].as_str().unwrap().starts_with("seeded_"))
        .map(|case| replay(&fixture, case, |[x, y, z]| [x, -y, z]).len())
        .sum();
    assert!(seeded > 100, "{seeded}");
}
