//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The mixer answers as plan B's does: src/avionics/golden/mixer.json, which golden/build_mixer.py builds from
//! src/glider/mixer.py (and sg90's rounding) on CPython. Compare exact: every input is dyadic, so every sum is
//! exact on both sides and the whole-degree angles must agree to the degree, ties included.

use control::mixer::{Fin, Mixer};
use golden::Fixture;
use serde_json::{Map, Value, json};

/// plan B's mix() and bind()/actuate() over test_mixer.py's cases, bench_cases.py's, the rounding and a sweep.
const MIXER: &str = include_str!("../../../golden/mixer.json");

/// A section of the fixture as golden fixture text: `{compare, note, cases}`.
fn section(name: &str) -> String {
    Fixture::parse(MIXER).get(name).to_string()
}

/// A command's axis value as f32.
fn axis(command: &Value, index: usize) -> f32 {
    command[index].as_f64().expect("a command is three numbers") as f32
}

/// The angles a mixer answers, keyed by surface name.
fn named(mixer: &Mixer, angles: &[i32]) -> Value {
    let map: Map<String, Value> = mixer
        .surfaces()
        .iter()
        .zip(angles)
        .map(|(surface, angle)| (surface.name.clone(), json!(angle)))
        .collect();
    Value::Object(map)
}

/// A fin that records its last command (plan B's test `_Fin`).
struct RecordingFin {
    /// The last commanded angle, None before the first.
    angle: Option<i32>,
}

impl Fin for RecordingFin {
    fn set_angle(&mut self, angle: i32) {
        self.angle = Some(angle);
    }

    fn angle(&self) -> Option<i32> {
        self.angle
    }
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let meta = Fixture::parse(MIXER).get("meta").clone();
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "mixer");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/mixer.py"));
}

#[test]
fn mix_matches_plan_b() {
    golden::run(&section("mix"), |input| {
        let mut mixer = Mixer::new(&input["config"]);
        let command = &input["command"];
        let angles = mixer.mix(axis(command, 0), axis(command, 1), axis(command, 2)).to_vec();
        named(&mixer, &angles)
    });
}

#[test]
fn actuate_matches_plan_b() {
    golden::run(&section("actuate"), |input| {
        let mut mixer = Mixer::new(&input["config"]);
        let bound: Vec<&str> = input["bound"]
            .as_array()
            .unwrap()
            .iter()
            .map(|name| name.as_str().unwrap())
            .collect();
        let mut fins: Vec<Option<RecordingFin>> = mixer
            .surfaces()
            .iter()
            .map(|surface| {
                bound
                    .contains(&surface.name.as_str())
                    .then_some(RecordingFin { angle: None })
            })
            .collect();
        for command in input["commands"].as_array().unwrap() {
            mixer.actuate(&mut fins, axis(command, 0), axis(command, 1), axis(command, 2));
        }
        let mut missing: Vec<&str> = mixer.missing(&fins);
        missing.sort_unstable();
        // plan B's operator view, Mixer.angles(): each bound fin's stored command, None if never commanded
        let angles: Map<String, Value> = mixer
            .angles(&fins)
            .into_iter()
            .map(|(name, angle)| (name.to_string(), json!(angle)))
            .collect();
        json!({"missing": missing, "angles": angles})
    });
}

#[test]
#[should_panic(expected = "golden cases failed")]
fn rounding_half_away_from_zero_fails_the_ties() {
    // NEGATIVE: f32::round (half away from zero) is not plan B's round(); the tie cases must catch it
    golden::run(&section("mix"), |input| {
        let mixer = Mixer::new(&input["config"]);
        let command = &input["command"];
        let (roll, pitch, yaw) = (axis(command, 0), axis(command, 1), axis(command, 2));
        let limit = mixer.limit() as f32;
        let angles: Vec<i32> = mixer
            .surfaces()
            .iter()
            .map(|surface| {
                let deflection = (surface.roll * roll + surface.pitch * pitch + surface.yaw * yaw).clamp(-limit, limit);
                (mixer.neutral() as f32 + deflection).round() as i32
            })
            .collect();
        named(&mixer, &angles)
    });
}
