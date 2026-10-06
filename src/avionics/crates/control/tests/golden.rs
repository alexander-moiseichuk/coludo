//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The navigation geometry answers as plan B does: src/avionics/golden/navigation_zone.json, which
//! golden/build_navigation_zone.py builds from src/glider/navigation.py on CPython. Each of its three sections
//! is a golden fixture of its own, with the tolerance its unit needs (stated and justified in the builder).

use control::navigation::{self, Position};
use golden::Fixture;
use serde_json::{Value, json};

/// plan B's distance, zone and zone_aspect over explicit cases and a seeded sweep.
const NAVIGATION_ZONE: &str = include_str!("../../../golden/navigation_zone.json");

/// A section of the fixture as golden fixture text: `{compare, note, cases}`.
fn section(name: &str) -> String {
    Fixture::parse(NAVIGATION_ZONE).get(name).to_string()
}

/// A fixture position, `[latitude_e7, longitude_e7]`.
fn position(value: &Value) -> Position {
    let unit = |index: usize| {
        value[index]
            .as_i64()
            .and_then(|units| i32::try_from(units).ok())
            .unwrap_or_else(|| panic!("a position is two i32 units: {value}"))
    };
    Position {
        latitude_e7: unit(0),
        longitude_e7: unit(1),
    }
}

/// A position as the fixture writes zone points: `[latitude_e7, longitude_e7]`.
fn units(point: Position) -> Value {
    json!([point.latitude_e7, point.longitude_e7])
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(NAVIGATION_ZONE);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], "micropython_plan_B");
    assert_eq!(meta["generator"], "navigation_zone");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/navigation.py"));
}

#[test]
fn distance_matches_plan_b() {
    golden::run(&section("distance"), |input| {
        json!(navigation::distance(position(&input["from"]), position(&input["to"])))
    });
}

#[test]
fn zone_matches_plan_b() {
    golden::run(&section("zone"), |input| {
        let (target, gate_a, gate_b) = navigation::zone(position(&input["top_left"]), position(&input["bottom_right"]));
        json!({"target": units(target), "gate_a": units(gate_a), "gate_b": units(gate_b)})
    });
}

#[test]
fn zone_aspect_matches_plan_b() {
    golden::run(&section("zone_aspect"), |input| {
        json!(navigation::zone_aspect(
            position(&input["top_left"]),
            position(&input["bottom_right"])
        ))
    });
}

#[test]
#[should_panic(expected = "golden cases failed")]
fn a_wrong_answer_fails_the_comparison() {
    // the tolerance is real: a distance 2 cm long fails the 1 cm one
    golden::run(&section("distance"), |input| {
        json!(navigation::distance(position(&input["from"]), position(&input["to"])) + 0.02)
    });
}

#[test]
fn a_gate_one_unit_off_fails_every_zone_case() {
    // The zone tolerance is half a unit, the most the integer centre may round, so a one-unit slip (truncation
    // for rounding, an off-by-one) must fail. A gate always has one coordinate straight from a corner, which
    // plan B and the port share exactly; slipping both coordinates makes every case show it.
    let fixture = Fixture::parse(&section("zone"));
    let compare = fixture.compare();
    for case in fixture.get("cases").as_array().unwrap() {
        let input = &case["input"];
        let (target, gate_a, gate_b) = navigation::zone(position(&input["top_left"]), position(&input["bottom_right"]));
        let name = case["name"].as_str().unwrap();
        let answer = json!({"target": units(target), "gate_a": units(gate_a), "gate_b": units(gate_b)});
        assert_eq!(compare.check(name, &case["output"], &answer), Ok(()));
        let slipped = Position {
            latitude_e7: gate_a.latitude_e7 + 1,
            longitude_e7: gate_a.longitude_e7 + 1,
        };
        let answer = json!({"target": units(target), "gate_a": units(slipped), "gate_b": units(gate_b)});
        assert!(
            compare.check(name, &case["output"], &answer).is_err(),
            "{name}: a one-unit slip passed"
        );
    }
}
