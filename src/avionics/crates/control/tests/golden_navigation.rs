//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The navigation geometry answers as plan B does: src/avionics/golden/navigation.json, which
//! golden/build_navigation.py builds from src/glider/navigation.py on CPython (offset, advance, compass,
//! bearing/distance/range_bearing, inside, steer/steer_to, cross_track, approach/approach_to). Each answer is
//! grouped by unit, and each group compares under the tolerance the fixture states and justifies (see
//! tests/grouped).

mod grouped;

use control::navigation::{self, Position};
use grouped::{Grouped, number};
use serde_json::{Value, json};

/// plan B's navigation over explicit cases and seeded sweeps.
const NAVIGATION: &str = include_str!("../../../golden/navigation.json");

/// The fixture, loaded.
fn fixture() -> Grouped {
    Grouped::parse(NAVIGATION)
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

/// A position as the fixture writes one, `[latitude_e7, longitude_e7]`.
fn units(point: Position) -> Value {
    json!([point.latitude_e7, point.longitude_e7])
}

/// The steer answer of `steering`, grouped as the fixture writes it.
fn steering(steering: navigation::Steering) -> Value {
    json!({"degrees": steering.bearing, "exact": {"waypoint": units(steering.waypoint), "leg": steering.leg as u8}})
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let meta = fixture().get("meta").clone();
    assert_eq!(meta["tag"], "micropython_plan_B_2");
    assert_eq!(meta["generator"], "navigation");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/navigation.py"));
}

#[test]
fn offset_matches_plan_b() {
    fixture().run("offset", |input| {
        let (east, north) = navigation::offset(position(&input["from"]), position(&input["to"]));
        json!({"metres": [east, north]})
    });
}

#[test]
fn range_bearing_matches_plan_b() {
    fixture().run("range_bearing", |input| {
        let (distance, bearing) = navigation::range_bearing(position(&input["from"]), position(&input["to"]));
        json!({"metres": distance, "degrees": bearing})
    });
}

#[test]
fn distance_and_bearing_answer_what_range_bearing_does() {
    fixture().run("range_bearing", |input| {
        let (from, to) = (position(&input["from"]), position(&input["to"]));
        json!({"metres": navigation::distance(from, to), "degrees": navigation::bearing(from, to)})
    });
}

#[test]
fn advance_matches_plan_b() {
    fixture().run("advance", |input| {
        let moved = navigation::advance(
            position(&input["position"]),
            number(input, "east"),
            number(input, "north"),
        );
        json!({"units": units(moved)})
    });
}

#[test]
fn compass_matches_plan_b() {
    fixture().run(
        "compass",
        |input| json!({"degrees": navigation::compass(number(input, "east"), number(input, "north"))}),
    );
}

#[test]
fn inside_matches_plan_b() {
    fixture().run("inside", |input| {
        json!({"exact": navigation::inside(
            position(&input["position"]),
            position(&input["top_left"]),
            position(&input["bottom_right"]),
        )})
    });
}

#[test]
fn steer_matches_plan_b() {
    fixture().run("steer", |input| {
        steering(navigation::steer(
            position(&input["position"]),
            position(&input["top_left"]),
            position(&input["bottom_right"]),
        ))
    });
}

#[test]
fn steer_to_answers_what_steer_does() {
    fixture().run("steer", |input| {
        let (top_left, bottom_right) = (position(&input["top_left"]), position(&input["bottom_right"]));
        let (target, gate_a, gate_b) = navigation::zone(top_left, bottom_right);
        steering(navigation::steer_to(
            position(&input["position"]),
            top_left,
            bottom_right,
            target,
            gate_a,
            gate_b,
        ))
    });
}

#[test]
fn cross_track_matches_plan_b() {
    fixture().run("cross_track", |input| {
        json!({"metres": navigation::cross_track(
            position(&input["position"]),
            position(&input["point"]),
            number(input, "heading"),
        )})
    });
}

#[test]
fn approach_matches_plan_b() {
    fixture().run("approach", |input| {
        json!({"degrees": navigation::approach(
            position(&input["position"]),
            position(&input["top_left"]),
            position(&input["bottom_right"]),
            number(input, "heading"),
            number(input, "cross_gain"),
            number(input, "intercept_max"),
        )})
    });
}

#[test]
fn approach_to_answers_what_approach_does() {
    fixture().run("approach", |input| {
        let (target, gate_a, gate_b) = navigation::zone(position(&input["top_left"]), position(&input["bottom_right"]));
        json!({"degrees": navigation::approach_to(
            position(&input["position"]),
            target,
            gate_a,
            gate_b,
            number(input, "heading"),
            number(input, "cross_gain"),
            number(input, "intercept_max"),
        )})
    });
}

#[test]
#[should_panic(expected = "golden cases failed in 'approach'")]
fn a_heading_off_by_a_tenth_fails() {
    // the 0.05 degree tolerance is real: an approach a tenth of a degree off fails
    fixture().run("approach", |input| {
        let heading = navigation::approach(
            position(&input["position"]),
            position(&input["top_left"]),
            position(&input["bottom_right"]),
            number(input, "heading"),
            number(input, "cross_gain"),
            number(input, "intercept_max"),
        );
        json!({"degrees": heading + 0.1})
    });
}

#[test]
#[should_panic(expected = "golden cases failed in 'offset'")]
fn an_offset_a_metre_long_fails() {
    fixture().run("offset", |input| {
        let (east, north) = navigation::offset(position(&input["from"]), position(&input["to"]));
        json!({"metres": [east + 1.0, north]})
    });
}

#[test]
fn a_one_unit_slip_fails_every_advance_case() {
    // advance's 0.6 unit tolerance admits the rounding to a whole unit and nothing more
    let fixture = fixture();
    for case in fixture.cases("advance") {
        let input = &case["input"];
        let moved = navigation::advance(
            position(&input["position"]),
            number(input, "east"),
            number(input, "north"),
        );
        let name = case["name"].as_str().unwrap();
        assert_eq!(
            fixture.check(name, &case["output"], &json!({"units": units(moved)})),
            Ok(())
        );
        let slipped = Position {
            latitude_e7: moved.latitude_e7 + 1,
            longitude_e7: moved.longitude_e7 - 1,
        };
        assert!(
            fixture
                .check(name, &case["output"], &json!({"units": units(slipped)}))
                .is_err(),
            "{name}: a one-unit slip passed"
        );
    }
}

#[test]
fn a_wrong_leg_or_waypoint_fails() {
    let fixture = fixture();
    let case = &fixture.cases("steer")[0];
    let mut answer = case["output"].clone();
    answer["exact"]["leg"] = json!(1);
    assert!(fixture.check("leg", &case["output"], &answer).is_err());
    let mut answer = case["output"].clone();
    answer["exact"]["waypoint"][0] = json!(answer["exact"]["waypoint"][0].as_i64().unwrap() + 1);
    assert!(fixture.check("waypoint", &case["output"], &answer).is_err());
}
