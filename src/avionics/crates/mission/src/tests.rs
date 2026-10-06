//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the mission rules, positive and negative. The plan B comparison itself is tests/golden.rs.

use super::{DEFAULT_MAX_RANGE_M, Mission, Position, TimeSet, Zone, clock_text, epoch_request};
use serde_json::{Map, Value, json};
use std::collections::BTreeSet;

/// A JSON object literal as the property map a request carries.
fn properties(value: Value) -> Map<String, Value> {
    value.as_object().expect("an object literal").clone()
}

/// The position at these decimal degrees.
fn at(latitude: f64, longitude: f64) -> Position {
    Position::from_degrees(latitude, longitude)
}

/// A mission loaded from this launch.config text, with the default range.
fn loaded(text: &str) -> Mission {
    Mission::load(Some(text), DEFAULT_MAX_RANGE_M)
}

/// The keys of a JSON object as a set: what an answer carries, whatever order serde_json keeps them in (a
/// workspace member may turn on its `preserve_order`).
fn keys(value: &Value) -> BTreeSet<&str> {
    value
        .as_object()
        .expect("an object")
        .keys()
        .map(String::as_str)
        .collect()
}

/// Every number in a JSON answer, at any depth.
fn numbers(value: &Value) -> Vec<&serde_json::Number> {
    match value {
        Value::Number(number) => vec![number],
        Value::Array(items) => items.iter().flat_map(numbers).collect(),
        Value::Object(members) => members.values().flat_map(numbers).collect(),
        _ => Vec::new(),
    }
}

#[test]
fn no_stored_document_loads_the_hprc_default() {
    let hprc = Mission::load(None, DEFAULT_MAX_RANGE_M);
    assert_eq!((hprc.launch_id.as_str(), hprc.site.as_str()), ("", "HPRC"));
    assert_eq!(hprc.launch_point(None), Some(at(25.514379, -80.391795)));
    assert_eq!(hprc.altitude, Some(2.0));
    assert_eq!(
        hprc.zone,
        Some(Zone {
            top_left: at(25.514944, -80.392972),
            bottom_right: at(25.514583, -80.391111)
        })
    );
    assert!(hprc.sites.is_empty());
    assert_eq!(hprc.probe(None), None);
}

#[test]
fn text_that_is_not_a_json_object_loads_the_hprc_default() {
    let hprc = Mission::load(None, DEFAULT_MAX_RANGE_M);
    for text in ["", "{ not json", "[1, 2]", "null", "\"HPRC\"", "7"] {
        assert_eq!(loaded(text), hprc, "{text:?}");
    }
}

#[test]
fn an_empty_object_is_a_blank_mission_not_the_default() {
    let blank = loaded("{}");
    assert_eq!((blank.launch_id.as_str(), blank.site.as_str()), ("", ""));
    assert_eq!(
        (blank.latitude_e7, blank.longitude_e7, blank.altitude),
        (None, None, None)
    );
    assert_eq!(blank.zone, None);
    assert_eq!(
        blank.probe(None).as_deref(),
        Some("launch/zone: launch position not set (CC or GNSS)")
    );
}

#[test]
fn load_keeps_valid_fields_and_unsets_invalid_ones() {
    let mission =
        loaded(r#"{"launch_id": "hprc-t1", "site": "pad-a", "latitude": 45.5, "longitude": -73.5, "altitude": 120}"#);
    assert_eq!(
        (mission.launch_id.as_str(), mission.site.as_str()),
        ("hprc-t1", "pad-a")
    );
    assert_eq!(mission.launch_point(None), Some(at(45.5, -73.5)));
    assert_eq!(mission.altitude, Some(120.0));
    // out of range, a bool, text: a coordinate must be an in-range number
    let invalid = loaded(r#"{"latitude": 200, "longitude": -73.5}"#);
    assert_eq!((invalid.latitude_e7, invalid.longitude_e7), (None, Some(-735_000_000)));
    let invalid = loaded(r#"{"latitude": true, "longitude": "11.0", "zone": [[48.0, 11.0], [48.1]]}"#);
    assert_eq!(
        (invalid.latitude_e7, invalid.longitude_e7, invalid.zone),
        (None, None, None)
    );
    // the deliberate type rules: text fields that are not text, an altitude that is not a number
    let typed = loaded(r#"{"launch_id": 7, "site": null, "altitude": "high"}"#);
    assert_eq!(
        (typed.launch_id.as_str(), typed.site.as_str(), typed.altitude),
        ("", "", None)
    );
}

#[test]
fn a_zone_needs_exactly_two_valid_corners() {
    for zone in [
        json!([[48.0, 11.0]]),
        json!([[48.0, 11.0], [48.1, 11.1], [48.2, 11.2]]),
        json!([[200.0, 11.0], [48.0, 11.0]]),
        json!([[48.0, 181.0], [48.0, 11.0]]),
        json!([[true, 11.0], [48.0, 11.0]]),
        json!([[48.0, 11.0, 0.0], [48.0, 11.0]]),
        json!("north"),
        json!(null),
    ] {
        assert_eq!(Zone::from_json(&zone), None, "{zone}");
    }
    assert!(Zone::from_json(&json!([[-90, -180], [90, 180]])).is_some());
}

#[test]
fn update_lists_only_changes_in_plan_b_order() {
    let mut mission = Mission::load(None, DEFAULT_MAX_RANGE_M);
    let request = json!({"zone": [[48.001, 11.0], [48.0, 11.01]], "altitude": 5, "longitude": 20.0,
        "latitude": 10.0, "site": "home", "launch_id": "flight.7", "unknown": 1});
    assert_eq!(
        mission.update(&properties(request.clone())),
        ["launch_id", "site", "latitude", "longitude", "altitude", "zone"]
    );
    assert!(
        mission.update(&properties(request)).is_empty(),
        "the same values change nothing"
    );
    assert!(
        mission.update(&properties(json!({"altitude": 5.0}))).is_empty(),
        "5 and 5.0 are one altitude"
    );
}

#[test]
fn update_ignores_invalid_values() {
    let mut mission = Mission::load(None, DEFAULT_MAX_RANGE_M);
    let before = mission.clone();
    for request in [
        json!({"latitude": 999}),
        json!({"latitude": -90.0000001}),
        json!({"longitude": 180.5}),
        json!({"latitude": true}),
        json!({"longitude": "-80.39"}),
        json!({"latitude": null, "longitude": null}),
        json!({"zone": [[48.0, 11.0]]}),
        json!({"zone": "north"}),
        json!({"launch_id": 7, "site": null}),
        json!({"altitude": "high"}),
        json!({"altitude": false}),
        json!({"sites": [{"name": "x", "pad": [48.0, 11.0], "zone": [[48.0, 11.0], [48.1, 11.1]]}]}),
        json!({"epoch": 1_781_000_000}),
    ] {
        assert!(mission.update(&properties(request.clone())).is_empty(), "{request}");
    }
    assert_eq!(mission, before);
}

#[test]
fn update_clears_the_altitude_but_never_the_pad() {
    let mut mission = Mission::load(None, DEFAULT_MAX_RANGE_M);
    assert_eq!(mission.update(&properties(json!({"altitude": null}))), ["altitude"]);
    assert_eq!(mission.altitude, None);
    assert!(mission.update(&properties(json!({"altitude": null}))).is_empty());
    assert!(mission.position_set());
}

#[test]
fn merge_replaces_sites_with_a_list_and_keeps_them_otherwise() {
    let mut mission = Mission::load(None, DEFAULT_MAX_RANGE_M);
    let east = json!({"name": "east", "pad": [48.0, 11.002], "zone": [[48.001, 11.003], [48.0005, 11.005]]});
    let changed = mission.merge(&properties(
        json!({"launch_id": "f1", "sites": [east, {"name": "bad", "pad": [48.0]}, {"pad": [1, 2], "zone": [[1, 2], [0, 3]]}]}),
    ));
    assert_eq!(changed, ["launch_id", "sites"]);
    let names: Vec<&str> = mission.sites.iter().map(|site| site.name.as_str()).collect();
    assert_eq!(
        names,
        ["east", "site1"],
        "invalid entries dropped, an unnamed one numbered"
    );
    // no sites, sites that are not a list: kept; the same list again: unchanged
    assert!(mission.merge(&properties(json!({"launch_id": "f1"}))).is_empty());
    assert!(mission.merge(&properties(json!({"sites": "none"}))).is_empty());
    assert!(mission.merge(&properties(json!({"sites": {"east": east}}))).is_empty());
    assert_eq!(mission.sites.len(), 2);
    let persisted = mission.persisted();
    assert!(
        mission
            .merge(&properties(json!({"sites": persisted["sites"]})))
            .is_empty()
    );
    // an empty list clears them
    assert_eq!(mission.merge(&properties(json!({"sites": []}))), ["sites"]);
    assert!(mission.sites.is_empty());
}

#[test]
fn a_site_name_that_is_not_text_is_numbered_and_non_objects_use_no_number() {
    // plan B skips a non-object entry without using up a site<n>, as here; it names a non-text name by its
    // repr ('5', 'None'), where the port numbers it as if it had none (the crate notes)
    let pad = json!([48.0, 11.0]);
    let zone = json!([[48.0005, 11.0], [48.0, 11.001]]);
    let request = json!({"sites": [7, null, "north", [48.0, 11.0], {"name": 5, "pad": pad, "zone": zone},
        {"name": null, "pad": pad, "zone": zone}, {"name": true, "pad": pad, "zone": zone},
        {"pad": pad, "zone": zone}]});
    // the same rule at load and on the set-config path
    let mut merged = loaded("{}");
    assert_eq!(merged.merge(&properties(request.clone())), ["sites"]);
    for mission in [loaded(&request.to_string()), merged] {
        let names: Vec<&str> = mission.sites.iter().map(|site| site.name.as_str()).collect();
        assert_eq!(names, ["site0", "site1", "site2", "site3"]);
    }
}

#[test]
fn persisted_keeps_sites_across_a_reboot() {
    let stored = r#"{"site": "field", "latitude": 48.0, "longitude": 11.0, "sites": [
        {"name": "north", "pad": [48.001, 11.0], "zone": [[48.002, 10.999], [48.0015, 11.001]]}]}"#;
    let mut mission = loaded(stored);
    assert_eq!(mission.merge(&properties(json!({"launch_id": "f1"}))), ["launch_id"]);
    let rebooted = loaded(&mission.persisted().to_string());
    assert_eq!(rebooted, mission, "plan B lost the sites here");
    assert_eq!(rebooted.sites[0].name, "north");
}

#[test]
fn persisted_writes_the_degrees_and_metres_it_was_given() {
    let mut mission = Mission::load(None, DEFAULT_MAX_RANGE_M);
    mission.update(&properties(json!({"altitude": 1.7})));
    let text = mission.persisted().to_string();
    for literal in ["25.514379", "-80.391795", "25.514944", "-80.392972", "1.7"] {
        assert!(text.contains(literal), "{literal} in {text}");
    }
    assert!(!text.contains("1.70000"), "{text}");
    assert_eq!(
        keys(&mission.persisted()),
        BTreeSet::from([
            "altitude",
            "latitude",
            "launch_id",
            "longitude",
            "site",
            "sites",
            "zone"
        ])
    );
    let blank = loaded("{}").persisted();
    assert_eq!(
        blank,
        json!({"launch_id": "", "site": "", "latitude": null, "longitude": null, "altitude": null, "zone": null, "sites": []})
    );
}

#[test]
fn the_set_position_wins_over_the_fix() {
    let fix = Some(at(49.0, 12.0));
    let mut mission = loaded("{}");
    assert_eq!(mission.launch_point(None), None);
    assert_eq!(mission.launch_point(fix), fix);
    mission.update(&properties(json!({"latitude": 48.0})));
    assert_eq!(mission.launch_point(fix), fix, "half a position is no position");
    assert!(!mission.position_set());
    mission.update(&properties(json!({"longitude": 11.0})));
    assert_eq!(mission.launch_point(fix), Some(at(48.0, 11.0)));
    assert!(mission.position_set());
}

#[test]
fn geometry_needs_a_zone_and_an_origin() {
    let mut mission = loaded("{}");
    assert_eq!(mission.geometry(Some(at(48.0, 11.0))), None, "no zone");
    mission.update(&properties(json!({"zone": [[48.0005, 11.0], [48.0, 11.001]]})));
    assert_eq!(mission.geometry(None), None, "no origin");
    let geometry = mission.geometry(Some(at(48.00025, 11.0005))).unwrap();
    assert_eq!(geometry.target, at(48.00025, 11.0005));
    assert_eq!(geometry.gates, [at(48.00025, 11.0), at(48.00025, 11.001)]);
    assert_eq!(geometry.distances_m, [0.0, 37.2, 37.2]);
    assert_eq!(geometry.max_distance_m, 37.2);
    assert!(geometry.in_range);
}

#[test]
fn probe_names_what_fails() {
    let mut mission = loaded("{}");
    assert_eq!(
        mission.probe(None).as_deref(),
        Some("launch/zone: launch position not set (CC or GNSS)")
    );
    assert_eq!(mission.probe(Some(at(48.0, 11.0))), None, "a fix and no zone passes");
    mission.update(&properties(
        json!({"latitude": 48.005, "longitude": 11.0005, "zone": [[48.0005, 11.0], [48.0, 11.001]]}),
    ));
    assert_eq!(
        mission.probe(None).as_deref(),
        Some("launch/zone: landing zone out of range (530 m > 200 m)")
    );
    let mut tight = Mission::load(Some("{}"), 10.0);
    tight.update(&properties(
        json!({"latitude": 48.00025, "longitude": 11.0005, "zone": [[48.0005, 11.0], [48.0, 11.001]]}),
    ));
    assert_eq!(
        tight.probe(None).as_deref(),
        Some("launch/zone: landing zone out of range (37 m > 10 m)")
    );
}

#[test]
fn inspect_reports_the_origin_and_where_it_came_from() {
    let mut mission = loaded("{}");
    let snapshot = mission.inspect(None, "2026-06-09T10:13:20", 1_781_000_000);
    assert_eq!(snapshot["launch_point"], Value::Null);
    assert_eq!(snapshot["launch_source"], Value::Null);
    assert_eq!(snapshot["clock"], "2026-06-09T10:13:20");
    assert_eq!(snapshot["epoch"], 1_781_000_000);
    assert!(snapshot.get("target").is_none());
    let snapshot = mission.inspect(Some(at(48.0, 11.0)), "", 0);
    assert_eq!(snapshot["launch_point"], json!([48.0, 11.0]));
    assert_eq!(snapshot["launch_source"], "gnss");
    assert_eq!(snapshot["latitude"], Value::Null);
    mission.update(&properties(
        json!({"latitude": 48.00025, "longitude": 11.0005, "zone": [[48.0005, 11.0], [48.0, 11.001]]}),
    ));
    let snapshot = mission.inspect(Some(at(49.0, 12.0)), "", 0);
    assert_eq!(snapshot["launch_source"], "set");
    assert_eq!(snapshot["launch_point"], json!([48.00025, 11.0005]));
    assert_eq!(snapshot["target"], json!([48.00025, 11.0005]));
    assert_eq!(snapshot["gates"], json!([[48.00025, 11.0], [48.00025, 11.001]]));
    assert_eq!(
        snapshot["distances_m"],
        json!({"target": 0.0, "gate_a": 37.2, "gate_b": 37.2})
    );
    assert_eq!(snapshot["in_range"], true);
    assert_eq!(
        keys(&snapshot),
        BTreeSet::from([
            "altitude",
            "clock",
            "distances_m",
            "epoch",
            "gates",
            "in_range",
            "latitude",
            "launch_id",
            "launch_point",
            "launch_source",
            "longitude",
            "site",
            "target",
            "zone"
        ])
    );
}

#[test]
fn metres_and_degrees_sent_as_integers_go_out_as_floats() {
    // the deliberate number-kind divergence: plan B echoes 48 and 500, the port writes 48.0 and 500.0
    let mut mission = loaded("{}");
    let request = json!({"latitude": 48, "longitude": 11, "altitude": 500, "zone": [[48, 11], [47, 12]],
        "sites": [{"name": "whole", "pad": [48, 11], "zone": [[48, 11], [47, 12]]}]});
    assert_eq!(
        mission.merge(&properties(request)),
        ["latitude", "longitude", "altitude", "zone", "sites"]
    );
    let persisted = mission.persisted();
    assert_eq!(persisted["latitude"], json!(48.0));
    assert_eq!(persisted["altitude"], json!(500.0));
    let snapshot = mission.inspect(None, "", 0);
    for answer in [
        &persisted,
        &snapshot["latitude"],
        &snapshot["zone"],
        &snapshot["launch_point"],
    ] {
        let kinds: Vec<bool> = numbers(answer).iter().map(|number| number.is_f64()).collect();
        assert!(!kinds.is_empty() && kinds.iter().all(|is_float| *is_float), "{answer}");
    }
    // and the same values sent as floats change nothing
    let again = json!({"latitude": 48.0, "longitude": 11.0, "altitude": 500.0, "zone": [[48.0, 11.0], [47.0, 12.0]]});
    assert!(mission.update(&properties(again)).is_empty());
}

#[test]
fn a_position_is_kept_to_the_nearest_unit() {
    // the deliberate rounding to 1e-7 degree: a move of under half a unit is no change, where plan B lists it
    let mut mission = Mission::load(None, DEFAULT_MAX_RANGE_M);
    assert_eq!(
        mission.update(&properties(json!({"latitude": 25.51437912}))),
        ["latitude"]
    );
    assert!(mission.update(&properties(json!({"latitude": 25.51437914}))).is_empty());
    assert_eq!(mission.latitude_e7, Some(255_143_791));
    assert_eq!(mission.persisted()["latitude"], json!(25.5143791));
    assert_eq!(mission.inspect(None, "", 0)["latitude"], json!(25.5143791));
    // a move that reaches the next unit is a change
    assert_eq!(
        mission.update(&properties(json!({"latitude": 25.51437918}))),
        ["latitude"]
    );
    assert_eq!(mission.latitude_e7, Some(255_143_792));
}

#[test]
fn an_epoch_request_is_an_integer_from_2000() {
    let request = |epoch: Value| epoch_request(&properties(json!({"epoch": epoch})));
    assert_eq!(request(json!(1_781_000_000)), Some(1_781_000_000));
    assert_eq!(request(json!(946_684_800)), Some(946_684_800));
    assert_eq!(request(json!(946_684_799)), None);
    assert_eq!(request(json!(-1)), None);
    assert_eq!(request(json!(1.5e9)), None, "a float is not an epoch");
    assert_eq!(request(json!(true)), None, "a bool is not an epoch");
    assert_eq!(request(json!("now")), None);
    assert_eq!(request(json!(u64::MAX)), None, "past i64 is no clock");
    assert_eq!(epoch_request(&properties(json!({"launch_id": "x"}))), None);
}

#[test]
fn time_set_inputs_validate_each_on_its_own() {
    let time_set = TimeSet::from_properties(&properties(
        json!({"epoch": 1_781_000_000, "utc_offset": -240, "cc_position": [25.5, -80.25], "source": "cc-auto"}),
    ));
    assert_eq!(
        time_set,
        TimeSet {
            utc_offset: Some(-240),
            cc_position: Some(at(25.5, -80.25)),
            source: Some("cc-auto".to_owned())
        }
    );
    let offset = |minutes: Value| TimeSet::from_properties(&properties(json!({"utc_offset": minutes}))).utc_offset;
    assert_eq!(offset(json!(-720)), Some(-720));
    assert_eq!(offset(json!(840)), Some(840));
    for invalid in [
        json!(-721),
        json!(841),
        json!(100_000),
        json!("-240"),
        json!(true),
        json!(60.0),
    ] {
        assert_eq!(offset(invalid.clone()), None, "{invalid}");
    }
    let invalid = TimeSet::from_properties(&properties(json!({"cc_position": [200.0, 0.0], "source": 7})));
    assert_eq!((invalid.cc_position, invalid.source), (None, None));
    let invalid = TimeSet::from_properties(&properties(json!({"cc_position": [25.5], "source": null})));
    assert_eq!(
        (invalid.utc_offset, invalid.cc_position, invalid.source),
        (None, None, None)
    );
}

#[test]
fn clock_text_is_utc_civil_time() {
    assert_eq!(clock_text(0), "1970-01-01T00:00:00");
    assert_eq!(clock_text(946_684_800), "2000-01-01T00:00:00");
    assert_eq!(clock_text(951_782_400), "2000-02-29T00:00:00", "2000 is a leap year");
    assert_eq!(clock_text(4_107_542_399), "2100-02-28T23:59:59");
    assert_eq!(clock_text(4_107_542_400), "2100-03-01T00:00:00", "2100 is not");
    assert_eq!(clock_text(1_781_000_000), "2026-06-09T10:13:20");
    assert_eq!(clock_text(-1), "1969-12-31T23:59:59");
}
