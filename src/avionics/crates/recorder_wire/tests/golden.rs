//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The wire codec answers as plan B and the host reference do: src/avionics/golden/recorder_wire.json, which
//! golden/build_recorder_wire.py builds from src/glider/recorder.py (its real drain), test_recorder.py's literal
//! vectors and tools/recorder_wire.py on CPython. Every member compares exactly. A case whose Rust answer
//! differs on purpose carries `divergence` and the oracle's own answer in `oracle`; `output` is the Rust answer,
//! and the tests below prove each such divergence is real.

use golden::{Compare, Fixture};
use recorder_wire::{
    Cell, Counters, SESSION_INDEX, SESSION_INDEX_HEADER, SessionLabel, SessionRow, Status, cell, classify, crc32,
    header, is_label, log_line, row, session_label, session_prefix, status_message, verify, wrap,
};
use serde_json::{Value, json};

/// The fixture, compiled in.
const FIXTURE: &str = include_str!("../../../golden/recorder_wire.json");

/// The fixture members that hold `{name, input, output}` cases.
const MEMBERS: [&str; 10] = [
    "crc32",
    "wraps",
    "damage",
    "telemetry",
    "logs",
    "session_rows",
    "statuses",
    "cells",
    "labels",
    "prefixes",
];

/// A member of the fixture as golden fixture text, under the fixture's own compare block.
fn section(member: &str) -> String {
    let fixture = Fixture::parse(FIXTURE);
    json!({"compare": fixture.get("compare"), "cases": fixture.get(member)}).to_string()
}

/// The bytes a fixture byte string names: its UTF-8 text, or `{"hex": ...}`.
fn bytes(value: &Value) -> Vec<u8> {
    if let Some(text) = value.as_str() {
        return text.as_bytes().to_vec();
    }
    let hex = value["hex"]
        .as_str()
        .unwrap_or_else(|| panic!("not a byte string: {value}"));
    (0..hex.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).expect("hex digits"))
        .collect()
}

/// A byte string as the fixture writes it.
fn bytes_json(data: &[u8]) -> Value {
    match std::str::from_utf8(data) {
        Ok(text) => json!(text),
        Err(_) => json!({"hex": golden::hex(data)}),
    }
}

/// An unsigned input number.
fn unsigned(value: &Value) -> u64 {
    value
        .as_u64()
        .unwrap_or_else(|| panic!("not an unsigned number: {value}"))
}

/// An i32 input number.
fn integer(value: &Value) -> i32 {
    i32::try_from(value.as_i64().unwrap_or_else(|| panic!("not an integer: {value}"))).expect("an i32")
}

/// An optional u32 input number (null for None).
fn boot_id(value: &Value) -> Option<u32> {
    value.as_u64().map(|number| u32::try_from(number).expect("a u32"))
}

/// A typed row cell, `{"kind", "value"}`, as the ring record carries it.
fn row_cell(cell: &Value) -> Cell<'_> {
    let value = &cell["value"];
    match cell["kind"].as_str().expect("a cell kind") {
        "none" => Cell::None,
        "i32" => Cell::I32(integer(value)),
        "i64" => Cell::I64(value.as_i64().expect("an i64")),
        "f32" => Cell::F32(match value.as_str() {
            Some(text) => text.parse().expect("NaN, inf or -inf"),
            None => value.as_f64().expect("a float") as f32,
        }),
        "bool" => Cell::Bool(value.as_bool().expect("a flag")),
        "e7" => Cell::E7(integer(value)),
        "text" => Cell::Text(value.as_str().expect("a text")),
        other => panic!("unknown cell kind {other}"),
    }
}

/// What the firmware hands the codec for a JSON value that may not be text: the text, or "" (an empty cell).
fn text_or_empty(value: &Value) -> &str {
    value.as_str().unwrap_or("")
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(FIXTURE);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "recorder_wire");
    for source in [
        "src/glider/recorder.py",
        "src/glider/test/test_recorder.py",
        "tools/recorder_wire.py",
    ] {
        assert!(meta["source"].as_str().unwrap().contains(source), "{source}");
    }
    assert_eq!(fixture.compare(), Compare::Exact);
}

#[test]
fn crc32_matches_zlib() {
    golden::run(&section("crc32"), |input| {
        let previous = u32::try_from(unsigned(&input["previous"])).expect("a u32");
        json!(crc32(previous, &bytes(&input["bytes"])))
    });
}

#[test]
fn wraps_match_plan_b_and_the_host_reference() {
    golden::run(&section("wraps"), |input| {
        bytes_json(&wrap(input["routing"].as_str(), &bytes(&input["payload"])))
    });
}

#[test]
fn the_literal_board_vectors_are_all_there() {
    // test_recorder.py's 4 routed + 4 log vectors, and uptimes past 2^30, 2^32 and u64::MAX
    let wraps = Fixture::parse(FIXTURE).get("wraps").as_array().unwrap().clone();
    let names: Vec<&str> = wraps.iter().map(|case| case["name"].as_str().unwrap()).collect();
    for index in 0..4 {
        assert!(
            names.contains(&format!("routed_vector_{index}").as_str()),
            "routed {index}"
        );
        assert!(names.contains(&format!("log_vector_{index}").as_str()), "log {index}");
    }
    for name in ["at_2_30", "past_2_32", "u64_max", "past_u64", "log_u64_max"] {
        assert!(names.contains(&name), "{name}");
    }
}

#[test]
fn damage_verdicts_match_the_host_reference() {
    golden::run(&section("damage"), |input| {
        let line = bytes(input);
        let read = classify(&line);
        let status = match read.status {
            Status::Good => "good",
            Status::Bad => "bad",
            Status::Legacy => "legacy",
        };
        json!({"status": status, "routing": read.routing.map(bytes_json), "payload": bytes_json(read.payload)})
    });
}

#[test]
fn the_board_verifies_every_record_the_host_salvages() {
    // the board does not salvage, but each record the host's recover() proves must verify here under the routing
    // it was proven under, to the same payload: the two CRC readers agree
    let fixture = Fixture::parse(FIXTURE);
    let mut records = 0;
    for case in fixture.get("damage").as_array().unwrap() {
        let Some(salvage) = case.get("salvage") else {
            continue;
        };
        assert_eq!(
            case["output"]["status"], "bad",
            "{}: only a bad line is salvaged",
            case["name"]
        );
        for record in salvage["records"].as_array().unwrap() {
            let routing = record["routing"].as_str().map(str::as_bytes);
            let body = bytes(&record["record"]);
            assert_eq!(
                verify(routing, &body).map(bytes_json),
                Some(record["payload"].clone()),
                "{}",
                case["name"]
            );
            records += 1;
        }
    }
    assert!(
        records >= 8,
        "the fixture salvages too little to say anything: {records}"
    );
}

#[test]
fn telemetry_lines_match_plan_b() {
    golden::run(&section("telemetry"), |input| {
        let routing = format!(
            "{}_{}",
            input["session"].as_str().unwrap(),
            input["stream"].as_str().unwrap()
        );
        let fields: Vec<&str> = input["fields"]
            .as_array()
            .unwrap()
            .iter()
            .map(|field| field.as_str().unwrap())
            .collect();
        let mut payload = Vec::new();
        header(&mut payload, &fields);
        let mut lines = vec![bytes_json(&wrap(Some(&routing), &payload))];
        for pushed in input["rows"].as_array().unwrap() {
            let cells: Vec<Cell> = pushed["cells"].as_array().unwrap().iter().map(row_cell).collect();
            payload.clear();
            row(&mut payload, unsigned(&pushed["uptime_us"]), &cells);
            lines.push(bytes_json(&wrap(Some(&routing), &payload)));
        }
        json!(lines)
    });
}

#[test]
fn log_lines_match_plan_b() {
    golden::run(&section("logs"), |input| {
        let mut payload = Vec::new();
        log_line(
            &mut payload,
            unsigned(&input["uptime_us"]),
            input["descriptor"].as_str().unwrap(),
            input["message"].as_str().unwrap(),
            usize::try_from(unsigned(&input["limit"])).unwrap(),
        );
        bytes_json(&wrap(None, &payload))
    });
}

#[test]
fn session_rows_match_plan_b() {
    golden::run(&section("session_rows"), |input| {
        let session_row = SessionRow {
            uptime_us: unsigned(&input["uptime_us"]),
            boot_id: boot_id(&input["boot_id"]),
            session: input["session"].as_str().unwrap(),
            epoch: input["epoch"].as_i64().unwrap(),
            utc_offset: input["utc_offset"]
                .as_i64()
                .map(|minutes| i32::try_from(minutes).unwrap()),
            board: text_or_empty(&input["board"]),
            firmware: text_or_empty(&input["firmware"]),
            config_id: input["config_id"].as_str().unwrap(),
            source: text_or_empty(&input["source"]),
            cc_position: input["cc_position"]
                .as_array()
                .map(|units| (integer(&units[0]), integer(&units[1]))),
        };
        let mut payload = Vec::new();
        session_row.append_to(&mut payload);
        json!([
            bytes_json(&wrap(Some(SESSION_INDEX), SESSION_INDEX_HEADER.as_bytes())),
            bytes_json(&wrap(Some(SESSION_INDEX), &payload)),
        ])
    });
}

#[test]
fn status_messages_match_plan_b() {
    golden::run(&section("statuses"), |input| {
        let counters = |ring: &Value| Counters {
            count: u32::try_from(unsigned(&ring["count"])).unwrap(),
            max: u32::try_from(unsigned(&ring["max"])).unwrap(),
            dropped: u32::try_from(unsigned(&ring["dropped"])).unwrap(),
        };
        let members: Vec<(&str, &str)> = input["members"]
            .as_array()
            .unwrap()
            .iter()
            .map(|member| (member[0].as_str().unwrap(), member[1].as_str().unwrap()))
            .collect();
        json!(status_message(
            input["session"].as_str().unwrap(),
            &counters(&input["tlm"]),
            &counters(&input["log"]),
            &members
        ))
    });
}

#[test]
fn cells_match_plan_b() {
    golden::run(&section("cells"), |input| json!(cell(text_or_empty(input))));
}

#[test]
fn labels_match_plan_b() {
    golden::run(&section("labels"), |input| json!(is_label(input.as_str().unwrap())));
}

#[test]
fn session_prefixes_match_plan_b() {
    golden::run(&section("prefixes"), |input| {
        let label = input["configured"].as_str().map_or(SessionLabel::Unset, session_label);
        let random = u32::try_from(unsigned(&input["random"])).unwrap();
        json!(session_prefix(
            &label,
            boot_id(&input["boot_id"]),
            input["epoch"].as_i64().unwrap(),
            random
        ))
    });
}

#[test]
fn every_member_has_cases_and_only_marked_cases_diverge() {
    let fixture = Fixture::parse(FIXTURE);
    for member in MEMBERS {
        let cases = fixture.get(member).as_array().unwrap();
        assert!(!cases.is_empty(), "{member}");
        for case in cases {
            // a divergence names its reason and carries the oracle's own, different answer; nothing else does
            assert_eq!(
                case.get("divergence").is_some(),
                case.get("oracle").is_some(),
                "{member} {}",
                case["name"]
            );
            if let Some(oracle) = case.get("oracle") {
                assert_ne!(
                    oracle, &case["output"],
                    "{member} {}: a divergence that is not",
                    case["name"]
                );
            }
        }
    }
}

#[test]
fn the_divergences_are_the_ones_the_crate_documents() {
    let fixture = Fixture::parse(FIXTURE);
    let marked: Vec<(String, String)> = MEMBERS
        .iter()
        .flat_map(|member| {
            fixture
                .get(member)
                .as_array()
                .unwrap()
                .iter()
                .filter(|case| case.get("divergence").is_some())
                .map(move |case| (member.to_string(), case["name"].as_str().unwrap().to_string()))
        })
        .collect();
    let expected = [
        ("damage", "a_leading_zero_made_a_space"),
        ("damage", "a_leading_zero_made_a_sign"),
        ("damage", "a_zero_made_an_underscore"),
        ("telemetry", "gnss_degrees"),
        ("telemetry", "integral_floats"),
        ("telemetry", "floats_plan_b_writes_with_an_exponent"),
        ("telemetry", "floats_that_are_no_numbers"),
        ("telemetry", "f64_digits_no_f32_holds"),
        ("telemetry", "text_with_separators"),
        ("logs", "an_inner_newline"),
        ("logs", "a_trailing_newline"),
        ("logs", "tabs"),
        ("session_rows", "a_set_with_the_cc_position"),
        ("session_rows", "a_southern_eastern_cc_position"),
        ("prefixes", "true"),
    ];
    let expected: Vec<(String, String)> = expected
        .iter()
        .map(|(member, name)| (member.to_string(), name.to_string()))
        .collect();
    assert_eq!(marked, expected);
}

#[test]
fn the_host_verifies_a_damaged_tag_the_board_refuses() {
    // a leading '0' flipped to a space (one bit): the host's int(tag, 16) strips it and the line checks out there
    let fixture = Fixture::parse(FIXTURE);
    let case = fixture
        .get("damage")
        .as_array()
        .unwrap()
        .iter()
        .find(|case| case["name"] == "a_leading_zero_made_a_space")
        .unwrap();
    assert_eq!(case["oracle"]["status"], "good");
    let line = bytes(&case["input"]);
    assert!(
        line.windows(2).any(|pair| pair == b"{ "),
        "the damaged tag is in the line"
    );
    assert_eq!(classify(&line).status, Status::Bad);
}

#[test]
fn rust_prints_a_float_no_longer_than_plan_b_on_the_board() {
    // the 2026-10-03 captures carry MicroPython's text; Rust's reads back to the same f32 and is never longer
    for board in [
        "1.7639999",
        "0.343",
        "-0.098",
        "0.982831936",
        "-36.806876",
        "101767.91",
        "4.244812",
    ] {
        let value: f32 = board.parse().unwrap();
        let mut text = Vec::new();
        row(&mut text, 0, &[Cell::F32(value)]);
        let rust = std::str::from_utf8(&text[2..]).unwrap();
        assert_eq!(rust.parse::<f32>().unwrap(), value, "{board}");
        assert!(rust.len() <= board.len(), "{board} -> {rust}");
    }
}
