//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board config answers as plan B does: the fixtures golden/build_config_{id,validate,load,wifi}.py build
//! from src/glider config.py, drivers/wifi.py and tasks/cc_link.py on CPython, every one compared exactly. A
//! fixture input names its document as golden/config_corpus.py describes: a committed board file (compiled in
//! below), a base the fixture carries, or the document itself, then edits (plain data moves) on top.
//!
//! Also here: the canonical writer reproduces every committed launches file byte for byte.

use board_config::{Hub, Wifi, canon, config_id, load, validate, wifi, write};
use golden::{Compare, Fixture};
use serde_json::{Value, json};

/// config_id() over the corpus, plus _canon() over the value sweep and the printable-table edges.
const CONFIG_ID: &str = include_str!("../../../golden/config_id.json");

/// validate() over the corpus, test_config.py's inputs and each rule broken once.
const CONFIG_VALIDATE: &str = include_str!("../../../golden/config_validate.json");

/// load() over stored texts and defaults.
const CONFIG_LOAD: &str = include_str!("../../../golden/config_load.json");

/// The Wi-Fi plan B's wifi driver and CC link set up from a document.
const CONFIG_WIFI: &str = include_str!("../../../golden/config_wifi.json");

/// Every launches/20261003/*/*.config, by its repository path, as the bytes git holds.
const LAUNCH_FILES: [(&str, &str); 5] = [
    (
        "launches/20261003/TMS-7C/tms7c.config",
        include_str!("../../../../../launches/20261003/TMS-7C/tms7c.config"),
    ),
    (
        "launches/20261003/TMS-7D/tms7d.config",
        include_str!("../../../../../launches/20261003/TMS-7D/tms7d.config"),
    ),
    (
        "launches/20261003/TMS-7D/tms7d_control.config",
        include_str!("../../../../../launches/20261003/TMS-7D/tms7d_control.config"),
    ),
    (
        "launches/20261003/TMS-7E/tms7e.config",
        include_str!("../../../../../launches/20261003/TMS-7E/tms7e.config"),
    ),
    (
        "launches/20261003/TMS-7F/tms7f.config",
        include_str!("../../../../../launches/20261003/TMS-7F/tms7f.config"),
    ),
];

/// A committed board file's text, by its repository path.
fn file(path: &str) -> &'static str {
    LAUNCH_FILES
        .iter()
        .find(|(name, _)| *name == path)
        .unwrap_or_else(|| panic!("{path} is not compiled in"))
        .1
}

/// A member of a fixture as golden fixture text, under the fixture's own compare block.
fn section(text: &str, member: &str) -> String {
    let fixture = Fixture::parse(text);
    json!({"compare": fixture.get("compare"), "cases": fixture.get(member)}).to_string()
}

/// The member of `value` an edit path step names: an object key or a list index.
fn step<'value>(value: &'value mut Value, key: &Value) -> &'value mut Value {
    let found = match key {
        Value::String(key) => value.get_mut(key.as_str()),
        Value::Number(index) => value.get_mut(index.as_u64().expect("a list index") as usize),
        _ => None,
    };
    found.unwrap_or_else(|| panic!("edit path step {key} names nothing"))
}

/// Apply one fixture edit: set (or append, one past a list's end) or delete the member at its path.
fn apply(document: &mut Value, edit: &Value) {
    let path = edit["path"].as_array().expect("an edit has a path");
    let (last, parents) = path.split_last().expect("an edit path is not empty");
    let parent = parents.iter().fold(document, step);
    let delete = edit["delete"] == true;
    let value = || edit.get("value").expect("a set edit has a value").clone();
    match (parent, last) {
        (Value::Object(members), Value::String(key)) if delete => {
            members.remove(key).expect("a deleted key exists");
        }
        (Value::Object(members), Value::String(key)) => {
            members.insert(key.clone(), value());
        }
        (Value::Array(items), Value::Number(index)) => {
            let index = index.as_u64().expect("a list index") as usize;
            if delete {
                items.remove(index);
            } else if index == items.len() {
                items.push(value());
            } else {
                items[index] = value();
            }
        }
        (parent, last) => panic!("edit {edit}: {last} does not address a member of {parent}"),
    }
}

/// The document a fixture input names (see the module notes), with its edits applied.
fn document(fixture: &Fixture, specification: &Value) -> Value {
    let forms = ["file", "base", "document"].map(|form| specification.get(form));
    let mut document = match forms {
        [Some(path), None, None] => serde_json::from_str(file(path.as_str().expect("a path"))).expect("JSON"),
        [None, Some(base), None] => {
            let name = base.as_str().expect("a base name");
            let bases = fixture.get("bases");
            bases.get(name).unwrap_or_else(|| panic!("no base {name}")).clone()
        }
        [None, None, Some(document)] => document.clone(),
        _ => panic!("{specification} names its document by none or several of file, base, document"),
    };
    for edit in specification
        .get("edits")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
    {
        apply(&mut document, edit);
    }
    document
}

/// A case's input and output, by its name.
fn case(fixture: &Fixture, name: &str) -> (Value, Value) {
    let cases = fixture.get("cases").as_array().expect("cases");
    let found = cases.iter().find(|case| case["name"] == name);
    let found = found.unwrap_or_else(|| panic!("no case {name}"));
    (found["input"].clone(), found["output"].clone())
}

/// The view in the fixture's shape: the hub as "gateway", "off" or {"host": h}.
fn wifi_answer(view: &Wifi) -> Value {
    let hub = match &view.hub {
        Hub::Gateway => json!("gateway"),
        Hub::Off => json!("off"),
        Hub::Host(host) => json!({"host": host}),
    };
    let networks: Vec<Value> = view
        .networks
        .iter()
        .map(|network| {
            json!({"ssid": network.ssid, "retry_ms": network.retry_ms, "password": network.password,
                   "tx_power_dbm": network.tx_power_dbm})
        })
        .collect();
    json!({"policy_enabled": view.policy_enabled, "networks": networks, "hub": hub, "hub_port": view.hub_port})
}

#[test]
fn the_fixtures_say_where_they_came_from() {
    for (text, generator) in [
        (CONFIG_ID, "config_id"),
        (CONFIG_VALIDATE, "config_validate"),
        (CONFIG_LOAD, "config_load"),
        (CONFIG_WIFI, "config_wifi"),
    ] {
        let fixture = Fixture::parse(text);
        let meta = fixture.get("meta");
        assert_eq!(meta["tag"], golden::PLAN_B_TAG);
        assert_eq!(meta["generator"], generator);
        assert_eq!(meta["python"], "3.12");
        assert!(meta["source"].as_str().unwrap().contains("src/glider/"));
        assert_eq!(fixture.compare(), Compare::Exact);
    }
    assert_eq!(Fixture::parse(CONFIG_ID).get("unicode_version"), "15.0.0");
}

#[test]
fn config_ids_match_plan_b() {
    let fixture = Fixture::parse(CONFIG_ID);
    golden::run(CONFIG_ID, |input| json!(config_id(&document(&fixture, input))));
}

#[test]
fn the_published_ids_hold() {
    // doc/specs/avionics-port.md M2's vectors, read through the fixture's own inputs
    let fixture = Fixture::parse(CONFIG_ID);
    for (name, published) in [
        ("tms7c", "e95f341166d4"),
        ("tms7d", "7f06b2f496a9"),
        ("tms7d_control", "04b9c8644717"),
        ("tms7e", "2492734e035f"),
        ("tms7f", "f80fcbb3231e"),
        ("default", "95e1af1d7b1d"),
        ("empty", "44136fa355b3"),
        ("taster_copy_of_tms7f", "38052780281d"),
    ] {
        let (input, output) = case(&fixture, name);
        assert_eq!(output, published, "{name}: the fixture");
        assert_eq!(config_id(&document(&fixture, &input)), published, "{name}: the port");
    }
}

#[test]
fn an_id_off_by_one_document_change_differs() {
    // the comparison is exact: a config one value away from a published one never passes for it
    let fixture = Fixture::parse(CONFIG_ID);
    let (input, output) = case(&fixture, "tms7f");
    let mut changed = document(&fixture, &input);
    changed["board"]["rev"] = json!(2);
    assert_ne!(json!(config_id(&changed)), output);
    changed["board"]["rev"] = json!(1.0);
    assert_ne!(
        json!(config_id(&changed)),
        output,
        "an int written as a float is another config"
    );
}

#[test]
fn canon_matches_python_repr() {
    golden::run(&section(CONFIG_ID, "canon"), |input| json!(canon(input)));
}

#[test]
fn ties_and_narrow_powers_of_two_print_as_cpython() {
    // each one alone, through canon and through the writer (json.dumps writes a float as its repr): an exact tie
    // takes the even spelling, and a power of two keeps the shortest digits that read back
    let fixture = Fixture::parse(CONFIG_ID);
    let cases: Vec<&Value> = fixture
        .get("canon")
        .as_array()
        .unwrap()
        .iter()
        .filter(|case| {
            let name = case["name"].as_str().unwrap();
            name.starts_with("float_tie_") || name.starts_with("float_narrow_")
        })
        .collect();
    let outputs: Vec<&str> = cases.iter().map(|case| case["output"].as_str().unwrap()).collect();
    for spelling in [
        "1371010358211522.2",
        "259189143840433.12",
        "23219187932159.812",
        "5.960464477539063e-08",
    ] {
        assert!(
            outputs.contains(&spelling),
            "{spelling} is not in the fixture: {outputs:?}"
        );
    }
    for case in cases {
        let output = case["output"].as_str().unwrap();
        assert_eq!(canon(&case["input"]), output, "{}", case["name"]);
        assert_eq!(write(&case["input"]), format!("{output}\n"), "{}", case["name"]);
    }
}

#[test]
fn canon_holds_the_printable_table_to_cpython_at_every_edge() {
    // the table's 1422 edges, each with its lower neighbour: U+007F/U+0080 up to U+10FFFF
    let fixture = Fixture::parse(CONFIG_ID);
    let text: String = fixture
        .get("unicode")
        .as_array()
        .unwrap()
        .iter()
        .map(|case| case["input"].as_str().unwrap())
        .collect();
    assert!(text.chars().count() >= 1422, "{} characters", text.chars().count());
    for edge in ['\u{7f}', '\u{80}', '\u{a1}', '\u{10ffff}'] {
        assert!(text.contains(edge), "{edge:?}");
    }
    golden::run(&section(CONFIG_ID, "unicode"), |input| json!(canon(input)));
}

#[test]
fn validate_matches_plan_b() {
    let fixture = Fixture::parse(CONFIG_VALIDATE);
    golden::run(CONFIG_VALIDATE, |input| json!(validate(&document(&fixture, input))));
}

#[test]
fn a_message_out_of_order_or_reworded_fails_the_comparison() {
    let fixture = Fixture::parse(CONFIG_VALIDATE);
    let (input, output) = case(&fixture, "every_section_at_once");
    let answer = json!(validate(&document(&fixture, &input)));
    assert_eq!(Compare::Exact.check("every_section_at_once", &output, &answer), Ok(()));
    let mut swapped = answer.clone();
    swapped.as_array_mut().unwrap().swap(0, 1);
    assert!(Compare::Exact.check("swapped", &output, &swapped).is_err());
    let mut reworded = answer;
    reworded[0] = json!("board.id must not contain whitespace");
    assert!(Compare::Exact.check("reworded", &output, &reworded).is_err());
}

#[test]
fn the_corpus_validates_clean() {
    let fixture = Fixture::parse(CONFIG_VALIDATE);
    for name in [
        "tms7c",
        "tms7d",
        "tms7d_control",
        "tms7e",
        "tms7f",
        "default",
        "taster_copy_of_tms7f",
    ] {
        let (input, output) = case(&fixture, name);
        assert_eq!(output, json!([]), "{name}: plan B");
        assert_eq!(
            validate(&document(&fixture, &input)),
            Vec::<String>::new(),
            "{name}: the port"
        );
    }
}

#[test]
fn load_matches_plan_b() {
    let fixture = Fixture::parse(CONFIG_LOAD);
    golden::run(CONFIG_LOAD, |input| {
        let stored = match &input["stored"] {
            Value::Null => None,
            Value::String(text) => Some(text.as_str()),
            named => Some(file(named["file"].as_str().expect("a stored file path"))),
        };
        let loaded = load(stored, &document(&fixture, &input["defaults"]));
        json!({"document": loaded.document, "source": loaded.source, "errors": loaded.errors})
    });
}

#[test]
fn the_taster_copy_boots_as_the_m2_proof_says() {
    let fixture = Fixture::parse(CONFIG_LOAD);
    let (_, output) = case(&fixture, "taster_copy_on_the_firmware_default");
    assert_eq!(
        output["source"],
        "active(config 20260828, firmware 20261005 -- re-save to adopt)"
    );
    assert_eq!(config_id(&output["document"]), "38052780281d");
}

#[test]
fn wifi_matches_plan_b() {
    let fixture = Fixture::parse(CONFIG_WIFI);
    golden::run(CONFIG_WIFI, |input| wifi_answer(&wifi(&document(&fixture, input))));
}

#[test]
fn the_writer_reproduces_every_launches_file() {
    for (path, text) in LAUNCH_FILES {
        let parsed: Value = serde_json::from_str(text).unwrap();
        assert_eq!(write(&parsed), text, "{path}");
        // the same document in any key order and spacing writes the same bytes
        let compact: Value = serde_json::from_str(&serde_json::to_string(&parsed).unwrap()).unwrap();
        assert_eq!(write(&compact), text, "{path}: from compact text");
    }
}

#[test]
fn the_writer_is_not_serde_json_pretty() {
    // the files are json.dumps(indent=1): one space per level, and Python's float text
    let (path, text) = LAUNCH_FILES[0];
    let parsed: Value = serde_json::from_str(text).unwrap();
    assert_ne!(serde_json::to_string_pretty(&parsed).unwrap() + "\n", text, "{path}");
    assert_eq!(
        write(&json!({"g": 1e-5, "h": 1e16})),
        "{\n \"g\": 1e-05,\n \"h\": 1e+16\n}\n"
    );
}

#[test]
fn written_documents_read_back_with_the_same_id() {
    for text in [CONFIG_ID, CONFIG_VALIDATE] {
        let fixture = Fixture::parse(text);
        let bases = fixture.get("bases").as_object().unwrap();
        for (name, base) in bases {
            let back: Value = serde_json::from_str(&write(base)).unwrap();
            assert_eq!(&back, base, "{name}");
            assert_eq!(config_id(&back), config_id(base), "{name}");
        }
    }
}
