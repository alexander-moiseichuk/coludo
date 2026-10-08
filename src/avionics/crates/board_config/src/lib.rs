//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board config (src/glider config.py): the document a board is built from, and its rules. The hardware
//! map (buses, pins, sensors, components), the board identity and the Wi-Fi, stable across flights, unlike
//! the per-launch mission. This crate is the document with no storage: the firmware's config store feeds
//! [`load`] the stored text and persists [`write()`]'s.
//!
//! * [`canon`] and [`config_id`]: the identity CC announces, sha256 over Python's repr of the document.
//! * [`validate`]: config.validate, the same messages in the same order, plus one rule of the port's own: a
//!   device's `debounce_ms` is at most [`LONGEST_DEBOUNCE_MS`].
//! * [`load`]: which document a boot runs, and the source text whoami reports.
//! * [`write()`]: the canonical pretty text (`json.dumps(indent=1, sort_keys=True)` plus a newline).
//! * A typed view of the few keys the firmware reads so far: [`board_id`], [`mcu`], [`wifi`],
//!   [`max_range_m`], [`camera`]; and for the device bring-up (M5) the buses ([`i2c_bus`], [`spi_bus`]), the
//!   device [`directory`] with each entry's [`pin`]s and [`provides`], and the [`setup_retries`]; and for the
//!   supervisor (M8) the [`checkpoint`] cadence, the [`warm_start`] toggle and the [`watchdog`] settings.
//! * [`python`]: Python's `==`, `%s` and truthiness over config values, for the ports that read a config as
//!   plan B does.
//!
//! Python semantics are CPython 3.12's, the golden oracle (src/avionics/golden), including the float repr
//! and the str repr's escapes. A document is a `serde_json::Value`, whose numbers are f64 or integers: the
//! f64s are kept for the hash and the bytes, and never computed with (D18).
//!
//! Where serde_json reads a text differently from CPython's json (both refuse everything else alike):
//!  * `-0` is the float -0.0 here and the int 0 there. So its repr (and the id) differs, `-0.0` vs `0`, and
//!    validate() reads it as a float: it fails the checks that want an int and take a zero (a pin, a
//!    device's bus `id` and `addr`, a `provides` priority or timeout, `recorder.telemetry_ms`), so load()
//!    falls back to the defaults where plan B runs the document; and an enabled watchdog's
//!    `wdt_timeout_ms` of `-0` goes unchecked here, where plan B refuses it below the floor;
//!  * an integer past the u64 / i64 range is a float here and an exact int there, so validate() reads it
//!    as a float too (and the repr prints it as one);
//!  * `NaN`, `Infinity`, a lone surrogate escape and a number past f64 (`1e400`) parse there and are not
//!    JSON here (load() falls back for the text; plan B would have loaded it);
//!  * nesting: serde_json refuses a document with arrays and objects nested 128 levels deep or more (its
//!    recursion limit, which keeps a hostile text from overflowing the stack), and CPython's json reads one
//!    until its own recursion limit (about 1000). load() falls back for the text, where plan B runs it.
//!
//! No writer the project has emits any of these.

mod printable;
pub mod python;
mod validate;
mod view;
mod writer;

use serde_json::Value;
use sha2::{Digest, Sha256};

pub use validate::{LONGEST_DEBOUNCE_MS, validate};
pub use view::{
    Camera, CameraClock, Entry, Hub, I2cSpecification, Network, Provide, Section, SpiSpecification, Watchdog, Wifi,
    board_id, camera, checkpoint, directory, entries, i2c_bus, max_range_m, mcu, pin, provides, schedule_hz,
    setup_retries, spi_bus, warm_start, watchdog, wifi,
};
pub use writer::write;

/// How many hex digits of the sha256 a config_id keeps.
const CONFIG_ID_DIGITS: usize = 12;

/// The firmware version load() restamps when the defaults' board names none (config.py
/// `defaults['board'].get('firmware_version', 'dev')`).
const UNNAMED_FIRMWARE: &str = "dev";

/// The schema version outdated() reports for a document that carries none.
const UNVERSIONED: &str = "unversioned";

/// load()'s source for a boot with no stored config.
const SOURCE_DEFAULT: &str = "default";

/// load()'s source and its one message for stored text that is not JSON.
const SOURCE_NOT_JSON: &str = "default(fallback: board.config is not valid JSON)";

/// The message for stored text that is not JSON.
const NOT_JSON: &str = "board.config is not valid JSON";

/// load()'s source for a stored config validate() refuses.
const SOURCE_INVALID: &str = "default(fallback: invalid board.config)";

/// load()'s source for a stored config of the firmware's schema version.
const SOURCE_ACTIVE: &str = "active";

/// Plan B's `_canon`: the text config_id hashes, Python's repr of the document with sorted keys.
///
/// A dict prints `{` + `repr(key):canon(value)` joined by `,` + `}`, its keys sorted (by code point, sorted
/// here explicitly, whichever map serde_json was built with); a list prints `[a,b]`; anything else is its
/// Python repr: a str with Python's quote choice and escapes, an int in decimal, a float as Python's repr
/// (`1e-05`, `1e+16`, `5.0`), True, False and None.
pub fn canon(value: &Value) -> String {
    let mut text = String::new();
    write_canon(&mut text, value);
    text
}

/// Append the canon text of `value` (see [`canon`]).
fn write_canon(out: &mut String, value: &Value) {
    match value {
        Value::Object(members) => {
            out.push('{');
            for (index, (key, item)) in writer::sorted(members).into_iter().enumerate() {
                if index > 0 {
                    out.push(',');
                }
                python::write_string(out, key);
                out.push(':');
                write_canon(out, item);
            }
            out.push('}');
        }
        Value::Array(items) => {
            out.push('[');
            for (index, item) in items.iter().enumerate() {
                if index > 0 {
                    out.push(',');
                }
                write_canon(out, item);
            }
            out.push(']');
        }
        scalar => python::write_representation(out, scalar),
    }
}

/// Plan B's `config_id`: the first 12 hex digits of sha256(canon(document)) over its UTF-8 bytes, with
/// `board.firmware_version` left out.
///
/// load() restamps the firmware version from the running build, so keeping it would make the id a property
/// of the build: a byte-identical config would announce another id after every deploy, and the id
/// `set-config` returns would not match the one whoami then announces.
pub fn config_id(document: &Value) -> String {
    let stamped = document
        .get("board")
        .and_then(Value::as_object)
        .is_some_and(|board| board.contains_key("firmware_version"));
    let digest = if stamped {
        let mut unstamped = document.clone();
        if let Some(board) = unstamped.get_mut("board").and_then(Value::as_object_mut) {
            board.remove("firmware_version");
        }
        Sha256::digest(canon(&unstamped).as_bytes())
    } else {
        Sha256::digest(canon(document).as_bytes())
    };
    let digits: String = digest.iter().map(|byte| format!("{byte:02x}")).collect();
    digits[..CONFIG_ID_DIGITS].to_string()
}

/// What load() chose for this boot.
#[derive(Clone, Debug, PartialEq)]
pub struct Loaded {
    /// The document the board runs: the stored one (with the running firmware version restamped), or the
    /// defaults.
    pub document: Value,
    /// How it was chosen, as whoami's `config_source` reports it: `active`,
    /// `active(config X, firmware Y -- re-save to adopt)`, `default`, or `default(fallback: ...)`.
    pub source: String,
    /// validate()'s messages for the document chosen: the defaults' own when nothing is stored, the stored
    /// text's when they made it fall back, and none for a stored config that runs.
    pub errors: Vec<String>,
}

/// Plan B's `load`: the stored config if it is valid JSON and valid, else the defaults. Never fails: a bad
/// stored config degrades to the defaults, and the source says why.
///
/// * `stored` None (nothing stored): the defaults, source `default`, and the defaults' own messages.
/// * Text that is not JSON: the defaults, `default(fallback: board.config is not valid JSON)` and that one
///   message.
/// * A document validate() refuses: the defaults, `default(fallback: invalid board.config)` and its messages.
/// * Otherwise the stored document wins as it is, but for `board.firmware_version`, which is restamped from
///   the defaults' (the running build; 'dev' when they name none) whenever both board sections are objects:
///   it describes the running binary, not configuration. The source is `active`, or names the schema
///   versions when they differ (`outdated`).
pub fn load(stored: Option<&str>, defaults: &Value) -> Loaded {
    let fallback = |source: &str, errors: Vec<String>| Loaded {
        document: defaults.clone(),
        source: source.to_string(),
        errors,
    };
    let Some(text) = stored else {
        return fallback(SOURCE_DEFAULT, validate(defaults));
    };
    let Ok(mut document) = serde_json::from_str::<Value>(text) else {
        return fallback(SOURCE_NOT_JSON, vec![NOT_JSON.to_string()]);
    };
    let errors = validate(&document);
    if !errors.is_empty() {
        return fallback(SOURCE_INVALID, errors);
    }
    if let (Some(board), Some(running)) = (
        document.get_mut("board").and_then(Value::as_object_mut),
        defaults.get("board").and_then(Value::as_object),
    ) {
        let firmware = running.get("firmware_version").cloned();
        board.insert(
            "firmware_version".to_string(),
            firmware.unwrap_or_else(|| Value::from(UNNAMED_FIRMWARE)),
        );
    }
    let source = match outdated(&document, defaults) {
        None => SOURCE_ACTIVE.to_string(),
        Some((config, firmware)) => format!("active(config {config}, firmware {firmware} -- re-save to adopt)"),
    };
    Loaded {
        document,
        source,
        errors: Vec::new(),
    }
}

/// Plan B's `schema_version`: a document's top-level `version`, or None when it carries none (absent, or
/// falsy: null, '', 0, false, an empty list or object; also any document that is not an object).
fn schema_version(document: &Value) -> Option<&Value> {
    document.get("version").filter(|version| python::truthy(version))
}

/// Plan B's `outdated`: the (config, firmware) schema versions when the document's differs from the
/// defaults' (the firmware's), else None.
///
/// A mismatch never changes what runs (see [`load`]): the config came from another config tree, so it may
/// lack devices this firmware knows, and the operator re-saves to adopt them. A missing version reads
/// `unversioned`; versions compare by Python's `==` (an int version never equals a str one) and print as
/// `%s` does.
fn outdated(config: &Value, defaults: &Value) -> Option<(String, String)> {
    let saved = schema_version(config);
    let firmware = schema_version(defaults);
    let same = match (saved, firmware) {
        (Some(saved), Some(firmware)) => python::equals(saved, firmware),
        (saved, firmware) => saved.is_none() && firmware.is_none(),
    };
    let named = |version: Option<&Value>| version.map_or_else(|| UNVERSIONED.to_string(), python::formatted);
    (!same).then(|| (named(saved), named(firmware)))
}

#[cfg(test)]
mod tests {
    use super::{Loaded, canon, config_id, load, outdated, validate};
    use serde_json::{Value, json};

    /// A small valid document of schema version `version`.
    fn document(version: Value) -> Value {
        json!({"version": version, "board": {"id": "taster", "mcu": "esp32p4", "firmware_version": "saved"},
               "buses": {}, "pins": {}})
    }

    /// The running firmware's defaults: version 20261005, build 'running'.
    fn defaults() -> Value {
        json!({"version": "20261005", "board": {"id": "default", "mcu": "esp32p4", "firmware_version": "running"},
               "buses": {}, "pins": {}})
    }

    #[test]
    fn canon_sorts_keys_and_writes_python_scalars() {
        let value = json!({"b": [1, 2.0, "x"], "a": {"z": null, "y": true}, "": false});
        assert_eq!(canon(&value), "{'':False,'a':{'y':True,'z':None},'b':[1,2.0,'x']}");
        assert_eq!(canon(&json!([])), "[]");
        assert_eq!(canon(&json!({})), "{}");
        assert_eq!(canon(&json!("it's")), "\"it's\"");
    }

    #[test]
    fn the_empty_document_has_the_published_id() {
        assert_eq!(config_id(&json!({})), "44136fa355b3");
    }

    #[test]
    fn the_firmware_version_does_not_move_the_id() {
        let mut stamped = document(json!("20261005"));
        let id = config_id(&stamped);
        stamped["board"]["firmware_version"] = json!("2026.12.31.other");
        assert_eq!(config_id(&stamped), id);
        stamped["board"].as_object_mut().unwrap().remove("firmware_version");
        assert_eq!(config_id(&stamped), id);
        // ...while any configuration change does, the firmware version anywhere else included
        stamped["board"]["rev"] = json!(2);
        assert_ne!(config_id(&stamped), id);
        let mut elsewhere = document(json!("20261005"));
        elsewhere["firmware_version"] = json!("x");
        assert_ne!(config_id(&elsewhere), id);
    }

    #[test]
    fn an_id_is_twelve_lowercase_hex_digits() {
        for value in [json!(null), json!([1, 2]), json!("text"), defaults()] {
            let id = config_id(&value);
            assert_eq!(id.len(), 12);
            assert!(
                id.bytes()
                    .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte)),
                "{id}"
            );
        }
    }

    #[test]
    fn nothing_stored_boots_the_defaults() {
        let loaded = load(None, &defaults());
        assert_eq!(
            loaded,
            Loaded {
                document: defaults(),
                source: "default".to_string(),
                errors: Vec::new()
            }
        );
        assert_eq!(load(None, &json!({})).errors, validate(&json!({})));
    }

    #[test]
    fn text_that_is_not_json_falls_back() {
        for text in ["", "{ not json ", "{} x", "[1,]", "\u{feff}{}"] {
            let loaded = load(Some(text), &defaults());
            assert_eq!(loaded.document, defaults(), "{text:?}");
            assert_eq!(loaded.source, "default(fallback: board.config is not valid JSON)");
            assert_eq!(loaded.errors, ["board.config is not valid JSON"]);
        }
    }

    #[test]
    fn python_only_numbers_are_not_json_here() {
        // CPython's json reads NaN and Infinity (plan B then refuses the document as invalid); serde_json refuses
        // the text itself, so the fallback names another reason: a documented difference (crate notes)
        for text in ["NaN", "{\"x\": Infinity}", "{\"x\": 1e400}"] {
            assert_eq!(
                load(Some(text), &defaults()).source,
                "default(fallback: board.config is not valid JSON)"
            );
        }
    }

    #[test]
    fn a_document_nested_past_serde_jsons_limit_is_not_json_here() {
        // plan B loads this one as `active` (CPython's json nests until its recursion limit): a documented
        // difference (crate notes)
        let nested = |depth: usize| {
            let mut stored = document(json!("20261005")).to_string();
            stored.pop();
            format!("{stored},\"notes\":{}{}}}", "[".repeat(depth), "]".repeat(depth))
        };
        assert_eq!(
            load(Some(&nested(200)), &defaults()).source,
            "default(fallback: board.config is not valid JSON)"
        );
        // 127 levels at most: the document object is one, so the notes may nest 126 deep and no deeper
        assert_eq!(load(Some(&nested(126)), &defaults()).source, "active");
        assert_eq!(
            load(Some(&nested(127)), &defaults()).source,
            "default(fallback: board.config is not valid JSON)"
        );
    }

    #[test]
    fn a_minus_zero_int_is_a_float_here() {
        // CPython's json reads `-0` as the int 0, so plan B accepts the pin and runs the document: a documented
        // difference (crate notes)
        let stored = r#"{"board":{"id":"taster","mcu":"esp32p4"},"buses":{},"pins":{"boot":-0},"version":"20261005"}"#;
        let document: Value = serde_json::from_str(stored).unwrap();
        assert_eq!(
            validate(&document),
            ["pins.boot pin must be a non-negative int (got -0.0)"]
        );
        let loaded = load(Some(stored), &defaults());
        assert_eq!(loaded.source, "default(fallback: invalid board.config)");
        assert_eq!(loaded.document, defaults());
        // a plain 0 is the int both read
        assert_eq!(load(Some(&stored.replace("-0", "0")), &defaults()).source, "active");
        // the other way round: plan B refuses an enabled watchdog's `-0` timeout below the floor, and this
        // port, reading a float, leaves it unchecked
        let watchdog = |timeout: &str| {
            let component = json!({"name": "watchdog", "activity": "watchdog", "wdt_timeout_ms": "TIMEOUT"});
            let stored = json!({"board": {"id": "taster", "mcu": "esp32p4"}, "buses": {}, "pins": {},
                                "components": [component]});
            validate(&serde_json::from_str(&stored.to_string().replace("\"TIMEOUT\"", timeout)).unwrap())
        };
        assert_eq!(watchdog("-0"), Vec::<String>::new());
        assert_eq!(
            watchdog("0"),
            ["watchdog.wdt_timeout_ms 0 is below the 4000 ms boot floor (the board boot-loops)"]
        );
    }

    #[test]
    fn an_invalid_document_falls_back_with_its_messages() {
        let loaded = load(Some("[1, 2]"), &defaults());
        assert_eq!(loaded.document, defaults());
        assert_eq!(loaded.source, "default(fallback: invalid board.config)");
        assert_eq!(loaded.errors, ["config is not an object"]);
    }

    #[test]
    fn a_valid_document_wins_with_the_running_firmware_version() {
        let stored = document(json!("20261005"));
        let loaded = load(Some(&stored.to_string()), &defaults());
        assert_eq!(loaded.source, "active");
        assert_eq!(loaded.errors, Vec::<String>::new());
        let mut expected = stored.clone();
        expected["board"]["firmware_version"] = json!("running");
        assert_eq!(loaded.document, expected);
        assert_eq!(
            config_id(&loaded.document),
            config_id(&stored),
            "the restamp keeps the id"
        );
    }

    #[test]
    fn defaults_without_a_firmware_version_restamp_dev() {
        let mut running = defaults();
        running["board"].as_object_mut().unwrap().remove("firmware_version");
        let loaded = load(Some(&document(json!("20261005")).to_string()), &running);
        assert_eq!(loaded.document["board"]["firmware_version"], "dev");
        // and defaults with no board section restamp nothing
        let loaded = load(
            Some(&document(json!("20261005")).to_string()),
            &json!({"version": "20261005"}),
        );
        assert_eq!(loaded.document["board"]["firmware_version"], "saved");
    }

    #[test]
    fn an_older_schema_still_runs_and_says_so() {
        let loaded = load(Some(&document(json!("19700101")).to_string()), &defaults());
        assert_eq!(
            loaded.source,
            "active(config 19700101, firmware 20261005 -- re-save to adopt)"
        );
        assert_eq!(loaded.document["board"]["id"], "taster");
    }

    #[test]
    fn outdated_compares_as_python() {
        assert_eq!(outdated(&document(json!("20261005")), &defaults()), None);
        let pair = |config: &str, firmware: &str| Some((config.to_string(), firmware.to_string()));
        assert_eq!(outdated(&json!({}), &defaults()), pair("unversioned", "20261005"));
        assert_eq!(
            outdated(&document(json!(null)), &defaults()),
            pair("unversioned", "20261005")
        );
        assert_eq!(
            outdated(&document(json!(0)), &defaults()),
            pair("unversioned", "20261005")
        );
        assert_eq!(
            outdated(&document(json!(20261005)), &defaults()),
            pair("20261005", "20261005")
        );
        assert_eq!(outdated(&json!({}), &json!({})), None);
        assert_eq!(outdated(&document(json!("1")), &json!({})), pair("1", "unversioned"));
        assert_eq!(outdated(&document(json!(2)), &json!({"version": 2.0})), None);
        assert_eq!(outdated(&json!([1]), &json!(null)), None, "no object, no version");
    }
}
