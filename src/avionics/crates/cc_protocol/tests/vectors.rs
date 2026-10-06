//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The vectors of src/glider/test/test_cc_protocol.py, positive and negative, against the Rust port.

use std::collections::BTreeMap;

fn named(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
    pairs
        .iter()
        .map(|(key, value)| (key.to_string(), value.to_string()))
        .collect()
}

#[test]
fn bare_command() {
    let message = cc_protocol::parse("ping").unwrap();
    assert_eq!(message.command.as_deref(), Some("ping"));
    assert!(message.args.is_empty() && message.named.is_empty());
}

#[test]
fn positional_and_named() {
    let message = cc_protocol::parse("log 3000").unwrap();
    assert_eq!(message.args, ["3000"]);
    let message = cc_protocol::parse("tel ms=3000").unwrap();
    assert_eq!(message.named, named(&[("ms", "3000")]));
    assert!(message.args.is_empty());
}

#[test]
fn simple_values_stay_bare() {
    assert_eq!(cc_protocol::encode("192.168.10.1"), "192.168.10.1");
    assert_eq!(cc_protocol::encode(&3000.to_string()), "3000");
    let token = cc_protocol::encode("pad 7, gusty");
    assert!(token.starts_with("base64:"));
    assert_eq!(cc_protocol::decode(&token).unwrap(), "pad 7, gusty");
    let token = cc_protocol::encode("a=b"); // '=' must not look like a named param
    assert!(token.starts_with("base64:"));
    assert_eq!(cc_protocol::decode(&token).unwrap(), "a=b");
    assert!(cc_protocol::encode("").starts_with("base64:")); // empty is not simple
    assert!(cc_protocol::encode("base64:x").starts_with("base64:")); // nor a lookalike prefix
}

#[test]
fn json_rides_as_base64() {
    let payload = r#"{"board": {"id": "g7a"}, "n": 2}"#;
    let message = cc_protocol::parse(&cc_protocol::build("set-config", &["board", payload], &[])).unwrap();
    assert_eq!(message.command.as_deref(), Some("set-config"));
    assert_eq!(message.args, ["board", payload]);
    let message = cc_protocol::parse(&cc_protocol::build("inspect", &["wifi", "a=b"], &[])).unwrap();
    assert_eq!(message.args, ["wifi", "a=b"]);
}

#[test]
fn command_lowercased_values_keep_case() {
    let message = cc_protocol::parse("STAGE Glider1").unwrap();
    assert_eq!(message.command.as_deref(), Some("stage"));
    assert_eq!(message.args, ["Glider1"]);
}

#[test]
fn responses_parse() {
    let message = cc_protocol::parse("iam taster base64:eyJhIjogMX0=").unwrap();
    assert_eq!(message.args, ["taster", r#"{"a": 1}"#]);
    assert_eq!(cc_protocol::parse("pong").unwrap().command.as_deref(), Some("pong"));
    assert_eq!(cc_protocol::parse("err badcmd nope").unwrap().args, ["badcmd", "nope"]);
}

#[test]
fn empty_line() {
    let message = cc_protocol::parse("   ").unwrap();
    assert_eq!(message.command, None);
    assert!(message.args.is_empty());
}

#[test]
fn build_round_trips() {
    let line = cc_protocol::build("note", &["taster"], &[("msg", "pad 7, gusty")]);
    let message = cc_protocol::parse(&line).unwrap();
    assert_eq!(message.command.as_deref(), Some("note"));
    assert_eq!(message.args, ["taster"]);
    assert_eq!(message.named, named(&[("msg", "pad 7, gusty")]));
}

#[test]
fn bad_base64_is_an_error() {
    assert_eq!(cc_protocol::decode("base64:@@@"), Err(cc_protocol::Error::Base64));
    assert_eq!(cc_protocol::parse("set base64:!!"), Err(cc_protocol::Error::Base64));
    assert_eq!(cc_protocol::decode("base64:/w=="), Err(cc_protocol::Error::Utf8)); // 0xff alone is not UTF-8
}
