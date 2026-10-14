//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Wire conformance: the Rust port and src/glider/cc_protocol.py must produce the same bytes.
//!
//! Runs the Python module under CPython 3 (the host side of the shared protocol) over a set of awkward
//! values and compares every encode() and build() token by token, then has each side parse the other's
//! lines.

use std::io::Write;
use std::process::{Command, Stdio};

/// Values chosen to walk every branch of the bare/base64 decision.
const VALUES: &[&str] = &[
    "taster",
    "192.168.102.1",
    "-12.5",
    "a=b",
    "=leading",
    "pad 7, gusty",
    "",
    "base64:lookalike",
    "{\"board\": {\"id\": \"g7a\"}}",
    "ümlaut",
    "tab\there",
    "path/to+file:1_2-3.csv",
    "semi;colon",
];

/// Run `script` under python3 with src/glider importable, feeding `input` on stdin; returns stdout.
fn python(script: &str, input: &str) -> String {
    let glider = concat!(env!("CARGO_MANIFEST_DIR"), "/../../../glider");
    let mut child = Command::new("python3")
        .arg("-c")
        .arg(format!(
            "import sys; sys.path.insert(0, {glider:?}); import cc_protocol as cc\n{script}"
        ))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .expect("python3 is part of the host toolchain");
    child.stdin.take().unwrap().write_all(input.as_bytes()).unwrap();
    let output = child.wait_with_output().unwrap();
    assert!(output.status.success(), "python3 failed");
    String::from_utf8(output.stdout).unwrap()
}

#[test]
fn encode_matches_python() {
    // one value per line can not carry a tab or newline safely, so ship them as JSON
    let input: Vec<String> = VALUES.iter().map(|value| format!("{value:?}")).collect();
    let script = "import json\nfor line in sys.stdin.read().splitlines(): print(cc.encode(json.loads(line)))";
    let expected = python(script, &input.join("\n"));
    let actual: Vec<String> = VALUES.iter().map(|value| cc_protocol::encode(value)).collect();
    assert_eq!(expected.lines().collect::<Vec<_>>(), actual);
}

#[test]
fn lines_cross_parse() {
    let line = cc_protocol::build("note", VALUES, &[("msg", "pad 7, gusty"), ("ms", "3000")]);
    let script = "import json\nmsg = cc.parse(sys.stdin.read())\nprint(json.dumps([msg.command, msg.args, msg.named], ensure_ascii=False))\n\
                  print(cc.build('note', msg.args, msg.named))";
    let output = python(script, &line);
    let mut lines = output.lines();
    let parsed = lines.next().unwrap();
    let rebuilt = lines.next().unwrap();
    for value in VALUES {
        let quoted = format!("{value:?}"); // Rust Debug and JSON quote these values alike
        assert!(
            parsed.contains(&quoted) || value.is_empty(),
            "python lost {value:?} in {parsed}"
        );
    }
    assert_eq!(rebuilt, line, "python rebuilt a different line");
    assert_eq!(cc_protocol::parse(rebuilt).unwrap().args, VALUES);
}
