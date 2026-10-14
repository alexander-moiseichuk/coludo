//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! `write`: the canonical pretty text a board config is stored as. It is CPython's
//! `json.dumps(document, indent=1, sort_keys=True) + '\n'`, the writer of the `launches/<date>/<BOARD>/*.config`
//! files (tools/make_telemetry_config.py), and it reproduces every one of them byte for byte.
//!
//! It is NOT what plan B's own save() writes: commons.atomic_write_json writes `json.dumps(document)`, one
//! line in the document's key order with `, ` and `: ` separators and no newline. Both are the same JSON to
//! every reader (load() and config_id() see the same value), so the board files are the format kept: one key
//! per line, sorted, so a diff shows what moved.
//!
//! The text is pure ASCII, as json.dumps's default `ensure_ascii` makes it: every non-ASCII character and DEL
//! become `\uhhhh` escapes (two, a surrogate pair, past U+FFFF). Numbers print as Python prints them (a float
//! is its repr: `1e-05`, `5.0`).

use serde_json::{Map, Value};

use crate::python::write_number;

/// The spaces one nesting level indents by.
const INDENT: usize = 1;

/// The canonical text of a config document: `json.dumps(document, indent=1, sort_keys=True) + '\n'`.
///
/// Keys are sorted explicitly, so the text is the same whichever map serde_json was built with.
pub fn write(document: &Value) -> String {
    let mut text = String::new();
    write_value(&mut text, document, 0);
    text.push('\n');
    text
}

/// Append `value` at nesting `depth`: its members start on their own lines, indented one level deeper.
fn write_value(out: &mut String, value: &Value, depth: usize) {
    match value {
        Value::Null => out.push_str("null"),
        Value::Bool(flag) => out.push_str(if *flag { "true" } else { "false" }),
        Value::Number(number) => write_number(out, number),
        Value::String(text) => write_string(out, text),
        Value::Array(items) if items.is_empty() => out.push_str("[]"),
        Value::Array(items) => {
            out.push('[');
            for (index, item) in items.iter().enumerate() {
                separate(out, index, depth + 1);
                write_value(out, item, depth + 1);
            }
            new_line(out, depth);
            out.push(']');
        }
        Value::Object(members) if members.is_empty() => out.push_str("{}"),
        Value::Object(members) => {
            out.push('{');
            for (index, (key, item)) in sorted(members).into_iter().enumerate() {
                separate(out, index, depth + 1);
                write_string(out, key);
                out.push_str(": ");
                write_value(out, item, depth + 1);
            }
            new_line(out, depth);
            out.push('}');
        }
    }
}

/// An object's members in Python's `sort_keys` order: by code point, which is the byte order of UTF-8.
pub(crate) fn sorted(members: &Map<String, Value>) -> Vec<(&String, &Value)> {
    let mut sorted: Vec<(&String, &Value)> = members.iter().collect();
    sorted.sort_unstable_by_key(|(key, _)| *key);
    sorted
}

/// Start the member at `index` of a container: a comma after the previous one, then a new line at `depth`.
fn separate(out: &mut String, index: usize, depth: usize) {
    if index > 0 {
        out.push(',');
    }
    new_line(out, depth);
}

/// A new line, indented to `depth`.
fn new_line(out: &mut String, depth: usize) {
    out.push('\n');
    out.extend(std::iter::repeat_n(' ', depth * INDENT));
}

/// Append a JSON string as json.dumps writes it with `ensure_ascii`: `\"`, `\\`, `\b`, `\f`, `\n`, `\r`,
/// `\t`, and `\uhhhh` (lowercase) for every other control, DEL and non-ASCII character.
fn write_string(out: &mut String, text: &str) {
    out.push('"');
    for character in text.chars() {
        match character {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            ' '..='~' => out.push(character),
            _ => {
                let mut units = [0u16; 2];
                for unit in character.encode_utf16(&mut units) {
                    out.push_str(&format!("\\u{unit:04x}"));
                }
            }
        }
    }
    out.push('"');
}

#[cfg(test)]
mod tests {
    use super::write;
    use serde_json::json;

    #[test]
    fn a_document_is_one_sorted_key_per_line() {
        let document = json!({"version": "20261005", "board": {"id": "taster", "rev": 1}, "pins": {}, "list": []});
        let expected = "{\n \"board\": {\n  \"id\": \"taster\",\n  \"rev\": 1\n },\n \"list\": [],\n \"pins\": {},\n \
                        \"version\": \"20261005\"\n}\n";
        assert_eq!(write(&document), expected);
    }

    #[test]
    fn lists_nest_one_space_per_level() {
        assert_eq!(write(&json!([1, [true, null]])), "[\n 1,\n [\n  true,\n  null\n ]\n]\n");
    }

    #[test]
    fn numbers_print_as_python() {
        assert_eq!(
            write(&json!([5.0, 1e-5, 1e16, -0.0, 0.25, 7])),
            "[\n 5.0,\n 1e-05,\n 1e+16,\n -0.0,\n 0.25,\n 7\n]\n"
        );
        // an exact tie between two shortest spellings takes the even one, as json.dumps (the float repr) does
        assert_eq!(
            write(&json!([1371010358211522.2, 259189143840433.12, 23219187932159.812])),
            "[\n 1371010358211522.2,\n 259189143840433.12,\n 23219187932159.812\n]\n"
        );
    }

    #[test]
    fn strings_are_escaped_to_ascii() {
        let text = "q\"b\\\u{8}\u{c}\n\r\t\u{1}\u{7f}é\u{1f600}";
        let expected = "\"q\\\"b\\\\\\b\\f\\n\\r\\t\\u0001\\u007f\\u00e9\\ud83d\\ude00\"\n";
        assert_eq!(write(&json!(text)), expected);
    }

    #[test]
    fn a_scalar_document_is_one_line() {
        assert_eq!(write(&json!(null)), "null\n");
        assert_eq!(write(&json!("x")), "\"x\"\n");
    }

    #[test]
    fn the_text_reads_back_as_the_same_document() {
        let document = json!({"b": [1.5, {"z": "é", "a": -3}], "a": {"nested": [[], {}]}});
        let back: serde_json::Value = serde_json::from_str(&write(&document)).unwrap();
        assert_eq!(back, document);
    }
}
