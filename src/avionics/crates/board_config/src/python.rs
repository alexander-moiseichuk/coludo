//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Python's value semantics over JSON values, the ones plan B's config.py leans on: `repr()` (the config_id
//! hash and every `%r` in validate's messages), `%s` formatting, truthiness (`x or y`, `if x`) and `==`.
//!
//! A value is what CPython's json.loads makes of a document: an object is a dict, an array a list, a string a
//! str, an integer an int, any other number a float, and true/false/null are True/False/None. The oracle is
//! CPython 3.12, not MicroPython: plan B's board prints non-ASCII text raw and floats in single precision, and
//! the port plan (§6, `config_id` floats) fixes the CPython meaning as the contract.

use serde_json::{Number, Value};

use crate::printable::printable;

/// The decimal point positions CPython's float repr writes positionally: the value is 0.DIGITS times
/// 10^point, and outside these it switches to exponent form (`1e-05`, `1e+16`).
const POSITIONAL_POINTS: std::ops::RangeInclusive<i32> = -3..=16;

/// Python's `repr(value)`.
///
/// A dict prints its members in the map's iteration order, which plan B's dict keeps as the document's
/// order: the same text whenever the document's keys are sorted, as every file plan B and the tools write
/// them (serde_json's default map is sorted; with its `preserve_order` it keeps the document order, as
/// Python does).
pub(crate) fn representation(value: &Value) -> String {
    let mut text = String::new();
    write_representation(&mut text, value);
    text
}

/// Append Python's `repr(value)` to `out` (see [`representation`]).
pub(crate) fn write_representation(out: &mut String, value: &Value) {
    match value {
        Value::Null => out.push_str("None"),
        Value::Bool(true) => out.push_str("True"),
        Value::Bool(false) => out.push_str("False"),
        Value::Number(number) => write_number(out, number),
        Value::String(text) => write_string(out, text),
        Value::Array(items) => {
            out.push('[');
            for (index, item) in items.iter().enumerate() {
                if index > 0 {
                    out.push_str(", ");
                }
                write_representation(out, item);
            }
            out.push(']');
        }
        Value::Object(members) => {
            out.push('{');
            for (index, (key, item)) in members.iter().enumerate() {
                if index > 0 {
                    out.push_str(", ");
                }
                write_string(out, key);
                out.push_str(": ");
                write_representation(out, item);
            }
            out.push('}');
        }
    }
}

/// Python's `'%s' % value`: the text itself for a string, the repr of anything else (a float's str is its
/// repr in Python 3).
pub(crate) fn formatted(value: &Value) -> String {
    match value {
        Value::String(text) => text.clone(),
        other => representation(other),
    }
}

/// Append a JSON number as Python prints it: an int in decimal, a float as its repr ([`write_float`]). This
/// is also how json.dumps writes them.
pub(crate) fn write_number(out: &mut String, number: &Number) {
    match number.as_f64() {
        Some(float) if number.is_f64() => write_float(out, float),
        _ => out.push_str(&number.to_string()),
    }
}

/// Append Python's `repr(value)` of a float.
///
/// The digits are CPython's dtoa's ([`shortest_digits`]). CPython then lays them out by its 'r' rules:
/// positionally while the decimal point falls within [`POSITIONAL_POINTS`] (`0.0001`, `1e+16` is past them),
/// with `.0` when no fraction is left (`5.0`); otherwise as one digit, the rest after a point, and a signed
/// exponent of at least two digits (`1e-05`, `1.5e+16`). The sign stays on a negative zero (`-0.0`).
///
/// `value` is finite: a serde_json number cannot hold an infinity or a NaN (JSON has none).
pub(crate) fn write_float(out: &mut String, value: f64) {
    let (digits, exponent) = shortest_digits(value.abs());
    if value.is_sign_negative() {
        out.push('-');
    }
    let point = exponent + 1;
    if !POSITIONAL_POINTS.contains(&point) {
        out.push_str(&digits[..1]);
        if digits.len() > 1 {
            out.push('.');
            out.push_str(&digits[1..]);
        }
        let sign = if exponent < 0 { '-' } else { '+' };
        out.push_str(&format!("e{sign}{:02}", exponent.unsigned_abs()));
        return;
    }
    let length = digits.len() as i32;
    if point <= 0 {
        out.push_str("0.");
        out.extend(std::iter::repeat_n('0', point.unsigned_abs() as usize));
        out.push_str(&digits);
    } else if point < length {
        out.push_str(&digits[..point as usize]);
        out.push('.');
        out.push_str(&digits[point as usize..]);
    } else {
        out.push_str(&digits);
        out.extend(std::iter::repeat_n('0', (point - length) as usize));
        out.push_str(".0");
    }
}

/// The digits CPython's float repr prints for a finite, non-negative `magnitude`, and the decimal exponent of
/// the first: the fewest digits that read back as the same f64, and of those the closest to it.
///
/// Rust's `{:e}` finds how many digits that takes, but where two candidates are EQUALLY close (the f64 is an
/// exact tie, as 1371010358211522.25 is between ...522.2 and ...522.3) it rounds half up, and CPython's dtoa
/// half to even. So the digits are taken again from `{:.*e}` at that count, Rust's exact mode, which rounds
/// the exact binary value half to even. Should that text not read back as the same f64 (the closest digits
/// can fall outside the narrower rounding interval just above a power of two), the shortest text stands, as
/// it does in dtoa.
fn shortest_digits(magnitude: f64) -> (String, i32) {
    let shortest = format!("{magnitude:e}");
    let count = shortest
        .bytes()
        .take_while(|byte| *byte != b'e')
        .filter(u8::is_ascii_digit)
        .count();
    let closest = format!("{magnitude:.precision$e}", precision = count - 1);
    let chosen = if closest.parse::<f64>() == Ok(magnitude) {
        closest
    } else {
        shortest
    };
    let (mantissa, exponent) = chosen
        .split_once('e')
        .expect("a finite float formats as <digits>e<exponent>");
    let digits = mantissa.chars().filter(char::is_ascii_digit).collect();
    (digits, exponent.parse().expect("the exponent is a decimal integer"))
}

/// Append Python's `repr(text)` of a str, as CPython 3.12 writes it.
///
/// The quote is `"` when the text holds a `'` and no `"`, else `'`. Escaped: the chosen quote and the
/// backslash with a backslash; tab, newline and carriage return as `\t`, `\n`, `\r`; the other ASCII
/// controls and DEL as `\xhh`; and a non-ASCII character CPython does not call printable as `\xhh`, `\uhhhh`
/// or `\Uhhhhhhhh` by its size, in lowercase hex. Everything else is copied as it is.
pub(crate) fn write_string(out: &mut String, text: &str) {
    let quote = if text.contains('\'') && !text.contains('"') {
        '"'
    } else {
        '\''
    };
    out.push(quote);
    for character in text.chars() {
        match character {
            '\\' => out.push_str("\\\\"),
            _ if character == quote => {
                out.push('\\');
                out.push(quote);
            }
            '\t' => out.push_str("\\t"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            ' '..='~' => out.push(character),
            _ if !character.is_ascii() && printable(character) => out.push(character),
            _ => {
                let code = u32::from(character);
                match code {
                    0..=0xff => out.push_str(&format!("\\x{code:02x}")),
                    0x100..=0xffff => out.push_str(&format!("\\u{code:04x}")),
                    _ => out.push_str(&format!("\\U{code:08x}")),
                }
            }
        }
    }
    out.push(quote);
}

/// Python's truthiness, as `x or y` and `if x` read a value: None, False, a zero, and an empty str, list or
/// dict are false; everything else is true.
pub(crate) fn truthy(value: &Value) -> bool {
    match value {
        Value::Null => false,
        Value::Bool(flag) => *flag,
        Value::Number(number) => number.as_f64() != Some(0.0),
        Value::String(text) => !text.is_empty(),
        Value::Array(items) => !items.is_empty(),
        Value::Object(members) => !members.is_empty(),
    }
}

/// A number as Python compares it: a bool is the int 0 or 1, and an int against a float compares exactly
/// (not through a rounded float).
#[derive(Clone, Copy, Debug)]
enum Numeric {
    /// An int (or a bool); every JSON integer fits.
    Integer(i128),
    /// A float.
    Float(f64),
}

impl Numeric {
    /// The number `value` holds for Python's comparisons, or None for a non-number (None, str, list, dict).
    fn of(value: &Value) -> Option<Numeric> {
        match value {
            Value::Bool(flag) => Some(Numeric::Integer(i128::from(*flag))),
            Value::Number(number) => match (number.as_i64(), number.as_u64(), number.as_f64()) {
                (Some(integer), _, _) => Some(Numeric::Integer(i128::from(integer))),
                (None, Some(integer), _) => Some(Numeric::Integer(i128::from(integer))),
                (None, None, float) => float.map(Numeric::Float),
            },
            _ => None,
        }
    }

    /// Python's `==` between two numbers.
    fn equals(self, other: Numeric) -> bool {
        match (self, other) {
            (Numeric::Integer(left), Numeric::Integer(right)) => left == right,
            (Numeric::Float(left), Numeric::Float(right)) => left == right,
            (Numeric::Integer(integer), Numeric::Float(float)) | (Numeric::Float(float), Numeric::Integer(integer)) => {
                float.fract() == 0.0 && float.abs() < 2f64.powi(127) && float as i128 == integer
            }
        }
    }
}

/// Python's `left == right` between two JSON values: numbers (and bools) by value across int and float, str
/// by text, list by items in order, dict by the same keys with equal values; different kinds are unequal.
pub(crate) fn equals(left: &Value, right: &Value) -> bool {
    match (left, right) {
        (Value::Null, Value::Null) => true,
        (Value::String(left), Value::String(right)) => left == right,
        (Value::Array(left), Value::Array(right)) => {
            left.len() == right.len() && left.iter().zip(right).all(|(item, other)| equals(item, other))
        }
        (Value::Object(left), Value::Object(right)) => {
            left.len() == right.len()
                && left
                    .iter()
                    .all(|(key, item)| right.get(key).is_some_and(|other| equals(item, other)))
        }
        _ => match (Numeric::of(left), Numeric::of(right)) {
            (Some(left), Some(right)) => left.equals(right),
            _ => false,
        },
    }
}

/// Whether `value` equals the int `integer` under Python's `==` (a bool or a float can).
pub(crate) fn equals_integer(value: &Value, integer: i64) -> bool {
    Numeric::of(value).is_some_and(|number| number.equals(Numeric::Integer(i128::from(integer))))
}

#[cfg(test)]
mod tests {
    use super::{equals, equals_integer, formatted, representation, truthy};
    use serde_json::{Value, json};

    /// Python's repr of a float, through the JSON value.
    fn float(value: f64) -> String {
        representation(&json!(value))
    }

    #[test]
    fn floats_print_as_python_repr() {
        assert_eq!(float(5.0), "5.0");
        assert_eq!(float(0.0), "0.0");
        assert_eq!(float(-0.0), "-0.0");
        assert_eq!(float(1e-5), "1e-05");
        assert_eq!(float(0.0001), "0.0001");
        assert_eq!(float(0.00011), "0.00011");
        assert_eq!(float(1e16), "1e+16");
        assert_eq!(float(1e15), "1000000000000000.0");
        assert_eq!(float(1234567890123456.0), "1234567890123456.0");
        assert_eq!(float(12345678901234568.0), "1.2345678901234568e+16");
        assert_eq!(float(0.1 + 0.2), "0.30000000000000004");
        assert_eq!(float(-1.5e-7), "-1.5e-07");
        assert_eq!(float(1.7976931348623157e308), "1.7976931348623157e+308");
        assert_eq!(float(5e-324), "5e-324");
        assert_eq!(float(123.456), "123.456");
    }

    #[test]
    fn an_exact_tie_rounds_half_to_even_as_dtoa_does() {
        // the f64s are exactly ...522.25, ...433.125, ...159.8125 and 2**-25: two shortest spellings are equally
        // close, and CPython takes the even one where Rust's `{:e}` rounds half up
        assert_eq!(float(1371010358211522.2), "1371010358211522.2");
        assert_eq!(float(259189143840433.12), "259189143840433.12");
        assert_eq!(float(23219187932159.812), "23219187932159.812");
        assert_eq!(float(2.9802322387695312e-08), "2.9802322387695312e-08");
        assert_eq!(float(-1371010358211522.2), "-1371010358211522.2");
    }

    #[test]
    fn digits_that_do_not_read_back_are_never_printed() {
        // 2**-24 and 2**896: the closest 16 digits lie below the value, outside the rounding interval there (half
        // as wide under a power of two), so the shortest digits that read back stand (CPython prints the same)
        assert_eq!(float(5.960464477539063e-08), "5.960464477539063e-08");
        assert_eq!(float(5.282945311356653e+269), "5.282945311356653e+269");
        for value in [5.960464477539063e-08, 5.282945311356653e+269, 1371010358211522.2] {
            assert_eq!(float(value).parse::<f64>(), Ok(value));
        }
    }

    #[test]
    fn floats_are_not_written_as_rust_or_serde_json_would() {
        // the two spellings a port gets for free and Python never writes
        assert_ne!(float(1e16), "1e16");
        assert_ne!(float(1e-5), "1e-5");
        assert_ne!(float(5.0), "5");
        // ...and `{:e}`'s digits at a tie, rounded half up
        assert_ne!(float(1371010358211522.2), "1371010358211522.3");
        assert_ne!(float(2.9802322387695312e-08), "2.9802322387695313e-08");
    }

    #[test]
    fn integers_print_in_decimal() {
        assert_eq!(representation(&json!(0)), "0");
        assert_eq!(representation(&json!(-7)), "-7");
        assert_eq!(representation(&json!(u64::MAX)), "18446744073709551615");
        assert_eq!(representation(&json!(i64::MIN)), "-9223372036854775808");
    }

    #[test]
    fn strings_pick_their_quote_and_escape_as_python() {
        assert_eq!(representation(&json!("taster")), "'taster'");
        assert_eq!(representation(&json!("it's")), r#""it's""#);
        assert_eq!(representation(&json!(r#"say "hi""#)), r#"'say "hi"'"#);
        assert_eq!(representation(&json!(r#"it's "both""#)), r#"'it\'s "both"'"#);
        assert_eq!(representation(&json!("a\\b")), r"'a\\b'");
        assert_eq!(
            representation(&json!("\t\n\r\u{0}\u{1f}\u{7f}")),
            r"'\t\n\r\x00\x1f\x7f'"
        );
        assert_eq!(representation(&json!("Prés")), "'Prés'");
        assert_eq!(
            representation(&json!("\u{a0}\u{ad}\u{200b}\u{e000}")),
            r"'\xa0\xad\u200b\ue000'"
        );
        assert_eq!(representation(&json!("\u{1f600}\u{e0001}")), "'\u{1f600}\\U000e0001'");
    }

    #[test]
    fn containers_print_with_python_separators() {
        assert_eq!(
            representation(&json!([1, "a", null, true, [], {}])),
            "[1, 'a', None, True, [], {}]"
        );
        assert_eq!(
            representation(&json!({"b": 1.0, "a": [false]})),
            "{'a': [False], 'b': 1.0}"
        );
    }

    #[test]
    fn percent_s_prints_a_string_bare_and_the_rest_as_repr() {
        assert_eq!(formatted(&json!("watchdog")), "watchdog");
        assert_eq!(formatted(&json!(5)), "5");
        assert_eq!(formatted(&Value::Null), "None");
        assert_eq!(formatted(&json!(["a"])), "['a']");
    }

    #[test]
    fn truthiness_follows_python() {
        for falsy in [
            json!(null),
            json!(false),
            json!(0),
            json!(0.0),
            json!(-0.0),
            json!(""),
            json!([]),
            json!({}),
        ] {
            assert!(!truthy(&falsy), "{falsy}");
        }
        for truthy_value in [
            json!(true),
            json!(1),
            json!(-1),
            json!(0.5),
            json!("0"),
            json!([0]),
            json!({"a": 0}),
        ] {
            assert!(truthy(&truthy_value), "{truthy_value}");
        }
    }

    #[test]
    fn equality_follows_python() {
        assert!(equals(&json!(1), &json!(1.0)));
        assert!(equals(&json!(true), &json!(1)));
        assert!(equals(&json!(false), &json!(0.0)));
        assert!(equals(&json!("20261005"), &json!("20261005")));
        assert!(!equals(&json!(20261005), &json!("20261005")), "an int is never a str");
        assert!(!equals(&json!(null), &json!(0)));
        assert!(equals(&json!([1, {"a": 2}]), &json!([1.0, {"a": 2}])));
        assert!(!equals(&json!([1]), &json!([1, 1])));
        assert!(!equals(&json!({"a": 1}), &json!({"b": 1})));
        // int against float compares exactly, not through a rounded float
        assert!(!equals(&json!(9007199254740993_u64), &json!(9007199254740992.0)));
        assert!(equals(&json!(9007199254740992_u64), &json!(9007199254740992.0)));
        assert!(!equals(&json!(1e300), &json!(i64::MAX)));
    }

    #[test]
    fn integer_membership_follows_python() {
        assert!(equals_integer(&json!(2), 2));
        assert!(equals_integer(&json!(2.0), 2));
        assert!(equals_integer(&json!(-0.0), 0));
        assert!(equals_integer(&json!(true), 1));
        assert!(!equals_integer(&json!(2.5), 2));
        assert!(!equals_integer(&json!("2"), 2));
        assert!(!equals_integer(&json!(null), 0));
    }
}
