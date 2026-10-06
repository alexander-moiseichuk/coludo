//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The drivers' half of src/glider commons.py: the primitives more than one device needs, each next to the plan B
//! name it ports. The flight-math half is in crates/control (`commons`), the stage machine's in crates/sequencer.
//!
//! Plan B's `_upy`/`_opt` variant pairs (bytecode against viper) are a MicroPython speed workaround and are not
//! ported: each primitive is one plain function.
//!
//! * [`sensirion_crc8`] -- the CRC-8 over one two-byte word of a Sensirion (SDP8xx) or TDK (ICP-101xx) frame.
//! * [`id_classify`] -- an identity byte read back after a failed setup, sorted into the most likely wiring fault.
//! * [`python_float`] -- an operator's JSON property read as plan B's `float()` reads it (a builtin, not commons.py,
//!   but every barometer's `update` applies it, so it lives with the other shared primitives).

use serde_json::Value;

/// The CRC-8 over the two data bytes of one Sensirion frame word (commons.sensirion_crc8): polynomial 0x31
/// (x^8 + x^5 + x^4 + 1), seed 0xFF, no reflection, no final XOR. The datasheet's worked example is 0xBEEF -> 0x92.
///
/// The ICP-10111 uses the same CRC over its words, so both drivers check frames with this one function.
pub fn sensirion_crc8(first: u8, second: u8) -> u8 {
    let mut crc: u8 = 0xFF;
    for byte in [first, second] {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x31 } else { crc << 1 };
        }
    }
    crc
}

/// Sort an identity byte read back from a part into an operator-readable wire-level diagnosis (commons.id_classify).
///
/// The deeper 'why' a driver's diagnose answers when its setup failed, so `verify`/`probe` say 'chip-select not
/// asserting' instead of only 'absent / miswired?'. `read` is None when the read itself failed (no I2C ack, an SPI
/// error); `expected` is the part's documented identity byte. The texts are plan B's, word for word: the operator
/// and the dashboard read them on both firmwares.
pub fn id_classify(read: Option<u8>, expected: u8) -> String {
    match read {
        None => "no bus response -- device not acking (absent / unpowered / miswired)".to_string(),
        Some(read) if read == expected => {
            format!("id 0x{read:02X} ok -- device present; setup failed AFTER detect (init / config / timing)")
        }
        Some(0x00) => "id reads 0x00 -- chip-select not asserting, or device unpowered".to_string(),
        Some(0xFF) => "id reads 0xFF -- bus idle-high: no device driving MISO (absent / MISO miswired)".to_string(),
        Some(read) => {
            format!("id reads 0x{read:02X}, expected 0x{expected:02X} -- wrong device on this bus/select (crosswired)")
        }
    }
}

/// An operator's JSON property as plan B's `float(props[key])` reads it, or None where `float()` raised.
///
/// `float()` is more lenient than a JSON number: it takes a numeral STRING ("12.5", with surrounding whitespace,
/// an exponent, a sign) and a bool (True is 1.0), so `{"ground": "12.5"}` set a ground on plan B and must here too.
/// It raises on a non-numeric string, a null, a list and an object.
///
/// Two deliberate differences, both refusals of values `float()` took: a value that is not finite in f32 ("nan",
/// "inf", or a magnitude past f32's range), since a NaN or infinite ground would poison every later elevation --
/// the endgame band, the landing fallback and the launch backup -- and no JSON number can be one, so only a typed
/// string carries it; and CPython's digit-group underscores ("1_000"), which Rust's number parser does not take
/// and no operator types into a ground.
pub fn python_float(value: &Value) -> Option<f32> {
    let number = match value {
        Value::Number(number) => number.as_f64().map(|number| number as f32),
        Value::String(text) => text.trim().parse::<f32>().ok(),
        Value::Bool(flag) => Some(if *flag { 1.0 } else { 0.0 }),
        Value::Null | Value::Array(_) | Value::Object(_) => None,
    };
    number.filter(|number| number.is_finite())
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{id_classify, python_float, sensirion_crc8};

    #[test]
    fn the_datasheet_example() {
        assert_eq!(sensirion_crc8(0xBE, 0xEF), 0x92);
    }

    #[test]
    fn a_flipped_bit_changes_the_crc() {
        // NEGATIVE: every single-bit error in a word is caught (a CRC-8 with this polynomial catches all of them)
        for bit in 0..16 {
            let word: u16 = 0xBEEF ^ (1 << bit);
            assert_ne!(sensirion_crc8((word >> 8) as u8, word as u8), 0x92, "bit {bit}");
        }
    }

    #[test]
    fn the_seed_is_ff_not_zero() {
        // a zero word under a zero seed would CRC to 0; the 0xFF seed makes an all-zero frame fail its check
        assert_ne!(sensirion_crc8(0x00, 0x00), 0x00);
    }

    #[test]
    fn diagnoses_name_the_likely_fault() {
        assert!(id_classify(None, 0xEB).starts_with("no bus response"));
        assert_eq!(
            id_classify(Some(0xEB), 0xEB),
            "id 0xEB ok -- device present; setup failed AFTER detect (init / config / timing)"
        );
        assert!(id_classify(Some(0x00), 0xEB).contains("chip-select not asserting"));
        assert!(id_classify(Some(0xFF), 0xEB).contains("idle-high"));
        assert_eq!(
            id_classify(Some(0xEA), 0xEB),
            "id reads 0xEA, expected 0xEB -- wrong device on this bus/select (crosswired)"
        );
        // a part whose real id is 0x00 is present, not unpowered: the match on `expected` comes first, as in plan B
        assert!(id_classify(Some(0x00), 0x00).contains("ok -- device present"));
    }

    #[test]
    fn a_property_reads_as_plan_b_s_float_reads_it() {
        assert_eq!(python_float(&json!(12.5)), Some(12.5));
        assert_eq!(python_float(&json!(-3)), Some(-3.0));
        // float() takes a numeral string and a bool, which a bare JSON-number read refused
        assert_eq!(python_float(&json!("12.5")), Some(12.5));
        assert_eq!(python_float(&json!(" -1.5e2\n")), Some(-150.0));
        assert_eq!(python_float(&json!("+7")), Some(7.0));
        assert_eq!(python_float(&json!(true)), Some(1.0));
        assert_eq!(python_float(&json!(false)), Some(0.0));
    }

    #[test]
    fn what_float_refuses_and_what_the_port_refuses_besides() {
        // NEGATIVE: what float() raised on
        for refused in [
            json!("high"),
            json!(""),
            json!("12.5 m"),
            json!(null),
            json!([1]),
            json!({"m": 1}),
        ] {
            assert_eq!(python_float(&refused), None, "{refused}");
        }
        // NEGATIVE: what float() took and the port refuses on purpose -- a value no ground may be
        for refused in [
            json!("nan"),
            json!("inf"),
            json!("-Infinity"),
            json!("1e39"),
            json!(1e300),
            json!("1_000"),
        ] {
            assert_eq!(python_float(&refused), None, "{refused}");
        }
    }
}
