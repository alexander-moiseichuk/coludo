//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BLE radio switch (src/glider/drivers/bluetooth.py), a stub in plan B's no-BLE shape (doc/specs/avionics-port.md
//! §6). Plan B's entry declares whether Bluetooth should be on (`radio`, default false) and its setup applies it; on a
//! board with no BLE module its `active` reads None. This firmware starts no BLE radio (the C6 coprocessor's BLE is
//! unused on the glider), so the stub keeps only what the operator sees: the requested `radio`, which `update` changes,
//! beside an `active` that is always null (the firmware's inspect), and a probe that passes, since an absent radio is
//! never in the wrong state.

use board_config::python::equals;
use serde_json::{Map, Value};

/// One `bluetooth` entry: the BLE radio state the operator asks for (plan B's `radio`).
#[derive(Clone, Debug, PartialEq)]
pub struct Bluetooth {
    /// The requested state: the entry's `radio`, kept as plan B keeps it (any JSON value), then each `update`'s.
    radio: Value,
}

impl Bluetooth {
    /// Plan B's setup: the radio state `config` (the entry) declares, its `radio`, false when absent
    /// (`config.get('radio', False)`; an explicit null stays null).
    pub fn from_config(config: &Map<String, Value>) -> Bluetooth {
        Bluetooth {
            radio: config.get("radio").cloned().unwrap_or(Value::Bool(false)),
        }
    }

    /// The requested radio state (`inspect`'s `radio`).
    pub fn radio(&self) -> &Value {
        &self.radio
    }

    /// Plan B's `update`: a `radio` in `properties` unequal to the requested state (Python's `!=`, so an int 1 is
    /// true) becomes it.
    ///
    /// Returns: `["radio"]` when it changed, else nothing (no `radio`, or the same state).
    pub fn update(&mut self, properties: &Map<String, Value>) -> Vec<&'static str> {
        match properties.get("radio") {
            Some(radio) if !equals(radio, &self.radio) => {
                self.radio = radio.clone();
                vec!["radio"]
            }
            _ => Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use serde_json::{Map, Value, json};

    use super::Bluetooth;

    /// The JSON object `value` as a map (a config entry, an update's properties).
    fn object(value: Value) -> Map<String, Value> {
        match value {
            Value::Object(map) => map,
            other => panic!("not an object: {other}"),
        }
    }

    #[test]
    fn the_radio_is_the_entrys_and_off_by_default() {
        assert_eq!(
            Bluetooth::from_config(&object(json!({"name": "bluetooth"}))).radio(),
            &json!(false)
        );
        for radio in [json!(true), json!(false), Value::Null, json!("on")] {
            let entry = object(json!({"name": "bluetooth", "radio": radio}));
            assert_eq!(Bluetooth::from_config(&entry).radio(), &radio, "{radio}");
        }
    }

    #[test]
    fn an_update_to_another_state_changes_the_radio() {
        let mut bluetooth = Bluetooth::from_config(&Map::new());
        assert_eq!(bluetooth.update(&object(json!({"radio": true}))), ["radio"]);
        assert_eq!(bluetooth.radio(), &json!(true));
        assert_eq!(
            bluetooth.update(&object(json!({"radio": false, "other": 1}))),
            ["radio"]
        );
        assert_eq!(bluetooth.radio(), &json!(false));
        // plan B keeps whatever the operator sends
        assert_eq!(bluetooth.update(&object(json!({"radio": "on"}))), ["radio"]);
        assert_eq!(bluetooth.radio(), &json!("on"));
    }

    #[test]
    fn an_update_to_the_same_state_or_without_a_radio_changes_nothing() {
        let mut bluetooth = Bluetooth::from_config(&object(json!({"radio": true})));
        assert!(bluetooth.update(&object(json!({"radio": true}))).is_empty());
        assert!(bluetooth.update(&object(json!({}))).is_empty());
        assert!(bluetooth.update(&object(json!({"active": false}))).is_empty());
        // Python's `1 != True` is False: the same state
        assert!(bluetooth.update(&object(json!({"radio": 1}))).is_empty());
        assert_eq!(bluetooth.radio(), &json!(true));
    }
}
