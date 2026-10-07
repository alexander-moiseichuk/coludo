//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! `validate`: config.validate and every helper it calls, ported line by line, with the same messages in
//! the same order. It checks FILE INTEGRITY only (structure, types, pin uniqueness, bus references, reserved
//! pins, the watchdog floor), never hardware health, which the running board reports.
//!
//! One rule is the port's own, so a document breaking it is the one place the messages differ from plan B's: a
//! device's `debounce_ms` is an int of at most [`LONGEST_DEBOUNCE_MS`] (`Validation::debounce`). The drivers read
//! their config blocks infallibly, with plan B's default for a missing or malformed key, so a value that is
//! well-formed but absurd is refused here, where the operator sees why: at `set-config`, and at boot, where load()
//! then runs the defaults.
//!
//! As in plan B, one pass threads three accumulators through a helper per section ([`Validation`]): the
//! messages, the GPIO owners (uniqueness and reserved pins) and the buses the `buses` section defines (which
//! devices then address).
//!
//! Order: plan B walks each dict in its document order, this port in the map's order. serde_json's default
//! map is sorted, so the two agree on every document whose keys are sorted, which is every file plan B or
//! the tools write (`write`). Where a document's keys are not sorted, the messages are the same set, but two
//! messages may swap places, and of two claims on one GPIO the other one may be named first.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::{Map, Value};

use crate::python::{equals_integer, formatted, representation, truthy};

/// The MCUs a board config may name (config.py `KNOWN_MCUS`).
const KNOWN_MCUS: [&str; 3] = ["esp32p4", "esp32c6", "firebeetle2p4"];

/// The GPIOs a board config must never assign, per MCU: on the WaveShare ESP32-P4-WIFI6 the ESP32-C6 Wi-Fi
/// link (6, 14-19, 54), USB-JTAG (24, 25) and the serial console (37, 38). The other MCUs reserve none
/// (config.py `RESERVED_PINS`).
const RESERVED_PINS: [(&str, &[u64]); 1] = [("esp32p4", &[6, 14, 15, 16, 17, 18, 19, 24, 25, 37, 38, 54])];

/// The lowest hardware watchdog timeout an enabled watchdog may ask for, in milliseconds: below it the board
/// boot-loops (3000 still did, 4000 was stable), and with servos enabled every boot re-centres every fin
/// (config.py `_WDT_FLOOR_MS`).
const WATCHDOG_FLOOR_MS: i64 = 4000;

/// The longest `debounce_ms` a device may ask for, in milliseconds: a port rule, plan B checks none.
///
/// A debounced latch moves no sooner than its debounce after the line settles, and the separation switch's latch is
/// the BOOSTING -> GLIDING trigger: a debounce past a second makes GLIDING come a second or more late, with the
/// glider already off the booster. The driver's default is 10 ms (plan B's config_default still writes 20); the
/// operator's physics (2 cm pads parting at 3 m/s, about 7 ms of wobble) need nothing near this bound, which only
/// refuses a typo or a unit mistake (seconds for ms).
pub const LONGEST_DEBOUNCE_MS: u64 = 1000;

/// The bus spec keys that name a GPIO, in the order plan B claims them.
const BUS_PIN_KEYS: [&str; 7] = ["sda", "scl", "tx", "rx", "sck", "mosi", "miso"];

/// The bus kinds a config may define, in plan B's message order.
const BUS_KINDS: [&str; 3] = ["uart", "i2c", "spi"];

/// The pins a bus kind cannot be constructed without; a missing one would KeyError at bring-up. A UART needs
/// only `tx`: the recorder link is tx-only.
fn required_bus_pins(kind: &str) -> &'static [&'static str] {
    match kind {
        "i2c" => &["scl", "sda"],
        "spi" => &["sck", "mosi", "miso"],
        _ => &["tx"],
    }
}

/// The component fields the control path does arithmetic on, which must be numbers: a quoted number
/// survives save() and load(), then TypeErrors at 100 Hz in flight (config.py `_NUMERIC_FIELDS`).
const NUMERIC_FIELDS: [&str; 15] = [
    "limit_multiplier",
    "trim",
    "still_g",
    "stall_speed_1g",
    "stall_margin",
    "nav_bank_gain",
    "land_bank_gain",
    "loiter_gain",
    "final_cross_gain",
    "pitot_gain",
    "glide_ratio",
    "bank_limit",
    "land_bank_limit",
    "period_ms",
    "telemetry_ms",
];

/// The recorder sizes that must be positive ints.
const RECORDER_POSITIVE_KEYS: [&str; 4] = ["tlm_capacity", "log_capacity", "cell_size", "stats_ms"];

/// Plan B's `_is_int`: an int and not a bool (a JSON true is no number here).
fn is_integer(value: &Value) -> bool {
    matches!(value, Value::Number(number) if !number.is_f64())
}

/// Plan B's `_numeric`: an int or a float, and not a bool.
fn is_numeric(value: &Value) -> bool {
    value.is_number()
}

/// An int greater than zero.
fn is_positive_integer(value: &Value) -> bool {
    is_integer(value) && value.as_u64().is_some_and(|integer| integer > 0)
}

/// An int below zero.
fn is_negative_integer(value: &Value) -> bool {
    is_integer(value) && value.as_i64().is_some_and(|integer| integer < 0)
}

/// A value read as a dict, or None for any other kind.
fn object(value: Option<&Value>) -> Option<&Map<String, Value>> {
    value.and_then(Value::as_object)
}

/// Whether a `mode` read with `.get('mode')` is absent, None or 'sta': Python's `not in ('sta', None)`.
fn is_station_mode(mode: Option<&Value>) -> bool {
    matches!(mode, None | Some(Value::Null)) || mode.and_then(Value::as_str) == Some("sta")
}

/// The three accumulators validate() threads through its section helpers.
#[derive(Default)]
struct Validation {
    /// The messages, in the order found: the return value.
    errors: Vec<String>,
    /// GPIO -> the label that claimed it first.
    pin_owner: BTreeMap<u64, String>,
    /// (kind, id) of every bus the `buses` section defines, ids as the JSON object keys (strings).
    bus_references: BTreeSet<(String, String)>,
    /// Device names, unique across sensors and components.
    seen_names: BTreeSet<String>,
}

impl Validation {
    /// Record a message.
    fn error(&mut self, message: String) {
        self.errors.push(message);
    }

    /// Plan B's `_claim`: record `label`'s claim on a GPIO, or a message for a non-int or negative pin, or for
    /// a GPIO already claimed.
    fn claim(&mut self, label: &str, pin: &Value) {
        // only a non-negative int reads as a u64: a float, a bool or a negative int does not
        let Some(gpio) = pin.as_u64() else {
            self.error(format!(
                "{label} pin must be a non-negative int (got {})",
                representation(pin)
            ));
            return;
        };
        match self.pin_owner.get(&gpio) {
            Some(owner) => {
                let message = format!("GPIO{gpio} used by both {owner} and {label}");
                self.error(message);
            }
            None => {
                self.pin_owner.insert(gpio, label.to_string());
            }
        }
    }

    /// Plan B's `_validate_board`: the `board` section. Returns the mcu when it is a string (known or not),
    /// so the reserved-pin check can run; None skips that check.
    fn board(&mut self, board: Option<&Value>) -> Option<String> {
        let Some(board) = object(board) else {
            self.error("missing or invalid 'board' section".to_string());
            return None;
        };
        match board.get("id").and_then(Value::as_str) {
            None | Some("") => self.error("board.id must be a non-empty string".to_string()),
            Some(id) if id.contains([' ', '\t', '\r', '\n']) => {
                self.error("board.id must not contain whitespace (it is a bare wire token)".to_string());
            }
            Some(_) => {}
        }
        let Some(mcu) = board.get("mcu").and_then(Value::as_str) else {
            self.error("board.mcu must be a string".to_string());
            return None;
        };
        if !KNOWN_MCUS.contains(&mcu) {
            self.error(format!("board.mcu '{mcu}' is not one of {}", KNOWN_MCUS.join(", ")));
        }
        Some(mcu.to_string())
    }

    /// Plan B's `_validate_wifi`: the optional `wifi` section (STA only, a string ssid) and its `networks`,
    /// where a bare string entry is sugar for `{ssid}`.
    fn wifi(&mut self, wifi: Option<&Value>) {
        let Some(wifi) = wifi.filter(|wifi| !wifi.is_null()) else {
            return;
        };
        let Some(wifi) = wifi.as_object() else {
            self.error("'wifi' must be an object".to_string());
            return;
        };
        if !is_station_mode(wifi.get("mode")) {
            self.error("wifi.mode must be 'sta'".to_string());
        }
        if wifi.get("ssid").is_some_and(|ssid| !ssid.is_string()) {
            self.error("wifi.ssid must be a string".to_string());
        }
        let Some(networks) = wifi.get("networks").filter(|networks| !networks.is_null()) else {
            return;
        };
        let Some(networks) = networks.as_array() else {
            self.error("wifi.networks must be a list".to_string());
            return;
        };
        for (index, entry) in networks.iter().enumerate() {
            if entry.is_string() {
                continue;
            }
            let named = entry
                .get("ssid")
                .and_then(Value::as_str)
                .is_some_and(|ssid| !ssid.is_empty());
            if !entry.is_object() || !named {
                self.error(format!("wifi.networks[{index}] needs a string ssid"));
            } else if !is_station_mode(entry.get("mode")) {
                self.error(format!("wifi.networks[{index}].mode must be 'sta'"));
            }
        }
    }

    /// Plan B's `_validate_buses`: claims every bus pin, records every (kind, id) a device may address, and
    /// checks the pins each kind needs and the SPI mode.
    fn buses(&mut self, buses: Option<&Value>) {
        let Some(buses) = object(buses) else {
            self.error("missing or invalid 'buses' section".to_string());
            return;
        };
        for (kind, group) in buses {
            if !BUS_KINDS.contains(&kind.as_str()) {
                self.error(format!("bus type '{kind}' is not one of uart/i2c/spi"));
                continue;
            }
            let Some(group) = group.as_object() else {
                self.error(format!("buses.{kind} must be an object"));
                continue;
            };
            for (identifier, specification) in group {
                self.bus_references.insert((kind.clone(), identifier.clone()));
                let label = format!("{kind}:{identifier}");
                let Some(specification) = specification.as_object() else {
                    self.error(format!("bus {label} must be an object"));
                    continue;
                };
                for key in BUS_PIN_KEYS {
                    if let Some(pin) = specification.get(key) {
                        self.claim(&format!("{label}.{key}"), pin);
                    }
                }
                for key in required_bus_pins(kind) {
                    if !specification.contains_key(*key) {
                        self.error(format!(
                            "bus {label} is missing '{key}' -- it would KeyError at bring-up"
                        ));
                    }
                }
                // absent reads as 3, the runtime default; machine.SPI's polarity and phase are each 0 or 1
                let invalid_mode = specification
                    .get("mode")
                    .filter(|mode| !(0..=3).any(|allowed| equals_integer(mode, allowed)));
                if kind == "spi"
                    && let Some(mode) = invalid_mode
                {
                    self.error(format!("bus {label}.mode must be 0..3 (got {})", representation(mode)));
                }
            }
        }
    }

    /// Plan B's `_validate_pins`: claims each discrete pin. A None or negative pin is a feature wired off,
    /// and claims nothing.
    fn pins(&mut self, pins: Option<&Value>) {
        let Some(pins) = object(pins) else {
            self.error("missing or invalid 'pins' section".to_string());
            return;
        };
        for (name, pin) in pins {
            if pin.is_null() || is_negative_integer(pin) {
                continue;
            }
            self.claim(&format!("pins.{name}"), pin);
        }
    }

    /// Plan B's `_validate_reserved`: every claimed GPIO the mcu reserves for Wi-Fi, USB or the console, in
    /// GPIO order. An mcu without a reserved list checks nothing.
    fn reserved(&mut self, mcu: &str) {
        let Some((_, reserved)) = RESERVED_PINS.iter().find(|(name, _)| *name == mcu) else {
            return;
        };
        let collisions: Vec<String> = self
            .pin_owner
            .iter()
            .filter(|(gpio, _)| reserved.contains(gpio))
            .map(|(gpio, owner)| format!("{owner} uses reserved GPIO{gpio} (breaks Wi-Fi/USB/console on {mcu})"))
            .collect();
        self.errors.extend(collisions);
    }

    /// Plan B's `_validate_recorder`: the optional `recorder` section's sizes are positive ints, and
    /// `telemetry_ms` a non-negative one (0, the shipped value, means no decimation).
    fn recorder(&mut self, recorder: Option<&Value>) {
        let Some(recorder) = recorder.filter(|recorder| !recorder.is_null()) else {
            return;
        };
        let Some(recorder) = recorder.as_object() else {
            self.error("'recorder' must be an object".to_string());
            return;
        };
        for key in RECORDER_POSITIVE_KEYS {
            if recorder.get(key).is_some_and(|value| !is_positive_integer(value)) {
                self.error(format!("recorder.{key} must be a positive int"));
            }
        }
        let telemetry = recorder.get("telemetry_ms");
        if telemetry.is_some_and(|value| !(is_integer(value) && value.as_u64().is_some())) {
            self.error("recorder.telemetry_ms must be a non-negative int (0 = no decimation)".to_string());
        }
    }

    /// Plan B's `_validate_numeric`: the fields the control path multiplies are numbers, and so are the
    /// component's own `fins.limit_multiplier` and every PID gain.
    fn numeric(&mut self, component: &Map<String, Value>, label: &str) {
        for key in NUMERIC_FIELDS {
            if let Some(value) = component.get(key).filter(|value| !is_numeric(value)) {
                self.error(format!(
                    "{label}.{key} must be a number (got {})",
                    representation(value)
                ));
            }
        }
        let multiplier = object(component.get("fins")).and_then(|fins| fins.get("limit_multiplier"));
        if let Some(multiplier) = multiplier.filter(|value| !is_numeric(value)) {
            let got = representation(multiplier);
            self.error(format!("{label}.fins.limit_multiplier must be a number (got {got})"));
        }
        let Some(gains) = object(component.get("gains")) else {
            return;
        };
        for (axis, terms) in gains {
            let Some(terms) = terms.as_object() else {
                self.error(format!("{label}.gains.{axis} must be an object"));
                continue;
            };
            for (term, value) in terms.iter().filter(|(_, value)| !is_numeric(value)) {
                let got = representation(value);
                self.error(format!("{label}.gains.{axis}.{term} must be a number (got {got})"));
            }
        }
    }

    /// Plan B's `_validate_rate_band`: the governor's airspeed rate band is positive ints and ordered, or it
    /// divides by zero at boot (or throttles the wrong end).
    fn rate_band(&mut self, component: &Map<String, Value>, label: &str) {
        let floor = component.get("airspeed_floor_hz").filter(|value| !value.is_null());
        let ceiling = component.get("airspeed_ceiling_hz").filter(|value| !value.is_null());
        for (key, value) in [("airspeed_floor_hz", floor), ("airspeed_ceiling_hz", ceiling)] {
            if let Some(value) = value.filter(|value| !is_positive_integer(value)) {
                self.error(format!(
                    "{label}.{key} must be a positive int (got {})",
                    representation(value)
                ));
            }
        }
        let positive = |value: Option<&Value>| value.filter(|value| is_positive_integer(value)).and_then(Value::as_u64);
        if let (Some(floor), Some(ceiling)) = (positive(floor), positive(ceiling))
            && floor > ceiling
        {
            self.error(format!(
                "{label}.airspeed_floor_hz {floor} exceeds airspeed_ceiling_hz {ceiling}"
            ));
        }
    }

    /// Plan B's `_validate_devices`: a `sensors` or `components` list. Each entry is an object with a
    /// unique non-empty name (across both lists) that names its implementation (`driver` or `activity`),
    /// with a bool `enabled`, numeric control fields, a defined bus, an int or None `addr`, and well-formed
    /// `provides`.
    fn devices(&mut self, items: Option<&Value>, section: &str) {
        let Some(items) = items.filter(|items| !items.is_null()) else {
            return;
        };
        let Some(items) = items.as_array() else {
            self.error(format!("'{section}' must be a list"));
            return;
        };
        for (index, device) in items.iter().enumerate() {
            let mut label = format!("{section}[{index}]");
            let Some(device) = device.as_object() else {
                self.error(format!("{label} must be an object"));
                continue;
            };
            match device.get("name").and_then(Value::as_str) {
                None | Some("") => self.error(format!("{label}.name must be a non-empty string")),
                Some(name) => {
                    label = format!("{section} '{name}'");
                    if !self.seen_names.insert(name.to_string()) {
                        self.error(format!("duplicate device name '{name}'"));
                    }
                }
            }
            // `driver or activity`: the driver when it is truthy, else the activity
            let runs = device
                .get("driver")
                .filter(|driver| truthy(driver))
                .or(device.get("activity"));
            if !runs.and_then(Value::as_str).is_some_and(|runs| !runs.is_empty()) {
                self.error(format!(
                    "{label} must name an implementation: `driver` (drivers/) or `activity` (tasks/)"
                ));
            }
            if device.get("enabled").is_some_and(|enabled| !enabled.is_boolean()) {
                self.error(format!("{label}.enabled must be a bool"));
            }
            self.rate_band(device, &label);
            self.numeric(device, &label);
            self.debounce(device, &label);
            self.bus(device, &label);
            if device
                .get("addr")
                .is_some_and(|address| !address.is_null() && !is_integer(address))
            {
                self.error(format!("{label}.addr must be an int or null"));
            }
            self.provides(device.get("provides"), &label);
        }
    }

    /// A device's `debounce_ms`, when it has one: an int of 0..=[`LONGEST_DEBOUNCE_MS`] (the port's own rule). Any
    /// device's, by the key: the debounce behind it is the drivers' one generic debounce (the separation switch's
    /// today). Plan B took any value, and its settle raised on a float, text or null.
    fn debounce(&mut self, device: &Map<String, Value>, label: &str) {
        let Some(debounce) = device.get("debounce_ms") else {
            return;
        };
        let sane = is_integer(debounce)
            && debounce
                .as_u64()
                .is_some_and(|debounce| debounce <= LONGEST_DEBOUNCE_MS);
        if !sane {
            self.error(format!(
                "{label}.debounce_ms must be an int from 0 to {LONGEST_DEBOUNCE_MS} ms (got {})",
                representation(debounce)
            ));
        }
    }

    /// The bus a device addresses, by kind (`bus`) and int id (`id`): it must be one `buses` defines.
    fn bus(&mut self, device: &Map<String, Value>, label: &str) {
        let Some(kind) = device.get("bus").filter(|kind| !kind.is_null()) else {
            return;
        };
        let identifier = device.get("id").unwrap_or(&Value::Null);
        let Some(kind) = kind.as_str() else {
            self.error(format!("{label}.bus must be a string (the bus kind, e.g. \"i2c\")"));
            return;
        };
        if !is_integer(identifier) {
            self.error(format!("{label}.id must be the int bus id (with bus \"{kind}\")"));
            return;
        }
        let reference = (kind.to_string(), identifier.to_string());
        if !self.bus_references.contains(&reference) {
            self.error(format!(
                "{label} addresses bus {kind}:{identifier} which is not defined"
            ));
        }
    }

    /// A device's `provides`: an object of quantity -> {priority, timeout_ms}, both ints.
    fn provides(&mut self, provides: Option<&Value>, label: &str) {
        let Some(provides) = provides.filter(|provides| !provides.is_null()) else {
            return;
        };
        let Some(provides) = provides.as_object() else {
            self.error(format!("{label}.provides must be an object"));
            return;
        };
        for (quantity, specification) in provides {
            let Some(specification) = specification.as_object() else {
                self.error(format!("{label}.provides.{quantity} must be an object"));
                continue;
            };
            for key in ["priority", "timeout_ms"] {
                if !specification.get(key).is_some_and(is_integer) {
                    self.error(format!("{label}.provides.{quantity}.{key} must be an int"));
                }
            }
        }
    }

    /// Plan B's `_validate_watchdog`: an enabled watchdog component must not time out below
    /// [`WATCHDOG_FLOOR_MS`]. Absent `enabled` reads as enabled and absent `wdt_timeout_ms` as the floor;
    /// a bool timeout is an int here, as in Python, and a float one is not checked.
    fn watchdog(&mut self, components: Option<&Value>) {
        let Some(components) = components.and_then(Value::as_array) else {
            return;
        };
        for device in components.iter().filter_map(Value::as_object) {
            if device.get("activity").and_then(Value::as_str) != Some("watchdog") {
                continue;
            }
            let enabled = device.get("enabled").is_none_or(truthy);
            let timeout = match device.get("wdt_timeout_ms") {
                Some(Value::Bool(flag)) => Some(i64::from(*flag)),
                Some(value) if is_integer(value) => value.as_i64(),
                _ => None,
            };
            if let Some(timeout) = timeout.filter(|timeout| enabled && *timeout < WATCHDOG_FLOOR_MS) {
                let name = device.get("name").map_or_else(|| "watchdog".to_string(), formatted);
                self.error(format!(
                    "{name}.wdt_timeout_ms {timeout} is below the {WATCHDOG_FLOOR_MS} ms boot floor (the board \
                     boot-loops)"
                ));
            }
        }
    }
}

/// Validate a config document for FILE INTEGRITY: config.validate, with the same messages in the same order.
///
/// Returns the messages; empty means valid. A document that is not an object gets the single message
/// `config is not an object`. Never panics: every value of every kind is a message or nothing, which is also
/// why plan B's `validation raised` fallback in load() has no counterpart here (no JSON document reaches it).
pub fn validate(document: &Value) -> Vec<String> {
    let Some(config) = document.as_object() else {
        return vec!["config is not an object".to_string()];
    };
    let mut validation = Validation::default();
    let mcu = validation.board(config.get("board"));
    validation.wifi(config.get("wifi"));
    validation.buses(config.get("buses"));
    validation.pins(config.get("pins"));
    if let Some(mcu) = &mcu {
        validation.reserved(mcu);
    }
    validation.recorder(config.get("recorder"));
    // `fins` is a top-level section (flight.py reads board.get('fins', {})), so the component sweep misses it
    match config.get("fins") {
        Some(fins) if !fins.is_null() && !fins.is_object() => {
            validation.error("fins is not an object".to_string());
        }
        fins => validation.numeric(object(fins).unwrap_or(&Map::new()), "fins"),
    }
    validation.devices(config.get("sensors"), "sensors");
    validation.devices(config.get("components"), "components");
    validation.watchdog(config.get("components"));
    validation.errors
}

#[cfg(test)]
mod tests {
    use super::validate;
    use serde_json::{Value, json};

    /// A small valid document: a board, one bus of each kind, two pins and one device on each list.
    fn valid() -> Value {
        json!({
            "board": {"id": "taster", "mcu": "esp32p4"},
            "buses": {"i2c": {"0": {"scl": 8, "sda": 7}}, "spi": {"1": {"sck": 48, "mosi": 47, "miso": 46}},
                      "uart": {"1": {"tx": 20}}},
            "pins": {"servo_yaw": 26, "laser_int": null},
            "sensors": [{"name": "baro", "driver": "icp10111", "bus": "i2c", "id": 0, "addr": 99,
                         "provides": {"altitude": {"priority": 0, "timeout_ms": 200}}}],
            "components": [{"name": "watchdog", "activity": "watchdog", "enabled": true, "wdt_timeout_ms": 5000}],
        })
    }

    /// The messages for `valid()` after `edit`.
    fn errors_after(edit: impl FnOnce(&mut Value)) -> Vec<String> {
        let mut document = valid();
        edit(&mut document);
        validate(&document)
    }

    #[test]
    fn a_valid_document_has_no_messages() {
        assert_eq!(validate(&valid()), Vec::<String>::new());
    }

    #[test]
    fn the_camera_entry_is_a_valid_device() {
        // board-config.md's example: no bus, and `clock_hz` is none of the numeric control fields
        let errors = errors_after(|document| {
            document["components"]
                .as_array_mut()
                .expect("a components list")
                .push(json!(
                    {"name": "camera", "driver": "ov5647", "enabled": true, "clock_hz": 27_000_000}
                ));
        });
        assert_eq!(errors, Vec::<String>::new());
    }

    #[test]
    fn a_non_object_is_one_message() {
        for document in [json!(null), json!([]), json!("board"), json!(5), json!(true)] {
            assert_eq!(validate(&document), ["config is not an object"], "{document}");
        }
    }

    #[test]
    fn missing_sections_are_named() {
        assert_eq!(
            validate(&json!({})),
            [
                "missing or invalid 'board' section",
                "missing or invalid 'buses' section",
                "missing or invalid 'pins' section"
            ]
        );
    }

    #[test]
    fn a_collision_names_the_first_claim() {
        let errors = errors_after(|document| document["pins"]["servo_yaw"] = json!(7));
        assert_eq!(errors, ["GPIO7 used by both i2c:0.sda and pins.servo_yaw"]);
    }

    #[test]
    fn a_reserved_pin_is_refused_on_the_p4_only() {
        let errors = errors_after(|document| document["pins"]["servo_yaw"] = json!(18));
        assert_eq!(
            errors,
            ["pins.servo_yaw uses reserved GPIO18 (breaks Wi-Fi/USB/console on esp32p4)"]
        );
        let errors = errors_after(|document| {
            document["pins"]["servo_yaw"] = json!(18);
            document["board"]["mcu"] = json!("esp32c6");
        });
        assert_eq!(errors, Vec::<String>::new());
    }

    #[test]
    fn a_pin_value_is_quoted_as_python_repr() {
        let errors = errors_after(|document| document["pins"]["servo_yaw"] = json!("26"));
        assert_eq!(errors, ["pins.servo_yaw pin must be a non-negative int (got '26')"]);
        let errors = errors_after(|document| document["pins"]["servo_yaw"] = json!(26.0));
        assert_eq!(errors, ["pins.servo_yaw pin must be a non-negative int (got 26.0)"]);
        let errors = errors_after(|document| document["buses"]["i2c"]["0"]["sda"] = json!(true));
        assert_eq!(errors, ["i2c:0.sda pin must be a non-negative int (got True)"]);
    }

    #[test]
    fn disabled_pins_claim_nothing() {
        let errors = errors_after(|document| {
            document["pins"]["laser_xshut"] = json!(-7);
            document["pins"]["laser_int"] = json!(null);
        });
        assert_eq!(errors, Vec::<String>::new());
    }

    #[test]
    fn the_spi_mode_reads_as_python_membership() {
        for mode in [json!(0), json!(3), json!(2.0), json!(true), json!(-0.0)] {
            let errors = errors_after(|document| document["buses"]["spi"]["1"]["mode"] = mode.clone());
            assert_eq!(errors, Vec::<String>::new(), "{mode}");
        }
        let errors = errors_after(|document| document["buses"]["spi"]["1"]["mode"] = json!(4));
        assert_eq!(errors, ["bus spi:1.mode must be 0..3 (got 4)"]);
        let errors = errors_after(|document| document["buses"]["spi"]["1"]["mode"] = json!(null));
        assert_eq!(errors, ["bus spi:1.mode must be 0..3 (got None)"]);
    }

    #[test]
    fn the_watchdog_floor_holds_for_an_enabled_watchdog_only() {
        let below = "watchdog.wdt_timeout_ms 1000 is below the 4000 ms boot floor (the board boot-loops)";
        let errors = errors_after(|document| document["components"][0]["wdt_timeout_ms"] = json!(1000));
        assert_eq!(errors, [below]);
        let errors = errors_after(|document| {
            document["components"][0]["wdt_timeout_ms"] = json!(1000);
            document["components"][0]["enabled"] = json!(false);
        });
        assert_eq!(errors, Vec::<String>::new());
        let errors = errors_after(|document| document["components"][0]["wdt_timeout_ms"] = json!(4000));
        assert_eq!(errors, Vec::<String>::new());
        // a bool is an int to Python's isinstance, a float is not checked at all
        let errors = errors_after(|document| document["components"][0]["wdt_timeout_ms"] = json!(true));
        assert_eq!(
            errors,
            ["watchdog.wdt_timeout_ms 1 is below the 4000 ms boot floor (the board boot-loops)"]
        );
        let errors = errors_after(|document| document["components"][0]["wdt_timeout_ms"] = json!(1000.0));
        assert_eq!(errors, Vec::<String>::new());
    }

    #[test]
    fn an_implementation_is_the_driver_or_else_the_activity() {
        let errors = errors_after(|document| document["sensors"][0]["driver"] = json!(""));
        assert_eq!(
            errors,
            ["sensors 'baro' must name an implementation: `driver` (drivers/) or `activity` (tasks/)"]
        );
        let errors = errors_after(|document| {
            document["sensors"][0]["driver"] = json!("");
            document["sensors"][0]["activity"] = json!("baro");
        });
        assert_eq!(errors, Vec::<String>::new());
    }

    #[test]
    fn a_debounce_up_to_the_bound_is_valid() {
        // the shipped 20, the edges of the range, and the key on any device, sensor or component
        for debounce in [json!(20), json!(0), json!(1000)] {
            let errors = errors_after(|document| {
                document["components"]
                    .as_array_mut()
                    .expect("a components list")
                    .push(json!({"name": "separation", "driver": "separation", "debounce_ms": debounce.clone()}));
                document["sensors"][0]["debounce_ms"] = debounce.clone();
            });
            assert_eq!(errors, Vec::<String>::new(), "{debounce}");
        }
        assert_eq!(super::LONGEST_DEBOUNCE_MS, 1000);
    }

    #[test]
    fn a_debounce_past_the_bound_or_not_an_int_is_refused() {
        // NEGATIVE: one past the bound, a seconds-for-ms slip, a negative, and every non-int (a bool is no int here,
        // as in plan B's _is_int), each named on its device as Python would print it
        for (debounce, got) in [
            (json!(1001), "1001"),
            (json!(20_000), "20000"),
            (json!(-1), "-1"),
            (json!(20.0), "20.0"),
            (json!("20"), "'20'"),
            (json!(true), "True"),
            (Value::Null, "None"),
            (json!(u64::MAX), "18446744073709551615"),
        ] {
            let errors = errors_after(|document| {
                document["components"]
                    .as_array_mut()
                    .expect("a components list")
                    .push(json!({"name": "separation", "driver": "separation", "debounce_ms": debounce.clone()}));
            });
            assert_eq!(
                errors,
                [format!(
                    "components 'separation'.debounce_ms must be an int from 0 to 1000 ms (got {got})"
                )],
                "{debounce}"
            );
        }
        // after the numeric fields, before the bus, as the device's own messages go
        let errors = errors_after(|document| {
            document["sensors"][0]["debounce_ms"] = json!(5000);
            document["sensors"][0]["period_ms"] = json!("10");
            document["sensors"][0]["id"] = json!(9);
        });
        assert_eq!(
            errors,
            [
                "sensors 'baro'.period_ms must be a number (got '10')",
                "sensors 'baro'.debounce_ms must be an int from 0 to 1000 ms (got 5000)",
                "sensors 'baro' addresses bus i2c:9 which is not defined",
            ]
        );
    }

    #[test]
    fn a_device_must_address_a_defined_bus() {
        let errors = errors_after(|document| document["sensors"][0]["id"] = json!(9));
        assert_eq!(errors, ["sensors 'baro' addresses bus i2c:9 which is not defined"]);
        let errors = errors_after(|document| document["sensors"][0]["id"] = json!(0.0));
        assert_eq!(errors, ["sensors 'baro'.id must be the int bus id (with bus \"i2c\")"]);
    }
}
