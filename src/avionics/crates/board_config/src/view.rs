//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The typed, read-only view of a config document: only what the firmware consumes so far (M2): the board
//! id and MCU, the Wi-Fi networks and the CC hub, and the field's range limit. Each reader follows the plan B
//! code that reads the same keys, and is meant for a document [`crate::validate`] accepts (what load()
//! returns); for anything else it answers the nearest safe value, as each one says.
//!
//! Where plan B would carry a value of the wrong type into the radio or the socket and fail there (a text
//! `retry_ms`, a numeric password, a host that is not text), the view reads it as the default or as off; each
//! such reading is named on its function.

use std::fmt;

use serde_json::{Map, Value};

use crate::python::truthy;

/// How long a network waits between two attempts to join it when the config does not say, in milliseconds
/// (drivers/wifi.py `wifi.get('retry_ms', 10000)`).
const DEFAULT_RETRY_MS: u32 = 10_000;

/// The CC hub's TCP port when the config does not say (cc_client.Client `wifi.get('cc_port', 1234)`).
const DEFAULT_HUB_PORT: u16 = 1234;

/// What Debug prints for a network's password when the config gives one.
const REDACTED: &str = "<redacted>";

/// What Debug prints for a network's password when the config gives none.
const NO_PASSWORD: &str = "<empty>";

/// The board's Wi-Fi as plan B's wifi driver and CC link set it up.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Wifi {
    /// Whether the radio is used at all: false when `wifi.policy` is 'disabled', which leaves the radio
    /// untouched for the session (drivers/wifi.py run()).
    pub policy_enabled: bool,
    /// The networks to join, in round-robin order (the first is tried first). Listed even when
    /// `policy_enabled` is false, as plan B's setup() lists them.
    pub networks: Vec<Network>,
    /// Where the CC link dials.
    pub hub: Hub,
    /// The hub's TCP port (`wifi.cc_port`).
    pub hub_port: u16,
}

/// One network the board may join.
///
/// Its Debug never prints the password, only whether there is one: a Wi-Fi view ends up in log lines, panic
/// and assert messages, which reach the console, the recorder and CC's log stream.
#[derive(Clone, PartialEq, Eq)]
pub struct Network {
    /// The SSID, never empty.
    pub ssid: String,
    /// The least time between two attempts on THIS network, in milliseconds (its own backoff clock).
    pub retry_ms: u32,
    /// The config's password for it: the entry's, else the section's, else empty. It is the FALLBACK: plan
    /// B prefers a non-empty first line of `<ssid>.creds`, the firmware the credential store's entry.
    pub password: String,
}

impl fmt::Debug for Network {
    /// The network with its password redacted (see [`Network`]).
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let password = if self.password.is_empty() {
            NO_PASSWORD
        } else {
            REDACTED
        };
        formatter
            .debug_struct("Network")
            .field("ssid", &self.ssid)
            .field("retry_ms", &self.retry_ms)
            .field("password", &format_args!("{password}"))
            .finish()
    }
}

/// Where the CC link dials (tasks/cc_link.py and cc_client.Client).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Hub {
    /// No `wifi.cc_host` (absent or null): the `.1` of whatever subnet the board joins, the Control hub by
    /// convention.
    Gateway,
    /// `wifi.cc_host` is '': CC is off, and the board flies standalone.
    Off,
    /// This host, as configured.
    Host(String),
}

/// The board id (`board.id`), as the hub and CC address the board. Empty when the document has none, where
/// plan B would raise (never for a document validate() accepts).
pub fn board_id(document: &Value) -> String {
    board_text(document, "id")
}

/// The MCU (`board.mcu`). Empty when the document has none (never for a document validate() accepts).
pub fn mcu(document: &Value) -> String {
    board_text(document, "mcu")
}

/// A text member of the `board` section, or empty.
fn board_text(document: &Value, key: &str) -> String {
    document["board"][key].as_str().unwrap_or_default().to_string()
}

/// The Wi-Fi view, following plan B's drivers/wifi.py setup() for the networks and the policy, and
/// tasks/cc_link.py with cc_client.Client for the hub.
///
/// Networks: `wifi.networks` when it is a non-empty list, else `[wifi.ssid]` when that is non-empty, else
/// none. A bare string entry is `{ssid}`. Each entry starts from the section's defaults (policy 'auto',
/// enabled, the section's `retry_ms` or 10000 ms, the section's `password` or empty) and its own
/// keys override them. Note: the entry defaults for `policy` and `enabled` are fixed, NOT the section's: plan
/// B's setup() never reads a top-level `enabled`, and the top-level `policy` only decides `policy_enabled`.
/// Dropped: an entry whose ssid is empty (or not truthy), whose `enabled` is not truthy, or whose policy is
/// 'disabled'.
///
/// Readings where plan B could not join or dial at all (validate() lets these through):
///  * an entry that is neither text nor an object, or a non-list `networks`, is skipped (validate refuses
///    them); an ssid that is not text drops the entry (plan B's connect() raises on it);
///  * `retry_ms`: a number is the least whole number of milliseconds it allows (Python compares the integer
///    tick difference with it), held within 0..=u32::MAX; a bool is 0 or 1; anything else is
///    10000 ms, the default (plan B's comparison raises TypeError at the second attempt);
///  * a password that is not text is empty (MicroPython's connect() takes None as no password, and raises on
///    other types);
///  * `cc_host`: a falsy non-text value (false, 0, [], {}) dials the gateway, as plan B's `host or ...` does;
///    a truthy one is [`Hub::Off`] (plan B dials it and never connects);
///  * `cc_port`: an integer in 0..=65535, or a float equal to one (the typed view's rule for integer knobs);
///    anything else is 1234, the default (plan B never connects with it);
///  * a `wifi` section that is present but not an object (null, which validate accepts, or another kind):
///    plan B's wifi and CC setups both raise on it, so neither runs: the radio is untouched (`policy_enabled`
///    false, no networks) and [`Hub::Off`].
pub fn wifi(document: &Value) -> Wifi {
    let empty = Map::new();
    let section = match document.get("wifi") {
        None => &empty,
        Some(Value::Object(section)) => section,
        Some(_) => {
            return Wifi {
                policy_enabled: false,
                networks: Vec::new(),
                hub: Hub::Off,
                hub_port: DEFAULT_HUB_PORT,
            };
        }
    };
    Wifi {
        policy_enabled: section.get("policy").and_then(Value::as_str) != Some("disabled"),
        networks: networks(section),
        hub: hub(section.get("cc_host")),
        hub_port: hub_port(section.get("cc_port")),
    }
}

/// The networks plan B's setup() keeps, in order (see [`wifi`]).
fn networks(section: &Map<String, Value>) -> Vec<Network> {
    let default_retry = section.get("retry_ms");
    let default_password = section.get("password");
    let fallback_ssid: Vec<Value> = section
        .get("ssid")
        .filter(|ssid| truthy(ssid))
        .map(|ssid| vec![ssid.clone()])
        .unwrap_or_default();
    let entries = match section.get("networks") {
        Some(Value::Array(entries)) if !entries.is_empty() => entries,
        Some(networks) if truthy(networks) => return Vec::new(),
        _ => &fallback_ssid,
    };
    entries
        .iter()
        .filter_map(|entry| {
            let entry: Map<String, Value> = match entry {
                Value::String(_) => Map::from_iter([("ssid".to_string(), entry.clone())]),
                Value::Object(entry) => entry.clone(),
                _ => return None,
            };
            let ssid = entry
                .get("ssid")
                .and_then(Value::as_str)
                .filter(|ssid| !ssid.is_empty())?;
            let enabled = entry.get("enabled").is_none_or(truthy);
            let disabled = entry.get("policy").and_then(Value::as_str) == Some("disabled");
            (enabled && !disabled).then(|| Network {
                ssid: ssid.to_string(),
                retry_ms: retry_ms(entry.get("retry_ms").or(default_retry)),
                password: entry
                    .get("password")
                    .or(default_password)
                    .and_then(Value::as_str)
                    .unwrap_or_default()
                    .to_string(),
            })
        })
        .collect()
}

/// A network's `retry_ms` (see [`wifi`]): absent is [`DEFAULT_RETRY_MS`].
fn retry_ms(value: Option<&Value>) -> u32 {
    match value {
        Some(Value::Bool(flag)) => u32::from(*flag),
        // f64 at the JSON boundary; `as` saturates into 0..=u32::MAX
        Some(Value::Number(number)) => number
            .as_f64()
            .map_or(DEFAULT_RETRY_MS, |milliseconds| milliseconds.ceil() as u32),
        _ => DEFAULT_RETRY_MS,
    }
}

/// The hub from `wifi.cc_host` (see [`wifi`]).
fn hub(host: Option<&Value>) -> Hub {
    match host {
        None | Some(Value::Null) => Hub::Gateway,
        Some(Value::String(host)) if host.is_empty() => Hub::Off,
        Some(Value::String(host)) => Hub::Host(host.clone()),
        Some(other) if !truthy(other) => Hub::Gateway,
        Some(_) => Hub::Off,
    }
}

/// The hub port from `wifi.cc_port` (see [`wifi`]).
fn hub_port(port: Option<&Value>) -> u16 {
    let whole =
        |number: f64| (number.fract() == 0.0 && (0.0..=f64::from(u16::MAX)).contains(&number)).then_some(number as u16);
    match port {
        Some(Value::Number(number)) => number.as_f64().and_then(whole).unwrap_or(DEFAULT_HUB_PORT),
        _ => DEFAULT_HUB_PORT,
    }
}

/// The airframe's glide range, in metres: `max_range_m` on the first sensor or component named 'field'
/// (main.py `config.device(cfg, name='field')`, which the Mission takes as its range limit).
///
/// None when the config does not say (no field device, or no `max_range_m`): the caller then uses the
/// Mission's default, as plan B does (`field_cfg.get('max_range_m', 200)`). Also None for a value that is not
/// a number (null, text, a bool), which plan B would carry into the Mission's range comparisons and fail on.
pub fn max_range_m(document: &Value) -> Option<f32> {
    let devices = ["sensors", "components"]
        .into_iter()
        .filter_map(|section| document.get(section).and_then(Value::as_array))
        .flatten();
    let field = devices
        .filter_map(Value::as_object)
        .find(|device| device.get("name").and_then(Value::as_str) == Some("field"))?;
    // f64 at the JSON boundary, metres in f32 from here on
    field
        .get("max_range_m")
        .and_then(Value::as_f64)
        .map(|metres| metres as f32)
}

#[cfg(test)]
mod tests {
    use super::{DEFAULT_HUB_PORT, DEFAULT_RETRY_MS, Hub, Network, board_id, max_range_m, mcu, wifi};
    use serde_json::{Value, json};

    /// A network as the view lists it.
    fn network(ssid: &str, retry_ms: u32, password: &str) -> Network {
        Network {
            ssid: ssid.to_string(),
            retry_ms,
            password: password.to_string(),
        }
    }

    /// The networks of a document holding just this wifi section.
    fn networks_of(section: Value) -> Vec<Network> {
        wifi(&json!({"wifi": section})).networks
    }

    #[test]
    fn the_board_id_and_mcu_come_from_the_board_section() {
        let document = json!({"board": {"id": "TMS-7F", "mcu": "esp32p4"}});
        assert_eq!(board_id(&document), "TMS-7F");
        assert_eq!(mcu(&document), "esp32p4");
        assert_eq!(board_id(&json!({})), "");
        assert_eq!(mcu(&json!({"board": {"mcu": 5}})), "");
        assert_eq!(board_id(&json!([])), "");
    }

    #[test]
    fn no_wifi_section_dials_the_gateway_with_no_networks() {
        let view = wifi(&json!({}));
        assert!(view.policy_enabled);
        assert_eq!(view.networks, []);
        assert_eq!(view.hub, Hub::Gateway);
        assert_eq!(view.hub_port, DEFAULT_HUB_PORT);
    }

    #[test]
    fn a_wifi_section_that_is_not_an_object_leaves_the_radio_alone() {
        for section in [json!(null), json!([]), json!("panda")] {
            let view = wifi(&json!({"wifi": section}));
            assert!(!view.policy_enabled, "{section}");
            assert_eq!(view.networks, [], "{section}");
            assert_eq!(view.hub, Hub::Off, "{section}");
        }
    }

    #[test]
    fn the_ssid_is_the_fallback_for_no_networks() {
        let expected = [network("field", DEFAULT_RETRY_MS, "")];
        assert_eq!(networks_of(json!({"ssid": "field"})), expected);
        assert_eq!(networks_of(json!({"ssid": "field", "networks": []})), expected);
        assert_eq!(networks_of(json!({"ssid": "field", "networks": null})), expected);
        assert_eq!(networks_of(json!({"ssid": ""})), []);
    }

    #[test]
    fn entries_inherit_the_section_and_override_it() {
        let section = json!({"retry_ms": 2500, "password": "made-up-shared", "networks": [
            "lab", {"ssid": "field", "retry_ms": 500, "password": "made-up-field"}, {"ssid": "car"}]});
        assert_eq!(
            networks_of(section),
            [
                network("lab", 2500, "made-up-shared"),
                network("field", 500, "made-up-field"),
                network("car", 2500, "made-up-shared")
            ]
        );
    }

    #[test]
    fn parked_and_unnamed_entries_are_dropped() {
        let section = json!({"networks": ["", {"ssid": "off", "enabled": false}, {"ssid": "zero", "enabled": 0},
            {"ssid": "null", "enabled": null}, {"ssid": "parked", "policy": "disabled"}, {"ssid": ""},
            {"ssid": 5}, 7, null, {"ssid": "kept", "enabled": "yes", "policy": "auto"}]});
        assert_eq!(networks_of(section), [network("kept", DEFAULT_RETRY_MS, "")]);
    }

    #[test]
    fn the_top_level_policy_and_enabled_do_not_reach_the_entries() {
        let view = wifi(&json!({"wifi": {"policy": "disabled", "enabled": false, "networks": ["lab"]}}));
        assert!(!view.policy_enabled);
        assert_eq!(view.networks, [network("lab", DEFAULT_RETRY_MS, "")]);
    }

    #[test]
    fn retry_ms_reads_as_plan_b_compares_it() {
        let retry = |value: Value| networks_of(json!({"networks": [{"ssid": "lab", "retry_ms": value}]}))[0].retry_ms;
        assert_eq!(retry(json!(1500)), 1500);
        assert_eq!(retry(json!(1500.0)), 1500);
        assert_eq!(retry(json!(1500.25)), 1501, "a tick difference passes 1500.25 at 1501");
        assert_eq!(retry(json!(-5)), 0);
        assert_eq!(retry(json!(1e12)), u32::MAX);
        assert_eq!(retry(json!(true)), 1);
        assert_eq!(retry(json!("1500")), DEFAULT_RETRY_MS);
        assert_eq!(retry(json!(null)), DEFAULT_RETRY_MS);
    }

    #[test]
    fn a_password_that_is_not_text_is_empty() {
        let section = json!({"password": 1234, "networks": [{"ssid": "a"}, {"ssid": "b", "password": null}]});
        assert_eq!(
            networks_of(section),
            [network("a", DEFAULT_RETRY_MS, ""), network("b", DEFAULT_RETRY_MS, "")]
        );
    }

    #[test]
    fn debug_never_prints_a_password() {
        let view = wifi(&json!({"wifi": {"networks": [{"ssid": "lab", "password": "made-up-secret"}, "open"]}}));
        let printed = format!("{view:?}");
        assert!(!printed.contains("made-up-secret"), "{printed}");
        assert!(printed.contains("password: <redacted>"), "{printed}");
        // ...and says when there is none, which is what a failed join needs to know
        assert!(printed.contains("password: <empty>"), "{printed}");
        assert!(printed.contains("\"lab\"") && printed.contains("\"open\""), "{printed}");
        let pretty = format!("{:#?}", view.networks[0]);
        assert!(!pretty.contains("made-up-secret"), "{pretty}");
        // the comparison still sees the password
        assert_ne!(
            view.networks[0],
            network("lab", DEFAULT_RETRY_MS, "another-made-up-secret")
        );
    }

    #[test]
    fn the_hub_follows_cc_host() {
        let hub = |host: Value| wifi(&json!({"wifi": {"cc_host": host}})).hub;
        assert_eq!(hub(json!(null)), Hub::Gateway);
        assert_eq!(hub(json!("")), Hub::Off);
        assert_eq!(hub(json!("192.168.4.1")), Hub::Host("192.168.4.1".to_string()));
        assert_eq!(hub(json!("hub.local")), Hub::Host("hub.local".to_string()));
        assert_eq!(hub(json!(false)), Hub::Gateway);
        assert_eq!(hub(json!(0)), Hub::Gateway);
        assert_eq!(hub(json!(5)), Hub::Off);
        assert_eq!(hub(json!(["x"])), Hub::Off);
    }

    #[test]
    fn the_hub_port_takes_whole_numbers_in_range() {
        let port = |value: Value| wifi(&json!({"wifi": {"cc_port": value}})).hub_port;
        assert_eq!(port(json!(4321)), 4321);
        assert_eq!(port(json!(4321.0)), 4321);
        assert_eq!(port(json!(0)), 0);
        assert_eq!(port(json!(65535)), 65535);
        for refused in [
            json!(65536),
            json!(-1),
            json!(4321.5),
            json!("4321"),
            json!(null),
            json!(true),
        ] {
            assert_eq!(port(refused.clone()), DEFAULT_HUB_PORT, "{refused}");
        }
    }

    #[test]
    fn the_range_limit_is_the_field_devices() {
        let components = |field: Value| json!({"components": [{"name": "flight"}, field]});
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": 200}))),
            Some(200.0)
        );
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": 350.5}))),
            Some(350.5)
        );
        assert_eq!(max_range_m(&components(json!({"name": "field"}))), None);
        assert_eq!(
            max_range_m(&components(json!({"name": "other", "max_range_m": 50}))),
            None
        );
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": "200"}))),
            None
        );
        assert_eq!(
            max_range_m(&components(json!({"name": "field", "max_range_m": true}))),
            None
        );
        assert_eq!(max_range_m(&json!({})), None);
        // sensors are searched first, and the first match wins
        let both = json!({"sensors": [{"name": "field", "max_range_m": 80}],
                          "components": [{"name": "field", "max_range_m": 200}]});
        assert_eq!(max_range_m(&both), Some(80.0));
    }
}
