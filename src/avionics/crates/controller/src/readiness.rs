//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight-readiness gate (doc/specs/avionics-port.md M7, "Arm and verify"): plan B's `_readiness`
//! (cc_client.py:47-107), the config choices that leave the board ballistic or unrecoverable in flight while every
//! part still probes healthy -- the watchdog off, the flight loop off or without a gain on an axis, a bench fin
//! derating still applied, and no landing zone with no CC-less site to pick one from. `verify` reports them as its
//! separate `ready` verdict and leaves the hardware `pass` alone: bench and HITL sessions trip them on purpose.
//!
//! The port, against plan B:
//!  * A component without `enabled` is enabled, as the Controller reads it ([`board_config::directory`]). Plan B's
//!    `_readiness` alone read it as disabled, an inconsistency that is not ported (§6, "`_readiness` and `enabled`").
//!  * Plan B's `fins` check (a control surface whose servo has no driver) reads the flight loop's mixer, which the
//!    port has not built yet. The caller adds it once the mixer exists, as `verify` adds the BNO055's
//!    `imu_calibration` to the same map.
//!  * Plan B raises, so its `verify` answers an error, on two documents validate() admits: a null `fins`, and a
//!    `gains` that is not an object. Here the first is no derating and the second holds no gain on any axis.
//!  * A document validate() refuses is never run (load() falls back to the defaults), but the answer stays defined:
//!    a bool multiplier is Python's int, as plan B prints it (`false` is `0`); any other multiplier that is not a
//!    number is printed as Python's `%s` prints it, a derating all the same, where plan B's `%g` raises; an axis
//!    that is not an object holds no gain; and a name used twice is its first enabled component, as the Controller
//!    lists them, where plan B's dict keeps the last component of the name, enabled or not.

use std::collections::BTreeMap;

use board_config::python::{equals, formatted, truthy};
use board_config::{Entry, Section};
use serde_json::Value;

/// The watchdog's component name, which is also its key in the verdict.
const WATCHDOG: &str = "watchdog";

/// The flight loop's component name, which is also its key in the verdict.
const FLIGHT: &str = "flight";

/// The verdict's key for a flight loop that has no gain on some axis.
const GAINS: &str = "gains";

/// The verdict's key for a bench fin derating: the config path of the knob.
const MULTIPLIER: &str = "fins.limit_multiplier";

/// The verdict's key for a mission with no landing zone and no site to select one from.
const ZONE: &str = "zone";

/// The flight loop's PID axes, in the order plan B lists the ones with no gain.
const AXES: [&str; 3] = ["roll", "pitch", "yaw"];

/// The fin multiplier that derates nothing: plan B's default for an absent key (cc_client.py:82).
const NO_DERATING: f64 = 1.0;

/// The significant digits Python's `%g` writes (its default precision).
const SIGNIFICANT_DIGITS: i32 = 6;

/// The lowest decimal exponent `%g` still writes positionally: `0.0001`, but `1e-05`.
const LOWEST_POSITIONAL_EXPONENT: i32 = -4;

/// The flight-readiness problems of the board config `document`, `{check: problem}`, in plan B's words; empty when
/// the board is flight-ready (plan B's `_readiness`, cc_client.py:47-107).
///
/// `zone_set` and `has_sites` are the mission's: whether it holds a landing zone, and whether it holds CC-less sites
/// to select one from. The checks:
///  * `watchdog` -- no enabled component named `watchdog`: a wedged flight loop never reboots;
///  * `flight` -- no enabled component named `flight`: the fins hold neutral;
///  * `gains` -- an enabled `flight` whose `gains` has no non-zero term (Python's truthiness) on roll, pitch or yaw,
///    naming each such axis: the loop computes but cannot act;
///  * `fins.limit_multiplier` -- the top-level `fins.limit_multiplier` present and unequal to 1.0 under Python's
///    `==` (an int 1 is no derating), printed as Python's `%g` prints it;
///  * `zone` -- neither a zone nor a site.
pub fn readiness(document: &Value, zone_set: bool, has_sites: bool) -> BTreeMap<&'static str, String> {
    let mut problems = BTreeMap::new();
    if component(document, WATCHDOG).is_none() {
        problems.insert(WATCHDOG, "disabled: a wedged flight loop never reboots".to_string());
    }
    match component(document, FLIGHT) {
        None => {
            problems.insert(FLIGHT, "disabled: fins hold neutral -- ballistic".to_string());
        }
        Some(flight) => {
            let slack: Vec<&str> = AXES.into_iter().filter(|axis| !acts(&flight, axis)).collect();
            if !slack.is_empty() {
                problems.insert(
                    GAINS,
                    format!("zero/unset on {} -- the loop computes but cannot act", slack.join(", ")),
                );
            }
        }
    }
    let multiplier = document.get("fins").and_then(|fins| fins.get("limit_multiplier"));
    if let Some(multiplier) = multiplier.filter(|multiplier| !equals(multiplier, &Value::from(NO_DERATING))) {
        problems.insert(
            MULTIPLIER,
            format!("{}: a bench derating is still applied", general(multiplier)),
        );
    }
    if !zone_set && !has_sites {
        problems.insert(
            ZONE,
            "no landing zone set and no CC-less sites to select one from".to_string(),
        );
    }
    problems
}

/// The enabled component of `document` named `name`, as plan B's `_enabled` finds it (cc_client.py:32-44), with a
/// missing `enabled` read as enabled, and the first of them for a name used twice (see the module notes). A sensor of
/// that name is not one.
fn component<'d>(document: &'d Value, name: &str) -> Option<Entry<'d>> {
    board_config::directory(document)
        .into_iter()
        .find(|entry| entry.section == Section::Components && entry.name == name)
}

/// Whether the flight loop's gains on `axis` can act: one of its terms is truthy, as plan B's
/// `any(gains.get(axis, {}).values())` reads them. An absent `gains` or axis holds no term, and so does one that is
/// not an object (see the module notes).
fn acts(flight: &Entry, axis: &str) -> bool {
    flight
        .fields
        .get("gains")
        .and_then(|gains| gains.get(axis))
        .and_then(Value::as_object)
        .is_some_and(|terms| terms.values().any(truthy))
}

/// Python's `'%g' % value` for a JSON number: rounded to six significant digits, half to even on the exact binary
/// value as CPython's dtoa rounds; written positionally while the rounded exponent is from -4 to 5, else as one digit,
/// the rest after a point and a signed exponent of at least two digits; with the zeros that end a fraction dropped,
/// and the point once no fraction is left (`0.5`, `100000`, `1e-05`, `1.23457e+08`, `-0`). An int is formatted as the
/// float nearest it, as Python converts it, and a bool as the int 0 or 1 it is in Python. Any other value (see the
/// module notes) is Python's `%s`.
fn general(value: &Value) -> String {
    let Some(number) = value.as_f64().or_else(|| value.as_bool().map(f64::from)) else {
        return formatted(value);
    };
    let scientific = format!("{number:.precision$e}", precision = (SIGNIFICANT_DIGITS - 1) as usize);
    let (mantissa, exponent) = scientific
        .split_once('e')
        .expect("a float formats as <mantissa>e<exponent>");
    let exponent: i32 = exponent.parse().expect("the exponent is a decimal integer");
    if !(LOWEST_POSITIONAL_EXPONENT..SIGNIFICANT_DIGITS).contains(&exponent) {
        let sign = if exponent < 0 { '-' } else { '+' };
        return format!(
            "{}e{sign}{:02}",
            without_trailing_zeros(mantissa),
            exponent.unsigned_abs()
        );
    }
    let decimals = (SIGNIFICANT_DIGITS - 1 - exponent) as usize;
    without_trailing_zeros(&format!("{number:.decimals$}")).to_string()
}

/// `%g`'s cut of a number's text: the zeros that end its fraction, then the point once no fraction is left
/// (`0.500000` is `0.5`, `1.00000` is `1`); a text with no point is whole as it is (`100000`).
fn without_trailing_zeros(text: &str) -> &str {
    if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.')
    } else {
        text
    }
}

#[cfg(test)]
mod tests {
    use super::{general, readiness};
    use serde_json::{Value, json};

    /// A flight-ready document: an enabled watchdog, and an enabled flight loop with a gain on every axis.
    fn ready() -> Value {
        json!({
            "fins": {"limit_multiplier": 1.0},
            "components": [
                {"name": "watchdog", "activity": "watchdog", "enabled": true},
                {"name": "flight", "activity": "flight", "enabled": true,
                 "gains": {"roll": {"kp": 2.0}, "pitch": {"kp": 1.5}, "yaw": {"kd": 0.1}}},
            ],
        })
    }

    /// The verdict's keys for `document`, with a landing zone set.
    fn keys(document: &Value) -> Vec<&'static str> {
        readiness(document, true, false).into_keys().collect()
    }

    /// The `gains` problem of `document`, with a landing zone set.
    fn gains(document: &Value) -> Option<String> {
        readiness(document, true, false).remove("gains")
    }

    #[test]
    fn a_flight_ready_board_has_no_problem() {
        assert!(readiness(&ready(), true, false).is_empty());
        assert!(
            readiness(&ready(), false, true).is_empty(),
            "a site stands in for the zone"
        );
        assert!(readiness(&ready(), true, true).is_empty());
    }

    #[test]
    fn an_empty_document_trips_every_config_gate() {
        let problems = readiness(&json!({}), false, false);
        assert_eq!(
            problems.into_iter().collect::<Vec<_>>(),
            [
                ("flight", "disabled: fins hold neutral -- ballistic".to_string()),
                ("watchdog", "disabled: a wedged flight loop never reboots".to_string()),
                (
                    "zone",
                    "no landing zone set and no CC-less sites to select one from".to_string()
                ),
            ]
        );
        assert_eq!(
            keys(&Value::Null),
            ["flight", "watchdog"],
            "a document that is not an object"
        );
    }

    #[test]
    fn a_missing_enabled_reads_as_enabled_unlike_plan_b() {
        // plan B's _readiness reads both as disabled (task.get('enabled') is None); the port takes the Controller's
        // meaning, so the golden fixture refuses these documents and this test pins them
        let mut document = ready();
        for component in document["components"].as_array_mut().unwrap() {
            component.as_object_mut().unwrap().remove("enabled");
        }
        assert!(readiness(&document, true, false).is_empty());
        document["components"][1]["gains"] = json!({});
        assert_eq!(keys(&document), ["gains"], "an enabled flight loop's gains are read");
    }

    #[test]
    fn enabled_is_python_s_truthiness() {
        for (enabled, ready_watchdog) in [
            (json!(false), false),
            (json!(0), false),
            (json!(0.0), false),
            (json!(""), false),
            (Value::Null, false),
            (json!([]), false),
            (json!(1), true),
            (json!("no"), true),
            (json!([0]), true),
        ] {
            let mut document = ready();
            document["components"][0]["enabled"] = enabled.clone();
            assert_eq!(
                !keys(&document).contains(&"watchdog"),
                ready_watchdog,
                "enabled {enabled}"
            );
        }
    }

    #[test]
    fn a_component_is_found_by_its_name_in_components_only() {
        let mut document = ready();
        document["components"][0]["name"] = json!("dog");
        assert_eq!(keys(&document), ["watchdog"], "a watchdog by another name is none");
        document["sensors"] = json!([{"name": "watchdog", "activity": "watchdog", "enabled": true}]);
        assert_eq!(keys(&document), ["watchdog"], "a sensor of the name is no component");
        let mut document = ready();
        document["components"][1]["name"] = json!("autopilot");
        assert_eq!(keys(&document), ["flight"], "a flight loop by another name is none");
    }

    #[test]
    fn a_name_used_twice_is_its_first_enabled_component() {
        // validate() refuses a name used twice; plan B's dict keeps the last component of it, enabled or not
        let watchdog = |enabled: bool| json!({"name": "watchdog", "activity": "watchdog", "enabled": enabled});
        for pair in [[false, true], [true, false]] {
            let mut document = ready();
            document["components"][0] = watchdog(pair[0]);
            document["components"].as_array_mut().unwrap().push(watchdog(pair[1]));
            assert_eq!(keys(&document), Vec::<&str>::new(), "watchdogs enabled {pair:?}");
        }
        let mut document = ready();
        document["components"][0] = watchdog(false);
        document["components"].as_array_mut().unwrap().push(watchdog(false));
        assert_eq!(keys(&document), ["watchdog"], "neither enabled");
        // two enabled flight loops: the first one's gains are read
        let mut document = ready();
        let mut slack = document["components"][1].clone();
        slack["gains"] = json!({});
        document["components"].as_array_mut().unwrap().push(slack);
        assert_eq!(gains(&document), None, "the first loop acts");
        document["components"].as_array_mut().unwrap().swap(1, 2);
        assert_eq!(
            gains(&document).as_deref(),
            Some("zero/unset on roll, pitch, yaw -- the loop computes but cannot act"),
            "the first loop has no gain"
        );
    }

    #[test]
    fn a_disabled_flight_loop_is_reported_without_its_gains() {
        let mut document = ready();
        document["components"][1]["enabled"] = json!(false);
        document["components"][1]["gains"] = json!({});
        assert_eq!(keys(&document), ["flight"]);
    }

    #[test]
    fn each_axis_without_a_gain_is_named_in_plan_b_s_order() {
        let mut document = ready();
        document["components"][1]["gains"] = json!({"yaw": {"kp": 0}, "roll": {}, "pitch": {"kp": 1.0}});
        assert_eq!(
            gains(&document).as_deref(),
            Some("zero/unset on roll, yaw -- the loop computes but cannot act")
        );
        for axis in ["roll", "pitch", "yaw"] {
            let mut document = ready();
            document["components"][1]["gains"][axis] = json!({"kp": 0, "ki": 0.0, "kd": false});
            assert_eq!(
                gains(&document),
                Some(format!("zero/unset on {axis} -- the loop computes but cannot act")),
                "{axis}"
            );
            document["components"][1]["gains"].as_object_mut().unwrap().remove(axis);
            assert!(
                gains(&document).is_some_and(|problem| problem.contains(axis)),
                "{axis} absent"
            );
        }
    }

    #[test]
    fn one_truthy_term_lets_an_axis_act() {
        let mut document = ready();
        document["components"][1]["gains"] =
            json!({"roll": {"kp": 0, "kd": -0.1}, "pitch": {"ki": 0.0, "kp": 3}, "yaw": {"kp": 0, "kff": 1}});
        assert_eq!(gains(&document), None);
        // validate() refuses a term that is not a number; plan B's any() reads it by Python's truthiness
        document["components"][1]["gains"] =
            json!({"roll": {"kp": true}, "pitch": {"kp": 0, "kd": "x"}, "yaw": {"kp": [1]}});
        assert_eq!(gains(&document), None, "terms that are not numbers");
    }

    #[test]
    fn a_term_python_reads_as_false_is_no_gain() {
        let mut document = ready();
        document["components"][1]["gains"] =
            json!({"roll": {"kp": ""}, "pitch": {"kp": null, "kd": false}, "yaw": {"kp": [], "ki": {}}});
        assert_eq!(
            gains(&document).as_deref(),
            Some("zero/unset on roll, pitch, yaw -- the loop computes but cannot act")
        );
    }

    #[test]
    fn gains_that_are_not_an_object_hold_no_gain() {
        // validate() admits a gains that is not an object, and plan B's _readiness raises on it
        for not_an_object in [Value::Null, json!("pid"), json!([1, 2, 3]), json!(5)] {
            let mut document = ready();
            document["components"][1]["gains"] = not_an_object.clone();
            assert_eq!(
                gains(&document).as_deref(),
                Some("zero/unset on roll, pitch, yaw -- the loop computes but cannot act"),
                "gains {not_an_object}"
            );
        }
        let mut document = ready();
        document["components"][1]["gains"]["pitch"] = json!(2.0);
        assert_eq!(
            gains(&document).as_deref(),
            Some("zero/unset on pitch -- the loop computes but cannot act"),
            "an axis that is not an object (validate() refuses it)"
        );
    }

    #[test]
    fn a_multiplier_equal_to_one_derates_nothing() {
        for one in [json!(1.0), json!(1), json!(true)] {
            let mut document = ready();
            document["fins"]["limit_multiplier"] = one.clone();
            assert!(readiness(&document, true, false).is_empty(), "{one}");
        }
        let mut document = ready();
        document["fins"].as_object_mut().unwrap().remove("limit_multiplier");
        assert!(
            readiness(&document, true, false).is_empty(),
            "absent: plan B's default 1.0"
        );
        for fins in [Value::Null, json!([0.5]), json!("0.5")] {
            document["fins"] = fins.clone();
            assert!(readiness(&document, true, false).is_empty(), "fins {fins}");
        }
    }

    #[test]
    fn a_derating_is_printed_as_python_s_g() {
        let mut document = ready();
        document["fins"]["limit_multiplier"] = json!(0.5);
        assert_eq!(
            readiness(&document, true, false)["fins.limit_multiplier"],
            "0.5: a bench derating is still applied"
        );
        // validate() refuses a bool; Python's False is the int 0, which plan B's %g prints
        document["fins"]["limit_multiplier"] = json!(false);
        assert_eq!(
            readiness(&document, true, false)["fins.limit_multiplier"],
            "0: a bench derating is still applied"
        );
    }

    #[test]
    fn a_multiplier_that_is_not_a_number_is_still_a_derating() {
        // validate() refuses each of these, and plan B's %g raises on them: the port prints them as %s does
        for (multiplier, printed) in [(json!("1.0"), "1.0"), (Value::Null, "None"), (json!([1.0]), "[1.0]")] {
            let mut document = ready();
            document["fins"]["limit_multiplier"] = multiplier;
            assert_eq!(
                readiness(&document, true, false)["fins.limit_multiplier"],
                format!("{printed}: a bench derating is still applied")
            );
        }
    }

    #[test]
    fn g_writes_six_significant_digits_as_python_does() {
        for (value, printed) in [
            (json!(0.5), "0.5"),
            (json!(1e-05), "1e-05"),
            (json!(123456789), "1.23457e+08"),
            (json!(2), "2"),
            (json!(-0.5), "-0.5"),
            (json!(0), "0"),
            (json!(0.0), "0"),
            (json!(-0.0), "-0"),
            (json!(0.0001), "0.0001"),
            (json!(0.00001234), "1.234e-05"),
            (json!(100000), "100000"),
            (json!(1000000), "1e+06"),
            (json!(999999.5), "1e+06"),
            (json!(99999.95), "99999.9"),
            (json!(9.999995), "10"),
            (json!(1.0000001), "1"),
            (json!(0.99999999), "1"),
            (json!(1234565), "1.23456e+06"),
            (json!(1234575), "1.23458e+06"),
            (json!(1e300), "1e+300"),
            (json!(5e-324), "4.94066e-324"),
            (json!(u64::MAX), "1.84467e+19"),
            (json!(i64::MIN), "-9.22337e+18"),
            (json!(1.23456789), "1.23457"),
            (json!(123.456789), "123.457"),
            (json!(false), "0"),
            (json!(true), "1"),
        ] {
            assert_eq!(general(&value), printed, "{value}");
        }
    }

    #[test]
    fn the_zone_gate_trips_only_with_neither_a_zone_nor_a_site() {
        assert_eq!(
            readiness(&ready(), false, false).into_iter().collect::<Vec<_>>(),
            [(
                "zone",
                "no landing zone set and no CC-less sites to select one from".to_string()
            )]
        );
        assert!(!readiness(&ready(), true, false).contains_key("zone"));
        assert!(!readiness(&ready(), false, true).contains_key("zone"));
    }
}
