//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The two laser rangers answer as plan B's do: src/avionics/golden/drivers_vl53.json, which
//! golden/build_drivers_vl53.py builds from src/glider/drivers/vl53l1x.py and vl53l4cx.py on CPython. The default
//! configuration blocks byte for byte, the L4CX timing-budget words, the range-status map, and the I2C transcripts
//! of setup (per silicon, the other silicon refused), probe and diagnose, replayed through the mock bus. Two
//! deliberate divergences -- the data-ready read before every range, the L4CX probe's full 16-bit id -- are the
//! fixture's own `divergence` members, and the port owes those.

mod transcript;

use drivers::device::Device;
use drivers::mock::MockPin;
use drivers::vl53l1x::{self, Settings, Silicon, Vl53};
use drivers::vl53l4cx;
use golden::{Compare, hex};
use serde_json::{Value, json};
use transcript::{check_cases, replay, section};

/// plan B's two VL53 drivers over the default_configuration, timing_budget, status, setup, probe and diagnose
/// sections.
const VL53: &str = include_str!("../../../golden/drivers_vl53.json");

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(VL53, "drivers_vl53", name)
}

/// The silicon a case names by its driver.
fn silicon(input: &Value) -> &'static Silicon {
    match input["silicon"].as_str() {
        Some("vl53l1x") => &vl53l1x::VL53L1X,
        Some("vl53l4cx") => &vl53l4cx::VL53L4CX,
        other => panic!("unknown silicon {other:?}"),
    }
}

/// The transcript and answer a case owes: its divergence's when it has one, else plan B's.
fn owed<'a>(case: &'a Value, member: &str) -> (&'a Value, Value) {
    match case.get("divergence") {
        Some(divergence) => {
            assert!(divergence["note"].as_str().is_some_and(|note| !note.is_empty()));
            (&divergence["script"], json!({member: divergence[member]}))
        }
        None => (&case["input"]["script"], case["output"].clone()),
    }
}

#[test]
fn the_default_configuration_blocks_byte_for_byte() {
    check_cases(&part("default_configuration"), "default_configuration", |input| {
        json!(hex(silicon(input).default_configuration))
    });
    // the L1X block ends on SYSTEM__MODE_START stopped, and the L4CX's too: ranging is started explicitly
    assert_eq!(vl53l1x::DEFAULT_CONFIGURATION[90], 0x00);
    assert_eq!(vl53l4cx::DEFAULT_CONFIGURATION[90], 0x00);
}

#[test]
fn the_timing_budget_words() {
    let count = check_cases(&part("timing_budget"), "timing_budget", |input| {
        let oscillator = u16::try_from(input["oscillator"].as_u64().unwrap()).unwrap();
        let words = vl53l4cx::timing_budget(oscillator, input["budget_ms"].as_i64().unwrap());
        json!(words.map_or_else(Vec::new, |words| words.to_vec()))
    });
    assert!(count > 40);
}

#[test]
fn the_range_status_behind_data_ready() {
    let section = part("status");
    let compare = Compare::from_json(&section["compare"]);
    let mut valid = 0;
    for case in section["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let (script, owed) = owed(case, "agl");
        let mock = replay(script);
        // polarity 1 until setup reads GPIO_HV_MUX__CTRL: the fixture's data-ready answer is bit 0 set
        let ranger = Vl53::<MockPin>::new(&vl53l1x::VL53L1X, &Settings::from_config(&json!({})), None);
        let ranging = ranger.range(&mut mock.clone()).expect("a scripted range");
        mock.done();
        valid += usize::from(ranging.agl().is_some());
        compare
            .check(name, &owed, &json!({"agl": ranging.agl().map(f64::from)}))
            .unwrap_or_else(|failure| panic!("{failure}"));
    }
    assert!(valid >= 5, "only {valid} valid ranges");
}

#[test]
fn setup_makes_plan_b_s_transactions() {
    let count = check_cases(&part("setup"), "setup", |input| {
        let mock = replay(&input["script"]);
        let config = &input["config"];
        let xshut = config.get("xshut_pin").map(|_| mock.pin());
        let mut ranger = Vl53::new(silicon(input), &Settings::from_config(config), xshut);
        let outcome = ranger.setup(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok()})
    });
    assert!(count >= 20);
}

#[test]
fn the_probe_checks_the_whole_model_id() {
    let section = part("probe");
    let compare = Compare::from_json(&section["compare"]);
    for case in section["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let (script, owed) = owed(case, "ok");
        let mock = replay(script);
        let expected = silicon(&case["input"]);
        let ranger = Vl53::<MockPin>::new(expected, &Settings::from_config(&json!({})), None);
        let outcome = ranger.probe(&mut mock.clone());
        mock.done();
        compare
            .check(name, &owed, &json!({"ok": outcome.is_ok()}))
            .unwrap_or_else(|failure| panic!("{failure}"));
        // a passing probe answers this silicon's whole id, the one the owner's ok line prints (0xebaa for the L4CX,
        // where plan B's printed the high byte 0xeb: a consequence of the 16-bit read)
        if let Ok(model_id) = outcome {
            assert_eq!(model_id, expected.model_id, "{name}");
        }
    }
}

#[test]
fn diagnose_tells_the_other_silicon_from_an_empty_socket() {
    check_cases(&part("diagnose"), "diagnose", |input| {
        let mock = replay(&input["script"]);
        let ranger = Vl53::<MockPin>::new(silicon(input), &Settings::from_config(&json!({})), None);
        let verdict = ranger.diagnose(&mut mock.clone());
        mock.done();
        json!(verdict)
    });
}
