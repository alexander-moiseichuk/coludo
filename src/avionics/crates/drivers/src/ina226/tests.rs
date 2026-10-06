//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the INA226 port beyond its golden fixture (tests/golden_ina226.rs): the conversions at their edges,
//! the re-arm's preconditions, and the settings.

use super::{ADDRESS, Ina226, Measurement, Settings, alert_limit, calibration};
use crate::device::{Device, Fault};
use crate::mock::MockBus;
use serde_json::json;

/// A monitor with these settings.
fn monitor(config: serde_json::Value, alert_wired: bool) -> Ina226 {
    Ina226::new(&Settings::from_config(&config, alert_wired))
}

/// Script a healthy setup: the die id, CONFIGURATION, CALIBRATION 3355.
fn healthy_setup(mock: &MockBus) {
    mock.expect_write_read(ADDRESS, &[0xFF], &[0x22, 0x60])
        .expect_write(ADDRESS, &[0x00, 0x43, 0x27])
        .expect_write(ADDRESS, &[0x05, 0x0D, 0x1B]);
}

#[test]
fn conversions_floor_like_python() {
    let power = monitor(json!({}), false);
    assert_eq!(
        power.convert(0x0F95, -1, 1),
        Measurement {
            voltage_mv: 4986,
            current_ma: -1,
            power_mw: 0
        }
    );
    assert_eq!(power.convert(65535, -32768, 65535).current_ma, -5000);
    // NEGATIVE: a 64-bit product never wraps and an absurd full scale saturates instead of overflowing
    let absurd = monitor(json!({"max_current_ma": 1_000_000_000_000_i64}), false);
    assert_eq!(absurd.convert(0, 32767, 65535).power_mw, i32::MAX);
}

#[test]
fn the_calibration_and_the_alert_limit_at_their_edges() {
    assert_eq!(calibration(2561, 1), Some(65510));
    assert_eq!(calibration(2560, 1), None, "65536 does not fit the register");
    assert_eq!(calibration(0, 10), None);
    assert_eq!(calibration(-5000, 10), None);
    assert_eq!(
        calibration(i64::MAX, i64::MAX),
        Some(0),
        "plan B's integers never overflow: the quotient is 0"
    );
    assert_eq!(alert_limit(3000, 10), Some(12000));
    assert_eq!(alert_limit(16384, 10), None);
    assert_eq!(alert_limit(-1, 10), None);
}

#[test]
fn a_rearm_before_any_setup_is_refused() {
    // NEGATIVE: there is no calibration to restore, so nothing is written
    let mock = MockBus::new();
    let outcome = monitor(json!({}), false).rearm(&mut mock.clone(), &mut mock.delay());
    assert_eq!(outcome, Err(Fault::Configuration));
    mock.done();
}

#[test]
fn rearm_writes_the_calibration_setup_wrote() {
    let mock = MockBus::new();
    healthy_setup(&mock);
    let mut power = monitor(json!({}), false);
    power.setup(&mut mock.clone(), &mut mock.delay()).expect("setup");
    assert_eq!(power.calibration(), Some(3355));
    mock.expect_write(ADDRESS, &[0x00, 0x43, 0x27])
        .expect_write(ADDRESS, &[0x05, 0x0D, 0x1B]);
    power.rearm(&mut mock.clone(), &mut mock.delay()).expect("rearm");
    mock.done();
}

#[test]
fn a_failed_alert_leaves_the_monitor_polling() {
    let mock = MockBus::new();
    healthy_setup(&mock);
    mock.expect_write(ADDRESS, &[0x07, 0x2E, 0xE0]).then_fail();
    let mut power = monitor(json!({"alert_ma": 3000}), true);
    assert_eq!(power.setup(&mut mock.clone(), &mut mock.delay()), Ok(()));
    assert_eq!(power.alert_ma(), 0, "poll only: no trip is reported");
    // the re-arm then restores the two plan B restored, and no alert
    mock.expect_write(ADDRESS, &[0x00, 0x43, 0x27])
        .expect_write(ADDRESS, &[0x05, 0x0D, 0x1B]);
    power.rearm(&mut mock.clone(), &mut mock.delay()).expect("rearm");
    mock.done();
}

/// Set a monitor up with the alert armed at 3000 mA (ALERT LIMIT 12000 = 0x2EE0, MASK/ENABLE SOL), then script the
/// re-arm's CONFIGURATION write.
fn armed_monitor(mock: &MockBus) -> Ina226 {
    healthy_setup(mock);
    mock.expect_write(ADDRESS, &[0x07, 0x2E, 0xE0])
        .expect_write(ADDRESS, &[0x06, 0x80, 0x00]);
    let mut power = monitor(json!({"alert_ma": 3000}), true);
    power.setup(&mut mock.clone(), &mut mock.delay()).expect("setup");
    assert_eq!(power.alert_ma(), 3000);
    mock.expect_write(ADDRESS, &[0x00, 0x43, 0x27]);
    power
}

#[test]
fn a_rearm_whose_calibration_naks_is_reported() {
    // NEGATIVE: a CALIBRATION that silently failed to restore reads every current and power wrong, not absent
    let mock = MockBus::new();
    healthy_setup(&mock);
    let mut power = monitor(json!({}), false);
    power.setup(&mut mock.clone(), &mut mock.delay()).expect("setup");
    mock.expect_write(ADDRESS, &[0x00, 0x43, 0x27])
        .expect_write(ADDRESS, &[0x05, 0x0D, 0x1B])
        .then_fail();
    let outcome = power.rearm(&mut mock.clone(), &mut mock.delay());
    assert!(matches!(outcome, Err(Fault::Bus(_))), "{outcome:?}");
    mock.done();
}

#[test]
fn a_rearm_whose_alert_limit_naks_is_reported_and_writes_no_mask() {
    // NEGATIVE: the trip did not restore, so the SOL mask is not armed over a limit of 0
    let mock = MockBus::new();
    let mut power = armed_monitor(&mock);
    mock.expect_write(ADDRESS, &[0x05, 0x0D, 0x1B])
        .expect_write(ADDRESS, &[0x07, 0x2E, 0xE0])
        .then_fail();
    let outcome = power.rearm(&mut mock.clone(), &mut mock.delay());
    assert!(matches!(outcome, Err(Fault::Bus(_))), "{outcome:?}");
    mock.done();
}

#[test]
fn a_rearm_whose_mask_naks_is_reported() {
    // NEGATIVE: the alert stays disarmed after the general call, which the owner must hear about
    let mock = MockBus::new();
    let mut power = armed_monitor(&mock);
    mock.expect_write(ADDRESS, &[0x05, 0x0D, 0x1B])
        .expect_write(ADDRESS, &[0x07, 0x2E, 0xE0])
        .expect_write(ADDRESS, &[0x06, 0x80, 0x00])
        .then_fail();
    let outcome = power.rearm(&mut mock.clone(), &mut mock.delay());
    assert!(matches!(outcome, Err(Fault::Bus(_))), "{outcome:?}");
    mock.done();
}

#[test]
fn a_scaling_number_that_is_not_an_integer_fails_setup_after_the_configuration() {
    // NEGATIVE: plan B's struct.pack refused the float CALIBRATION; calibrating for the default 10 mOhm instead
    // would read ~4 % wrong without a word. 10.0 is a float to struct.pack too
    for block in [
        json!({"shunt_mohms": 10.4}),
        json!({"shunt_mohms": 10.0}),
        json!({"max_current_ma": 5000.5}),
        json!({"max_current_ma": "5000"}),
        json!({"shunt_mohms": null}),
    ] {
        let settings = Settings::from_config(&block, false);
        assert!(!settings.scaling_is_integer, "{block}");
        let mock = MockBus::new();
        mock.expect_write_read(ADDRESS, &[0xFF], &[0x22, 0x60])
            .expect_write(ADDRESS, &[0x00, 0x43, 0x27]);
        let mut power = Ina226::new(&settings);
        let outcome = power.setup(&mut mock.clone(), &mut mock.delay());
        assert_eq!(outcome, Err(Fault::Configuration), "{block}");
        assert_eq!(power.calibration(), None, "{block}");
        mock.done();
    }
    // positive: an integer, or no key at all, calibrates
    assert!(Settings::from_config(&json!({"shunt_mohms": 2, "max_current_ma": 20000}), false).scaling_is_integer);
    assert!(Settings::from_config(&json!({}), false).scaling_is_integer);
}

#[test]
fn a_fractional_alert_trip_polls_only() {
    // NEGATIVE: plan B's alert block failed its pack and fell back to polling; no trip is armed or reported
    let settings = Settings::from_config(&json!({"alert_ma": 2500.5}), true);
    assert_eq!(settings.alert_ma, None);
    assert!(settings.scaling_is_integer, "the monitor itself still calibrates");
    let mock = MockBus::new();
    healthy_setup(&mock);
    let mut power = Ina226::new(&settings);
    assert_eq!(power.setup(&mut mock.clone(), &mut mock.delay()), Ok(()));
    assert_eq!(power.alert_ma(), 0);
    mock.done();
}

#[test]
fn settings_take_plan_b_s_defaults() {
    let defaults = Settings::from_config(&json!({}), false);
    assert_eq!(
        (
            defaults.address,
            defaults.period_ms,
            defaults.shunt_mohms,
            defaults.max_current_ma,
            defaults.alert_ma
        ),
        (0x40, 100, 10, 5000, None)
    );
    assert_eq!(Settings::from_config(&json!({}), true).alert_ma, Some(3000));
    // NEGATIVE: an alert trip with no pin wired arms nothing
    assert_eq!(Settings::from_config(&json!({"alert_ma": 2000}), false).alert_ma, None);
}

#[test]
fn a_wrong_die_is_named() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[0xFF], &[0x54, 0x49]);
    let outcome = monitor(json!({}), false).identify(&mut mock.clone());
    assert_eq!(
        outcome,
        Err(Fault::Identity {
            expected: 0x2260,
            read: 0x5449
        })
    );
    mock.done();
}
