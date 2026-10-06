//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the SDP810 port beyond its golden fixture (tests/golden_sdp810.rs): the general-call re-arm plan B
//! did not have, the read's sample, the settings, and the refusals.

use super::{
    ADDRESS, AIR_DENSITY, COMMAND_START, COMMAND_STOP, FIRST_RESULT_MS, Refusal, STOP_SETTLE_MS, Sdp810, Settings,
    airspeed_cms,
};
use crate::commons::sensirion_crc8;
use crate::device::{Device, Fault};
use crate::mock::MockBus;
use serde_json::json;

/// A good frame: the pressure, temperature and scale words, each with its CRC.
fn frame(pressure: i16, temperature: i16, scale: u16) -> [u8; 9] {
    let mut frame = [0u8; 9];
    for (index, word) in [pressure.to_be_bytes(), temperature.to_be_bytes(), scale.to_be_bytes()]
        .into_iter()
        .enumerate()
    {
        frame[index * 3] = word[0];
        frame[index * 3 + 1] = word[1];
        frame[index * 3 + 2] = sensirion_crc8(word[0], word[1]);
    }
    frame
}

/// A pitot with the defaults, untared.
fn pitot() -> Sdp810 {
    Sdp810::new(&Settings::from_config(&json!({})))
}

#[test]
fn rearm_stops_then_starts_at_once() {
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &COMMAND_STOP)
        .expect_delay_ms(STOP_SETTLE_MS)
        .expect_write(ADDRESS, &COMMAND_START)
        .expect_delay_ms(FIRST_RESULT_MS);
    assert_eq!(pitot().rearm(&mut mock.clone(), &mut mock.delay()), Ok(()));
    mock.done();
}

#[test]
fn a_rearm_whose_stop_naks_never_starts() {
    // NEGATIVE: the part refuses a start while streaming, so a failed stop must not be followed by one
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &COMMAND_STOP).then_fail();
    let outcome = pitot().rearm(&mut mock.clone(), &mut mock.delay());
    assert!(matches!(outcome, Err(Fault::Bus(_))), "{outcome:?}");
    mock.done();
}

#[test]
fn a_read_converts_one_frame_for_the_channels_and_the_row() {
    let mock = MockBus::new();
    mock.expect_read(ADDRESS, &frame(8100, 5003, 60));
    let mut pitot = Sdp810::new(&Settings::from_config(&json!({"air_density": 1.18})));
    let sample = pitot.read(&mut mock.clone(), &mut mock.delay()).expect("a good frame");
    mock.done();
    assert_eq!(sample.dynamic_pressure, 135.0);
    assert!((sample.airspeed - (270.0_f32 / 1.18).sqrt()).abs() < 1e-5);
    assert!((sample.temperature - 25.015).abs() < 1e-4);
    assert_eq!(sample.row, [13500, 1512, 2501]);
}

#[test]
fn a_corrupt_scale_word_is_rejected_not_latched() {
    // NEGATIVE: the third CRC is the one that guards every later reading
    let mut corrupt = frame(12, 5000, 60);
    corrupt[7] ^= 0x01;
    let mock = MockBus::new();
    mock.expect_read(ADDRESS, &corrupt);
    let failure = pitot().read(&mut mock.clone(), &mut mock.delay()).unwrap_err();
    assert_eq!((failure.fault, failure.restart), (Fault::Checksum, None));
    mock.done();
}

#[test]
fn the_airspeed_never_takes_a_root_of_nothing() {
    let pitot = pitot();
    assert_eq!(pitot.airspeed(-10.0), 0.0);
    assert_eq!(pitot.airspeed(0.0), 0.0);
    assert_eq!(
        pitot.airspeed(f32::NAN),
        0.0,
        "a NaN q reads 0 m/s, as plan B's `q > 0.0` guard does"
    );
    assert_eq!(airspeed_cms(14.999_99), 1499, "truncated, never rounded up");
}

#[test]
fn settings_take_plan_b_s_defaults() {
    let defaults = Settings::from_config(&json!({}));
    assert_eq!(
        (
            defaults.address,
            defaults.period_ms,
            defaults.air_density,
            defaults.zero_offset
        ),
        (0x25, 20, 1.18, 0)
    );
    let custom =
        Settings::from_config(&json!({"addr": 38, "period_ms": 40, "air_density": 1.25, "zero_offset_pa": 2.5}));
    assert_eq!((custom.address, custom.period_ms, custom.zero_offset), (38, 40, 250));
    // NEGATIVE: a key of the wrong type takes the default rather than a garbage value
    let wrong = Settings::from_config(&json!({"addr": "0x26", "period_ms": -1, "air_density": "dense"}));
    assert_eq!((wrong.address, wrong.period_ms, wrong.air_density), (0x25, 20, 1.18));
}

#[test]
fn the_zero_offset_truncates_toward_zero_as_plan_b_s_fixnum() {
    let mut pitot = pitot();
    // the figures the golden tare and setup sections carry, each plan B's hundredths
    for (pascal, hundredths) in [
        (1.18, 118),
        (-1.18, -118),
        (1.5, 150),
        (-1.25, -125),
        (2.0, 200),
        (2.5, 250),
    ] {
        pitot.set_zero_offset(pascal);
        assert_eq!(pitot.zero_offset(), hundredths, "{pascal} Pa");
    }
    assert!(pitot.tared());
    // NEGATIVE: a figure finer than a hundredth still truncates toward zero, never rounds
    pitot.set_zero_offset(-1.259);
    assert_eq!(pitot.zero_offset(), -125);
    pitot.set_zero_offset(3.339);
    assert_eq!(pitot.zero_offset(), 333);
    // the deliberate divergence: 0.29 Pa x 100 is 28.999998 in f32 (28.999999999999996 in CPython), which plan B
    // truncated to 28; the port reads the 29 that was meant
    pitot.set_zero_offset(0.29);
    assert_eq!(pitot.zero_offset(), 29);
}

#[test]
fn the_warm_start_round_trip_returns_every_tare() {
    // zero_offset_pa() (f32 Pa) -> the crumb's f32 -> set_zero_offset: plan B's truncation lost a hundredth on about
    // half of these (333, the plan's own 200 -> 333 anchor, came back 332)
    let mut pitot = pitot();
    for tare in [1, 29, 333, -1999, 0, -1, 54613, -54613] {
        pitot.restore_zero(tare);
        let crumb = pitot.zero_offset_pa();
        pitot.set_zero_offset(crumb);
        assert_eq!(pitot.zero_offset(), tare, "{tare} via {crumb} Pa");
    }
    // every tare the part can capture: raw +-32768 at scale 60 is +-546.13 Pa
    for tare in -54_613..=54_613 {
        pitot.restore_zero(tare);
        let crumb = pitot.zero_offset_pa();
        pitot.set_zero_offset(crumb);
        assert_eq!(pitot.zero_offset(), tare, "{tare} via {crumb} Pa");
    }
}

#[test]
fn the_dynamic_pressure_stays_signed() {
    // reverse flow and sub-zero noise are real readings for the governor's saturation and in-band logic, not 0
    let mut pitot = pitot();
    assert_eq!(pitot.pressure(-600), -10.0);
    assert_eq!(pitot.pressure_hundredths(-600), -1000);
    assert_eq!(pitot.pressure(-1), -1.0 / 60.0);
    pitot.restore_zero(125);
    assert_eq!(
        pitot.pressure(0),
        -1.25,
        "a tare above a still reading leaves it below 0"
    );
}

#[test]
fn an_air_density_that_cannot_divide_is_refused() {
    // NEGATIVE: plan B took 0 (then raised ZeroDivisionError on every positive q) and -1 (a complex root); in f32
    // they would be an infinite and a NaN airspeed handed to the governor
    let mut pitot = pitot();
    pitot.pressure(200);
    for density in [json!(0), json!(0.0), json!(-1.0), json!(1e-50), json!(false)] {
        let properties = json!({"zero": true, "air_density": density});
        assert_eq!(
            pitot.update(&properties),
            Err(Refusal::OutOfRange("air_density")),
            "{properties}"
        );
        assert_eq!(pitot.air_density(), AIR_DENSITY, "{properties}");
        assert_eq!(
            (pitot.zero_offset(), pitot.tared()),
            (0, false),
            "{properties}: refused as a whole"
        );
    }
    assert_eq!(
        Refusal::OutOfRange("air_density").to_string(),
        "air_density must be above 0"
    );
    for density in [0.0, -1.18, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert_eq!(pitot.set_air_density(density), Err(Refusal::OutOfRange("air_density")));
        assert_eq!(pitot.air_density(), AIR_DENSITY);
    }
    // so q = 135 Pa still reads a finite airspeed and a sane telemetry figure
    let airspeed = pitot.airspeed(135.0);
    assert!(airspeed.is_finite() && airspeed_cms(airspeed) == 1512, "{airspeed}");
    // the config falls back to the default for the same values
    for density in [json!(0), json!(-1.0), json!(1e-50)] {
        let settings = Settings::from_config(&json!({ "air_density": density }));
        assert_eq!(settings.air_density, AIR_DENSITY, "{density}");
    }
    // positive: a real trim is taken, by either door
    assert_eq!(
        pitot.update(&json!({"air_density": 1.2})).map(|update| update.changed),
        Ok(vec!["air_density"])
    );
    assert_eq!(pitot.air_density(), 1.2);
    assert_eq!(pitot.set_air_density(1.25), Ok(()));
    assert_eq!(pitot.air_density(), 1.25);
}

#[test]
fn a_non_numeric_update_changes_nothing() {
    // NEGATIVE: refused as a whole, the tare untouched though `zero` came first
    let mut pitot = pitot();
    pitot.pressure(200);
    let refusal = pitot
        .update(&json!({"zero": true, "air_density": "thick"}))
        .unwrap_err();
    assert_eq!(refusal, Refusal::NotANumber("air_density"));
    assert_eq!(refusal.to_string(), "air_density must be a number");
    assert_eq!((pitot.zero_offset(), pitot.tared()), (0, false));
}

#[test]
fn the_tare_prompt_is_owed_until_a_tare() {
    let mut pitot = pitot();
    assert_eq!(
        pitot.calibration(Some(-1.25)).as_deref(),
        Some("keep the pitot in STILL AIR -- do NOT blow into it, this captures the zero tare (now -1.25 Pa)")
    );
    assert_eq!(pitot.calibrate(), Err(super::CALIBRATE_NO_READING));
    pitot.restore_zero(0);
    assert_eq!(pitot.calibration(Some(3.0)), None, "a stored 0 Pa is a tare");
}

#[test]
fn setup_latches_the_scale_and_stop_makes_no_answer() {
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &COMMAND_STOP)
        .expect_delay_ms(STOP_SETTLE_MS)
        .expect_write(ADDRESS, &COMMAND_START)
        .expect_delay_ms(FIRST_RESULT_MS)
        .expect_read(ADDRESS, &frame(0, 0, 240))
        .expect_write(ADDRESS, &COMMAND_STOP)
        .then_fail();
    let mut pitot = pitot();
    pitot.setup(&mut mock.clone(), &mut mock.delay()).expect("a good frame");
    assert_eq!(pitot.scale(), 240);
    pitot.stop(&mut mock.clone()); // a dead bus on teardown is swallowed
    mock.done();
}
