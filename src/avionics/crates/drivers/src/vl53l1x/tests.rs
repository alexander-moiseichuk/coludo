//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the VL53 family engine beyond its golden fixture (tests/golden_vl53.rs): the data-ready gate plan B
//! lacked, the XSHUT line, the re-arm, and the small decoders.

use super::{ADDRESS, DEFAULT_CONFIGURATION, Ranging, STATUS_RTN, Settings, VL53L1X, Vl53, polarity, range_status};
use crate::device::{Device, Fault};
use crate::mock::{MockBus, MockPin, NoPin};
use serde_json::json;

/// A ranger for the L1X silicon, no XSHUT, at the default address.
fn ranger() -> Vl53<NoPin> {
    Vl53::new(&VL53L1X, &Settings::from_config(&json!({})), None)
}

/// Script a whole L1X bring-up that boots and reports ready at once (polarity 1).
fn bring_up(mock: &MockBus) {
    let mut block = vec![0x00, 0x2D];
    block.extend_from_slice(&DEFAULT_CONFIGURATION);
    mock.expect_delay_ms(2)
        .expect_write_read(ADDRESS, &[0x00, 0xE5], &[0x03])
        .expect_write_read(ADDRESS, &[0x01, 0x0F], &[0xEA, 0xCC])
        .expect_write(ADDRESS, &block)
        .expect_write_read(ADDRESS, &[0x00, 0x30], &[0x01])
        .expect_write(ADDRESS, &[0x00, 0x87, 0x40])
        .expect_write_read(ADDRESS, &[0x00, 0x31], &[0x01])
        .expect_write(ADDRESS, &[0x00, 0x86, 0x01])
        .expect_write(ADDRESS, &[0x00, 0x87, 0x00])
        .expect_write(ADDRESS, &[0x00, 0x08, 0x09])
        .expect_write(ADDRESS, &[0x00, 0x0B, 0x00])
        .expect_write(ADDRESS, &[0x00, 0x87, 0x40]);
}

#[test]
fn no_new_measurement_reads_nothing_more() {
    // NEGATIVE: the range plan B re-read in poll mode is not read, not cleared, not published
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[0x00, 0x31], &[0x00]);
    assert_eq!(ranger().range(&mut mock.clone()), Ok(Ranging::NotReady));
    mock.done();
}

#[test]
fn a_waiting_measurement_is_read_and_released() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[0x00, 0x31], &[0x01])
        .expect_write_read(ADDRESS, &[0x00, 0x89], &[0xE9])
        .expect_write_read(ADDRESS, &[0x00, 0x96], &[0x04, 0xD2])
        .expect_write(ADDRESS, &[0x00, 0x86, 0x01]);
    let ranging = ranger().range(&mut mock.clone()).unwrap();
    assert_eq!(ranging, Ranging::Valid { distance_mm: 1234 });
    assert_eq!(ranging.agl(), Some(1.234));
    mock.done();
}

#[test]
fn an_invalid_status_is_cleared_but_never_published() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[0x00, 0x31], &[0x01])
        .expect_write_read(ADDRESS, &[0x00, 0x89], &[0x04])
        .expect_write_read(ADDRESS, &[0x00, 0x96], &[0x1F, 0x40])
        .expect_write(ADDRESS, &[0x00, 0x86, 0x01]);
    let ranging = ranger().range(&mut mock.clone()).unwrap();
    assert_eq!(ranging, Ranging::Invalid { status: 2 });
    assert_eq!(ranging.agl(), None);
    mock.done();
}

#[test]
fn the_xshut_pulse_comes_first() {
    let mock = MockBus::new();
    mock.expect_pin(false).expect_delay_ms(2).expect_pin(true);
    bring_up(&mock);
    let mut ranger = Vl53::new(&VL53L1X, &Settings::from_config(&json!({})), Some(mock.pin()));
    assert!(ranger.has_xshut());
    assert_eq!(ranger.setup(&mut mock.clone(), &mut mock.delay()), Ok(()));
    mock.done();
}

#[test]
fn a_refused_xshut_fails_setup_before_the_bus() {
    // NEGATIVE: no I2C traffic after the pin refuses
    let mock = MockBus::new();
    mock.expect_pin(false).then_fail();
    let mut ranger: Vl53<MockPin> = Vl53::new(&VL53L1X, &Settings::from_config(&json!({})), Some(mock.pin()));
    assert_eq!(ranger.setup(&mut mock.clone(), &mut mock.delay()), Err(Fault::Pin));
    mock.done();
}

#[test]
fn rearm_runs_the_whole_bring_up_again() {
    let mock = MockBus::new();
    bring_up(&mock);
    bring_up(&mock);
    let mut ranger = ranger();
    ranger.setup(&mut mock.clone(), &mut mock.delay()).expect("setup");
    ranger.rearm(&mut mock.clone(), &mut mock.delay()).expect("rearm");
    mock.done();
}

#[test]
fn the_other_silicon_is_refused_after_its_id() {
    // NEGATIVE: an L4CX in the socket gets no L1X block written to it
    let mock = MockBus::new();
    mock.expect_delay_ms(2)
        .expect_write_read(ADDRESS, &[0x00, 0xE5], &[0x01])
        .expect_write_read(ADDRESS, &[0x01, 0x0F], &[0xEB, 0xAA]);
    let outcome = ranger().setup(&mut mock.clone(), &mut mock.delay());
    assert_eq!(
        outcome,
        Err(Fault::Identity {
            expected: 0xEACC,
            read: 0xEBAA
        })
    );
    mock.done();
}

#[test]
fn the_decoders() {
    assert_eq!(STATUS_RTN[9], 0);
    assert_eq!(range_status(0x09 | 0xE0), 0, "only the low five bits count");
    assert_eq!(range_status(24), 255, "past the table is reserved");
    assert_eq!(range_status(31), 255);
    assert_eq!((polarity(0x01), polarity(0x11), polarity(0x10)), (1, 0, 0));
}

#[test]
fn settings_keep_the_period_at_least_a_millisecond() {
    let defaults = Settings::from_config(&json!({}));
    assert_eq!(
        (defaults.address, defaults.period_ms, defaults.timing_budget_ms),
        (0x29, 50, 50)
    );
    // NEGATIVE: a 0 or negative period would spin the owner's schedule; plan B's max(1, ...) holds
    assert_eq!(Settings::from_config(&json!({"period_ms": 0})).period_ms, 1);
    assert_eq!(Settings::from_config(&json!({"period_ms": -5})).period_ms, 1);
}
