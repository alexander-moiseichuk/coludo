//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the VL53L4CX specifics beyond the golden fixture (tests/golden_vl53.rs): the macro period, the
//! budget words at their edges, and the budget's place in the bring-up.

use super::{
    DEFAULT_CONFIGURATION, RANGE_CONFIG_TIMEOUT_MACROP_A, RANGE_CONFIG_TIMEOUT_MACROP_B, VL53L4CX, macro_period,
    timing_budget,
};
use crate::device::Device;
use crate::mock::{MockBus, NoPin};
use crate::vl53l1x::{ADDRESS, Settings, Vl53};
use serde_json::json;

#[test]
fn the_macro_period_multiplies_before_it_divides() {
    assert_eq!(macro_period(0), None);
    // (2304 x 2^30 // 2970) >> 6, as plan B computes it
    assert_eq!(macro_period(2970), Some(((2304_i128 << 30) / 2970) >> 6));
    assert_eq!(macro_period(1), Some((2304_i128 << 30) >> 6));
}

#[test]
fn the_budget_words() {
    // the golden fixture's anchor: a 50 ms budget at an oscillator of 2970
    assert_eq!(
        timing_budget(2970, 50),
        Some([(RANGE_CONFIG_TIMEOUT_MACROP_A, 59), (RANGE_CONFIG_TIMEOUT_MACROP_B, 79)])
    );
    // a long budget spills into the exponent byte
    let [(_, long), _] = timing_budget(2970, 1_000_000_000).unwrap();
    assert_eq!(long >> 8, 23);
    // NEGATIVE: an oscillator of 0 programs nothing, and a budget under the 2.5 ms overhead writes the low byte of a
    // negative count, as plan B's `& 0xFF` does, rather than panicking
    assert_eq!(timing_budget(0, 50), None);
    assert_eq!(timing_budget(2970, 2), Some([(0x005E, 254), (0x0061, 254)]));
}

#[test]
fn the_l4cx_programs_its_budget_after_the_vhv_writes() {
    let mock = MockBus::new();
    let mut block = vec![0x00, 0x2D];
    block.extend_from_slice(&DEFAULT_CONFIGURATION);
    mock.expect_delay_ms(2)
        .expect_write_read(ADDRESS, &[0x00, 0xE5], &[0x03])
        .expect_write_read(ADDRESS, &[0x01, 0x0F], &[0xEB, 0xAA])
        .expect_write(ADDRESS, &block)
        .expect_write_read(ADDRESS, &[0x00, 0x30], &[0x01])
        .expect_write(ADDRESS, &[0x00, 0x87, 0x21])
        .expect_write_read(ADDRESS, &[0x00, 0x31], &[0x01])
        .expect_write(ADDRESS, &[0x00, 0x86, 0x01])
        .expect_write(ADDRESS, &[0x00, 0x87, 0x00])
        .expect_write(ADDRESS, &[0x00, 0x08, 0x09])
        .expect_write(ADDRESS, &[0x00, 0x0B, 0x00])
        .expect_write(ADDRESS, &[0x00, 0x24, 0x05, 0x00])
        .expect_write_read(ADDRESS, &[0x00, 0x06], &[0x0B, 0x9A])
        .expect_write(ADDRESS, &[0x00, 0x5E, 0x00, 59])
        .expect_write(ADDRESS, &[0x00, 0x61, 0x00, 79])
        .expect_write(ADDRESS, &[0x00, 0x87, 0x21]);
    let mut ranger: Vl53<NoPin> = Vl53::new(&VL53L4CX, &Settings::from_config(&json!({})), None);
    assert_eq!(ranger.setup(&mut mock.clone(), &mut mock.delay()), Ok(()));
    mock.done();
}

#[test]
fn the_silicon_descriptor() {
    assert_eq!(
        (VL53L4CX.model_id, VL53L4CX.start_continuous, VL53L4CX.timing_budget),
        (0xEBAA, 0x21, true)
    );
    assert_eq!(VL53L4CX.name, "vl53l4cx");
    // NEGATIVE: the two blocks differ from their very first byte, which is why neither ranges the other's part
    assert_ne!(DEFAULT_CONFIGURATION[0], crate::vl53l1x::DEFAULT_CONFIGURATION[0]);
}
