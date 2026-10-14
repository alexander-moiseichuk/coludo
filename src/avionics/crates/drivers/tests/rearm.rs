//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The operator's rule of 2026-10-06 (plan B commit 76c138c): after an I2C general-call reset, the bus owner
//! re-initialises EVERY device registered on that bus, found by membership, never by a hard-coded list. Here the
//! membership is a list of `dyn Device` -- the shape the firmware's registry walks -- holding the pitot, the power
//! monitor and the laser, and each one's re-arm must restore what the general call took.

use drivers::device::{Device, Fault};
use drivers::ina226::{self, Ina226};
use drivers::mock::{MockBus, MockError, NoPin};
use drivers::sdp810::{self, Sdp810};
use drivers::vl53l1x::{self, Vl53};
use serde_json::json;

/// One device as the bus owner holds it: behind the trait, whatever the part.
type Member = Box<dyn Device<MockBus, BusError = MockError>>;

/// Script a VL53L1X bring-up that boots and reports ready at once.
fn laser_bring_up(mock: &MockBus) {
    let mut block = vec![0x00, 0x2D];
    block.extend_from_slice(&vl53l1x::DEFAULT_CONFIGURATION);
    mock.expect_delay_ms(2)
        .expect_write_read(0x29, &[0x00, 0xE5], &[0x03])
        .expect_write_read(0x29, &[0x01, 0x0F], &[0xEA, 0xCC])
        .expect_write(0x29, &block)
        .expect_write_read(0x29, &[0x00, 0x30], &[0x01])
        .expect_write(0x29, &[0x00, 0x87, 0x40])
        .expect_write_read(0x29, &[0x00, 0x31], &[0x01])
        .expect_write(0x29, &[0x00, 0x86, 0x01])
        .expect_write(0x29, &[0x00, 0x87, 0x00])
        .expect_write(0x29, &[0x00, 0x08, 0x09])
        .expect_write(0x29, &[0x00, 0x0B, 0x00])
        .expect_write(0x29, &[0x00, 0x87, 0x40]);
}

/// The INA226's setup with the alert armed: die id, CONFIGURATION, CALIBRATION, ALERT LIMIT, MASK/ENABLE.
fn monitor_setup(mock: &MockBus) {
    mock.expect_write_read(0x40, &[0xFF], &[0x22, 0x60]);
    monitor_rearm(mock);
}

/// What the INA226 must write again: everything setup wrote but the die id read.
fn monitor_rearm(mock: &MockBus) {
    mock.expect_write(0x40, &[0x00, 0x43, 0x27])
        .expect_write(0x40, &[0x05, 0x0D, 0x1B])
        .expect_write(0x40, &[0x07, 0x2E, 0xE0])
        .expect_write(0x40, &[0x06, 0x80, 0x00]);
}

/// The pitot's restart: stop, settle, start, first result.
fn pitot_restart(mock: &MockBus) {
    mock.expect_write(0x25, &sdp810::COMMAND_STOP)
        .expect_delay_ms(sdp810::STOP_SETTLE_MS)
        .expect_write(0x25, &sdp810::COMMAND_START)
        .expect_delay_ms(sdp810::FIRST_RESULT_MS);
}

/// The three parts, set up over the mock.
fn members(mock: &MockBus) -> Vec<Member> {
    let mut pitot = Sdp810::new(&sdp810::Settings::from_config(&json!({})));
    let mut frame = [0u8; 9];
    for index in 0..3 {
        let word: [u8; 2] = if index == 2 { 60u16.to_be_bytes() } else { [0, 0] };
        frame[index * 3..index * 3 + 2].copy_from_slice(&word);
        frame[index * 3 + 2] = drivers::commons::sensirion_crc8(word[0], word[1]);
    }
    pitot_restart(mock);
    mock.expect_read(0x25, &frame);
    pitot.setup(&mut mock.clone(), &mut mock.delay()).expect("the pitot");
    let mut power = Ina226::new(&ina226::Settings::from_config(&json!({"alert_ma": 3000}), true));
    monitor_setup(mock);
    power.setup(&mut mock.clone(), &mut mock.delay()).expect("the monitor");
    let mut laser: Vl53<NoPin> = Vl53::new(&vl53l1x::VL53L1X, &vl53l1x::Settings::from_config(&json!({})), None);
    laser_bring_up(mock);
    laser.setup(&mut mock.clone(), &mut mock.delay()).expect("the laser");
    mock.done();
    vec![Box::new(pitot), Box::new(power), Box::new(laser)]
}

#[test]
fn a_general_call_re_arms_every_member() {
    let mock = MockBus::new();
    let mut bus = members(&mock);
    pitot_restart(&mock);
    monitor_rearm(&mock);
    laser_bring_up(&mock);
    let outcomes: Vec<Result<(), Fault<MockError>>> = bus
        .iter_mut()
        .map(|member| member.rearm(&mut mock.clone(), &mut mock.delay()))
        .collect();
    assert_eq!(outcomes, [Ok(()), Ok(()), Ok(())]);
    mock.done();
}

#[test]
fn one_member_failing_does_not_stop_the_walk() {
    // NEGATIVE: the pitot's stop NAKs; the walk goes on and the other two are restored all the same
    let mock = MockBus::new();
    let mut bus = members(&mock);
    mock.expect_write(0x25, &sdp810::COMMAND_STOP).then_fail();
    monitor_rearm(&mock);
    laser_bring_up(&mock);
    let outcomes: Vec<bool> = bus
        .iter_mut()
        .map(|member| member.rearm(&mut mock.clone(), &mut mock.delay()).is_ok())
        .collect();
    assert_eq!(outcomes, [false, true, true]);
    mock.done();
}
