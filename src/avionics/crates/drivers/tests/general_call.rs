//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ICP-10111's general call and the re-arm by membership, end to end (the operator's rule of 2026-10-06; plan
//! B commit 76c138c and its test in test_icp10111.py). On 7C the SEN0697 shares i2c:0 with the ICP: a recovery of
//! the primary barometer resets the BMI323, the BMM350 and the BMP581 with it, and plan B's old fixed-list re-arm left
//! all three in their power-on state. Here the ICP's run loop escalates to the general call and reports
//! [`Poll::Rearm`]; the owner then walks its members of THAT bus, as `dyn Device` -- the shape the firmware's
//! registry holds -- and re-arms each one once. POSITIVE: every peer on the bus is re-armed, once per walk, whatever
//! the part, and a general call at setup is reported even when the ICP then fails. NEGATIVE: a peer whose re-arm fails
//! does not stop the walk, and a part that stays dead is not escalated a second time.

use drivers::bmi323::{self, Bmi323};
use drivers::bmm350::{self, Bmm350};
use drivers::bmp581::{self, Bmp581};
use drivers::device::Device;
use drivers::icp10111::{self, Icp10111, Poll};
use drivers::mock::{MockBus, MockError};

/// One device as the bus owner holds it: behind the trait, whatever the part.
type Member = Box<dyn Device<MockBus, BusError = MockError>>;

/// Microseconds per millisecond.
const MILLISECOND: u64 = 1000;

/// A BMI323 burst with its gyro running (two dummy bytes, then six words).
fn running_burst() -> [u8; 14] {
    let mut burst = [0u8; 14];
    burst[6..8].copy_from_slice(&2048_i16.to_le_bytes());
    burst
}

/// Script the BMI323's re-arm: the reset, the configuration, one start-up look.
fn script_bmi323(mock: &MockBus) {
    let (register, value) = bmi323::SOFT_RESET;
    let [low, high] = value.to_le_bytes();
    mock.expect_write(bmi323::ADDRESS, &[register, low, high])
        .expect_delay_ms(bmi323::SOFT_RESET_SETTLE_MS);
    for (register, value) in bmi323::CONFIGURATION {
        let [low, high] = value.to_le_bytes();
        mock.expect_write(bmi323::ADDRESS, &[register, low, high]);
    }
    mock.expect_delay_ms(bmi323::START_UP_POLL_MS).expect_write_read(
        bmi323::ADDRESS,
        &[bmi323::ACC_DATA_X],
        &running_burst(),
    );
}

/// Script the BMM350's re-arm: its configuration table, the OTP power-off included.
fn script_bmm350(mock: &MockBus) {
    for step in bmm350::CONFIGURATION {
        match step {
            bmm350::Step::Write { register, value } => mock.expect_write(bmm350::ADDRESS, &[register, value]),
            bmm350::Step::Wait(milliseconds) => mock.expect_delay_ms(milliseconds),
        };
    }
}

/// Script the BMP581's re-arm: the mode, out of standby.
fn script_bmp581(mock: &MockBus) {
    for (register, value) in bmp581::CONFIGURATION {
        mock.expect_write(bmp581::ADDRESS, &[register, value]);
    }
}

/// Drive the ICP's run loop through three failed measurements to the general call, and answer the owner's question:
/// the millisecond at which it asked for the re-arm.
fn escalate(barometer: &mut Icp10111, bus: &MockBus, from_ms: u64) -> u64 {
    for _ in 0..icp10111::RECOVER_AFTER {
        bus.expect_write(icp10111::ADDRESS, &icp10111::MEASURE).then_fail();
    }
    bus.expect_write(icp10111::GENERAL_CALL_ADDRESS, &icp10111::GENERAL_CALL_RESET);
    let mut port = bus.clone();
    for now in from_ms..from_ms + 2000 {
        if barometer.poll(&mut port, now * MILLISECOND) == Poll::Rearm {
            return now;
        }
    }
    panic!("the recovery never asked for the re-arm");
}

/// The owner's walk: re-arm every member of the bus, keep going past a failure, and report which failed.
fn rearm_members(members: &mut [(&str, Member)], bus: &MockBus) -> Vec<String> {
    let mut port = bus.clone();
    let mut delay = bus.delay();
    members
        .iter_mut()
        .filter_map(|(name, member)| {
            member
                .rearm(&mut port, &mut delay)
                .err()
                .map(|fault| format!("peer {name} rearm failed: {fault}"))
        })
        .collect()
}

#[test]
fn a_recovery_re_arms_every_peer_on_its_bus_once() {
    // i2c:0 on 7C: the ICP and the SEN0697. The ICP is a member of the bus too, but the walk is of its PEERS
    let shared = MockBus::new();
    let mut barometer = Icp10111::new(icp10111::ADDRESS, 100);
    let mut peers: Vec<(&str, Member)> = vec![
        ("imu_bmi323", Box::new(Bmi323::new(bmi323::ADDRESS, false))),
        ("mag_bmm350", Box::new(Bmm350::new(bmm350::ADDRESS))),
        ("baro_bmp581", Box::new(Bmp581::new(bmp581::ADDRESS))),
    ];

    let rearm_at = escalate(&mut barometer, &shared, 0);
    assert_eq!(
        rearm_at, 330,
        "a period to drain, the general call at 300, 30 ms to settle"
    );
    script_bmi323(&shared);
    script_bmm350(&shared);
    script_bmp581(&shared);
    assert!(rearm_members(&mut peers, &shared).is_empty());
    shared.done();

    // a second walk re-arms them again, once each: never a cached "already done" (plan B's test walks again
    // directly; a second ESCALATION needs a good read in between, which the golden run section shows)
    script_bmi323(&shared);
    script_bmm350(&shared);
    script_bmp581(&shared);
    assert!(rearm_members(&mut peers, &shared).is_empty());
    shared.done();
    // and the part that stays dead is not escalated a second time: the count runs on past the limit
    shared.expect_write(icp10111::ADDRESS, &icp10111::MEASURE).then_fail();
    let mut port = shared.clone();
    for now in rearm_at + 1..rearm_at + 101 {
        assert!(!matches!(
            barometer.poll(&mut port, now * MILLISECOND),
            Poll::Rearm | Poll::GeneralCall
        ));
    }
    shared.done();
}

#[test]
fn a_peer_that_fails_its_re_arm_does_not_stop_the_walk() {
    let shared = MockBus::new();
    let mut peers: Vec<(&str, Member)> = vec![
        ("imu_bmi323", Box::new(Bmi323::new(bmi323::ADDRESS, false))),
        ("mag_bmm350", Box::new(Bmm350::new(bmm350::ADDRESS))),
        ("baro_bmp581", Box::new(Bmp581::new(bmp581::ADDRESS))),
    ];
    // the BMI323 is gone: its first write NAKs; the BMM350 and the BMP581 behind it are still re-armed
    let (register, value) = bmi323::SOFT_RESET;
    let [low, high] = value.to_le_bytes();
    shared.expect_write(bmi323::ADDRESS, &[register, low, high]).then_fail();
    script_bmm350(&shared);
    script_bmp581(&shared);
    let failures = rearm_members(&mut peers, &shared);
    shared.done();
    assert_eq!(failures.len(), 1);
    assert!(
        failures[0].starts_with("peer imu_bmi323 rearm failed: bus"),
        "{failures:?}"
    );
}

#[test]
fn a_general_call_at_setup_is_reported_for_the_walk() {
    // the setup path: a latched ICP gets the general call during its first touch, which also reset the peers set up
    // before it (on 7C the SEN0697 can precede the ICP), so the owner must walk them after setup too
    let shared = MockBus::new();
    shared
        .expect_write(icp10111::ADDRESS, &icp10111::READ_ID)
        .then_fail()
        .expect_delay_ms(icp10111::RESET_RETRY_MS)
        .expect_write(icp10111::ADDRESS, &icp10111::READ_ID)
        .then_fail()
        .expect_write(icp10111::GENERAL_CALL_ADDRESS, &icp10111::GENERAL_CALL_RESET)
        .expect_delay_ms(icp10111::RESET_RETRY_MS)
        .expect_write(icp10111::ADDRESS, &icp10111::READ_ID)
        .then_fail()
        .expect_write(icp10111::GENERAL_CALL_ADDRESS, &icp10111::GENERAL_CALL_RESET)
        .expect_delay_ms(icp10111::RESET_RETRY_MS)
        .expect_write(icp10111::ADDRESS, &icp10111::READ_ID)
        .then_fail()
        .expect_write(icp10111::ADDRESS, &icp10111::READ_ID)
        .then_fail();
    let mut barometer = Icp10111::new(icp10111::ADDRESS, 100);
    assert!(barometer.setup(&mut shared.clone(), &mut shared.delay()).is_err());
    shared.done();
    // the ICP never came up, yet its general calls reset the peers: the owner is told, and walks them
    assert!(barometer.take_general_call());
    let mut peers: Vec<(&str, Member)> = vec![("baro_bmp581", Box::new(Bmp581::new(bmp581::ADDRESS)))];
    script_bmp581(&shared);
    assert!(rearm_members(&mut peers, &shared).is_empty());
    shared.done();
}
