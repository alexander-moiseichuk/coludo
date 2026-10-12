//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The parts' runs on the drivers' mock bus: what a tick publishes for the executor, and the inspect keys read back
//! from it, positive and negative. No databoard quantities and no device table here: the runs' channel writes and
//! health are the drivers' and the controller's tests; these cover what R2 moved out of the firmware.

use std::sync::atomic::{AtomicU32, Ordering};

use controller::Topology;
use drivers::axes::Axes;
use drivers::ina226::{self, Ina226};
use drivers::mock::{MockBus, MockPin};
use parts::{Lookup, Member, Own, Part, Platform, Run, Shared, own};
use recorder::Rows;
use serde_json::json;

/// A platform with no interrupt service, a settable ALERT count and no recorder.
struct Bench {
    alerts: AtomicU32,
}

impl Platform for Bench {
    fn interrupts_ready(&self) -> bool {
        false
    }

    fn arm_second_interrupt(&self, _gpio: u8) -> Result<(), String> {
        Err("no interrupt service".to_string())
    }

    fn take_second_edges(&self) -> u32 {
        0
    }

    fn arm_alert_interrupt(&self, _gpio: u8) -> Result<(), String> {
        Err("no interrupt service".to_string())
    }

    fn alert_edges(&self) -> u32 {
        self.alerts.load(Ordering::Relaxed)
    }

    fn lapses(&self, _name: &str) -> Vec<(String, u32)> {
        vec![("accel".to_string(), 3)]
    }

    fn rows(&self, name: &'static str) -> Rows<'static> {
        Rows::new(name, "row:", None)
    }
}

/// The board's quantities: none.
fn topology() -> &'static Topology {
    Box::leak(Box::new(Topology::default()))
}

/// A bench platform for the board's lifetime, its ALERT count `alerts`.
fn bench(alerts: u32) -> &'static Bench {
    Box::leak(Box::new(Bench {
        alerts: AtomicU32::new(alerts),
    }))
}

/// The power monitor `power_ina226`, with its run, on a 10 ms period.
fn power_member(platform: &'static Bench) -> Member<MockPin> {
    let monitor = Ina226::new(&ina226::Settings::from_config(&json!({}), false));
    let mut member = Member::new("power_ina226", Part::Ina226(monitor), Axes::default(), None, Some(10));
    member.run = Run::new(&member, topology(), None, platform);
    member
}

/// One INA226 read on the mock: the bus voltage, the current, the power registers.
fn reading(mock: &MockBus) {
    mock.expect_write_read(ina226::ADDRESS, &[ina226::REGISTER_BUS_VOLTAGE], &[0x0F, 0xA0])
        .expect_write_read(ina226::ADDRESS, &[ina226::REGISTER_CURRENT], &[0x00, 0x64])
        .expect_write_read(ina226::ADDRESS, &[ina226::REGISTER_POWER], &[0x00, 0x0A]);
}

/// One owner wake of `member` at `now_us` on `mock`.
fn tick(member: &mut Member<MockPin>, mock: &MockBus, now_us: u64) {
    let name = member.name;
    let Member { part, run, .. } = member;
    part.tick(
        name,
        run.as_mut().expect("a run"),
        &mut mock.clone(),
        &mut mock.delay(),
        now_us,
    );
}

/// `inspect`'s own keys of `member`, as JSON.
fn inspected(implementation: &str, member: &Member<MockPin>, platform: &dyn Platform) -> Option<serde_json::Value> {
    let lookup = Lookup {
        topology: topology(),
        name: member.name,
        now_us: 0,
        addr: Some("0x40".to_string()),
    };
    own(implementation, member.shared, &lookup, platform).map(|own: Own| serde_json::to_value(own).expect("json"))
}

#[test]
fn a_power_reading_is_published_with_the_alerts() {
    let platform = bench(2);
    let mut member = power_member(platform);
    let mock = MockBus::new();
    reading(&mock);
    tick(&mut member, &mock, 0);
    mock.done();
    // the driver's own conversion of the same registers
    let expected = {
        let check = MockBus::new();
        reading(&check);
        let monitor = Ina226::new(&ina226::Settings::from_config(&json!({}), false));
        monitor.read(&mut check.clone()).expect("read")
    };
    let Some(Shared::Power(published)) = member.shared else {
        panic!("a power monitor publishes its readings");
    };
    let state = published.read().expect("a reading");
    assert_eq!(
        [state.voltage_mv, state.current_ma, state.power_mw],
        [expected.voltage_mv, expected.current_ma, expected.power_mw]
    );
    let keys = inspected("ina226", &member, platform).expect("keys");
    assert_eq!(keys["voltage_mv"], json!(expected.voltage_mv));
    assert_eq!(keys["alerts"], json!(2));
}

#[test]
fn a_failed_power_read_publishes_nothing() {
    // NEGATIVE: the bus fails the first register: no reading, and the inspect keys stay null with no alerts
    let platform = bench(5);
    let mut member = power_member(platform);
    let mock = MockBus::new();
    mock.expect_write_read(ina226::ADDRESS, &[ina226::REGISTER_BUS_VOLTAGE], &[0, 0])
        .then_fail();
    tick(&mut member, &mock, 0);
    mock.done();
    let Some(Shared::Power(published)) = member.shared else {
        panic!("a power monitor publishes its readings");
    };
    assert_eq!(published.read(), None);
    let keys = inspected("ina226", &member, platform).expect("keys");
    assert_eq!(keys["voltage_mv"], json!(null));
    assert_eq!(keys["alerts"], json!(0), "no reading: no alerts shown");
}

#[test]
fn a_power_read_waits_for_its_period() {
    // NEGATIVE: a second wake inside the period reads nothing (the mock would panic at an unscripted call)
    let platform = bench(0);
    let mut member = power_member(platform);
    let mock = MockBus::new();
    reading(&mock);
    tick(&mut member, &mock, 0);
    tick(&mut member, &mock, 9_999);
    mock.done();
    reading(&mock);
    tick(&mut member, &mock, 10_000);
    mock.done();
}

#[test]
fn the_keys_keep_plan_b_s_order() {
    // a part that came up shows its kind's keys from its published state, in plan B's order
    // (the text, as the firmware answers it: a JSON Value would sort the keys)
    let platform = bench(0);
    let member = power_member(platform);
    let lookup = Lookup {
        topology: topology(),
        name: member.name,
        now_us: 0,
        addr: None,
    };
    let text = serde_json::to_string(&own("ina226", member.shared, &lookup, platform).expect("keys")).expect("json");
    assert_eq!(
        text,
        r#"{"voltage_mv":null,"current_ma":null,"power_mw":null,"alerts":0}"#
    );
}

#[test]
fn a_part_with_no_state_shows_its_kind_s_empty_keys() {
    let platform = bench(0);
    let lookup = Lookup {
        topology: topology(),
        name: "imu_bmi323",
        now_us: 0,
        addr: None,
    };
    let keys = serde_json::to_value(own("bmi323", None, &lookup, platform).expect("keys")).expect("json");
    assert_eq!(keys["interrupt"], json!(false));
    assert_eq!(keys["odr_hz"], json!(null));
    assert_eq!(keys["lapses"], json!({"accel": 3}));
    assert_eq!(keys["raced"], json!([0, 0]));
    let keys = serde_json::to_value(own("bmp581", None, &lookup, platform).expect("keys")).expect("json");
    assert_eq!(keys["torn"], json!(0));
    let keys = serde_json::to_value(own("sdp810", None, &lookup, platform).expect("keys")).expect("json");
    assert_eq!(keys["tared"], json!(null));
}

#[test]
fn parts_without_keys_here_are_left_to_the_firmware() {
    // NEGATIVE: the LSM6DSO32 (the control thread's on SPI), a BNO055 with no state, any non-I2C part
    let platform = bench(0);
    let lookup = Lookup {
        topology: topology(),
        name: "x",
        now_us: 0,
        addr: None,
    };
    for implementation in ["lsm6dso32", "bno055", "neo6mv2", "sg90", "sequencer"] {
        assert!(
            own(implementation, None, &lookup, platform).is_none(),
            "{implementation}"
        );
    }
}

#[test]
fn a_pitot_s_state_is_published_from_its_coming_up() {
    let pitot = drivers::sdp810::Sdp810::new(&drivers::sdp810::Settings::from_config(&json!({})));
    let member: Member<MockPin> = Member::new("airspeed_sdp810", Part::Sdp810(pitot), Axes::default(), None, None);
    let Some(Shared::Pitot(shared)) = member.shared else {
        panic!("a pitot publishes its state");
    };
    let state = shared.state().expect("published when it came up");
    assert!(!state.tared);
    assert_eq!(shared.take_owed(), None, "nothing owed before a calibrate");
}

#[test]
fn a_restored_pitot_zero_is_published_but_never_owed() {
    // the warm start's tare: in force and shown, never written to flash mid-air
    let pitot = drivers::sdp810::Sdp810::new(&drivers::sdp810::Settings::from_config(&json!({})));
    let mut member: Member<MockPin> = Member::new("airspeed_sdp810", Part::Sdp810(pitot), Axes::default(), None, None);
    assert!(member.restore_pitot_zero(-1.5));
    let Some(Shared::Pitot(shared)) = member.shared else {
        panic!("a pitot publishes its state");
    };
    assert_eq!(shared.state().expect("state").zero_offset_pa, -1.5);
    assert_eq!(shared.take_owed(), None);
}

#[test]
fn only_a_pitot_takes_a_pitot_zero() {
    // NEGATIVE
    let mut member = power_member(bench(0));
    assert!(!member.restore_pitot_zero(-1.5));
}

#[test]
fn a_member_without_a_blob_refuses_one() {
    // NEGATIVE: only the BMM350 and the BNO055 take a calibration blob
    let mut member = power_member(bench(0));
    assert_eq!(
        member.apply_calibration(&[1, 2, 3]),
        Some("power_ina226 takes no calibration blob".to_string())
    );
}
