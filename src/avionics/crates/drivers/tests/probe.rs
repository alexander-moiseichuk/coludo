//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The two device probes M5 adds for the CC `probe` command and bustune's verdicts (avionics-port.md M5 design
//! decisions, 'Device probes vs bus probe'), over the scripted bus: the ICP-10111's one direct conversion
//! ([`Icp10111::probe`]; plan B's waited for the run loop's fresh pressure instead, icp10111.py:398-426) and the VL53
//! family's 16-bit model id ([`Vl53::probe`]; vl53l1x.py:281-307, vl53l4cx.py:277-301).
//!
//! POSITIVE: the frame is decoded into plan B's reading, the id comes back whole for both silicons, at the
//! configured address, a conversion the run loop has in flight is waited out rather than collided with, and a
//! probe during the recovery leaves the general call and the re-arm on time.
//! NEGATIVE: a NAK is [`Fault::Bus`], the other silicon in the socket (or a part sharing only the high byte) is
//! [`Fault::Identity`], a frame failing any of its three CRCs is [`Fault::Checksum`], a probe before setup sends
//! nothing, a drained conversion that fails does not fail the probe, and no probe -- failed or good -- moves the
//! run loop's count toward the general call.

use drivers::commons::sensirion_crc8;
use drivers::device::{Device, Fault};
use drivers::icp10111::{self, Icp10111, Otp, Poll, Reading};
use drivers::mock::{MockBus, MockError, NoPin};
use drivers::vl53l1x::{self, Settings, Silicon, Vl53};
use drivers::vl53l4cx;
use serde_json::json;

/// test_icp10111.py's OTP constants, captured from the wired sensor.
const OTP: Otp = [211, 371, 553, 3833];

/// test_icp10111.py's live frame, captured from the real part: pressure 0xAE543D, temperature 0x6E7A.
const LIVE: [u8; 9] = [0xAE, 0x54, 0xA9, 0x3D, 0x00, 0x73, 0x6E, 0x7A, 0x9A];

/// The run loop's period in these tests, ms.
const PERIOD_MS: u32 = 100;

/// One millisecond, in the run loop's microseconds.
const MILLISECOND: u64 = 1000;

/// A frame with good CRCs for these raw words.
fn frame(pressure_raw: u32, temperature_raw: u16) -> [u8; 9] {
    let [_, high, middle, low] = pressure_raw.to_be_bytes();
    let [temperature_high, temperature_low] = temperature_raw.to_be_bytes();
    [
        high,
        middle,
        sensirion_crc8(high, middle),
        low,
        0x00,
        sensirion_crc8(low, 0x00),
        temperature_high,
        temperature_low,
        sensirion_crc8(temperature_high, temperature_low),
    ]
}

/// A word answer (the id or an OTP constant) and its CRC.
fn word(value: i16) -> [u8; 3] {
    let [high, low] = value.to_be_bytes();
    [high, low, sensirion_crc8(high, low)]
}

/// Script one blocking conversion answered with `answer`: the command, the wait, the frame.
fn script_conversion(mock: &MockBus, answer: &[u8; 9]) {
    mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE)
        .expect_delay_ms(icp10111::MEASURE_MS)
        .expect_read(icp10111::ADDRESS, answer);
}

/// An ICP-10111 brought up over a clean script: the first touch, the id, the OTP and the ground-zero conversions.
fn barometer() -> (MockBus, Icp10111) {
    let mock = MockBus::new();
    for _ in 0..2 {
        mock.expect_write(icp10111::ADDRESS, &icp10111::READ_ID)
            .expect_read(icp10111::ADDRESS, &word(0x0848));
    }
    mock.expect_write(icp10111::ADDRESS, &icp10111::OTP_UNLOCK);
    for constant in OTP {
        mock.expect_write(icp10111::ADDRESS, &icp10111::OTP_READ)
            .expect_read(icp10111::ADDRESS, &word(constant));
    }
    for _ in 0..icp10111::GROUND_SAMPLES {
        script_conversion(&mock, &frame(11_441_968, 27_034));
    }
    let mut barometer = Icp10111::new(icp10111::ADDRESS, PERIOD_MS);
    barometer
        .setup(&mut mock.clone(), &mut mock.delay())
        .expect("a clean bring-up");
    mock.done();
    (mock, barometer)
}

/// A VL53 ranger for `silicon` with no XSHUT, at the address the config gives (default 0x29).
fn ranger(silicon: &'static Silicon, config: serde_json::Value) -> Vl53<NoPin> {
    Vl53::new(silicon, &Settings::from_config(&config), None)
}

#[test]
fn the_icp_probe_decodes_one_conversion() {
    let (mock, mut barometer) = barometer();
    let ground = barometer.ground();
    script_conversion(&mock, &LIVE);
    let reading = barometer
        .probe(&mut mock.clone(), &mut mock.delay())
        .expect("the live frame");
    mock.done();
    // the live frame through the live OTP: plan B's _measure answer (drivers_icp10111.json, the_live_frame),
    // ~1017.1 hPa at 30.52 degrees C, the temperature exact in f32
    assert_eq!(reading, Reading::convert(&OTP, 0xAE543D, 0x6E7A));
    assert!(
        (f64::from(reading.pressure) - 101_708.494_281).abs() < 0.05,
        "{reading:?}"
    );
    assert_eq!(f64::from(reading.temperature), 30.521_087_646_484_375);
    // a self-test, not a calibrate: the ground zero setup captured is left alone
    assert_eq!(barometer.ground(), ground);
}

#[test]
fn the_icp_probe_waits_out_the_conversion_in_flight() {
    // the owner serves a probe between two polls: the run loop's conversion is collected and dropped, then the
    // probe's own runs, and the run loop issues afresh instead of collecting a frame that is no longer coming
    let (mock, mut barometer) = barometer();
    let mut bus = mock.clone();
    mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE);
    assert_eq!(barometer.poll(&mut bus, 0), Poll::Issued);
    mock.expect_delay_ms(icp10111::MEASURE_MS)
        .expect_read(icp10111::ADDRESS, &frame(11_000_000, 27_034));
    script_conversion(&mock, &LIVE);
    let reading = barometer
        .probe(&mut bus, &mut mock.delay())
        .expect("the probe's own frame");
    assert_eq!(
        reading,
        Reading::convert(&OTP, 0xAE543D, 0x6E7A),
        "the probe's frame, not the dropped one"
    );
    mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE);
    assert_eq!(barometer.poll(&mut bus, 12 * MILLISECOND), Poll::Issued);
    mock.done();
}

#[test]
fn an_icp_nak_is_a_bus_fault() {
    // NEGATIVE: the command NAKs (a latched part), then the frame read NAKs (a part gone mid-conversion)
    let (mock, mut barometer) = barometer();
    mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE).then_fail();
    let outcome = barometer.probe(&mut mock.clone(), &mut mock.delay());
    assert_eq!(outcome, Err(Fault::Bus(MockError)));
    mock.done();
    mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE)
        .expect_delay_ms(icp10111::MEASURE_MS)
        .expect_read(icp10111::ADDRESS, &LIVE)
        .then_fail();
    let outcome = barometer.probe(&mut mock.clone(), &mut mock.delay());
    assert_eq!(outcome, Err(Fault::Bus(MockError)));
    mock.done();
}

#[test]
fn an_icp_frame_failing_any_crc_is_refused() {
    // NEGATIVE: one flipped bit in each byte of the frame, data and CRC alike, fails that word's CRC
    let (mock, mut barometer) = barometer();
    for byte in 0..LIVE.len() {
        let mut corrupt = LIVE;
        corrupt[byte] ^= 0x01;
        script_conversion(&mock, &corrupt);
        let outcome = barometer.probe(&mut mock.clone(), &mut mock.delay());
        assert_eq!(outcome, Err(Fault::Checksum), "byte {byte}");
    }
    mock.done();
}

#[test]
fn an_icp_probe_before_setup_sends_nothing() {
    // NEGATIVE: no OTP, no conversion to make sense of -- refused before the bus is touched
    let mock = MockBus::new();
    let mut barometer = Icp10111::new(icp10111::ADDRESS, PERIOD_MS);
    let outcome = barometer.probe(&mut mock.clone(), &mut mock.delay());
    assert_eq!(outcome, Err(Fault::Configuration));
    mock.done();
    assert_eq!(mock.performed(), 0);
}

#[test]
fn an_icp_probe_never_moves_the_run_loop_s_count() {
    // plan B's probe never touched the part, so it could not move the recovery's count either way: the general call
    // comes on the run loop's own RECOVER_AFTER-th failure, whatever the operator probed in between
    assert_eq!(
        icp10111::RECOVER_AFTER,
        3,
        "the run this test walks: two failures, a good probe, the third"
    );
    let (mock, mut barometer) = barometer();
    let mut bus = mock.clone();
    let fail_poll = |barometer: &mut Icp10111, bus: &mut MockBus, at_ms: u64| {
        mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE).then_fail();
        match barometer.poll(bus, at_ms * MILLISECOND) {
            Poll::Failed { recovering, .. } => recovering,
            other => panic!("a failed issue expected at {at_ms} ms, got {other:?}"),
        }
    };
    // NEGATIVE: failed probes are not strikes: one short of a run, so striking probes would make the run loop's
    // FIRST failure the escalating one (a strike fires only on the failure that brings the run to the limit) ...
    for _ in 0..icp10111::RECOVER_AFTER - 1 {
        mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE).then_fail();
        assert!(barometer.probe(&mut bus, &mut mock.delay()).is_err());
    }
    assert!(
        !fail_poll(&mut barometer, &mut bus, 0),
        "the failed probes counted as strikes"
    );
    assert!(!fail_poll(&mut barometer, &mut bus, 100));
    // ... and a good probe does not rearm the run: the next run-loop failure completes it, and escalates
    script_conversion(&mock, &LIVE);
    barometer
        .probe(&mut bus, &mut mock.delay())
        .expect("a good probe between failures");
    assert!(
        fail_poll(&mut barometer, &mut bus, 200),
        "the good probe rearmed the run"
    );
    mock.done();
}

#[test]
fn an_icp_probe_answers_its_own_conversion_when_the_drained_one_fails() {
    // NEGATIVE: the run loop's conversion in flight NAKs on its collect, or fails its CRC; the probe drops that
    // outcome uncounted and still answers with its own conversion, and the run loop issues afresh
    let mut corrupt = LIVE;
    corrupt[2] ^= 0x01;
    for nak in [true, false] {
        let (mock, mut barometer) = barometer();
        let mut bus = mock.clone();
        mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE);
        assert_eq!(barometer.poll(&mut bus, 0), Poll::Issued);
        mock.expect_delay_ms(icp10111::MEASURE_MS);
        if nak {
            mock.expect_read(icp10111::ADDRESS, &LIVE).then_fail();
        } else {
            mock.expect_read(icp10111::ADDRESS, &corrupt);
        }
        script_conversion(&mock, &LIVE);
        let outcome = barometer.probe(&mut bus, &mut mock.delay());
        assert_eq!(outcome, Ok(Reading::convert(&OTP, 0xAE543D, 0x6E7A)), "nak {nak}");
        mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE);
        assert_eq!(barometer.poll(&mut bus, 24 * MILLISECOND), Poll::Issued, "nak {nak}");
        mock.done();
    }
}

#[test]
fn an_icp_probe_during_the_recovery_leaves_it_on_time() {
    // the run loop's third failed issue starts the recovery: the general call a period later, then the re-arm
    // RESET_RETRY_MS after that; a good probe before the call and a failed one while the part settles move neither
    let (mock, mut barometer) = barometer();
    let mut bus = mock.clone();
    let period = u64::from(PERIOD_MS) * MILLISECOND;
    for pass in 1..=u64::from(icp10111::RECOVER_AFTER) {
        mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE).then_fail();
        match barometer.poll(&mut bus, (pass - 1) * period) {
            Poll::Failed { recovering, .. } => {
                assert_eq!(recovering, pass == u64::from(icp10111::RECOVER_AFTER), "pass {pass}")
            }
            other => panic!("a failed issue expected on pass {pass}, got {other:?}"),
        }
    }
    let escalated = u64::from(icp10111::RECOVER_AFTER - 1) * period;
    script_conversion(&mock, &LIVE);
    assert!(
        barometer.probe(&mut bus, &mut mock.delay()).is_ok(),
        "a part answering before the call"
    );
    assert_eq!(barometer.poll(&mut bus, escalated + period - 1), Poll::Waiting);
    mock.expect_write(icp10111::GENERAL_CALL_ADDRESS, &icp10111::GENERAL_CALL_RESET);
    assert_eq!(barometer.poll(&mut bus, escalated + period), Poll::GeneralCall);
    // NEGATIVE: a part still resetting NAKs the probe; the settle runs its course regardless
    mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE).then_fail();
    assert_eq!(barometer.probe(&mut bus, &mut mock.delay()), Err(Fault::Bus(MockError)));
    let settled = escalated + period + u64::from(icp10111::RESET_RETRY_MS) * MILLISECOND;
    assert_eq!(barometer.poll(&mut bus, settled - 1), Poll::Waiting);
    assert_eq!(barometer.poll(&mut bus, settled), Poll::Rearm);
    mock.expect_write(icp10111::ADDRESS, &icp10111::MEASURE);
    assert_eq!(barometer.poll(&mut bus, settled + period), Poll::Issued);
    mock.done();
}

#[test]
fn the_vl53_probe_answers_the_whole_model_id_of_either_silicon() {
    for (silicon, answer) in [(&vl53l1x::VL53L1X, [0xEA, 0xCC]), (&vl53l4cx::VL53L4CX, [0xEB, 0xAA])] {
        // one register read and nothing else: no XSHUT pulse, no boot wait, no range read
        let mock = MockBus::new();
        mock.expect_write_read(vl53l1x::ADDRESS, &[0x01, 0x0F], &answer);
        let outcome = ranger(silicon, json!({})).probe(&mut mock.clone());
        assert_eq!(outcome, Ok(silicon.model_id), "{}", silicon.name);
        assert_eq!(outcome, Ok(u16::from_be_bytes(answer)));
        mock.done();
    }
    // at the address the config gives, not the default
    let mock = MockBus::new();
    mock.expect_write_read(0x30, &[0x01, 0x0F], &[0xEA, 0xCC]);
    assert_eq!(
        ranger(&vl53l1x::VL53L1X, json!({"addr": 0x30})).probe(&mut mock.clone()),
        Ok(0xEACC)
    );
    mock.done();
}

#[test]
fn the_vl53_probe_refuses_the_other_silicon() {
    // NEGATIVE: each driver refuses the other's silicon in the shared 0x29 socket, and a part that shares only the
    // high byte (the one-byte check plan B's L1X probe once made) is refused too
    let cases: [(&Silicon, u16); 4] = [
        (&vl53l1x::VL53L1X, 0xEBAA),
        (&vl53l4cx::VL53L4CX, 0xEACC),
        (&vl53l1x::VL53L1X, 0xEA00),
        (&vl53l4cx::VL53L4CX, 0xEB00),
    ];
    for (silicon, read) in cases {
        let mock = MockBus::new();
        mock.expect_write_read(vl53l1x::ADDRESS, &[0x01, 0x0F], &read.to_be_bytes());
        let outcome = ranger(silicon, json!({})).probe(&mut mock.clone());
        assert_eq!(
            outcome,
            Err(Fault::Identity {
                expected: silicon.model_id,
                read
            }),
            "{} on 0x{read:04x}",
            silicon.name
        );
        mock.done();
    }
}

#[test]
fn a_vl53_nak_is_a_bus_fault() {
    // NEGATIVE: an empty socket NAKs the id read
    let mock = MockBus::new();
    mock.expect_write_read(vl53l1x::ADDRESS, &[0x01, 0x0F], &[0xEA, 0xCC])
        .then_fail();
    let outcome = ranger(&vl53l1x::VL53L1X, json!({})).probe(&mut mock.clone());
    assert_eq!(outcome, Err(Fault::Bus(MockError)));
    mock.done();
}
