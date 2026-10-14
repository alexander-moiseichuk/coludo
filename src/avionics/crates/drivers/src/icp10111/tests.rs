//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ICP-10111's unit tests: the frame CRCs on test_icp10111.py's live capture (positive and one flipped bit), the
//! polynomial on its live OTP, the hardened first touch and its general call, and the two-phase run loop with its
//! recovery, each against the scripted bus and a clock passed in. The polynomial's f32-against-f64 verdict and plan
//! B's own transcripts are in the golden fixture drivers_icp10111.json.

use control::commons::altitude_m;
use serde_json::json;

use super::{
    ADDRESS, GENERAL_CALL_ADDRESS, GENERAL_CALL_RESET, GROUND_INSTRUCTION, GROUND_SAMPLES, Icp10111, MEASURE,
    MEASURE_MS, OTP_READ, OTP_UNLOCK, Otp, Poll, READ_ID, RESET_RETRY_MS, Reading, Refusal, compensate, decode,
    temperature,
};
use crate::commons::sensirion_crc8;
use crate::device::{Device, Fault};
use crate::mock::{MockBus, MockError};

/// test_icp10111.py's OTP constants, captured from the wired sensor.
const OTP: Otp = [211, 371, 553, 3833];

/// test_icp10111.py's live frame, captured from the real part.
const LIVE: [u8; 9] = [0xAE, 0x54, 0xA9, 0x3D, 0x00, 0x73, 0x6E, 0x7A, 0x9A];

/// One millisecond, in the run loop's microseconds.
const MILLISECOND: u64 = 1000;

/// The run loop's period in these tests, ms.
const PERIOD_MS: u64 = 100;

/// The owner's tick on the board, in the run loop's microseconds: the 100 Hz control clock.
const TICK: u64 = 10 * MILLISECOND;

/// Poll at each wake (microseconds), keeping every event but [`Poll::Waiting`] with its time.
fn wake(
    barometer: &mut Icp10111,
    bus: &mut MockBus,
    wakes: impl IntoIterator<Item = u64>,
) -> Vec<(u64, Poll<MockError>)> {
    wakes
        .into_iter()
        .filter_map(|now| match barometer.poll(bus, now) {
            Poll::Waiting => None,
            event => Some((now, event)),
        })
        .collect()
}

/// `count` milliseconds in the run loop's microseconds.
fn milliseconds(count: u64) -> u64 {
    count * MILLISECOND
}

/// The owner's wakes on an exact 10 ms grid, from `from` through `until` (microseconds).
fn ticks(from: u64, until: u64) -> impl Iterator<Item = u64> {
    (from..=until).step_by(TICK as usize)
}

/// A phase the bus failed with `error`, as the run loop reports it.
fn bus_failed(error: MockError, recovering: bool) -> Poll<MockError> {
    Poll::Failed {
        fault: Fault::Bus(error),
        recovering,
    }
}

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

/// Script the id command and its answer.
fn script_identity(mock: &MockBus, identity: i16) {
    mock.expect_write(ADDRESS, &READ_ID)
        .expect_read(ADDRESS, &word(identity));
}

/// Script everything after the first touch: the id, the OTP and the ground-zero conversions.
fn script_after_first_touch(mock: &MockBus) {
    script_identity(mock, 0x0848);
    mock.expect_write(ADDRESS, &OTP_UNLOCK);
    for constant in OTP {
        mock.expect_write(ADDRESS, &OTP_READ)
            .expect_read(ADDRESS, &word(constant));
    }
    for _ in 0..GROUND_SAMPLES {
        mock.expect_write(ADDRESS, &MEASURE)
            .expect_delay_ms(MEASURE_MS)
            .expect_read(ADDRESS, &frame(11_441_968, 27_034));
    }
}

/// A driver brought up over a clean script, with its ground zero at the live reading.
fn ready() -> (MockBus, Icp10111) {
    let mock = MockBus::new();
    script_identity(&mock, 0x0848);
    script_after_first_touch(&mock);
    let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    barometer.setup(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    (mock, barometer)
}

#[test]
fn the_live_frame_passes_its_crcs_and_a_flipped_bit_does_not() {
    // POSITIVE: the captured frame validates; NEGATIVE: one flipped data bit, CRC untouched, is refused. Both matter:
    // a validator that accepts everything is worthless, one that rejects everything takes out the primary barometer
    assert_eq!(decode(&LIVE), Some((0xAE543D, 0x6E7A)));
    for byte in [0, 1, 3, 4, 6, 7] {
        let mut corrupt = LIVE;
        corrupt[byte] ^= 0x01;
        assert_eq!(decode(&corrupt), None, "byte {byte}");
    }
}

#[test]
fn the_polynomial_reads_the_live_pressure() {
    // the live OTP and raw words -> ~1017.97 hPa, as the wired sensor read
    let pressure = compensate(&OTP, 11_441_968, 27_034);
    assert!((pressure - 101_797.24).abs() < 0.05, "{pressure}");
    // NEGATIVE: another part's OTP gives another pressure (c3, the offset constant, dominates at room temperature)
    assert!((compensate(&[211, 371, 553, 3800], 11_441_968, 27_034) - pressure).abs() > 1.0);
    assert!(
        (temperature(27_034) - 27.188_568).abs() < 1e-4,
        "{}",
        temperature(27_034)
    );
    assert_eq!(temperature(0), -45.0);
}

#[test]
fn the_temperature_is_exact_to_the_last_count() {
    // -45 + 175 x raw / 2^16 needs at most 24 significant bits, so f32 holds it exactly: (175 x raw - 45 x 2^16) / 2^16
    let exact = |raw: u16| (175 * i32::from(raw) - 45 * 65_536) as f32 / 65_536.0;
    for raw in [0, 1, 27_034, 27_035, 32_767, 32_768, 48_685, 65_535] {
        assert_eq!(temperature(raw), exact(raw), "raw {raw}");
    }
    // NEGATIVE: an odd count is its own temperature, a 175/65536 degree step from its even neighbour (a decode that
    // drops the low bit reads 27034's temperature here)
    assert_ne!(temperature(27_035), temperature(27_034));
    assert_eq!(temperature(27_035) - temperature(27_034), 175.0 / 65_536.0);
}

#[test]
fn the_altitude_is_the_shared_one_of_the_unrounded_pressure() {
    // the altitude channel is commons.altitude_m of the very pressure published beside it, never a rounded copy
    for (pressure_raw, temperature_raw) in [(11_441_968, 27_034), (7_000_000, 1_872), (13_500_000, 48_684)] {
        let reading = Reading::convert(&OTP, pressure_raw, temperature_raw);
        assert_eq!(reading.pressure, compensate(&OTP, pressure_raw, temperature_raw));
        assert_eq!(reading.altitude, altitude_m(reading.pressure));
        assert_eq!(reading.temperature, temperature(temperature_raw));
    }
}

#[test]
fn a_clean_boot_touches_the_part_plainly() {
    let (_, barometer) = ready();
    assert_eq!(barometer.otp(), Some(OTP));
    assert!(barometer.ground().is_some());
    assert_eq!(barometer.instruction(), "", "a held ground zero leaves nothing to do");
}

#[test]
fn a_busy_part_is_waited_out_and_a_latched_one_gets_the_general_call() {
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &READ_ID)
        .then_fail()
        .expect_delay_ms(RESET_RETRY_MS) // first NAK: a conversion draining, only wait
        .expect_write(ADDRESS, &READ_ID)
        .expect_read(ADDRESS, &[0; 3])
        .then_fail()
        .expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET)
        .expect_delay_ms(RESET_RETRY_MS);
    script_identity(&mock, 0x0848); // the third touch answers
    script_after_first_touch(&mock);
    let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    barometer.setup(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    assert!(
        barometer.take_general_call(),
        "the owner must re-arm the peers the general call reset"
    );
    assert!(!barometer.take_general_call(), "told once");
}

#[test]
fn a_dead_part_is_reported_and_its_general_calls_are_still_told() {
    // NEGATIVE: four failed touches (general calls on the 2nd and 3rd, none after the last), then the id read fails.
    // Plan B re-armed no peer here; the port tells the owner, because the general calls DID reset them
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &READ_ID)
        .then_fail()
        .expect_delay_ms(RESET_RETRY_MS);
    for _ in 0..2 {
        mock.expect_write(ADDRESS, &READ_ID)
            .then_fail()
            .expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET)
            .then_fail()
            .expect_delay_ms(RESET_RETRY_MS);
    }
    mock.expect_write(ADDRESS, &READ_ID)
        .then_fail()
        .expect_write(ADDRESS, &READ_ID)
        .then_fail();
    let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    assert!(matches!(
        barometer.setup(&mut mock.clone(), &mut mock.delay()),
        Err(Fault::Bus(_))
    ));
    mock.done();
    assert!(barometer.take_general_call());
    assert_eq!(barometer.instruction(), GROUND_INSTRUCTION);
}

#[test]
fn another_part_and_a_half_recovered_part_fail_setup() {
    // NEGATIVE: the id's low six bits are not 0x08
    let mock = MockBus::new();
    script_identity(&mock, 0x0848);
    script_identity(&mock, 0x0849);
    let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    assert_eq!(
        barometer.setup(&mut mock.clone(), &mut mock.delay()),
        Err(Fault::Identity {
            expected: 0x08,
            read: 0x09
        })
    );
    // NEGATIVE: id and OTP ack, the first measure NAKs -- the post-latch-up state; setup fails gracefully
    script_identity(&mock, 0x0848);
    script_identity(&mock, 0x0848);
    mock.expect_write(ADDRESS, &OTP_UNLOCK);
    for constant in OTP {
        mock.expect_write(ADDRESS, &OTP_READ)
            .expect_read(ADDRESS, &word(constant));
    }
    mock.expect_write(ADDRESS, &MEASURE).then_fail();
    assert!(matches!(
        barometer.setup(&mut mock.clone(), &mut mock.delay()),
        Err(Fault::Bus(_))
    ));
    assert!(!barometer.take_general_call());
    mock.done();
}

#[test]
fn the_run_loop_issues_collects_and_keeps_the_cadence() {
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    mock.expect_write(ADDRESS, &MEASURE)
        .expect_read(ADDRESS, &frame(11_441_968, 27_034))
        .expect_write(ADDRESS, &MEASURE);
    assert_eq!(barometer.poll(&mut bus, 0), Poll::Issued);
    assert_eq!(
        barometer.poll(&mut bus, 11 * MILLISECOND),
        Poll::Waiting,
        "not before the 12 ms conversion"
    );
    let Poll::Reading(reading) = barometer.poll(&mut bus, 12 * MILLISECOND) else {
        panic!("the conversion is due");
    };
    assert!((reading.pressure - 101_797.24).abs() < 0.05);
    assert!(
        barometer.elevation(&reading).unwrap().abs() < 1e-3,
        "the ground zero is this reading"
    );
    // the next measurement a period after the collect
    assert_eq!(barometer.poll(&mut bus, 111 * MILLISECOND), Poll::Waiting);
    assert_eq!(barometer.poll(&mut bus, 112 * MILLISECOND), Poll::Issued);
    mock.done();
}

#[test]
fn a_crc_reject_counts_like_a_nak() {
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    let mut corrupt = frame(11_441_968, 27_034);
    corrupt[1] ^= 0x01;
    mock.expect_write(ADDRESS, &MEASURE).expect_read(ADDRESS, &corrupt);
    barometer.poll(&mut bus, 0);
    assert_eq!(
        barometer.poll(&mut bus, 12 * MILLISECOND),
        Poll::Failed {
            fault: Fault::Checksum,
            recovering: false
        }
    );
    mock.done();
}

#[test]
fn a_wedged_part_escalates_once_then_recovers_and_a_later_wedge_escalates_again() {
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    let mut now = 0;
    let mut tick = |barometer: &mut Icp10111, bus: &mut MockBus, until: u64| {
        let mut events = Vec::new();
        while now <= until {
            match barometer.poll(bus, now) {
                Poll::Waiting => {}
                event => events.push((now / MILLISECOND, event)),
            }
            now += MILLISECOND;
        }
        events
    };
    // three failed issues, a period apart; the third starts the recovery
    for _ in 0..3 {
        mock.expect_write(ADDRESS, &MEASURE).then_fail();
    }
    // a period to drain, the general call (nothing honours it here), 30 ms to settle, then the re-arm walk
    mock.expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET).then_fail();
    let events = tick(&mut barometer, &mut bus, 429 * MILLISECOND);
    let failed = |recovering| Poll::Failed {
        fault: Fault::Bus(MockError::NAK),
        recovering,
    };
    assert_eq!(
        events,
        [
            (0, failed(false)),
            (100, failed(false)),
            (200, failed(true)),
            (300, Poll::GeneralCall),
            (330, Poll::Rearm),
        ]
    );
    // the part is back: the next issue a period after the re-arm, and a good read rearms the count
    mock.expect_write(ADDRESS, &MEASURE)
        .expect_read(ADDRESS, &frame(11_441_968, 27_034));
    // ...and a LATER wedge escalates again, once
    for _ in 0..3 {
        mock.expect_write(ADDRESS, &MEASURE).then_fail();
    }
    mock.expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET);
    let events = tick(&mut barometer, &mut bus, 971 * MILLISECOND);
    let kinds: Vec<u64> = events.iter().map(|(at, _)| *at).collect();
    assert_eq!(kinds, [430, 442, 542, 642, 742, 842, 872]);
    assert!(matches!(events[1].1, Poll::Reading(_)));
    assert_eq!(events[4].1, failed(true));
    assert_eq!(events[6].1, Poll::Rearm);
    mock.done();
}

/// Script `cycles` good measurements: the issue and its frame.
fn script_cycles(mock: &MockBus, cycles: usize) {
    for _ in 0..cycles {
        mock.expect_write(ADDRESS, &MEASURE)
            .expect_read(ADDRESS, &frame(11_441_968, 27_034));
    }
}

#[test]
fn on_the_exact_tick_grid_the_cycle_is_120_ms() {
    // the board's cadence: the issue at 0, the collect on the 20 ms wake (the 10 ms one is short of the conversion),
    // the next issue a period after the collect -- on a tick boundary, which the 120 ms wake meets exactly
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    script_cycles(&mock, 2);
    mock.expect_write(ADDRESS, &MEASURE);
    let events = wake(&mut barometer, &mut bus, ticks(0, milliseconds(250)));
    let times: Vec<u64> = events.iter().map(|(at, _)| *at).collect();
    assert_eq!(times, [0, 20, 120, 140, 240].map(milliseconds));
    assert!(events.iter().step_by(2).all(|(_, event)| *event == Poll::Issued));
    assert!(
        events
            .iter()
            .skip(1)
            .step_by(2)
            .all(|(_, event)| matches!(event, Poll::Reading(_)))
    );
    mock.done();
}

#[test]
fn a_wake_1_us_earlier_than_the_collect_s_slips_the_issue_to_130_ms() {
    // the collect's wake lands 1 us after its tick and the 120 ms wake exactly on its own: a period after the collect
    // is 120.001 ms, so that wake waits and the issue slips a whole tick; the next collect on time, 120 ms again
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    script_cycles(&mock, 2);
    mock.expect_write(ADDRESS, &MEASURE);
    let late_collect = milliseconds(20);
    let wakes = ticks(0, milliseconds(250)).map(|now| if now == late_collect { now + 1 } else { now });
    let events = wake(&mut barometer, &mut bus, wakes);
    let times: Vec<u64> = events.iter().map(|(at, _)| *at).collect();
    assert_eq!(times, [0, 20_001, 130_000, 150_000, 250_000]);
    assert_eq!(events[2].1, Poll::Issued);
    mock.done();
}

#[test]
fn no_collect_before_the_12_ms_conversion() {
    // NEGATIVE: every wake short of MEASURE_MS after the issue waits, to the microsecond -- a frame read early is the
    // previous conversion's, or a NAK from a part still converting
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    script_cycles(&mock, 1);
    assert_eq!(barometer.poll(&mut bus, 0), Poll::Issued);
    let due = milliseconds(u64::from(MEASURE_MS));
    for early in [1, TICK, due - 1] {
        assert_eq!(barometer.poll(&mut bus, early), Poll::Waiting, "{early} us");
    }
    assert!(matches!(barometer.poll(&mut bus, due), Poll::Reading(_)));
    mock.done();
}

#[test]
fn a_dead_part_escalates_once() {
    // NEGATIVE: three NAKs start the recovery (the general call a period later, the re-arm 30 ms after it); a part
    // that never answers again then fails every period with no second general call
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    for _ in 0..3 {
        mock.expect_write(ADDRESS, &MEASURE).then_fail();
    }
    mock.expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET).then_fail();
    for _ in 0..10 {
        mock.expect_write(ADDRESS, &MEASURE).then_fail();
    }
    let events = wake(&mut barometer, &mut bus, ticks(0, milliseconds(1_330)));
    let mut expected = vec![
        (milliseconds(0), bus_failed(MockError::NAK, false)),
        (milliseconds(100), bus_failed(MockError::NAK, false)),
        (milliseconds(200), bus_failed(MockError::NAK, true)),
        (milliseconds(300), Poll::GeneralCall),
        (milliseconds(330), Poll::Rearm),
    ];
    expected.extend((0..10).map(|cycle| (milliseconds(430 + 100 * cycle), bus_failed(MockError::NAK, false))));
    assert_eq!(events, expected);
    assert_eq!(barometer.strikes.count(), 13, "still counting, never firing again");
    mock.done();
}

#[test]
fn failures_the_port_refuses_start_no_recovery_and_leave_the_run_as_it_was() {
    // NEGATIVE: two NAKs, then three refusals of kind Other -- on an issue, on a collect, on an issue -- each
    // reported, each followed by the next issue a period later, none counted: counted, the first would complete the
    // run. Nor do they clear it: the NAK after them completes the run of three
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    mock.expect_write(ADDRESS, &MEASURE)
        .then_fail()
        .expect_write(ADDRESS, &MEASURE)
        .then_fail()
        .expect_write(ADDRESS, &MEASURE)
        .then_fail_other()
        .expect_write(ADDRESS, &MEASURE)
        .expect_read(ADDRESS, &frame(11_441_968, 27_034))
        .then_fail_other()
        .expect_write(ADDRESS, &MEASURE)
        .then_fail_other();
    let events = wake(&mut barometer, &mut bus, ticks(0, milliseconds(510)));
    assert_eq!(
        events,
        [
            (milliseconds(0), bus_failed(MockError::NAK, false)),
            (milliseconds(100), bus_failed(MockError::NAK, false)),
            (milliseconds(200), bus_failed(MockError::OTHER, false)),
            (milliseconds(300), Poll::Issued),
            (milliseconds(320), bus_failed(MockError::OTHER, false)),
            (milliseconds(420), bus_failed(MockError::OTHER, false)),
        ]
    );
    assert_eq!(barometer.strikes.count(), 2, "the run of NAKs, untouched");
    mock.expect_write(ADDRESS, &MEASURE)
        .then_fail()
        .expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET);
    let events = wake(&mut barometer, &mut bus, ticks(milliseconds(520), milliseconds(650)));
    assert_eq!(
        events,
        [
            (milliseconds(520), bus_failed(MockError::NAK, true)),
            (milliseconds(620), Poll::GeneralCall),
            (milliseconds(650), Poll::Rearm),
        ]
    );
    mock.done();
}

#[test]
fn a_nak_a_refusal_and_two_naks_recover_at_the_third_nak() {
    // NEGATIVE: the refusal between the NAKs neither completes the run (it would at the second NAK) nor restarts it
    // (it would need a fourth)
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    mock.expect_write(ADDRESS, &MEASURE)
        .then_fail()
        .expect_write(ADDRESS, &MEASURE)
        .then_fail_other()
        .expect_write(ADDRESS, &MEASURE)
        .then_fail()
        .expect_write(ADDRESS, &MEASURE)
        .then_fail()
        .expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET)
        .then_fail();
    let events = wake(&mut barometer, &mut bus, ticks(0, milliseconds(520)));
    assert_eq!(
        events,
        [
            (milliseconds(0), bus_failed(MockError::NAK, false)),
            (milliseconds(100), bus_failed(MockError::OTHER, false)),
            (milliseconds(200), bus_failed(MockError::NAK, false)),
            (milliseconds(300), bus_failed(MockError::NAK, true)),
            (milliseconds(400), Poll::GeneralCall),
            (milliseconds(430), Poll::Rearm),
        ]
    );
    mock.done();
}

#[test]
fn a_general_call_the_port_refuses_goes_again_and_only_the_one_on_the_wire_rearms() {
    // NEGATIVE: three NAKs start the recovery, but the port refuses the general call (the bus lock not had: the
    // camera's SCCB on 7C's i2c:0). Nothing was reset, so neither GeneralCall nor Rearm is reported -- the owner
    // would walk the peers for nothing and the run would be spent -- and nothing is issued meanwhile (the mock
    // panics on an unscripted measure). POSITIVE: the call goes again a period later; that one reaches the wire
    // (NAKed, as nothing honours it) and the recovery completes 30 ms after it
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    for _ in 0..3 {
        mock.expect_write(ADDRESS, &MEASURE).then_fail();
    }
    mock.expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET)
        .then_fail_other()
        .expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET)
        .then_fail_other()
        .expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET)
        .then_fail()
        .expect_write(ADDRESS, &MEASURE);
    let events = wake(&mut barometer, &mut bus, ticks(0, milliseconds(630)));
    assert_eq!(
        events,
        [
            (milliseconds(0), bus_failed(MockError::NAK, false)),
            (milliseconds(100), bus_failed(MockError::NAK, false)),
            (milliseconds(200), bus_failed(MockError::NAK, true)),
            (milliseconds(300), bus_failed(MockError::OTHER, false)),
            (milliseconds(400), bus_failed(MockError::OTHER, false)),
            (milliseconds(500), Poll::GeneralCall),
            (milliseconds(530), Poll::Rearm),
            (milliseconds(630), Poll::Issued),
        ]
    );
    assert_eq!(
        barometer.strikes.count(),
        3,
        "the refusals neither count nor clear the run"
    );
    mock.done();
}

#[test]
fn setup_tells_the_owner_of_no_general_call_the_port_refused() {
    // NEGATIVE: the second touch NAKs, and the general call it fires is refused by the port: nothing was reset, so
    // the owner has no peer to re-arm when the third touch answers (a_busy_part_is_waited_out_and_a_latched_one_gets_
    // the_general_call is the positive: the same call NAKed on the wire is told)
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &READ_ID)
        .then_fail()
        .expect_delay_ms(RESET_RETRY_MS)
        .expect_write(ADDRESS, &READ_ID)
        .then_fail()
        .expect_write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET)
        .then_fail_other()
        .expect_delay_ms(RESET_RETRY_MS);
    script_identity(&mock, 0x0848);
    script_after_first_touch(&mock);
    let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    barometer.setup(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    assert!(!barometer.take_general_call());
}

#[test]
fn the_rearm_writes_nothing_and_drops_a_conversion_in_flight() {
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    mock.expect_write(ADDRESS, &MEASURE).expect_write(ADDRESS, &MEASURE);
    assert_eq!(barometer.poll(&mut bus, 0), Poll::Issued);
    barometer.rearm(&mut bus, &mut mock.delay()).unwrap();
    // the general call aborted the conversion: issue afresh rather than collect a frame that is not coming
    assert_eq!(barometer.poll(&mut bus, 12 * MILLISECOND), Poll::Issued);
    mock.done();
}

#[test]
fn a_calibrate_waits_out_the_conversion_in_flight() {
    let (mock, mut barometer) = ready();
    let mut bus = mock.clone();
    mock.expect_write(ADDRESS, &MEASURE)
        .expect_delay_ms(MEASURE_MS)
        .expect_read(ADDRESS, &frame(11_441_968, 27_034));
    for _ in 0..GROUND_SAMPLES {
        mock.expect_write(ADDRESS, &MEASURE)
            .expect_delay_ms(MEASURE_MS)
            .expect_read(ADDRESS, &frame(11_000_000, 27_034));
    }
    assert_eq!(barometer.poll(&mut bus, 0), Poll::Issued);
    let ground = barometer.capture_ground(&mut bus, &mut mock.delay()).unwrap();
    assert_eq!(barometer.ground(), Some(ground));
    mock.done();
}

#[test]
fn the_ground_is_set_or_re_zeroed_from_a_fresh_altitude_only() {
    let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    assert_eq!(barometer.update(&json!({"ground": 7.0}), None), Ok(vec!["ground"]));
    assert_eq!(
        barometer.update(&json!({"rezero": true}), None),
        Err(Refusal::NoFreshAltitude)
    );
    assert_eq!(
        barometer.ground(),
        Some(7.0),
        "a refused re-zero leaves the ground untouched"
    );
    assert_eq!(
        barometer.update(&json!({"rezero": true}), Some(123.5)),
        Ok(vec!["ground"])
    );
    assert_eq!(barometer.ground(), Some(123.5));
    assert_eq!(barometer.update(&json!({"rezero": false}), Some(1.0)), Ok(vec![]));
    assert_eq!(
        barometer.update(&json!({"ground": null}), None),
        Err(Refusal::NotANumber)
    );
}

#[test]
fn the_ground_reads_as_plan_b_s_float_reads_it() {
    // plan B's float(props['ground']) takes a numeral string and a bool, not only a JSON number
    let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    assert_eq!(barometer.update(&json!({"ground": "12.5"}), None), Ok(vec!["ground"]));
    assert_eq!(barometer.ground(), Some(12.5));
    assert_eq!(barometer.update(&json!({"ground": true}), None), Ok(vec!["ground"]));
    assert_eq!(barometer.ground(), Some(1.0));
    // NEGATIVE: a word, a NaN or a list is refused and the ground stays
    for refused in [json!("high"), json!("nan"), json!([12.5])] {
        assert_eq!(
            barometer.update(&json!({"ground": refused}), Some(9.0)),
            Err(Refusal::NotANumber)
        );
    }
    assert_eq!(barometer.ground(), Some(1.0));
    assert_eq!(Refusal::NotANumber.to_string(), "ground must be a number");
}

#[test]
fn the_diagnosis_masks_the_product_id() {
    let mock = MockBus::new();
    script_identity(&mock, 0x0848);
    mock.expect_write(ADDRESS, &READ_ID).then_fail();
    let barometer = Icp10111::new(ADDRESS, PERIOD_MS as u32);
    assert!(barometer.diagnose(&mut mock.clone()).starts_with("id 0x08 ok"));
    assert!(barometer.diagnose(&mut mock.clone()).starts_with("no bus response"));
    mock.done();
}
