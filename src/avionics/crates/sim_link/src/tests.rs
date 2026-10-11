//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The frames against doc/specs/sim-link.md: the pinned bytes are Python's (`struct.pack('<...')` and
//! `binascii.crc32`, as tools/hitl_host.py packs them), so the board and the host agree byte for byte.

use super::{Fins, Refusal, Sensors, newer};

/// A SENSORS frame with every field but the laser's.
fn full() -> Sensors {
    Sensors {
        seq: 7,
        sim_time_ms: 12345,
        accel: Some([0.0, 0.0, 1.0]),
        rate: Some([1.5, -2.25, 0.5]),
        attitude: Some([123.5, -10.0, 5.25]),
        mag: Some([1000.0, -200.0, 700.0]),
        altitude: Some(12.5),
        elevation: Some(2.5),
        agl: None,
        airspeed: Some(12.0),
        dynamic_pressure: Some(88.2),
        position: Some([455_000_000, -735_000_000]),
        speed: Some(11.5),
        course: Some(270.0),
    }
}

/// [`full`] as Python packs it.
const FULL: &str = "43534c310101bf0f070000003930000000000000000000000000803f0000c03f000010c00000003f0000f742000020c10000a84000007a\
                    44000048c300002f44000048410000204000000000000040416666b042c0bf1e1b40ca30d400003841000087433a68f729";

/// A FINS frame with its fins.
fn answer() -> Fins {
    Fins {
        seq: 7,
        uptime_ms: 4242,
        stage: 3,
        armed: true,
        launched: true,
        fins: Some([95, 85, 90]),
    }
}

/// [`answer`] as Python packs it.
const ANSWER: &str = "43534c31010201000700000092100000030101005f0055005a000000cdd3a798";

/// The bytes of a hex string.
fn bytes(hex: &str) -> Vec<u8> {
    (0..hex.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).expect("hex"))
        .collect()
}

#[test]
fn the_frames_are_the_hosts_bytes() {
    assert_eq!(full().encode().to_vec(), bytes(FULL));
    assert_eq!(Sensors::decode(&bytes(FULL)), Ok(full()));
    assert_eq!(answer().encode().to_vec(), bytes(ANSWER));
    assert_eq!(Fins::decode(&bytes(ANSWER)), Ok(answer()));
}

#[test]
fn an_absent_field_is_zero_and_decodes_absent() {
    let empty = Sensors {
        seq: 1,
        ..Sensors::default()
    };
    let frame = empty.encode();
    assert_eq!(&frame[6..8], &[0, 0], "no flags");
    assert!(frame[16..100].iter().all(|byte| *byte == 0));
    assert_eq!(Sensors::decode(&frame), Ok(empty));
    let passive = Fins { fins: None, ..answer() };
    let frame = passive.encode();
    assert_eq!(&frame[20..26], &[0; 6]);
    assert_eq!(Fins::decode(&frame), Ok(passive));
}

#[test]
fn a_bad_frame_is_refused_for_its_reason() {
    let good = full().encode();
    assert_eq!(Sensors::decode(&good[..103]), Err(Refusal::Length));
    assert_eq!(Sensors::decode(&answer().encode()), Err(Refusal::Length));
    let altered = |offset: usize, value: u8| {
        let mut frame = good;
        frame[offset] = value;
        frame
    };
    assert_eq!(Sensors::decode(&altered(0, b'X')), Err(Refusal::Magic));
    assert_eq!(Sensors::decode(&altered(4, 2)), Err(Refusal::Version));
    assert_eq!(Sensors::decode(&altered(5, 2)), Err(Refusal::Kind));
    assert_eq!(Sensors::decode(&altered(70, 0x41)), Err(Refusal::Crc));
    let mut fins = answer().encode();
    fins[5] = 1;
    assert_eq!(Fins::decode(&fins), Err(Refusal::Kind));
}

#[test]
fn a_non_finite_present_field_is_refused_and_an_absent_one_ignored() {
    let poisoned = Sensors {
        elevation: Some(f32::NAN),
        ..full()
    };
    assert_eq!(Sensors::decode(&poisoned.encode()), Err(Refusal::NotFinite));
    let infinite = Sensors {
        rate: Some([0.0, f32::INFINITY, 0.0]),
        ..full()
    };
    assert_eq!(Sensors::decode(&infinite.encode()), Err(Refusal::NotFinite));
    // a NaN in an absent slot (a host that leaves stale bytes) is never read: re-seal such a frame by hand
    let mut frame = Sensors { agl: None, ..full() }.encode();
    frame[72..76].copy_from_slice(&f32::NAN.to_le_bytes());
    let crc = recorder_wire::crc32(0, &frame[..100]);
    frame[100..].copy_from_slice(&crc.to_le_bytes());
    assert_eq!(Sensors::decode(&frame).map(|sensors| sensors.agl), Ok(None));
}

#[test]
fn newer_follows_the_wrap() {
    assert!(newer(8, 7));
    assert!(!newer(7, 7), "a repeat is not newer");
    assert!(!newer(6, 7), "a late frame is not newer");
    assert!(newer(0, u32::MAX), "across the wrap");
    assert!(newer(3, u32::MAX - 2));
    assert!(!newer(u32::MAX, 0));
    assert!(newer(7 + (1 << 31) - 1, 7));
    assert!(!newer(7 + (1 << 31), 7), "half the circle away is behind");
}
