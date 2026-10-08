//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The LSM6DSO32's unit tests: the burst decode and scaling, the write-then-verify bring-up over the scripted I2C
//! bus (the same register window plan B's i2c path uses), the SPI framing over a local scripted `SpiDevice`, the
//! INT1 silent verdict, the status burst's new-conversion flags (an impossible status, as a dead SPI part's 0xFF,
//! never fresh) and the part's own ODR. Plan B's own test (test_lsm6dso32.py) covers registration, absence and a
//! live part on the board; its positive scaling is in the golden fixture drivers_lsm6dso32.json, which
//! golden_lsm6dso32.rs also replays through the status burst.

use std::collections::VecDeque;

use embedded_hal::spi::{self, ErrorType, Operation, SpiDevice};
use serde_json::json;

use super::{
    CONFIGURATION, FRAME_LENGTH, Fresh, IDENTITY, IDENTITY_ROUNDS, INTERNAL_FREQ_FINE, INTERRUPT_SILENT_LIMIT,
    Lsm6dso32, OUTX_L_G, OverI2c, OverSpi, PIN_INTERFACE, ROUTE_INTERRUPT, STATUS_FRAME_LENGTH, STATUS_HEADER_LENGTH,
    STATUS_REG, Sample, Transport, WHO_AM_I, odr_hz,
};
use crate::device::{Device, Fault};
use crate::mock::MockBus;

/// The part's I2C address in these tests.
const ADDRESS: u8 = 0x6A;

/// A driver on the I2C window, INT1 wired or not.
fn driver(interrupt_wired: bool) -> Lsm6dso32<OverI2c> {
    Lsm6dso32::new(OverI2c { address: ADDRESS }, interrupt_wired)
}

/// Script one write-then-verify round answered with `identity`.
fn script_round(mock: &MockBus, identity: u8) {
    mock.expect_write(ADDRESS, &[PIN_INTERFACE.0, PIN_INTERFACE.1])
        .expect_write_read(ADDRESS, &[WHO_AM_I], &[identity]);
}

/// Script the configuration writes.
fn script_configuration(mock: &MockBus) {
    for (register, value) in CONFIGURATION {
        mock.expect_write(ADDRESS, &[register, value]);
    }
}

#[test]
fn the_burst_is_gyro_first_then_accel_little_endian() {
    let frame = [
        0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80, // gyro 1, -1, -32768
        0x00, 0x04, 0xFF, 0x7F, 0x34, 0x12, // accel 1024, 32767, 0x1234
    ];
    let sample = Sample::decode(&frame);
    assert_eq!(sample.gyro, [1, -1, -32768]);
    assert_eq!(sample.accel, [1024, 32767, 0x1234]);
}

#[test]
fn the_scales_are_the_datasheets() {
    let sample = Sample {
        gyro: [1, -1, 32767],
        accel: [1024, -1024, 0],
    };
    let accel = sample.accel_g();
    assert!((accel[0] - 0.999_424).abs() < 1e-6, "{accel:?}");
    assert_eq!(accel[1], -accel[0], "the accel keeps its sign");
    assert_eq!(accel[2], 0.0);
    // 70 mdps a count: the telemetry integer is exactly raw x 7 centidegrees per second, both signs
    assert_eq!(sample.rate_centidegrees_per_second(), [7, -7, 229_369]);
    let rate = sample.rate_degrees_per_second();
    assert!(
        (rate[0] - 0.07).abs() < 1e-7 && (rate[1] + 0.07).abs() < 1e-7,
        "{rate:?}"
    );
    assert!((rate[2] - 2293.69).abs() < 1e-3, "{rate:?}");
}

#[test]
fn a_present_part_is_pinned_verified_and_configured() {
    let mock = MockBus::new();
    script_round(&mock, IDENTITY);
    script_configuration(&mock);
    let mut imu = driver(false);
    imu.setup(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
}

#[test]
fn a_part_out_of_step_is_recovered_by_the_write_not_the_read() {
    // TMS-7E: WHO_AM_I read 0x00 for 30 reads; the CTRL3_C write before each read is what brings it back
    let mock = MockBus::new();
    for _ in 0..30 {
        script_round(&mock, 0x00);
    }
    script_round(&mock, IDENTITY);
    script_configuration(&mock);
    driver(false).setup(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
}

#[test]
fn an_absent_part_is_refused_after_every_round() {
    // NEGATIVE: a part that never answers 0x6C is not an LSM6DSO32, and nothing is configured
    let mock = MockBus::new();
    for _ in 0..IDENTITY_ROUNDS {
        script_round(&mock, 0x00);
    }
    let fault = driver(false).setup(&mut mock.clone(), &mut mock.delay()).unwrap_err();
    assert_eq!(
        fault,
        Fault::Identity {
            expected: 0x6C,
            read: 0x00
        }
    );
    mock.done();
}

#[test]
fn a_bus_failure_ends_the_bring_up_at_once() {
    // NEGATIVE: plan B's single try around the loop -- a NAK is not retried like a wrong identity
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &[PIN_INTERFACE.0, PIN_INTERFACE.1])
        .then_fail();
    let fault = driver(false).setup(&mut mock.clone(), &mut mock.delay()).unwrap_err();
    assert!(matches!(fault, Fault::Bus(_)), "{fault:?}");
    mock.done();
}

#[test]
fn a_wired_int1_is_routed_then_cleared() {
    let mock = MockBus::new();
    script_round(&mock, IDENTITY);
    script_configuration(&mock);
    mock.expect_write(ADDRESS, &[ROUTE_INTERRUPT.0, ROUTE_INTERRUPT.1])
        .expect_write_read(ADDRESS, &[OUTX_L_G], &[0; 12]);
    driver(true).setup(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
}

#[test]
fn the_rearm_is_the_whole_bring_up() {
    // a reset part comes back powered down, so the re-arm pins, verifies and configures again
    let mock = MockBus::new();
    script_round(&mock, 0x00);
    script_round(&mock, IDENTITY);
    script_configuration(&mock);
    mock.expect_write(ADDRESS, &[ROUTE_INTERRUPT.0, ROUTE_INTERRUPT.1])
        .expect_write_read(ADDRESS, &[OUTX_L_G], &[0; 12]);
    driver(true).rearm(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
}

#[test]
fn the_probe_wants_the_identity_then_a_sample() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[WHO_AM_I], &[IDENTITY])
        .expect_write_read(ADDRESS, &[OUTX_L_G], &[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x04]);
    let sample = driver(false).probe(&mut mock.clone()).unwrap();
    assert_eq!(sample.accel, [0, 0, 1024]);
    // NEGATIVE: another part's identity fails the probe before any sample is read
    mock.expect_write_read(ADDRESS, &[WHO_AM_I], &[0x6B]);
    let fault = driver(false).probe(&mut mock.clone()).unwrap_err();
    assert_eq!(
        fault,
        Fault::Identity {
            expected: 0x6C,
            read: 0x6B
        }
    );
    mock.done();
}

#[test]
fn the_diagnosis_names_the_wire_fault() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[WHO_AM_I], &[0x00])
        .expect_write_read(ADDRESS, &[WHO_AM_I], &[0xFF])
        .expect_write_read(ADDRESS, &[WHO_AM_I], &[0x00])
        .then_fail();
    let imu = driver(false);
    assert!(imu.diagnose(&mut mock.clone()).contains("chip-select not asserting"));
    assert!(imu.diagnose(&mut mock.clone()).contains("idle-high"));
    assert!(imu.diagnose(&mut mock.clone()).starts_with("no bus response"));
    mock.done();
}

#[test]
fn a_dead_int1_is_called_silent_once_and_an_edge_clears_it() {
    let mut imu = driver(true);
    let notes: Vec<bool> = (0..6).map(|_| imu.wake(0)).collect();
    // the note fires once, on the wake that reaches the limit, not on every timeout after it
    let mut expected = vec![false; 6];
    expected[INTERRUPT_SILENT_LIMIT as usize - 1] = true;
    assert_eq!(notes, expected);
    assert!(imu.interrupt_silent());
    // a late edge resumes interrupt mode, and the count starts over
    assert!(!imu.wake(1));
    assert!(!imu.interrupt_silent());
    assert!(!imu.wake(0) && !imu.wake(0));
    assert!(imu.wake(0), "a later dead spell is reported again");
    // NEGATIVE: an overrun (2 edges) is not silence
    assert!(!imu.wake(2) && !imu.interrupt_silent());
}

#[test]
fn an_unwired_part_is_counted_too_as_plan_b_counts_it() {
    // plan B's run() never reads the wire on this part (unlike the BMI323's): three timed-out wakes call an UNWIRED
    // LSM6DSO32 silent as well, the note fires once, and an edge clears it
    let mut imu = driver(false);
    assert!(!imu.interrupt_wired());
    let notes: Vec<bool> = (0..INTERRUPT_SILENT_LIMIT + 2).map(|_| imu.wake(0)).collect();
    assert_eq!(notes.iter().filter(|note| **note).count(), 1);
    assert!(notes[INTERRUPT_SILENT_LIMIT as usize - 1]);
    assert!(imu.interrupt_silent());
    assert!(!imu.wake(1));
    assert!(!imu.interrupt_silent());
}

#[test]
fn the_config_names_the_transport() {
    assert_eq!(Transport::from_config(&json!({})), Transport::Spi);
    assert_eq!(Transport::from_config(&json!({"bus": "spi", "id": 1})), Transport::Spi);
    assert_eq!(
        Transport::from_config(&json!({"bus": "i2c", "id": 0})),
        Transport::I2c { address: 0x6A }
    );
    assert_eq!(
        Transport::from_config(&json!({"bus": "i2c", "addr": 0x6B})),
        Transport::I2c { address: 0x6B }
    );
    // NEGATIVE: an address that is no 7-bit integer takes the default instead of a wrapped byte or an 8-bit one
    for address in [
        json!(0x16A),
        json!(0xEA),
        json!(0x80),
        json!(-1),
        json!("0x6B"),
        json!(null),
    ] {
        assert_eq!(
            Transport::from_config(&json!({"bus": "i2c", "addr": address})),
            Transport::I2c { address: 0x6A },
            "addr {address}"
        );
    }
    assert_eq!(
        Transport::from_config(&json!({"bus": "i2c", "addr": 0x7F})),
        Transport::I2c { address: 0x7F },
        "the top 7-bit address is still one"
    );
    // NEGATIVE: plan B's `config.get('bus', 'spi') == 'spi'` is false for a bus that is present but not the string
    // 'spi', so a null, a number or another case takes the I2C path, not the SPI default
    for bus in [json!(null), json!(1), json!("SPI"), json!(["spi"])] {
        assert_eq!(
            Transport::from_config(&json!({"bus": bus})),
            Transport::I2c { address: 0x6A },
            "bus {bus}"
        );
    }
}

/// One chip-select frame a scripted SPI device saw: the bytes written, and how many were read after them.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Frame {
    /// Every byte written in the frame.
    written: Vec<u8>,
    /// How many bytes the frame read.
    read: usize,
}

/// A scripted `SpiDevice`: it records each chip-select frame and answers reads from a queue.
#[derive(Debug, Default)]
struct ScriptedSpi {
    /// The frames seen, in order.
    frames: Vec<Frame>,
    /// The answers to the reads, in order.
    answers: VecDeque<Vec<u8>>,
}

impl ErrorType for ScriptedSpi {
    type Error = spi::ErrorKind;
}

impl SpiDevice for ScriptedSpi {
    /// Record one frame; a read takes the next scripted answer, which must be of its length.
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), spi::ErrorKind> {
        let mut frame = Frame {
            written: Vec::new(),
            read: 0,
        };
        for operation in operations {
            match operation {
                Operation::Write(bytes) => frame.written.extend_from_slice(bytes),
                Operation::Read(buffer) => {
                    let answer = self.answers.pop_front().expect("an unscripted SPI read");
                    buffer.copy_from_slice(&answer);
                    frame.read += buffer.len();
                }
                _ => panic!("an SPI operation the LSM6DSO32 never uses"),
            }
        }
        self.frames.push(frame);
        Ok(())
    }
}

#[test]
fn spi_reads_carry_the_read_flag_and_no_multi_byte_bit() {
    let mut bus = ScriptedSpi::default();
    bus.answers.push_back(vec![IDENTITY]);
    bus.answers.push_back(vec![0; 12]);
    let imu = Lsm6dso32::new(OverSpi, true);
    assert_eq!(imu.identity(&mut bus), Ok(IDENTITY));
    imu.read(&mut bus).unwrap();
    // 0x80 | 0x22 = 0xA2 for the 12-byte burst: bit 6 (the ADXL family's multi-byte bit) stays clear, because
    // this part auto-increments from CTRL3_C.IF_INC and 0xE2 would address another register
    assert_eq!(
        bus.frames,
        [
            Frame {
                written: vec![0x8F],
                read: 1
            },
            Frame {
                written: vec![0xA2],
                read: 12
            },
        ]
    );
}

#[test]
fn spi_writes_are_the_register_then_the_value_in_one_frame() {
    let mut bus = ScriptedSpi::default();
    bus.answers.push_back(vec![IDENTITY]);
    let mut imu = Lsm6dso32::new(OverSpi, false);
    let mock = MockBus::new();
    imu.setup(&mut bus, &mut mock.delay()).unwrap();
    let written: Vec<Vec<u8>> = bus.frames.iter().map(|frame| frame.written.clone()).collect();
    assert_eq!(
        written,
        [vec![0x12, 0x44], vec![0x8F], vec![0x10, 0x44], vec![0x11, 0x4C]]
    );
}

/// The 12 output bytes the status-burst tests read: gyro 1, -1, -32768, then accel 1024, 32767, 0x1234.
const OUTPUT: [u8; FRAME_LENGTH] = [
    0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80, // gyro
    0x00, 0x04, 0xFF, 0x7F, 0x34, 0x12, // accel
];

/// A status burst: `status`, then a reserved byte and a temperature that are not zero (so a decode that took them
/// for output would show), then [`OUTPUT`].
fn status_frame(status: u8) -> [u8; STATUS_FRAME_LENGTH] {
    let mut frame = [0u8; STATUS_FRAME_LENGTH];
    frame[..STATUS_HEADER_LENGTH].copy_from_slice(&[status, 0xA5, 0x5A, 0xC3]);
    frame[STATUS_HEADER_LENGTH..].copy_from_slice(&OUTPUT);
    frame
}

#[test]
fn the_status_burst_is_one_i2c_read_from_status_reg() {
    // the register 0x1E, a repeated start, then all 16 bytes: the mock refuses any other shape or length
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[STATUS_REG], &status_frame(0x03));
    let (fresh, sample) = driver(true).read_fresh(&mut mock.clone()).unwrap();
    mock.done();
    assert_eq!(
        fresh,
        Fresh {
            accel: true,
            rate: true,
            ..Fresh::default()
        }
    );
    // the output decodes exactly as the bare 12-byte burst does: the reserved byte and OUT_TEMP are skipped
    assert_eq!(sample, Sample::decode(&OUTPUT));
    assert_eq!(sample.gyro, [1, -1, -32768]);
    assert_eq!(sample.accel, [1024, 32767, 0x1234]);
}

#[test]
fn each_flag_speaks_for_its_own_half() {
    // XLDA (bit 0) is the accel's, GDA (bit 1) the gyro's; TDA (bit 2) belongs to neither
    let cases = [
        (0x03, true, true),
        (0x02, false, true),
        (0x01, true, false),
        (0x07, true, true),
        (0x06, false, true),
        (0x05, true, false),
        // NEGATIVE: no new conversion at all, the temperature alone
        (0x00, false, false),
        (0x04, false, false),
        // NEGATIVE: bits 7..3 read 0 on a live part, so a status with one set is a bus nothing drives (0xFF is an
        // SPI part that lost power, chip-select or MISO), never fresh, whatever XLDA and GDA say
        (0xFF, false, false),
        (0x0B, false, false),
        (0x83, false, false),
        (0xFC, false, false),
    ];
    for (status, accel, rate) in cases {
        let mock = MockBus::new();
        mock.expect_write_read(ADDRESS, &[STATUS_REG], &status_frame(status));
        let (fresh, sample) = driver(true).read_fresh(&mut mock.clone()).unwrap();
        mock.done();
        // a driver with nothing stored judges by the flags alone: nothing raced, nothing identical
        assert_eq!(
            fresh,
            Fresh {
                accel,
                rate,
                ..Fresh::default()
            },
            "status 0x{status:02x}"
        );
        // the burst still returns the output registers: the flags, not the bytes, say whether they are new
        assert_eq!(sample, Sample::decode(&OUTPUT), "status 0x{status:02x}");
    }
}

#[test]
fn every_bit_the_datasheet_fixes_at_zero_makes_the_status_impossible() {
    // NEGATIVE: each of bits 7..3 alone, over both flags set, turns the burst not fresh
    for bit in 3..8 {
        let status = 0x03 | (1u8 << bit);
        let mock = MockBus::new();
        mock.expect_write_read(ADDRESS, &[STATUS_REG], &status_frame(status));
        let (fresh, _) = driver(true).read_fresh(&mut mock.clone()).unwrap();
        mock.done();
        assert_eq!(fresh, Fresh::default(), "status 0x{status:02x}");
    }
}

#[test]
fn a_dead_spi_part_is_never_fresh() {
    // NEGATIVE: SPI has no ACK, so a part that stops driving MISO answers the burst with 16 x 0xFF and no bus
    // error (id_classify's "idle-high"), and a chip-select that never asserts with 16 x 0x00: neither is a new
    // conversion, so the firmware's stale run turns the part unhealthy instead of publishing -1 counts forever
    for byte in [0xFF, 0x00] {
        let mut bus = ScriptedSpi::default();
        bus.answers.push_back(vec![byte; STATUS_FRAME_LENGTH]);
        let (fresh, _) = Lsm6dso32::new(OverSpi, true).read_fresh(&mut bus).unwrap();
        assert_eq!(fresh, Fresh::default(), "16 x 0x{byte:02x}");
        assert_eq!(
            bus.frames,
            [Frame {
                written: vec![0x9E],
                read: STATUS_FRAME_LENGTH
            }]
        );
    }
}

#[test]
fn a_failed_status_burst_is_a_bus_fault() {
    // NEGATIVE: a NAK on the burst is the transport's fault, not a stale sample or a default one
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[STATUS_REG], &status_frame(0x03))
        .then_fail();
    let fault = driver(true).read_fresh(&mut mock.clone()).unwrap_err();
    assert!(matches!(fault, Fault::Bus(_)), "{fault:?}");
    mock.done();
}

#[test]
fn spi_reads_the_status_burst_in_one_frame() {
    let mut bus = ScriptedSpi::default();
    bus.answers.push_back(status_frame(0x02).to_vec());
    let mut imu = Lsm6dso32::new(OverSpi, true);
    let (fresh, sample) = imu.read_fresh(&mut bus).unwrap();
    assert_eq!(
        fresh,
        Fresh {
            rate: true,
            ..Fresh::default()
        }
    );
    assert_eq!(sample, Sample::decode(&OUTPUT));
    // 0x80 | 0x1E = 0x9E, then 16 bytes clocked in, under ONE chip-select: no multi-byte bit (0xDE would address
    // another register), and no second frame for the output, so the window between the flags and the output is a
    // few clocked bytes, not a second transfer
    assert_eq!(
        bus.frames,
        [Frame {
            written: vec![0x9E],
            read: 16
        }]
    );
}

#[test]
fn the_frequency_fine_byte_is_twos_complement() {
    let mock = MockBus::new();
    for (byte, fine) in [(0x00, 0), (0xDF, -33), (0x7F, 127), (0x80, -128), (0xFF, -1)] {
        mock.expect_write_read(ADDRESS, &[INTERNAL_FREQ_FINE], &[byte]);
        assert_eq!(driver(true).frequency_fine(&mut mock.clone()), Ok(fine), "0x{byte:02x}");
    }
    mock.done();
    // over SPI: 0x80 | 0x63 = 0xE3, one byte clocked in
    let mut bus = ScriptedSpi::default();
    bus.answers.push_back(vec![0xDF]);
    assert_eq!(Lsm6dso32::new(OverSpi, true).frequency_fine(&mut bus), Ok(-33));
    assert_eq!(
        bus.frames,
        [Frame {
            written: vec![0xE3],
            read: 1
        }]
    );
}

#[test]
fn a_refused_frequency_fine_is_a_bus_fault() {
    // NEGATIVE: a NAK is reported, never read as a 0 (which would claim the nominal 104.17 Hz)
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[INTERNAL_FREQ_FINE], &[0x00])
        .then_fail();
    let fault = driver(true).frequency_fine(&mut mock.clone()).unwrap_err();
    assert!(matches!(fault, Fault::Bus(_)), "{fault:?}");
    mock.done();
}

#[test]
fn the_odr_is_the_datasheets_formula() {
    // (6667 + 0.0015 x fine x 6667) / 64, computed in f64 for the reference
    let reference = |fine: i8| (6667.0 + 0.0015 * f64::from(fine) * 6667.0) / 64.0;
    assert_eq!(odr_hz(0), 104.171_875, "6667 / 64 is exact in f32");
    for fine in [0, -33, 127, -128, 1, -1] {
        let odr = f64::from(odr_hz(fine));
        assert!((odr - reference(fine)).abs() < 1e-4, "fine {fine}: {odr} Hz");
    }
    // the taster's case, about 99 Hz (slower than the 100 Hz tick), and the top of the range
    assert!((odr_hz(-33) - 99.015).abs() < 1e-3, "{}", odr_hz(-33));
    assert!((odr_hz(127) - 124.017).abs() < 1e-3, "{}", odr_hz(127));
    // NEGATIVE: the sign matters -- a negative trim is a SLOWER part, below the tick, a positive one faster
    assert!(odr_hz(-1) < odr_hz(0) && odr_hz(0) < odr_hz(1));
    assert!(odr_hz(-33) < 100.0 && odr_hz(33) > 100.0);
}

/// The 12 output bytes of a sample: gyro x, y, z then accel x, y, z, little-endian.
fn output(gyro: [i16; 3], accel: [i16; 3]) -> [u8; FRAME_LENGTH] {
    let mut bytes = [0u8; FRAME_LENGTH];
    for (index, word) in gyro.iter().chain(accel.iter()).enumerate() {
        bytes[2 * index..2 * index + 2].copy_from_slice(&word.to_le_bytes());
    }
    bytes
}

/// One sample's counts: (gyro x, y, z; accel x, y, z).
type Counts = ([i16; 3], [i16; 3]);

/// A part at rest: a small gyro bias, 1 g on z.
const REST: Counts = ([3, -2, 1], [5, -7, 1025]);

/// [`REST`] one accel conversion later (one count on x).
const REST_ACCEL_MOVED: Counts = ([3, -2, 1], [6, -7, 1025]);

/// [`REST`] one gyro conversion later (one count on z).
const REST_GYRO_MOVED: Counts = ([3, -2, 2], [5, -7, 1025]);

/// [`REST`] with both halves one conversion later.
const REST_BOTH_MOVED: Counts = ([3, -2, 2], [6, -7, 1025]);

/// Read status bursts in turn on one driver, each `(status, (gyro, accel))`, and return what each one judged.
fn read_in_turn(imu: &mut Lsm6dso32<OverI2c>, bursts: &[(u8, Counts)]) -> Vec<Fresh> {
    let mock = MockBus::new();
    for (status, (gyro, accel)) in bursts {
        let mut frame = status_frame(*status);
        frame[STATUS_HEADER_LENGTH..].copy_from_slice(&output(*gyro, *accel));
        mock.expect_write_read(ADDRESS, &[STATUS_REG], &frame);
    }
    let verdicts = bursts
        .iter()
        .map(|_| imu.read_fresh(&mut mock.clone()).expect("a scripted burst").0)
        .collect();
    mock.done();
    verdicts
}

/// Both halves new by their flags, nothing corrected.
const BOTH: Fresh = Fresh {
    accel: true,
    rate: true,
    accel_raced: false,
    rate_raced: false,
    accel_identical: false,
    rate_identical: false,
};

#[test]
fn a_flag_alone_is_fresh() {
    // the first burst has nothing to compare with; then each flag pushes its own half, even over identical bytes
    let mut imu = driver(true);
    let verdicts = read_in_turn(
        &mut imu,
        &[(0x03, REST), (0x01, REST_ACCEL_MOVED), (0x02, REST_ACCEL_MOVED)],
    );
    assert_eq!(verdicts[0], BOTH);
    assert_eq!(
        verdicts[1],
        Fresh {
            accel: true,
            ..Fresh::default()
        }
    );
    // the gyro converted to the very same bytes: a flagged half is never stale for identical bytes, only counted
    assert_eq!(
        verdicts[2],
        Fresh {
            rate: true,
            rate_identical: true,
            ..Fresh::default()
        }
    );
}

#[test]
fn bytes_changed_under_a_clear_flag_are_a_raced_conversion_pushed_once() {
    let mut imu = driver(true);
    let verdicts = read_in_turn(
        &mut imu,
        &[
            (0x03, REST),
            (0x00, REST_ACCEL_MOVED),
            (0x01, REST_ACCEL_MOVED),
            (0x00, REST_ACCEL_MOVED),
        ],
    );
    // the conversion landed after the status byte: its bytes are in the burst, its flag is not
    assert_eq!(
        verdicts[1],
        Fresh {
            accel: true,
            accel_raced: true,
            ..Fresh::default()
        }
    );
    // its flag, read on the next burst over the same bytes, is the duplicate: not pushed a second time
    assert_eq!(
        verdicts[2],
        Fresh {
            accel_identical: true,
            ..Fresh::default()
        }
    );
    assert_eq!(verdicts[3], Fresh::default());
    // the gyro's race, the same way
    let mut imu = driver(true);
    let verdicts = read_in_turn(
        &mut imu,
        &[(0x03, REST), (0x00, REST_GYRO_MOVED), (0x02, REST_GYRO_MOVED)],
    );
    assert!(verdicts[1].rate && verdicts[1].rate_raced && !verdicts[1].accel);
    assert!(!verdicts[2].rate && verdicts[2].rate_identical);
}

#[test]
fn a_flag_and_changed_bytes_are_fresh_and_corrected_by_nothing() {
    let mut imu = driver(true);
    let verdicts = read_in_turn(&mut imu, &[(0x03, REST), (0x03, REST_BOTH_MOVED)]);
    assert_eq!(verdicts, [BOTH, BOTH]);
    // a raced push followed by a flag over NEW bytes: the next conversion, pushed
    let mut imu = driver(true);
    let verdicts = read_in_turn(&mut imu, &[(0x03, REST), (0x00, REST_ACCEL_MOVED), (0x01, REST)]);
    assert!(verdicts[1].accel_raced);
    assert!(verdicts[2].accel && !verdicts[2].accel_identical);
}

#[test]
fn neither_a_flag_nor_a_change_is_not_fresh() {
    // NEGATIVE: a part slower than the tick, read between two conversions
    let mut imu = driver(true);
    let verdicts = read_in_turn(&mut imu, &[(0x03, REST), (0x00, REST), (0x04, REST)]);
    assert_eq!(verdicts[1..], [Fresh::default(), Fresh::default()]);
}

#[test]
fn a_flag_over_identical_bytes_after_a_raced_push_is_skipped() {
    // NEGATIVE: both halves raced, then both flags arrive late over the same bytes: nothing is pushed twice
    let mut imu = driver(true);
    let verdicts = read_in_turn(
        &mut imu,
        &[(0x03, REST), (0x00, REST_BOTH_MOVED), (0x03, REST_BOTH_MOVED)],
    );
    assert!(verdicts[1].accel_raced && verdicts[1].rate_raced);
    assert_eq!(
        verdicts[2],
        Fresh {
            accel_identical: true,
            rate_identical: true,
            ..Fresh::default()
        }
    );
}

#[test]
fn a_flag_over_identical_bytes_after_a_flagged_push_is_pushed() {
    // NEGATIVE of the correction: a conversion that repeats the last one bit for bit (common at rest) is new
    let mut imu = driver(true);
    let verdicts = read_in_turn(&mut imu, &[(0x03, REST), (0x03, REST), (0x03, REST)]);
    let identical = Fresh {
        accel_identical: true,
        rate_identical: true,
        ..BOTH
    };
    assert_eq!(verdicts, [BOTH, identical, identical]);
}

#[test]
fn a_reserved_status_bit_is_never_fresh_and_never_stored() {
    // NEGATIVE: bytes changed AND flags set, under an impossible status
    let mut imu = driver(true);
    let verdicts = read_in_turn(&mut imu, &[(0x03, REST), (0x83, REST_BOTH_MOVED), (0x00, REST)]);
    assert_eq!(verdicts[1], Fresh::default());
    // the impossible frame's bytes were not stored: the part's own bytes again are no change, so no raced push
    assert_eq!(verdicts[2], Fresh::default());
}

#[test]
fn all_twelve_output_bytes_zero_are_never_fresh() {
    // NEGATIVE: a chip-select that never asserts, with flags that happen to read set, and with them clear
    let mut imu = driver(true);
    let zero: Counts = ([0; 3], [0; 3]);
    let verdicts = read_in_turn(&mut imu, &[(0x03, REST), (0x03, zero), (0x00, zero), (0x00, REST)]);
    assert_eq!(verdicts[1..], [Fresh::default(), Fresh::default(), Fresh::default()]);
}

#[test]
fn a_zero_gyro_half_alone_is_fresh() {
    // a well-biased gyro at rest reads (0, 0, 0): the zero guard is the whole frame, never a half
    let mut imu = driver(true);
    let still: Counts = ([0; 3], [5, -7, 1025]);
    let verdicts = read_in_turn(&mut imu, &[(0x03, still), (0x02, still)]);
    assert_eq!(verdicts[0], BOTH);
    assert!(verdicts[1].rate && verdicts[1].rate_identical && !verdicts[1].accel);
}

#[test]
fn after_forget_a_burst_is_judged_by_its_flag() {
    let mut imu = driver(true);
    read_in_turn(&mut imu, &[(0x03, REST)]);
    imu.forget();
    // NEGATIVE: changed bytes under a clear flag are no raced conversion with nothing stored
    assert_eq!(read_in_turn(&mut imu, &[(0x00, REST_BOTH_MOVED)]), [Fresh::default()]);
    // the burst after it is compared again: a flag over its bytes is pushed and counted identical
    let verdicts = read_in_turn(&mut imu, &[(0x01, REST_BOTH_MOVED)]);
    assert!(verdicts[0].accel && verdicts[0].accel_identical && !verdicts[0].rate);
    // a raced mark is forgotten too: the flag read late after a forget is pushed, never skipped
    read_in_turn(&mut imu, &[(0x00, REST)]);
    imu.forget();
    assert_eq!(read_in_turn(&mut imu, &[(0x03, REST)]), [BOTH]);
}

#[test]
fn the_rearm_forgets_the_stored_bytes() {
    let mut imu = driver(true);
    read_in_turn(&mut imu, &[(0x03, REST)]);
    let mock = MockBus::new();
    script_round(&mock, IDENTITY);
    script_configuration(&mock);
    mock.expect_write(ADDRESS, &[ROUTE_INTERRUPT.0, ROUTE_INTERRUPT.1])
        .expect_write_read(ADDRESS, &[OUTX_L_G], &output(REST_BOTH_MOVED.0, REST_BOTH_MOVED.1));
    imu.rearm(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    // the re-armed part's first burst, flag clear over bytes that changed: judged by the flag alone
    assert_eq!(read_in_turn(&mut imu, &[(0x00, REST_BOTH_MOVED)]), [Fresh::default()]);
}

#[test]
fn a_bare_read_leaves_the_judge_alone() {
    // the probe's 12-byte read takes a conversion from under the flags; the next burst finds its bytes changed
    // under a clear flag and pushes it, raced: it never was pushed
    let mut imu = driver(true);
    read_in_turn(&mut imu, &[(0x03, REST)]);
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[OUTX_L_G], &output(REST_ACCEL_MOVED.0, REST_ACCEL_MOVED.1));
    imu.read(&mut mock.clone()).unwrap();
    mock.done();
    let verdicts = read_in_turn(&mut imu, &[(0x00, REST_ACCEL_MOVED)]);
    assert!(verdicts[0].accel && verdicts[0].accel_raced);
}
