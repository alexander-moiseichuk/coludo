//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The LSM6DSO32's unit tests: the burst decode and scaling, the write-then-verify bring-up over the scripted I2C
//! bus (the same register window plan B's i2c path uses), the SPI framing over a local scripted `SpiDevice`, and the
//! INT1 silent verdict. Plan B's own test (test_lsm6dso32.py) covers registration, absence and a live part on the
//! board; its positive scaling is in the golden fixture drivers_lsm6dso32.json.

use std::collections::VecDeque;

use embedded_hal::spi::{self, ErrorType, Operation, SpiDevice};
use serde_json::json;

use super::{
    CONFIGURATION, IDENTITY, IDENTITY_ROUNDS, INTERRUPT_SILENT_LIMIT, Lsm6dso32, OUTX_L_G, OverI2c, OverSpi,
    PIN_INTERFACE, ROUTE_INTERRUPT, Sample, Transport, WHO_AM_I,
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
