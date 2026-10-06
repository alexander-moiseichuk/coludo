//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The scripted bus the host tests drive the drivers with: an embedded-hal `I2c` ([`MockBus`]), a `DelayNs`
//! ([`MockDelay`]) and an `OutputPin` ([`MockPin`]) that all check their calls against ONE script, in order.
//!
//! One script for the three is the point: a driver's correctness is often an ORDER -- the SDP810 must be stopped,
//! left 3 ms, and only then started; the INA226's three registers are three separate reads; the VL53's XSHUT goes
//! low, waits, goes high, waits, then polls -- and only a shared timeline can say the wait came between the two
//! writes and not after them. A call that does not match the next scripted step panics at once, naming the step,
//! both sides and the bytes in hex; [`MockBus::done`] then insists every scripted step happened.
//!
//! Plan B has no such mock (its tests run on the board against real parts); the golden builders capture the same
//! kind of transcript from plan B's own code, and the crate's golden tests replay it through this bus.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::convert::Infallible;
use std::rc::Rc;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{self, OutputPin};
use embedded_hal::i2c::{self, ErrorKind, I2c, NoAcknowledgeSource, Operation};

/// One step of a script: what the driver must do next, and what the part answers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Step {
    /// A plain write: the register pointer (if any) and the data, in one transaction.
    Write {
        /// The 7-bit address.
        address: u8,
        /// Every byte written.
        bytes: Vec<u8>,
    },
    /// A plain read; `bytes` is what the part answers, and the driver must ask for exactly that many.
    Read {
        /// The 7-bit address.
        address: u8,
        /// The part's answer.
        bytes: Vec<u8>,
    },
    /// A register read: a write (the register address), a repeated start, then a read of `read.len()` bytes.
    WriteRead {
        /// The 7-bit address.
        address: u8,
        /// The bytes written before the repeated start (the register address).
        write: Vec<u8>,
        /// The part's answer.
        read: Vec<u8>,
    },
    /// A wait, in nanoseconds.
    Delay {
        /// How long the driver waits.
        nanoseconds: u64,
    },
    /// A control pin driven high (true) or low (false).
    Pin {
        /// The level driven.
        high: bool,
    },
}

/// One scripted step and whether the bus (or the pin) fails it.
#[derive(Clone, Debug)]
struct Expectation {
    /// What the driver must do.
    step: Step,
    /// True to answer this step with an error (a NAK on the bus, a refusal on the pin) after checking it matches.
    fails: bool,
}

/// The script the bus, the delay and the pin share.
#[derive(Debug, Default)]
struct Script {
    /// The steps still to happen, in order.
    pending: VecDeque<Expectation>,
    /// How many steps have happened: the index the panic messages name.
    performed: usize,
}

impl Script {
    /// The next step, checked against what the driver did; panics on a mismatch or an exhausted script.
    fn next(&mut self, actual: &Step) -> Expectation {
        let index = self.performed;
        let Some(expected) = self.pending.pop_front() else {
            panic!(
                "mock: step {index} {} was not scripted (the script is exhausted)",
                describe(actual)
            );
        };
        if !matches_step(&expected.step, actual) {
            panic!(
                "mock: step {index} expected {}, got {}",
                describe(&expected.step),
                describe(actual)
            );
        }
        self.performed += 1;
        expected
    }
}

/// Whether the driver's call matches the scripted step: everything compares, except that a read compares only its
/// length (the script supplies the bytes).
fn matches_step(expected: &Step, actual: &Step) -> bool {
    match (expected, actual) {
        (
            Step::Read { address, bytes },
            Step::Read {
                address: asked_address,
                bytes: asked,
            },
        ) => address == asked_address && bytes.len() == asked.len(),
        (
            Step::WriteRead { address, write, read },
            Step::WriteRead {
                address: asked_address,
                write: asked_write,
                read: asked_read,
            },
        ) => address == asked_address && write == asked_write && read.len() == asked_read.len(),
        _ => expected == actual,
    }
}

/// A step as the panic messages print it, bytes in hex.
fn describe(step: &Step) -> String {
    let hex = |bytes: &[u8]| bytes.iter().map(|byte| format!("{byte:02x}")).collect::<String>();
    match step {
        Step::Write { address, bytes } => format!("write 0x{address:02x} [{}]", hex(bytes)),
        Step::Read { address, bytes } => format!("read 0x{address:02x} {} bytes", bytes.len()),
        Step::WriteRead { address, write, read } => {
            format!("write_read 0x{address:02x} [{}] then {} bytes", hex(write), read.len())
        }
        Step::Delay { nanoseconds } => format!("delay {nanoseconds} ns"),
        Step::Pin { high } => format!("pin {}", if *high { "high" } else { "low" }),
    }
}

/// The scripted I2C bus. Clone it to share the script; [`MockBus::delay`] and [`MockBus::pin`] hand out the delay
/// and the pin on the same script.
#[derive(Clone, Debug, Default)]
pub struct MockBus {
    /// The shared script.
    script: Rc<RefCell<Script>>,
}

/// The error the scripted bus answers a failing step with: a NAK on the address, plan B's `OSError(19)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MockError;

impl i2c::Error for MockError {
    /// Always a NAK: what an absent or reset part answers.
    fn kind(&self) -> ErrorKind {
        ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
    }
}

impl MockBus {
    /// An empty script.
    pub fn new() -> MockBus {
        MockBus::default()
    }

    /// Append a step to the script.
    fn push(&self, step: Step) -> &MockBus {
        self.script
            .borrow_mut()
            .pending
            .push_back(Expectation { step, fails: false });
        self
    }

    /// Script a plain write of `bytes` to `address`.
    pub fn expect_write(&self, address: u8, bytes: &[u8]) -> &MockBus {
        self.push(Step::Write {
            address,
            bytes: bytes.to_vec(),
        })
    }

    /// Script a plain read from `address`, answered with `bytes`.
    pub fn expect_read(&self, address: u8, bytes: &[u8]) -> &MockBus {
        self.push(Step::Read {
            address,
            bytes: bytes.to_vec(),
        })
    }

    /// Script a register read: `write` (the register address) then a read answered with `read`.
    pub fn expect_write_read(&self, address: u8, write: &[u8], read: &[u8]) -> &MockBus {
        self.push(Step::WriteRead {
            address,
            write: write.to_vec(),
            read: read.to_vec(),
        })
    }

    /// Script a wait of `milliseconds`.
    pub fn expect_delay_ms(&self, milliseconds: u32) -> &MockBus {
        self.push(Step::Delay {
            nanoseconds: u64::from(milliseconds) * 1_000_000,
        })
    }

    /// Script a control pin driven `high` (true) or low.
    pub fn expect_pin(&self, high: bool) -> &MockBus {
        self.push(Step::Pin { high })
    }

    /// Script any step, as a golden transcript lists them.
    pub fn expect(&self, step: Step) -> &MockBus {
        self.push(step)
    }

    /// Make the last scripted step fail: the bus NAKs it (nothing is read back), the pin refuses it.
    ///
    /// Panics when nothing is scripted yet.
    pub fn then_fail(&self) -> &MockBus {
        self.script
            .borrow_mut()
            .pending
            .back_mut()
            .expect("mock: then_fail needs a scripted step before it")
            .fails = true;
        self
    }

    /// The delay on this bus's script.
    pub fn delay(&self) -> MockDelay {
        MockDelay {
            script: Rc::clone(&self.script),
        }
    }

    /// A control pin on this bus's script.
    pub fn pin(&self) -> MockPin {
        MockPin {
            script: Rc::clone(&self.script),
        }
    }

    /// How many scripted steps have happened so far.
    pub fn performed(&self) -> usize {
        self.script.borrow().performed
    }

    /// Insist that every scripted step happened; panics listing the ones that did not.
    pub fn done(&self) {
        let script = self.script.borrow();
        if !script.pending.is_empty() {
            let left: Vec<String> = script
                .pending
                .iter()
                .map(|expectation| describe(&expectation.step))
                .collect();
            panic!(
                "mock: {} scripted steps never happened after step {}: {}",
                left.len(),
                script.performed,
                left.join(", ")
            );
        }
    }
}

impl i2c::ErrorType for MockBus {
    type Error = MockError;
}

impl I2c for MockBus {
    /// Match one transaction against the next scripted step: a lone write, a lone read, or a write then a read
    /// (the register read). Any other shape panics: no driver here needs one.
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), MockError> {
        let actual = match operations {
            [Operation::Write(bytes)] => Step::Write {
                address,
                bytes: bytes.to_vec(),
            },
            [Operation::Read(buffer)] => Step::Read {
                address,
                bytes: vec![0; buffer.len()],
            },
            [Operation::Write(write), Operation::Read(buffer)] => Step::WriteRead {
                address,
                write: write.to_vec(),
                read: vec![0; buffer.len()],
            },
            _ => panic!(
                "mock: an I2C transaction shape no driver uses ({} operations)",
                operations.len()
            ),
        };
        let expected = self.script.borrow_mut().next(&actual);
        if expected.fails {
            return Err(MockError);
        }
        match (operations, expected.step) {
            ([Operation::Read(buffer)], Step::Read { bytes, .. })
            | ([Operation::Write(_), Operation::Read(buffer)], Step::WriteRead { read: bytes, .. }) => {
                buffer.copy_from_slice(&bytes);
            }
            _ => {}
        }
        Ok(())
    }
}

/// The scripted delay: each wait must be the next scripted step, to the nanosecond.
#[derive(Debug)]
pub struct MockDelay {
    /// The shared script.
    script: Rc<RefCell<Script>>,
}

impl MockDelay {
    /// Check one wait against the script.
    fn wait(&mut self, nanoseconds: u64) {
        self.script.borrow_mut().next(&Step::Delay { nanoseconds });
    }
}

impl DelayNs for MockDelay {
    /// A wait in nanoseconds.
    fn delay_ns(&mut self, nanoseconds: u32) {
        self.wait(u64::from(nanoseconds));
    }

    /// A wait in microseconds, checked as one step (the default would split it into nanosecond chunks).
    fn delay_us(&mut self, microseconds: u32) {
        self.wait(u64::from(microseconds) * 1_000);
    }

    /// A wait in milliseconds, checked as one step.
    fn delay_ms(&mut self, milliseconds: u32) {
        self.wait(u64::from(milliseconds) * 1_000_000);
    }
}

/// The scripted control pin (an XSHUT line): each level driven must be the next scripted step.
#[derive(Debug)]
pub struct MockPin {
    /// The shared script.
    script: Rc<RefCell<Script>>,
}

/// The error the scripted pin answers a failing step with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MockPinError;

impl digital::Error for MockPinError {
    /// A pin has no finer error to report.
    fn kind(&self) -> digital::ErrorKind {
        digital::ErrorKind::Other
    }
}

impl MockPin {
    /// Check one level against the script.
    fn drive(&mut self, high: bool) -> Result<(), MockPinError> {
        if self.script.borrow_mut().next(&Step::Pin { high }).fails {
            Err(MockPinError)
        } else {
            Ok(())
        }
    }
}

impl digital::ErrorType for MockPin {
    type Error = MockPinError;
}

impl OutputPin for MockPin {
    /// Drive the line low.
    fn set_low(&mut self) -> Result<(), MockPinError> {
        self.drive(false)
    }

    /// Drive the line high.
    fn set_high(&mut self) -> Result<(), MockPinError> {
        self.drive(true)
    }
}

/// The pin a part without a control line is given: there is nothing to drive, so nothing can fail.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoPin;

impl digital::ErrorType for NoPin {
    type Error = Infallible;
}

impl OutputPin for NoPin {
    /// Nothing to drive.
    fn set_low(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    /// Nothing to drive.
    fn set_high(&mut self) -> Result<(), Infallible> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::MockBus;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::digital::OutputPin;
    use embedded_hal::i2c::I2c;

    #[test]
    fn a_script_played_in_order_passes() {
        let mock = MockBus::new();
        mock.expect_write(0x25, &[0x3f, 0xf9])
            .expect_delay_ms(3)
            .expect_write_read(0x40, &[0xff], &[0x22, 0x60])
            .expect_read(0x25, &[1, 2, 3])
            .expect_pin(false);
        let mut bus = mock.clone();
        let mut delay = mock.delay();
        let mut pin = mock.pin();
        bus.write(0x25, &[0x3f, 0xf9]).unwrap();
        delay.delay_ms(3);
        let mut identity = [0u8; 2];
        bus.write_read(0x40, &[0xff], &mut identity).unwrap();
        assert_eq!(identity, [0x22, 0x60]);
        let mut frame = [0u8; 3];
        bus.read(0x25, &mut frame).unwrap();
        assert_eq!(frame, [1, 2, 3]);
        pin.set_low().unwrap();
        mock.done();
        assert_eq!(mock.performed(), 5);
    }

    #[test]
    fn a_failing_step_answers_an_error_and_reads_nothing() {
        let mock = MockBus::new();
        mock.expect_read(0x25, &[9, 9]).then_fail();
        let mut bus = mock.clone();
        let mut frame = [0u8; 2];
        assert!(bus.read(0x25, &mut frame).is_err());
        assert_eq!(frame, [0, 0]);
        mock.done();
    }

    #[test]
    #[should_panic(expected = "mock: step 1 expected delay 3000000 ns, got write 0x25 [3615]")]
    fn a_missing_wait_is_caught() {
        // NEGATIVE: start straight after stop, the very order the SDP810 refuses
        let mock = MockBus::new();
        mock.expect_write(0x25, &[0x3f, 0xf9])
            .expect_delay_ms(3)
            .expect_write(0x25, &[0x36, 0x15]);
        let mut bus = mock.clone();
        bus.write(0x25, &[0x3f, 0xf9]).unwrap();
        bus.write(0x25, &[0x36, 0x15]).unwrap();
    }

    #[test]
    #[should_panic(expected = "expected write_read 0x40 [02] then 2 bytes, got write_read 0x40 [02] then 6 bytes")]
    fn a_read_of_the_wrong_length_is_caught() {
        // NEGATIVE: the INA226's one-6-byte-read "optimisation" plan B measured as returning padding
        let mock = MockBus::new();
        mock.expect_write_read(0x40, &[0x02], &[0x0f, 0x95]);
        let mut bus = mock.clone();
        let mut registers = [0u8; 6];
        let _ = bus.write_read(0x40, &[0x02], &mut registers);
    }

    #[test]
    #[should_panic(expected = "was not scripted")]
    fn an_extra_step_is_caught() {
        let mut bus = MockBus::new();
        let _ = bus.write(0x25, &[0x00]);
    }

    #[test]
    #[should_panic(expected = "1 scripted steps never happened after step 0: read 0x25 9 bytes")]
    fn a_step_never_taken_is_caught() {
        let mock = MockBus::new();
        mock.expect_read(0x25, &[0; 9]);
        mock.done();
    }
}
