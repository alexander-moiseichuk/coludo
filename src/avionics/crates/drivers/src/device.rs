//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device half of src/glider task.py: what every bus driver offers its owner ([`Device`]), the fault a device
//! conversation ends in ([`Fault`]), and the consecutive-failure counter the run loops escalate on ([`Strikes`],
//! plan B's `Task.strike`).
//!
//! Plan B's Task also owns the run loop, the channels, the telemetry stream and the inspect/update surface. In Rust
//! those belong to the bus owner thread and its per-part glue (avionics-port.md §4.1: one owner per resource), so a
//! driver is a passive object: the owner hands it the bus and a delay for each conversation, and serves its own
//! mailbox between conversations. That is also why plan B's `Task.claim()` has no counterpart here: the owner
//! thread is the only caller there is.
//!
//! The general-call rule (operator, 2026-10-06; plan B commit 76c138c): the ICP-10111 un-wedges a latched part with
//! an I2C general-call reset, which resets EVERY device on that bus that honours it. The bus owner then calls
//! [`Device::rearm`] on every device registered on that port, found by membership, never by a hard-coded list.
//! Every driver offers it, including the ones plan B left to recover on their own failure path.

use core::fmt;

use embedded_hal::delay::DelayNs;

/// One device on a bus, as its owner thread drives it.
///
/// `Bus` is the transport (an embedded-hal `I2c`, or an `SpiDevice` for the SPI parts); a driver implements the
/// trait for every bus type that carries it, so the firmware's `I2cPort` and the host tests' scripted bus run the
/// same code. The methods take the delay as a `dyn` so the owner can keep its devices as `dyn Device` and walk
/// them by membership after a general call.
pub trait Device<Bus> {
    /// The transport's own error: a NAK, a lost arbitration, a timeout (plan B's `OSError`).
    type BusError;

    /// Bring the part from its power-on (or an unknown) state to producing data: plan B setup()'s bus half.
    ///
    /// An `Err` is plan B's `setup() -> False`: the device is not up, and the bring-up's retries and diagnose
    /// decide what happens next.
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Self::BusError>>;

    /// Restore what a bus-wide reset (the ICP's general call) took from the part, so it produces valid data again.
    ///
    /// Called on every device registered on the bus the general call went out on (see the module notes). The
    /// part has kept its address and its identity but lost its volatile state: a mode, a calibration register,
    /// a configuration block. An `Err` is logged by the owner; the device's own failure path is still there.
    fn rearm(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Self::BusError>>;
}

/// Why a device conversation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault<BusError> {
    /// The transport failed: nothing acked, arbitration was lost, the transfer timed out. Plan B's `OSError`, the
    /// fault the run loops are built to ride out.
    Bus(BusError),
    /// The part answered, but as another part: its identity register read `read` where `expected` belongs. Two
    /// lasers share 0x29, so this is how a driver refuses the other one's silicon.
    Identity {
        /// The identity this driver's part carries.
        expected: u16,
        /// What the part at the address answered.
        read: u16,
    },
    /// A frame arrived and failed its CRC: a different device at the address, or bus noise.
    Checksum,
    /// The part never became ready within its datasheet window (a firmware that does not boot).
    Timeout,
    /// The configuration cannot be programmed into the part (a calibration that does not fit its register).
    Configuration,
    /// A control line the part needs (an XSHUT pin) refused to switch.
    Pin,
}

impl<BusError: fmt::Debug> fmt::Display for Fault<BusError> {
    /// The one-line text the owner logs (`<device> :: <fault>`).
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Fault::Bus(error) => write!(formatter, "bus {error:?}"),
            Fault::Identity { expected, read } => write!(formatter, "id 0x{read:04x} != 0x{expected:04x}"),
            Fault::Checksum => write!(formatter, "crc reject"),
            Fault::Timeout => write!(formatter, "not ready in time"),
            Fault::Configuration => write!(formatter, "configuration does not fit the part"),
            Fault::Pin => write!(formatter, "control pin refused"),
        }
    }
}

/// A run of CONSECUTIVE failures, acted on once (plan B `Task.strike`).
///
/// Four plan B drivers had hand-rolled this (icp10111 read failures -> general-call reset, sdp810 -> continuous-mode
/// restart, lsm6dso32 INT1 timeouts -> poll fallback, bno055 frozen fusion -> withhold attitude) and they had already
/// drifted apart: two fired at `== limit`, one at `>= limit`, each with its own counter and reset. The subtle part
/// is the same every time: act ONCE while the fault persists, not on every tick, and rearm only on a genuine
/// success.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Strikes {
    /// Consecutive failures since the last good pass. Saturates instead of wrapping: plan B's integer never wraps,
    /// and a wrap would let a fault that never ends fire a second time.
    count: u32,
}

impl Strikes {
    /// Count one pass: true exactly ONCE, on the failure that brings the run to `limit`; false on every other pass,
    /// further failures included. A good pass (`failed` false) rearms the count.
    pub fn strike(&mut self, failed: bool, limit: u32) -> bool {
        if !failed {
            self.count = 0;
            return false;
        }
        self.count = self.count.saturating_add(1);
        self.count == limit
    }

    /// The consecutive failures so far.
    pub fn count(&self) -> u32 {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::{Fault, Strikes};

    #[test]
    fn strikes_fire_once_at_the_limit() {
        let mut strikes = Strikes::default();
        let fired: Vec<bool> = (0..8).map(|_| strikes.strike(true, 5)).collect();
        assert_eq!(fired, [false, false, false, false, true, false, false, false]);
        assert_eq!(strikes.count(), 8);
    }

    #[test]
    fn a_good_pass_rearms_the_run() {
        let mut strikes = Strikes::default();
        for _ in 0..4 {
            assert!(!strikes.strike(true, 5));
        }
        assert!(!strikes.strike(false, 5), "a good pass never fires");
        assert_eq!(strikes.count(), 0);
        // NEGATIVE: the four failures before the good pass do not count toward the next run
        for _ in 0..4 {
            assert!(!strikes.strike(true, 5));
        }
        assert!(strikes.strike(true, 5));
    }

    #[test]
    fn a_fault_that_never_ends_fires_only_once() {
        let mut strikes = Strikes { count: u32::MAX - 1 };
        assert!(!strikes.strike(true, 5));
        assert!(
            !strikes.strike(true, 5),
            "saturated, not wrapped back through the limit"
        );
        assert_eq!(strikes.count(), u32::MAX);
    }

    #[test]
    fn faults_print_as_one_line() {
        let identity: Fault<()> = Fault::Identity {
            expected: 0xEACC,
            read: 0xEBAA,
        };
        assert_eq!(identity.to_string(), "id 0xebaa != 0xeacc");
        assert_eq!(Fault::Bus("nak").to_string(), "bus \"nak\"");
        assert_eq!(Fault::<()>::Checksum.to_string(), "crc reject");
    }
}
