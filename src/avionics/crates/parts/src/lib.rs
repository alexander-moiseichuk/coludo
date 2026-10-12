//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The I2C parts as their bus owner runs them (doc/specs/avionics-refactor.md R2; plan B's task registry and its
//! drivers' run loops), over embedded-hal, so the host tests drive them on the drivers' mock bus:
//!
//! - what runs each board-config implementation ([`Kind`], [`classify`]);
//! - [`Part`]: one I2C part's driver, built afresh from its entry for every setup attempt, as plan B creates a task
//!   per attempt; its owner sets it up, re-arms it after a general call and asks its diagnose when the last attempt
//!   failed;
//! - [`Member`]: a part that came up, with its [`Run`] (its channels, its stream, its health; plan B's run loop) and
//!   its [`Shared`] state, which the owner publishes ([`published::Published`]) and the executor reads (plan B's
//!   `inspect`, [`own`]);
//! - the calibrations a part keeps across power cycles, read through the board's [`Store`] ([`restore_profiles`],
//!   [`restore_saved`]).
//!
//! The board's own facilities -- the control thread's GPIO interrupts and the recorder's streams -- come in through
//! [`Platform`]; the firmware keeps the platform, the registry of names and the calibrations' NVS writes.

mod absolute;
mod barometer;
mod due;
mod inertial;
mod inspect;
mod magnetometer;
mod member;
mod part;
mod pitot;
mod power;
pub mod published;
mod ranger;
mod restore;

use board_config::Entry;
use drivers::axes::Axes;
use recorder::Rows;
use serde_json::Value;

pub use absolute::AbsoluteState;
pub use barometer::BarometerState;
pub use inertial::InertialState;
pub use inspect::{Lookup, Own, own};
pub use magnetometer::MagnetometerState;
pub use member::{Member, Shared};
pub use part::{Calibrated, Event, Part, Run};
pub use pitot::{PitotShared, PitotState};
pub use power::PowerState;
pub use restore::{Store, restore_profiles, restore_saved};

/// The failure reason of a plan B implementation this port has not reached yet (D13 of the M5 plan): plan B's
/// `no driver/activity`, with why.
pub const NOT_PORTED_YET: &str = "no driver/activity -- not ported yet";

/// What runs an entry's implementation (its `driver`, else its `activity`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A [`Part`] on an I2C port.
    I2c,
    /// A part on SPI2: the LSM6DSO32 on its default transport.
    Spi,
    /// The separation switch: a GPIO the control thread reads.
    Separation,
    /// The stage machine: its settings read from the entry, its steps on the control thread.
    Sequencer,
    /// The attitude filter: its settings read from the entry, its steps on the control thread (M9).
    Attitude,
    /// The checkpoint: its cadence read from the entry, its passes on the supervisor (the firmware's warm start).
    Checkpoint,
    /// The watchdog: its settings read from the entry, the task watchdog and the stall check on the supervisor.
    Watchdog,
    /// The GNSS on its UART: its own thread reads it (M11).
    Gnss,
    /// The pad drift calibration: its settings read from the entry, its passes on the supervisor (M11).
    GnssCalib,
    /// The CC-less field agent: its settings read from the entry, its passes on the executor (M11).
    Field,
    /// The HITL sim-link: its port read from the entry, its frames on its own thread (M12).
    Hitl,
    /// A fin servo (SG90, MG90S): its pin and settings read from the entry, its PWM on LEDC (M13).
    Servo,
    /// The flight loop: its law read from the entry and the fins section, its steps on the control thread (M13).
    Flight,
    /// Run natively by the firmware, with nothing to set up: up.
    Native,
    /// A plan B implementation not ported yet: failed with [`NOT_PORTED_YET`].
    NotPorted,
    /// No implementation plan B knows: failed with plan B's own `no driver/activity`.
    Unknown,
}

/// Implementations the firmware runs natively: the recorder, the CC link, Wi-Fi, the supervisor's health, the camera,
/// and the BLE radio switch, a stub in plan B's no-BLE shape (drivers::bluetooth; its inspect, update and probe are
/// the commands').
const NATIVE: [&str; 6] = ["recorder", "cc", "wifi", "health", "ov5647", "bluetooth"];

/// Plan B implementations not ported: the ADXL375, no longer fitted (M18 leaves it out).
const NOT_PORTED: [&str; 1] = ["adxl375"];

/// What runs `entry`. The LSM6DSO32 is an SPI part unless its entry binds it to I2C (plan B's lsm6dso32.py:144: the
/// `bus` defaults to 'spi').
pub fn classify(entry: &Entry) -> Kind {
    match entry.implementation {
        "lsm6dso32" => match controller::binding(entry, "spi", Some(drivers::lsm6dso32::ADDRESS)) {
            controller::Binding::I2c { .. } => Kind::I2c,
            _ => Kind::Spi,
        },
        implementation if default_address(implementation).is_some() => Kind::I2c,
        "separation" => Kind::Separation,
        "sequencer" => Kind::Sequencer,
        "attitude" => Kind::Attitude,
        "checkpoint" => Kind::Checkpoint,
        "watchdog" => Kind::Watchdog,
        "neo6mv2" | "atgm336h" | "neom8n" => Kind::Gnss,
        "gnss_calib" => Kind::GnssCalib,
        "field" => Kind::Field,
        "hitl" => Kind::Hitl,
        "sg90" | "mg90s" => Kind::Servo,
        "flight" => Kind::Flight,
        implementation if NATIVE.contains(&implementation) => Kind::Native,
        implementation if NOT_PORTED.contains(&implementation) => Kind::NotPorted,
        _ => Kind::Unknown,
    }
}

/// Whether the implementation re-zeroes a barometer's ground: the ICP-10111, the BMP280 and the BMP581 (M9), whose
/// `calibrate` and `update` (`ground`, `rezero`) also run at DONE, the way back from a false launch ([`Part::rezeroes`]
/// says it of a built part).
pub fn rezeroes(implementation: &str) -> bool {
    matches!(implementation, "icp10111" | "bmp280" | "bmp581")
}

/// The `axes` of an IMU entry (drivers::axes, M9): the identity when absent.
///
/// Errors: a malformed key or a mirror, naming the key (the part's setup fails with it).
pub fn imu_axes(entry: &Entry) -> Result<Axes, String> {
    Axes::from_config(&Value::Object(entry.fields.clone())).map_err(|error| error.to_string())
}

/// Every I2C part this port drives, by implementation name, with the datasheet address its driver binds when its
/// entry has no `addr` (plan B i2cbus.py:301). A new part is a line here, its driver, its run and its [`Part`] arm.
pub const KNOWN: [(&str, u8); 11] = [
    ("bmi323", drivers::bmi323::ADDRESS),
    ("bmm350", drivers::bmm350::ADDRESS),
    ("bmp280", drivers::bmp280::ADDRESS),
    ("bmp581", drivers::bmp581::ADDRESS),
    ("bno055", drivers::bno055::ADDRESS),
    ("icp10111", drivers::icp10111::ADDRESS),
    ("ina226", drivers::ina226::ADDRESS),
    ("lsm6dso32", drivers::lsm6dso32::ADDRESS),
    ("sdp810", drivers::sdp810::ADDRESS),
    ("vl53l1x", drivers::vl53l1x::ADDRESS),
    ("vl53l4cx", drivers::vl53l1x::ADDRESS),
];

/// The datasheet address an I2C part's driver binds when its entry has no `addr` ([`KNOWN`]); None for an
/// implementation that is no I2C part.
pub fn default_address(implementation: &str) -> Option<u8> {
    KNOWN
        .iter()
        .find_map(|(known, address)| (*known == implementation).then_some(*address))
}

/// What a part's run needs of the board beyond its bus: the GPIO interrupts the control thread serves (it installs
/// the ISR service), the control thread's lapse watches, and the recorder's streams. Every call is cheap and never
/// blocks: the owner calls them on its wake.
pub trait Platform: Sync {
    /// Whether the GPIO interrupt service runs: a run arms its line only after.
    fn interrupts_ready(&self) -> bool;
    /// Counts the second IMU's INT1 edges on `gpio` from now on.
    ///
    /// Errors: the pin would not arm, as one line.
    fn arm_second_interrupt(&self, gpio: u8) -> Result<(), String>;
    /// The second IMU's INT1 edges since the last take.
    fn take_second_edges(&self) -> u32;
    /// Counts the power monitor's ALERT edges on `gpio` from now on.
    ///
    /// Errors: the pin would not arm, as one line.
    fn arm_alert_interrupt(&self, gpio: u8) -> Result<(), String>;
    /// The power monitor's ALERT edges since boot.
    fn alert_edges(&self) -> u32;
    /// The lapses of source `name`'s channels the control thread watched, by quantity.
    fn lapses(&self, name: &str) -> Vec<(String, u32)>;
    /// The rows of the stream registered under `name` (a refused row logged once a run).
    fn rows(&self, name: &'static str) -> Rows<'static>;
}

/// The length of a vector of three.
fn magnitude([x, y, z]: [f32; 3]) -> f32 {
    (x * x + y * y + z * z).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_known_part_has_its_datasheet_address() {
        assert_eq!(default_address("icp10111"), Some(0x63));
        assert_eq!(default_address("ina226"), Some(0x40));
        assert_eq!(
            default_address("vl53l4cx"),
            default_address("vl53l1x"),
            "one silicon family, one address"
        );
    }

    #[test]
    fn no_other_implementation_has_an_address() {
        // NEGATIVE
        for implementation in ["sg90", "neo6mv2", "sequencer", "adxl375", "ov5647", ""] {
            assert_eq!(default_address(implementation), None, "{implementation}");
        }
    }

    #[test]
    fn known_names_are_unique() {
        let mut names: Vec<&str> = KNOWN.iter().map(|(name, _)| *name).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), KNOWN.len());
    }
}
