//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board's side of the I2C parts (crates/parts, doc/specs/avionics-refactor.md R2): the platform they run on
//! ([`PLATFORM`]: the control thread's GPIO interrupts and lapse watches, the recorder's streams), a laser's XSHUT
//! line on esp-idf-hal ([`Xshut`]), the registry of their published state by name ([`register`], [`shared`]), and
//! the calibrations' NVS: read at bring-up ([`restore_calibrations`]), written on the executor
//! ([`persist_pitot_tare`]; the BMM350's and the BNO055's blobs through owner::persist).

use std::mem::ManuallyDrop;
use std::sync::{Mutex, PoisonError};

use embedded_hal::digital::OutputPin;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::gpio::{AnyOutputPin, GpioError, Output, PinDriver};
use esp_idf_svc::sys::EspError;
use firmware::buses::i2c::I2cPort;
use parts::{AbsoluteState, Platform, Shared, Store};
use recorder::Rows;

/// One I2C part on this board: its laser XSHUT on an esp-idf-hal pin.
pub type Part = parts::Part<Xshut>;

/// An I2C part that came up on this board.
pub type Member = parts::Member<Xshut>;

/// The board the parts run on.
pub struct Board;

/// The one platform every part's run is handed.
pub static PLATFORM: Board = Board;

impl Platform for Board {
    fn interrupts_ready(&self) -> bool {
        crate::control::running()
    }

    fn arm_second_interrupt(&self, gpio: u8) -> Result<(), String> {
        crate::control::arm_second_interrupt(gpio).map_err(|error| error.to_string())
    }

    fn take_second_edges(&self) -> u32 {
        crate::control::take_second_edges()
    }

    fn arm_alert_interrupt(&self, gpio: u8) -> Result<(), String> {
        crate::control::arm_alert_interrupt(gpio).map_err(|error| error.to_string())
    }

    fn alert_edges(&self) -> u32 {
        crate::control::alert_edges()
    }

    fn lapses(&self, name: &str) -> Vec<(String, u32)> {
        crate::control::lapses_of(name)
    }

    fn rows(&self, name: &'static str) -> Rows<'static> {
        crate::recorder::rows(name)
    }
}

/// Every part's published state by its entry's name, registered as the part came up (bring-up, on main) and read by
/// the executor's commands; never on an owner's tick.
static REGISTRY: Mutex<Vec<(&'static str, Shared)>> = Mutex::new(Vec::new());

/// Registers `member`'s published state under its name, when it has one.
pub fn register(member: &Member) {
    if let Some(shared) = member.shared {
        REGISTRY
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push((member.name, shared));
    }
}

/// The published state of the part named `name`.
pub fn shared(name: &str) -> Option<Shared> {
    registered()
        .into_iter()
        .find_map(|(registered, shared)| (registered == name).then_some(shared))
}

/// Every registered part's published state, in the order the parts came up.
fn registered() -> Vec<(&'static str, Shared)> {
    REGISTRY.lock().unwrap_or_else(PoisonError::into_inner).clone()
}

/// The calibrations the magnetometers and the BNO055s owe, by name, in plan B's words (bmm350.calibration(),
/// bno055.calibration()): worded here, on the executor, from their published state, so the owner never formats on its
/// tick (D9 of the M9 plan).
pub fn owed_calibrations() -> Vec<(&'static str, String)> {
    registered()
        .into_iter()
        .filter_map(|(name, shared)| match shared {
            Shared::Magnetometer(published) => {
                let text = published.read().instruction();
                (!text.is_empty()).then_some((name, text))
            }
            Shared::Absolute(published) => published.read().instruction().map(|text| (name, text)),
            _ => None,
        })
        .collect()
}

/// The BNO055's state, None when no BNO055 came up (`verify`, the heartbeat).
pub fn absolute_state() -> Option<AbsoluteState> {
    registered().into_iter().find_map(|(_, shared)| match shared {
        Shared::Absolute(published) => Some(published.read()),
        _ => None,
    })
}

/// The pitot's zero offset in force, Pa, None before its state is known or when no pitot came up (the checkpoint).
pub fn pitot_zero_pa() -> Option<f32> {
    registered().into_iter().find_map(|(_, shared)| match shared {
        Shared::Pitot(pitot) => pitot.state().map(|state| state.zero_offset_pa),
        _ => None,
    })
}

/// Persists the pitot tare the owner captured, if one waits (plan B sdp810._persist_zero: NVS `sdp810_zero`, the
/// Pa x 100 i32 shared with plan B), on the executor after the owner answered a `calibrate` or an `update`. None when
/// nothing waited or it persisted, else why it did not (the tare stays in force either way).
pub fn persist_pitot_tare(partition: &esp_idf_svc::nvs::EspDefaultNvsPartition) -> Option<String> {
    use warmstart::Nvs;
    let tare = registered().into_iter().find_map(|(_, shared)| match shared {
        Shared::Pitot(pitot) => pitot.take_owed(),
        _ => None,
    })?;
    let written = crate::warm::ColudoNvs::open(partition)
        .map_err(|error| error.to_string())
        .and_then(|mut nvs| {
            nvs.set_i32(drivers::sdp810::NVS_ZERO_KEY, tare)
                .and_then(|()| nvs.commit())
                .map_err(|error| format!("NVS error {:#x}", error.0))
        });
    match written {
        Ok(()) => {
            log::info!("sdp810 :: tare {:.2} Pa persisted", tare as f32 / 100.0);
            None
        }
        Err(why) => Some(format!("tared but not persisted: {why}")),
    }
}

/// The calibrations' store: NVS, opened for each read.
struct Saved<'a>(&'a esp_idf_svc::nvs::EspDefaultNvsPartition);

impl Store for Saved<'_> {
    fn blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, String> {
        use warmstart::Nvs;
        crate::warm::ColudoNvs::open(self.0)
            .map_err(|error| error.to_string())
            .and_then(|mut nvs| {
                nvs.get_blob(key, buffer)
                    .map_err(|error| format!("NVS error {:#x}", error.0))
            })
    }

    fn integer(&mut self, key: &str) -> Result<i32, String> {
        use warmstart::Nvs;
        crate::warm::ColudoNvs::open(self.0)
            .map_err(|error| error.to_string())
            .and_then(|mut nvs| nvs.get_i32(key).map_err(|error| format!("NVS error {:#x}", error.0)))
    }
}

/// Restores what each part learned from NVS after its setup (crates/parts restore): each BNO055's profile over its
/// lent port, then the pitot's tare and each magnetometer's calibration. Once at bring-up, on main, before the owners
/// run.
pub fn restore_calibrations(
    partition: &esp_idf_svc::nvs::EspDefaultNvsPartition,
    members: &mut [Vec<Member>; 2],
    ports: &mut [(crate::owner::Parked, I2cPort)],
) {
    let mut store = Saved(partition);
    let mut delay = Delay::new_default();
    for (_, port) in ports.iter_mut() {
        let id = usize::from(port.id());
        parts::restore_profiles(&mut store, &mut members[id], port, &mut delay);
    }
    parts::restore_saved(&mut store, members.iter_mut().flatten());
}

/// A VL53 ranger's XSHUT line, when the config wires one (v0.1; layout nulls it on v1.x), driven for the rest of the
/// run once a part first takes it. Dropping an esp-idf-hal PinDriver disables the pad, so the line would float when a
/// failed attempt's part is dropped: on a pin two laser entries share (the default config's GPIO5), the fitted
/// laser's XSHUT after the other entry's attempts. Plan B's `Pin` was never released either.
pub struct Xshut(ManuallyDrop<PinDriver<'static, Output>>);

impl Xshut {
    /// GPIO `gpio` as an output for a ranger's XSHUT (crates/parts Part::build's `xshut`).
    ///
    /// Errors: the pin would not configure, as one line.
    pub fn output(gpio: u8) -> Result<Xshut, String> {
        // SAFETY: the board config gives this GPIO to the rangers' XSHUT alone; the driver is never released.
        let driver =
            PinDriver::output(unsafe { AnyOutputPin::steal(gpio) }).map_err(|error: EspError| error.to_string())?;
        Ok(Xshut(ManuallyDrop::new(driver)))
    }
}

impl embedded_hal::digital::ErrorType for Xshut {
    type Error = GpioError;
}

impl OutputPin for Xshut {
    fn set_low(&mut self) -> Result<(), GpioError> {
        OutputPin::set_low(&mut *self.0)
    }

    fn set_high(&mut self) -> Result<(), GpioError> {
        OutputPin::set_high(&mut *self.0)
    }
}
