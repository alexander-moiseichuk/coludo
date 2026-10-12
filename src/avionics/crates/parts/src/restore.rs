//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! What each part learned, restored at bring-up from the board's store, so a power cycle does not cost the operator a
//! turn (plan B's `_restore` in each driver's setup). Once, on main, before the owners run; best effort, as plan B's:
//! no saved value or a bus error leaves the calibration owed.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::member::Member;
use crate::part::Part;

/// The board's store of calibrations (NVS on the board): read-only here; the executor writes.
pub trait Store {
    /// The blob saved under `key`, into `buffer`: its length.
    ///
    /// Errors: nothing saved, or the store failed, as one line.
    fn blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, String>;

    /// The integer saved under `key`.
    ///
    /// Errors: nothing saved, or the store failed, as one line.
    fn integer(&mut self, key: &str) -> Result<i32, String>;
}

/// Restores each BNO055's learned profile (plan B bno055._restore_profile) among `members`, the parts of one port,
/// written over `port` (CONFIG mode, the profile, NDOF).
pub fn restore_profiles<Pin, Bus: I2c>(
    store: &mut dyn Store,
    members: &mut [Member<Pin>],
    port: &mut Bus,
    delay: &mut dyn DelayNs,
) {
    for member in members {
        let Part::Bno055(fused) = &mut member.part else {
            continue;
        };
        let mut profile = [0u8; drivers::bno055::PROFILE_BYTES];
        match store.blob(drivers::bno055::NVS_KEY, &mut profile) {
            Ok(drivers::bno055::PROFILE_BYTES) => match fused.restore_profile(port, delay, &profile) {
                Ok(()) => log::info!("{} :: calibration profile restored from NVS", member.name),
                Err(fault) => log::warn!("{} :: calibration restore failed: {fault}", member.name),
            },
            Ok(length) => log::warn!(
                "{} :: the saved profile is no profile ({length} bytes): the figure-8 is owed",
                member.name
            ),
            Err(why) => log::info!("{} :: no saved profile ({why}): the figure-8 is owed", member.name),
        }
        if let Some(shared) = member.shared {
            shared.note(&member.part);
        }
    }
}

/// Restores what needs no bus among `members`, every port's: the pitot's tare (the store's wins over the config's
/// `zero_offset_pa`, plan B sdp810.setup, so a reboot mid-glide keeps it), then each magnetometer's hard/soft-iron
/// calibration (plan B bmm350._restore).
pub fn restore_saved<'a, Pin: 'a>(store: &mut dyn Store, members: impl Iterator<Item = &'a mut Member<Pin>>) {
    let members: Vec<&mut Member<Pin>> = members.collect();
    let mut magnetometers = Vec::new();
    for member in members {
        match &mut member.part {
            Part::Sdp810(pitot) => {
                match store.integer(drivers::sdp810::NVS_ZERO_KEY) {
                    Ok(zero) => {
                        pitot.restore_zero(zero);
                        log::info!(
                            "{} :: zero restored from NVS: {:.2} Pa",
                            member.name,
                            zero as f32 / 100.0
                        );
                    }
                    Err(why) => log::info!(
                        "{} :: no saved tare ({why}): the config's zero until a calibrate",
                        member.name
                    ),
                }
                if let Some(shared) = member.shared {
                    shared.note(&member.part);
                }
            }
            Part::Bmm350(_) => magnetometers.push(member),
            _ => {}
        }
    }
    for member in magnetometers {
        let Part::Bmm350(magnetometer) = &mut member.part else {
            continue;
        };
        let name = member.name;
        let mut blob = [0u8; drivers::bmm350::CALIBRATION_BYTES];
        match store
            .blob(drivers::bmm350::NVS_KEY, &mut blob)
            .map(|length| magnetometer.restore(&blob[..length]))
        {
            Ok(Some(calibration)) => log::info!(
                "{name} :: calibration restored: centre ({}, {}) radii ({}, {})",
                calibration.centre_x,
                calibration.centre_y,
                calibration.radius_x,
                calibration.radius_y
            ),
            Ok(None) => log::warn!("{name} :: the saved calibration is no calibration (wrong length): axes raw"),
            Err(why) => log::info!("{name} :: no saved calibration ({why}): axes raw until the operator's turn"),
        }
        if let Some(shared) = member.shared {
            shared.note(&member.part);
        }
    }
}
