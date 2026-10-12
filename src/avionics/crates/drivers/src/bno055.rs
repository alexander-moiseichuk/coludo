//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BNO055 9-DOF IMU on the SEN0253 (src/glider/drivers/bno055.py), M18: the BACKUP attitude source of the v0.1
//! boards (TMS-7C, 7D), at priority 2 below the attitude filter. In NDOF mode the chip fuses its own accelerometer,
//! gyro and magnetometer and reports absolute orientation; [`Sample`] reads the 24-byte ACC..EUL block (accel, mag,
//! gyro, Euler, all int16 little-endian): the Euler angles (heading, roll, pitch, 16 LSB a degree, in plan B's
//! attitude convention: the channel carries them as plan B pushed them) for `attitude`, the accelerometer (100 LSB a
//! m/s², gravity included) in g for `accel` as a low-g backup.
//!
//! THE STALL (plan B's measured failure, [`Bno055::judge`]). The fusion core can latch: the Euler triple repeats bit
//! for bit forever while the raw accel and gyro in the same block keep streaming (measured across a power cycle, both
//! fusion modes, either clock). The channel then stays FRESH, so no staleness guard downstream fires. A still part
//! legitimately repeats its Euler too, so the judgement is made ONLY WHILE THE PART ROTATES, by its own gyro in the
//! same block (|gx| + |gy| + |gz| above [`TURNING_LSB`]): [`STALL_SAMPLES`] rotating samples in a row with a frozen
//! Euler latch the stall, the firmware withholds `attitude` (it goes stale, the databoard moves on) and keeps `accel`
//! (that half still works), and the first Euler that moves clears it.
//!
//! THE RESTART (measured on the taster, 2026-10-12). After every switch into NDOF (the bring-up, a re-arm, the
//! profile's read and write) the fusion reads an all-zero Euler for 60-80 ms before its first real one: a level
//! attitude that is no measurement. [`Bno055::judge`] answers [`Fusion::Starting`] for those, until the first non-zero
//! triple.
//!
//! CALIBRATION (plan B's latch and profile). NDOF does not converge without motion, and the magnetometer is the gate:
//! the operator's figure-8 takes CALIB_STAT's mag field to 3. The register is the chip's confidence in RECENT data and
//! falls back to 2 within a minute of sitting still while the learned offsets stay, so convergence is a LATCH
//! ([`Bno055::calibrated`]), set when mag reaches [`MAG_CALIBRATED`] or when a saved profile is restored. The learned
//! profile ([`PROFILE_BYTES`] at 0x55, readable and writable in CONFIG mode only) is saved by `calibrate`
//! ([`Bno055::read_profile`]) and written back at bring-up ([`Bno055::restore_profile`]), so a power cycle on the rail
//! needs no figure-8. [`Bno055::owed`] says what is still owed: the figure-8, then the save.
//!
//! Not here (firmware glue): the channel pushes and the stream, the profile's NVS key, the probe's log lines.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::commons::id_classify;
use crate::device::{Device, Fault};

/// The default address (COM3 low; 0x29 when high).
pub const ADDRESS: u8 = 0x28;

/// CHIP_ID: [`IDENTITY`].
pub const CHIP_ID: u8 = 0x00;

/// OPR_MODE: the operating mode.
pub const OPR_MODE: u8 = 0x3D;

/// PWR_MODE: the power mode.
pub const PWR_MODE: u8 = 0x3E;

/// The ACC..EUL block's first register: [`BLOCK_LENGTH`] bytes.
pub const DATA: u8 = 0x08;

/// CALIB_STAT: sys bits 7..6, gyr 5..4, acc 3..2, mag 1..0; 3 is fully calibrated.
pub const CALIB_STAT: u8 = 0x35;

/// The learned profile's first register (ACC_OFFSET_X_LSB .. MAG_RADIUS_MSB): [`PROFILE_BYTES`].
pub const PROFILE: u8 = 0x55;

/// The BNO055's identity.
pub const IDENTITY: u8 = 0xA0;

/// CONFIG mode: the fusion off, the profile readable and writable.
pub const MODE_CONFIG: u8 = 0x00;

/// NDOF mode: full 9-DOF absolute-orientation fusion.
pub const MODE_NDOF: u8 = 0x0C;

/// The normal power mode.
pub const POWER_NORMAL: u8 = 0x00;

/// The block read's default period (plan B's `period_ms` 20: 50 Hz; the fusion runs at 100 Hz inside).
pub const DEFAULT_PERIOD_MS: u32 = 20;

/// How often the firmware reads CALIB_STAT for the operator (plan B's ~1 Hz).
pub const CALIBRATION_PERIOD_MS: u32 = 1000;

/// The NVS key of the saved profile (plan B's `bno055_calib`, shared with it).
pub const NVS_KEY: &str = "bno055_calib";

/// The wait after a mode switch (plan B's 25 ms; the datasheet's 7 ms to CONFIG, 19 ms from it).
pub const MODE_SETTLE_MS: u32 = 25;

/// The block's length: accel, mag, gyro, Euler, 6 bytes each.
pub const BLOCK_LENGTH: usize = 24;

/// The Euler triple's offset in the block.
pub const EULER_OFFSET: usize = 18;

/// The gyro's offset in the block.
pub const GYRO_OFFSET: usize = 12;

/// The profile's length.
pub const PROFILE_BYTES: usize = 22;

/// |gx| + |gy| + |gz| above this (16 LSB a deg/s, ~5 deg/s summed) is a part that rotates: a frozen Euler is a fault.
pub const TURNING_LSB: i32 = 80;

/// Rotating samples in a row with a frozen Euler that latch the stall: ~1 s at the 50 Hz default.
pub const STALL_SAMPLES: u32 = 50;

/// CALIB_STAT's mag level that means the figure-8 is done.
pub const MAG_CALIBRATED: u8 = 3;

/// Euler LSB a degree.
const EULER_PER_DEGREE: f32 = 16.0;

/// Accel LSB a g: 100 LSB a m/s², 9.80665 m/s² a g.
const ACCEL_PER_G: f32 = 980.665;

/// One read of the block, in the channels' units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Heading, roll, pitch in degrees, plan B's attitude convention.
    pub euler: [f32; 3],
    /// The accelerometer in g, gravity included, the part's own axes.
    pub accel: [f32; 3],
    /// The raw Euler counts: the stall judge compares them bit for bit.
    pub euler_raw: [i16; 3],
    /// The raw gyro counts (16 LSB a deg/s): the stall judge's rotation.
    pub gyro_raw: [i16; 3],
}

impl Sample {
    /// Decode the 24-byte block (bno055.sample).
    pub fn decode(block: &[u8; BLOCK_LENGTH]) -> Sample {
        let word = |at: usize| i16::from_le_bytes([block[at], block[at + 1]]);
        let triple = |at: usize| [word(at), word(at + 2), word(at + 4)];
        let euler_raw = triple(EULER_OFFSET);
        let accel_raw = triple(0);
        Sample {
            euler: euler_raw.map(|count| f32::from(count) / EULER_PER_DEGREE),
            accel: accel_raw.map(|count| f32::from(count) / ACCEL_PER_G),
            euler_raw,
            gyro_raw: triple(GYRO_OFFSET),
        }
    }
}

/// CALIB_STAT's four fields, 0..3 each.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CalibrationState {
    /// The fusion system's.
    pub system: u8,
    /// The gyro's.
    pub gyro: u8,
    /// The accelerometer's.
    pub accel: u8,
    /// The magnetometer's: the gate.
    pub mag: u8,
}

impl CalibrationState {
    /// CALIB_STAT again: what [`CalibrationState::decode`] reads back.
    pub fn encode(self) -> u8 {
        (self.system << 6) | (self.gyro << 4) | (self.accel << 2) | self.mag
    }

    /// Decode CALIB_STAT.
    pub fn decode(raw: u8) -> CalibrationState {
        CalibrationState {
            system: raw >> 6,
            gyro: (raw >> 4) & 3,
            accel: (raw >> 2) & 3,
            mag: raw & 3,
        }
    }
}

/// What is still owed (bno055.calibration), from the last CALIB_STAT read (`state`, None before the first), the
/// convergence latch and whether the profile is saved: the figure-8 while unconverged, then the save; None once both
/// are done. A free function, so the firmware words it from its run's published state, off the bus owner.
pub fn owed(state: Option<CalibrationState>, converged: bool, saved: bool) -> Option<String> {
    if converged {
        return (!saved)
            .then(|| "converged -- run `calibrate imu_bno055` NOW to save the profile across power cycles".into());
    }
    Some(match state {
        None => "move the airframe in a slow figure-8 (BNO055 calibration not read yet)".into(),
        Some(state) => format!(
            "move the airframe in a slow FIGURE-8 until mag reads 3 (now sys {} gyr {} acc {} mag {})",
            state.system, state.gyro, state.accel, state.mag
        ),
    })
}

/// What [`Bno055::judge`] makes of a sample's fusion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fusion {
    /// The Euler moved, or the part is still, or the stall is not yet proven: publish the attitude.
    Live,
    /// The stall latched on this sample: withhold the attitude, and log it (once).
    Stalled,
    /// Still stalled: keep withholding.
    StillStalled,
    /// The fusion restarted (a switch into NDOF) and its Euler still reads all zero: withhold the attitude.
    Starting,
}

/// One BNO055 on an I2C bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Bno055 {
    /// The 7-bit address.
    address: u8,
    /// The last Euler counts judged.
    last_euler: Option<[i16; 3]>,
    /// Rotating samples in a row with a frozen Euler.
    strikes: u32,
    /// The stall latch.
    stalled: bool,
    /// The fusion restarted and has not yet read a non-zero Euler.
    starting: bool,
    /// The last CALIB_STAT read, None until the first.
    state: Option<CalibrationState>,
    /// The convergence latch: mag reached [`MAG_CALIBRATED`] once, or a profile was restored.
    converged: bool,
    /// The latch came from a restored profile, not a figure-8 this session.
    restored: bool,
    /// The learned profile is saved (restored, or `calibrate` saved it).
    saved: bool,
}

impl Bno055 {
    /// A driver for the part at `address`.
    pub fn new(address: u8) -> Bno055 {
        Bno055 {
            address,
            last_euler: None,
            strikes: 0,
            stalled: false,
            starting: false,
            state: None,
            converged: false,
            restored: false,
            saved: false,
        }
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Read CHIP_ID.
    pub fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>> {
        let mut identity = [0u8; 1];
        bus.write_read(self.address, &[CHIP_ID], &mut identity)
            .map_err(Fault::Bus)?;
        Ok(identity[0])
    }

    /// Read one sample of the block.
    pub fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Sample, Fault<Bus::Error>> {
        let mut block = [0u8; BLOCK_LENGTH];
        bus.write_read(self.address, &[DATA], &mut block).map_err(Fault::Bus)?;
        Ok(Sample::decode(&block))
    }

    /// Judge a sample's fusion (bno055._fusion_alive; see the module notes): a moved Euler clears the stall, a frozen
    /// one counts a strike only while the part's own gyro says it rotates, [`STALL_SAMPLES`] strikes latch it.
    pub fn judge(&mut self, sample: &Sample) -> Fusion {
        if self.starting {
            if sample.euler_raw == [0; 3] {
                return Fusion::Starting;
            }
            self.starting = false;
        }
        if self.last_euler != Some(sample.euler_raw) {
            self.last_euler = Some(sample.euler_raw);
            self.strikes = 0;
            self.stalled = false;
            return Fusion::Live;
        }
        let [x, y, z] = sample.gyro_raw.map(|count| i32::from(count).abs());
        if x + y + z > TURNING_LSB {
            self.strikes = self.strikes.saturating_add(1);
        } else {
            self.strikes = 0;
        }
        if self.stalled {
            return Fusion::StillStalled;
        }
        if self.strikes >= STALL_SAMPLES {
            self.stalled = true;
            return Fusion::Stalled;
        }
        Fusion::Live
    }

    /// Whether the fusion is latched stalled.
    pub fn stalled(&self) -> bool {
        self.stalled
    }

    /// Read CALIB_STAT (bno055._poll_calibration's read; the firmware paces it to ~1 Hz). Returns true when this
    /// read set the convergence latch (log it once).
    pub fn poll_calibration<Bus: I2c>(&mut self, bus: &mut Bus) -> Result<bool, Fault<Bus::Error>> {
        let mut raw = [0u8; 1];
        bus.write_read(self.address, &[CALIB_STAT], &mut raw)
            .map_err(Fault::Bus)?;
        let state = CalibrationState::decode(raw[0]);
        self.state = Some(state);
        let newly = !self.converged && state.mag >= MAG_CALIBRATED;
        self.converged |= newly;
        Ok(newly)
    }

    /// The last CALIB_STAT read, None before the first.
    pub fn calibration_state(&self) -> Option<CalibrationState> {
        self.state
    }

    /// Has the magnetometer ever converged this session, or was a profile restored: the latch, which the live register
    /// regularly disagrees with (it regresses while the offsets stay).
    pub fn calibrated(&self) -> bool {
        self.converged
    }

    /// The latch came from a restored profile.
    pub fn restored(&self) -> bool {
        self.restored
    }

    /// What is still owed (bno055.calibration): [`owed`] of this part.
    pub fn owed(&self) -> Option<String> {
        owed(self.state, self.converged, self.saved)
    }

    /// The learned profile, for `calibrate` to save (bno055._save_profile): CONFIG mode for the read, NDOF again
    /// afterwards whatever happened, so a failed read never leaves the fusion off. The fusion pauses ~50 ms: a ground
    /// action, never on the poll path. Marks the profile saved only when the firmware says it persisted it
    /// ([`Bno055::mark_saved`]).
    pub fn read_profile<Bus: I2c>(
        &mut self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
    ) -> Result<[u8; PROFILE_BYTES], Fault<Bus::Error>> {
        let read = self.set_mode(bus, delay, MODE_CONFIG).and_then(|()| {
            let mut profile = [0u8; PROFILE_BYTES];
            bus.write_read(self.address, &[PROFILE], &mut profile)
                .map_err(Fault::Bus)
                .map(|()| profile)
        });
        let back = self.set_mode(bus, delay, MODE_NDOF);
        let profile = read?;
        back?;
        Ok(profile)
    }

    /// The firmware persisted the profile: nothing more is owed.
    pub fn mark_saved(&mut self) {
        self.saved = true;
    }

    /// The learned profile is saved (restored, or `calibrate` saved it).
    pub fn saved(&self) -> bool {
        self.saved
    }

    /// Write a saved profile back (bno055._restore_profile): CONFIG mode, the profile, NDOF again; sets the latch and
    /// the saved flag, so a boot with valid offsets owes nothing.
    pub fn restore_profile<Bus: I2c>(
        &mut self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
        profile: &[u8; PROFILE_BYTES],
    ) -> Result<(), Fault<Bus::Error>> {
        self.set_mode(bus, delay, MODE_CONFIG)?;
        let mut write = [0u8; PROFILE_BYTES + 1];
        write[0] = PROFILE;
        write[1..].copy_from_slice(profile);
        let written = bus.write(self.address, &write).map_err(Fault::Bus);
        self.set_mode(bus, delay, MODE_NDOF)?;
        written?;
        self.converged = true;
        self.restored = true;
        self.saved = true;
        Ok(())
    }

    /// The on-demand self-test (bno055.probe): the identity reads back, then one sample.
    pub fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<Sample, Fault<Bus::Error>> {
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        self.read(bus)
    }

    /// Classify a failed setup at the wire level (bno055.diagnose through the one-byte id read).
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String {
        id_classify(self.identity(bus).ok(), IDENTITY)
    }

    /// Switch the operating mode and wait it out.
    fn set_mode<Bus: I2c>(
        &mut self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
        mode: u8,
    ) -> Result<(), Fault<Bus::Error>> {
        // a fusion restarting reads zeros first: judged Starting from here (a failed write may have switched it too)
        self.starting |= mode == MODE_NDOF;
        bus.write(self.address, &[OPR_MODE, mode]).map_err(Fault::Bus)?;
        delay.delay_ms(MODE_SETTLE_MS);
        Ok(())
    }
}

impl<Bus: I2c> Device<Bus> for Bno055 {
    type BusError = Bus::Error;

    /// The identity, CONFIG mode, normal power, then NDOF (bno055.setup's bus half; the profile is restored after,
    /// by the firmware, from its NVS: [`Bno055::restore_profile`]).
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        self.set_mode(bus, delay, MODE_CONFIG)?;
        bus.write(self.address, &[PWR_MODE, POWER_NORMAL]).map_err(Fault::Bus)?;
        self.set_mode(bus, delay, MODE_NDOF)
    }

    /// NDOF again (plan B had no rearm: a reset part comes back in CONFIG mode, its fusion off, reading zeros).
    fn rearm(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.set_mode(bus, delay, MODE_NDOF)
    }
}

#[cfg(test)]
mod tests;
