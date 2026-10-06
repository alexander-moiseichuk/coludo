//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The LSM6DSO32 6-DoF IMU (src/glider/drivers/lsm6dso32.py): the PRIMARY raw `accel` and the primary gyro `rate`.
//! A +/-32 g accel range covers the 8-12 g boost without clipping and still resolves ~1 mg for the airspeed
//! integrator; the gyro runs at +/-2000 dps. Both at 104 Hz with block data update, so the 12-byte burst at
//! OUTX_L_G (gyro x..z, then accel x..z, contiguous, signed little-endian) never mixes two conversions.
//!
//! Wired on SPI (its own chip-select, 4-wire mode 3) for clean high-rate reads; I2C at 0x6A also works when the
//! config names bus 'i2c'. Multi-byte reads auto-increment through CTRL3_C.IF_INC, so the SPI command byte is the
//! register with the read flag and NO multi-byte address bit ([`OverSpi`]).
//!
//! WRITE-THEN-VERIFY bring-up, up to [`IDENTITY_ROUNDS`] rounds of "write CTRL3_C, read WHO_AM_I". It used to be 5
//! bare reads, and the shape was wrong as well as the count. MEASURED on TMS-7E: with the part out of step, WHO_AM_I
//! returns 0x00 for 30 consecutive reads and stays at 0x00 -- reading does not recover it, however long you read.
//! Writing CTRL3_C does, immediately. The interface has to be PINNED, and a write issued before the part is
//! listening is simply lost, so the write is retried rather than issued once up front. How far that risk reaches
//! was measured too: interrupting the driver mid-traffic and rebooting (12 tries) and a bus retune at 1, 8 and
//! 5 MHz (4 for 4) all recovered on the first read with the old one-shot strategy. The only reproducer found is a
//! peripheral CREATED AND DEINITIALISED under the part, which a pin test does and no flight path does. So this is
//! not a fix for a flight failure; it is the right shape for the operation, at microseconds a round.
//!
//! INT1 data-ready drives the sampling where a pin is wired: the firmware's GPIO interrupt wakes the control
//! thread, and how many edges arrived per wake (`irq_runs`) is the operator's evidence. A dead INT line used to
//! masquerade as a healthy sensor: plan B's fallback timeout became the sampling rate, measured at exactly 2.0 Hz
//! on the bench with the driver reporting healthy, while `rate` has a 20 ms freshness window and no backup on a
//! board without a BMI323 -- so the PID's D term quietly degraded to derivative-on-error. [`Lsm6dso32::wake`]
//! keeps plan B's verdict: [`INTERRUPT_SILENT_LIMIT`] timed-out wakes in a row mean the line is dead, and one edge
//! means it is back.
//!
//! Units (decision D18): accel in f32 g and gyro in f32 degrees per second for the channels. The telemetry row
//! keeps plan B's integers: the gyro columns are the RAW centidegree-per-second integer `raw x 70 x 100 // 1000`
//! (never a formatted decimal; the host tools divide by 100 when they render) next to the accel in g and
//! `irq_runs`.
//!
//! Not here (firmware glue): the GPIO interrupt and its wake count, the channel push, the telemetry stream, the
//! probe's log lines and the inspect JSON. The firmware arms its INT1 edge interrupt BEFORE calling setup: setup
//! ends with the clearing read, and plan B's rule is that the interrupt is armed first, so a conversion that landed
//! during configuration is a clean rising edge rather than a missed one (INT1 stays low until setup routes
//! data-ready to it, so arming early cannot see a spurious edge).

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use embedded_hal::spi::{Operation, SpiDevice};
use serde_json::Value;

use crate::commons::id_classify;
use crate::device::{Device, Fault, Strikes};

/// The default I2C address (SDO low; 0x6B when high), for a config that names bus 'i2c'.
pub const ADDRESS: u8 = 0x6A;

/// WHO_AM_I: the identity register.
pub const WHO_AM_I: u8 = 0x0F;

/// CTRL1_XL: accel output data rate and full scale.
pub const CTRL1_XL: u8 = 0x10;

/// CTRL2_G: gyro output data rate and full scale.
pub const CTRL2_G: u8 = 0x11;

/// CTRL3_C: BDU (block data update), IF_INC (register auto-increment) and SIM (3- or 4-wire SPI).
pub const CTRL3_C: u8 = 0x12;

/// INT1_CTRL: what drives the INT1 pin (accel data-ready is bit 0).
pub const INT1_CTRL: u8 = 0x0D;

/// OUTX_L_G: the first of the 12 output bytes, gyro x..z then accel x..z, signed little-endian and contiguous, so
/// one burst fetches both.
pub const OUTX_L_G: u8 = 0x22;

/// What WHO_AM_I reads on an LSM6DSO32.
pub const IDENTITY: u8 = 0x6C;

/// How many write-then-verify rounds setup spends before deciding the part is absent (see the module notes): each
/// is two short transfers, once, at setup, so an absent part is reported a few hundred microseconds later than a
/// one-shot read would.
pub const IDENTITY_ROUNDS: u32 = 64;

/// One register write of a bring-up table: (register, value).
pub type RegisterWrite = (u8, u8);

/// The write that PINS the interface, repeated before each identity read: CTRL3_C = 0x44, that is BDU = 1 (a
/// burst never mixes two conversions), IF_INC = 1 (multi-byte reads auto-increment) and SIM = 0 (4-wire SPI).
pub const PIN_INTERFACE: RegisterWrite = (CTRL3_C, 0x44);

/// The configuration written once the identity is verified, in plan B's order:
///  * CTRL1_XL = 0x44: accel 104 Hz (ODR 0100), FS_XL = 01, which is +/-32 g on this part;
///  * CTRL2_G = 0x4C: gyro 104 Hz (ODR 0100), FS_G = 11, +/-2000 dps.
pub const CONFIGURATION: [RegisterWrite; 2] = [(CTRL1_XL, 0x44), (CTRL2_G, 0x4C)];

/// Accel data-ready routed to INT1 (INT1_CTRL bit 0), written only when an INT1 wire is declared.
pub const ROUTE_INTERRUPT: RegisterWrite = (INT1_CTRL, 0x01);

/// The output burst: gyro x, y, z then accel x, y, z, two bytes each.
pub const FRAME_LENGTH: usize = 12;

/// Accel sensitivity at +/-32 g: 0.976 mg per count (datasheet), in g.
pub const ACCEL_G_PER_COUNT: f32 = 0.000976;

/// Gyro sensitivity at +/-2000 dps: 70 millidegrees per second per count (datasheet).
pub const GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT: i32 = 70;

/// Consecutive timed-out wakes (no INT1 edge) before the line is called dead rather than jittery.
pub const INTERRUPT_SILENT_LIMIT: u32 = 3;

/// The largest 7-bit I2C address: anything above it cannot be put on the wire.
const MAXIMUM_ADDRESS: u8 = 0x7F;

/// The SPI command byte's read flag (the LSM/ADXL convention: bit 7 set reads, clear writes).
const SPI_READ: u8 = 0x80;

/// Hundredths per unit: the telemetry gyro column is centidegrees per second.
const HUNDREDTHS: i32 = 100;

/// Milli-units per unit.
const THOUSANDTHS: i32 = 1000;

/// How the part is reached: plan B's register window over whichever bus the config names (lsm6dso32._transport).
///
/// The two windows read and write the same registers; only the framing differs. The LSM6DSO32 is generic over it,
/// so one driver serves both buses.
pub trait Interface<Bus> {
    /// The transport's own error (plan B's `OSError`).
    type BusError;

    /// Write one register.
    fn write(&self, bus: &mut Bus, register: u8, value: u8) -> Result<(), Self::BusError>;

    /// Read consecutive registers from `register` on into `buffer` (the part auto-increments through IF_INC).
    fn read(&self, bus: &mut Bus, register: u8, buffer: &mut [u8]) -> Result<(), Self::BusError>;
}

/// The SPI window (plan B's spibus device with `mb_bit=None`): the `SpiDevice` frames each transfer with the part's
/// own chip-select. A read sends `0x80 | register` then clocks the data in; a write sends the register then the
/// value. No multi-byte address bit: the part auto-increments from CTRL3_C.IF_INC, and setting one would address a
/// different register.
///
/// The bus owner discards the first transfers after the SPI peripheral appears (plan B measured 1 to 11 stale 0x00
/// reads on a fresh peripheral; avionics-port.md M5 "discard-N"), so this window has no resync of its own.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OverSpi;

impl<Bus: SpiDevice> Interface<Bus> for OverSpi {
    type BusError = Bus::Error;

    /// One chip-select frame: the register (read flag clear), then the value.
    fn write(&self, bus: &mut Bus, register: u8, value: u8) -> Result<(), Bus::Error> {
        bus.write(&[register, value])
    }

    /// One chip-select frame: the register with the read flag, then the data clocked in.
    fn read(&self, bus: &mut Bus, register: u8, buffer: &mut [u8]) -> Result<(), Bus::Error> {
        bus.transaction(&mut [Operation::Write(&[SPI_READ | register]), Operation::Read(buffer)])
    }
}

/// The I2C window at an address (plan B's i2cbus device): a register write is one write of the register and the
/// value (writeto_mem); a register read writes the register, then reads after a repeated start
/// (readfrom_mem_into).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OverI2c {
    /// The 7-bit address ([`ADDRESS`], or 0x6B with SDO high).
    pub address: u8,
}

impl<Bus: I2c> Interface<Bus> for OverI2c {
    type BusError = Bus::Error;

    /// One write: the register, then the value.
    fn write(&self, bus: &mut Bus, register: u8, value: u8) -> Result<(), Bus::Error> {
        bus.write(self.address, &[register, value])
    }

    /// The register, a repeated start, then the data.
    fn read(&self, bus: &mut Bus, register: u8, buffer: &mut [u8]) -> Result<(), Bus::Error> {
        bus.write_read(self.address, &[register], buffer)
    }
}

/// Which bus the config names for the part (plan B's `_transport()` dispatch).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Transport {
    /// No `bus`, or `bus` 'spi': the firmware builds an [`OverSpi`] window on the part's chip-select.
    Spi,
    /// Any other `bus`, a null or a number included (plan B's `config.get('bus', 'spi') == 'spi'` is false for every
    /// one of them): an [`OverI2c`] window at `addr`. The bus lookup that follows the dispatch then finds no
    /// `buses.<bus>` for a value that names none, and the part reports not connected, as plan B's `config.bus` does.
    I2c {
        /// `addr`, default [`ADDRESS`].
        address: u8,
    },
}

impl Transport {
    /// The transport a config block names: `bus` (absent means 'spi'; see [`Transport::I2c`] for every other value)
    /// and, for I2C, `addr` (default [`ADDRESS`]; a value that is not a 7-bit address, 0x00..=0x7F, takes the
    /// default too, where plan B would have failed its first transfer).
    pub fn from_config(config: &Value) -> Transport {
        match config.get("bus") {
            None => return Transport::Spi,
            Some(Value::String(bus)) if bus == "spi" => return Transport::Spi,
            Some(_) => {}
        }
        let address = config
            .get("addr")
            .and_then(Value::as_u64)
            .filter(|address| *address <= u64::from(MAXIMUM_ADDRESS))
            .map_or(ADDRESS, |address| address as u8);
        Transport::I2c { address }
    }
}

/// One output burst, as the part reports it: raw signed counts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sample {
    /// Gyro x, y, z counts (70 mdps each).
    pub gyro: [i16; 3],
    /// Accel x, y, z counts (0.976 mg each).
    pub accel: [i16; 3],
}

impl Sample {
    /// Decode the 12-byte burst: gyro x, y, z then accel x, y, z, each signed 16-bit little-endian (plan B's
    /// `struct.unpack('<hhhhhh')`). Gyro comes FIRST out of this part; the BMI323 puts accel first.
    pub fn decode(frame: &[u8; FRAME_LENGTH]) -> Sample {
        let word = |index: usize| i16::from_le_bytes([frame[2 * index], frame[2 * index + 1]]);
        Sample {
            gyro: [word(0), word(1), word(2)],
            accel: [word(3), word(4), word(5)],
        }
    }

    /// The accel in g: the `accel` channel and the telemetry's ax, ay, az.
    pub fn accel_g(&self) -> [f32; 3] {
        self.accel.map(|count| f32::from(count) * ACCEL_G_PER_COUNT)
    }

    /// The gyro in degrees per second: the `rate` channel, in the marks frame (x forward, y left, z up, through the
    /// board config's `axes` where a part is mounted otherwise). The (roll, pitch, yaw) rate is (gx, -gy, -gz):
    /// crates/control `attitude::rate_from_marks` maps it, for the attitude filter and the PID D term alike.
    pub fn rate_degrees_per_second(&self) -> [f32; 3] {
        self.gyro
            .map(|count| f32::from(count) * GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT as f32 / THOUSANDTHS as f32)
    }

    /// The gyro in centidegrees per second, plan B's integer to the digit (`raw x 70 x 100 // 1000`, floored): the
    /// telemetry's gx, gy, gz columns.
    pub fn rate_centidegrees_per_second(&self) -> [i32; 3] {
        self.gyro.map(|count| {
            (i32::from(count) * GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT * HUNDREDTHS).div_euclid(THOUSANDTHS)
        })
    }
}

/// One LSM6DSO32, reached through the window `F` ([`OverSpi`] or [`OverI2c`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Lsm6dso32<F> {
    /// The register window.
    interface: F,
    /// Whether an INT1 wire is declared: setup routes data-ready to it, and only then.
    interrupt_wired: bool,
    /// The run of timed-out wakes, toward the silent verdict.
    strikes: Strikes,
    /// The operator view: the INT line went silent and sampling runs on the fallback period.
    interrupt_silent: bool,
}

impl<F> Lsm6dso32<F> {
    /// A driver on this window; `interrupt_wired` when the config's `int_pin` resolves to a GPIO.
    pub fn new(interface: F, interrupt_wired: bool) -> Lsm6dso32<F> {
        Lsm6dso32 {
            interface,
            interrupt_wired,
            strikes: Strikes::default(),
            interrupt_silent: false,
        }
    }

    /// Whether an INT1 wire is declared (inspect's `interrupt`).
    pub fn interrupt_wired(&self) -> bool {
        self.interrupt_wired
    }

    /// Whether the INT line is called dead (inspect's `interrupt_silent`): a wired-but-DEAD INT1 is the dangerous
    /// state, where sampling continues, `rate` goes stale and the PID's D term degrades, so the operator sees it.
    pub fn interrupt_silent(&self) -> bool {
        self.interrupt_silent
    }

    /// Account one sampling wake: `interrupt_runs` INT1 edges arrived since the last (0 = the fallback period ran
    /// out, 1 = healthy, more = the loop was late and edges piled up, a scheduling symptom rather than a sensor one);
    /// the telemetry row carries the same count as its `irq_runs` column.
    ///
    /// Plan B's run-loop verdict: [`INTERRUPT_SILENT_LIMIT`] timed-out wakes IN A ROW set the silent flag, and the
    /// return is true exactly once then, for the firmware's one log line (`INT1 silent -- sampling on the <period>
    /// ms fallback`). A single later edge clears it, so a merely noisy line costs nothing permanent. As in plan B the
    /// count runs with or without a wire declared -- kept on purpose, and pinned by the golden fixture's unwired
    /// wake cases: plan B's run() never reads the wire here (the BMI323 driver later learnt to skip it without one;
    /// on every board the LSM6DSO32's INT1 is wired, so an unwired one being called silent is the truthful verdict
    /// of a part that is sampling on the fallback period).
    pub fn wake(&mut self, interrupt_runs: u32) -> bool {
        if self.strikes.strike(interrupt_runs == 0, INTERRUPT_SILENT_LIMIT) {
            self.interrupt_silent = true;
            return true;
        }
        if interrupt_runs > 0 {
            self.interrupt_silent = false;
        }
        false
    }

    /// Read WHO_AM_I once (the probe's first step).
    pub fn identity<Bus>(&self, bus: &mut Bus) -> Result<u8, Fault<F::BusError>>
    where
        F: Interface<Bus>,
    {
        let mut identity = [0u8; 1];
        self.interface.read(bus, WHO_AM_I, &mut identity).map_err(Fault::Bus)?;
        Ok(identity[0])
    }

    /// Read one sample: the 12-byte burst at OUTX_L_G, which also clears data-ready.
    pub fn read<Bus>(&self, bus: &mut Bus) -> Result<Sample, Fault<F::BusError>>
    where
        F: Interface<Bus>,
    {
        let mut frame = [0u8; FRAME_LENGTH];
        self.interface.read(bus, OUTX_L_G, &mut frame).map_err(Fault::Bus)?;
        Ok(Sample::decode(&frame))
    }

    /// The on-demand self-test (lsm6dso32.probe): WHO_AM_I reads back the LSM6DSO32's identity, then one sample
    /// reads. The firmware logs each step and folds the magnitude of the accel into the line.
    pub fn probe<Bus>(&self, bus: &mut Bus) -> Result<Sample, Fault<F::BusError>>
    where
        F: Interface<Bus>,
    {
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        self.read(bus)
    }

    /// Classify a failed setup at the wire level (lsm6dso32.diagnose through the bus window): read WHO_AM_I once
    /// and sort it with [`id_classify`] (0x00 is a chip-select not asserting, 0xFF a floating MISO).
    pub fn diagnose<Bus>(&self, bus: &mut Bus) -> String
    where
        F: Interface<Bus>,
    {
        id_classify(self.identity(bus).ok(), IDENTITY)
    }

    /// The write-then-verify bring-up (setup's bus half, and the re-arm): pin the interface and read WHO_AM_I, up
    /// to [`IDENTITY_ROUNDS`] times; then the [`CONFIGURATION`]; then, with INT1 wired, [`ROUTE_INTERRUPT`] and the
    /// clearing read, so the next conversion is a clean edge.
    ///
    /// A bus failure ends it at once, as plan B's single try around the loop does: only a wrong identity is retried.
    fn bring_up<Bus>(&self, bus: &mut Bus) -> Result<(), Fault<F::BusError>>
    where
        F: Interface<Bus>,
    {
        let mut read = 0;
        for _ in 0..IDENTITY_ROUNDS {
            let (register, value) = PIN_INTERFACE;
            self.interface.write(bus, register, value).map_err(Fault::Bus)?;
            read = self.identity(bus)?;
            if read == IDENTITY {
                break;
            }
        }
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        for (register, value) in CONFIGURATION {
            self.interface.write(bus, register, value).map_err(Fault::Bus)?;
        }
        if self.interrupt_wired {
            let (register, value) = ROUTE_INTERRUPT;
            self.interface.write(bus, register, value).map_err(Fault::Bus)?;
            self.read(bus)?;
        }
        Ok(())
    }
}

impl<Bus, F: Interface<Bus>> Device<Bus> for Lsm6dso32<F> {
    type BusError = F::BusError;

    /// The write-then-verify bring-up (see the module notes). An `Err` is plan B's `setup() -> False`: a wrong
    /// identity after every round is "not an LSM6DSO32 at this chip-select / address".
    fn setup(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<F::BusError>> {
        self.bring_up(bus)
    }

    /// The whole bring-up again. Plan B gave this part no re-arm, since on SPI no general call reaches it; the
    /// port re-arms every device, and a reset LSM6DSO32 comes back powered down (CTRL1_XL and CTRL2_G at 0), so the
    /// only correct re-arm is the bring-up itself, the identity check included.
    fn rearm(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<F::BusError>> {
        self.bring_up(bus)
    }
}

#[cfg(test)]
mod tests;
