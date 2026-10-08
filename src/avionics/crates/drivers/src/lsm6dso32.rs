//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The LSM6DSO32 6-DoF IMU (src/glider/drivers/lsm6dso32.py): the PRIMARY raw `accel` and the primary gyro `rate`.
//! A +/-32 g accel range covers the 8-12 g boost without clipping and still resolves ~1 mg for the airspeed
//! integrator; the gyro runs at +/-2000 dps. Both at 104 Hz with block data update, so the 12 output bytes at
//! OUTX_L_G (gyro x..z, then accel x..z, contiguous, signed little-endian) never mix two conversions.
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
//! NEW CONVERSIONS ONLY (avionics-port.md M6, M9 D1). The flight read is [`Lsm6dso32::read_fresh`]: ONE 16-byte
//! burst from STATUS_REG -- the status, the reserved 0x1F, OUT_TEMP, then the same 12 output bytes -- so the
//! data-available flags and the sample come out of one chip-select frame. The firmware pushes `accel` only on a fresh
//! accel half and `rate` only on a fresh gyro half, writes the row on either, and calls the part unhealthy after
//! [`STALE_LIMIT`] ticks in a row with neither (avionics-port.md §6's VL53 rule: a repeated value is never
//! republished as fresh). Plan B pushed every pass, but its pass was slower than the ODR; the 100 Hz tick is not, so
//! without the flags a part slower than the tick would publish an old sample as a new one. Never fresh, and never
//! stored: an impossible status (a bit the datasheet fixes at 0, as the 0xFF of an SPI bus nothing drives), so a part
//! that lost power, chip-select or MISO goes stale the same way; and a burst whose 12 output bytes are ALL zero (a
//! chip-select that never asserts reads 16 x 0x00). The zero guard is the whole frame, not a half: a well-biased gyro
//! at rest reads (0, 0, 0).
//!
//! THE FLAG DECIDES; THE BYTES ONLY CORRECT IT ([`crate::freshness::Flagged`], D1). One frame keeps the window
//! between the flags and the output small, not empty: a conversion that lands after the status byte reads with its
//! flag clear but its bytes changed. That half is a raced conversion, pushed once and marked raced, where M6 lost it
//! (a 30 ms gap between rows, measured at 9.2 and 8.4 a minute on the taster and 7F). If that read leaves its flag
//! set, the next read finds the flag over the same bytes: the duplicate, skipped. A flagged half is otherwise never
//! called stale for identical bytes, since at rest a new gyro conversion often repeats the last one bit for bit. The
//! residual is a raced conversion whose half is bit-identical to the one before: still lost, now counted (the
//! [`Fresh`] `identical` marks), not assumed. The driver stores the last read's 12 bytes and a raced mark per half;
//! setup and the re-arm forget them, and the firmware calls [`Lsm6dso32::forget`] when it re-adds the bus handle or
//! resumes reads it skipped, so the first read after any of them is judged by its flag alone. A bare
//! [`Lsm6dso32::read`] (the probe, setup's clearing read) clears data-ready without storing anything, so a conversion
//! it took from under the flags reads as raced on the next burst, and is pushed: it never was.
//!
//! THE PART'S OWN ODR. 104 Hz is nominal: the conversions run off the part's own oscillator, whose per-die offset
//! INTERNAL_FREQ_FINE reports ([`Lsm6dso32::frequency_fine`], [`odr_hz`], 0.15 % per LSB). A part slower than the
//! 100 Hz tick has no new conversion behind a tick now and then (the taster measured about 1 % of rows with
//! `irq_runs` 0, so it runs at about 99 Hz); a faster one always has, and skips about 4 conversions a second
//! instead. The firmware logs the ODR once at bring-up, so each board says which case it is.
//!
//! INT1 AND `irq_runs`. The control tick paces the reads: the part is read at the top of every tick, and the config's
//! `period_ms` is unused. INT1 data-ready is evidence, not a pacer: the firmware's GPIO interrupt only counts its
//! rising edges, and the tick swaps the count out just BEFORE the read -- plan B's order, whose run loop waited for
//! the edge, then read. With data-ready latched (COUNTER_BDR_REG1 `dataready_pulsed` 0, the reset default plan B
//! never wrote) INT1 stays high until the burst reads the output, so a read with a new conversion behind it sees 1;
//! 0 means no new conversion, or a dead line; and a 0 followed by a 2 is a conversion landing in the microseconds
//! between the swap and the burst. The latch also means a late tick never piles up edges: more than 2 needs another
//! read of the output (a probe) to clear data-ready between two swaps. The firmware arms the interrupt AFTER setup,
//! on the control thread: a conversion that lands between setup's clearing read and the arming leaves INT1 high
//! with no edge to count, so the first tick's read is what clears it, and the first row's `irq_runs` is 0.
//!
//! A dead INT line used to masquerade as a healthy sensor: plan B's fallback timeout became the sampling rate,
//! measured at exactly 2.0 Hz on the bench with the driver reporting healthy, while `rate` has a 20 ms freshness
//! window and no backup on a board without a BMI323 -- so the PID's D term quietly degraded to
//! derivative-on-error. On the tick a dead line no longer slows the sampling, but it still hides a broken wire, so
//! [`Lsm6dso32::wake`] keeps plan B's verdict as the operator's view: [`INTERRUPT_SILENT_LIMIT`] reads in a row
//! without an edge mean the line is dead, and one edge means it is back.
//!
//! Units (decision D18): accel in f32 g and gyro in f32 degrees per second for the channels. The telemetry row
//! keeps plan B's integers: the gyro columns are the RAW centidegree-per-second integer `raw x 70 x 100 // 1000`
//! (never a formatted decimal; the host tools divide by 100 when they render) next to the accel in g and
//! `irq_runs`.
//!
//! Not here (firmware glue): the GPIO interrupt and its edge count, the stale run and its log line, the channel
//! push, the telemetry stream, the probe's log lines, the bring-up's ODR line and the inspect JSON.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use embedded_hal::spi::{Operation, SpiDevice};
use serde_json::Value;

use crate::commons::id_classify;
use crate::device::{Device, Fault, Strikes};
pub use crate::freshness::Fresh;
use crate::freshness::{Flagged, HALF_LENGTH};

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

/// STATUS_REG: the data-available flags, XLDA (bit 0, accel) and GDA (bit 1, gyro), each set by a new conversion
/// and cleared when its output is read. The registers after it run on to the output bytes: 0x1F (reserved), OUT_TEMP
/// at 0x20..0x21, then OUTX_L_G, so one burst from here reads the flags and the sample together.
pub const STATUS_REG: u8 = 0x1E;

/// INTERNAL_FREQ_FINE: the part's own ODR offset from nominal, two's complement, 0.15 % per LSB ([`odr_hz`]).
pub const INTERNAL_FREQ_FINE: u8 = 0x63;

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

/// The bytes the status burst reads before the output: STATUS_REG, the reserved 0x1F and OUT_TEMP (two bytes).
const STATUS_HEADER_LENGTH: usize = 4;

/// The status burst from STATUS_REG: the header, then the 12 output bytes.
const STATUS_FRAME_LENGTH: usize = STATUS_HEADER_LENGTH + FRAME_LENGTH;

/// STATUS_REG bit 0, XLDA: a new accel conversion is waiting.
const ACCEL_DATA_AVAILABLE: u8 = 0x01;

/// STATUS_REG bit 1, GDA: a new gyro conversion is waiting.
const GYRO_DATA_AVAILABLE: u8 = 0x02;

/// STATUS_REG bits 7..3, which the datasheet fixes at 0 (only TDA, GDA and XLDA exist): a status with any of them
/// set was not read from the part, it is a bus nothing drives (an SPI part that lost power or chip-select, or a
/// broken MISO, reads 0xFF with no bus error).
const STATUS_RESERVED: u8 = 0xF8;

/// Consecutive reads with no new conversion (neither half fresh) before the firmware calls the part unhealthy: one
/// such tick is a part slower than the tick, three in a row are a part that stopped converting.
pub const STALE_LIMIT: u32 = 3;

/// Where the gyro half sits in the 12 output bytes (gyro x, y, z come first out of this part).
const GYRO_HALF: usize = 0;

/// Where the accel half sits in the 12 output bytes.
const ACCEL_HALF: usize = HALF_LENGTH;

/// The base rate every ODR setting divides, in Hz (the datasheet's 6667).
const BASE_RATE_HZ: f32 = 6667.0;

/// The datasheet's ODR_Coeff for the 104 Hz setting [`CONFIGURATION`] writes: 6667 / 64 = 104.17 Hz nominal.
const ODR_COEFFICIENT: f32 = 64.0;

/// INTERNAL_FREQ_FINE's step: 0.15 % of the nominal ODR per LSB.
const FREQUENCY_FINE_STEP: f32 = 0.0015;

/// Accel sensitivity at +/-32 g: 0.976 mg per count (datasheet), in g.
pub const ACCEL_G_PER_COUNT: f32 = 0.000976;

/// Gyro sensitivity at +/-2000 dps: 70 millidegrees per second per count (datasheet).
pub const GYRO_MILLIDEGREES_PER_SECOND_PER_COUNT: i32 = 70;

/// Consecutive reads with no INT1 edge behind them before the line is called dead rather than jittery.
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

/// One half of the 12 output bytes, from `start`.
fn half(output: &[u8; FRAME_LENGTH], start: usize) -> [u8; HALF_LENGTH] {
    let mut bytes = [0u8; HALF_LENGTH];
    bytes.copy_from_slice(&output[start..start + HALF_LENGTH]);
    bytes
}

/// The part's own output data rate at the configured 104 Hz setting, in Hz, from its INTERNAL_FREQ_FINE byte: the
/// datasheet's (6667 + 0.0015 x fine x 6667) / ODR_Coeff with ODR_Coeff 64, that is 104.17 Hz x (1 + 0.15 % per
/// LSB). 0 is the nominal 104.17 Hz; -33 is about 99.0 Hz, and the extremes are about 84.2 Hz (-128) and 124.0 Hz
/// (127).
pub fn odr_hz(fine: i8) -> f32 {
    BASE_RATE_HZ * (1.0 + FREQUENCY_FINE_STEP * f32::from(fine)) / ODR_COEFFICIENT
}

/// One LSM6DSO32, reached through the window `F` ([`OverSpi`] or [`OverI2c`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Lsm6dso32<F> {
    /// The register window.
    interface: F,
    /// Whether an INT1 wire is declared: setup routes data-ready to it, and only then.
    interrupt_wired: bool,
    /// The run of reads without an INT1 edge, toward the silent verdict.
    strikes: Strikes,
    /// The operator view: the INT line went silent (the tick still paces the reads; the wire is what is lost).
    interrupt_silent: bool,
    /// The status bursts' judge: the last burst's output bytes and its raced marks (see the module notes).
    flags: Flagged,
}

impl<F> Lsm6dso32<F> {
    /// A driver on this window; `interrupt_wired` when the config's `int_pin` resolves to a GPIO.
    pub fn new(interface: F, interrupt_wired: bool) -> Lsm6dso32<F> {
        Lsm6dso32 {
            interface,
            interrupt_wired,
            strikes: Strikes::default(),
            interrupt_silent: false,
            flags: Flagged::default(),
        }
    }

    /// Forget the stored output bytes and the raced marks, so the next [`Lsm6dso32::read_fresh`] is judged by its
    /// flags alone. Setup and the re-arm forget by themselves; the firmware calls it when it re-adds the bus handle (a
    /// bustune rung) and when it resumes reads it skipped (`imu mute`).
    pub fn forget(&mut self) {
        self.flags.forget();
    }

    /// Whether an INT1 wire is declared (inspect's `interrupt`).
    pub fn interrupt_wired(&self) -> bool {
        self.interrupt_wired
    }

    /// Whether the INT line is called dead (inspect's `interrupt_silent`): the tick still samples, so `rate` stays
    /// fresh; what is lost is the evidence that the INT1 wire works, so the operator sees it.
    pub fn interrupt_silent(&self) -> bool {
        self.interrupt_silent
    }

    /// Account one read: `interrupt_runs` INT1 edges arrived since the previous swap (see the module notes: 0 = no
    /// new conversion, or a dead line; 1 = healthy; 2 after a 0 = a conversion that landed between the swap and the
    /// burst; more = another read of the output (a probe) cleared data-ready between two swaps: with data-ready
    /// latched, a late tick never piles up edges); the telemetry row carries the same count as its `irq_runs` column.
    ///
    /// Plan B's run-loop verdict: [`INTERRUPT_SILENT_LIMIT`] reads without an edge IN A ROW set the silent flag, and
    /// the return is true exactly once then, for the firmware's one `INT1 silent` log line. A single later edge
    /// clears it, so a merely noisy line costs nothing permanent. As in plan B the count runs with or without a wire
    /// declared -- kept on purpose, and pinned by the golden fixture's unwired wake cases: plan B's run() never reads
    /// the wire here (the BMI323 driver later learnt to skip it without one; on every board the LSM6DSO32's INT1 is
    /// wired, so an unwired one being called silent is the truthful verdict of a part with no edge to show).
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

    /// Read one sample and which of its halves are new: ONE 16-byte burst from STATUS_REG (the status, the reserved
    /// 0x1F, OUT_TEMP, then the 12 output bytes). Each half is judged by its flag, XLDA (bit 0) for the accel and GDA
    /// (bit 1) for the gyro, corrected by its bytes against the last burst's (the module notes; TDA, bit 2, is
    /// ignored). Never fresh, and the stored bytes left as they were: a status with a [`STATUS_RESERVED`] bit set
    /// (impossible from a live part, so a dead SPI part goes stale and turns unhealthy after [`STALE_LIMIT`] ticks
    /// instead of publishing 0xFF bytes forever), or all 12 output bytes zero. The reserved byte and the temperature
    /// are discarded, and the output bytes decode exactly as [`Lsm6dso32::read`]'s. Over SPI that is one chip-select
    /// frame, `0x80 | 0x1E` then 16 bytes clocked in; over I2C, the register then a 16-byte read after a repeated
    /// start. Like `read`, it clears data-ready.
    pub fn read_fresh<Bus>(&mut self, bus: &mut Bus) -> Result<(Fresh, Sample), Fault<F::BusError>>
    where
        F: Interface<Bus>,
    {
        let mut frame = [0u8; STATUS_FRAME_LENGTH];
        self.interface.read(bus, STATUS_REG, &mut frame).map_err(Fault::Bus)?;
        let mut output = [0u8; FRAME_LENGTH];
        output.copy_from_slice(&frame[STATUS_HEADER_LENGTH..]);
        let status = frame[0];
        let fresh = if status & STATUS_RESERVED != 0 || output == [0u8; FRAME_LENGTH] {
            Fresh::default()
        } else {
            self.flags.judge(
                status & ACCEL_DATA_AVAILABLE != 0,
                &half(&output, ACCEL_HALF),
                status & GYRO_DATA_AVAILABLE != 0,
                &half(&output, GYRO_HALF),
            )
        };
        Ok((fresh, Sample::decode(&output)))
    }

    /// Read INTERNAL_FREQ_FINE once (the bring-up's ODR line, through [`odr_hz`]): the part's own ODR offset, a
    /// two's-complement byte.
    pub fn frequency_fine<Bus>(&self, bus: &mut Bus) -> Result<i8, Fault<F::BusError>>
    where
        F: Interface<Bus>,
    {
        let mut fine = [0u8; 1];
        self.interface
            .read(bus, INTERNAL_FREQ_FINE, &mut fine)
            .map_err(Fault::Bus)?;
        Ok(i8::from_le_bytes(fine))
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
    /// clearing read. The firmware arms its edge interrupt after this returns (the module notes say why the first
    /// row's `irq_runs` is then 0).
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
    /// identity after every round is "not an LSM6DSO32 at this chip-select / address". The stored output bytes are
    /// forgotten first ([`Lsm6dso32::forget`]).
    fn setup(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<F::BusError>> {
        self.forget();
        self.bring_up(bus)
    }

    /// The whole bring-up again. Plan B gave this part no re-arm, since on SPI no general call reaches it; the
    /// port re-arms every device, and a reset LSM6DSO32 comes back powered down (CTRL1_XL and CTRL2_G at 0), so the
    /// only correct re-arm is the bring-up itself, the identity check included, the stored output bytes forgotten.
    fn rearm(&mut self, bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<F::BusError>> {
        self.forget();
        self.bring_up(bus)
    }
}

#[cfg(test)]
mod tests;
