//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMI323 6-axis IMU on the SEN0697 (src/glider/drivers/bmi323.py): accel and gyro on the shared I2C bus,
//! publishing the SAME channels as the LSM6DSO32 -- `accel` in g and `rate` in degrees per second -- at priority 1
//! behind it, so the two are interchangeable sources for one channel and the attitude filter runs off whichever
//! the databoard hands it. `rate` was a single-source channel on every board before this part: a dead LSM6DSO32
//! took the PID's D term with it. Two independent 6-axis parts on different buses (one SPI, one I2C) remove that.
//!
//! Two details the register map does not make obvious:
//!  * every I2C register read returns TWO DUMMY BYTES before the data ([`DUMMY_BYTES`]), so a 6-word burst is a
//!    14-byte read whose payload starts at offset 2. Reading it like an LSM6DSO32 returns plausible nonsense, and
//!    a one-byte chip-id read returns a dummy byte: the first hardware bring-up rejected a perfectly good part that
//!    way, while the BMP581 beside it (no dummy bytes) came up fine;
//!  * registers are 16-bit and written least significant byte first.
//!
//! START-UP: the gyro reports 0x8000 ([`NO_SAMPLE`]) until it has a sample, measured at ~20 ms on the bench. Setup
//! waits it out rather than publish the marker, which scales to -2000 dps: a start-up artefact arriving in `rate` as
//! a full-scale rotation is the kind of thing a control loop acts on before anyone sees it. The accel's -32768 is
//! a legitimate -16 g rail, not a marker, and is never refused.
//!
//! Register values are Bosch's own (bmi3_defs.h). Scaling: +/-16 g is 2048 counts per g, and +/-2000 dps is
//! 2000/32768 dps per count, both exact in f32 (powers of two over small integers). The telemetry keeps plan B's
//! integer gyro columns: centidegrees per second, `raw x 2000 x 100 // 32768`, floored.
//!
//! MOUNTING: the part is mounted per its marks, x forward, y left, z up, and publishes in that frame. It is a
//! property of how the module is glued in, not of the part; the board config's optional `axes` key states an
//! exception ([`crate::axes`]). The consumers map the marks frame into their own in one place, crates/control
//! `attitude::Readings::from_marks` (the (roll, pitch, yaw) rate is (gx, -gy, -gz), `attitude::rate_from_marks`,
//! for the filter and the PID D term alike): plan B's gx -> roll, gy -> pitch, gz -> yaw read a part mounted per
//! its marks nose-down-positive and yawing left.
//!
//! Deliberate differences from plan B, both in the re-arm (the port's rule: a re-arm restores everything setup
//! wrote, as for the INA226):
//!  * it routes data-ready to INT1 again when the wire is declared. Plan B's rearm() restored the two sensor
//!    configurations only, so a general call left INT1 unrouted and the driver fell back to polling after 20
//!    silent wakes, reporting `interrupt_silent`;
//!  * a gyro that never leaves start-up after the re-arm is reported ([`Fault::Timeout`]) for the owner's log.
//!    Plan B awaited the start-up and dropped the verdict.
//!
//! NEW CONVERSIONS ONLY (avionics-port.md M9 D1b), the LSM6DSO32's rule ([`crate::freshness::Flagged`]). The flight
//! read is [`Bmi323::read_fresh`]: ONE 16-byte burst from STATUS 0x02 -- the dummy bytes, the STATUS word, then the
//! six data words -- where plan B read 14 bytes from ACC_DATA_X on the data-ready edge. MEASURED on the taster and
//! 7F (M9 stage 0): STATUS bit 7 is drdy_acc and bit 6 drdy_gyr (bit 5 drdy_temp, bit 0 por_detected, set once
//! after power-on), both rise together every 10.0 ms at the configured 100 Hz, exactly on the bursts whose data
//! changed, and reading STATUS clears them. So the flag in a burst describes that burst, and a conversion landing
//! between the STATUS word and the data reads as raced (flag clear, bytes changed: pushed once) with its flag left
//! set for the next burst over the same bytes (the duplicate: skipped). A flagged half is never called stale for
//! identical bytes: the gyro's LSB is 0.061 dps, so at rest two conversions in a row often repeat. Never fresh, and
//! never stored: a frame with any gyro axis at [`NO_SAMPLE`] or all three accel axes at it (a reset part reads
//! 0x8000 frames, which changed bytes would otherwise push once as -16 g and -2000 dps; one accel axis at -32768 is
//! the -16 g rail, fresh). Setup and the re-arm forget the stored bytes. The bare [`Bmi323::read`] stays plan B's
//! 14 bytes from ACC_DATA_X (setup's start-up looks and clearing read, the probe), so setup and the re-arm make plan
//! B's transactions; the STATUS burst is the golden fixture's one listed divergence. The firmware calls the part
//! unhealthy after [`STALE_LIMIT`] reads in a row with neither half fresh.
//!
//! Not here (firmware glue): the INT1 GPIO interrupt and its wake count, the channel push, the telemetry stream, the
//! probe's log lines and the inspect JSON. As for the LSM6DSO32, the firmware arms the GPIO edge interrupt before
//! setup, which ends with the clearing read.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::device::{Device, Fault, Strikes};
pub use crate::freshness::Fresh;
use crate::freshness::{Flagged, HALF_LENGTH};

/// The default I2C address (0x68 when the address pin is pulled the other way).
pub const ADDRESS: u8 = 0x69;

/// CHIP_ID: the identity register.
pub const CHIP_ID: u8 = 0x00;

/// STATUS: the data-ready flags, drdy_acc (bit 7) and drdy_gyr (bit 6), cleared by reading it; ACC_DATA_X follows,
/// so one burst from here reads the flags and the sample together.
pub const STATUS: u8 = 0x02;

/// ACC_DATA_X: the first of six contiguous 16-bit words, accel x, y, z then gyro x, y, z.
pub const ACC_DATA_X: u8 = 0x03;

/// ACC_CONF: the accel's mode, range and rate.
pub const ACC_CONF: u8 = 0x20;

/// GYR_CONF: the gyro's mode, range and rate.
pub const GYR_CONF: u8 = 0x21;

/// IO_INT_CTRL: the INT1 pin's electrical behaviour.
pub const IO_INT_CTRL: u8 = 0x38;

/// INT_MAP2: where the accel and gyro data-ready interrupts go.
pub const INT_MAP2: u8 = 0x3B;

/// CMD: the command register (the soft reset).
pub const CMD: u8 = 0x7E;

/// What CHIP_ID reads on a BMI323 (past the dummy bytes).
pub const IDENTITY: u8 = 0x43;

/// One 16-bit register write: (register, value), sent least significant byte first.
pub type RegisterWrite = (u8, u16);

/// The soft reset: CMD = 0xDEAF.
pub const SOFT_RESET: RegisterWrite = (CMD, 0xDEAF);

/// The wait after the soft reset before the configuration is written.
pub const SOFT_RESET_SETTLE_MS: u32 = 5;

/// The configuration after the reset, in plan B's order:
///  * ACC_CONF = 0x7038: high-performance mode, +/-16 g, 100 Hz;
///  * GYR_CONF = 0x7048: high-performance mode, +/-2000 dps, 100 Hz.
pub const CONFIGURATION: [RegisterWrite; 2] = [(ACC_CONF, 0x7038), (GYR_CONF, 0x7048)];

/// Accel data-ready to INT1, written only when an INT1 wire is declared:
///  * IO_INT_CTRL = 0x0005: INT1 output enabled (bit 2), active high (bit 0), push-pull;
///  * INT_MAP2 = 0x0400: acc_drdy -> INT1 (selection 1 at bit 10).
pub const ROUTE_INTERRUPT: [RegisterWrite; 2] = [(IO_INT_CTRL, 0x0005), (INT_MAP2, 0x0400)];

/// The bytes the part prepends to every I2C register read.
pub const DUMMY_BYTES: usize = 2;

/// The burst read: the dummy bytes, then six signed 16-bit words.
pub const FRAME_LENGTH: usize = DUMMY_BYTES + 12;

/// The status burst from STATUS: the dummy bytes, the STATUS word, then the six data words.
pub const STATUS_FRAME_LENGTH: usize = DUMMY_BYTES + STATUS_WORD_LENGTH + 12;

/// The STATUS word's length.
const STATUS_WORD_LENGTH: usize = 2;

/// Where the accel half sits in the status burst (accel x, y, z come first out of this part).
const ACCEL_HALF: usize = DUMMY_BYTES + STATUS_WORD_LENGTH;

/// Where the gyro half sits in the status burst.
const GYRO_HALF: usize = ACCEL_HALF + HALF_LENGTH;

/// STATUS bit 7, drdy_acc: a new accel conversion is waiting.
const ACCEL_DATA_READY: u16 = 0x0080;

/// STATUS bit 6, drdy_gyr: a new gyro conversion is waiting.
const GYRO_DATA_READY: u16 = 0x0040;

/// Consecutive reads with no new conversion (neither half fresh) before the firmware calls the part unhealthy: one
/// such read is a tick between two conversions, three in a row are a part that stopped converting.
pub const STALE_LIMIT: u32 = 3;

/// What the gyro reports until it has a sample (0x8000): never published as a rate.
pub const NO_SAMPLE: i16 = i16::MIN;

/// How many times setup looks for the gyro's first sample, [`START_UP_POLL_MS`] apart: half a second, against the
/// ~20 ms the bench measured.
pub const START_UP_POLLS: u32 = 50;

/// The wait before each start-up look.
pub const START_UP_POLL_MS: u32 = 10;

/// Consecutive timed-out wakes (no INT1 edge) before a DECLARED line is called dead: at 100 Hz a fifth of a second.
pub const INTERRUPT_SILENT_LIMIT: u32 = 20;

/// Accel counts per g at +/-16 g.
pub const ACCEL_COUNTS_PER_G: f32 = 2048.0;

/// The gyro's full scale in degrees per second, over [`GYRO_FULL_SCALE_COUNTS`].
pub const GYRO_FULL_SCALE_DEGREES_PER_SECOND: i64 = 2000;

/// The gyro's counts at full scale.
pub const GYRO_FULL_SCALE_COUNTS: i64 = 32768;

/// Hundredths per unit: the telemetry gyro column is centidegrees per second.
const HUNDREDTHS: i64 = 100;

/// One burst, past the dummy bytes: raw signed counts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sample {
    /// Accel x, y, z counts (1/2048 g each).
    pub accel: [i16; 3],
    /// Gyro x, y, z counts (2000/32768 dps each).
    pub gyro: [i16; 3],
}

impl Sample {
    /// Decode a 14-byte burst: two dummy bytes, then accel x, y, z and gyro x, y, z, each signed 16-bit
    /// little-endian (plan B's `struct.unpack_from('<6h', buffer, 2)`). Accel comes FIRST out of this part.
    pub fn decode(frame: &[u8; FRAME_LENGTH]) -> Sample {
        let word = |index: usize| {
            let at = DUMMY_BYTES + 2 * index;
            i16::from_le_bytes([frame[at], frame[at + 1]])
        };
        Sample {
            accel: [word(0), word(1), word(2)],
            gyro: [word(3), word(4), word(5)],
        }
    }

    /// Whether the gyro has left its start-up state: no axis holds [`NO_SAMPLE`].
    pub fn gyro_started(&self) -> bool {
        !self.gyro.contains(&NO_SAMPLE)
    }

    /// Whether the burst is a sample at all: the gyro started, and not all three accel axes at [`NO_SAMPLE`] (a
    /// reset part's 0x8000 frame; one accel axis there is the -16 g rail).
    pub fn is_sample(&self) -> bool {
        self.gyro_started() && self.accel != [NO_SAMPLE; 3]
    }

    /// The accel in g: the `accel` channel and the telemetry's ax, ay, az (exact: a power-of-two scale).
    pub fn accel_g(&self) -> [f32; 3] {
        self.accel.map(|count| f32::from(count) / ACCEL_COUNTS_PER_G)
    }

    /// The gyro in degrees per second: the `rate` channel. Exact in f32: 2000/32768 is 125/2048.
    pub fn rate_degrees_per_second(&self) -> [f32; 3] {
        let per_count = GYRO_FULL_SCALE_DEGREES_PER_SECOND as f32 / GYRO_FULL_SCALE_COUNTS as f32;
        self.gyro.map(|count| f32::from(count) * per_count)
    }

    /// The gyro in centidegrees per second, plan B's integer to the digit (`raw x 2000 x 100 // 32768`, floored, so
    /// -1 count is -7 and +1 is 6): the telemetry's gx, gy, gz columns. The full scale reads 199993 and -200000.
    pub fn rate_centidegrees_per_second(&self) -> [i32; 3] {
        self.gyro.map(|count| {
            let centidegrees =
                (i64::from(count) * GYRO_FULL_SCALE_DEGREES_PER_SECOND * HUNDREDTHS).div_euclid(GYRO_FULL_SCALE_COUNTS);
            centidegrees as i32 // within +/-200000 for every 16-bit count
        })
    }
}

/// One BMI323 on an I2C bus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bmi323 {
    /// The 7-bit address.
    address: u8,
    /// Whether an INT1 wire is declared (the config's `int_pin` resolves to a GPIO).
    interrupt_wired: bool,
    /// The run of timed-out wakes, toward the silent verdict.
    strikes: Strikes,
    /// The operator view: a declared INT line went silent.
    interrupt_silent: bool,
    /// The status bursts' judge: the last burst's data bytes and its raced marks (see the module notes).
    flags: Flagged,
}

impl Bmi323 {
    /// A driver for the part at `address`; `interrupt_wired` when its `int_pin` resolves to a GPIO (on the v1.x map
    /// layout routes `accel_int1` to it; the v0.1 attitude socket has no INT wire, and the driver polls).
    pub fn new(address: u8, interrupt_wired: bool) -> Bmi323 {
        Bmi323 {
            address,
            interrupt_wired,
            strikes: Strikes::default(),
            interrupt_silent: false,
            flags: Flagged::default(),
        }
    }

    /// Forget the stored data bytes and the raced marks, so the next [`Bmi323::read_fresh`] is judged by its flags
    /// alone. Setup and the re-arm forget by themselves.
    pub fn forget(&mut self) {
        self.flags.forget();
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// The operator view of the INT line (inspect's `interrupt_silent`): None with no wire declared (polled BY
    /// DESIGN), false while INT1 drives the sampling, true when a declared line went silent. Reporting true for an
    /// unwired board made "not connected" and "connected but dead" read alike, and only the second is a fault.
    pub fn interrupt_silent(&self) -> Option<bool> {
        self.interrupt_wired.then_some(self.interrupt_silent)
    }

    /// Account one sampling wake: `interrupt_runs` INT1 edges since the last (0 = the period ran out, 1 = healthy,
    /// more = an overrun, a scheduling symptom); the telemetry row carries the same count as its `irq_runs` column.
    ///
    /// With a wire declared, [`INTERRUPT_SILENT_LIMIT`] timed-out wakes in a row set the silent flag, and the return
    /// is true exactly once then, for the firmware's one log line (`INT1 silent -- sampling on the <period> ms
    /// fallback`); one later edge clears it. With NO wire the part is polled by design and nothing is counted:
    /// warning about a missing line every boot trains the operator to read past a message that does mean something
    /// on a board where the wire exists.
    pub fn wake(&mut self, interrupt_runs: u32) -> bool {
        if self.interrupt_wired && self.strikes.strike(interrupt_runs == 0, INTERRUPT_SILENT_LIMIT) {
            self.interrupt_silent = true;
            return true;
        }
        if interrupt_runs > 0 {
            self.interrupt_silent = false;
        }
        false
    }

    /// Write one 16-bit register, least significant byte first.
    fn write_register<Bus: I2c>(&self, bus: &mut Bus, (register, value): RegisterWrite) -> Result<(), Bus::Error> {
        let [low, high] = value.to_le_bytes();
        bus.write(self.address, &[register, low, high])
    }

    /// Read CHIP_ID PAST the two dummy bytes (a 3-byte read; the third byte is the identity).
    pub fn identity<Bus: I2c>(&self, bus: &mut Bus) -> Result<u8, Fault<Bus::Error>> {
        let mut answer = [0u8; DUMMY_BYTES + 1];
        bus.write_read(self.address, &[CHIP_ID], &mut answer)
            .map_err(Fault::Bus)?;
        Ok(answer[DUMMY_BYTES])
    }

    /// Read one burst: accel and gyro, which also clears data-ready.
    pub fn read<Bus: I2c>(&self, bus: &mut Bus) -> Result<Sample, Fault<Bus::Error>> {
        let mut frame = [0u8; FRAME_LENGTH];
        bus.write_read(self.address, &[ACC_DATA_X], &mut frame)
            .map_err(Fault::Bus)?;
        Ok(Sample::decode(&frame))
    }

    /// Read one sample and which of its halves are new: ONE 16-byte burst from STATUS (the dummy bytes, the STATUS
    /// word, then the six data words). Each half is judged by its flag, drdy_acc for the accel and drdy_gyr for the
    /// gyro, corrected by its bytes against the last burst's (the module notes); drdy_temp and por_detected are
    /// ignored. A frame that is no sample ([`Sample::is_sample`]) is never fresh and leaves the stored bytes as they
    /// were. Reading STATUS clears the flags.
    pub fn read_fresh<Bus: I2c>(&mut self, bus: &mut Bus) -> Result<(Fresh, Sample), Fault<Bus::Error>> {
        let mut burst = [0u8; STATUS_FRAME_LENGTH];
        bus.write_read(self.address, &[STATUS], &mut burst)
            .map_err(Fault::Bus)?;
        let status = u16::from_le_bytes([burst[DUMMY_BYTES], burst[DUMMY_BYTES + 1]]);
        let mut frame = [0u8; FRAME_LENGTH];
        frame[DUMMY_BYTES..].copy_from_slice(&burst[ACCEL_HALF..]);
        let sample = Sample::decode(&frame);
        if !sample.is_sample() {
            return Ok((Fresh::default(), sample));
        }
        let half = |start: usize| {
            let mut bytes = [0u8; HALF_LENGTH];
            bytes.copy_from_slice(&burst[start..start + HALF_LENGTH]);
            bytes
        };
        let fresh = self.flags.judge(
            status & ACCEL_DATA_READY != 0,
            &half(ACCEL_HALF),
            status & GYRO_DATA_READY != 0,
            &half(GYRO_HALF),
        );
        Ok((fresh, sample))
    }

    /// The on-demand self-test (bmi323.probe): the identity reads back, then one burst reads. The firmware logs the
    /// accel magnitude with it: gravity is the one reference on a bench, and a part reading far from 1 g at rest
    /// is mis-scaled or mis-configured, which a bare "it answered" probe would pass.
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

    /// Classify a failed setup (bmi323.diagnose), in plan B's words, on the port `port` (`i2c:<port>`). Not the
    /// shared one-byte id read, which on this part reads a dummy byte and would call a healthy sensor the wrong
    /// device.
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus, port: u8) -> String
    where
        Bus::Error: core::fmt::Debug,
    {
        let address = self.address;
        match self.identity(bus) {
            Err(fault) => format!("no ack at i2c:{port} 0x{address:02x} -- {fault}"),
            Ok(read) if read != IDENTITY => {
                format!("wrong device at i2c:{port} 0x{address:02x}: id 0x{read:02x}, expected 0x{IDENTITY:02x}")
            }
            Ok(_) => "present and answering, but setup did not complete".to_string(),
        }
    }

    /// Soft reset, then accel and gyro to high-performance mode at their flight ranges (bmi323._configure).
    fn configure<Bus: I2c>(&self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.write_register(bus, SOFT_RESET).map_err(Fault::Bus)?;
        delay.delay_ms(SOFT_RESET_SETTLE_MS);
        for write in CONFIGURATION {
            self.write_register(bus, write).map_err(Fault::Bus)?;
        }
        Ok(())
    }

    /// Wait out the gyro's start-up (bmi323._await_gyro): up to [`START_UP_POLLS`] looks, each after
    /// [`START_UP_POLL_MS`]; [`Fault::Timeout`] when every look still holds the marker.
    fn await_gyro<Bus: I2c>(&self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        for _ in 0..START_UP_POLLS {
            delay.delay_ms(START_UP_POLL_MS);
            if self.read(bus)?.gyro_started() {
                return Ok(());
            }
        }
        Err(Fault::Timeout)
    }

    /// Route accel data-ready to INT1 when the wire is declared, then the clearing read, so the next conversion is
    /// a clean rising edge (bmi323._setup_interrupt). Interrupt-driven sampling costs far less than polling at the
    /// ODR, and the wire is optional BY DESIGN: without it, or with a dead line, the part is polled.
    fn route_interrupt<Bus: I2c>(&self, bus: &mut Bus) -> Result<(), Fault<Bus::Error>> {
        if !self.interrupt_wired {
            return Ok(());
        }
        for write in ROUTE_INTERRUPT {
            self.write_register(bus, write).map_err(Fault::Bus)?;
        }
        self.read(bus)?;
        Ok(())
    }
}

impl<Bus: I2c> Device<Bus> for Bmi323 {
    type BusError = Bus::Error;

    /// The identity (past the dummy bytes), the reset and configuration, the start-up wait, then the INT1 routing
    /// (bmi323.setup's bus half). A gyro that never starts is [`Fault::Timeout`]: plan B's 'gyro never left its
    /// start-up state'. The stored data bytes are forgotten first ([`Bmi323::forget`]).
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.forget();
        let read = self.identity(bus)?;
        if read != IDENTITY {
            return Err(Fault::Identity {
                expected: u16::from(IDENTITY),
                read: u16::from(read),
            });
        }
        self.configure(bus, delay)?;
        self.await_gyro(bus, delay)?;
        self.route_interrupt(bus)
    }

    /// Re-apply the configuration after a reset underneath it: a BMI323 that took the ICP's general call powers up
    /// with both sensors DISABLED and would publish nothing. The reset, the configuration, the start-up wait, and
    /// (unlike plan B, see the module notes) the INT1 routing again; the stored data bytes are forgotten.
    fn rearm(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.forget();
        self.configure(bus, delay)?;
        self.await_gyro(bus, delay)?;
        self.route_interrupt(bus)
    }
}

#[cfg(test)]
mod tests;
