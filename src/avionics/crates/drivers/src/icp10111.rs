//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ICP-10111 barometer (TDK ICP-101xx, on the SEN0517; src/glider/drivers/icp10111.py): the PRIMARY altitude
//! channel, 8.5 cm accuracy. It publishes pressure (Pa), temperature (degrees C), altitude (m above the sea-level
//! datum) and elevation (m above its own startup ground zero), which drives the endgame band, the landing fallback
//! and the launch backup.
//!
//! Command-based, not register-mapped: [`Device::setup`] verifies the product id, reads the four OTP calibration
//! constants and averages the ground zero; [`Icp10111::poll`] runs the TWO-PHASE measurement the bus owner needs --
//! issue "measure, pressure first, normal mode" ([`MEASURE`], 6.3 ms and 1.6 Pa RMS), then collect the 9-byte frame
//! at least [`MEASURE_MS`] later -- so the owner's other devices keep their slots while the part converts. Plan B
//! slept through the conversion in its own asyncio task; the bus owner thread cannot sleep, so the wait became a
//! phase with a deadline, and the time arrives as an argument.
//!
//! THE CADENCE ON THE TICK. The rule is plan B's: the next measurement is issued a period after the collect. The
//! owner polls on the 100 Hz control tick, so each phase lands on a wake: the issue at 0, the collect on the first
//! wake at least [`MEASURE_MS`] later (20 ms), and the next issue a period after that collect. With the default
//! 100 ms period that deadline falls on a tick boundary (120 ms) and is met only when that wake lands no earlier
//! after its tick than the collect's did; a wake even 1 us earlier misses it and issues on the next tick (130 ms).
//! So the board's cycle is 120 or 130 ms, about 8 Hz, never the 10 Hz the period suggests (plan B's own loop,
//! sleeping a period after its read, ran at ~112 ms). Anchoring the next issue to the previous issue instead would
//! give 10 Hz, if the detectors fed by the elevation slope ever need a regular cadence.
//!
//! THE WIRE RULE: only what the wire says about the part strikes -- a NAK, a lost arbitration, a bus error, a CRC
//! reject. A bus failure the port reports as [`ErrorKind::Other`] never reached the part: the firmware's `BusBusy`
//! (the `i2c_master` bus lock not had in time, because another handle on the port held it -- the camera's SCCB on
//! 7C's i2c:0) or a driver refusal. It is reported ([`Poll::Failed`], never `recovering`), the next issue comes a
//! period later, and the run of strikes is left exactly as it was, neither counted nor rearmed: counted, a streaming
//! camera would fire general calls that reset every part on the port for nothing; rearmed, it would hide a wedge in
//! progress. The recovery's own general call obeys the same rule: one the port refuses reset nothing, so it is
//! reported the same way and goes again a period later, and [`Poll::GeneralCall`] and [`Poll::Rearm`] wait for the
//! call that reached the wire (a NAK there is still ignored: nothing may honour it). Reported anyway, they would send
//! the owner walking the peers for a reset that never happened and spend the run: a part latched at that moment
//! would never escalate again. Likewise setup tells the owner of a general call only when it reached the wire. Plan B
//! never met this failure (no camera on its buses).
//!
//! THE CRCs: the part appends a CRC-8 to each of its three words, and plan B once read them and threw them away.
//! That is the expensive kind of silence: a corrupted I2C word does not raise, it yields a plausible altitude. This
//! part has a documented latch-up habit on this board, which is exactly the state that puts bad bytes on the wire,
//! so a frame failing any CRC is REFUSED and takes the same path as a part that stopped answering.
//!
//! THE GENERAL CALL. Measured after the 2026-07-06 OOM-soak panic: a mid-conversion part NAKs until the conversion
//! drains, and an unclean reboot can LATCH its digital core -- the address still acks (a scan sees 0x63) but every
//! command NAKs. The addressed soft reset (0x805D) RE-WEDGED the latched bench part; the I2C GENERAL-CALL reset
//! (0x06 to address 0x00) recovered it fully (5 of 5 measurements after). So setup touches the part plainly (a clean
//! boot never needs a reset), drains a possible busy window on the first NAK, and fires the general call only from
//! the second; and the run loop, after [`RECOVER_AFTER`] consecutive failures, waits a period for a busy
//! conversion to drain, fires the general call, and lets the bus settle.
//!
//! The general call resets EVERY part on the bus that honours it, back to its power-on state (the operator's rule,
//! 2026-10-06; plan B commit 76c138c): the bus owner then calls [`Device::rearm`] on every OTHER device registered on
//! that port, found by membership. Plan B walked its registry from inside this driver; here the driver reports
//! it -- [`Icp10111::take_general_call`] after setup, [`Poll::Rearm`] from the run loop -- and the owner, who holds
//! the registry, walks it. One deliberate difference: plan B re-armed the peers only when its setup then SUCCEEDED,
//! so a general call followed by a dead ICP left the SEN0697 parts set up before it (7C, i2c:0) in their power-on
//! state; the port reports every general call that went out, success or not.
//!
//! THE POLYNOMIAL ([`compensate`]) is TDK's, in f32 (decision D18), and stays f32 by measurement (avionics-port.md
//! Q8): the golden fixture pins plan B's answer twice, under CPython's f64 and under the board's single precision
//! (MicroPython on the P4 computes it in f32, op by op), and the port matches the board's EXACTLY, bit for bit, while
//! staying within 0.024 Pa of the f64 over the fixture's cases (0.032 Pa over a denser sweep of 30-110 kPa and
//! -40..85 degrees C) -- a fiftieth of the part's 1.6 Pa RMS noise, under 3 mm of altitude. Plan B cached the
//! temperature-dependent half per 37 raw counts (~0.1 degrees C) to stop allocating floats with the GC off in flight;
//! that cache is a MicroPython workaround (avionics-port.md §6, not ported), so the port computes the whole
//! polynomial per sample, which moves the answer by up to 0.39 Pa against plan B's cached one (3.3 cm; the fixture's
//! `measured`) -- a larger change than f32 makes, and toward the part's true reading.
//!
//! Not ported: plan B's conversation guard and sample cache (`_read`, `claim`): the owner thread is the only caller.
//! For the same reason the probe ([`Icp10111::probe`]) is one direct conversion, where plan B's issued no I2C and
//! waited for the run loop's FRESH pressure (avionics-port.md M5 design decisions, 'Device probes vs bus probe').
//! Not here (firmware glue): the channel push, the telemetry stream, the freshness of the altitude a re-zero takes,
//! the log lines and the inspect JSON.

use control::commons::altitude_m;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{Error, ErrorKind, I2c};
use serde_json::Value;

use crate::commons::{id_classify, python_float, sensirion_crc8};
use crate::device::{Device, Fault, Strikes};

/// The fixed I2C address.
pub const ADDRESS: u8 = 0x63;

/// Read the product id: a 16-bit word (and its CRC), whose low six bits are 0x08 on an ICP-101xx.
pub const READ_ID: [u8; 2] = [0xEF, 0xC8];

/// The I2C general-call address.
pub const GENERAL_CALL_ADDRESS: u8 = 0x00;

/// The general-call reset command: it reaches a wedged core that every addressed command NAKs.
pub const GENERAL_CALL_RESET: [u8; 1] = [0x06];

/// Unlock the OTP for reading (address 0x0066 and its CRC 0x9C, behind the command 0xC595).
pub const OTP_UNLOCK: [u8; 5] = [0xC5, 0x95, 0x00, 0x66, 0x9C];

/// Read the next OTP word (auto-incrementing): issued once per calibration constant.
pub const OTP_READ: [u8; 2] = [0xC7, 0xF7];

/// Measure, pressure first, normal mode: 6.3 ms, 1.6 Pa RMS.
pub const MEASURE: [u8; 2] = [0x48, 0xA3];

/// Attempts at the first touch, spanning one maximum-length conversion window (~95 ms with the waits).
pub const RESET_TRIES: u32 = 4;

/// The wait after a failed first touch (and after a general call) before the next command.
pub const RESET_RETRY_MS: u32 = 30;

/// Consecutive run-loop failures before the recovery escalates to the general call.
pub const RECOVER_AFTER: u32 = 3;

/// The conversion wait for normal mode (6.3 ms typical), with margin: collect no earlier.
pub const MEASURE_MS: u32 = 12;

/// The product id's significant bits.
pub const IDENTITY_MASK: u16 = 0x3F;

/// The product id's significant bits on an ICP-101xx.
pub const IDENTITY: u16 = 0x08;

/// The answer to the id and OTP reads: one word and its CRC (unchecked, as plan B: the id mask and the OTP's later
/// use are the checks).
pub const WORD_LENGTH: usize = 3;

/// How many OTP calibration constants the part carries.
pub const OTP_WORDS: usize = 4;

/// The measurement frame, three words and their CRCs: pressure bytes 0 and 1, then pressure byte 3 and a pad, then
/// temperature bytes 6 and 7.
pub const FRAME_LENGTH: usize = 9;

/// Readings averaged at setup (and at a calibrate) to fix the ground-zero reference.
pub const GROUND_SAMPLES: u32 = 8;

/// The default read period, ms: the wait from a collect to the next issue, 120 or 130 ms a cycle on the 10 ms tick
/// (see the module notes).
pub const DEFAULT_PERIOD_MS: u32 = 100;

/// What the calibration column shows until a ground zero is held (plan B's calibration() text).
pub const GROUND_INSTRUCTION: &str = "stand the glider STILL on the pad -- this captures its ground reference";

/// 1 / 2^24: the scale of the squared temperature offset in the polynomial.
const QUADRATIC_SCALE: f32 = 1.0 / 16_777_216.0;

/// The lower look-up-table anchor (TDK's LUT_lower), 3.5 x 2^20.
const LOOKUP_TABLE_LOWER: f32 = 3.5 * 1_048_576.0;

/// The upper look-up-table anchor (TDK's LUT_upper), 11.5 x 2^20.
const LOOKUP_TABLE_UPPER: f32 = 11.5 * 1_048_576.0;

/// The offset factor of the fourth OTP constant.
const OFFSET: f32 = 2048.0;

/// The calibration pressures the look-up table is anchored at, Pa.
const CALIBRATION_PRESSURES: [f32; 3] = [45_000.0, 80_000.0, 105_000.0];

/// The raw temperature the polynomial centres on (2^15).
const TEMPERATURE_CENTRE: i32 = 32_768;

/// Microseconds per millisecond: the deadlines are in microseconds, the waits in milliseconds.
const MICROSECONDS_PER_MILLISECOND: u64 = 1000;

/// The four OTP calibration constants, signed 16-bit, read once at setup.
pub type Otp = [i16; OTP_WORDS];

/// The temperature-dependent half of the conversion (icp10111._calibrate): the triple the pressure line needs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coefficients {
    /// The offset term, Pa.
    pub a: f32,
    /// The scale term.
    pub b: f32,
    /// The raw-count offset term.
    pub c: f32,
}

/// TDK's ICP-101xx calibration: a raw temperature and the OTP constants to the (a, b, c) triple
/// (icp10111._calibrate). The raw pressure appears nowhere in it.
///
/// Every operation is plan B's, in plan B's order, so the f32 result is the board's to the bit (see the module
/// notes): `t x t` is an integer first (|t| <= 2^15, so it fits), then one rounding to f32.
pub fn coefficients(otp: &Otp, temperature_raw: u16) -> Coefficients {
    let [c0, c1, c2, c3] = otp.map(f32::from);
    let t = i32::from(temperature_raw) - TEMPERATURE_CENTRE;
    let quadratic = (t * t) as f32 * QUADRATIC_SCALE;
    let s1 = LOOKUP_TABLE_LOWER + c0 * quadratic;
    let s2 = OFFSET * c3 + c1 * quadratic;
    let s3 = LOOKUP_TABLE_UPPER + c2 * quadratic;
    let [p0, p1, p2] = CALIBRATION_PRESSURES;
    let c = (s1 * s2 * (p0 - p1) + s2 * s3 * (p1 - p2) + s3 * s1 * (p2 - p0))
        / (s3 * (p0 - p1) + s1 * (p1 - p2) + s2 * (p2 - p0));
    let a = (p0 * s1 - p1 * s2 - (p1 - p0) * c) / (s1 - s2);
    let b = (p0 - a) * (s1 + c);
    Coefficients { a, b, c }
}

/// The TDK polynomial: raw pressure and temperature to pressure in Pa (icp10111._compensate, without plan B's
/// per-37-counts cache of the triple: see the module notes).
pub fn compensate(otp: &Otp, pressure_raw: u32, temperature_raw: u16) -> f32 {
    let Coefficients { a, b, c } = coefficients(otp, temperature_raw);
    a + b / (c + pressure_raw as f32)
}

/// The temperature in degrees C of a raw temperature word: -45 + 175 x raw / 2^16 (exact but for the one sum).
pub fn temperature(temperature_raw: u16) -> f32 {
    -45.0 + 175.0 / 65_536.0 * f32::from(temperature_raw)
}

/// The raw words of a measurement frame, after every CRC passed: (pressure, 24-bit; temperature, 16-bit); None when
/// any of the three words fails its CRC.
pub fn decode(frame: &[u8; FRAME_LENGTH]) -> Option<(u32, u16)> {
    let words_ok = sensirion_crc8(frame[0], frame[1]) == frame[2]
        && sensirion_crc8(frame[3], frame[4]) == frame[5]
        && sensirion_crc8(frame[6], frame[7]) == frame[8];
    words_ok.then(|| {
        let pressure_raw = (u32::from(frame[0]) << 16) | (u32::from(frame[1]) << 8) | u32::from(frame[3]);
        (pressure_raw, u16::from_be_bytes([frame[6], frame[7]]))
    })
}

/// One conversion, in the units the channels and the telemetry row carry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    /// Metres above the sea-level datum (commons.altitude_m of the pressure).
    pub altitude: f32,
    /// Degrees C.
    pub temperature: f32,
    /// Pa.
    pub pressure: f32,
}

impl Reading {
    /// Convert a frame's raw words with the part's OTP.
    pub fn convert(otp: &Otp, pressure_raw: u32, temperature_raw: u16) -> Reading {
        let pressure = compensate(otp, pressure_raw, temperature_raw);
        Reading {
            altitude: altitude_m(pressure),
            temperature: temperature(temperature_raw),
            pressure,
        }
    }
}

/// What one [`Icp10111::poll`] did, for the owner to act on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Poll<BusError> {
    /// Nothing was due: the next measurement, the conversion or the recovery is still ahead.
    Waiting,
    /// The measurement command went out; the frame is collected [`MEASURE_MS`] later.
    Issued,
    /// A conversion was collected: publish it.
    Reading(Reading),
    /// The command or the collect failed, or the frame failed a CRC (plan B's deduplicated `icp10111 :: read
    /// <error>` note), or the port refused the recovery's general call (kind [`ErrorKind::Other`]; it goes again a
    /// period later). `recovering` is true on the one failure that completes a run of [`RECOVER_AFTER`]: the
    /// recovery has begun, and the general call goes out a period later. Never on a bus failure of kind
    /// [`ErrorKind::Other`], which is no strike (the wire rule, in the module notes).
    Failed {
        /// What failed.
        fault: Fault<BusError>,
        /// Whether this failure started the recovery.
        recovering: bool,
    },
    /// The recovery's general call reached the wire (a NAK is expected and ignored: nothing may honour it); the bus
    /// settles for [`RESET_RETRY_MS`] before [`Poll::Rearm`].
    GeneralCall,
    /// The bus has settled after the general call: re-arm every OTHER device registered on this port now, by
    /// membership, and log plan B's 'read recovery: general-call reset + peer rearm after 3 failures'.
    Rearm,
}

/// Where the run loop is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// Ready to issue the next measurement at `next` (microseconds).
    Idle {
        /// When the next measurement is due.
        next: u64,
    },
    /// A measurement was issued at `issued`; the frame is collected from `issued` + [`MEASURE_MS`] on.
    Converting {
        /// When the command went out.
        issued: u64,
    },
    /// The recovery waits out a busy conversion until `until`, then sends the general call (again a period later
    /// while the port refuses it).
    Draining {
        /// When the general call goes out.
        until: u64,
    },
    /// The general call went out; the bus settles until `until`, then the peers are re-armed.
    Settling {
        /// When the peers are re-armed.
        until: u64,
    },
}

/// Why an operator `update` was refused; nothing has changed when it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// A re-zero with no altitude fresher than three periods: a re-zero LATCHES a number every later elevation is
    /// reported against, so a stale altitude would bias the channel for the rest of the flight.
    NoFreshAltitude,
    /// `ground` is not a number [`python_float`] reads: plan B's `float()` raised. Plan B answered with the
    /// interpreter's own text ("could not convert string to float: 'high'" on CPython, "invalid syntax for number"
    /// on MicroPython) for a string, and with a TypeError its `update` command did not catch for a null, a list or
    /// an object; the port refuses them all alike, in its own words.
    NotANumber,
}

impl core::fmt::Display for Refusal {
    /// The refusal text: plan B's words for [`Refusal::NoFreshAltitude`]; the port's own for
    /// [`Refusal::NotANumber`], where plan B's were the interpreter's (see the variant).
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::NoFreshAltitude => write!(formatter, "no fresh altitude to re-zero from"),
            Refusal::NotANumber => write!(formatter, "ground must be a number"),
        }
    }
}

/// One ICP-10111 on an I2C bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Icp10111 {
    /// The 7-bit address.
    address: u8,
    /// The run loop's period, ms: the measurement cadence and the recovery's drain.
    period_ms: u32,
    /// The OTP constants; None until setup reads them.
    otp: Option<Otp>,
    /// The ground zero, m above the datum; None until setup (or a calibrate) captures it.
    ground: Option<f32>,
    /// Where the run loop is.
    phase: Phase,
    /// The run of consecutive failures, toward the recovery.
    strikes: Strikes,
    /// A general call went out that the owner has not yet been told about.
    general_call: bool,
}

impl Icp10111 {
    /// A driver for the part at `address`, measuring every `period_ms` (`period_ms` in the config, default
    /// [`DEFAULT_PERIOD_MS`]).
    pub fn new(address: u8, period_ms: u32) -> Icp10111 {
        Icp10111 {
            address,
            period_ms,
            otp: None,
            ground: None,
            phase: Phase::Idle { next: 0 },
            strikes: Strikes::default(),
            general_call: false,
        }
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// The measurement period, ms (`period_ms`): a re-zero takes an altitude fresher than three of them.
    pub fn period_ms(&self) -> u32 {
        self.period_ms
    }

    /// The OTP constants setup read.
    pub fn otp(&self) -> Option<Otp> {
        self.otp
    }

    /// The ground zero, m above the datum.
    pub fn ground(&self) -> Option<f32> {
        self.ground
    }

    /// Metres above the ground zero: the `elevation` channel (None before a ground zero is held).
    pub fn elevation(&self, reading: &Reading) -> Option<f32> {
        self.ground.map(|ground| reading.altitude - ground)
    }

    /// The outstanding instruction, or '' once a ground zero is held (plan B's calibration()).
    pub fn instruction(&self) -> &'static str {
        if self.ground.is_some() { "" } else { GROUND_INSTRUCTION }
    }

    /// Whether a general call went out since the last ask (setup's first-touch escalation), cleared as it reports:
    /// on true the owner re-arms every other device registered on this port.
    pub fn take_general_call(&mut self) -> bool {
        core::mem::take(&mut self.general_call)
    }

    /// One period in microseconds.
    fn period_us(&self) -> u64 {
        u64::from(self.period_ms) * MICROSECONDS_PER_MILLISECOND
    }

    /// The product id word, through the command and a 3-byte read (plan B's two separate transactions).
    fn identity_word<Bus: I2c>(&self, bus: &mut Bus) -> Result<u16, Bus::Error> {
        bus.write(self.address, &READ_ID)?;
        let mut word = [0u8; WORD_LENGTH];
        bus.read(self.address, &mut word)?;
        Ok(u16::from_be_bytes([word[0], word[1]]))
    }

    /// Send setup's general-call reset; a NAK is ignored (nothing on the bus may honour it -- nothing lost by
    /// asking), and the owner is told either way ([`Icp10111::take_general_call`]). A call the port refused reset
    /// nothing, so the owner is not told of it (the wire rule); setup's next touch decides whether another goes.
    fn general_call<Bus: I2c>(&mut self, bus: &mut Bus) {
        match bus.write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET) {
            Err(error) if refused(&error) => {}
            _ => self.general_call = true,
        }
    }

    /// One blocking conversion: the command, the wait, the frame, the CRCs (icp10111._measure).
    fn measure<Bus: I2c>(
        &self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
        otp: &Otp,
    ) -> Result<Reading, Fault<Bus::Error>> {
        bus.write(self.address, &MEASURE).map_err(Fault::Bus)?;
        delay.delay_ms(MEASURE_MS);
        self.collect(bus, otp)
    }

    /// Read the frame of a conversion that has had its time, check it and convert it.
    fn collect<Bus: I2c>(&self, bus: &mut Bus, otp: &Otp) -> Result<Reading, Fault<Bus::Error>> {
        let mut frame = [0u8; FRAME_LENGTH];
        bus.read(self.address, &mut frame).map_err(Fault::Bus)?;
        let (pressure_raw, temperature_raw) = decode(&frame).ok_or(Fault::Checksum)?;
        Ok(Reading::convert(otp, pressure_raw, temperature_raw))
    }

    /// Average [`GROUND_SAMPLES`] fresh conversions into the ground zero (icp10111._ground_zero; the calibrate
    /// path too): each is a genuine conversion, back to back, never a cached sample, or the average would collapse
    /// to one reading.
    ///
    /// A conversion the run loop has in flight is waited out and read first (and dropped): a second measure command
    /// into the middle of it would NAK -- plan B measured a 20.8 % failure rate when a diagnostic polled alongside
    /// the run loop. The run loop issues again on its next poll. Nothing changes when a conversion fails.
    pub fn capture_ground<Bus: I2c>(
        &mut self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
    ) -> Result<f32, Fault<Bus::Error>> {
        let otp = self.otp.ok_or(Fault::Configuration)?;
        self.drain_conversion(bus, delay, &otp);
        let mut total = 0.0;
        for _ in 0..GROUND_SAMPLES {
            total += self.measure(bus, delay, &otp)?.altitude;
        }
        let ground = total / GROUND_SAMPLES as f32;
        self.ground = Some(ground);
        Ok(ground)
    }

    /// The on-demand self-test (icp10111.probe, adapted per avionics-port.md M5 design decisions,
    /// 'Device probes vs bus probe'): one blocking conversion -- the command, the [`MEASURE_MS`] wait, the CRC-checked
    /// frame (icp10111._measure) -- converted with the part's OTP. [`Fault::Configuration`] before setup has read the
    /// OTP, and nothing is sent.
    ///
    /// Plan B's probe issued no I2C: it waited for the run loop to push a FRESH pressure, because a direct
    /// conversion raced the loop's own on the shared bus (icp10111.py:398-426). The owner thread is the only caller
    /// and cannot race itself, so the port converts directly. The probe's own conversion never moves the run loop's
    /// failure count, as plan B's never touched the part: a failed self-test is the operator's verdict, not a strike
    /// toward the general call, and a good one does not rearm a run of failures.
    ///
    /// Plan B's run loop always counted its own outcome; the port's can lose one to a probe. A conversion the run
    /// loop has in flight is waited out, read and dropped UNCOUNTED, as [`Icp10111::capture_ground`] does: a NAK or
    /// a CRC reject there is a strike that never lands (the general call waits for the next failing pass, a
    /// conversion or two later), and a good frame is a rearm that never lands (a run it would have broken can
    /// complete on the next failure). The run loop then issues afresh. A probe during the recovery leaves its
    /// schedule alone: in the period before the general call it finds the part as the failures left it, in the
    /// [`RESET_RETRY_MS`] after it a part still resetting may NAK and fail the probe, and the general call and
    /// [`Poll::Rearm`] come on time either way.
    pub fn probe<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<Reading, Fault<Bus::Error>> {
        let otp = self.otp.ok_or(Fault::Configuration)?;
        self.drain_conversion(bus, delay, &otp);
        self.measure(bus, delay, &otp)
    }

    /// Wait out and drop a conversion the run loop has in flight, so a blocking conversion does not start in the
    /// middle of it: a second measure command into a conversion NAKs (plan B measured a 20.8 % failure rate when a
    /// diagnostic polled alongside the run loop). Its outcome is not counted, good or bad: a strike completing a run
    /// would start the recovery here, with no clock to schedule it by and no [`Poll`] to report it through. The run
    /// loop issues afresh on its next poll.
    fn drain_conversion<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs, otp: &Otp) {
        if matches!(self.phase, Phase::Converting { .. }) {
            delay.delay_ms(MEASURE_MS);
            let _ = self.collect(bus, otp);
            self.phase = Phase::Idle { next: 0 };
        }
    }

    /// One step of the run loop at `now_us` (the owner's clock): at most one transfer, never a wait.
    ///
    /// Call it on every owner tick. It keeps plan B's cadence: a measurement is issued, collected at least
    /// [`MEASURE_MS`] later, and the next is issued a period after the collect (or after a failed phase): 120 or
    /// 130 ms a cycle on the 10 ms tick (the module notes). Every wire failure -- a NAK on either phase, or a CRC
    /// reject -- counts; a good frame rearms the count; a bus failure of kind [`ErrorKind::Other`] does neither (the
    /// wire rule). The failure that completes a run of [`RECOVER_AFTER`] starts the recovery: a period for a busy
    /// conversion to drain, the general call ([`Poll::GeneralCall`]; one the port refuses is reported as a failure
    /// and goes again a period later), [`RESET_RETRY_MS`] to settle, then [`Poll::Rearm`] for the owner's walk, and
    /// the next measurement a period after that. A good read later rearms the count, and a LATER wedge escalates
    /// again; a part that stays dead escalates once, not on every failure.
    pub fn poll<Bus: I2c>(&mut self, bus: &mut Bus, now_us: u64) -> Poll<Bus::Error> {
        match self.phase {
            Phase::Idle { next } if now_us >= next => match bus.write(self.address, &MEASURE) {
                Ok(()) => {
                    self.phase = Phase::Converting { issued: now_us };
                    Poll::Issued
                }
                Err(error) => self.failed(Fault::Bus(error), now_us),
            },
            Phase::Converting { issued } if now_us >= issued + u64::from(MEASURE_MS) * MICROSECONDS_PER_MILLISECOND => {
                let Some(otp) = self.otp else {
                    return self.failed(Fault::Configuration, now_us);
                };
                match self.collect(bus, &otp) {
                    Ok(reading) => {
                        self.strikes.strike(false, RECOVER_AFTER);
                        self.phase = Phase::Idle {
                            next: now_us + self.period_us(),
                        };
                        Poll::Reading(reading)
                    }
                    Err(fault) => self.failed(fault, now_us),
                }
            }
            Phase::Draining { until } if now_us >= until => {
                match bus.write(GENERAL_CALL_ADDRESS, &GENERAL_CALL_RESET) {
                    // the port refused it: nothing reached the wire, so nothing was reset -- again a period later,
                    // which also spaces out the bus-lock timeouts a busy port costs the owner
                    Err(error) if refused(&error) => {
                        self.phase = Phase::Draining {
                            until: now_us + self.period_us(),
                        };
                        Poll::Failed {
                            fault: Fault::Bus(error),
                            recovering: false,
                        }
                    }
                    // a NAK is ignored: nothing on the bus may honour it -- nothing lost by asking
                    _ => {
                        self.phase = Phase::Settling {
                            until: now_us + u64::from(RESET_RETRY_MS) * MICROSECONDS_PER_MILLISECOND,
                        };
                        Poll::GeneralCall
                    }
                }
            }
            Phase::Settling { until } if now_us >= until => {
                self.phase = Phase::Idle {
                    next: now_us + self.period_us(),
                };
                Poll::Rearm
            }
            _ => Poll::Waiting,
        }
    }

    /// Count a failure at `now_us` and decide what follows: the recovery on the failure that completes the run,
    /// else the next measurement a period later. A bus failure of kind [`ErrorKind::Other`] never reached the part,
    /// so it is not counted and leaves the run as it was (the wire rule).
    fn failed<BusError: Error>(&mut self, fault: Fault<BusError>, now_us: u64) -> Poll<BusError> {
        let recovering = match &fault {
            Fault::Bus(error) if refused(error) => false,
            _ => self.strikes.strike(true, RECOVER_AFTER),
        };
        self.phase = if recovering {
            Phase::Draining {
                until: now_us + self.period_us(),
            }
        } else {
            Phase::Idle {
                next: now_us + self.period_us(),
            }
        };
        Poll::Failed { fault, recovering }
    }

    /// Apply an operator property change (icp10111.update): `{"ground": <m>}` SETS the zero directly (the warm
    /// start rebases a rebooted barometer to the crumb's pad altitude: a mid-air re-zero would read ~0 at altitude and
    /// fake an immediate landing); `{"rezero": true}` re-captures it from `fresh_altitude`, the channel's own value
    /// when fresher than three periods (None: [`Refusal::NoFreshAltitude`]). Returns the changed property names.
    ///
    /// `ground` is read as plan B's `float()` reads it ([`python_float`]): a number, a numeral string such as
    /// "12.5", or a bool; anything else is [`Refusal::NotANumber`] and leaves the ground as it was.
    pub fn update(&mut self, properties: &Value, fresh_altitude: Option<f32>) -> Result<Vec<&'static str>, Refusal> {
        if let Some(ground) = properties.get("ground") {
            self.ground = Some(python_float(ground).ok_or(Refusal::NotANumber)?);
            return Ok(vec!["ground"]);
        }
        if properties.get("rezero").is_some_and(truthy) {
            self.ground = Some(fresh_altitude.ok_or(Refusal::NoFreshAltitude)?);
            return Ok(vec!["ground"]);
        }
        Ok(Vec::new())
    }

    /// Classify a failed setup (icp10111.diagnose): re-issue the product-id command and sort the masked word with
    /// [`id_classify`] (command-based, so not the bus's register diagnosis, but the same categories).
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String {
        let read = self.identity_word(bus).ok().map(|word| (word & IDENTITY_MASK) as u8);
        id_classify(read, IDENTITY as u8)
    }
}

/// Whether a bus failure is the port's, not the wire's: [`ErrorKind::Other`], the firmware's `BusBusy` or a driver
/// refusal, which never reached the part (the wire rule, in the module notes).
fn refused<BusError: Error>(error: &BusError) -> bool {
    error.kind() == ErrorKind::Other
}

/// Python's truthiness of a JSON value, for the `rezero` flag: false, null, 0, "" and empty containers are false.
fn truthy(value: &Value) -> bool {
    match value {
        Value::Null => false,
        Value::Bool(flag) => *flag,
        Value::Number(number) => number.as_f64().is_some_and(|number| number != 0.0),
        Value::String(text) => !text.is_empty(),
        Value::Array(items) => !items.is_empty(),
        Value::Object(members) => !members.is_empty(),
    }
}

impl<Bus: I2c> Device<Bus> for Icp10111 {
    type BusError = Bus::Error;

    /// The hardened first touch, the product id, the OTP, then the ground zero (icp10111.setup's bus half).
    ///
    /// The first touch is the id command and its read, up to [`RESET_TRIES`] times [`RESET_RETRY_MS`] apart: the
    /// first NAK only waits (a conversion may be draining), each later one fires the general call first. The id read
    /// after it reports a part that is truly absent. A part that half-recovers -- id and OTP ack, the first measure
    /// NAKs, the post-latch-up state that needs a power cycle -- fails here, gracefully. Ask
    /// [`Icp10111::take_general_call`] afterwards, whatever the outcome.
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        for attempt in 0..RESET_TRIES {
            if self.identity_word(bus).is_ok() || attempt == RESET_TRIES - 1 {
                break;
            }
            if attempt > 0 {
                self.general_call(bus);
            }
            delay.delay_ms(RESET_RETRY_MS);
        }
        let word = self.identity_word(bus).map_err(Fault::Bus)?;
        if word & IDENTITY_MASK != IDENTITY {
            return Err(Fault::Identity {
                expected: IDENTITY,
                read: word & IDENTITY_MASK,
            });
        }
        bus.write(self.address, &OTP_UNLOCK).map_err(Fault::Bus)?;
        let mut otp: Otp = [0; OTP_WORDS];
        for constant in otp.iter_mut() {
            bus.write(self.address, &OTP_READ).map_err(Fault::Bus)?;
            let mut word = [0u8; WORD_LENGTH];
            bus.read(self.address, &mut word).map_err(Fault::Bus)?;
            *constant = i16::from_be_bytes([word[0], word[1]]);
        }
        self.otp = Some(otp);
        self.phase = Phase::Idle { next: 0 };
        self.strikes = Strikes::default();
        self.capture_ground(bus, delay)?;
        Ok(())
    }

    /// Nothing to write: the part is command-based, its OTP constants are kept here, and its power-on state is the
    /// idle state it measures from. A general call does abort a conversion in flight, so the run loop drops it and
    /// issues afresh on its next poll.
    fn rearm(&mut self, _bus: &mut Bus, _delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        if matches!(self.phase, Phase::Converting { .. }) {
            self.phase = Phase::Idle { next: 0 };
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
