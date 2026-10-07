//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The VL53L1X time-of-flight laser ranger (src/glider/drivers/vl53l1x.py), and the VL53 family machinery it shares
//! with the VL53L4CX ([`crate::vl53l4cx`]): the above-ground-level (AGL) channel for the last metres of the glide,
//! where the barometer is useless. The family uses 16-BIT register addresses.
//!
//! ONE SOCKET, TWO SILICONS: the VL53L1X (model id 0xEACC, the VL53L1X Ultra-Lite-Driver init) and the
//! VL53L4CD/L4CX (0xEBAA, the VL53L4CD ULD init) both answer at 0x29, and neither configuration produces ranges on
//! the other part. An I2C scan cannot tell them apart, so each [`Silicon`] checks its own model id strictly and
//! refuses the other ([`Fault::Identity`]): a board declares both, and the fitted one wins, exactly as an absent
//! device is skipped. If that check ever loosened, the wrong init block would be written to a real part: it acks,
//! it configures, and it silently never ranges.
//!
//! RANGE IS THE REASON TO CARE WHICH IS FITTED: the L1X is declared 2-4 m against the L4CX's 4-6 m, and the landing
//! trigger (`land_agl_m`) defaults to 5.0 m -- unreachable by an L1X, so the transition would silently fall back to
//! barometric elevation, the thing the laser exists to replace. A board fitted with this part wants `land_agl_m`
//! around 3.0 (doc/hardware.md).
//!
//! The setup ([`Device::setup`]): an optional XSHUT reset, the firmware boot wait, the model id, the default
//! configuration block, one VHV calibration ranging cycle (start, wait, clear, stop, then the VHV writes), the
//! L4CX's timing budget, and continuous ranging. NO TIMING BUDGET IS SET on the L1X: the L4CX's RANGE_CONFIG math is
//! the VL53L4CD ULD's and does not apply to this silicon, whose ULD uses a table keyed on distance mode instead; the
//! configuration block's own timing stands until that table is checked against the part.
//!
//! DATA-READY BEFORE EVERY PUBLISH ([`Vl53::range`]): plan B read the status and the distance whenever its loop woke,
//! so in poll mode a range already published was read and published again as fresh. Here a range is read only when
//! GPIO__TIO_HV_STATUS says a new one is waiting (avionics-port.md §6, changed on purpose).
//!
//! The re-arm runs the whole setup again (XSHUT pulse included when wired): plan B had no re-arm for either part,
//! and a re-initialised ranger is the safe state after a general call whether or not the part honoured it.
//!
//! Not here (firmware glue): the GPIO1 data-ready interrupt and its `irq_runs` count, the channel push, the probe's
//! log lines and the inspect JSON.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use serde_json::Value;

use crate::commons::id_classify;
use crate::device::{Device, Fault};

/// The default I2C address of both silicons.
pub const ADDRESS: u8 = 0x29;

/// FIRMWARE__SYSTEM_STATUS: bit 0 set once the part's firmware has booted.
pub const FIRMWARE_SYSTEM_STATUS: u16 = 0x00E5;

/// IDENTIFICATION__MODEL_ID: two bytes, the silicon's model id.
pub const IDENTIFICATION_MODEL_ID: u16 = 0x010F;

/// The first register of the default configuration block (0x2D..=0x87).
pub const DEFAULT_CONFIGURATION_START: u16 = 0x002D;

/// The default configuration block's length: registers 0x2D..=0x87.
pub const DEFAULT_CONFIGURATION_LENGTH: usize = 91;

/// GPIO_HV_MUX__CTRL: bit 4 selects the data-ready interrupt polarity.
pub const GPIO_HV_MUX_CTRL: u16 = 0x0030;

/// GPIO__TIO_HV_STATUS: bit 0 equals the interrupt polarity while a measurement is waiting.
pub const GPIO_TIO_HV_STATUS: u16 = 0x0031;

/// SYSTEM__INTERRUPT_CLEAR: writing 1 releases the interrupt, which arms the next measurement.
pub const SYSTEM_INTERRUPT_CLEAR: u16 = 0x0086;

/// SYSTEM__MODE_START: the start value starts continuous ranging, 0x00 stops it.
pub const SYSTEM_MODE_START: u16 = 0x0087;

/// RESULT__RANGE_STATUS: the low 5 bits are the device range status, mapped through [`STATUS_RTN`].
pub const RESULT_RANGE_STATUS: u16 = 0x0089;

/// RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: two bytes, the distance in mm.
pub const RESULT_DISTANCE: u16 = 0x0096;

/// VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, written 0x09 after the calibration cycle (both ULDs).
pub const VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND: u16 = 0x0008;

/// VHV_CONFIG__INIT, written 0x00 after the calibration cycle: start the VHV from the last temperature (both ULDs).
pub const VHV_CONFIG_INIT: u16 = 0x000B;

/// How many 1 ms polls the firmware has to boot.
pub const BOOT_TIMEOUT_MS: u32 = 100;

/// How many 1 ms polls the VHV calibration ranging cycle has: it can take tens of ms.
pub const RANGE_TIMEOUT_MS: u32 = 200;

/// How long XSHUT is held low, and how long the part is left after it goes high (and with no XSHUT at all).
pub const XSHUT_MS: u32 = 2;

/// The default sampling period, ms: the interrupt-silent fallback cadence (plan B once read a `period_us` key no
/// config supplies, and every laser fell back to 500 ms against a 100 ms freshness window).
pub const DEFAULT_PERIOD_MS: u32 = 50;

/// The raw range status (RESULT__RANGE_STATUS & 0x1F) mapped to the ULD status; 0 is a valid measurement, 255 a
/// reserved code. The two ULDs share it.
pub const STATUS_RTN: [u8; 24] = [
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12,
];

/// The VL53L1X default configuration, registers 0x2D..=0x87: ST's canonical ULD array.
///
/// DO NOT hand-edit or re-transcribe this. The version plan B carried in its own history was WRONG in three ways and
/// the part simply never ranged: 0x2E/0x2F were 0x00 instead of 0x01 (I2C and GPIO pulled up at AVDD, not 1.8 V),
/// 0x5F was 0xCC instead of 0xDB, and -- the one that mattered -- a single 0x00 was missing around 0x76, so every
/// register from there on was written ONE ADDRESS LOW. Measured on hardware 2026-09-20: with this array the part
/// reports data-ready in 7 ms; with the historical one, no measurement completes in 3000 ms. Confirmed ranging a
/// palm on the sensor (8-34 mm at status 0) and 68 of 70 samples valid against a pale target at 1.0-1.5 m. The
/// golden fixture drivers_vl53.json pins it byte for byte against plan B's.
///
/// Register 0x87 (SYSTEM__MODE_START) is included and is 0x00: ranging is stopped here and started explicitly.
pub const DEFAULT_CONFIGURATION: [u8; DEFAULT_CONFIGURATION_LENGTH] = [
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, // 0x2d..0x36
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, // 0x37..0x40
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, // 0x41..0x4a
    0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, // 0x4b..0x54
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, // 0x55..0x5e
    0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08, // 0x5f..0x68
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, // 0x69..0x72
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, // 0x73..0x7c
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, // 0x7d..0x86
    0x00, // 0x87 SYSTEM__MODE_START: stopped
];

/// What tells one silicon of the family from the other: its id, its init block, its start value and whether it
/// takes the timing budget.
#[derive(Debug, PartialEq, Eq)]
pub struct Silicon {
    /// The driver name a board config selects it by (`vl53l1x`, `vl53l4cx`).
    pub name: &'static str,
    /// IDENTIFICATION__MODEL_ID's 16 bits for this silicon.
    pub model_id: u16,
    /// The default configuration block, registers 0x2D..=0x87.
    pub default_configuration: &'static [u8; DEFAULT_CONFIGURATION_LENGTH],
    /// The SYSTEM__MODE_START value that starts continuous ranging (the L1X 0x40, the L4CD/L4CX 0x21).
    pub start_continuous: u8,
    /// Whether setup programs the timing budget (the L4CX's RANGE_CONFIG math; see the module notes).
    pub timing_budget: bool,
}

/// The VL53L1X: model id 0xEACC (its high byte 0xEA tells it from the L4CX's 0xEB), the ULD block above, start
/// 0x40, no timing budget.
pub const VL53L1X: Silicon = Silicon {
    name: "vl53l1x",
    model_id: 0xEACC,
    default_configuration: &DEFAULT_CONFIGURATION,
    start_continuous: 0x40,
    timing_budget: false,
};

/// One range read ([`Vl53::range`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ranging {
    /// No new measurement is waiting: nothing was read, nothing is published.
    NotReady,
    /// A measurement with a non-zero ULD status (out of range, low signal): read and cleared, never published.
    Invalid {
        /// The ULD status ([`STATUS_RTN`]).
        status: u8,
    },
    /// A valid measurement.
    Valid {
        /// The distance, mm.
        distance_mm: u16,
    },
}

impl Ranging {
    /// The above-ground level in metres a valid range publishes (plan B's `distance_mm / 1000.0`), else None.
    pub fn agl(self) -> Option<f32> {
        match self {
            Ranging::Valid { distance_mm } => Some(f32::from(distance_mm) / 1000.0),
            _ => None,
        }
    }
}

/// The ULD status of a raw RESULT__RANGE_STATUS byte: its low 5 bits through [`STATUS_RTN`], 255 past the table.
pub fn range_status(raw: u8) -> u8 {
    STATUS_RTN.get(usize::from(raw & 0x1F)).copied().unwrap_or(255)
}

/// The data-ready polarity GPIO_HV_MUX__CTRL selects: bit 4 set means active-low data-ready (0), clear active-high
/// (1).
pub fn polarity(gpio_hv_mux_ctrl: u8) -> u8 {
    if gpio_hv_mux_ctrl & 0x10 != 0 { 0 } else { 1 }
}

/// A VL53 ranger's settings, from its config block (the keys plan B's setup() reads, with its defaults).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// The I2C address (`addr`, default [`ADDRESS`]).
    pub address: u8,
    /// The sampling period, ms, at least 1 (`period_ms`, default [`DEFAULT_PERIOD_MS`]).
    pub period_ms: u32,
    /// The ranging integration time, ms (`timing_budget_ms`, default 50; the L4CX only): longer is a lower sigma
    /// and a longer range.
    pub timing_budget_ms: i64,
}

/// The default timing budget, ms (the L4CX only).
pub const DEFAULT_TIMING_BUDGET_MS: i64 = 50;

impl Settings {
    /// Read a config block; a missing or non-integer key takes plan B's default.
    pub fn from_config(config: &Value) -> Settings {
        Settings {
            address: config
                .get("addr")
                .and_then(Value::as_u64)
                .and_then(|address| u8::try_from(address).ok())
                .unwrap_or(ADDRESS),
            period_ms: config
                .get("period_ms")
                .and_then(Value::as_i64)
                .map_or(DEFAULT_PERIOD_MS, |period| period.clamp(1, i64::from(u32::MAX)) as u32),
            timing_budget_ms: config
                .get("timing_budget_ms")
                .and_then(Value::as_i64)
                .unwrap_or(DEFAULT_TIMING_BUDGET_MS),
        }
    }
}

/// One VL53-family ranger on a bus: which silicon it expects, and its XSHUT line if one is wired.
#[derive(Debug)]
pub struct Vl53<Shutdown> {
    /// The silicon this driver initialises; any other answers [`Fault::Identity`].
    silicon: &'static Silicon,
    /// The I2C address.
    address: u8,
    /// The XSHUT line (active-low shutdown), None when the part is always on.
    xshut: Option<Shutdown>,
    /// The timing budget, ms (the L4CX only).
    timing_budget_ms: i64,
    /// The data-ready polarity setup read (see [`polarity`]).
    polarity: u8,
}

impl<Shutdown: OutputPin> Vl53<Shutdown> {
    /// A driver for `silicon` with these settings and, when wired, its XSHUT line; nothing is touched until setup.
    pub fn new(silicon: &'static Silicon, settings: &Settings, xshut: Option<Shutdown>) -> Vl53<Shutdown> {
        Vl53 {
            silicon,
            address: settings.address,
            xshut,
            timing_budget_ms: settings.timing_budget_ms,
            polarity: 1,
        }
    }

    /// The silicon this driver expects.
    pub fn silicon(&self) -> &'static Silicon {
        self.silicon
    }

    /// The I2C address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Whether an XSHUT line is wired.
    pub fn has_xshut(&self) -> bool {
        self.xshut.is_some()
    }

    /// Read `buffer.len()` bytes from a 16-bit register.
    fn read_register<Bus: I2c>(&self, bus: &mut Bus, register: u16, buffer: &mut [u8]) -> Result<(), Bus::Error> {
        bus.write_read(self.address, &register.to_be_bytes(), buffer)
    }

    /// Read one byte from a 16-bit register.
    fn read_byte<Bus: I2c>(&self, bus: &mut Bus, register: u16) -> Result<u8, Bus::Error> {
        let mut byte = [0u8; 1];
        self.read_register(bus, register, &mut byte)?;
        Ok(byte[0])
    }

    /// Read a big-endian word from a 16-bit register.
    pub(crate) fn read_word<Bus: I2c>(&self, bus: &mut Bus, register: u16) -> Result<u16, Bus::Error> {
        let mut word = [0u8; 2];
        self.read_register(bus, register, &mut word)?;
        Ok(u16::from_be_bytes(word))
    }

    /// Write bytes from a 16-bit register on, in one transaction.
    pub(crate) fn write_register<Bus: I2c>(&self, bus: &mut Bus, register: u16, data: &[u8]) -> Result<(), Bus::Error> {
        let mut message = [0u8; 2 + DEFAULT_CONFIGURATION_LENGTH];
        message[..2].copy_from_slice(&register.to_be_bytes());
        message[2..2 + data.len()].copy_from_slice(data);
        bus.write(self.address, &message[..2 + data.len()])
    }

    /// Pulse XSHUT low then high (if wired), then wait for the firmware to boot (vl53l1x._reset).
    ///
    /// Recovers a wedged ranger without a board reboot. [`Fault::Timeout`] when the firmware never boots: a
    /// dead-firmware part still acks its hard-silicon model id, so setup must gate on this to avoid a false-present
    /// detection.
    fn reset<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        if let Some(xshut) = self.xshut.as_mut() {
            xshut.set_low().map_err(|_| Fault::Pin)?;
            delay.delay_ms(XSHUT_MS);
            xshut.set_high().map_err(|_| Fault::Pin)?;
        }
        delay.delay_ms(XSHUT_MS);
        for _ in 0..BOOT_TIMEOUT_MS {
            if self.read_byte(bus, FIRMWARE_SYSTEM_STATUS).map_err(Fault::Bus)? & 0x01 != 0 {
                return Ok(());
            }
            delay.delay_ms(1);
        }
        Err(Fault::Timeout)
    }

    /// Whether a measurement is waiting: GPIO__TIO_HV_STATUS bit 0 equals the polarity.
    pub fn data_ready<Bus: I2c>(&self, bus: &mut Bus) -> Result<bool, Fault<Bus::Error>> {
        Ok(self.read_byte(bus, GPIO_TIO_HV_STATUS).map_err(Fault::Bus)? & 0x01 == self.polarity)
    }

    /// Poll data-ready once a millisecond, at most `timeout_ms` times (vl53l1x._await_ready). Returns either way:
    /// a calibration cycle that never reports ready does not fail setup, as in plan B.
    fn await_ready<Bus: I2c>(
        &self,
        bus: &mut Bus,
        delay: &mut dyn DelayNs,
        timeout_ms: u32,
    ) -> Result<(), Fault<Bus::Error>> {
        for _ in 0..timeout_ms {
            if self.data_ready(bus)? {
                return Ok(());
            }
            delay.delay_ms(1);
        }
        Ok(())
    }

    /// Write one byte to a 16-bit register.
    fn write_byte<Bus: I2c>(&self, bus: &mut Bus, register: u16, value: u8) -> Result<(), Fault<Bus::Error>> {
        self.write_register(bus, register, &[value]).map_err(Fault::Bus)
    }

    /// Read the latest measurement, if one is waiting, and release the interrupt (vl53l1x._range_locked, behind the
    /// data-ready check plan B lacked).
    ///
    /// Status, distance, then the interrupt clear that arms the next sample: three transactions the owner thread
    /// runs back to back (plan B serialised them with a claim, after a diagnostic interleaving a conversation on
    /// the ICP invented a 20.8 % failure rate).
    pub fn range<Bus: I2c>(&self, bus: &mut Bus) -> Result<Ranging, Fault<Bus::Error>> {
        if !self.data_ready(bus)? {
            return Ok(Ranging::NotReady);
        }
        let raw = self.read_byte(bus, RESULT_RANGE_STATUS).map_err(Fault::Bus)? & 0x1F;
        let distance_mm = self.read_word(bus, RESULT_DISTANCE).map_err(Fault::Bus)?;
        self.write_byte(bus, SYSTEM_INTERRUPT_CLEAR, 0x01)?;
        Ok(match range_status(raw) {
            0 => Ranging::Valid { distance_mm },
            status => Ranging::Invalid { status },
        })
    }

    /// Read the model id; [`Fault::Identity`] when it is not this silicon's (the check behind [`Vl53::probe`], and
    /// setup's id gate).
    ///
    /// The full 16 bits for both silicons. The L1X probe once checked one byte against 0xEB -- the L4CX's id, copied
    /// from that driver -- so a healthy L1X failed its own probe and `verify`/`arm` refused every board carrying
    /// one. (Plan B's L4CX probe still reads the high byte alone; the full word gives the same verdict on both real
    /// silicons and refuses a part that merely shares the byte.)
    pub fn identify<Bus: I2c>(&self, bus: &mut Bus) -> Result<(), Fault<Bus::Error>> {
        let read = self.read_word(bus, IDENTIFICATION_MODEL_ID).map_err(Fault::Bus)?;
        if read == self.silicon.model_id {
            Ok(())
        } else {
            Err(Fault::Identity {
                expected: self.silicon.model_id,
                read,
            })
        }
    }

    /// The on-demand self-test (vl53l1x.probe, vl53l4cx.probe): the model id reads back as this silicon's, all 16
    /// bits in one register read; answers that id, for the owner's 'probe: model id ok' line.
    ///
    /// Only the id: the range is legitimately empty with no target in range, so plan B never checked it
    /// (vl53l1x.py:281-307, vl53l4cx.py:277-301). One transaction, so it never splits the run loop's
    /// status-distance-clear sequence. The L4CX's full word is a deliberate divergence (see [`Vl53::identify`]):
    /// plan B's vl53l4cx.probe reads the high byte 0xEB alone (vl53l4cx.py:292-293), and drivers_vl53.json owes the
    /// port's 16-bit read instead. Its log text follows from that read: the L4CX's ok and FAILED lines print the
    /// full word (0xebaa) where plan B's printed the byte ('probe: model id ok 0x%02x' and 'VL53L4CX id 0x%02x !=
    /// 0xEB at i2c:%s 0x%02x', vl53l4cx.py:294-296), as the L1X's always did (0x%04x, vl53l1x.py:300-302).
    pub fn probe<Bus: I2c>(&self, bus: &mut Bus) -> Result<u16, Fault<Bus::Error>> {
        self.identify(bus)?;
        Ok(self.silicon.model_id)
    }

    /// Classify a failed setup at the wire level (vl53l1x.diagnose through i2cbus): read the model id's high byte
    /// (0xEA for the L1X, 0xEB for the L4CX) and sort it with [`id_classify`]; the other silicon in the socket reads
    /// as a 'wrong device'.
    pub fn diagnose<Bus: I2c>(&self, bus: &mut Bus) -> String {
        let read = self.read_byte(bus, IDENTIFICATION_MODEL_ID).ok();
        id_classify(read, self.silicon.model_id.to_be_bytes()[0])
    }

    /// The whole bring-up (vl53l1x.setup / vl53l4cx.setup's bus half): reset and boot, model id, default
    /// configuration, polarity, the VHV calibration cycle, the timing budget (the L4CX), and continuous ranging.
    fn initialise<Bus: I2c>(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.reset(bus, delay)?;
        self.identify(bus)?;
        self.write_register(bus, DEFAULT_CONFIGURATION_START, self.silicon.default_configuration)
            .map_err(Fault::Bus)?;
        self.polarity = polarity(self.read_byte(bus, GPIO_HV_MUX_CTRL).map_err(Fault::Bus)?);
        self.write_byte(bus, SYSTEM_MODE_START, self.silicon.start_continuous)?;
        self.await_ready(bus, delay, RANGE_TIMEOUT_MS)?;
        self.write_byte(bus, SYSTEM_INTERRUPT_CLEAR, 0x01)?;
        self.write_byte(bus, SYSTEM_MODE_START, 0x00)?;
        self.write_byte(bus, VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)?;
        self.write_byte(bus, VHV_CONFIG_INIT, 0x00)?;
        if self.silicon.timing_budget {
            crate::vl53l4cx::program_timing_budget(self, bus, self.timing_budget_ms)?;
        }
        self.write_byte(bus, SYSTEM_MODE_START, self.silicon.start_continuous)
    }
}

impl<Bus: I2c, Shutdown: OutputPin> Device<Bus> for Vl53<Shutdown> {
    type BusError = Bus::Error;

    /// The whole bring-up; the other silicon in the socket answers [`Fault::Identity`] and is left untouched past
    /// its id read.
    fn setup(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.initialise(bus, delay)
    }

    /// The whole bring-up again (see the module notes).
    fn rearm(&mut self, bus: &mut Bus, delay: &mut dyn DelayNs) -> Result<(), Fault<Bus::Error>> {
        self.initialise(bus, delay)
    }
}

#[cfg(test)]
mod tests;
