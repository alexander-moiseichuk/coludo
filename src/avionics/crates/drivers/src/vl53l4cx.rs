//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The VL53L4CX time-of-flight laser ranger, Adafruit 5425 (src/glider/drivers/vl53l4cx.py): the newer 0xEBAA
//! silicon shared by the VL53L4CD and L4CX, which takes the VL53L4CD Ultra-Lite-Driver init -- the older VL53L1X
//! (0xEACC) configuration does NOT produce ranges on it. Single-target distance; the L4CX multi-target extras are
//! unused.
//!
//! It shares everything else with the VL53L1X -- the register map, the boot wait, the VHV calibration cycle, the
//! range read with its data-ready check, the re-arm -- so the driver is [`crate::vl53l1x::Vl53`] with this module's
//! [`VL53L4CX`] silicon. What is this part's own: its model id, its default configuration block, its start value
//! (0x21), and the timing budget ([`timing_budget`]) setup programs after the VHV writes.
//!
//! THE TIMING BUDGET is the ULD's integer math, kept integer: its 2^30 term overflows a single-precision float (the
//! reason plan B kept it integer on MicroPython), and here i128 keeps any configured budget exact as Python's
//! integers did. The macro period comes from the part's measured oscillator frequency; the budget minus a fixed
//! 2.5 ms overhead (continuous mode, inter-measurement 0) is spread over the macro periods of RANGE_CONFIG A (x16)
//! and B (x12), each written as an 8-bit mantissa and an exponent byte.

use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;

use crate::device::Fault;
use crate::vl53l1x::{DEFAULT_CONFIGURATION_LENGTH, Silicon, Vl53};

/// The VL53L4CD default configuration, registers 0x2D..=0x87: the ST/Adafruit ULD block for the 0xEBAA silicon;
/// ranging is started separately (SYSTEM__MODE_START is 0x00 here).
pub const DEFAULT_CONFIGURATION: [u8; DEFAULT_CONFIGURATION_LENGTH] = [
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, // 0x2d..0x3c
    0x00, 0xff, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x14, 0x21, // 0x3d..0x4c
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00, // 0x4d..0x5c
    0x00, 0x01, 0xcc, 0x07, 0x01, 0xf1, 0x05, 0x00, 0xa0, 0x00, 0x80, 0x08, 0x38, 0x00, 0x00, 0x00, // 0x5d..0x6c
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00, // 0x6d..0x7c
    0x00, 0x02, 0xc7, 0xff, 0x9b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, // 0x7d..0x87
];

/// The VL53L4CD/L4CX: model id 0xEBAA, the ULD block above, start 0x21, and the timing budget.
pub const VL53L4CX: Silicon = Silicon {
    name: "vl53l4cx",
    model_id: 0xEBAA,
    default_configuration: &DEFAULT_CONFIGURATION,
    start_continuous: 0x21,
    timing_budget: true,
};

/// OSC_MEASURED__FAST_OSC__FREQUENCY: two bytes, the part's measured oscillator frequency; 0 leaves the budget
/// alone.
pub const OSC_MEASURED_FAST_OSC_FREQUENCY: u16 = 0x0006;

/// The register written [0x05, 0x00] before the timing budget: the VL53L4CD ULD's raw `WrWord(0x0024, 0x500)`,
/// which it names no further.
pub const REGISTER_0X0024: u16 = 0x0024;

/// The word written to [`REGISTER_0X0024`].
pub const REGISTER_0X0024_VALUE: [u8; 2] = [0x05, 0x00];

/// RANGE_CONFIG__TIMEOUT_MACROP_A: the A-phase integration time, as a mantissa and an exponent byte.
pub const RANGE_CONFIG_TIMEOUT_MACROP_A: u16 = 0x005E;

/// RANGE_CONFIG__TIMEOUT_MACROP_B: the B-phase integration time, as a mantissa and an exponent byte.
pub const RANGE_CONFIG_TIMEOUT_MACROP_B: u16 = 0x0061;

/// The fixed overhead of a continuous-mode measurement, us, taken off the budget.
const CONTINUOUS_OVERHEAD_US: i128 = 2_500;

/// The macro period, in us scaled by 2^12 (the oscillator register is MHz in 4.12 fixed point, and the budget is
/// scaled to match): `(2304 x 2^30 // oscillator) >> 6`, multiplied before dividing as plan B does it. None for an
/// oscillator of 0, where plan B writes nothing.
pub fn macro_period(oscillator: u16) -> Option<i128> {
    (oscillator != 0).then(|| (2304 * (1i128 << 30) / i128::from(oscillator)) >> 6)
}

/// The RANGE_CONFIG A and B words for a budget (vl53l4cx._set_timing_budget's arithmetic), in write order, or None
/// for an oscillator of 0.
///
/// Python's floor division and arithmetic shifts are reproduced for any integer budget, a negative or sub-overhead
/// one included (its mantissa is then the low byte of a negative count, as Python's `& 0xFF` takes it).
pub fn timing_budget(oscillator: u16, budget_ms: i64) -> Option<[(u16, u16); 2]> {
    let macro_period = macro_period(oscillator)?;
    let budget_us = (i128::from(budget_ms) * 1000 - CONTINUOUS_OVERHEAD_US) << 12;
    let word = |multiplier: i128| -> u16 {
        let denominator = (macro_period * multiplier) >> 6;
        let mut mantissa = (budget_us + (denominator >> 1)).div_euclid(denominator) - 1;
        let mut exponent: i128 = 0;
        while mantissa > 0xFF {
            mantissa >>= 1;
            exponent += 1;
        }
        // the mantissa starts below 2^127, so the exponent stays under 127 and the word fits 16 bits
        ((exponent << 8) + (mantissa & 0xFF)) as u16
    };
    Some([
        (RANGE_CONFIG_TIMEOUT_MACROP_A, word(16)),
        (RANGE_CONFIG_TIMEOUT_MACROP_B, word(12)),
    ])
}

/// Program the timing budget the L4CX setup writes after the VHV writes: [`REGISTER_0X0024`], then the oscillator
/// read, then RANGE_CONFIG A and B (nothing more when the oscillator reads 0).
pub(crate) fn program_timing_budget<Bus: I2c, Shutdown: OutputPin>(
    ranger: &Vl53<Shutdown>,
    bus: &mut Bus,
    budget_ms: i64,
) -> Result<(), Fault<Bus::Error>> {
    ranger
        .write_register(bus, REGISTER_0X0024, &REGISTER_0X0024_VALUE)
        .map_err(Fault::Bus)?;
    let oscillator = ranger
        .read_word(bus, OSC_MEASURED_FAST_OSC_FREQUENCY)
        .map_err(Fault::Bus)?;
    for (register, word) in timing_budget(oscillator, budget_ms).into_iter().flatten() {
        ranger
            .write_register(bus, register, &word.to_be_bytes())
            .map_err(Fault::Bus)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests;
