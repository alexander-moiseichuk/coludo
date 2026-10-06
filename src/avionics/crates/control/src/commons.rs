//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The shared primitives of the flight math (src/glider commons.py): the parts the ported modules use, each
//! next to the plan B name it ports. Each port adds what it needs here, so commons stays one module, as in plan B.
//!
//! Plan B's `_upy`/`_opt` variant pairs (bytecode against viper or native) are a MicroPython speed workaround and
//! are not ported: each primitive is one plain function.
//!
//! * [`between`] -- plan B's clamp, with Python's comparison semantics (it never panics, unlike `f32::clamp`).
//! * [`SERVO_NEUTRAL_DEGREES`] -- the zero-deflection fin angle the mixer centres on.
//! * [`fin_deflection_limit`] -- the dynamic-pressure fin governor's table (doc/specs/coludo.md "Fin
//!   authority"): the largest fin deflection the airframe takes at a given airspeed.
//! * [`altitude_m`] -- the barometric altitude every baro driver publishes, from [`SEA_LEVEL_PA`].
//!
//! The stage machine's half of commons.py (`dwell_step`, `apogee_step`) is in crates/sequencer, module `commons`.

/// The default fin/servo neutral angle in degrees (commons.SERVO_NEUTRAL_DEG): the zero-deflection centre the
/// mixer holds when disarmed or degraded, and the one the HITL sim assumes when it recovers fin deflections. A
/// board may override it through the mixer config `neutral_deg`.
pub const SERVO_NEUTRAL_DEGREES: i32 = 90;

/// The airspeed, in whole m/s, where the fin table saturates (commons._FIN_VMAX): well past any speed the
/// glider flies; every faster speed takes the last entry.
pub const FIN_TABLE_TOP_SPEED: usize = 80;

/// The fin table's constant K, in degrees x (m/s)^2: the limit is K / v^2, and 12 500 anchors 50 m/s to 5 degrees.
const FIN_DEFLECTION_CONSTANT: u32 = 12_500;

/// The fin table's ceiling in degrees: the fins' mechanical throw, and every entry below 17 m/s.
const FIN_DEFLECTION_MAXIMUM: u32 = 45;

/// The fin table's floor in degrees: always some authority, however fast.
const FIN_DEFLECTION_MINIMUM: u32 = 5;

/// The fin deflection limit in whole degrees for each whole airspeed 0..=80 m/s (commons._FIN_LIMIT).
///
/// Aero torque scales with the dynamic pressure q, which goes as v^2, so a fixed fin angle is too weak slow and
/// too violent fast. The limit goes as 1/v^2 to hold the angular authority about constant, clamped to [5, 45]
/// degrees; 0 m/s takes the full 45. Precomputed once (plan B did it at import, to keep the 1/v^2 off the
/// 100 Hz path), and here at compile time.
const FIN_LIMIT: [i32; FIN_TABLE_TOP_SPEED + 1] = fin_limit_table();

/// Build [`FIN_LIMIT`]: `45 if v == 0 else min(45, max(5, round(12500 / v^2)))`, as plan B does.
///
/// Python's `round()` rounds a half to even. The integer form here rounds the exact quotient the same way,
/// with no float at all. No quotient 12 500 / v^2 is a tie, and none lies closer than 1/12 800 to one (its
/// fraction is a multiple of 1/v^2), so it equals plan B's f64 quotient rounded, entry for entry (the golden
/// fixture fin_limit.json checks all 81).
const fn fin_limit_table() -> [i32; FIN_TABLE_TOP_SPEED + 1] {
    let mut table = [FIN_DEFLECTION_MAXIMUM as i32; FIN_TABLE_TOP_SPEED + 1];
    let mut speed = 1;
    while speed <= FIN_TABLE_TOP_SPEED {
        let square = (speed * speed) as u32;
        let quotient = FIN_DEFLECTION_CONSTANT / square;
        let twice_remainder = 2 * (FIN_DEFLECTION_CONSTANT % square);
        let rounded = if twice_remainder > square || (twice_remainder == square && quotient % 2 == 1) {
            quotient + 1
        } else {
            quotient
        };
        let limited = if rounded > FIN_DEFLECTION_MAXIMUM {
            FIN_DEFLECTION_MAXIMUM
        } else if rounded < FIN_DEFLECTION_MINIMUM {
            FIN_DEFLECTION_MINIMUM
        } else {
            rounded
        };
        table[speed] = limited as i32;
        speed += 1;
    }
    table
}

/// The largest fin deflection in whole degrees from neutral for an airspeed in m/s (commons.fin_deflection_limit):
/// the dynamic-pressure governor's table lookup.
///
/// The speed truncates toward zero to a whole m/s, as plan B's `int()` does, so 29.99 m/s reads the 29 m/s entry;
/// a negative speed reads the 0 m/s entry and anything past 80 m/s the last one. The board's
/// `fins.limit_multiplier` safety dial is the caller's to apply (the governor), never baked into the table.
///
/// NaN, which no airspeed estimate produces (the estimator's bound turns it into 0), reads the TIGHTEST entry:
/// plan B's `int()` raises there instead, and of the two directions an over-read is the safe one (a high airspeed
/// tightens the cap). Infinities saturate to the matching end.
pub fn fin_deflection_limit(speed: f32) -> i32 {
    if speed.is_nan() {
        return FIN_LIMIT[FIN_TABLE_TOP_SPEED];
    }
    let index = (speed as i64).clamp(0, FIN_TABLE_TOP_SPEED as i64);
    FIN_LIMIT[index as usize]
}

/// Clamp `value` to [`low`, `high`] (commons.between): `low` if below, `high` if above, else `value`.
///
/// Python's comparisons, in plan B's order: unlike `f32::clamp`, it never panics (a NaN bound or `low > high`
/// simply answers by the comparisons), and a NaN `value` passes through. With `low = -x, high = x` it is a
/// symmetric clamp; an infinite bound leaves that side open.
pub fn between(low: f32, value: f32, high: f32) -> f32 {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}

/// The ISA sea-level pressure in Pa (commons.SEA_LEVEL_PA): the datum of [`altitude_m`], one definition for every
/// baro driver, so two of them cannot disagree about it.
pub const SEA_LEVEL_PA: f32 = 101_325.0;

/// The exponent of the ISA altitude formula, 1/5.255 (plan B writes 0.190294957; this is its nearest f32).
const ALTITUDE_EXPONENT: f32 = 0.190_294_95;

/// The ISA altitude formula's scale in metres: the height of the troposphere's linear temperature model.
const ALTITUDE_SCALE_M: f32 = 44_330.0;

/// The barometric altitude in metres above the sea-level datum, by the ISA approximation (commons.altitude_m).
///
/// One definition, because a second baro is a CROSS-CHECK: two drivers computing altitude with their own copy of
/// this could differ by an edit rather than by the air, and the disagreement would read as a sensor fault.
///
/// A pressure of zero or below is a dead read, not a height: it answers 0.0, as plan B does (test_bmp581). NaN
/// passes through as NaN, as it does in plan B (`nan <= 0` is false there too).
///
/// f32 (decision D18): within 3.4 mm of plan B's f64 over 30-110 kPa (measured). The input alone rounds by
/// 0.0078 Pa near 100 kPa, and the 0.19 power of a ratio near 1 resolves to 6e-8, which the 44 330 m scale turns
/// into 2.6 mm -- a thirtieth of what the barometers resolve (about 1 Pa, 8.4 cm), where every decision taken on
/// an altitude is in whole metres. Not worth an f64.
pub fn altitude_m(pressure_pa: f32) -> f32 {
    if pressure_pa <= 0.0 {
        return 0.0;
    }
    ALTITUDE_SCALE_M * (1.0 - (pressure_pa / SEA_LEVEL_PA).powf(ALTITUDE_EXPONENT))
}

#[cfg(test)]
mod tests {
    use super::{FIN_LIMIT, FIN_TABLE_TOP_SPEED, between, fin_deflection_limit};

    #[test]
    fn the_table_holds_plan_b_anchors() {
        // test_commons.py: full 45 slow, 14 at 30 m/s, the 5 floor at 50 and beyond
        assert_eq!(fin_deflection_limit(0.0), 45);
        assert_eq!(fin_deflection_limit(15.0), 45);
        assert_eq!(fin_deflection_limit(30.0), 14);
        assert_eq!(fin_deflection_limit(40.0), 8);
        assert_eq!(fin_deflection_limit(50.0), 5);
        // bench_cases.py fin_limits: 45,45,45,14,5 for 0, 5, 15, 30, 90 m/s
        let bench: Vec<i32> = [0.0, 5.0, 15.0, 30.0, 90.0]
            .into_iter()
            .map(fin_deflection_limit)
            .collect();
        assert_eq!(bench, [45, 45, 45, 14, 5]);
    }

    #[test]
    fn the_table_is_bounded_and_never_opens_with_speed() {
        let mut previous = 45;
        for limit in FIN_LIMIT {
            assert!((5..=45).contains(&limit), "{limit}");
            assert!(limit <= previous, "the cap opened with speed");
            previous = limit;
        }
        // 1/v^2 takes over at 17 m/s: 12500 / 289 = 43.25
        assert_eq!(FIN_LIMIT[16], 45);
        assert_eq!(FIN_LIMIT[17], 43);
    }

    #[test]
    fn a_speed_truncates_toward_zero_like_int() {
        assert_eq!(fin_deflection_limit(29.99), FIN_LIMIT[29]);
        assert_eq!(fin_deflection_limit(30.99), FIN_LIMIT[30]);
        assert_ne!(
            FIN_LIMIT[29], FIN_LIMIT[30],
            "the case must straddle a step of the table"
        );
    }

    #[test]
    fn speeds_off_the_table_saturate() {
        assert_eq!(fin_deflection_limit(-5.0), 45);
        assert_eq!(fin_deflection_limit(-0.5), 45);
        assert_eq!(fin_deflection_limit(200.0), 5);
        assert_eq!(fin_deflection_limit(f32::INFINITY), FIN_LIMIT[FIN_TABLE_TOP_SPEED]);
        assert_eq!(fin_deflection_limit(f32::NEG_INFINITY), 45);
    }

    #[test]
    fn nan_takes_the_tightest_cap_not_the_widest() {
        assert_eq!(fin_deflection_limit(f32::NAN), 5);
    }

    #[test]
    fn between_clamps_like_plan_b() {
        assert_eq!(between(-45.0, 60.0, 45.0), 45.0);
        assert_eq!(between(-45.0, -60.0, 45.0), -45.0);
        assert_eq!(between(-45.0, 12.5, 45.0), 12.5);
        assert_eq!(between(f32::NEG_INFINITY, 1e30, f32::INFINITY), 1e30);
        assert!(
            between(-1.0, f32::NAN, 1.0).is_nan(),
            "a NaN value passes through, as in Python"
        );
        // an inverted range answers by the comparisons instead of panicking as f32::clamp would
        assert_eq!(between(5.0, 0.0, -5.0), 5.0);
        assert_eq!(between(5.0, 7.0, -5.0), -5.0);
    }
}
