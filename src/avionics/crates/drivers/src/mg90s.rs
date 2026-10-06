//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The MG90S metal-gear positional fin servo (src/glider/drivers/mg90s.py). Electrically IDENTICAL to the SG90 --
//! the same 50 Hz frame, ~500..2500 us pulse to angle, open-loop with no feedback -- so it is the SG90's
//! [`crate::sg90::Servo`] with this module's [`MG90S`] type: the angle-to-pulse map, the per-fin trim, the clamp, the
//! update and the probe verdict are all shared. What differs is MECHANICAL: metal gears hold more torque, so the
//! aerodynamic load cannot back-drive the horn (an SG90's plastic train lets wind slide the fin off its commanded
//! angle). Preferred on the yaw fin, the rudder that sees the most steady wind pressure.
//!
//! TRAVEL is per instance through the component's min_deg/max_deg, whose SPAN must equal the servo's mechanical
//! travel so one command degree is one degree of rotation. The 180-degree variant uses the default 0..180 and drops
//! in exactly where an SG90 was. The 360-degree-travel MG90S wants its 360 degrees CENTRED on the mixer neutral (90):
//! `min_deg: -90, max_deg: 270` -> command 90 = servo mid = 1500 us = fin centre (so boot, probe and failsafe all sit
//! centred), and +/-45 command = +/-45 degrees of fin. A fin only uses neutral +/- the mixer limit, so the spare range
//! is unused and the ~0.18-degree pulse step is still plenty. MIXED fleets are fine: each fin's `driver` is
//! independent (an mg90s yaw with sg90 elevons), and the mixer commands angles by name, blind to the servo type.

use crate::sg90::ServoType;

/// The MG90S: ~0.1 s per 60 degrees (metal gear, a touch quicker than the SG90's 0.15), and a probe window shifted
/// up for the larger current (600..5000 mW). Both are datasheet-approximate TYPE defaults; tune them per built rig
/// through the component's engine_*_mw config.
pub const MG90S: ServoType = ServoType {
    name: "mg90s",
    slew_ms_per_60: 100,
    settle_margin_ms: 60,
    engine_min_mw: 600,
    engine_max_mw: 5_000,
};

#[cfg(test)]
mod tests {
    use super::MG90S;
    use crate::sg90::{SG90, Servo, Settings};
    use serde_json::json;

    /// An MG90S from a component block.
    fn servo(config: serde_json::Value) -> Servo {
        Servo::new(&MG90S, &Settings::from_config(&config, &MG90S))
    }

    #[test]
    fn the_metal_gear_type_defaults() {
        assert_eq!((MG90S.slew_ms_per_60, SG90.slew_ms_per_60), (100, 150));
        assert_eq!((MG90S.engine_min_mw, MG90S.engine_max_mw), (600, 5000));
        let stock = servo(json!({}));
        assert_eq!((stock.engine_min_mw(), stock.engine_max_mw()), (600, 5000));
        assert_eq!(stock.travel_ms(60, false), 100);
        assert_eq!(stock.travel_ms(60, true), 160);
    }

    #[test]
    fn the_centred_360_degree_travel() {
        // test_mg90s.py: min_deg -90 / max_deg 270 -> neutral 90 = servo mid = 1500 us
        let mut fin = servo(json!({"min_deg": -90, "max_deg": 270}));
        assert_eq!((fin.angle(), fin.pulse_us()), (90, 1500));
        // 1 command degree = 1 rotation degree: +/-45 from neutral is +/-250 us
        assert_eq!(fin.apply(135.0).pulse_us, 1750);
        assert_eq!(fin.apply(45.0).pulse_us, 1250);
        assert_eq!(fin.apply(999.0).angle, 270);
        assert_eq!(fin.apply(-999.0).angle, -90);
        // the trim is inherited: +9 degrees over the 360 span is +50 us
        let (applied, command) = fin.update(&json!({"angle": 90, "trim": 9})).unwrap();
        assert_eq!(applied, ["trim", "angle"]);
        assert_eq!(command.unwrap().pulse_us, 1550);
    }

    #[test]
    fn a_180_degree_mg90s_drops_in_like_an_sg90() {
        let mut stock = servo(json!({}));
        let plastic = Servo::new(&SG90, &Settings::from_config(&json!({}), &SG90));
        assert_eq!((stock.angle(), stock.pulse_us()), (plastic.angle(), plastic.pulse_us()));
        // NEGATIVE: only the slew differs -- the same 45-degree move is estimated shorter on metal gears
        let metal = stock.begin_move(135.0);
        assert_eq!((metal.command.angle, metal.travel_ms), (135, 45 * 100 / 60 + 60));
        assert!(metal.travel_ms < plastic.travel_ms(45, true));
    }
}
