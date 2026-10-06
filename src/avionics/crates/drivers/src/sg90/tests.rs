//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the servo logic beyond its golden fixture (tests/golden_servo.rs): the probe verdict, the
//! compare-and-set, the floored pulse map below the range, and the refusals.

use super::{Command, ProbeVerdict, Refusal, SG90, Servo, Settings, round_half_even};
use serde_json::json;

/// An SG90 from a component block.
fn fin(config: serde_json::Value) -> Servo {
    Servo::new(&SG90, &Settings::from_config(&config, &SG90))
}

#[test]
fn neutral_at_setup_is_the_mid_pulse() {
    let yaw = fin(json!({}));
    assert_eq!(
        yaw.command(),
        Command {
            angle: 90,
            pulse_us: 1500
        }
    );
    assert_eq!(
        fin(json!({"trim": 9})).pulse_us(),
        1600,
        "+9 degrees over 0..180 / 500..2500 us is +100 us"
    );
}

#[test]
fn compare_and_set_writes_only_a_change() {
    let mut yaw = fin(json!({}));
    assert_eq!(yaw.set_angle(45.0).map(|command| command.pulse_us), Some(1000));
    assert_eq!(yaw.set_angle(45.0), None, "a held fin costs nothing");
    assert_eq!(yaw.set_angle(45.4), None, "it rounds to the same whole degree");
    assert_eq!(yaw.set_angle(999.0).map(|command| command.angle), Some(180));
    assert_eq!(yaw.set_angle(180.0), None, "clamp-equal is unchanged");
}

#[test]
fn the_pulse_map_floors_below_the_range() {
    // a trim that takes the command under min_deg: Python's // floors toward minus infinity, then the bound holds
    let yaw = fin(json!({"trim": -1}));
    assert_eq!(
        yaw.pulse(0),
        500,
        "500 + (-1 x 2000) // 180 = 500 - 12 = 488, bounded to 500"
    );
    assert_eq!(
        yaw.pulse(1),
        500,
        "500 + 0 = 500: the trim moved the whole map a degree"
    );
    assert_eq!(yaw.pulse(2), 511, "500 + 2000 // 180 = 511, floored like Python's //");
    assert_eq!(
        fin(json!({"min_deg": 90, "max_deg": 90})).pulse(90),
        500,
        "a zero span is min_us"
    );
}

#[test]
fn halves_round_to_even() {
    let pairs = [
        (0.5, 0),
        (1.5, 2),
        (2.5, 2),
        (-0.5, 0),
        (-1.5, -2),
        (90.5, 90),
        (91.5, 92),
        (2.6, 3),
        (-1.4, -1),
    ];
    for (degrees, rounded) in pairs {
        assert_eq!(round_half_even(degrees), rounded, "{degrees}");
    }
}

#[test]
fn a_trim_alone_re_drives_the_fin() {
    let mut yaw = fin(json!({"trim": 9}));
    let (applied, command) = yaw.update(&json!({"trim": 0})).unwrap();
    assert_eq!(
        (applied, command.map(|command| command.pulse_us)),
        (vec!["trim"], Some(1500))
    );
    let (applied, command) = yaw.update(&json!({})).unwrap();
    assert!(applied.is_empty() && command.is_none());
}

#[test]
fn a_non_numeric_update_changes_nothing() {
    // NEGATIVE: refused as a whole, the trim untouched though it came first
    let mut yaw = fin(json!({"trim": 2}));
    let refusal = yaw.update(&json!({"trim": 5, "angle": "left"})).unwrap_err();
    assert_eq!(refusal, Refusal::NotANumber("angle"));
    assert_eq!((yaw.trim(), yaw.angle(), yaw.pulse_us()), (2, 90, 1522));
}

#[test]
fn the_probe_sweeps_min_max_neutral() {
    let elevon = fin(json!({"min_deg": -30, "max_deg": 30}));
    assert_eq!(elevon.probe_targets(), [("min", -30), ("max", 30), ("neutral", 0)]);
}

#[test]
fn the_probe_verdict_judges_the_rise_over_the_baseline() {
    let yaw = fin(json!({}));
    assert_eq!(yaw.probe_verdict(false, 0, 0), ProbeVerdict::OpenLoop);
    assert_eq!(
        yaw.probe_verdict(true, 0, 0),
        ProbeVerdict::Stale,
        "no reading is not a dead servo"
    );
    assert_eq!(
        yaw.probe_verdict(true, 72, 1200),
        ProbeVerdict::Drew {
            peak_rise_mw: 1200,
            high: false
        }
    );
    assert_eq!(
        yaw.probe_verdict(true, 72, 3501),
        ProbeVerdict::Drew {
            peak_rise_mw: 3501,
            high: true
        }
    );
    // NEGATIVE: a dead servo, a lost PWM pin or an unpowered rail draws under the floor
    let dead = yaw.probe_verdict(true, 72, 499);
    assert_eq!(dead, ProbeVerdict::NoDraw { peak_rise_mw: 499 });
    assert_eq!(
        dead.failure().as_deref(),
        Some("no current draw (499 mW rise) -- servo dead / PWM pin lost / rail unpowered?")
    );
    assert_eq!(
        yaw.probe_verdict(true, 72, 500).failure(),
        None,
        "the floor itself passes"
    );
    // a component's own window overrides the type's
    let tuned = fin(json!({"engine_min_mw": 800}));
    assert!(matches!(tuned.probe_verdict(true, 1, 700), ProbeVerdict::NoDraw { .. }));
}

#[test]
fn a_move_estimates_from_where_the_horn_was() {
    let mut yaw = fin(json!({}));
    let moved = yaw.begin_move(120.0);
    assert_eq!((moved.command.angle, moved.travel_ms), (120, 30 * 150 / 60 + 60));
    assert_eq!(moved.command.row(false), [120, 1833, 0]);
    assert_eq!(moved.command.row(true), [120, 1833, 1]);
    // NEGATIVE: a move to where the horn already is still waits the settle margin, never less
    assert_eq!(yaw.begin_move(120.0).travel_ms, 60);
}

#[test]
fn the_mixer_drives_the_fins_through_compare_and_set() {
    use control::mixer::{Fin, Mixer};
    let mut mixer = Mixer::default();
    let mut fins = [
        Some(fin(json!({}))),
        Some(fin(json!({"trim": 9}))),
        Some(fin(json!({}))),
    ];
    mixer.actuate(&mut fins, 0.0, 10.0, 0.0);
    let angles: Vec<Option<i32>> = fins.iter().map(|servo| servo.as_ref().and_then(Fin::angle)).collect();
    assert_eq!(
        angles,
        [Some(90), Some(100), Some(100)],
        "pitch moves both elevons together"
    );
    assert_eq!(
        fins[1].as_ref().unwrap().pulse_us(),
        1711,
        "the trim rides on the mixer's angle"
    );
    // NEGATIVE: the same mix again writes nothing: every angle is the one already commanded
    let before: Vec<Command> = fins.iter().map(|servo| servo.as_ref().unwrap().command()).collect();
    mixer.actuate(&mut fins, 0.0, 10.0, 0.0);
    let after: Vec<Command> = fins.iter().map(|servo| servo.as_ref().unwrap().command()).collect();
    assert_eq!(before, after);
    // and a missing fin is skipped while the bound ones still drive
    let mut partial: [Option<Servo>; 3] = [None, Some(fin(json!({}))), None];
    mixer.actuate(&mut partial, 10.0, 0.0, 0.0);
    assert_eq!(partial[1].as_ref().and_then(Fin::angle), Some(100));
}
