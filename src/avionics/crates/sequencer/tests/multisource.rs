//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage machine on several sources per class: the fusion rule (crates/sequencer, module `sequencer`) against
//! the one-source machine plan B is, which golden/sequencer_trace.json holds the port to.
//!
//! - TMS-7's flight (launches/20261003/TMS-7/flight.csv: the nose logger's BMI323 and BMP581, 50 Hz, an F15 boost to
//!   479 m and a chute descent), made two sources per class: the BMP581 again 12 m off as a second barometer, the
//!   BMI323 again one step late as a second accelerometer. Every stage comes at the step the one-source machine on
//!   the primary gives it, or later only with a veto recorded. Then a barometer dropout at the top, a barometer back
//!   from a gap near the top, and handovers.
//!   The 10-03 corpora themselves are not replayed: TMS-7 carries one source per class, TMS-7C's stops 1.31 s after
//!   ignition.
//! - The rules one by one, positive and negative: who decides, the one dwell per detector carrying across a handover
//!   and draining under a veto, the peer quorum, the physics vetoes on every live accelerometer, the backstops through
//!   any veto, the GNSS that never votes, a source back from a gap (its climb confirming nothing, its apogee tracker
//!   joining the flight afresh), the vetoes.csv rows, the binding of sources to slots.
//!
//! "Plan B's step" is the step the one-source machine takes on plan B's fused read: the first live source of each
//! class in its first slot (no reconcile offset: a backup's raw value), the GNSS's elevation in the GNSS slot.

use std::mem::size_of;

use sequencer::{
    ACCELEROMETER_SLOTS, BAROMETER_SLOTS, Binding, Decider, Detector, Physics, Settings, SourceReadings, Stage,
    StageMachine, Veto, VetoEdge,
};
use serde_json::Value;

/// TMS-7's flight, 2026-10-03: the nose logger's 50 Hz trace (`t_s;ax_g;ay_g;az_g;a_g;gx_dps;gy_dps;gz_dps;
/// pressure_pa;temp_c;alt_m;segment;tick_ms`), launch at t = 0.
const TMS7: &str = include_str!("../../../../../launches/20261003/TMS-7/flight.csv");

/// The firmware's defaults: plan B's config_default, as golden/generate.py dumped it.
const CONFIG_DEFAULT: &str = include_str!("../../../golden/config_default.json");

/// The launch records' board configs, each with a sequencer entry.
const LAUNCH_CONFIGS: [&str; 5] = [
    include_str!("../../../../../launches/20261003/TMS-7C/tms7c.config"),
    include_str!("../../../../../launches/20261003/TMS-7D/tms7d.config"),
    include_str!("../../../../../launches/20261003/TMS-7D/tms7d_control.config"),
    include_str!("../../../../../launches/20261003/TMS-7E/tms7e.config"),
    include_str!("../../../../../launches/20261003/TMS-7F/tms7f.config"),
];

/// The machine's step period on the board, ms.
const PERIOD_MS: u32 = 50;

/// test_sequencer.py's SPEC thresholds with plan B's 2.5 g launch line: short dwells for short cases.
fn spec() -> Settings {
    Settings {
        boost_timeout_ms: 500,
        apogee_arm_ms: 50,
        land_ms: 100,
        ground_ms: 300,
        ..Settings::default()
    }
}

/// An accelerometer lying flat reading `g` (on +z): what the still band and the launch line take as |accel|.
fn flat(g: f32) -> Option<[f32; 3]> {
    Some([0.0, 0.0, g])
}

/// An accelerometer reading `g` on +x, the nose: what a thrust gate takes as a push.
fn nose(g: f32) -> Option<[f32; 3]> {
    Some([g, 0.0, 0.0])
}

/// One step's readings: the accelerometers' and the barometers' by slot (None: not live there).
fn read(accelerations: &[Option<[f32; 3]>], elevations: &[Option<f32>]) -> SourceReadings {
    let mut readings = SourceReadings::default();
    readings.accelerations[..accelerations.len()].copy_from_slice(accelerations);
    readings.elevations[..elevations.len()].copy_from_slice(elevations);
    readings
}

/// Plan B's fused read of `readings`: the first live source of each class alone in its first slot.
fn fused(readings: SourceReadings) -> SourceReadings {
    let first = |slots: &[Option<[f32; 3]>]| slots.iter().flatten().next().copied();
    let mut one = SourceReadings {
        gnss_elevation: readings.gnss_elevation,
        height_above_ground: readings.height_above_ground,
        ..SourceReadings::default()
    };
    one.accelerations[0] = first(&readings.accelerations);
    one.elevations[0] = readings.elevations.iter().flatten().next().copied();
    if one.elevations[0].is_some() {
        one.gnss_elevation = None; // plan B's read stops at the first fresh barometer
    }
    one
}

/// What a machine recorded over a run.
#[derive(Debug, Default)]
struct Flown {
    /// Its own advances: the step clock, the stage and the reason text.
    advances: Vec<(u32, Stage, String)>,
    /// The veto episodes' edges, each with its step clock.
    edges: Vec<(u32, VetoEdge)>,
}

impl Flown {
    /// The advances' clocks and stages.
    fn stages(&self) -> Vec<(u32, Stage)> {
        self.advances
            .iter()
            .map(|(now_ms, stage, _)| (*now_ms, *stage))
            .collect()
    }

    /// The edges' clocks, kinds and detectors.
    fn episodes(&self) -> Vec<(u32, &'static str, Detector)> {
        self.edges
            .iter()
            .map(|(now_ms, edge)| (*now_ms, edge.name(), edge.veto().detector))
            .collect()
    }
}

/// A machine on `settings` stepped from `stage` through `ticks`, each advance applied as the caller does.
fn fly(settings: Settings, stage: Stage, ticks: &[(u32, SourceReadings)]) -> Flown {
    let mut machine = StageMachine::new(settings);
    let mut current = stage;
    let mut flown = Flown::default();
    for &(now_ms, readings) in ticks {
        let step = machine.step(now_ms, current, readings, false);
        flown.edges.extend(step.vetoes().map(|edge| (now_ms, edge)));
        if let Some(advance) = step.advance {
            current = advance.to;
            flown.advances.push((now_ms, advance.to, advance.reason.to_string()));
        }
    }
    flown
}

/// The same run on plan B's fused read of each step.
fn fly_fused(settings: Settings, stage: Stage, ticks: &[(u32, SourceReadings)]) -> Flown {
    let one: Vec<(u32, SourceReadings)> = ticks
        .iter()
        .map(|&(now_ms, readings)| (now_ms, fused(readings)))
        .collect();
    fly(settings, stage, &one)
}

/// Steps every [`PERIOD_MS`] from `from_ms` through `to_ms`, each reading `readings(now_ms)`.
fn ticks(from_ms: u32, to_ms: u32, readings: impl Fn(u32) -> SourceReadings) -> Vec<(u32, SourceReadings)> {
    (from_ms..=to_ms)
        .step_by(PERIOD_MS as usize)
        .map(|now_ms| (now_ms, readings(now_ms)))
        .collect()
}

/// The advance text: `<stage>: <reason>`.
fn advanced(flown: &Flown) -> Vec<(u32, String)> {
    flown
        .advances
        .iter()
        .map(|(now_ms, stage, reason)| (*now_ms, format!("{}: {reason}", stage.name())))
        .collect()
}

/// The vetoes that began in a run.
fn begun(flown: &Flown) -> Vec<(u32, Veto)> {
    flown
        .edges
        .iter()
        .filter_map(|(now_ms, edge)| match edge {
            VetoEdge::Begin(veto) => Some((*now_ms, *veto)),
            VetoEdge::End(_) => None,
        })
        .collect()
}

#[test]
fn both_accelerometers_over_the_line_launch_on_plan_b_s_step() {
    let boost = ticks(0, 300, |now_ms| {
        let g = if now_ms == 0 { 1.0 } else { 3.0 };
        read(&[flat(g), flat(g)], &[])
    });
    let flown = fly(spec(), Stage::Setting, &boost);
    assert_eq!(
        advanced(&flown),
        [(150, "boosting: launch |a|=3.0g dwell=100ms".to_string())]
    );
    assert_eq!(flown.stages(), fly_fused(spec(), Stage::Setting, &boost).stages());
    assert!(flown.edges.is_empty(), "a peer over the line is no contrary peer");
}

#[test]
fn the_dwell_carries_on_when_the_decider_dies_and_its_peer_agrees() {
    // the LSM6DSO32 boosts from 50, stops publishing from 100; the BMI323 boosts alike and carries the dwell on
    let boost = ticks(0, 300, |now_ms| match now_ms {
        0 => read(&[flat(1.0), flat(1.0)], &[]),
        50 => read(&[flat(3.0), flat(3.0)], &[]),
        _ => read(&[None, flat(3.0)], &[]),
    });
    let flown = fly(spec(), Stage::Setting, &boost);
    assert_eq!(flown.stages(), [(150, Stage::Boosting)]);
    assert_eq!(flown.stages(), fly_fused(spec(), Stage::Setting, &boost).stages());
}

#[test]
fn the_laser_arriving_mid_barometer_dwell_lands_on_plan_b_s_step() {
    let mut glide = ticks(0, 200, |now_ms| match now_ms {
        0 => read(&[], &[Some(20.0), Some(20.5)]),
        _ => read(&[], &[Some(3.0), Some(3.5)]),
    });
    for (now_ms, readings) in &mut glide {
        if *now_ms >= 100 {
            readings.height_above_ground = Some(2.0);
        }
    }
    let flown = fly(spec(), Stage::Gliding, &glide);
    // land_ms after the barometer first read low at 50, not after the laser took over at 100
    assert_eq!(advanced(&flown), [(150, "landing: agl 2.0m".to_string())]);
    assert_eq!(flown.stages(), fly_fused(spec(), Stage::Gliding, &glide).stages());
}

/// SETTING on a pad at 0 m held for a second (from 0 through 1000), then `climb(now_ms)` on each barometer slot's
/// own `offset_m`, with the accelerometers `accelerations(now_ms)`.
fn pad_then(
    climb: impl Fn(u32) -> [Option<f32>; BAROMETER_SLOTS],
    accelerations: impl Fn(u32) -> Vec<Option<[f32; 3]>>,
    to_ms: u32,
) -> Vec<(u32, SourceReadings)> {
    ticks(0, to_ms, |now_ms| read(&accelerations(now_ms), &climb(now_ms)))
}

#[test]
fn an_altitude_launch_held_by_its_peer_leaves_the_accel_launch_its_step() {
    // the ICP-10111 climbs 15 m at 1050 while the BMP581 stays on the pad (contrary), and both IMUs boost at 3 g from
    // 1050: plan B launches by altitude at 1150, the port by its accel on the same step
    let climb = |now_ms: u32| [Some(if now_ms < 1050 { 0.0 } else { 15.0 }), Some(0.0)];
    let boost = |now_ms: u32| {
        let g = if now_ms < 1050 { 1.0 } else { 3.0 };
        vec![flat(g), flat(g)]
    };
    let launch = pad_then(climb, boost, 1300);
    let flown = fly(spec(), Stage::Setting, &launch);
    assert_eq!(
        advanced(&flown),
        [(1150, "boosting: launch |a|=3.0g dwell=100ms".to_string())]
    );
    assert_eq!(
        advanced(&fly_fused(spec(), Stage::Setting, &launch)),
        [(1150, "boosting: launch alt=15m dwell=100ms".to_string())]
    );
    // one peer episode: from the climb until BOOSTING ends the detector's time
    assert_eq!(
        flown.episodes(),
        [
            (1050, "begin", Detector::LaunchAltitude),
            (1200, "end", Detector::LaunchAltitude)
        ]
    );
    let (_, veto) = begun(&flown)[0];
    assert_eq!(
        (veto.decider, veto.kinds(), veto.contrary, veto.live, veto.value),
        (Decider::Barometer(0), "peer", 1, 2, 15.0)
    );
}

#[test]
fn the_altitude_launch_fires_with_the_accel_class_dead() {
    let climb = |now_ms: u32| {
        let metres = if now_ms < 1050 { 0.0 } else { 15.0 };
        [Some(metres), Some(metres + 0.4)]
    };
    let launch = pad_then(climb, |_| vec![], 1300);
    let flown = fly(spec(), Stage::Setting, &launch);
    assert_eq!(
        advanced(&flown),
        [(1150, "boosting: launch alt=15m dwell=100ms".to_string())]
    );
    assert!(flown.edges.is_empty());
}

#[test]
fn one_accelerometer_out_of_the_band_lifts_the_at_rest_veto() {
    // the LSM6DSO32 reads 1 g (it decides the accel launch: none), the BMI323 6 g; both baros rise 15 m in 0.5 s
    let climb = |now_ms: u32| {
        let metres = (now_ms.saturating_sub(1000) as f32 * 0.03).min(15.0);
        [Some(metres), Some(metres)]
    };
    let launch = pad_then(climb, |_| vec![flat(1.0), flat(6.0)], 1500);
    let flown = fly(spec(), Stage::Setting, &launch);
    // over both lines from 1350 (10.5 m), launch_ms later
    assert_eq!(
        advanced(&flown),
        [(1450, "boosting: launch alt=14m dwell=100ms".to_string())]
    );
    assert!(flown.edges.is_empty());
    // NEGATIVE: both at rest, the same climb is held for as long as they rest
    let flown = fly(
        spec(),
        Stage::Setting,
        &pad_then(climb, |_| vec![flat(1.0), flat(1.0)], 4000),
    );
    assert_eq!(flown.stages(), []);
    let (now_ms, veto) = begun(&flown)[0];
    assert_eq!(
        (now_ms, veto.detector, veto.kinds(), veto.peer, veto.physics),
        (1350, Detector::LaunchAltitude, "at_rest", false, Some(Physics::AtRest))
    );
    // until the pad's level leaves the window and the climb no longer counts as a rise
    assert_eq!(flown.episodes()[1], (2200, "end", Detector::LaunchAltitude));
}

#[test]
fn the_at_rest_veto_needs_the_whole_window_of_rest() {
    // still only since 500: half a second at rest is no veto, and the climb at 1000 launches
    let accelerations = |now_ms: u32| {
        let g = if now_ms < 500 { 1.6 } else { 1.0 };
        vec![flat(g), flat(g)]
    };
    let climb = |now_ms: u32| [Some(if now_ms < 1000 { 0.0 } else { 15.0 }), None];
    let flown = fly(spec(), Stage::Setting, &pad_then(climb, accelerations, 1500));
    assert_eq!(flown.stages(), [(1100, Stage::Boosting)]);
    // still since 0, the same climb is held
    let rested = |_| vec![flat(1.0), flat(1.0)];
    let flown = fly(spec(), Stage::Setting, &pad_then(climb, rested, 1500));
    assert_eq!(flown.stages(), []);
}

#[test]
fn a_missed_step_never_releases_the_at_rest_veto() {
    // NEGATIVE: both IMUs at rest from 0; the BMI323 misses the step at 1200, and the climb comes at 1250. A dropout is
    // no movement: its run at rest goes on, every live accelerometer has rested the whole window, the climb is held
    let missed = |now_ms: u32| vec![flat(1.0), if now_ms == 1200 { None } else { flat(1.0) }];
    let climb = |now_ms: u32| [Some(if now_ms < 1250 { 0.0 } else { 15.0 }), None];
    let flown = fly(spec(), Stage::Setting, &pad_then(climb, missed, 1500));
    assert_eq!(flown.stages(), []);
    let (now_ms, veto) = begun(&flown)[0];
    assert_eq!((now_ms, veto.kinds()), (1250, "at_rest"));
    // a reading out of the band in its place ends the run: not every live accelerometer has rested the whole window,
    // and the climb launches
    let moved = |now_ms: u32| vec![flat(1.0), flat(if now_ms == 1200 { 1.6 } else { 1.0 })];
    let flown = fly(spec(), Stage::Setting, &pad_then(climb, moved, 1500));
    assert_eq!((flown.stages(), flown.edges.len()), (vec![(1350, Stage::Boosting)], 0));
}

#[test]
fn a_peer_barometer_that_rose_half_the_line_agrees() {
    // the BMP581 lags the ICP-10111's 15 m: up 6 m, more than launch_alt_m / 2, it is no contrary peer
    let climb = |peer_m: f32| {
        move |now_ms: u32| {
            let up = now_ms >= 1050;
            [Some(if up { 15.0 } else { 0.0 }), Some(if up { peer_m } else { 0.0 })]
        }
    };
    let flown = fly(spec(), Stage::Setting, &pad_then(climb(6.0), |_| vec![], 1300));
    assert_eq!((flown.stages(), flown.edges.len()), (vec![(1150, Stage::Boosting)], 0));
    // up exactly 5 m, half the line, it is not less than half: it agrees
    let flown = fly(spec(), Stage::Setting, &pad_then(climb(5.0), |_| vec![], 1300));
    assert_eq!((flown.stages(), flown.edges.len()), (vec![(1150, Stage::Boosting)], 0));
    // NEGATIVE: up 4 m, less than half the line, it is contrary and holds the launch
    let flown = fly(spec(), Stage::Setting, &pad_then(climb(4.0), |_| vec![], 1300));
    assert_eq!(flown.stages(), []);
    assert_eq!(flown.episodes()[0], (1050, "begin", Detector::LaunchAltitude));
}

#[test]
fn a_peer_barometer_s_first_reading_is_no_raise() {
    // the ICP-10111 falls past its line at 100; the BMP581 publishes its first reading at 150, high: its tracker is
    // seeded, not raised, so it is not contrary, and the apogee comes on the ICP-10111's own schedule
    let long = Settings {
        boost_timeout_ms: 5000,
        ..spec()
    };
    let boost = ticks(0, 600, |now_ms| {
        let primary = if now_ms < 100 { 200.0 } else { 150.0 };
        let peer = (now_ms >= 150).then_some(250.0 + now_ms as f32);
        read(&[], &[Some(primary), peer])
    });
    let flown = fly(long, Stage::Boosting, &boost);
    // from the seed on its peak rises every step: contrary from 200, the second of its readings
    assert_eq!(flown.episodes()[0], (200, "begin", Detector::Apogee));
    assert_eq!(flown.stages(), []);
    // NEGATIVE: a peer that reads once and holds its value, seeded at 150, never raises, and holds nothing
    let boost = ticks(0, 600, |now_ms| {
        let primary = if now_ms < 100 { 200.0 } else { 150.0 };
        read(&[], &[Some(primary), (now_ms >= 150).then_some(250.0)])
    });
    let flown = fly(long, Stage::Boosting, &boost);
    assert_eq!((flown.stages(), flown.edges.len()), (vec![(200, Stage::Gliding)], 0));
}

#[test]
fn a_tap_a_stale_step_and_a_tap_never_launch() {
    // NEGATIVE: no per-source dwell credits the gap: the missing step drains the one dwell
    let taps = ticks(0, 1000, |now_ms| match now_ms {
        50 | 150 => read(&[flat(8.0)], &[]),
        100 => read(&[None], &[]),
        _ => read(&[flat(1.0)], &[]),
    });
    assert_eq!(fly(spec(), Stage::Setting, &taps).stages(), []);
}

#[test]
fn a_stale_gap_longer_than_land_ms_then_one_low_sample_never_lands() {
    // NEGATIVE: the gap restarts the landing dwell, so the one low sample after it starts a dwell afresh
    let glide = ticks(0, 500, |now_ms| match now_ms {
        0 | 450 => read(&[], &[Some(3.0)]),
        500 => read(&[], &[Some(6.0)]),
        _ => read(&[], &[None]),
    });
    assert_eq!(fly(spec(), Stage::Gliding, &glide).stages(), []);
}

#[test]
fn a_backup_over_the_line_never_launches_through_one_missed_primary_step() {
    // NEGATIVE: the BMI323 reads 3 g for a second while the LSM6DSO32 reads 1 g and misses one step at 500: the
    // BMI323 decides that one step, starting a dwell the LSM6DSO32 drains on the next
    let shaken = ticks(0, 1000, |now_ms| {
        let primary = if now_ms == 500 { None } else { flat(1.0) };
        read(&[primary, flat(3.0)], &[])
    });
    let flown = fly(spec(), Stage::Setting, &shaken);
    assert_eq!(flown.stages(), []);
    assert!(flown.edges.is_empty(), "a decider under the line is never vetoed");
}

#[test]
fn a_held_glitch_stays_held_when_its_contrary_peer_misses_one_step() {
    // NEGATIVE: the LSM6DSO32 glitches at 3 g for a second, the BMI323 at 1 g holds it; the BMI323 misses one step at
    // 500, which the LSM6DSO32 decides unvetoed, but the drained dwell restarts there and drains again on the next
    let glitch = ticks(0, 1000, |now_ms| {
        let peer = if now_ms == 500 { None } else { flat(1.0) };
        read(&[flat(3.0), peer], &[])
    });
    let flown = fly(spec(), Stage::Setting, &glitch);
    assert_eq!(flown.stages(), []);
    assert_eq!(
        flown.episodes(),
        [
            (0, "begin", Detector::LaunchAcceleration),
            (500, "end", Detector::LaunchAcceleration),
            (550, "begin", Detector::LaunchAcceleration),
        ]
    );
}

#[test]
fn an_offset_between_two_barometers_is_no_rise_and_no_drop() {
    // the ICP-10111 on the pad at 0 m, the BMP581 12 m off; the ICP-10111 goes stale for half a second. Plan B's
    // fused read hands over to the BMP581's raw 12 m: a 12 m rise within the window, a launch. Each barometer
    // compared with itself rose nothing
    let pad = ticks(0, 2500, |now_ms| {
        let primary = if now_ms < 2050 { Some(0.0) } else { None };
        read(&[], &[primary, Some(12.0)])
    });
    assert_eq!(
        advanced(&fly_fused(spec(), Stage::Setting, &pad)),
        [(2150, "boosting: launch alt=12m dwell=100ms".to_string())]
    );
    assert_eq!(fly(spec(), Stage::Setting, &pad).stages(), []);
    // in BOOSTING the BMP581 decides while the ICP-10111 is stale, then hands back: plan B's one tracker smooths a
    // 12 m drop and calls apogee; each tracker alone sees a flat 100 m and 112 m, and only the burnout ends BOOSTING
    let boost = ticks(0, 600, |now_ms| {
        let primary = if (50..=200).contains(&now_ms) {
            None
        } else {
            Some(100.0)
        };
        read(&[], &[primary, Some(112.0)])
    });
    assert_eq!(
        advanced(&fly_fused(spec(), Stage::Boosting, &boost)),
        [(400, "gliding: apogee 112m".to_string())]
    );
    assert_eq!(
        advanced(&fly(spec(), Stage::Boosting, &boost)),
        [(500, "gliding: burnout timeout (no separation)".to_string())]
    );
}

#[test]
fn a_glitch_on_one_accelerometer_is_vetoed() {
    // NEGATIVE: the LSM6DSO32 reads 3 g for 300 ms on a still pad, 3.5 g at the last, the BMI323 1 g, and an ADXL375
    // joins at 1 g from 200: no launch, one peer episode
    let glitch = ticks(0, 600, |now_ms| {
        let g = match now_ms {
            0..300 => 3.0,
            300 => 3.5,
            _ => 1.0,
        };
        let third = (now_ms >= 200).then_some([0.0, 0.0, 1.0]);
        read(&[flat(g), flat(1.0), third], &[])
    });
    let flown = fly(spec(), Stage::Setting, &glitch);
    assert_eq!(flown.stages(), []);
    assert_eq!(
        flown.episodes(),
        [
            (0, "begin", Detector::LaunchAcceleration),
            (350, "end", Detector::LaunchAcceleration)
        ]
    );
    let (_, veto) = begun(&flown)[0];
    assert_eq!(
        (veto.decider, veto.kinds(), veto.contrary, veto.live, veto.value),
        (Decider::Accelerometer(0), "peer", 1, 2, 3.0)
    );
    // the end row carries the episode's last vetoed step: 3.5 g, two of three contrary
    let VetoEdge::End(last) = flown.edges[1].1 else {
        panic!("{:?}", flown.edges)
    };
    assert_eq!((last.contrary, last.live, last.value), (2, 3, 3.5));
    // the same glitch on the BMI323: the LSM6DSO32 decides at 1 g, and nothing is held
    let glitch = ticks(0, 600, |now_ms| {
        let g = if now_ms <= 300 { 3.0 } else { 1.0 };
        read(&[flat(1.0), flat(g)], &[])
    });
    let flown = fly(spec(), Stage::Setting, &glitch);
    assert_eq!((flown.stages(), flown.edges.len()), (vec![], 0));
}

#[test]
fn a_stale_or_non_finite_peer_neither_decides_nor_vetoes() {
    // a NaN or missing BMI323 beside a boosting LSM6DSO32: the launch on plan B's step, nothing held. The machine
    // takes no health: a fresh source counts whatever its part's health says (plan B's read), which
    // a_glitch_on_one_accelerometer_is_vetoed shows from the other side
    for peer in [Some([f32::NAN, 0.0, 1.0]), Some([0.0, 0.0, f32::INFINITY]), None] {
        let boost = ticks(0, 300, |now_ms| {
            let g = if now_ms == 0 { 1.0 } else { 3.0 };
            read(&[flat(g), peer], &[])
        });
        let flown = fly(spec(), Stage::Setting, &boost);
        assert_eq!(
            (flown.stages(), flown.edges.len()),
            (vec![(150, Stage::Boosting)], 0),
            "{peer:?}"
        );
    }
    // a NaN LSM6DSO32 does not decide: the BMI323 under it does, at 1 g, and holds nothing either
    let flown = fly(
        spec(),
        Stage::Setting,
        &ticks(0, 300, |_| read(&[Some([f32::NAN, 0.0, 3.0]), flat(1.0)], &[])),
    );
    assert_eq!((flown.stages(), flown.edges.len()), (vec![], 0));
    // a NaN barometer neither: the BMP581 under it decides the climb and launches
    let climb = |now_ms: u32| [Some(f32::NAN), Some(if now_ms < 1050 { 0.0 } else { 15.0 })];
    let flown = fly(spec(), Stage::Setting, &pad_then(climb, |_| vec![], 1300));
    assert_eq!(flown.stages(), [(1150, Stage::Boosting)]);
}

/// BOOSTING (a warm start, armed at 50 ms) on both IMUs `accelerations(now_ms)`, the ICP-10111 falling from 200 m to
/// 150 m at 100: its smoothed fall crosses the drop line at 100, and plan B calls apogee at 200.
fn falling_under(settings: Settings, accelerations: impl Fn(u32) -> Vec<Option<[f32; 3]>>, to_ms: u32) -> Flown {
    let boost = ticks(0, to_ms, |now_ms| {
        let metres = if now_ms < 100 { 200.0 } else { 150.0 };
        read(&accelerations(now_ms), &[Some(metres)])
    });
    fly(settings, Stage::Boosting, &boost)
}

#[test]
fn the_apogee_is_held_while_every_accelerometer_sees_thrust() {
    let long = Settings {
        boost_timeout_ms: 5000,
        ..spec()
    };
    // both thrust gates see the boost from 100 (launch_ms of 8 g on +x) and never its end: no apogee
    let flown = falling_under(long, |_| vec![nose(8.0), nose(8.0)], 1000);
    assert_eq!(flown.stages(), []);
    let (now_ms, veto) = begun(&flown)[0];
    assert_eq!(
        (now_ms, veto.detector, veto.decider, veto.kinds(), veto.value),
        (100, Detector::Apogee, Decider::Barometer(0), "thrust", 150.0)
    );
    // the BMI323's boost ends at 150 (drag, -0.5 g), its gate opens at 200: no longer every live one, and the fall
    // that held since 100 counts afresh from 200
    let ended = |now_ms: u32| vec![nose(8.0), nose(if now_ms < 150 { 8.0 } else { -0.5 })];
    let flown = falling_under(long, ended, 1000);
    assert_eq!(advanced(&flown), [(300, "gliding: apogee 200m".to_string())]);
    // NEGATIVE: with no live accelerometer, no physics veto: plan B's step
    let flown = falling_under(long, |_| vec![], 1000);
    assert_eq!(flown.stages(), [(200, Stage::Gliding)]);
}

#[test]
fn the_backstops_fire_through_a_veto() {
    // the burnout under the thrust veto
    let flown = falling_under(spec(), |_| vec![nose(8.0), nose(8.0)], 1000);
    assert_eq!(
        advanced(&flown),
        [(500, "gliding: burnout timeout (no separation)".to_string())]
    );
    // the flight timeout under it
    let short = Settings {
        boost_timeout_ms: 5000,
        flight_timeout_ms: 800,
        ..spec()
    };
    let flown = falling_under(short, |_| vec![nose(8.0), nose(8.0)], 1000);
    assert_eq!(advanced(&flown), [(800, "done: flight timeout".to_string())]);
    // the landing timeout under a stillness veto: the BMI323 rocks at 1.5 g beside a still LSM6DSO32
    let landing = Settings {
        land_timeout_ms: 1000,
        ..spec()
    };
    let rocking = ticks(0, 1200, |_| read(&[flat(1.2), flat(1.5)], &[]));
    let flown = fly(landing, Stage::Landing, &rocking);
    assert_eq!(advanced(&flown), [(1000, "done: landing timeout 1 s".to_string())]);
    assert_eq!(flown.episodes()[0], (0, "begin", Detector::Stillness));
    // the row's value is the deciding LSM6DSO32's |accel|, in g
    let (_, veto) = begun(&flown)[0];
    assert_eq!((veto.decider, veto.value), (Decider::Accelerometer(0), 1.2));
    // NEGATIVE: both still, stillness lands it after ground_ms
    let still = ticks(0, 1200, |_| read(&[flat(1.0), flat(1.05)], &[]));
    assert_eq!(fly(landing, Stage::Landing, &still).stages(), [(300, Stage::Done)]);
}

#[test]
fn the_gnss_never_launches_and_never_votes() {
    // the GNSS's elevation over the line, risen from its own level: no launch, it has no barometer slot
    let gnss = ticks(0, 3000, |now_ms| SourceReadings {
        gnss_elevation: Some(if now_ms < 1500 { 0.0 } else { 15.0 }),
        ..SourceReadings::default()
    });
    assert_eq!(fly(spec(), Stage::Setting, &gnss).stages(), []);
    // the ICP-10111 falls past its line at 100, the BMP581 raises its peak every step, the GNSS reads flat: two live
    // voters, one contrary -- held; were the GNSS counted, one of three would not hold it
    let long = Settings {
        boost_timeout_ms: 5000,
        ..spec()
    };
    let boost = ticks(0, 1000, |now_ms| {
        let primary = if now_ms < 100 { 200.0 } else { 150.0 };
        let mut readings = read(&[], &[Some(primary), Some(200.0 + now_ms as f32 / 10.0)]);
        readings.gnss_elevation = Some(200.0);
        readings
    });
    let flown = fly(long, Stage::Boosting, &boost);
    assert_eq!(flown.stages(), []);
    let (_, veto) = begun(&flown)[0];
    assert_eq!((veto.kinds(), veto.contrary, veto.live), ("peer", 1, 2));
    // with no barometer live, the GNSS decides the apogee (plan B's fallback), and nothing votes against it
    let gnss_only = ticks(0, 1000, |now_ms| SourceReadings {
        gnss_elevation: Some(if now_ms < 100 { 200.0 } else { 150.0 }),
        ..SourceReadings::default()
    });
    let flown = fly(long, Stage::Boosting, &gnss_only);
    assert_eq!(advanced(&flown), [(200, "gliding: apogee 200m".to_string())]);
}

#[test]
fn a_veto_mid_apogee_dwell_restarts_it() {
    // the ICP-10111 falls past its line at 100, its dwell due at 200; the BMP581 raises its peak at 150 alone: one
    // vetoed step, after which the fall holds launch_ms afresh, from 200
    let long = Settings {
        boost_timeout_ms: 5000,
        ..spec()
    };
    let boost = ticks(0, 600, |now_ms| {
        let primary = if now_ms < 100 { 200.0 } else { 150.0 };
        let peer = match now_ms {
            150 => 210.0,
            200.. => 150.0,
            _ => 200.0,
        };
        read(&[], &[Some(primary), Some(peer)])
    });
    let flown = fly(long, Stage::Boosting, &boost);
    assert_eq!(advanced(&flown), [(300, "gliding: apogee 200m".to_string())]);
    assert_eq!(
        flown.episodes(),
        [(150, "begin", Detector::Apogee), (200, "end", Detector::Apogee)]
    );
}

#[test]
fn the_gnss_tracks_while_a_barometer_decides() {
    // the elevation climbs to 250 m at 500 and falls 10 m a step; the GNSS reads it 7 m high all along. The ICP-10111
    // decides and calls the apogee at 750 with its peak, 237 m
    let long = Settings {
        boost_timeout_ms: 5000,
        ..spec()
    };
    let elevation = |now_ms: u32| {
        if now_ms <= 500 {
            200.0 + now_ms as f32 / 10.0
        } else {
            250.0 - (now_ms - 500) as f32 / 5.0
        }
    };
    let flight = |dies_ms: u32| {
        ticks(0, 1500, move |now_ms| SourceReadings {
            gnss_elevation: Some(elevation(now_ms) + 7.0),
            ..read(&[], &[(now_ms < dies_ms).then(|| elevation(now_ms))])
        })
    };
    let flown = fly(long, Stage::Boosting, &flight(u32::MAX));
    assert_eq!(advanced(&flown), [(750, "gliding: apogee 237m".to_string())]);
    // the ICP-10111 dies at 650, mid-dwell: the GNSS stands in with its own tracker, current because it was fed while
    // the ICP-10111 decided, and calls the apogee on the same step with its own peak
    let flown = fly(long, Stage::Boosting, &flight(650));
    assert_eq!(advanced(&flown), [(750, "gliding: apogee 244m".to_string())]);
    assert!(flown.edges.is_empty());
}

#[test]
fn a_gnss_back_from_a_gap_sits_out_before_it_decides() {
    // the GNSS reads 200 m to 100 and is gone until 450 while the ICP-10111 reads 200 m on; the ICP-10111 dies at 450 as
    // the GNSS comes back at 150 m. It joins: below its own 200 m peak at once, it decides only launch_ms later, so the
    // fall holds from 550 and the apogee comes at 650
    let long = Settings {
        boost_timeout_ms: 5000,
        ..spec()
    };
    let boost = ticks(0, 1000, |now_ms| SourceReadings {
        gnss_elevation: match now_ms {
            0..=100 => Some(200.0),
            450.. => Some(150.0),
            _ => None,
        },
        ..read(&[], &[(now_ms < 450).then_some(200.0)])
    });
    let flown = fly(long, Stage::Boosting, &boost);
    assert_eq!(advanced(&flown), [(650, "gliding: apogee 200m".to_string())]);
}

#[test]
fn both_vetoes_at_once_name_both_kinds() {
    // the ICP-10111 climbs 15 m off a 1 m pad, the BMP581 stays on it, and both IMUs rest: held by the peer and at
    // rest. The row's value is the deciding elevation, 16 m, not its 15 m rise
    let climb = |now_ms: u32| [Some(if now_ms < 1050 { 1.0 } else { 16.0 }), Some(1.0)];
    let flown = fly(
        spec(),
        Stage::Setting,
        &pad_then(climb, |_| vec![flat(1.0), flat(1.0)], 1300),
    );
    assert_eq!(flown.stages(), []);
    let (now_ms, veto) = begun(&flown)[0];
    assert_eq!(
        (
            now_ms,
            veto.detector,
            veto.kinds(),
            veto.contrary,
            veto.live,
            veto.value
        ),
        (1050, Detector::LaunchAltitude, "peer+at_rest", 1, 2, 16.0)
    );
    // the ICP-10111 falls past its line at 100 while the BMP581 raises its peak, both IMUs under thrust
    let long = Settings {
        boost_timeout_ms: 5000,
        ..spec()
    };
    let boost = ticks(0, 400, |now_ms| {
        let metres = if now_ms < 100 { 200.0 } else { 150.0 };
        read(
            &[nose(8.0), nose(8.0)],
            &[Some(metres), Some(200.0 + now_ms as f32 / 10.0)],
        )
    });
    let flown = fly(long, Stage::Boosting, &boost);
    let (now_ms, veto) = begun(&flown)[0];
    assert_eq!(
        (now_ms, veto.detector, veto.kinds(), veto.contrary, veto.live),
        (100, Detector::Apogee, "peer+thrust", 1, 2)
    );
    // NEGATIVE: the GNSS deciding has no peers: held under thrust alone, with nothing live to vote
    let boost = ticks(0, 400, |now_ms| SourceReadings {
        gnss_elevation: Some(if now_ms < 100 { 200.0 } else { 150.0 }),
        ..read(&[nose(8.0), nose(8.0)], &[])
    });
    let flown = fly(long, Stage::Boosting, &boost);
    assert_eq!(flown.stages(), []);
    let (now_ms, veto) = begun(&flown)[0];
    assert_eq!(
        (now_ms, veto.decider, veto.kinds(), veto.contrary, veto.live),
        (100, Decider::Gnss, "thrust", 0, 0)
    );
}

#[test]
fn a_tie_of_two_in_four_vetoes_and_one_in_three_does_not() {
    let boost = |accelerations: Vec<Option<[f32; 3]>>| {
        let still: Vec<Option<[f32; 3]>> = accelerations.iter().map(|_| flat(1.0)).collect();
        ticks(0, 300, move |now_ms| {
            read(if now_ms == 0 { &still } else { &accelerations }, &[])
        })
    };
    assert_eq!(ACCELEROMETER_SLOTS, 4);
    let tie = fly(
        spec(),
        Stage::Setting,
        &boost(vec![flat(3.0), flat(3.0), flat(1.0), flat(1.0)]),
    );
    assert_eq!(tie.stages(), []);
    let (_, veto) = begun(&tie)[0];
    assert_eq!((veto.contrary, veto.live), (2, 4));
    let minority = fly(spec(), Stage::Setting, &boost(vec![flat(3.0), flat(3.0), flat(1.0)]));
    assert_eq!(
        (minority.stages(), minority.edges.len()),
        (vec![(150, Stage::Boosting)], 0)
    );
}

#[test]
fn a_source_back_from_a_gap_longer_than_the_window_confirms_no_level() {
    // NEGATIVE: the BMP581 alone, on the pad at 0 m, silent from 1050 to 2200, back at 15 m: the pad it held is past
    // the window, so the 15 m rose from nothing, then holds as its own level -- no launch
    let back = |now_ms: u32| match now_ms {
        0..=1000 => Some(0.0),
        1050..=2200 => None,
        _ => Some(15.0),
    };
    let alone = pad_then(|now_ms| [None, back(now_ms)], |_| vec![], 4000);
    assert_eq!(fly(spec(), Stage::Setting, &alone).stages(), []);
    // nor a peer vote: the ICP-10111 climbs 15 m at 2250 off the pad it held all along. The BMP581, back at 0 m
    // after its gap, votes only once it has held a level of its own for launch_ms: back at 2250, from 2350 on, the
    // step the climb's dwell would complete
    let climb_with = |back_ms: u32| {
        move |now_ms: u32| {
            let peer = (now_ms <= 1000 || now_ms >= back_ms).then_some(0.0);
            [Some(if now_ms < 2250 { 0.0 } else { 15.0 }), peer]
        }
    };
    let flown = fly(spec(), Stage::Setting, &pad_then(climb_with(2250), |_| vec![], 2600));
    assert_eq!(flown.stages(), []);
    assert_eq!(flown.episodes()[0], (2350, "begin", Detector::LaunchAltitude));
    // back one step later, it holds its level too late: the launch goes
    let flown = fly(spec(), Stage::Setting, &pad_then(climb_with(2300), |_| vec![], 2400));
    assert_eq!((flown.stages(), flown.edges.len()), (vec![(2350, Stage::Boosting)], 0));
    // NEGATIVE: the same peer live all along at 0 m holds the climb from its first step
    let flown = fly(spec(), Stage::Setting, &pad_then(climb_with(0), |_| vec![], 2600));
    assert_eq!(flown.stages(), []);
    assert_eq!(flown.episodes()[0], (2250, "begin", Detector::LaunchAltitude));
}

#[test]
fn a_decider_dying_as_its_peer_returns_fires_nothing_on_the_handover() {
    // NEGATIVE: the ICP-10111 climbs 15 m at 1050 off its pad and dies at 1150, half way through its dwell; the
    // BMP581, silent since 50, returns at 15 m on that step with no level of its own: the dwell drains, no launch.
    // Plan B's fused read carries the 15 m on and launches there
    let climb = |now_ms: u32| match now_ms {
        0..=1000 => [Some(0.0), if now_ms == 0 { Some(0.0) } else { None }],
        1050 | 1100 => [Some(15.0), None],
        _ => [None, Some(15.0)],
    };
    let launch = pad_then(climb, |_| vec![], 1600);
    assert_eq!(
        fly_fused(spec(), Stage::Setting, &launch).stages(),
        [(1150, Stage::Boosting)]
    );
    assert_eq!(fly(spec(), Stage::Setting, &launch).stages(), []);
}

#[test]
fn sources_past_their_class_s_slots_are_ignored_with_a_log_line() {
    let binding = Binding::new(
        ["imu_lsm6dso32", "imu_bmi323", "accel_adxl375", "imu_bno055", "imu_sim"],
        ["baro_icp10111", sequencer::GNSS_SOURCE, "baro_bmp581", "baro_bmp280"],
        ["laser_vl53l1x", "laser_vl53l4cx"],
    );
    let lines: Vec<String> = binding.ignored().iter().map(ToString::to_string).collect();
    assert_eq!(
        lines,
        [
            "imu_sim ignored: the stage machine's accel slots are full (4)",
            "baro_bmp280 ignored: the stage machine's elevation slots are full (2)",
            "laser_vl53l4cx ignored: the stage machine's agl slots are full (1)",
        ]
    );
    // the GNSS takes its own slot by name, wherever it ranks
    assert_eq!(binding.source(Decider::Barometer(1)), Some("baro_bmp581"));
    assert_eq!(binding.source(Decider::Gnss), Some(sequencer::GNSS_SOURCE));
    assert_eq!(binding.source(Decider::Accelerometer(3)), Some("imu_bno055"));
    // the readings by rank, an ignored source never read
    let readings = binding.readings(
        |source| match source {
            "imu_lsm6dso32" => flat(1.0),
            "imu_bmi323" => None,
            "imu_sim" => panic!("an ignored source was read"),
            _ => flat(1.1),
        },
        |source| match source {
            "baro_icp10111" => Some(1.0),
            "baro_bmp581" => Some(2.0),
            sequencer::GNSS_SOURCE => Some(3.0),
            _ => panic!("an ignored source was read"),
        },
        |source| match source {
            "laser_vl53l1x" => Some(0.5),
            _ => panic!("an ignored source was read"),
        },
    );
    assert_eq!(readings.accelerations, [flat(1.0), None, flat(1.1), flat(1.1)]);
    assert_eq!(
        (
            readings.elevations,
            readings.gnss_elevation,
            readings.height_above_ground
        ),
        ([Some(1.0), Some(2.0)], Some(3.0), Some(0.5))
    );
    // NEGATIVE: a board with one source each binds them all, ignores none, and leaves the other slots empty
    let one = Binding::new(["imu_lsm6dso32"], ["baro_icp10111"], []);
    assert!(one.ignored().is_empty());
    assert_eq!(
        (one.source(Decider::Accelerometer(1)), one.source(Decider::Gnss)),
        (None, None)
    );
    let readings = one.readings(|_| flat(1.0), |_| Some(1.0), |_| panic!("no laser is bound"));
    assert_eq!(readings.accelerations, [flat(1.0), None, None, None]);
    assert_eq!(readings.height_above_ground, None);
}

#[test]
fn the_machine_fits_its_budget() {
    // what sits on the control thread's stack: the climbs' histories (81 slots each) are boxed on the heap, and the
    // unit tests hold the whole within 3 KB
    assert!(size_of::<StageMachine>() <= 1024, "{} B", size_of::<StageMachine>());
}

/// The sequencer entry of a board config document (its `components`).
fn sequencer_entry(document: &Value) -> Settings {
    let entry = document["components"]
        .as_array()
        .expect("components")
        .iter()
        .find(|component| component["activity"] == "sequencer")
        .expect("a sequencer entry");
    Settings::from_config(entry).expect("a valid sequencer entry")
}

#[test]
fn every_shipped_launch_line_is_above_the_still_band() {
    // a boost over launch_g is never at rest and never a contrary peer of another boost
    let mut shipped = vec![Settings::default()];
    let defaults: Value = serde_json::from_str(CONFIG_DEFAULT).expect("config_default.json");
    shipped.push(sequencer_entry(&defaults["document"]));
    for text in LAUNCH_CONFIGS {
        shipped.push(sequencer_entry(&serde_json::from_str(text).expect("a launch config")));
    }
    for settings in shipped {
        assert!(settings.launch_g > 1.0 + settings.still_g, "{settings:?}");
    }
}

/// TMS-7's trace on the machine's step grid: every [`PERIOD_MS`] from its first row, the latest row at or before the
/// step (the 50 Hz logger read every 20 ms), as the step clock (ms from the first row), the accel in the body frame
/// (+x the nose: the logger's -y read +1 g on the pad nose up) and the barometer's elevation.
fn tms7() -> Vec<(u32, [f32; 3], f32)> {
    let rows: Vec<(i64, [f32; 3], f32)> = TMS7
        .lines()
        .skip(1)
        .map(|line| {
            // the gyro cells are empty on one impact row; the columns read here never are
            let cells: Vec<&str> = line.split(';').collect();
            let number = |column: usize| -> f64 { cells[column].parse().expect("a number") };
            let [x, y, z] = [number(1), number(2), number(3)].map(|axis| axis as f32);
            ((number(0) * 1000.0).round() as i64, [-y, x, z], number(10) as f32)
        })
        .collect();
    let first_ms = rows[0].0;
    let mut steps = Vec::new();
    let mut next = 0;
    for step_ms in (first_ms..=rows[rows.len() - 1].0).step_by(PERIOD_MS as usize) {
        while next + 1 < rows.len() && rows[next + 1].0 <= step_ms {
            next += 1;
        }
        let (_, accel, metres) = rows[next];
        steps.push(((step_ms - first_ms) as u32, accel, metres));
    }
    steps
}

/// The firmware's default sequencer settings: plan B's config_default entry.
fn shipped() -> Settings {
    let defaults: Value = serde_json::from_str(CONFIG_DEFAULT).expect("config_default.json");
    sequencer_entry(&defaults["document"])
}

/// TMS-7 as the primary alone: the nose logger's BMI323 and BMP581 in the first slots, each standing for its class's
/// primary (the LSM6DSO32 and the ICP-10111 on the boards).
fn tms7_alone() -> Vec<(u32, SourceReadings)> {
    tms7()
        .iter()
        .map(|&(now_ms, accel, metres)| (now_ms, read(&[Some(accel)], &[Some(metres)])))
        .collect()
}

/// TMS-7 with a second source per class: the barometer again `offset_m` off, the accelerometer again `late` steps
/// late; `alive(now_ms)` says which of the primary's (accel, barometer) publish.
fn tms7_twice(offset_m: f32, late: usize, alive: impl Fn(u32) -> (bool, bool)) -> Vec<(u32, SourceReadings)> {
    let trace = tms7();
    trace
        .iter()
        .enumerate()
        .map(|(index, &(now_ms, accel, metres))| {
            let (accelerating, climbing) = alive(now_ms);
            let backup = trace[index.saturating_sub(late)].1;
            let readings = read(
                &[accelerating.then_some(accel), Some(backup)],
                &[climbing.then_some(metres), Some(metres + offset_m)],
            );
            (now_ms, readings)
        })
        .collect()
}

/// The detector that advances to `stage`.
fn detector(stage: Stage) -> Option<Detector> {
    match stage {
        Stage::Boosting => Some(Detector::LaunchAcceleration),
        Stage::Gliding => Some(Detector::Apogee),
        Stage::Done => Some(Detector::Stillness),
        Stage::Setting | Stage::Landing => None,
    }
}

#[test]
fn tms7_on_two_sources_per_class_keeps_every_stage_or_delays_it_with_a_veto() {
    let reference = fly(shipped(), Stage::Setting, &tms7_alone());
    // the one-source machine flies TMS-7 through every stage
    let stages: Vec<Stage> = reference.advances.iter().map(|(_, stage, _)| *stage).collect();
    assert_eq!(stages, [Stage::Boosting, Stage::Gliding, Stage::Landing, Stage::Done]);
    assert!(
        reference.advances[0].2.starts_with("launch |a|="),
        "{:?}",
        reference.advances
    );
    assert!(
        reference.advances[1].2.starts_with("apogee "),
        "{:?}",
        reference.advances
    );
    let twice = fly(shipped(), Stage::Setting, &tms7_twice(12.0, 1, |_| (true, true)));
    assert_eq!(twice.advances.len(), reference.advances.len(), "{twice:?}");
    let mut previous_ms = 0;
    for ((reference_ms, stage, _), (now_ms, twice_stage, _)) in reference.advances.iter().zip(&twice.advances) {
        assert_eq!(stage, twice_stage);
        assert!(now_ms >= reference_ms, "{stage:?} at {now_ms}, before {reference_ms}");
        if now_ms > reference_ms {
            let vetoed = twice.edges.iter().any(|(edge_ms, edge)| {
                (previous_ms..=*now_ms).contains(edge_ms) && Some(edge.veto().detector) == detector(*stage)
            });
            assert!(
                vetoed,
                "{stage:?} late at {now_ms} without a veto: {:?}",
                twice.episodes()
            );
        }
        previous_ms = *now_ms;
    }
    // the late BMI323 still reads the pad on the first boost step, and still swings on the first step at rest: the
    // launch and the stillness each come one step late, the 12 m offset moves nothing
    let late: Vec<u32> = reference
        .advances
        .iter()
        .zip(&twice.advances)
        .map(|((reference_ms, _, _), (now_ms, _, _))| now_ms - reference_ms)
        .collect();
    assert_eq!(late, [PERIOD_MS, 0, 0, PERIOD_MS]);
}

#[test]
fn a_barometer_dropout_at_the_top_is_carried_by_its_peer() {
    let reference = fly(shipped(), Stage::Setting, &tms7_alone());
    let (apogee_ms, _, apogee) = reference.advances[1].clone();
    // the ICP-10111 silent from 1.5 s before the apogee to 1 s after: the BMP581, 12 m off, decides on its own
    // tracker and calls the apogee on the same step, its offset never read as a drop
    let back_ms = apogee_ms + 1000;
    let silent = |now_ms: u32| (true, !(apogee_ms - 1500..back_ms).contains(&now_ms));
    let twice = fly(shipped(), Stage::Setting, &tms7_twice(12.0, 0, silent));
    let (now_ms, stage, reason) = &twice.advances[1];
    assert_eq!((*now_ms, *stage), (apogee_ms, Stage::Gliding));
    // its peak, 12 m above the primary's
    let peak = |text: &str| -> f32 { text["apogee ".len()..text.len() - 1].parse().unwrap() };
    assert_eq!(peak(reason), peak(&apogee) + 12.0, "{reason} {apogee}");
    // NEGATIVE: the primary alone with the same dropout calls it only once it is back, later
    let alone: Vec<(u32, SourceReadings)> = tms7_alone()
        .into_iter()
        .map(|(now_ms, mut readings)| {
            if !silent(now_ms).1 {
                readings.elevations[0] = None;
            }
            (now_ms, readings)
        })
        .collect();
    let dropped = fly(shipped(), Stage::Setting, &alone);
    let (dropped_ms, stage, reason) = &dropped.advances[1];
    assert!(*dropped_ms >= back_ms && reason.starts_with("apogee "), "{dropped:?}");
    assert_eq!(*stage, Stage::Gliding);
}

#[test]
fn a_barometer_back_from_a_gap_joins_the_flight_afresh() {
    // the BMP581 an exact copy of the ICP-10111 on TMS-7, and the ICP-10111 silent from arming (or from half a second
    // after it) until 0.3 s before the apogee. Back, it joins: its smoothing restarts on its reading, and the BMP581,
    // whose record is unbroken, decides on: the apogee on the reference step. Held through the gap, its smoothing
    // crawled up from the elevation at arming, and the apogee came 1.2 s late after a 3 s gap and never after this one
    let reference = fly(shipped(), Stage::Setting, &tms7_alone());
    let (launch_ms, apogee_ms) = (reference.advances[0].0, reference.advances[1].0);
    let armed_ms = launch_ms + shipped().apogee_arm_ms;
    for silent_from_ms in [armed_ms, armed_ms + 500, apogee_ms - 3300] {
        let silent = |now_ms: u32| (true, !(silent_from_ms..apogee_ms - 300).contains(&now_ms));
        let twice = fly(shipped(), Stage::Setting, &tms7_twice(0.0, 0, silent));
        assert_eq!(twice.stages(), reference.stages(), "silent from {silent_from_ms}");
        assert!(twice.edges.is_empty(), "{:?}", twice.episodes());
    }
    // NEGATIVE: the BMP581 back 50 ms before the apogee after the same gap sits out, then votes on its own restarted
    // smoothing, which raises nothing past the top: no veto. Held through the gap, it voted the apogee 0.85 s late
    for silent_from_ms in [armed_ms, armed_ms + 500] {
        let silent = |now_ms: u32| !(silent_from_ms..apogee_ms - 50).contains(&now_ms);
        let ticks: Vec<(u32, SourceReadings)> = tms7_twice(0.0, 0, |_| (true, true))
            .into_iter()
            .map(|(now_ms, mut readings)| {
                readings.elevations[1] = readings.elevations[1].filter(|_| silent(now_ms));
                (now_ms, readings)
            })
            .collect();
        let twice = fly(shipped(), Stage::Setting, &ticks);
        assert_eq!(twice.stages(), reference.stages(), "silent from {silent_from_ms}");
        assert!(twice.edges.is_empty(), "{:?}", twice.episodes());
    }
    // a joined source still decides when no barometer with an unbroken record is live: the BMP581 gone from 0.2 s
    // before the apogee, the ICP-10111 calls it on its own record, which lacks the top -- later, and lower
    let alone: Vec<(u32, SourceReadings)> =
        tms7_twice(0.0, 0, |now_ms| (true, !(armed_ms..apogee_ms - 300).contains(&now_ms)))
            .into_iter()
            .map(|(now_ms, mut readings)| {
                if now_ms >= apogee_ms - 200 {
                    readings.elevations[1] = None;
                }
                (now_ms, readings)
            })
            .collect();
    let flown = fly(shipped(), Stage::Setting, &alone);
    assert_eq!(
        (flown.advances[1].0, flown.advances[1].2.as_str()),
        (apogee_ms + 750, "apogee 474m")
    );
}

#[test]
fn handovers_on_tms7_keep_every_stage() {
    // the backups as exact copies: the LSM6DSO32 dies one second after ignition, the ICP-10111 halfway down the
    // descent; the BMI323 and the BMP581 carry the boost's end, the apogee, the landing and the stillness
    let reference = fly(shipped(), Stage::Setting, &tms7_alone());
    let launch_ms = reference.advances[0].0;
    let landing_ms = reference.advances[2].0;
    let alive = |now_ms: u32| (now_ms < launch_ms + 1000, now_ms < landing_ms - 30_000);
    let twice = fly(shipped(), Stage::Setting, &tms7_twice(0.0, 0, alive));
    assert_eq!(twice.stages(), reference.stages());
    assert!(twice.edges.is_empty(), "{:?}", twice.episodes());
}
