//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The pad drift calibration answers as plan B does: src/avionics/golden/gnss_calib.json, which
//! golden/build_gnss_calib.py builds from src/glider/tasks/gnss_calib.py on CPython. Each case replays plan B's run()
//! pass by pass (the stage, the arming, the speed and course as read() answered them) and compares the state after
//! every pass.

use control::drift::DriftCalibrator;
use sequencer::Stage;
use serde_json::{Value, json};

/// plan B's calibrator over explicit and seeded pads.
const GNSS_CALIB: &str = include_str!("../../../golden/gnss_calib.json");

/// A reading as plan B's read() answered it: its value when it has a source (fresh), else None.
fn fresh(reading: &Value) -> Option<f32> {
    reading[1].as_str()?;
    reading[0].as_f64().map(|value| value as f32)
}

#[test]
fn the_drift_answers_as_plan_b() {
    golden::run(GNSS_CALIB, |input| {
        let minimum = u32::try_from(input["min_samples"].as_u64().expect("min_samples")).expect("a u32");
        let mut calibrator = DriftCalibrator::new(minimum);
        let states: Vec<Value> = input["steps"]
            .as_array()
            .expect("steps")
            .iter()
            .map(|step| {
                let stage = Stage::from_name(step["stage"].as_str().expect("a stage")).expect("a known stage");
                let armed = step["armed"].as_bool().expect("armed");
                let ground = fresh(&step["speed"])
                    .zip(fresh(&step["course"]))
                    .map(|(speed, course)| [speed, course]);
                calibrator.pass(stage, armed, ground);
                json!({"drift": calibrator.drift(), "samples": calibrator.samples(), "frozen": calibrator.frozen()})
            })
            .collect();
        Value::from(states)
    });
}

#[test]
fn the_freeze_reports_once_and_min_samples_zero_never_divides_by_zero() {
    let mut calibrator = DriftCalibrator::new(0);
    assert_eq!(calibrator.pass(Stage::Setting, true, None), None);
    assert_eq!(calibrator.pass(Stage::Boosting, true, None), Some(([0.0, 0.0], 0)));
    assert!(calibrator.frozen());
    assert_eq!(
        calibrator.pass(Stage::Gliding, true, Some([1.0, 0.0])),
        None,
        "frozen is final"
    );
    let mut calibrator = DriftCalibrator::new(2);
    calibrator.pass(Stage::Setting, true, Some([1.0, 90.0]));
    calibrator.pass(Stage::Setting, true, Some([1.0, 90.0]));
    let (drift, count) = calibrator.pass(Stage::Boosting, true, None).expect("the freeze");
    assert_eq!(count, 2);
    assert!((drift[0] - 1.0).abs() < 1e-6 && drift[1].abs() < 1e-6, "{drift:?}");
}
