//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The warm start answers as plan B does: src/avionics/golden/warmstart.json, which golden/build_warmstart.py builds
//! from src/glider/warmstart.py on CPython. Six sections, each a golden fixture of its own: the gate, the crumb
//! read-back, the save order, the apply plan, the boot decision, and the checkpoint schedule replayed poll by poll
//! against plan B's own Checkpoint loop. A case marked `divergence` (the operator's boot rules, a stage id that is
//! none, CPU_LOCKUP, a blob that is no object) carries the Rust answer, which is owed instead of plan B's.

use golden::{Compare, Fixture};
use sequencer::Stage;
use serde_json::{Map, Value, json};
use warmstart::{
    ApplyPlan, CRUMB_KEY, Checkpoint, Crumb, Freeze, Identity, Live, Nvs, NvsError, Restore, STAGE_KEY, load, restore,
    save, should_restore,
};

/// plan B's warm start over the gate, load, save, apply and restore cases.
const WARMSTART: &str = include_str!("../../../golden/warmstart.json");

/// An in-memory NVS namespace holding a stage and a blob, recording each write as the fixture writes it.
struct Memory {
    /// The stage i32, None when never written.
    stage: Option<i32>,
    /// The crumb blob, None when never written.
    blob: Option<Vec<u8>>,
    /// The writes: `["set_blob", key, <the blob's JSON>]`, `["set_i32", key, value]`, `["commit"]`.
    writes: Vec<Value>,
}

impl Memory {
    /// The store a case's input describes: `stage` and `blob` (text) or `blob_hex` (bytes), null when unwritten.
    fn from_case(input: &Value) -> Memory {
        let blob = match (input.get("blob"), input.get("blob_hex")) {
            (_, Some(hex)) => Some(bytes(hex.as_str().expect("blob_hex is text"))),
            (Some(Value::String(text)), _) => Some(text.as_bytes().to_vec()),
            _ => None,
        };
        Memory {
            stage: input["stage"]
                .as_i64()
                .map(|stage| i32::try_from(stage).expect("an i32 stage")),
            blob,
            writes: Vec::new(),
        }
    }
}

impl Nvs for Memory {
    fn get_i32(&mut self, key: &str) -> Result<i32, NvsError> {
        assert_eq!(key, STAGE_KEY);
        self.stage.ok_or(NvsError(0x1102))
    }

    fn get_blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, NvsError> {
        assert_eq!(key, CRUMB_KEY);
        let blob = self.blob.as_ref().ok_or(NvsError(0x1102))?;
        buffer
            .get_mut(..blob.len())
            .ok_or(NvsError(0x110c))?
            .copy_from_slice(blob);
        Ok(blob.len())
    }

    fn set_blob(&mut self, key: &str, value: &[u8]) -> Result<(), NvsError> {
        let parsed: Value = serde_json::from_slice(value).expect("the blob is JSON");
        self.writes.push(json!(["set_blob", key, parsed]));
        self.blob = Some(value.to_vec());
        Ok(())
    }

    fn set_i32(&mut self, key: &str, value: i32) -> Result<(), NvsError> {
        self.writes.push(json!(["set_i32", key, value]));
        self.stage = Some(value);
        Ok(())
    }

    fn commit(&mut self) -> Result<(), NvsError> {
        self.writes.push(json!(["commit"]));
        Ok(())
    }
}

/// The bytes a hex string names.
fn bytes(hex: &str) -> Vec<u8> {
    (0..hex.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).expect("hex"))
        .collect()
}

/// A fixture crumb: null is no crumb, an object is read by the port's own reader.
fn crumb(value: &Value) -> Option<Crumb> {
    if value.is_null() {
        return None;
    }
    Some(Crumb::from_json(value).expect("a fixture crumb is an object"))
}

/// An f32 as the JSON number its shortest text is: how the Rust writer puts it in a blob, and so the decimal plan B
/// wrote (every fixture float has at most 7 significant digits; the builder checks).
fn decimal(value: f32) -> Value {
    serde_json::from_str(&serde_json::to_string(&value).expect("an f32 serializes")).expect("JSON")
}

/// A crumb as the fixture's load section holds it: the blob the Rust writer makes of it, nulls dropped.
fn readable(crumb: &Crumb) -> Value {
    let mut value: Value = serde_json::from_str(&crumb.to_json()).expect("the writer writes JSON");
    value
        .as_object_mut()
        .expect("an object")
        .retain(|_key, member| !member.is_null());
    value
}

/// The identity as the apply section's `seed` holds it: its members, none of them null.
fn seed(identity: Identity) -> Value {
    let blob = readable(&Crumb {
        identity,
        ..Crumb::default()
    });
    let members: Map<String, Value> = blob
        .as_object()
        .expect("an object")
        .iter()
        .filter(|(key, _value)| ["launch", "zone", "pad_altitude", "pitot_zero"].contains(&key.as_str()))
        .map(|(key, value)| (key.clone(), value.clone()))
        .collect();
    Value::Object(members)
}

/// The answer a case owes: the Rust answer a divergence carries (its members but the note), else plan B's.
fn owed(case: &Value) -> Value {
    match case.get("divergence") {
        Some(Value::Object(divergence)) => {
            assert!(
                divergence
                    .get("note")
                    .and_then(Value::as_str)
                    .is_some_and(|note| !note.is_empty()),
                "a divergence says why: {case}"
            );
            let answer: Map<String, Value> = divergence
                .iter()
                .filter(|(key, _value)| key.as_str() != "note")
                .map(|(key, value)| (key.clone(), value.clone()))
                .collect();
            Value::Object(answer)
        }
        Some(other) => panic!("a divergence is an object: {other}"),
        None => case["output"].clone(),
    }
}

/// Run a section's cases through `port`, each against the answer it owes, under the section's comparison.
///
/// Panics listing every failing case, and when the section has no cases.
fn check(name: &str, mut port: impl FnMut(&Value) -> Value) {
    let fixture = Fixture::parse(WARMSTART);
    let section = fixture.get(name);
    let compare = Compare::from_json(&section["compare"]);
    let cases = section["cases"].as_array().expect("cases");
    assert!(!cases.is_empty(), "{name}: no cases");
    let failures: Vec<String> = cases
        .iter()
        .filter_map(|case| {
            let label = format!("{name} {}", case["name"].as_str().expect("a case name"));
            compare.check(&label, &owed(case), &port(&case["input"])).err()
        })
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} cases failed:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(WARMSTART);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG, "{meta}");
    assert_eq!(meta["generator"], "warmstart");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/warmstart.py"));
}

#[test]
fn the_gate_matches_plan_b() {
    check("gate", |input| {
        let (restore, reason) = should_restore(
            crumb(&input["crumb"]).as_ref(),
            input["separated"].as_bool().expect("separated"),
            input["now_s"].as_i64().expect("now_s"),
        );
        json!({"restore": restore, "reason": reason.to_string()})
    });
}

#[test]
fn the_crumb_reads_back_as_plan_b_reads_it() {
    check("load", |input| {
        let loaded = load(&mut Memory::from_case(input));
        json!({"crumb": loaded.as_ref().map(readable)})
    });
}

#[test]
fn save_writes_what_plan_b_writes_in_its_order() {
    check("save", |input| {
        let mut nvs = Memory::from_case(&json!({}));
        assert!(save(&mut nvs, &crumb(input).expect("a crumb")));
        json!({"writes": nvs.writes})
    });
}

#[test]
fn the_apply_plan_matches_plan_b() {
    check("apply", |input| {
        let sensors: Vec<(&str, &str)> = input["sensors"]
            .as_array()
            .expect("sensors")
            .iter()
            .map(|sensor| (sensor["name"].as_str().unwrap(), sensor["driver"].as_str().unwrap()))
            .collect();
        let plan = ApplyPlan::new(&crumb(&input["crumb"]).expect("a crumb"), sensors).expect("a flight stage");
        let mut updates = Vec::new();
        if let Some(rebase) = &plan.rebase {
            for baro in &rebase.baros {
                updates.push(json!([baro, {"ground": decimal(rebase.ground_m)}]));
            }
        }
        if let Some(zero) = plan.pitot_zero_pa {
            updates.push(json!(["airspeed_sdp810", {"zero_offset_pa": decimal(zero)}]));
        }
        if let Some(airspeed) = plan.airspeed_ms {
            updates.push(json!(["flight", {"seed_airspeed": decimal(airspeed)}]));
        }
        json!({"mission": plan.mission_update(), "updates": updates, "seed": seed(plan.identity),
            "stage": plan.stage as u8, "armed": plan.armed, "warm_started": true})
    });
}

#[test]
fn the_boot_decision_matches_plan_b() {
    check("restore", |input| {
        let mut nvs = Memory::from_case(input);
        let decision = restore(
            &mut nvs,
            input["warm_start"].as_bool().unwrap_or(true),
            input["separated"].as_bool().expect("separated"),
            input["now_s"].as_i64().expect("now_s"),
            [("baro_icp10111", "icp10111")],
        );
        json!({"warm": matches!(decision, Restore::Warm { .. }), "writes": nvs.writes})
    });
}

#[test]
fn every_divergence_is_one() {
    // a marked case must differ from plan B, or the mark hides a real comparison
    let fixture = Fixture::parse(WARMSTART);
    let mut marked = 0;
    for section in ["gate", "load", "save", "apply", "restore"] {
        for case in fixture.get(section)["cases"].as_array().unwrap() {
            if case.get("divergence").is_some() {
                marked += 1;
                assert_ne!(owed(case), case["output"], "{section} {}", case["name"]);
            }
        }
    }
    assert!(marked > 0, "the boot rules diverge from plan B somewhere");
}

#[test]
#[should_panic(expected = "cases failed")]
fn a_wrong_gate_fails_the_comparison() {
    // plan B's own gate (no boot rules: DONE and SETTING restore) must fail where the fixture marks the rules
    check("gate", |input| {
        let crumb = crumb(&input["crumb"]);
        let stage = crumb.as_ref().and_then(|crumb| crumb.stage);
        let on_ground = matches!(stage, Some(1) | Some(5));
        let (restore, reason) = should_restore(
            crumb.as_ref(),
            input["separated"].as_bool().unwrap(),
            input["now_s"].as_i64().unwrap(),
        );
        json!({"restore": restore || on_ground, "reason": reason.to_string()})
    });
}

/// A fixture reading as f32, None for null (a stale source).
fn reading(value: &Value) -> Option<f32> {
    value.as_f64().map(|value| value as f32)
}

/// Replay every checkpoint scenario poll by poll, the cadence scaled by `cadence`; the failing polls.
fn replay_checkpoints(cadence: f32) -> Vec<String> {
    let fixture = Fixture::parse(WARMSTART);
    let section = fixture.get("checkpoint");
    let compare = Compare::from_json(&section["compare"]);
    let mut failures = Vec::new();
    let mut polls = 0;
    for scenario in section["scenarios"].as_array().expect("scenarios") {
        let name = scenario["name"].as_str().expect("a scenario name");
        let init = &scenario["init"];
        let checkpoint_s = init["checkpoint_s"].as_f64().expect("checkpoint_s") as f32 * cadence;
        let mut schedule = Checkpoint::new(checkpoint_s, init["start_us"].as_u64().expect("start_us"));
        if let Some(seeded) = crumb(&init["seed"]) {
            schedule.seed(seeded.identity);
        }
        let mission = Crumb::from_json(&json!({"zone": init["zone"], "launch": init["launch"]})).expect("an object");
        let pitot_zero_pa = reading(&init["pitot_zero"]);
        let flight = init["flight"].as_bool().expect("flight");
        for (index, recorded) in scenario["steps"].as_array().expect("steps").iter().enumerate() {
            polls += 1;
            let input = &recorded["in"];
            let id = u8::try_from(input["stage"].as_u64().expect("stage")).expect("a stage id");
            let stage = Stage::from_id(id).expect("a flight stage");
            let altitude_m = reading(&input["altitude"]);
            let freeze = || Freeze {
                zone: mission.identity.zone,
                launch_point: mission.identity.launch,
                fresh_altitude_m: altitude_m,
                pitot_zero_pa,
                // plan B froze no barometer's own ground
                grounds_m: [None; warmstart::GROUND_SLOTS],
            };
            let step = schedule.step(stage, input["now_us"].as_u64().expect("now_us"), altitude_m, freeze);
            let ticks_ms = input["ticks_ms"].as_u64().expect("ticks_ms");
            let live = Live {
                altitude_m,
                speed_ms: reading(&input["speed"]),
                airspeed_ms: flight.then(|| reading(&input["airspeed"])).flatten(),
                ticks_ms,
            };
            let blob: Value = serde_json::from_str(
                &schedule
                    .crumb(stage, live, input["stamp_s"].as_i64().expect("stamp_s"))
                    .to_json(),
            )
            .expect("JSON");
            let armed = input["armed"].as_bool().expect("armed");
            let answer = json!({
                "checkpoint": step.checkpoint,
                "row": step.checkpoint.then(|| json!([id, blob["altitude"], blob["speed"], blob["airspeed"], ticks_ms])),
                "crumb": (step.checkpoint && armed).then_some(&blob),
                "identity": seed(*schedule.identity()),
            });
            if let Err(failure) = compare.check(&format!("{name} poll {index}"), &recorded["out"], &answer) {
                failures.push(failure);
            }
        }
    }
    assert!(polls > 0, "the checkpoint section has no polls");
    failures
}

#[test]
fn the_checkpoint_schedule_matches_plan_b() {
    let failures = replay_checkpoints(1.0);
    assert!(
        failures.is_empty(),
        "{} polls failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

#[test]
fn a_wrong_cadence_fails_the_checkpoint_comparison() {
    assert!(!replay_checkpoints(2.0).is_empty());
}
