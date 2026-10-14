//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the warm start, positive and negative: the crumb serde, the NVS order, the gate with its boot rules
//! and no time or cause check (Q19's step 2), the apply plan, the boot decision, the checkpoint schedule and its keeper over NVS. The plan B
//! comparison itself is tests/golden.rs.

use super::{
    ApplyPlan, BLOB_MAX, CRUMB_KEY, Checkpoint, Counters, Crumb, Freeze, GROUND_SLOTS, Identity, Keeper, Live,
    NO_STAGE, Nvs, NvsError, Pass, Reason, ResetReason, Restore, STAGE_KEY, Step, WARM_CAP, Write, clear, load,
    restore, save, should_restore,
};
use control::attitude::BIAS_SLOTS;
use control::navigation::Position;
use sequencer::{Flight, FlightWord, Request, Stage, Transition};
use serde_json::{Value, json};

/// ESP_ERR_NVS_NOT_FOUND, what esp-idf answers for a key never written.
const NOT_FOUND: NvsError = NvsError(0x1102);

/// ESP_ERR_NVS_NOT_ENOUGH_SPACE, what the [`Memory`] budget answers once spent.
const REFUSED: NvsError = NvsError(0x1105);

/// An in-memory NVS namespace that records every write, and can be told to refuse writes from the nth on.
#[derive(Default)]
struct Memory {
    /// The stage i32, None when never written.
    stage: Option<i32>,
    /// The crumb blob, None when never written.
    blob: Option<Vec<u8>>,
    /// Every write, in order: ("blob", text), ("i32", value) or ("commit", "").
    writes: Vec<(&'static str, String)>,
    /// Writes left before the store refuses (None: never refuses), to cut a save part way.
    writes_left: Option<usize>,
}

impl Memory {
    /// A store holding this stage and blob.
    fn holding(stage: Option<i32>, blob: Option<&[u8]>) -> Memory {
        Memory {
            stage,
            blob: blob.map(<[u8]>::to_vec),
            ..Memory::default()
        }
    }

    /// Count one write against the budget: Err once it is spent.
    fn spend(&mut self) -> Result<(), NvsError> {
        match self.writes_left {
            Some(0) => Err(REFUSED),
            Some(left) => {
                self.writes_left = Some(left - 1);
                Ok(())
            }
            None => Ok(()),
        }
    }
}

impl Nvs for Memory {
    fn get_i32(&mut self, key: &str) -> Result<i32, NvsError> {
        assert_eq!(key, STAGE_KEY);
        self.stage.ok_or(NOT_FOUND)
    }

    fn get_blob(&mut self, key: &str, buffer: &mut [u8]) -> Result<usize, NvsError> {
        assert_eq!(key, CRUMB_KEY);
        let blob = self.blob.as_ref().ok_or(NOT_FOUND)?;
        let target = buffer.get_mut(..blob.len()).ok_or(NvsError(0x110c))?;
        target.copy_from_slice(blob);
        Ok(blob.len())
    }

    fn set_blob(&mut self, key: &str, value: &[u8]) -> Result<(), NvsError> {
        assert_eq!(key, CRUMB_KEY);
        self.spend()?;
        self.blob = Some(value.to_vec());
        self.writes.push(("blob", String::from_utf8(value.to_vec()).unwrap()));
        Ok(())
    }

    fn set_i32(&mut self, key: &str, value: i32) -> Result<(), NvsError> {
        assert_eq!(key, STAGE_KEY);
        self.spend()?;
        self.stage = Some(value);
        self.writes.push(("i32", value.to_string()));
        Ok(())
    }

    fn commit(&mut self) -> Result<(), NvsError> {
        self.spend()?;
        self.writes.push(("commit", String::new()));
        Ok(())
    }
}

/// A position at these decimal degrees.
fn at(latitude: f64, longitude: f64) -> Position {
    Position::from_degrees(latitude, longitude)
}

/// test_warmstart's crumb: an armed GLIDING flight at HPRC, stamped at 1000 s.
fn glide() -> Crumb {
    Crumb {
        stage: Some(3),
        stamp_s: Some(1000),
        armed: true,
        altitude_m: Some(412.5),
        speed_ms: None,
        airspeed_ms: Some(16.5),
        ticks_ms: Some(73215),
        identity: Identity {
            launch: Some(at(25.5, -80.4)),
            zone: Some([at(25.514, -80.393), at(25.514, -80.391)]),
            pad_altitude_m: Some(100.0),
            pitot_zero_pa: Some(-1.75),
            grounds_m: [None; GROUND_SLOTS],
            gyro_bias: [None; BIAS_SLOTS],
            warm_starts: 0,
        },
    }
}

/// The crumb with this stage id and stamp, the rest of [`glide`].
fn staged(stage: i32, stamp_s: i64) -> Crumb {
    Crumb {
        stage: Some(stage),
        stamp_s: Some(stamp_s),
        ..glide()
    }
}

/// The sensors of a v1.x board config: every baro driver, a pitot and an IMU, as (name, driver).
const SENSORS: [(&str, &str); 5] = [
    ("imu_lsm6dso32", "lsm6dso32"),
    ("baro_icp10111", "icp10111"),
    ("baro_bmp581", "bmp581"),
    ("baro_bmp280", "bmp280"),
    ("airspeed_sdp810", "sdp810"),
];

#[test]
fn a_crumb_round_trips_through_its_blob() {
    let crumb = glide();
    let text = crumb.to_json();
    let value: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(Crumb::from_json(&value), Some(crumb));
    // plan B's checkpoint order (identity, then the live state), nulls written, f32 at their shortest
    assert_eq!(
        text,
        r#"{"launch":[25.5,-80.4],"zone":[[25.514,-80.393],[25.514,-80.391]],"pad_altitude":100.0,"pitot_zero":-1.75,"stage":3,"armed":true,"altitude":412.5,"speed":null,"airspeed":16.5,"ticks_ms":73215,"stamp":1000}"#
    );
}

#[test]
fn the_largest_crumb_fits_the_read_buffer() {
    let extreme = Position {
        latitude_e7: -899_999_999,
        longitude_e7: -1_799_999_999,
    };
    let crumb = Crumb {
        // the writer's stage is a Stage id (Checkpoint::crumb), a u8
        stage: Some(i32::from(u8::MAX)),
        stamp_s: Some(i64::MIN),
        armed: true,
        altitude_m: Some(-f32::MIN_POSITIVE),
        speed_ms: Some(f32::MIN),
        airspeed_ms: Some(-f32::MIN_POSITIVE),
        ticks_ms: Some(u64::MAX),
        identity: Identity {
            launch: Some(extreme),
            zone: Some([extreme, extreme]),
            pad_altitude_m: Some(-f32::MIN_POSITIVE),
            pitot_zero_pa: Some(-f32::MIN_POSITIVE),
            grounds_m: [Some(-f32::MIN_POSITIVE); GROUND_SLOTS],
            // the longest a freeze writes: rounded to 0.001, under the filter's 1.5 deg/s at rest
            gyro_bias: [Some([-1.499; 3]); BIAS_SLOTS],
            warm_starts: u32::MAX,
        },
    };
    let length = crumb.to_json().len();
    assert!(length < BLOB_MAX, "{length} bytes");
}

#[test]
fn the_writer_puts_no_nan_in_the_blob() {
    let crumb = Crumb {
        altitude_m: Some(f32::NAN),
        airspeed_ms: Some(f32::INFINITY),
        ..glide()
    };
    let value: Value = serde_json::from_str(&crumb.to_json()).unwrap();
    assert_eq!(value["altitude"], Value::Null);
    assert_eq!(value["airspeed"], Value::Null);
    assert_eq!(Crumb::from_json(&value).unwrap().altitude_m, None);
}

#[test]
fn a_crumb_reads_only_well_typed_members() {
    assert_eq!(Crumb::from_json(&json!([3, 1000])), None);
    assert_eq!(Crumb::from_json(&json!(null)), None);
    assert_eq!(Crumb::from_json(&json!("crumb")), None);
    assert_eq!(Crumb::from_json(&json!({})), Some(Crumb::default()));
    let odd = json!({"stage": 3.0, "stamp": "1000", "armed": 1, "altitude": "412", "ticks_ms": -5,
        "launch": [95.0, 10.0], "zone": [[25.5, -80.4]], "pad_altitude": true, "pitot_zero": [0.0],
        "airspeed": 1e300, "unknown": {"x": 1}});
    assert_eq!(Crumb::from_json(&odd), Some(Crumb::default()));
    let integers = json!({"stage": 2, "stamp": 7, "armed": true, "pad_altitude": 100, "launch": [48, 11]});
    let crumb = Crumb::from_json(&integers).unwrap();
    assert_eq!((crumb.stage, crumb.stamp_s, crumb.armed), (Some(2), Some(7), true));
    assert_eq!(crumb.identity.pad_altitude_m, Some(100.0));
    assert_eq!(crumb.identity.launch, Some(at(48.0, 11.0)));
    assert_eq!(
        Crumb::from_json(&json!({"stage": 4_000_000_000_i64})).unwrap().stage,
        None
    );
}

#[test]
fn save_writes_the_blob_then_the_stage_then_commits() {
    let mut nvs = Memory::default();
    assert!(save(&mut nvs, &glide()));
    let kinds: Vec<&str> = nvs.writes.iter().map(|(kind, _value)| *kind).collect();
    assert_eq!(kinds, ["blob", "i32", "commit"]);
    assert_eq!(nvs.writes[1].1, "3");
    assert_eq!(load(&mut nvs), Some(glide()));
}

#[test]
fn save_refuses_a_crumb_without_a_stage_and_reports_a_refused_write() {
    let mut nvs = Memory::default();
    let stageless = Crumb { stage: None, ..glide() };
    assert!(!save(&mut nvs, &stageless));
    assert!(nvs.writes.is_empty(), "nothing written for a stageless crumb");
    // the blob goes in, the stage write is refused: the old stage (none) stays the flag
    let mut cut = Memory {
        writes_left: Some(1),
        ..Memory::default()
    };
    assert!(!save(&mut cut, &glide()));
    assert_eq!(cut.stage, None);
    assert_eq!(load(&mut cut), None, "a blob without its stage is no crumb");
}

#[test]
fn clear_zeroes_the_stage_and_reports_a_refusal() {
    let blob = glide().to_json();
    let mut nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    assert!(clear(&mut nvs));
    assert_eq!(nvs.stage, Some(NO_STAGE));
    assert_eq!(nvs.writes, [("i32", "0".to_owned()), ("commit", String::new())]);
    assert_eq!(load(&mut nvs), None);
    let mut refusing = Memory {
        writes_left: Some(0),
        ..Memory::holding(Some(3), Some(blob.as_bytes()))
    };
    assert!(!clear(&mut refusing));
    assert_eq!(refusing.stage, Some(3));
    // the stage lands but the commit is refused: not committed, though the i32 changed
    let mut uncommitted = Memory {
        writes_left: Some(1),
        ..Memory::holding(Some(3), Some(blob.as_bytes()))
    };
    assert!(!clear(&mut uncommitted));
}

#[test]
fn load_gates_on_the_stage_and_reads_the_blob() {
    let blob = glide().to_json();
    assert_eq!(load(&mut Memory::holding(None, Some(blob.as_bytes()))), None);
    assert_eq!(load(&mut Memory::holding(Some(NO_STAGE), Some(blob.as_bytes()))), None);
    assert_eq!(load(&mut Memory::holding(Some(3), None)), None);
    // the i32 wins over the blob's own stage (a torn write can lag it)
    let loaded = load(&mut Memory::holding(Some(4), Some(blob.as_bytes()))).unwrap();
    assert_eq!(loaded.stage, Some(4));
    // a blob that fills the buffer exactly reads; one byte more does not
    let mut padded = blob.clone().into_bytes();
    padded.resize(BLOB_MAX, b' ');
    assert!(load(&mut Memory::holding(Some(3), Some(&padded))).is_some());
    padded.push(b' ');
    assert_eq!(load(&mut Memory::holding(Some(3), Some(&padded))), None);
    let torn_blobs: [&[u8]; 6] = [
        &blob.as_bytes()[..40],
        b"",
        b"[1, 2]",
        b"null",
        b"{\"altitude\": NaN}",
        b"\xff{}",
    ];
    for torn in torn_blobs {
        assert_eq!(load(&mut Memory::holding(Some(3), Some(torn))), None, "{torn:?}");
    }
}

#[test]
fn reset_reasons_map_the_esp_idf_codes() {
    for (code, reason) in ResetReason::ALL.into_iter().enumerate() {
        assert_eq!(ResetReason::from_code(code as u32), reason);
        assert_eq!(reason as u32, code as u32);
        assert_eq!(ResetReason::from_name(reason.name()), Some(reason));
    }
    assert_eq!(ResetReason::from_code(16), ResetReason::Unknown);
    assert_eq!(ResetReason::from_code(u32::MAX), ResetReason::Unknown);
    assert_eq!(ResetReason::from_name("SOFT"), None);
    assert_eq!(ResetReason::from_name("sw"), None);
}

#[test]
fn the_gate_passes_an_interrupted_flight() {
    let (allowed, reason) = should_restore(Some(&glide()), true, 1030);
    assert!(allowed);
    assert_eq!(
        reason,
        Reason::Recover {
            stage: Stage::Gliding,
            age_s: 30
        }
    );
    assert_eq!(reason.to_string(), "recover gliding, 30s after checkpoint");
    // BOOSTING is the boost itself: the latch is still nested
    assert!(should_restore(Some(&staged(2, 1000)), false, 1030).0);
    assert!(should_restore(Some(&staged(4, 1000)), true, 1030).0);
}

#[test]
fn no_time_check_refuses_a_crumb() {
    // Q19's step 2: a power cycle restarts the clock, so the age says nothing -- behind the stamp, an hour past it,
    // or absurd, the crumb recovers and the age is only reported
    for (now_s, age_s) in [(999, -1), (1000 + 3600, 3600), (0, -1000)] {
        assert_eq!(
            should_restore(Some(&glide()), true, now_s),
            (
                true,
                Reason::Recover {
                    stage: Stage::Gliding,
                    age_s
                }
            )
        );
    }
    // the age saturates rather than wrapping on an absurd stamp
    let ancient = staged(3, i64::MIN);
    assert_eq!(
        should_restore(Some(&ancient), true, i64::MAX).1,
        Reason::Recover {
            stage: Stage::Gliding,
            age_s: i64::MAX
        }
    );
    // NEGATIVE: the separation latch still refuses a glide stage, whatever the age
    assert_eq!(
        should_restore(Some(&glide()), false, 1000 + 3600),
        (false, Reason::Nested)
    );
}

#[test]
fn the_gate_refuses_with_the_first_failing_reason() {
    let gate = |crumb: Option<&Crumb>, separated| should_restore(crumb, separated, 1030);
    assert_eq!(gate(None, true), (false, Reason::NoCrumb));
    let stageless = Crumb { stage: None, ..glide() };
    let unstamped = Crumb {
        stamp_s: None,
        ..glide()
    };
    assert_eq!(gate(Some(&stageless), true), (false, Reason::Torn));
    assert_eq!(gate(Some(&unstamped), true), (false, Reason::Torn));
    assert_eq!(gate(Some(&glide()), false), (false, Reason::Nested));
    assert_eq!(gate(Some(&staged(4, 1000)), false), (false, Reason::Nested));
    assert_eq!(
        gate(Some(&glide()), false).1.to_string(),
        "separation switch reads nested"
    );
}

#[test]
fn the_boot_rules_refuse_whatever_the_latch_or_the_age() {
    // DONE is a new session, SETTING a cold pad boot, whatever the latch or the age
    for now_s in [1030, 99_999] {
        for separated in [true, false] {
            let done = should_restore(Some(&staged(5, 1000)), separated, now_s);
            assert_eq!(done, (false, Reason::AfterDone));
            let setting = should_restore(Some(&staged(1, 1000)), separated, now_s);
            assert_eq!(setting, (false, Reason::OnThePad));
        }
    }
    for id in [0, 6, -1, i32::MAX] {
        let (allowed, reason) = should_restore(Some(&staged(id, 1000)), true, 1030);
        assert_eq!((allowed, reason), (false, Reason::NotAStage(id)));
    }
    assert_eq!(
        Reason::NotAStage(9).to_string(),
        "crumb stage 9 is no flight stage -> cold boot"
    );
    assert_eq!(
        Reason::AfterDone.to_string(),
        "crumb reads done -> a new session in setting"
    );
}

#[test]
fn the_apply_plan_restores_what_the_crumb_carries() {
    let plan = ApplyPlan::new(&glide(), SENSORS).unwrap();
    assert_eq!(
        plan.mission,
        Some((at(25.5, -80.4), [at(25.514, -80.393), at(25.514, -80.391)]))
    );
    let rebase = plan.rebase.as_ref().unwrap();
    assert_eq!(rebase.ground_m, 100.0);
    assert_eq!(rebase.baros, ["baro_icp10111", "baro_bmp581", "baro_bmp280"]);
    assert_eq!(plan.pitot_zero_pa, Some(-1.75));
    assert_eq!(plan.airspeed_ms, Some(16.5));
    assert_eq!(
        plan.identity,
        Identity {
            warm_starts: 1,
            ..glide().identity
        },
        "the crumb's identity, this warm start counted"
    );
    assert_eq!((plan.stage, plan.armed), (Stage::Gliding, true));
    let update = plan.mission_update().unwrap();
    assert_eq!(
        Value::Object(update),
        json!({"latitude": 25.5, "longitude": -80.4, "zone": [[25.514, -80.393], [25.514, -80.391]]})
    );
}

#[test]
fn the_apply_plan_leaves_alone_what_the_crumb_lacks() {
    let bare = Crumb {
        stage: Some(2),
        stamp_s: Some(1000),
        ..Crumb::default()
    };
    let plan = ApplyPlan::new(&bare, SENSORS).unwrap();
    assert_eq!(plan.mission, None);
    assert_eq!(plan.rebase, None);
    assert_eq!((plan.pitot_zero_pa, plan.airspeed_ms), (None, None));
    assert!(plan.identity.is_empty(), "counted, but nothing frozen");
    assert_eq!(plan.identity.warm_starts, 1);
    assert!(!plan.armed, "a disarmed crumb does not re-arm");
    assert_eq!(plan.mission_update(), None);
    // a launch without a zone (or a zone without a launch) is no mission update
    let launch_only = Crumb {
        identity: Identity {
            zone: None,
            ..glide().identity
        },
        ..glide()
    };
    assert_eq!(ApplyPlan::new(&launch_only, SENSORS).unwrap().mission, None);
    // a zero tare is a tare, and a rebase with no baro in the config names none
    let zeroed = Crumb {
        identity: Identity {
            pitot_zero_pa: Some(0.0),
            ..glide().identity
        },
        ..glide()
    };
    let plan = ApplyPlan::new(&zeroed, std::iter::empty()).unwrap();
    assert_eq!(plan.pitot_zero_pa, Some(0.0));
    assert!(plan.rebase.unwrap().baros.is_empty());
    assert_eq!(ApplyPlan::new(&staged(7, 1000), SENSORS), None);
    assert_eq!(ApplyPlan::new(&Crumb::default(), SENSORS), None);
}

#[test]
fn restore_boots_cold_without_a_crumb_and_writes_nothing() {
    let mut nvs = Memory::default();
    assert_eq!(restore(&mut nvs, true, true, 1030, SENSORS), Restore::Cold);
    assert!(nvs.writes.is_empty());
}

#[test]
fn restore_clears_the_stage_when_disabled_or_refused() {
    let blob = glide().to_json();
    let mut disabled = Memory::holding(Some(3), Some(blob.as_bytes()));
    assert_eq!(restore(&mut disabled, false, true, 1030, SENSORS), Restore::Disabled);
    assert_eq!(disabled.stage, Some(NO_STAGE));
    let mut refused = Memory::holding(Some(3), Some(blob.as_bytes()));
    assert_eq!(
        restore(&mut refused, true, false, 1030, SENSORS),
        Restore::Refused(Reason::Nested)
    );
    assert_eq!(refused.stage, Some(NO_STAGE));
    let mut done = Memory::holding(Some(5), Some(blob.as_bytes()));
    assert_eq!(
        restore(&mut done, true, true, 1030, SENSORS),
        Restore::Refused(Reason::AfterDone)
    );
}

#[test]
fn a_refused_clear_leaves_the_crumb_to_the_next_boot_s_gate() {
    // restore ignores a refused clear, as plan B did: a lasting refusal refuses the crumb again on the next boot
    let refusing = |crumb: Crumb| Memory {
        writes_left: Some(0),
        ..Memory::holding(crumb.stage, Some(crumb.to_json().as_bytes()))
    };
    let mut done = refusing(staged(5, 1000));
    for now_s in [1030, 1060] {
        assert_eq!(
            restore(&mut done, true, true, now_s, SENSORS),
            Restore::Refused(Reason::AfterDone)
        );
    }
    // a nested latch's refusal does not last: the crumb it failed to clear warm-starts once the latch reads separated
    let mut glided = refusing(glide());
    assert_eq!(
        restore(&mut glided, true, false, 1030, SENSORS),
        Restore::Refused(Reason::Nested)
    );
    assert_eq!(glided.stage, Some(3));
    assert!(restore(&mut glided, true, true, 1060, SENSORS).keeps_flight());
}

#[test]
fn only_a_warm_start_keeps_the_recorder_s_flight_flag() {
    let blob = glide().to_json();
    // a stage i32 over a blob that does not read: Cold, with nothing cleared, and still the flag goes down
    let mut torn = Memory::holding(Some(3), Some(&blob.as_bytes()[..40]));
    let decision = restore(&mut torn, true, true, 1030, SENSORS);
    assert_eq!(decision, Restore::Cold);
    assert!(!decision.keeps_flight());
    assert_eq!(torn.stage, Some(3), "the i32 stays for the next boot");
    let refusals = [
        (Memory::default(), true, true),
        (Memory::holding(Some(3), Some(blob.as_bytes())), false, true),
        (Memory::holding(Some(3), Some(blob.as_bytes())), true, false),
    ];
    for (mut nvs, warm_start, separated) in refusals {
        let decision = restore(&mut nvs, warm_start, separated, 1030, SENSORS);
        assert!(!decision.keeps_flight(), "{decision:?}");
    }
    let mut flying = Memory::holding(Some(3), Some(blob.as_bytes()));
    assert!(restore(&mut flying, true, true, 1030, SENSORS).keeps_flight());
}

#[test]
fn a_warm_start_after_a_software_reset_suppresses_the_stall_restart() {
    let blob = glide().to_json();
    let mut nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    let warm = restore(&mut nvs, true, true, 1030, SENSORS);
    assert!(warm.keeps_flight(), "a software reset in flight warm-starts");
    assert!(warm.stall_restart_suppressed(ResetReason::Software));
}

#[test]
fn every_other_boot_restarts_for_a_stall() {
    let blob = glide().to_json();
    // a warm start after any other reset: a panic warm boot after a stall restart lifts the suppression
    let mut nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    let warm = restore(&mut nvs, true, true, 1030, SENSORS);
    assert!(warm.keeps_flight());
    let others = ResetReason::ALL
        .into_iter()
        .filter(|reset| *reset != ResetReason::Software);
    for reset in others {
        assert!(!warm.stall_restart_suppressed(reset), "{reset:?}");
    }
    // a software reset that boots cold, disabled or refused is no stall restart's warm start
    let mut empty = Memory::default();
    let cold = restore(&mut empty, true, true, 1030, SENSORS);
    assert_eq!(cold, Restore::Cold);
    let mut toggled_off = Memory::holding(Some(3), Some(blob.as_bytes()));
    let disabled = restore(&mut toggled_off, false, true, 1030, SENSORS);
    assert_eq!(disabled, Restore::Disabled);
    let mut nested = Memory::holding(Some(3), Some(blob.as_bytes()));
    let refused = restore(&mut nested, true, false, 1030, SENSORS);
    assert_eq!(refused, Restore::Refused(Reason::Nested));
    for decision in [cold, disabled, refused] {
        assert!(
            !decision.stall_restart_suppressed(ResetReason::Software),
            "{decision:?}"
        );
    }
}

#[test]
fn restore_warm_starts_a_passed_gate_and_keeps_the_crumb() {
    let blob = glide().to_json();
    let mut nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    let Restore::Warm { reason, plan } = restore(&mut nvs, true, true, 1030, SENSORS) else {
        panic!("no warm start");
    };
    assert_eq!(reason.to_string(), "recover gliding, 30s after checkpoint");
    assert_eq!(*plan, ApplyPlan::new(&glide(), SENSORS).unwrap());
    assert!(
        nvs.writes.is_empty(),
        "a warm start leaves the crumb for a second reset"
    );
}

/// The identity inputs of an HPRC pad with a fix, a pitot tare and a latched altitude.
fn pad() -> Freeze {
    Freeze {
        zone: Some([at(25.514, -80.393), at(25.514, -80.391)]),
        launch_point: Some(at(25.5, -80.4)),
        fresh_altitude_m: Some(120.0),
        pitot_zero_pa: Some(-1.75),
        grounds_m: [Some(120.0), Some(125.3), None],
        gyro_bias: [Some([0.12345, -0.5678, 0.0004]), None, Some([1.0, -1.0, 0.0]), None],
    }
}

#[test]
fn the_schedule_polls_at_most_every_half_second_and_floors_the_period() {
    assert_eq!(Checkpoint::new(1.0, 0).poll_ms(), 500);
    assert_eq!(Checkpoint::new(0.2, 0).poll_ms(), 500, "checkpoint_s is floored at 1 s");
    assert_eq!(Checkpoint::new(f32::NAN, 0).poll_ms(), 500);
    let mut slow = Checkpoint::new(5.0, 0);
    assert_eq!(slow.poll_ms(), 500);
    let never = || -> Freeze { panic!("no freeze outside BOOSTING entry") };
    assert!(slow.step(Stage::Gliding, 0, None, never).checkpoint);
    assert!(!slow.step(Stage::Gliding, 4_999_999, None, never).checkpoint);
    assert!(slow.step(Stage::Gliding, 5_000_000, None, never).checkpoint);
}

#[test]
fn a_cadence_past_the_clock_saturates_instead_of_overflowing() {
    // nothing validates checkpoint_s: a huge or infinite one must not panic at bring-up, and never falls due
    let never = || -> Freeze { panic!("no freeze outside BOOSTING entry") };
    for checkpoint_s in [f32::MAX, f32::INFINITY, 1e20, 2e13] {
        let mut schedule = Checkpoint::new(checkpoint_s, 0);
        assert_eq!(schedule.poll_ms(), 500, "{checkpoint_s}");
        assert!(
            schedule.step(Stage::Gliding, 0, None, never).checkpoint,
            "the stage change"
        );
        let a_century_us = 100 * 365 * 86_400 * 1_000_000_u64;
        assert!(
            !schedule.step(Stage::Gliding, a_century_us, None, never).checkpoint,
            "{checkpoint_s}"
        );
        assert!(
            schedule.step(Stage::Landing, a_century_us, None, never).checkpoint,
            "a change still writes"
        );
    }
}

#[test]
fn the_schedule_checkpoints_on_change_and_periodically_only_aloft() {
    let mut schedule = Checkpoint::new(1.0, 0);
    let mut steps = Vec::new();
    for (stage, now_us) in [
        (Stage::Setting, 500_000),
        (Stage::Setting, 1_000_000),
        (Stage::Setting, 9_000_000),
        (Stage::Boosting, 9_500_000),
        (Stage::Boosting, 10_000_000),
        (Stage::Boosting, 10_500_000),
        (Stage::Gliding, 11_000_000),
        (Stage::Gliding, 11_999_999),
        (Stage::Gliding, 12_000_000),
        (Stage::Landing, 12_500_000),
        (Stage::Done, 13_000_000),
        (Stage::Done, 20_000_000),
    ] {
        steps.push(schedule.step(stage, now_us, Some(100.0), pad).checkpoint);
    }
    assert_eq!(
        steps,
        [
            true, false, false, true, false, true, true, false, true, true, true, false
        ]
    );
}

#[test]
fn boosting_freezes_the_identity_with_the_latched_pad() {
    let mut schedule = Checkpoint::new(1.0, 0);
    schedule.step(Stage::Setting, 0, Some(101.5), pad);
    schedule.step(Stage::Setting, 500_000, None, pad); // a stale reading keeps the last good pad
    let step = schedule.step(Stage::Boosting, 1_000_000, Some(180.0), pad);
    assert_eq!(
        step,
        Step {
            froze: true,
            checkpoint: true
        }
    );
    let identity = *schedule.identity();
    assert_eq!(
        identity.pad_altitude_m,
        Some(101.5),
        "the latched pad, not the live reading"
    );
    assert_eq!(identity.launch, Some(at(25.5, -80.4)));
    assert_eq!(identity.pitot_zero_pa, Some(-1.75));
    // the identity rides every later crumb, armed, the airspeed to 0.1
    let live = Live {
        altitude_m: Some(300.0),
        speed_ms: None,
        airspeed_ms: Some(16.25),
        ticks_ms: 1000,
    };
    let crumb = schedule.crumb(Stage::Gliding, live, 844_560_000);
    assert_eq!(
        (crumb.stage, crumb.stamp_s, crumb.armed),
        (Some(3), Some(844_560_000), true)
    );
    assert_eq!(crumb.airspeed_ms, Some(16.2), "round half to even, as plan B's round()");
    assert_eq!(crumb.identity, identity);
}

#[test]
fn a_freeze_without_a_fix_or_a_zone_keeps_what_it_can() {
    let mut no_fix = Checkpoint::new(1.0, 0);
    let centre_only = || Freeze {
        launch_point: None,
        ..pad()
    };
    no_fix.step(Stage::Boosting, 0, None, centre_only);
    let launch = no_fix.identity().launch.unwrap();
    assert_eq!(
        launch,
        at(25.514, -80.392),
        "the zone's centre stands in for the launch point"
    );
    assert_eq!(
        no_fix.identity().pad_altitude_m,
        Some(120.0),
        "no latched pad: the fresh reading"
    );
    let mut no_zone = Checkpoint::new(1.0, 0);
    let bare = || Freeze {
        zone: None,
        fresh_altitude_m: None,
        pitot_zero_pa: None,
        grounds_m: [None; GROUND_SLOTS],
        gyro_bias: [None; BIAS_SLOTS],
        ..pad()
    };
    assert!(no_zone.step(Stage::Boosting, 0, None, bare).froze);
    assert!(no_zone.identity().is_empty(), "no zone, no launch or zone at all");
}

#[test]
fn the_zone_centre_rounds_toward_zero() {
    // the corners' units sum to an odd negative number: the mean is -803921000.5, which rounds toward zero
    // (a floor would give -803921001), as the mission's own zone centre does
    let corner = |longitude_e7| Position {
        latitude_e7: 255_140_001,
        longitude_e7,
    };
    let mut schedule = Checkpoint::new(1.0, 0);
    let odd = || Freeze {
        zone: Some([corner(-803_930_001), corner(-803_912_000)]),
        launch_point: None,
        ..pad()
    };
    schedule.step(Stage::Boosting, 0, None, odd);
    let launch = schedule.identity().launch.unwrap();
    assert_eq!(launch.longitude_e7, -803_921_000);
    assert_eq!(launch.latitude_e7, 255_140_001, "an even sum is exact");
    // and a positive odd sum rounds down, toward zero too
    let mut positive = Checkpoint::new(1.0, 0);
    let east = || Freeze {
        zone: Some([corner(803_930_001), corner(803_912_000)]),
        launch_point: None,
        ..pad()
    };
    positive.step(Stage::Boosting, 0, None, east);
    assert_eq!(positive.identity().launch.unwrap().longitude_e7, 803_921_000);
}

#[test]
fn a_freeze_that_found_nothing_is_not_retried_later_in_the_boost() {
    // plan B freezes only on the poll that ENTERS boosting: an empty freeze (no mission, a stale altitude, no
    // pitot) stays empty, or a later poll would take a mid-boost altitude as the pad
    let mut schedule = Checkpoint::new(1.0, 0);
    let nothing = || Freeze::default();
    schedule.step(Stage::Setting, 0, None, nothing);
    let entry = schedule.step(Stage::Boosting, 500_000, None, nothing);
    assert_eq!(
        entry,
        Step {
            froze: true,
            checkpoint: true
        }
    );
    assert!(schedule.identity().is_empty());
    let again = || -> Freeze { panic!("no second freeze in the same boost") };
    for now_us in [1_000_000, 1_500_000, 2_000_000] {
        let step = schedule.step(Stage::Boosting, now_us, Some(250.0), again);
        assert!(!step.froze);
        assert!(schedule.identity().is_empty(), "the mid-boost 250 m is no pad");
    }
}

#[test]
fn a_seeded_identity_survives_boosting_and_the_ground_drops_it() {
    // test_warmstart._checkpoint_identity: a warm start into BOOSTING keeps the identity it launched with
    let mut schedule = Checkpoint::new(1.0, 0);
    schedule.seed(glide().identity);
    let step = schedule.step(Stage::Boosting, 0, Some(500.0), || -> Freeze {
        panic!("must not freeze over a seed")
    });
    assert_eq!(
        step,
        Step {
            froze: false,
            checkpoint: true
        }
    );
    assert_eq!(*schedule.identity(), glide().identity);
    // back on the ground the next BOOSTING is a new flight
    schedule.step(Stage::Setting, 500_000, None, pad);
    assert!(schedule.identity().is_empty());
}

#[test]
fn a_checkpointed_crumb_restores_on_a_warm_boot() {
    // the schedule's crumb, saved and loaded back, passes the gate into the stage it was written in
    let mut schedule = Checkpoint::new(1.0, 0);
    schedule.step(Stage::Setting, 0, Some(100.0), pad);
    schedule.step(Stage::Boosting, 500_000, None, pad);
    let live = Live {
        altitude_m: Some(250.0),
        speed_ms: Some(30.0),
        airspeed_ms: Some(25.0),
        ticks_ms: 900,
    };
    let mut nvs = Memory::default();
    assert!(save(&mut nvs, &schedule.crumb(Stage::Boosting, live, 5000)));
    let Restore::Warm { plan, .. } = restore(&mut nvs, true, false, 5001, SENSORS) else {
        panic!("no warm start");
    };
    assert_eq!(plan.stage, Stage::Boosting);
    assert_eq!(plan.rebase.unwrap().ground_m, 100.0);
}

/// The pad of the keeper tests: 100 m above sea level, where the barometer was zeroed.
const PAD_M: f32 = 100.0;

/// A pass that did nothing: no freeze, no row, no NVS action.
const NOTHING: Pass = Pass {
    froze: false,
    row: None,
    wrote: None,
    failed: None,
};

/// The stage machine's own advance into `stage`, with a reason it gives there; only the advance into BOOSTING (the
/// launch) sets `launched`, and none clears it.
fn advance(stage: Stage) -> Transition {
    let reason = match stage {
        Stage::Boosting => sequencer::Reason::LaunchAcceleration {
            magnitude_g: 3.2,
            dwell_ms: 100,
        },
        Stage::Gliding => sequencer::Reason::Apogee { peak_m: 300.0 },
        Stage::Landing => sequencer::Reason::HeightAboveGround { height_m: 20.0 },
        Stage::Setting | Stage::Done => sequencer::Reason::Stationary { magnitude_g: 1.0 },
    };
    Transition { to: stage, reason }
}

/// A keeper under test, with its NVS and a count of the stamps it read.
struct Board {
    /// The keeper.
    keeper: Keeper,
    /// Its NVS namespace.
    nvs: Memory,
    /// How many times a pass read its stamp.
    stamps: u32,
}

impl Board {
    /// A keeper at a 1 s cadence started at 0, under this pad ceiling, over an empty NVS.
    fn new(pad_ceiling_m: Option<f32>) -> Board {
        Board {
            keeper: Keeper::new(1.0, 0, pad_ceiling_m),
            nvs: Memory::default(),
            stamps: 0,
        }
    }

    /// One pass at `now_us` over `flight`, the board `elevation_m` above the pad: both readings fresh, the altitude
    /// the pad's plus the elevation, [`pad`]'s mission frozen with that altitude, the stamp the whole second.
    fn pass(&mut self, flight: Flight, now_us: u64, elevation_m: f32) -> Pass {
        let altitude_m = Some(PAD_M + elevation_m);
        let live = Live {
            altitude_m,
            speed_ms: None,
            airspeed_ms: None,
            ticks_ms: now_us / 1000,
        };
        let freeze = || Freeze {
            fresh_altitude_m: altitude_m,
            ..pad()
        };
        let stamps = &mut self.stamps;
        let stamp_s = || {
            *stamps += 1;
            i64::try_from(now_us / 1_000_000).unwrap()
        };
        self.keeper
            .pass(&mut self.nvs, flight, now_us, Some(elevation_m), live, freeze, stamp_s)
    }

    /// The crumb NVS holds now, None when the stage i32 says none. The blob's own stage must be the i32's: [`load`]
    /// reads the i32 over it, so only this check sees a blob written for another stage.
    fn crumb(&mut self) -> Option<Crumb> {
        let crumb = load(&mut self.nvs)?;
        let blob: Value = serde_json::from_slice(self.nvs.blob.as_ref()?).unwrap();
        assert_eq!(blob["stage"], json!(crumb.stage), "the blob's stage");
        Some(crumb)
    }

    /// How many clears NVS took: the stage i32 written as [`NO_STAGE`].
    fn clears(&self) -> usize {
        let cleared = ("i32", NO_STAGE.to_string());
        self.nvs.writes.iter().filter(|write| **write == cleared).count()
    }
}

/// The pass that saved the crumb at `stage`, its row due there.
fn saved(stage: Stage) -> Pass {
    Pass {
        row: Some(stage),
        wrote: Some(Write::Crumb),
        ..NOTHING
    }
}

/// The pass that only pushed a row at `stage`.
fn row(stage: Stage) -> Pass {
    Pass {
        row: Some(stage),
        ..NOTHING
    }
}

/// `pass`, on the pass that froze the identity.
fn frozen(pass: Pass) -> Pass {
    Pass { froze: true, ..pass }
}

/// The pass that only committed a clear.
fn cleared() -> Pass {
    Pass {
        wrote: Some(Write::Clear),
        ..NOTHING
    }
}

#[test]
fn the_keeper_polls_on_the_schedule_s_period() {
    assert_eq!(Keeper::new(1.0, 0, None).poll_ms(), 500);
    assert_eq!(Keeper::new(0.2, 0, Some(5.0)).poll_ms(), 500, "floored at 1 s");
    let fresh = Keeper::new(1.0, 0, None);
    assert_eq!((fresh.owed(), fresh.counters()), (None, Counters::default()));
}

#[test]
fn an_armed_launched_boost_saves_a_crumb_of_one_flight() {
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    // on the pad, disarmed then armed: the SETTING row, no crumb
    assert_eq!(board.pass(writer.flight(), 0, 0.0), row(Stage::Setting));
    writer.apply(Request::Arm).unwrap();
    assert_eq!(board.pass(writer.flight(), 500_000, 0.0), NOTHING);
    // the stage machine's launch: armed and launched in the one flight the pass reads
    writer.advance(advance(Stage::Boosting));
    let pass = board.pass(writer.flight(), 1_000_000, 12.0);
    assert_eq!(pass, frozen(saved(Stage::Boosting)));
    let crumb = board.crumb().unwrap();
    assert_eq!((crumb.stage, crumb.armed, crumb.stamp_s), (Some(2), true, Some(1)));
    assert_eq!(crumb.altitude_m, Some(112.0));
    assert_eq!(crumb.identity.pad_altitude_m, Some(PAD_M), "the pad latched in SETTING");
    assert_eq!(crumb.identity.launch, pad().launch_point);
    // flying with nothing due: no row, no write, and no stamp read
    assert_eq!(board.pass(writer.flight(), 1_500_000, 40.0), NOTHING);
    assert_eq!(board.stamps, 1);
    // the period aloft re-saves, re-stamped
    assert_eq!(board.pass(writer.flight(), 2_000_000, 80.0), saved(Stage::Boosting));
    assert_eq!(board.crumb().unwrap().stamp_s, Some(2));
    let counters = Counters {
        saves: 2,
        ..Counters::default()
    };
    assert_eq!(board.keeper.counters(), counters);
}

#[test]
fn an_armed_hold_saves_no_crumb_only_rows() {
    // a bench `arm` then `stage gliding`: armed, but no launch, so a reset must never resume it
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    board.pass(writer.flight(), 0, 0.0);
    writer.apply(Request::Hold(Stage::Gliding)).unwrap();
    assert!(writer.flight().armed && !writer.flight().launched);
    let mut passes = Vec::new();
    for now_us in [500_000, 1_000_000, 1_500_000] {
        passes.push(board.pass(writer.flight(), now_us, 0.0));
    }
    writer.apply(Request::Hold(Stage::Landing)).unwrap();
    passes.push(board.pass(writer.flight(), 2_000_000, 0.0));
    writer.apply(Request::Hold(Stage::Done)).unwrap();
    passes.push(board.pass(writer.flight(), 2_500_000, 0.0));
    assert_eq!(
        passes,
        [
            row(Stage::Gliding),
            NOTHING,
            row(Stage::Gliding),
            row(Stage::Landing),
            row(Stage::Done)
        ]
    );
    assert!(board.nvs.writes.is_empty(), "rows only");
    assert_eq!((board.stamps, board.keeper.owed()), (0, None));
}

#[test]
fn a_disarmed_walk_saves_no_crumb() {
    // the machine's own launch with the board disarmed: launched, but a passive flight never warm-starts armed
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    board.pass(writer.flight(), 0, 0.0);
    writer.advance(advance(Stage::Boosting));
    assert_eq!(board.pass(writer.flight(), 500_000, 12.0), frozen(row(Stage::Boosting)));
    writer.separate().unwrap();
    assert_eq!(board.pass(writer.flight(), 1_000_000, 200.0), row(Stage::Gliding));
    writer.advance(advance(Stage::Landing));
    assert_eq!(board.pass(writer.flight(), 1_500_000, 20.0), row(Stage::Landing));
    writer.advance(advance(Stage::Done));
    assert_eq!(board.pass(writer.flight(), 2_000_000, 0.0), row(Stage::Done));
    assert!(writer.flight().launched && !writer.flight().armed);
    assert!(board.nvs.writes.is_empty());
    assert_eq!(board.stamps, 0);
}

#[test]
fn a_disarm_clears_the_crumb_exactly_once() {
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    assert_eq!(board.pass(writer.flight(), 0, 12.0).wrote, Some(Write::Crumb));
    writer.separate().unwrap();
    assert_eq!(board.pass(writer.flight(), 500_000, 200.0), saved(Stage::Gliding));
    writer.apply(Request::Disarm).unwrap();
    assert_eq!(board.pass(writer.flight(), 700_000, 190.0), cleared());
    assert_eq!(board.crumb(), None);
    // the glide goes on disarmed: rows, and no second clear
    assert_eq!(board.pass(writer.flight(), 1_500_000, 150.0), row(Stage::Gliding));
    writer.advance(advance(Stage::Landing));
    assert_eq!(board.pass(writer.flight(), 2_000_000, 20.0), row(Stage::Landing));
    assert_eq!(board.clears(), 1);
    assert_eq!(board.stamps, 2);
}

#[test]
fn a_hold_out_of_a_launched_flight_clears() {
    // an operator hold is never the machine's flight: it clears `launched`, still armed, and the crumb goes
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    writer.separate().unwrap();
    assert_eq!(board.pass(writer.flight(), 0, 200.0).wrote, Some(Write::Crumb));
    writer.apply(Request::Hold(Stage::Gliding)).unwrap();
    assert!(writer.flight().armed);
    assert_eq!(board.pass(writer.flight(), 500_000, 190.0).wrote, Some(Write::Clear));
    assert_eq!(board.crumb(), None);
    assert_eq!(board.pass(writer.flight(), 1_000_000, 180.0), row(Stage::Gliding));
    assert_eq!(board.clears(), 1);
}

#[test]
fn re_arming_and_launching_saves_again() {
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    board.pass(writer.flight(), 0, 12.0);
    // the false-launch recovery: disarm, back to the ground, re-arm
    writer.apply(Request::Disarm).unwrap();
    assert_eq!(board.pass(writer.flight(), 500_000, 12.0).wrote, Some(Write::Clear));
    writer.apply(Request::Hold(Stage::Setting)).unwrap();
    assert_eq!(board.pass(writer.flight(), 1_000_000, 1.0), row(Stage::Setting));
    writer.apply(Request::Arm).unwrap();
    assert_eq!(board.pass(writer.flight(), 1_500_000, 0.5), NOTHING);
    writer.advance(advance(Stage::Boosting));
    assert_eq!(
        board.pass(writer.flight(), 2_000_000, 15.0),
        frozen(saved(Stage::Boosting))
    );
    assert_eq!(
        board.crumb().unwrap().identity.pad_altitude_m,
        Some(PAD_M + 0.5),
        "the new pad"
    );
    writer.separate().unwrap();
    assert_eq!(board.pass(writer.flight(), 2_500_000, 200.0), saved(Stage::Gliding));
    assert_eq!(board.crumb().unwrap().stage, Some(3));
}

#[test]
fn a_seeded_flight_keeps_saving_until_a_disarm_clears_it() {
    // the warm start restored an armed glide: its crumb is in NVS, and the keeper carries its identity on
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    let blob = glide().to_json();
    board.nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    board.keeper.seed(glide().identity);
    writer.restore(Stage::Gliding, true).unwrap();
    assert_eq!(board.pass(writer.flight(), 0, 300.0), saved(Stage::Gliding));
    assert_eq!(
        board.crumb().unwrap().identity,
        glide().identity,
        "the seed, not a new freeze"
    );
    assert_eq!(board.pass(writer.flight(), 1_000_000, 290.0), saved(Stage::Gliding));
    writer.apply(Request::Disarm).unwrap();
    assert_eq!(board.pass(writer.flight(), 1_500_000, 280.0).wrote, Some(Write::Clear));
    assert_eq!(board.crumb(), None);
}

#[test]
fn a_seed_alone_makes_the_crumb_live() {
    // seeded, then disarmed before the first pass: the restored crumb is cleared, never re-saved
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut seeded = Board::new(None);
    let blob = glide().to_json();
    seeded.nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    seeded.keeper.seed(glide().identity);
    writer.restore(Stage::Gliding, true).unwrap();
    writer.apply(Request::Disarm).unwrap();
    let cleared_with_a_row = Pass {
        row: Some(Stage::Gliding),
        ..cleared()
    };
    assert_eq!(seeded.pass(writer.flight(), 0, 300.0), cleared_with_a_row);
    assert_eq!(seeded.crumb(), None);
    assert_eq!(seeded.stamps, 0);
    // unseeded, the keeper knows of no crumb and leaves NVS alone
    let mut unseeded = Board::new(None);
    unseeded.nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    assert_eq!(unseeded.pass(writer.flight(), 0, 300.0), row(Stage::Gliding));
    assert!(unseeded.nvs.writes.is_empty());
}

#[test]
fn a_refused_done_crumb_lands_on_a_later_pass() {
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    board.pass(writer.flight(), 0, 12.0);
    writer.separate().unwrap();
    board.pass(writer.flight(), 500_000, 200.0);
    writer.advance(advance(Stage::Landing));
    assert_eq!(board.pass(writer.flight(), 1_000_000, 20.0), saved(Stage::Landing));
    // NVS refuses through the landing
    board.nvs.writes_left = Some(0);
    writer.advance(advance(Stage::Done));
    let refused = Pass {
        row: Some(Stage::Done),
        failed: Some(REFUSED),
        ..NOTHING
    };
    assert_eq!(board.pass(writer.flight(), 1_500_000, 0.0), refused);
    assert_eq!(board.keeper.owed(), Some(Write::Crumb));
    assert_eq!(board.nvs.stage, Some(4), "the landing crumb is still the flag");
    // DONE never re-checkpoints: only the owed crumb tries again
    let again = Pass {
        failed: Some(REFUSED),
        ..NOTHING
    };
    assert_eq!(board.pass(writer.flight(), 2_000_000, 0.0), again);
    board.nvs.writes_left = None;
    let landed = Pass {
        wrote: Some(Write::Crumb),
        ..NOTHING
    };
    assert_eq!(board.pass(writer.flight(), 3_200_000, 0.0), landed);
    let crumb = board.crumb().unwrap();
    assert_eq!(crumb.stage, Some(5));
    assert_eq!(
        (crumb.stamp_s, crumb.ticks_ms),
        (Some(3), Some(3200)),
        "rebuilt from the pass that landed it"
    );
    assert_eq!(board.keeper.owed(), None);
    assert_eq!(board.pass(writer.flight(), 4_000_000, 0.0), NOTHING);
    let counters = Counters {
        saves: 4,
        failures: 2,
        retries: 2,
    };
    assert_eq!(board.keeper.counters(), counters);
    assert_eq!(board.stamps, 6, "one per attempt, refused ones included");
}

#[test]
fn a_refused_clear_is_retried_until_it_lands() {
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    board.pass(writer.flight(), 0, 12.0);
    board.nvs.writes_left = Some(0);
    writer.apply(Request::Disarm).unwrap();
    for now_us in [500_000, 700_000] {
        let pass = board.pass(writer.flight(), now_us, 30.0);
        assert_eq!((pass.wrote, pass.failed), (None, Some(REFUSED)));
        assert_eq!(board.keeper.owed(), Some(Write::Clear));
        assert_eq!(
            board.nvs.stage,
            Some(2),
            "the crumb stays the flag until the clear lands"
        );
    }
    board.nvs.writes_left = None;
    assert_eq!(board.pass(writer.flight(), 900_000, 40.0).wrote, Some(Write::Clear));
    assert_eq!(board.crumb(), None);
    assert_eq!(board.pass(writer.flight(), 1_100_000, 50.0), row(Stage::Boosting));
    assert_eq!(board.clears(), 1);
    let counters = Counters {
        saves: 1,
        failures: 2,
        retries: 2,
    };
    assert_eq!(board.keeper.counters(), counters);
    assert_eq!(board.stamps, 1, "a clear reads no stamp");
}

#[test]
fn a_clear_replaces_a_refused_crumb() {
    // the flight's first crumb was refused, then it ended: a clear, never the crumb of a flight that is over
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    board.nvs.writes_left = Some(0);
    assert_eq!(board.pass(writer.flight(), 0, 12.0).failed, Some(REFUSED));
    writer.apply(Request::Disarm).unwrap();
    board.nvs.writes_left = None;
    assert_eq!(board.pass(writer.flight(), 500_000, 30.0).wrote, Some(Write::Clear));
    assert_eq!(board.crumb(), None);
    assert_eq!(board.nvs.writes.len(), 2, "the clear's i32 and commit, no crumb");
    let counters = Counters {
        saves: 0,
        failures: 1,
        retries: 0,
    };
    assert_eq!(board.keeper.counters(), counters);
}

#[test]
fn a_crumb_replaces_an_owed_clear_once_flying_again() {
    // a refused clear, then back to the ground, re-armed and launched between two polls: the pass sees the same
    // BOOSTING with nothing due, and the new flight's crumb, never the old one, becomes the flag
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    board.pass(writer.flight(), 0, 12.0);
    board.nvs.writes_left = Some(0);
    writer.apply(Request::Disarm).unwrap();
    assert_eq!(board.pass(writer.flight(), 500_000, 12.0).failed, Some(REFUSED));
    assert_eq!(board.keeper.owed(), Some(Write::Clear));
    board.nvs.writes_left = None;
    writer.apply(Request::Hold(Stage::Setting)).unwrap();
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    let crumb_only = Pass {
        wrote: Some(Write::Crumb),
        ..NOTHING
    };
    assert_eq!(board.pass(writer.flight(), 700_000, 15.0), crumb_only);
    assert_eq!(board.crumb().unwrap().ticks_ms, Some(700), "this pass's crumb");
    assert_eq!(board.clears(), 0);
    let counters = Counters {
        saves: 2,
        failures: 1,
        retries: 0,
    };
    assert_eq!(
        board.keeper.counters(),
        counters,
        "a crumb replacing a clear is no retry"
    );
}

#[test]
fn a_retry_on_a_due_checkpoint_still_counts() {
    // the separation's crumb is refused, and its next attempt falls on the period, where a crumb is due anyway
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(None);
    writer.apply(Request::Arm).unwrap();
    writer.advance(advance(Stage::Boosting));
    board.pass(writer.flight(), 0, 12.0);
    board.nvs.writes_left = Some(0);
    writer.separate().unwrap();
    let refused = Pass {
        row: Some(Stage::Gliding),
        failed: Some(REFUSED),
        ..NOTHING
    };
    assert_eq!(board.pass(writer.flight(), 500_000, 200.0), refused);
    board.nvs.writes_left = None;
    assert_eq!(board.pass(writer.flight(), 1_500_000, 190.0), saved(Stage::Gliding));
    assert_eq!(board.crumb().unwrap().stage, Some(3));
    let counters = Counters {
        saves: 2,
        failures: 1,
        retries: 1,
    };
    assert_eq!(board.keeper.counters(), counters);
}

/// The pad a baro-detected launch freezes, walking SETTING up through the dwell under this ceiling.
fn pad_after_a_baro_launch_dwell(pad_ceiling_m: Option<f32>) -> Option<f32> {
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut board = Board::new(pad_ceiling_m);
    writer.apply(Request::Arm).unwrap();
    // the accel trigger missed: SETTING lasts while the elevation climbs through the baro launch dwell
    for (index, elevation_m) in [0.0, 3.0, 4.5, 5.0, 8.0, 12.0].into_iter().enumerate() {
        board.pass(writer.flight(), index as u64 * 100_000, elevation_m);
    }
    writer.advance(advance(Stage::Boosting));
    assert!(board.pass(writer.flight(), 600_000, 14.0).froze);
    board.crumb().unwrap().identity.pad_altitude_m
}

#[test]
fn the_pad_ceiling_stops_the_latch_on_a_baro_launch_dwell() {
    assert_eq!(
        pad_after_a_baro_launch_dwell(Some(5.0)),
        Some(PAD_M + 4.5),
        "the last reading under the ceiling; 5 m is on it"
    );
    assert_eq!(
        pad_after_a_baro_launch_dwell(None),
        Some(PAD_M + 12.0),
        "plan B: the dwell's top"
    );
}

/// The pad a launch freezes under this ceiling after a SETTING pass whose altitude is fresh at the pad but whose
/// elevation is stale.
fn pad_after_a_stale_elevation(pad_ceiling_m: Option<f32>) -> Option<f32> {
    let word = FlightWord::new();
    let mut writer = word.writer().unwrap();
    let mut nvs = Memory::default();
    let mut keeper = Keeper::new(1.0, 0, pad_ceiling_m);
    let live = Live {
        altitude_m: Some(PAD_M),
        ..Live::default()
    };
    let never = || -> i64 { panic!("no stamp in SETTING") };
    writer.apply(Request::Arm).unwrap();
    assert_eq!(
        keeper.pass(&mut nvs, writer.flight(), 0, None, live, pad, never),
        row(Stage::Setting)
    );
    writer.advance(advance(Stage::Boosting));
    let pass = keeper.pass(&mut nvs, writer.flight(), 500_000, None, live, pad, || 7);
    assert_eq!(pass, frozen(saved(Stage::Boosting)));
    load(&mut nvs).unwrap().identity.pad_altitude_m
}

#[test]
fn a_stale_elevation_latches_a_pad_only_without_a_ceiling() {
    // with a ceiling the latch cannot be checked, so the freeze falls back to its own reading
    assert_eq!(pad_after_a_stale_elevation(Some(5.0)), pad().fresh_altitude_m);
    // without one the elevation is never asked, and the fresh altitude latches as plan B's did
    assert_eq!(pad_after_a_stale_elevation(None), Some(PAD_M));
}

#[test]
fn each_barometer_s_own_ground_and_the_warm_count_round_trip_and_stay_out_of_a_plan_b_crumb() {
    // a plan B crumb (no ground of its own, no warm start) writes exactly plan B's keys
    let plain = glide().to_json();
    assert!(!plain.contains("grounds") && !plain.contains("warm_starts"), "{plain}");
    let mut crumb = glide();
    crumb.identity.grounds_m = [Some(120.0), None, Some(125.3)];
    crumb.identity.warm_starts = 2;
    let text = crumb.to_json();
    assert!(
        text.contains(r#""grounds":[120.0,null,125.3]"#) && text.contains(r#""warm_starts":2"#),
        "{text}"
    );
    assert_eq!(
        Crumb::from_json(&serde_json::from_str(&text).unwrap()),
        Some(crumb.clone())
    );
    // the list stops at the last known ground
    crumb.identity.grounds_m = [Some(120.0), None, None];
    assert!(crumb.to_json().contains(r#""grounds":[120.0]"#), "{}", crumb.to_json());
    // NEGATIVE: a list of the wrong shape or past the slots reads what it can, a non-number as absent
    let odd = serde_json::json!({"stage": 3, "stamp": 1, "grounds": [1.5, "x", 2.5, 9.0], "warm_starts": -1});
    let read = Crumb::from_json(&odd).unwrap();
    assert_eq!(read.identity.grounds_m, [Some(1.5), None, Some(2.5)]);
    assert_eq!(read.identity.warm_starts, 0, "a negative count reads as none");
    let scalar = serde_json::json!({"stage": 3, "stamp": 1, "grounds": 7.0});
    assert_eq!(
        Crumb::from_json(&scalar).unwrap().identity.grounds_m,
        [None; GROUND_SLOTS]
    );
}

#[test]
fn a_warm_start_rebases_each_barometer_to_its_own_ground_else_the_pad() {
    let mut crumb = glide();
    crumb.identity.grounds_m = [Some(100.2), Some(105.5), None];
    let sensors = [
        ("baro_icp10111", "icp10111"),
        ("imu", "lsm6dso32"),
        ("baro_bmp581", "bmp581"),
        ("baro_bmp280", "bmp280"),
    ];
    let plan = ApplyPlan::new(&crumb, sensors).unwrap();
    let rebase = plan.rebase.unwrap();
    assert_eq!(rebase.baros, ["baro_icp10111", "baro_bmp581", "baro_bmp280"]);
    assert_eq!(
        (0..3).map(|index| rebase.ground_of(index)).collect::<Vec<_>>(),
        [100.2, 105.5, 100.0],
        "its own ground; the pad altitude for the one with none"
    );
    // a plan B crumb: every barometer to the pad, as plan B rebased them
    let plan_b = ApplyPlan::new(&glide(), sensors).unwrap().rebase.unwrap();
    assert!((0..3).all(|index| plan_b.ground_of(index) == 100.0));
}

#[test]
fn the_freeze_keeps_each_barometer_s_ground() {
    let mut schedule = Checkpoint::new(1.0, 0);
    schedule.step(Stage::Setting, 0, Some(120.0), pad);
    assert!(schedule.step(Stage::Boosting, 1, Some(150.0), pad).froze);
    let crumb = schedule.crumb(Stage::Boosting, Live::default(), 1000);
    assert_eq!(crumb.identity.grounds_m, [Some(120.0), Some(125.3), None]);
}

#[test]
fn the_gyro_biases_freeze_rounded_round_trip_and_reach_the_plan() {
    let mut schedule = Checkpoint::new(1.0, 0);
    assert!(schedule.step(Stage::Boosting, 0, Some(150.0), pad).froze);
    let frozen = [Some([0.123, -0.568, 0.0]), None, Some([1.0, -1.0, 0.0]), None];
    assert_eq!(schedule.identity().gyro_bias, frozen, "rounded to 0.001 deg/s");
    let crumb = schedule.crumb(Stage::Gliding, Live::default(), 1000);
    let text = crumb.to_json();
    assert!(
        text.contains(r#""gyro_bias":[[0.123,-0.568,0.0],null,[1.0,-1.0,0.0]]"#),
        "up to the last slot known: {text}"
    );
    let read = Crumb::from_json(&serde_json::from_str(&text).unwrap()).unwrap();
    assert_eq!(read, crumb);
    assert_eq!(ApplyPlan::new(&read, SENSORS).unwrap().gyro_bias, frozen);
    // a plan B crumb carries none, and its plan seeds none
    assert!(!glide().to_json().contains("gyro_bias"));
    assert_eq!(ApplyPlan::new(&glide(), SENSORS).unwrap().gyro_bias, [None; BIAS_SLOTS]);
    // NEGATIVE: a slot of the wrong shape reads as absent, a list past the slots reads what fits, a non-list nothing
    let odd = json!({"stage": 3, "stamp": 1,
                     "gyro_bias": [[0.1, 0.2], "x", [0.1, "y", 0.3], [0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]});
    assert_eq!(
        Crumb::from_json(&odd).unwrap().identity.gyro_bias,
        [None, None, None, Some([0.1, 0.2, 0.3])]
    );
    let scalar = json!({"stage": 3, "stamp": 1, "gyro_bias": 0.1});
    assert_eq!(
        Crumb::from_json(&scalar).unwrap().identity.gyro_bias,
        [None; BIAS_SLOTS]
    );
}

#[test]
fn past_the_cap_a_warm_start_recovers_disarmed_and_counts_on() {
    for (warm_starts, armed) in [
        (0, true),
        (WARM_CAP - 1, true),
        (WARM_CAP, false),
        (WARM_CAP + 5, false),
    ] {
        let mut crumb = glide();
        crumb.identity.warm_starts = warm_starts;
        let plan = ApplyPlan::new(&crumb, SENSORS).unwrap();
        assert_eq!(plan.armed, armed, "{warm_starts} warm starts before");
        assert_eq!(plan.capped, !armed, "{warm_starts}");
        assert_eq!(plan.identity.warm_starts, warm_starts + 1, "this one counts");
        assert_eq!(plan.stage, Stage::Gliding, "the stage machine still walks it to DONE");
    }
    // NEGATIVE: a disarmed crumb is never capped (nothing to take away)
    let mut disarmed = glide();
    disarmed.armed = false;
    disarmed.identity.warm_starts = WARM_CAP + 1;
    let plan = ApplyPlan::new(&disarmed, SENSORS).unwrap();
    assert!(!plan.armed && !plan.capped);
    // the count saturates
    let mut many = glide();
    many.identity.warm_starts = u32::MAX;
    assert_eq!(ApplyPlan::new(&many, SENSORS).unwrap().identity.warm_starts, u32::MAX);
}
