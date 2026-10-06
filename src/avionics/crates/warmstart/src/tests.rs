//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the warm start, positive and negative: the crumb serde, the NVS order, the gate with its boot rules
//! and reset classes, the apply plan, the boot decision and the checkpoint schedule. The plan B comparison itself is
//! tests/golden.rs.

use super::{
    ApplyPlan, BLOB_MAX, CRUMB_KEY, Checkpoint, Crumb, Freeze, Identity, Live, MAX_AGE_S, NO_STAGE, Nvs, NvsError,
    Reason, ResetReason, Restore, STAGE_KEY, Step, clear, load, restore, save, should_restore,
};
use control::navigation::Position;
use sequencer::Stage;
use serde_json::{Value, json};

/// ESP_ERR_NVS_NOT_FOUND, what esp-idf answers for a key never written.
const NOT_FOUND: NvsError = NvsError(0x1102);

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
            Some(0) => Err(NvsError(0x1105)),
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
        stage: Some(i32::MIN),
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
fn clear_zeroes_the_stage_and_swallows_a_failure() {
    let blob = glide().to_json();
    let mut nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    clear(&mut nvs);
    assert_eq!(nvs.stage, Some(NO_STAGE));
    assert_eq!(nvs.writes, [("i32", "0".to_owned()), ("commit", String::new())]);
    assert_eq!(load(&mut nvs), None);
    let mut refusing = Memory {
        writes_left: Some(0),
        ..Memory::holding(Some(3), Some(blob.as_bytes()))
    };
    clear(&mut refusing);
    assert_eq!(refusing.stage, Some(3));
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
fn only_the_firmware_s_own_resets_count() {
    let resets: Vec<&str> = ResetReason::ALL
        .into_iter()
        .filter(|reason| reason.is_reset())
        .map(ResetReason::name)
        .collect();
    assert_eq!(
        resets,
        ["EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT", "WDT", "CPU_LOCKUP"]
    );
}

#[test]
fn the_gate_passes_a_recent_reset_of_a_real_flight() {
    let (allowed, reason) = should_restore(Some(&glide()), true, ResetReason::TaskWatchdog, 1030);
    assert!(allowed);
    assert_eq!(
        reason,
        Reason::Recover {
            stage: Stage::Gliding,
            age_s: 30
        }
    );
    assert_eq!(reason.to_string(), "recover gliding, 30s after checkpoint");
    // BOOSTING is the boost itself: the latch is still nested, and the reset and the age carry it
    assert!(should_restore(Some(&staged(2, 1000)), false, ResetReason::Panic, 1030).0);
    assert!(should_restore(Some(&staged(4, 1000)), true, ResetReason::CpuLockup, 1030).0);
}

#[test]
fn the_age_window_is_zero_to_six_hundred_seconds() {
    let gate = |now_s| should_restore(Some(&glide()), true, ResetReason::Software, now_s);
    assert!(gate(1000).0);
    assert!(gate(1000 + MAX_AGE_S).0);
    assert_eq!(gate(1000 + MAX_AGE_S + 1), (false, Reason::Age(601)));
    assert_eq!(gate(999), (false, Reason::Age(-1)));
    assert_eq!(gate(999).1.to_string(), "crumb age -1s outside 0..600s");
    // the age saturates rather than wrapping on an absurd stamp
    let ancient = staged(3, i64::MIN);
    assert_eq!(
        should_restore(Some(&ancient), true, ResetReason::Software, i64::MAX).1,
        Reason::Age(i64::MAX)
    );
}

#[test]
fn the_gate_refuses_with_the_first_failing_reason() {
    let gate = |crumb: Option<&Crumb>, separated, reset| should_restore(crumb, separated, reset, 1030);
    assert_eq!(gate(None, true, ResetReason::Software), (false, Reason::NoCrumb));
    let stageless = Crumb { stage: None, ..glide() };
    let unstamped = Crumb {
        stamp_s: None,
        ..glide()
    };
    assert_eq!(
        gate(Some(&stageless), true, ResetReason::Software),
        (false, Reason::Torn)
    );
    assert_eq!(
        gate(Some(&unstamped), true, ResetReason::Software),
        (false, Reason::Torn)
    );
    assert_eq!(
        gate(Some(&glide()), true, ResetReason::PowerOn),
        (false, Reason::PowerOn)
    );
    assert_eq!(
        gate(Some(&glide()), true, ResetReason::Brownout),
        (false, Reason::PowerOn)
    );
    assert_eq!(
        gate(Some(&glide()), false, ResetReason::Software),
        (false, Reason::Nested)
    );
    assert_eq!(
        gate(Some(&staged(4, 1000)), false, ResetReason::Watchdog),
        (false, Reason::Nested)
    );
    assert_eq!(
        gate(Some(&glide()), false, ResetReason::Software).1.to_string(),
        "separation switch reads nested"
    );
}

#[test]
fn the_boot_rules_come_before_the_reset_and_the_age() {
    // DONE is a new session, SETTING a cold pad boot, whatever the reason, the latch or the age
    for (reset, now_s) in [(ResetReason::Software, 1030), (ResetReason::PowerOn, 99_999)] {
        for separated in [true, false] {
            let done = should_restore(Some(&staged(5, 1000)), separated, reset, now_s);
            assert_eq!(done, (false, Reason::AfterDone));
            let setting = should_restore(Some(&staged(1, 1000)), separated, reset, now_s);
            assert_eq!(setting, (false, Reason::OnThePad));
        }
    }
    for id in [0, 6, -1, i32::MAX] {
        let (allowed, reason) = should_restore(Some(&staged(id, 1000)), true, ResetReason::Software, 1030);
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
    assert_eq!(plan.identity, glide().identity);
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
    assert!(plan.identity.is_empty());
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
    assert_eq!(
        restore(&mut nvs, true, true, ResetReason::Software, 1030, SENSORS),
        Restore::Cold
    );
    assert!(nvs.writes.is_empty());
}

#[test]
fn restore_clears_the_stage_when_disabled_or_refused() {
    let blob = glide().to_json();
    let mut disabled = Memory::holding(Some(3), Some(blob.as_bytes()));
    assert_eq!(
        restore(&mut disabled, false, true, ResetReason::Software, 1030, SENSORS),
        Restore::Disabled
    );
    assert_eq!(disabled.stage, Some(NO_STAGE));
    let mut refused = Memory::holding(Some(3), Some(blob.as_bytes()));
    assert_eq!(
        restore(&mut refused, true, true, ResetReason::PowerOn, 1030, SENSORS),
        Restore::Refused(Reason::PowerOn)
    );
    assert_eq!(refused.stage, Some(NO_STAGE));
    let mut done = Memory::holding(Some(5), Some(blob.as_bytes()));
    assert_eq!(
        restore(&mut done, true, true, ResetReason::Software, 1030, SENSORS),
        Restore::Refused(Reason::AfterDone)
    );
}

#[test]
fn restore_warm_starts_a_passed_gate_and_keeps_the_crumb() {
    let blob = glide().to_json();
    let mut nvs = Memory::holding(Some(3), Some(blob.as_bytes()));
    let Restore::Warm { reason, plan } = restore(&mut nvs, true, true, ResetReason::Panic, 1030, SENSORS) else {
        panic!("no warm start");
    };
    assert_eq!(reason.to_string(), "recover gliding, 30s after checkpoint");
    assert_eq!(plan, ApplyPlan::new(&glide(), SENSORS).unwrap());
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
    let Restore::Warm { plan, .. } = restore(&mut nvs, true, false, ResetReason::TaskWatchdog, 5001, SENSORS) else {
        panic!("no warm start");
    };
    assert_eq!(plan.stage, Stage::Boosting);
    assert_eq!(plan.rebase.unwrap().ground_m, 100.0);
}
