//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The telemetry streams (doc/specs/avionics-port.md M6, M9): which entry writes which stream, with its file, its
//! columns, its decimation window and whether it is durable, as plan B's drivers and tasks build their Telemetry at
//! setup (drivers/lsm6dso32.py:124-126, drivers/bmi323.py:100-102, drivers/icp10111.py:123-125,
//! drivers/bmp581.py:69-71, drivers/bmm350.py:99-100, tasks/attitude.py:100-102, drivers/separation.py:57,
//! tasks/sequencer.py:110, warmstart.py:318). M6 records the LSM6DSO32, the ICP-10111 and the separation switch, M7
//! the stage machine's transitions, M8 the checkpoint's timeline, and M9 the SEN0697 (the BMI323, the BMP581 and the
//! BMM350), the attitude filter and the stage machine's veto episodes; every other part has none yet.
//!
//! The port, against plan B:
//!  * Plan B registers a stream in its task's setup. The firmware registers one at `recorder::open` for every named
//!    entry, enabled or not (board_config's `entries`), so a part the layout enables at boot still has its stream.
//!  * MEASURED -> REPORTED (the operator, 2026-10-08): no device stream is decimated. Plan B thinned each one to its
//!    entry's `telemetry_ms` (0 or absent: the global `recorder.telemetry_ms`) and attitude.csv to 100 ms by default,
//!    a MicroPython-era economy. Here every sample a part measures, and every attitude the filter publishes, is a row,
//!    and each part runs at the rate its consumers need instead. `telemetry_ms` is read by nothing here, whatever it
//!    holds (plan B multiplied it by 1000: null raised at setup, text made a text that failed later, a bool read as 0
//!    or 1 ms; validate() still refuses all three).
//!  * Plan B passes `separation.csv` no window, so the global decimates it, while its note says "durable, every
//!    event". Here the stream is durable and never decimated.
//!  * Plan B's sequencer passes its `<name>.csv` no window either. Its rows are the stage transitions, a handful a
//!    flight, so here that stream too is durable and never decimated.
//!  * Plan B's checkpoint passes `checkpoint.csv` no window either. Its rows come once a stage change and once a
//!    `checkpoint_s` aloft, and the last one before a reset is the post-mortem's record of where the flight was, so
//!    here it is durable and never decimated too.
//!  * So every stream here is never decimated; the device streams stay not durable (many rows a second, which the
//!    ring_writer's own cadence carries), the event records durable.
//!  * A barometer's row gains a fifth column, `change`, its climb rate in m/s (crates/control `change`), appended so
//!    plan B's four stay where they were. A reader takes plan B's four-column rows as they are
//!    (tools/recorder_flight.py).
//!  * separation.csv gains two columns, `moment` and `duration`: the first edge of the contact's episode the row
//!    concludes, on the uptime column's own clock, microseconds, and the microseconds from it to the episode's last
//!    edge (the wobble). A row's `uptime` is the tick that concluded the episode, a quiet after its last edge, so the
//!    latency is in the data. A reader takes plan B's two-column rows as they are (tools/recorder_flight.py).
//!  * Its `event` gains a third value: plan B's `separated` and `nested`, and the port's `blip`, an episode that ended
//!    on the level the latch already held (the contact moved and came back), so a flight shows every movement of the
//!    contact. A blip moves neither the latch nor the stage; the readers take the event as a word, whichever it is.
//!  * The stage machine's entry writes a second stream, the port's own ([`vetoes`]): `vetoes.csv`, a begin and an end
//!    row per veto episode of the multi-source machine (crates/sequencer `VetoEdge`), fixed, durable and never
//!    decimated like the transitions' record. Those rows stay out of `<name>.csv`, whose every row
//!    tools/assemble_capture.py takes for a stage event.

use board_config::Entry;
use control::attitude;

/// The IMUs' columns, the LSM6DSO32's and the BMI323's alike (plan B drivers/lsm6dso32.py:125,
/// drivers/bmi323.py:101): accel in g, the gyro as raw centidegrees a second, and the INT1 edges the tick saw.
pub const IMU_FIELDS: [&str; 7] = ["ax", "ay", "az", "gx", "gy", "gz", "irq_runs"];

/// The barometers' columns, the ICP-10111's and the BMP581's alike: plan B's four (drivers/icp10111.py:124,
/// drivers/bmp581.py:70), m AMSL, degrees C, Pa and m above the ground, then the port's `change`, the climb rate in
/// m/s (crates/control `change`), appended.
pub const BAROMETER_FIELDS: [&str; 5] = ["altitude", "temperature", "pressure", "elevation", "change"];

/// The BNO055's columns (plan B drivers/bno055.py:105, M18): the heading in degrees, the roll and the pitch in
/// centidegrees (integers, as plan B pushed its fixnums), the accelerometer in g.
pub const ABSOLUTE_FIELDS: [&str; 6] = ["heading", "roll", "pitch", "ax", "ay", "az"];

/// The BMM350's columns (plan B drivers/bmm350.py:99): the field per axis in the part's own counts, raw until the
/// operator calibrates it, then centred and normalised (crates/drivers `bmm350`).
pub const MAGNETOMETER_FIELDS: [&str; 3] = ["mx", "my", "mz"];

/// The SDP810 pitot's columns (plan B drivers/sdp810.py:164-166): the tared dynamic pressure in Pa x 100, the
/// airspeed in cm/s and the temperature in degrees C x 100, all integers as plan B pushed them.
pub const PITOT_FIELDS: [&str; 3] = ["dynamic_pressure", "airspeed_cms", "temperature"];

/// The INA226's columns (plan B drivers/ina226.py:93-95): mV, mA, mW and the over-current ALERT trips since boot.
pub const POWER_FIELDS: [&str; 4] = ["voltage_mv", "current_ma", "power_mw", "alerts"];

/// The flight loop's columns (plan B tasks/flight.py:146-148), as `flight.csv` whatever the entry's name: the stage id,
/// active, the airspeed estimate in cm/s, the fin cap, the roll and pitch setpoints in centidegrees, the heading error,
/// the per-axis demands in whole degrees, the wind in cm/s and its from-direction, and whether it navigates
/// dead-reckoned.
pub const FLIGHT_FIELDS: [&str; 13] = [
    "stage",
    "active",
    "airspeed_cms",
    "fin_cap",
    "roll_sp",
    "pitch_sp",
    "heading_err",
    "roll_cmd",
    "pitch_cmd",
    "yaw_cmd",
    "wind_cms",
    "wind_from",
    "reckoning",
];

/// flight.csv, the flight loop's stream whatever its entry's name (plan B's `Telemetry('flight.csv', ...)`).
const FLIGHT_FILE: &str = "flight.csv";

/// The fin servos' columns (plan B drivers/sg90.py:187, the MG90S alike): the commanded angle in whole degrees, the
/// pulse driven in us, and done (0 a command issued, 1 a move completed by its estimate).
pub const SERVO_FIELDS: [&str; 3] = ["angle", "pulse_us", "done"];

/// The laser rangers' columns (plan B drivers/vl53l1x.py:150, vl53l4cx.py alike): the height above ground in m and
/// the data-ready edges its read took.
pub const LASER_FIELDS: [&str; 2] = ["agl", "irq_runs"];

/// The GNSS fix's columns (plan B gnss.py:173): lat and lon (written from the 1e-7 degree integers, 7 decimals), the
/// ground speed in knots and the course in degrees, as an RMC gives them.
pub const GNSS_FIELDS: [&str; 4] = ["lat", "lon", "speed_kn", "course"];

/// The GNSS's GGA columns (plan B gnss.py:186-188, `<name>_gga.csv`): altitude and elevation in m, the fix quality,
/// the satellites used and the HDOP in hundredths.
pub const GGA_FIELDS: [&str; 5] = ["altitude_m", "elevation_m", "quality", "satellites", "hdop_cd"];

/// The GNSS's sky columns (plan B gnss.py:195-197, `<name>_sky.csv`): one row per diagnostics burst.
pub const SKY_FIELDS: [&str; 8] = ["in_view", "used", "mode", "cn0_1", "cn0_2", "cn0_3", "cn0_4", "antenna"];

/// A task's durable event record (plan B task.py:104, `<name>_events.csv`): one text per event.
pub const EVENT_FIELDS: [&str; 1] = ["event"];

/// The GNSS drivers (plan B's two NMEA modules): each writes `<name>.csv` and the three companions ([`companions`]).
const GNSS_DRIVERS: [&str; 2] = ["neo6mv2", "atgm336h"];

/// The separation switch's columns: plan B's event (`separated` or `nested`, and the port's `blip`) and the stage after
/// it (drivers/separation.py:57), then the port's `moment`, the uptime (µs) of the first edge of the episode the event
/// concludes (drivers::separation `Event::moment_us`: for a separation, the separation moment), and `duration`, the µs
/// from it to the episode's last edge (`Event::duration_us`: the contact's wobble, 0 for a clean single edge).
pub const SEPARATION_FIELDS: [&str; 4] = ["event", "stage", "moment", "duration"];

/// The stage machine's columns (plan B tasks/sequencer.py:110): the stage it moved to, or found the board moved to,
/// and why (`launch |a|=...`, or `external` for a move it did not make).
pub const SEQUENCER_FIELDS: [&str; 2] = ["stage", "reason"];

/// The stage machine's veto episodes' columns, the port's own (crates/sequencer `VetoEdge`, one row per edge):
///  * `detector` -- the detector held (`Detector::name`: `launch_altitude`, `launch_acceleration`, `apogee` or
///    `stillness`);
///  * `edge` -- `begin` on the episode's first vetoed step, `end` on the first step after it (`VetoEdge::name`); both
///    rows carry a vetoed step, an end row the episode's last;
///  * `decider` -- the deciding source's device name (`Binding::source` of `Veto::decider`);
///  * `kinds` -- the vetoes that held it (`Veto::kinds`: `peer`, `at_rest` or `thrust`, two joined by `+`);
///  * `contrary` and `live` -- how many of the decider's live peers were contrary, and how many voting sources were
///    live, the decider included (`Veto::contrary`, `Veto::live`);
///  * `value` -- the decider's value (`Veto::value`): |accel| in g for the accel launch and the stillness, the
///    elevation in m for the altitude launch and the apogee.
pub const VETOES_FIELDS: [&str; 7] = ["detector", "edge", "decider", "kinds", "contrary", "live", "value"];

/// The checkpoint's columns (plan B warmstart.py:318-319): the stage's id, 1 (SETTING) to 5 (DONE), as plan B pushes
/// `controller.stage`; the altitude in m AMSL, the speed and the airspeed in m/s; and the uptime in ms the checkpoint
/// was taken at.
pub const CHECKPOINT_FIELDS: [&str; 5] = ["stage", "altitude", "speed", "airspeed", "ticks_ms"];

/// The stage machine's implementation in a device entry (plan B's `@task.activity('sequencer')`): the entry that
/// writes both `<name>.csv` and [`VETOES_FILE`].
const SEQUENCER: &str = "sequencer";

/// The separation switch's file, whatever its entry's name (plan B drivers/separation.py:57).
const SEPARATION_FILE: &str = "separation.csv";

/// The checkpoint's file, whatever its entry's name (plan B warmstart.py:318).
const CHECKPOINT_FILE: &str = "checkpoint.csv";

/// The stage machine's veto episodes' file, whatever its entry's name.
const VETOES_FILE: &str = "vetoes.csv";

/// The window of a stream that is never decimated, every row going out: negative, as the recorder's
/// `Telemetry::window_us` reads it. Every stream here has it.
const NEVER_DECIMATED: i32 = -1;

/// A telemetry stream, as the recorder registers it.
#[derive(Debug, PartialEq, Eq)]
pub struct Stream {
    /// The file, without the session prefix.
    pub file: String,
    /// The columns after `uptime`.
    pub fields: &'static [&'static str],
    /// The decimation window in microseconds, as the recorder's registry takes it (positive its own, 0 the global
    /// `recorder.telemetry_ms`, negative never decimated): negative for every stream here, measured -> reported.
    pub decimate_us: i32,
    /// Whether each row wakes the ring_writer at once: separation.csv; the stage machine's, the stage transitions'
    /// record, and vetoes.csv beside it; and checkpoint.csv, the last state before a reset.
    pub durable: bool,
}

/// The stream `entry` writes, when its part is one the port records, every one never decimated: the IMUs (LSM6DSO32,
/// BMI323), the barometers (ICP-10111, BMP581) and the BMM350 as `<name>.csv`, the attitude filter as the fixed
/// `attitude.csv`, the fin servos as `<name>.csv`; the separation switch as the fixed `separation.csv`, the stage machine as `<name>.csv` and the
/// checkpoint as the fixed `checkpoint.csv`, the last three durable. None for any other part. The stage machine's
/// second stream is [`vetoes`].
pub fn stream(entry: &Entry) -> Option<Stream> {
    let named = || format!("{}.csv", entry.name);
    let (file, fields, durable): (String, &'static [&'static str], bool) = match entry.implementation {
        "lsm6dso32" | "bmi323" => (named(), &IMU_FIELDS, false),
        "icp10111" | "bmp280" | "bmp581" => (named(), &BAROMETER_FIELDS, false),
        "bmm350" => (named(), &MAGNETOMETER_FIELDS, false),
        "bno055" => (named(), &ABSOLUTE_FIELDS, false),
        "sdp810" => (named(), &PITOT_FIELDS, false),
        "ina226" => (named(), &POWER_FIELDS, false),
        "vl53l1x" | "vl53l4cx" => (named(), &LASER_FIELDS, false),
        "sg90" | "mg90s" => (named(), &SERVO_FIELDS, false),
        "flight" => (FLIGHT_FILE.to_string(), &FLIGHT_FIELDS, false),
        driver if GNSS_DRIVERS.contains(&driver) => (named(), &GNSS_FIELDS, false),
        "attitude" => (attitude::STREAM.to_string(), &attitude::FIELDS, false),
        "separation" => (SEPARATION_FILE.to_string(), &SEPARATION_FIELDS, true),
        SEQUENCER => (named(), &SEQUENCER_FIELDS, true),
        "checkpoint" => (CHECKPOINT_FILE.to_string(), &CHECKPOINT_FIELDS, true),
        _ => return None,
    };
    Some(Stream {
        file,
        fields,
        decimate_us: NEVER_DECIMATED,
        durable,
    })
}

/// The streams an entry writes beside its own `<name>.csv`, each with the device key it is registered under: the
/// GNSS's `<name>_gga.csv`, `<name>_sky.csv` and the durable `<name>_events.csv` (plan B gnss.py, task.py:104), all
/// never decimated. Empty for any other entry.
pub fn companions(entry: &Entry) -> Vec<(String, Stream)> {
    if !GNSS_DRIVERS.contains(&entry.implementation) {
        return Vec::new();
    }
    let companion = |suffix: &str, fields: &'static [&'static str], durable: bool| {
        let key = format!("{}_{suffix}", entry.name);
        let stream = Stream {
            file: format!("{key}.csv"),
            fields,
            decimate_us: NEVER_DECIMATED,
            durable,
        };
        (key, stream)
    };
    vec![
        companion("gga", &GGA_FIELDS, false),
        companion("sky", &SKY_FIELDS, false),
        companion("events", &EVENT_FIELDS, true),
    ]
}

/// The stream the stage machine's entry writes beside its own `<name>.csv`: its veto episodes, the fixed `vetoes.csv`
/// whatever the entry's name, durable and never decimated like the transitions' record. None for any other entry.
pub fn vetoes(entry: &Entry) -> Option<Stream> {
    (entry.implementation == SEQUENCER).then(|| Stream {
        file: VETOES_FILE.to_string(),
        fields: &VETOES_FIELDS,
        decimate_us: NEVER_DECIMATED,
        durable: true,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        ABSOLUTE_FIELDS, BAROMETER_FIELDS, CHECKPOINT_FIELDS, FLIGHT_FIELDS, IMU_FIELDS, LASER_FIELDS,
        MAGNETOMETER_FIELDS, PITOT_FIELDS, POWER_FIELDS, SEPARATION_FIELDS, SEQUENCER_FIELDS, Stream, VETOES_FIELDS,
        stream, vetoes,
    };
    use super::{EVENT_FIELDS, GGA_FIELDS, GNSS_FIELDS, SERVO_FIELDS, SKY_FIELDS, companions};
    use board_config::{directory, entries};
    use serde_json::{Value, json};

    // The windows below are literals, never the module's constants: -1 (never decimated) is the recorder's contract
    // (`Telemetry::window_us`), so a wrong sentinel must fail a test.

    /// The firmware's defaults: golden/build_config_default.py's dump of plan B's config_default.py.
    const CONFIG_DEFAULT: &str = include_str!("../../../golden/config_default.json");

    /// The defaults document (the fixture's `document`).
    fn defaults() -> Value {
        let fixture: Value = serde_json::from_str(CONFIG_DEFAULT).expect("the fixture is JSON");
        fixture["document"].clone()
    }

    /// The stream of the one entry `document` names.
    fn only(document: &Value) -> Option<Stream> {
        let named = entries(document);
        assert_eq!(named.len(), 1, "the document names one entry");
        stream(&named[0])
    }

    /// The veto stream of the one entry `document` names.
    fn only_vetoes(document: &Value) -> Option<Stream> {
        let named = entries(document);
        assert_eq!(named.len(), 1, "the document names one entry");
        vetoes(&named[0])
    }

    /// A document with one sensor `name` run by `driver`, carrying `telemetry_ms` when it is Some.
    fn sensor(name: &str, driver: &str, telemetry_ms: Option<Value>) -> Value {
        let mut device = json!({"name": name, "driver": driver});
        if let Some(window) = telemetry_ms {
            device["telemetry_ms"] = window;
        }
        json!({"sensors": [device]})
    }

    /// A device stream: `<file>` with `fields`, never decimated, not durable.
    fn measured(file: &str, fields: &'static [&'static str]) -> Stream {
        Stream {
            file: file.to_string(),
            fields,
            decimate_us: -1,
            durable: false,
        }
    }

    /// An event record: `<file>` with `fields`, never decimated, durable.
    fn event(file: &str, fields: &'static [&'static str]) -> Stream {
        Stream {
            file: file.to_string(),
            fields,
            decimate_us: -1,
            durable: true,
        }
    }

    #[test]
    fn the_defaults_record_every_ported_part_and_the_stage_machine_s_vetoes() {
        let document = defaults();
        let streams: Vec<(&str, Stream)> = entries(&document)
            .iter()
            .filter_map(|entry| Some((entry.name, stream(entry)?)))
            .collect();
        assert_eq!(
            streams,
            [
                ("imu_lsm6dso32", measured("imu_lsm6dso32.csv", &IMU_FIELDS)),
                ("imu_bno055", measured("imu_bno055.csv", &ABSOLUTE_FIELDS)),
                ("attitude", measured("attitude.csv", &control::attitude::FIELDS)),
                ("baro_icp10111", measured("baro_icp10111.csv", &BAROMETER_FIELDS)),
                ("baro_bmp280", measured("baro_bmp280.csv", &BAROMETER_FIELDS)),
                ("imu_bmi323", measured("imu_bmi323.csv", &IMU_FIELDS)),
                ("mag_bmm350", measured("mag_bmm350.csv", &MAGNETOMETER_FIELDS)),
                ("baro_bmp581", measured("baro_bmp581.csv", &BAROMETER_FIELDS)),
                ("airspeed_sdp810", measured("airspeed_sdp810.csv", &PITOT_FIELDS)),
                ("laser_agl", measured("laser_agl.csv", &LASER_FIELDS)),
                ("laser_agl_l1x", measured("laser_agl_l1x.csv", &LASER_FIELDS)),
                ("power_ina226", measured("power_ina226.csv", &POWER_FIELDS)),
                ("gnss", measured("gnss.csv", &GNSS_FIELDS)),
                ("separation", event("separation.csv", &SEPARATION_FIELDS)),
                ("servo_yaw", measured("servo_yaw.csv", &SERVO_FIELDS)),
                ("servo_eleron_left", measured("servo_eleron_left.csv", &SERVO_FIELDS)),
                ("servo_eleron_right", measured("servo_eleron_right.csv", &SERVO_FIELDS)),
                ("sequencer", event("sequencer.csv", &SEQUENCER_FIELDS)),
                ("checkpoint", event("checkpoint.csv", &CHECKPOINT_FIELDS)),
                ("flight", measured("flight.csv", &FLIGHT_FIELDS)),
            ]
        );
        let vetoes: Vec<(&str, Stream)> = entries(&document)
            .iter()
            .filter_map(|entry| Some((entry.name, vetoes(entry)?)))
            .collect();
        assert_eq!(vetoes, [("sequencer", event("vetoes.csv", &VETOES_FIELDS))]);
    }

    #[test]
    fn the_columns_are_plan_b_s_with_the_port_s_appended() {
        assert_eq!(IMU_FIELDS.join(";"), "ax;ay;az;gx;gy;gz;irq_runs");
        assert_eq!(MAGNETOMETER_FIELDS.join(";"), "mx;my;mz");
        // plan B's four first and unchanged, so a reader by position or by name still finds them; the change trails
        assert_eq!(
            BAROMETER_FIELDS.join(";"),
            "altitude;temperature;pressure;elevation;change"
        );
        assert_eq!(
            BAROMETER_FIELDS[..4],
            ["altitude", "temperature", "pressure", "elevation"]
        );
        assert_eq!(
            control::attitude::FIELDS.join(";"),
            "heading_cd;roll_cd;pitch_cd;free;mag_known;mag_offset_cd",
            "plan B tasks/attitude.py:101"
        );
    }

    #[test]
    fn every_device_stream_reports_every_measurement_whatever_its_window() {
        // the operator's measured -> reported: plan B's own window (100 ms), its global (0 or absent), a negative, and
        // what plan B could not multiply (text, null, a bool, a list, an object): never decimated, every one
        let windows = [
            None,
            Some(json!(100)),
            Some(json!(0)),
            Some(json!(12.5)),
            Some(json!(-1)),
            Some(json!(1e12)),
            Some(json!("50")),
            Some(Value::Null),
            Some(json!(true)),
            Some(json!([50])),
            Some(json!({"ms": 50})),
        ];
        for driver in ["lsm6dso32", "bmi323", "icp10111", "bmp280", "bmp581", "bmm350"] {
            for telemetry_ms in &windows {
                let device = only(&sensor("part", driver, telemetry_ms.clone())).expect("a stream");
                assert_eq!(
                    (device.decimate_us, device.durable),
                    (-1, false),
                    "{driver} {telemetry_ms:?}"
                );
            }
        }
        for telemetry_ms in [json!(100), json!(0), json!(-1)] {
            let document =
                json!({"sensors": [{"name": "attitude", "activity": "attitude", "telemetry_ms": telemetry_ms}]});
            assert_eq!(only(&document).expect("a stream").decimate_us, -1, "{telemetry_ms}");
        }
    }

    #[test]
    fn the_sen0697_writes_plan_b_s_columns_under_its_entries_names() {
        assert_eq!(
            only(&sensor("imu_backup", "bmi323", Some(json!(10)))),
            Some(measured("imu_backup.csv", &IMU_FIELDS))
        );
        assert_eq!(
            only(&sensor("baro_backup", "bmp581", None)),
            Some(measured("baro_backup.csv", &BAROMETER_FIELDS))
        );
        assert_eq!(
            only(&sensor("compass", "bmm350", Some(json!(0)))),
            Some(measured("compass.csv", &MAGNETOMETER_FIELDS))
        );
    }

    #[test]
    fn the_attitude_filter_writes_attitude_csv_whatever_its_name() {
        let document = json!({"sensors": [
            {"name": "filter", "activity": "attitude", "enabled": false, "telemetry_ms": 100},
        ]});
        assert_eq!(
            only(&document),
            Some(measured("attitude.csv", &control::attitude::FIELDS)),
            "disabled, renamed and with plan B's window: still attitude.csv, never decimated"
        );
    }

    #[test]
    fn the_attitude_filter_is_known_by_its_activity_not_its_name() {
        // the SEN0253's BNO055 publishes `attitude` too: an entry of that name records the BNO055's columns
        assert_eq!(
            only(&sensor("attitude", "bno055", None)),
            Some(measured("attitude.csv", &ABSOLUTE_FIELDS))
        );
        assert_eq!(
            only(&json!({"sensors": [{"name": "attitude", "activity": "attitudes"}]})),
            None
        );
    }

    #[test]
    fn the_checkpoint_writes_plan_b_s_columns_to_checkpoint_csv_whatever_its_name() {
        assert_eq!(CHECKPOINT_FIELDS.join(";"), "stage;altitude;speed;airspeed;ticks_ms");
        let document = json!({"components": [
            {"name": "crumbs", "activity": "checkpoint", "enabled": false, "telemetry_ms": 50},
        ]});
        assert_eq!(
            only(&document),
            Some(event("checkpoint.csv", &CHECKPOINT_FIELDS)),
            "disabled, renamed and with a window of its own: still checkpoint.csv, durable and never decimated"
        );
        assert_eq!(only_vetoes(&document), None, "the checkpoint writes no vetoes");
    }

    #[test]
    fn the_checkpoint_is_known_by_its_activity_not_its_name() {
        assert_eq!(
            only(&json!({"components": [{"name": "checkpoint", "activity": "watchdog"}]})),
            None
        );
        assert_eq!(
            only(&json!({"components": [{"name": "checkpoint", "driver": "x", "activity": "checkpoint"}]})),
            None,
            "a truthy driver runs the entry instead"
        );
    }

    #[test]
    fn the_stage_machine_writes_plan_b_s_columns_under_its_entry_s_name() {
        assert_eq!(SEQUENCER_FIELDS.join(";"), "stage;reason");
        let document = json!({"components": [
            {"name": "stages", "activity": "sequencer", "enabled": false, "telemetry_ms": 50},
        ]});
        assert_eq!(
            only(&document),
            Some(event("stages.csv", &SEQUENCER_FIELDS)),
            "disabled, and with a window of its own: still durable and never decimated"
        );
    }

    #[test]
    fn the_stage_machine_writes_its_vetoes_to_vetoes_csv_whatever_its_name() {
        // the cells crates/sequencer's VetoEdge carries, the edge beside the detector it qualifies
        assert_eq!(
            VETOES_FIELDS.join(";"),
            "detector;edge;decider;kinds;contrary;live;value"
        );
        let document = json!({"components": [
            {"name": "stages", "activity": "sequencer", "enabled": false, "telemetry_ms": 50},
        ]});
        assert_eq!(
            only_vetoes(&document),
            Some(event("vetoes.csv", &VETOES_FIELDS)),
            "disabled, renamed and with a window of its own: still vetoes.csv, durable and never decimated"
        );
    }

    #[test]
    fn only_the_stage_machine_writes_vetoes() {
        for document in [
            sensor("imu_lsm6dso32", "lsm6dso32", None),
            sensor("baro_icp10111", "icp10111", None),
            json!({"sensors": [{"name": "attitude", "activity": "attitude"}]}),
            json!({"components": [{"name": "separation", "activity": "separation"}]}),
            json!({"components": [{"name": "vetoes", "activity": "flight"}]}),
            json!({"components": [{"name": "sequencer", "activity": "flight"}]}),
            json!({"components": [{"name": "sequencer", "driver": "x", "activity": "sequencer"}]}),
        ] {
            assert_eq!(only_vetoes(&document), None, "{document}");
        }
    }

    #[test]
    fn the_flight_loop_writes_plan_b_s_columns_to_flight_csv_whatever_its_name() {
        assert_eq!(
            FLIGHT_FIELDS.join(";"),
            "stage;active;airspeed_cms;fin_cap;roll_sp;pitch_sp;heading_err;roll_cmd;pitch_cmd;yaw_cmd;wind_cms;\
             wind_from;reckoning",
            "plan B tasks/flight.py:146"
        );
        assert_eq!(
            only(&json!({"components": [{"name": "law", "activity": "flight", "telemetry_ms": 100}]})),
            Some(measured("flight.csv", &FLIGHT_FIELDS)),
            "renamed and with plan B's window: still flight.csv, never decimated"
        );
    }

    #[test]
    fn the_stage_machine_is_known_by_its_activity_not_its_name() {
        assert_eq!(
            only(&json!({"components": [{"name": "sequencer", "activity": "flight"}]})),
            Some(measured("flight.csv", &FLIGHT_FIELDS)),
            "the flight loop, not the stage machine"
        );
        assert_eq!(
            only(&json!({"components": [{"name": "sequencer", "activity": "sequencers"}]})),
            None
        );
    }

    #[test]
    fn separation_appends_the_moment_and_the_duration_to_plan_b_s_columns() {
        // plan B's two columns first and unchanged, so a reader by position or by name still finds them; the moment
        // and the duration trail them
        assert_eq!(SEPARATION_FIELDS.join(";"), "event;stage;moment;duration");
        assert_eq!(SEPARATION_FIELDS[..2], ["event", "stage"]);
    }

    #[test]
    fn a_disabled_entry_still_gets_its_stream() {
        let document = json!({"sensors": [
            {"name": "imu_lsm6dso32", "driver": "lsm6dso32", "enabled": false, "telemetry_ms": 20},
            {"name": "baro_icp10111", "driver": "icp10111", "enabled": false},
        ]});
        assert!(
            directory(&document).is_empty(),
            "the directory lists only enabled entries"
        );
        let streams: Vec<(String, i32)> = entries(&document)
            .iter()
            .filter_map(stream)
            .map(|stream| (stream.file, stream.decimate_us))
            .collect();
        assert_eq!(
            streams,
            [
                ("imu_lsm6dso32.csv".to_string(), -1),
                ("baro_icp10111.csv".to_string(), -1)
            ]
        );
    }

    #[test]
    fn separation_is_durable_never_decimated_and_always_separation_csv() {
        let document = json!({"components": [
            {"name": "pads", "activity": "separation", "telemetry_ms": 50},
        ]});
        assert_eq!(only(&document), Some(event("separation.csv", &SEPARATION_FIELDS)));
    }

    #[test]
    fn the_pitot_the_power_monitor_and_the_lasers_write_plan_b_s_columns_never_decimated() {
        for (driver, fields) in [
            ("sdp810", &PITOT_FIELDS[..]),
            ("ina226", &POWER_FIELDS[..]),
            ("vl53l1x", &LASER_FIELDS[..]),
            ("vl53l4cx", &LASER_FIELDS[..]),
            ("sg90", &SERVO_FIELDS[..]),
            ("mg90s", &SERVO_FIELDS[..]),
        ] {
            let found = only(&sensor("part", driver, Some(json!(100)))).expect(driver);
            assert_eq!(
                (found.file.as_str(), found.fields, found.decimate_us, found.durable),
                ("part.csv", fields, -1, false)
            );
        }
        assert_eq!(PITOT_FIELDS.join(";"), "dynamic_pressure;airspeed_cms;temperature");
        assert_eq!(POWER_FIELDS.join(";"), "voltage_mv;current_ma;power_mw;alerts");
        assert_eq!(LASER_FIELDS.join(";"), "agl;irq_runs");
        assert_eq!(SERVO_FIELDS.join(";"), "angle;pulse_us;done");
    }

    #[test]
    fn the_gnss_writes_its_fix_and_three_companions() {
        for driver in ["neo6mv2", "atgm336h"] {
            let document = sensor("gnss", driver, Some(json!(200)));
            assert_eq!(only(&document), Some(measured("gnss.csv", &GNSS_FIELDS)), "{driver}");
            let found = companions(&entries(&document)[0]);
            assert_eq!(
                found,
                vec![
                    ("gnss_gga".to_string(), measured("gnss_gga.csv", &GGA_FIELDS)),
                    ("gnss_sky".to_string(), measured("gnss_sky.csv", &SKY_FIELDS)),
                    ("gnss_events".to_string(), event("gnss_events.csv", &EVENT_FIELDS)),
                ]
            );
        }
        assert!(companions(&entries(&sensor("imu", "lsm6dso32", None))[0]).is_empty());
        assert_eq!(GNSS_FIELDS.join(";"), "lat;lon;speed_kn;course");
        assert_eq!(
            GGA_FIELDS.join(";"),
            "altitude_m;elevation_m;quality;satellites;hdop_cd"
        );
        assert_eq!(
            SKY_FIELDS.join(";"),
            "in_view;used;mode;cn0_1;cn0_2;cn0_3;cn0_4;antenna"
        );
    }

    #[test]
    fn a_part_the_port_does_not_record_has_no_stream() {
        assert_eq!(only(&sensor("part", "adxl375", Some(json!(10)))), None);
        assert_eq!(only(&json!({"sensors": [{"name": "no_implementation"}]})), None);
        let document = defaults();
        let adxl375 = entries(&document)
            .into_iter()
            .find(|entry| entry.implementation == "adxl375")
            .expect("the defaults carry an ADXL375");
        assert_eq!(stream(&adxl375), None);
    }
}
