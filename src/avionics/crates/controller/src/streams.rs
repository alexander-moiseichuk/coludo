//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device telemetry streams (doc/specs/avionics-port.md M6): which entry writes which stream,
//! with its file, its columns, its decimation window and whether it is durable, as plan B's drivers build their
//! Telemetry at setup (drivers/lsm6dso32.py:124-126, drivers/icp10111.py:123-125, drivers/separation.py:57). M6
//! records the LSM6DSO32, the ICP-10111 and the separation switch; every other part, the BMI323 among them until
//! M9, has none yet.
//!
//! The port, against plan B:
//!  * Plan B registers a stream in its task's setup. The firmware registers one at `recorder::open` for every named
//!    entry, enabled or not (board_config's `entries`), so a part the layout enables at boot still has its stream.
//!  * Plan B multiplies whatever `telemetry_ms` holds by 1000: null raises at setup, text makes a 1000-fold text
//!    that fails later, and a bool reads as 0 or 1 ms. validate() refuses all three. Here a number, int or float,
//!    is honoured, and anything else reads as 0, the global window.
//!  * Plan B passes `separation.csv` no window, so the global decimates it, while its note says "durable, every
//!    event". Here the stream is durable and never decimated.
//!  * separation.csv gains two columns, `moment` and `duration`: when the pads first parted (or, for a re-nest,
//!    closed), on the uptime column's own clock, microseconds, and how long they wobbled before the level held for
//!    good, microseconds. A row's `uptime` stays the verdict's read, so the debounce's latency is in the data. A reader
//!    takes plan B's two-column rows as they are (tools/recorder_flight.py).

use board_config::Entry;
use serde_json::Value;

/// The LSM6DSO32's columns (plan B drivers/lsm6dso32.py:125): accel in g, the gyro as raw centidegrees a second,
/// and the INT1 edges the tick saw.
pub const IMU_FIELDS: [&str; 7] = ["ax", "ay", "az", "gx", "gy", "gz", "irq_runs"];

/// The ICP-10111's columns (plan B drivers/icp10111.py:124): m AMSL, degrees C, Pa, m above the ground.
pub const BAROMETER_FIELDS: [&str; 4] = ["altitude", "temperature", "pressure", "elevation"];

/// The separation switch's columns: plan B's event and the stage after it (drivers/separation.py:57), then the port's
/// `moment`, the uptime (µs) the pin first left its old level in the episode the event concludes (drivers::separation
/// `Event::moment_us`: for a separation, the separation moment), and `duration`, the µs from it to the sustained new
/// level (`Event::duration_us`: the pads' wobble, 0 for a clean single edge).
pub const SEPARATION_FIELDS: [&str; 4] = ["event", "stage", "moment", "duration"];

/// The separation switch's file, whatever its entry's name (plan B drivers/separation.py:57).
const SEPARATION_FILE: &str = "separation.csv";

/// The window that reads the global `recorder.telemetry_ms` at each use: 0, as the recorder's
/// `Telemetry::window_us` reads it.
const GLOBAL_WINDOW_US: i32 = 0;

/// The window of a stream that is never decimated, every row going out: negative, as the recorder's
/// `Telemetry::window_us` reads it.
const NEVER_DECIMATED: i32 = -1;

/// Microseconds in a millisecond, as `telemetry_ms` is scaled to a window.
const MICROSECONDS_PER_MILLISECOND: f64 = 1000.0;

/// A device's telemetry stream, as the recorder registers it.
#[derive(Debug, PartialEq, Eq)]
pub struct Stream {
    /// The file, without the session prefix.
    pub file: String,
    /// The columns after `uptime`.
    pub fields: &'static [&'static str],
    /// The decimation window in microseconds: positive its own, 0 the global `recorder.telemetry_ms`, negative
    /// never decimated.
    pub decimate_us: i32,
    /// Whether each row wakes the ring_writer at once (separation.csv: the stage transition's record).
    pub durable: bool,
}

/// The stream `entry` writes, when its part is one the port records: the LSM6DSO32 and the ICP-10111 as
/// `<name>.csv` with their entry's window (`window_us`), and the separation switch as the fixed `separation.csv`,
/// durable and never decimated. None for any other part.
pub fn stream(entry: &Entry) -> Option<Stream> {
    let (file, fields, decimate_us, durable): (String, &'static [&'static str], i32, bool) = match entry.implementation
    {
        "lsm6dso32" => (format!("{}.csv", entry.name), &IMU_FIELDS, window_us(entry), false),
        "icp10111" => (
            format!("{}.csv", entry.name),
            &BAROMETER_FIELDS,
            window_us(entry),
            false,
        ),
        "separation" => (SEPARATION_FILE.to_string(), &SEPARATION_FIELDS, NEVER_DECIMATED, true),
        _ => return None,
    };
    Some(Stream {
        file,
        fields,
        decimate_us,
        durable,
    })
}

/// The decimation window `entry`'s `telemetry_ms` asks for, in microseconds.
///
/// A positive number is its own window, ms x 1000 rounded, saturating at `i32::MAX`, and at least 1 us: a window
/// below half a microsecond is still the entry's own (every row), never the global. A negative number is never
/// decimated ([`NEVER_DECIMATED`]). 0, an absent key and anything that is not a number (text, null, a bool) are the
/// global window. The scaling is f64, the JSON number's own width: f32 would not hold every whole-millisecond
/// window exactly past 16.8 s.
fn window_us(entry: &Entry) -> i32 {
    match entry.fields.get("telemetry_ms").and_then(Value::as_f64) {
        Some(milliseconds) if milliseconds > 0.0 => {
            ((milliseconds * MICROSECONDS_PER_MILLISECOND).round() as i32).max(1)
        }
        Some(milliseconds) if milliseconds < 0.0 => NEVER_DECIMATED,
        _ => GLOBAL_WINDOW_US,
    }
}

#[cfg(test)]
mod tests {
    use super::{BAROMETER_FIELDS, IMU_FIELDS, SEPARATION_FIELDS, Stream, stream};
    use board_config::{directory, entries};
    use serde_json::{Value, json};

    // The windows below are literals, never the module's constants: 0 (the global) and -1 (never decimated) are
    // the recorder's contract (`Telemetry::window_us`), so a wrong sentinel must fail a test.

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

    /// A document with one sensor `name` run by `driver`, carrying `telemetry_ms` when it is Some.
    fn sensor(name: &str, driver: &str, telemetry_ms: Option<Value>) -> Value {
        let mut device = json!({"name": name, "driver": driver});
        if let Some(window) = telemetry_ms {
            device["telemetry_ms"] = window;
        }
        json!({"sensors": [device]})
    }

    /// The window the LSM6DSO32 entry gets for `telemetry_ms` (absent when None).
    fn window(telemetry_ms: Option<Value>) -> i32 {
        only(&sensor("imu_lsm6dso32", "lsm6dso32", telemetry_ms))
            .expect("the LSM6DSO32 writes a stream")
            .decimate_us
    }

    #[test]
    fn the_defaults_record_the_imu_the_barometer_and_the_separation_switch() {
        let document = defaults();
        let streams: Vec<(&str, Stream)> = entries(&document)
            .iter()
            .filter_map(|entry| Some((entry.name, stream(entry)?)))
            .collect();
        assert_eq!(
            streams,
            [
                (
                    "imu_lsm6dso32",
                    Stream {
                        file: "imu_lsm6dso32.csv".to_string(),
                        fields: &IMU_FIELDS,
                        decimate_us: 0,
                        durable: false,
                    }
                ),
                (
                    "baro_icp10111",
                    Stream {
                        file: "baro_icp10111.csv".to_string(),
                        fields: &BAROMETER_FIELDS,
                        decimate_us: 0,
                        durable: false,
                    }
                ),
                (
                    "separation",
                    Stream {
                        file: "separation.csv".to_string(),
                        fields: &SEPARATION_FIELDS,
                        decimate_us: -1,
                        durable: true,
                    }
                ),
            ]
        );
    }

    #[test]
    fn the_columns_are_plan_b_s() {
        assert_eq!(IMU_FIELDS.join(";"), "ax;ay;az;gx;gy;gz;irq_runs");
        assert_eq!(BAROMETER_FIELDS.join(";"), "altitude;temperature;pressure;elevation");
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
                ("imu_lsm6dso32.csv".to_string(), 20_000),
                ("baro_icp10111.csv".to_string(), 0)
            ]
        );
    }

    #[test]
    fn a_number_is_its_window_in_microseconds() {
        assert_eq!(window(None), 0);
        assert_eq!(window(Some(json!(0))), 0);
        assert_eq!(window(Some(json!(0.0))), 0);
        assert_eq!(window(Some(json!(50))), 50_000);
        assert_eq!(window(Some(json!(50.0))), 50_000);
        assert_eq!(window(Some(json!(12.5))), 12_500);
        assert_eq!(window(Some(json!(-1))), -1);
        assert_eq!(window(Some(json!(-250.5))), -1);
    }

    #[test]
    fn a_window_rounds_saturates_and_never_falls_to_the_global() {
        assert_eq!(window(Some(json!(0.0625))), 63);
        assert_eq!(window(Some(json!(0.0624))), 62);
        assert_eq!(window(Some(json!(0.0001))), 1);
        assert_eq!(window(Some(json!(-0.0001))), -1);
        assert_eq!(window(Some(json!(2_147_483))), 2_147_483_000);
        assert_eq!(window(Some(json!(1e12))), i32::MAX);
        assert_eq!(window(Some(json!(u64::MAX))), i32::MAX);
    }

    #[test]
    fn the_barometer_takes_its_window_too() {
        let barometer = only(&sensor("baro_icp10111", "icp10111", Some(json!(100)))).expect("a stream");
        assert_eq!(
            (barometer.file.as_str(), barometer.decimate_us),
            ("baro_icp10111.csv", 100_000)
        );
    }

    #[test]
    fn separation_is_durable_never_decimated_and_always_separation_csv() {
        let document = json!({"components": [
            {"name": "pads", "activity": "separation", "telemetry_ms": 50},
        ]});
        assert_eq!(
            only(&document),
            Some(Stream {
                file: "separation.csv".to_string(),
                fields: &SEPARATION_FIELDS,
                decimate_us: -1,
                durable: true,
            })
        );
    }

    #[test]
    fn a_window_that_is_not_a_number_is_the_global() {
        for not_a_number in [
            json!("50"),
            Value::Null,
            json!(true),
            json!(false),
            json!([50]),
            json!({"ms": 50}),
        ] {
            assert_eq!(window(Some(not_a_number.clone())), 0, "{not_a_number}");
        }
    }

    #[test]
    fn a_part_the_port_does_not_record_has_no_stream() {
        assert_eq!(only(&sensor("imu_bmi323", "bmi323", Some(json!(10)))), None);
        assert_eq!(only(&sensor("baro_bmp581", "bmp581", None)), None);
        assert_eq!(only(&json!({"sensors": [{"name": "no_implementation"}]})), None);
        let document = defaults();
        let bmi323 = entries(&document)
            .into_iter()
            .find(|entry| entry.implementation == "bmi323")
            .expect("the defaults carry a BMI323");
        assert_eq!(stream(&bmi323), None);
    }
}
