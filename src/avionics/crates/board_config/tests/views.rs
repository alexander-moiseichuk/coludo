//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The typed view's M5 readers (the buses, the device directory, pins, `provides` and the setup attempts) over the
//! boards' own documents, and each reader's refusals; and the M8 readers (the checkpoint, the warm start's toggle and
//! the watchdog) over the same documents, whose refusals view.rs's own tests hold.
//!
//! Positive: the firmware's defaults (golden/config_default.json, plan B's config_default.py as the generator dumped
//! it) and the committed launches files of the two boards M5 meets on hardware: TMS-7F (v1.x wiring: i2c:1 at
//! 100 kHz, the laser pins still named, since layout has not run) and TMS-7C (v0.1: no `board.layout`, the BNO055
//! and BMP280 fitted). The expected directories and pins are what plan B's own `Controller.directory()` and
//! `Task._pin_gpio` answer over the same files on CPython.
//!
//! Negative: each way a document names no bus, no pin, no source or no retry count, read as plan B reads it.

use board_config::{
    Entry, I2cSpecification, Provide, Section, SpiSpecification, Watchdog, checkpoint, directory, i2c_bus, pin,
    provides, setup_retries, spi_bus, warm_start, watchdog,
};
use serde_json::{Value, json};

/// The firmware's defaults: golden/build_config_default.py's dump of plan B's config_default.py.
const CONFIG_DEFAULT: &str = include_str!("../../../golden/config_default.json");

/// TMS-7F's committed board file (a v1.x board, never flown).
const TMS7F: &str = include_str!("../../../../../launches/20261003/TMS-7F/tms7f.config");

/// TMS-7C's committed board file (the v0.1 board flown on 2026-10-03).
const TMS7C: &str = include_str!("../../../../../launches/20261003/TMS-7C/tms7c.config");

/// The defaults document (the fixture's `document`).
fn defaults() -> Value {
    let fixture: Value = serde_json::from_str(CONFIG_DEFAULT).expect("the fixture is JSON");
    fixture["document"].clone()
}

/// A committed board file, parsed.
fn board_file(text: &str) -> Value {
    serde_json::from_str(text).expect("a board file is JSON")
}

/// The names the directory lists, in its order.
fn names(document: &Value) -> Vec<&str> {
    directory(document).into_iter().map(|entry| entry.name).collect()
}

/// The entry named `name`, which the directory must list.
fn entry<'d>(document: &'d Value, name: &str) -> Entry<'d> {
    directory(document)
        .into_iter()
        .find(|entry| entry.name == name)
        .unwrap_or_else(|| panic!("the directory does not list {name}"))
}

/// The GPIO the entry named `name` gives for its pin `key`.
fn gpio(document: &Value, name: &str, key: &str) -> Option<u8> {
    pin(document, &entry(document, name), key)
}

/// The sources the entry named `name` registers.
fn sources<'d>(document: &'d Value, name: &str) -> Vec<Provide<'d>> {
    provides(&entry(document, name))
}

/// One source as [`provides`] lists it.
fn source<'d>(quantity: &'d str, priority: i32, timeout_ms: u32, reconcile: bool) -> Provide<'d> {
    Provide {
        quantity,
        priority,
        timeout_ms,
        reconcile,
    }
}

/// An I2C bus as [`i2c_bus`] reads it.
fn i2c(id: u8, sda: u8, scl: u8, clock_hz: u32) -> Option<I2cSpecification> {
    Some(I2cSpecification { id, sda, scl, clock_hz })
}

/// The board's one SPI bus, spi:1 on GPIO 48/47/46, at this clock and mode.
fn spi(clock_hz: u32, mode: u8) -> Option<SpiSpecification> {
    Some(SpiSpecification {
        id: 1,
        sck: 48,
        mosi: 47,
        miso: 46,
        clock_hz,
        mode,
    })
}

#[test]
fn the_defaults_buses_are_plan_bs() {
    let document = defaults();
    assert_eq!(i2c_bus(&document, 0), i2c(0, 7, 8, 400_000));
    assert_eq!(i2c_bus(&document, 1), i2c(1, 31, 30, 400_000));
    assert_eq!(spi_bus(&document, 1), spi(5_000_000, 3));
    // nothing else: no third I2C bus, and SPI's only id is 1 (a UART id is no I2C bus either)
    assert_eq!(i2c_bus(&document, 2), None);
    assert_eq!(spi_bus(&document, 0), None);
    assert_eq!(setup_retries(&document), 3);
}

#[test]
fn the_defaults_directory_is_plan_bs() {
    let document = defaults();
    // plan B's directory() over the same document: the disabled BNO055, BMP280, flight, watchdog and field are absent
    assert_eq!(
        names(&document),
        [
            "accel_adxl375",
            "imu_lsm6dso32",
            "attitude",
            "baro_icp10111",
            "imu_bmi323",
            "mag_bmm350",
            "baro_bmp581",
            "airspeed_sdp810",
            "laser_agl",
            "laser_agl_l1x",
            "power_ina226",
            "gnss",
            "recorder",
            "separation",
            "servo_yaw",
            "servo_eleron_left",
            "servo_eleron_right",
            "sequencer",
            "checkpoint",
            "gnss_calib",
            "health",
            "bluetooth",
            "wifi",
            "cc"
        ]
    );
    let sections: Vec<Section> = directory(&document).iter().map(|entry| entry.section).collect();
    assert_eq!(
        sections.iter().filter(|section| **section == Section::Sensors).count(),
        12
    );
    assert_eq!(sections[11..13], [Section::Sensors, Section::Components]);
}

#[test]
fn the_defaults_entries_carry_their_bus_fields_as_written() {
    let document = defaults();
    let baro = entry(&document, "baro_icp10111");
    assert_eq!(
        (
            baro.section,
            baro.implementation,
            baro.bus,
            baro.bus_id.as_deref(),
            baro.address
        ),
        (Section::Sensors, "icp10111", Some("i2c"), Some("0"), Some(0x63))
    );
    assert_eq!(baro.fields.get("driver"), Some(&json!("icp10111")));
    // an SPI entry keeps its fallback I2C address
    let imu = entry(&document, "imu_lsm6dso32");
    assert_eq!(
        (imu.implementation, imu.bus, imu.bus_id.as_deref(), imu.address),
        ("lsm6dso32", Some("spi"), Some("1"), Some(0x6a))
    );
    // an activity, and a UART entry, carry no address
    let attitude = entry(&document, "attitude");
    assert_eq!(
        (attitude.implementation, attitude.bus, attitude.bus_id, attitude.address),
        ("attitude", None, None, None)
    );
    let recorder = entry(&document, "recorder");
    assert_eq!(
        (
            recorder.section,
            recorder.bus,
            recorder.bus_id.as_deref(),
            recorder.address
        ),
        (Section::Components, Some("uart"), Some("1"), None)
    );
}

#[test]
fn the_defaults_pins_are_plan_bs() {
    let document = defaults();
    for (name, key, expected) in [
        ("accel_adxl375", "cs_pin", Some(49)),
        ("accel_adxl375", "int_pin", Some(4)),
        ("imu_lsm6dso32", "cs_pin", Some(50)),
        ("imu_lsm6dso32", "int_pin", Some(28)),
        ("imu_bmi323", "int_pin", None), // the entry's own null: no interrupt wired
        ("laser_agl", "xshut_pin", Some(5)),
        ("laser_agl", "int_pin", Some(3)),
        ("laser_agl_l1x", "xshut_pin", Some(5)),
        ("laser_agl_l1x", "int_pin", Some(3)),
        ("power_ina226", "alert_pin", Some(29)),
        ("separation", "pin", Some(33)),
        ("servo_yaw", "pin", Some(26)),
        ("servo_eleron_left", "pin", Some(27)),
        ("servo_eleron_right", "pin", Some(32)),
        ("baro_icp10111", "int_pin", None), // the entry has no such key
    ] {
        assert_eq!(gpio(&document, name, key), expected, "{name}.{key}");
    }
}

#[test]
fn the_defaults_sources_are_plan_bs() {
    let document = defaults();
    assert_eq!(
        sources(&document, "baro_icp10111"),
        [
            source("altitude", 0, 200, true),
            source("elevation", 0, 200, false),
            source("pressure", 0, 200, true),
            source("temperature", 0, 500, false)
        ]
    );
    // sorted by quantity (serde_json's Map already is; the explicit sort keeps it so if preserve_order is ever
    // unified in)
    assert_eq!(
        sources(&document, "gnss"),
        [
            source("altitude", 3, 2000, false),
            source("course", 0, 500, false),
            source("elevation", 3, 2000, false),
            source("position", 0, 200, false),
            source("speed", 0, 500, false)
        ]
    );
    assert_eq!(sources(&document, "laser_agl_l1x"), [source("agl", 0, 100, false)]);
    assert_eq!(sources(&document, "recorder"), []);
}

#[test]
fn tms7f_runs_i2c1_at_100_khz_with_the_laser_pins_named() {
    let document = board_file(TMS7F);
    assert_eq!(i2c_bus(&document, 0), i2c(0, 7, 8, 400_000));
    assert_eq!(i2c_bus(&document, 1), i2c(1, 31, 30, 100_000));
    assert_eq!(spi_bus(&document, 1), spi(5_000_000, 3));
    assert_eq!(setup_retries(&document), 3);
    // plan B's directory(): the ADXL375, the BNO055, the BMP280, the VL53L4CX, flight and field are disabled
    assert_eq!(
        names(&document),
        [
            "imu_lsm6dso32",
            "attitude",
            "baro_icp10111",
            "imu_bmi323",
            "mag_bmm350",
            "baro_bmp581",
            "airspeed_sdp810",
            "laser_agl_l1x",
            "power_ina226",
            "gnss",
            "recorder",
            "separation",
            "servo_yaw",
            "servo_eleron_left",
            "servo_eleron_right",
            "sequencer",
            "checkpoint",
            "gnss_calib",
            "watchdog",
            "health",
            "bluetooth",
            "wifi",
            "cc"
        ]
    );
    // the laser's pins resolve until layout nulls them for v1.x
    assert_eq!(gpio(&document, "laser_agl_l1x", "xshut_pin"), Some(5));
    assert_eq!(gpio(&document, "laser_agl_l1x", "int_pin"), Some(3));
    assert_eq!(gpio(&document, "imu_bmi323", "int_pin"), Some(4));
    for (name, address) in [
        ("baro_icp10111", 0x63),
        ("airspeed_sdp810", 0x25),
        ("laser_agl_l1x", 0x29),
    ] {
        let on_second_bus = entry(&document, name);
        assert_eq!(on_second_bus.bus_id.as_deref(), Some("1"), "{name}");
        assert_eq!(on_second_bus.address, Some(address), "{name}");
    }
    assert_eq!(entry(&document, "gnss").implementation, "neo6mv2");
}

#[test]
fn tms7c_is_the_v01_board_as_flown() {
    let document = board_file(TMS7C);
    assert_eq!(i2c_bus(&document, 0), i2c(0, 7, 8, 400_000));
    assert_eq!(i2c_bus(&document, 1), i2c(1, 31, 30, 400_000));
    assert_eq!(spi_bus(&document, 1), spi(5_000_000, 3));
    assert_eq!(setup_retries(&document), 3);
    // plan B's directory(): the INA226, the servos, flight and field are disabled
    assert_eq!(
        names(&document),
        [
            "accel_adxl375",
            "imu_lsm6dso32",
            "imu_bno055",
            "attitude",
            "baro_icp10111",
            "baro_bmp280",
            "airspeed_sdp810",
            "laser_agl",
            "gnss",
            "recorder",
            "separation",
            "sequencer",
            "checkpoint",
            "gnss_calib",
            "watchdog",
            "health",
            "bluetooth",
            "wifi",
            "cc"
        ]
    );
    // its own pin names: the ADXL375's interrupt is `adxl375_int`
    assert_eq!(gpio(&document, "accel_adxl375", "cs_pin"), Some(49));
    assert_eq!(gpio(&document, "accel_adxl375", "int_pin"), Some(4));
    assert_eq!(gpio(&document, "laser_agl", "xshut_pin"), Some(5));
    assert_eq!(gpio(&document, "separation", "pin"), Some(33));
    // the SEN0253 on i2c:0 and every I2C part on it
    for (name, address) in [
        ("imu_bno055", 0x28),
        ("baro_icp10111", 0x63),
        ("baro_bmp280", 0x76),
        ("airspeed_sdp810", 0x25),
        ("laser_agl", 0x29),
    ] {
        let on_first_bus = entry(&document, name);
        assert_eq!(
            (on_first_bus.bus, on_first_bus.bus_id.as_deref(), on_first_bus.address),
            (Some("i2c"), Some("0"), Some(address)),
            "{name}"
        );
    }
    assert_eq!(
        sources(&document, "imu_bno055"),
        [source("accel", 2, 40, false), source("attitude", 0, 40, false)]
    );
}

#[test]
fn a_bus_that_cannot_be_built_is_none() {
    let with = |i2c: Value| json!({"buses": {"i2c": i2c}});
    assert_eq!(
        i2c_bus(&with(json!({"0": {"sda": 7, "scl": 8}})), 0),
        i2c(0, 7, 8, 400_000)
    );
    assert_eq!(i2c_bus(&with(json!({"0": {"sda": 7}})), 0), None, "no scl");
    assert_eq!(i2c_bus(&with(json!({"0": {"scl": 8}})), 0), None, "no sda");
    assert_eq!(i2c_bus(&with(json!({"0": null})), 0), None, "a null bus");
    assert_eq!(i2c_bus(&with(json!({"0": [7, 8]})), 0), None, "a bus that is no object");
    assert_eq!(
        i2c_bus(&with(json!({"0": {"sda": 7, "scl": 8}})), 7),
        None,
        "bus id 7 is not defined"
    );
    for document in [
        json!({"buses": {"i2c": null}}),
        json!({"buses": {"i2c": [{"sda": 7, "scl": 8}]}}),
        json!({"buses": null}),
        json!({}),
        json!([]),
    ] {
        assert_eq!(i2c_bus(&document, 0), None, "{document}");
    }
    // a pin that is no GPIO loses the bus, as plan B's Pin() raises on it
    for sda in [json!(-1), json!(7.0), json!("7"), json!(null), json!(256), json!([7])] {
        assert_eq!(
            i2c_bus(&with(json!({"0": {"sda": sda, "scl": 8}})), 0),
            None,
            "sda {sda}"
        );
    }
    // ...but a bool is Python's int: GPIO 1
    assert_eq!(
        i2c_bus(&with(json!({"0": {"sda": true, "scl": 8}})), 0).map(|bus| bus.sda),
        Some(1)
    );
    // each of SPI's three pins is required, and an I2C bus is no SPI bus
    let three = json!({"sck": 48, "mosi": 47, "miso": 46});
    for missing in ["sck", "mosi", "miso"] {
        let mut specification = three.clone();
        specification.as_object_mut().unwrap().remove(missing);
        assert_eq!(
            spi_bus(&json!({"buses": {"spi": {"1": specification}}}), 1),
            None,
            "no {missing}"
        );
    }
    assert_eq!(spi_bus(&json!({"buses": {"spi": {"1": three}}}), 1), spi(5_000_000, 3));
    assert_eq!(
        spi_bus(&with(json!({"1": {"sck": 48, "mosi": 47, "miso": 46}})), 1),
        None
    );
}

#[test]
fn clocks_read_as_whole_hertz_or_the_default() {
    let clock = |clock: Value| {
        let document = json!({"buses": {"i2c": {"0": {"sda": 7, "scl": 8, "freq": clock}}}});
        i2c_bus(&document, 0).map(|bus| bus.clock_hz)
    };
    assert_eq!(clock(json!(100_000)), Some(100_000));
    assert_eq!(clock(json!(1_000_000)), Some(1_000_000));
    assert_eq!(
        clock(json!(100_000.0)),
        Some(100_000),
        "a whole float is the typed view's integer knob"
    );
    assert_eq!(clock(json!(1)), Some(1));
    assert_eq!(clock(json!(u32::MAX)), Some(u32::MAX));
    for refused in [
        json!(null),
        json!("100000"),
        json!(true),
        json!(0),
        json!(-100_000),
        json!(100_000.5),
        json!(u64::from(u32::MAX) + 1),
    ] {
        assert_eq!(clock(refused.clone()), Some(400_000), "freq {refused}");
    }
    let baud = |baud: Option<Value>| {
        let mut document = json!({"buses": {"spi": {"1": {"sck": 48, "mosi": 47, "miso": 46}}}});
        if let Some(baud) = baud {
            document["buses"]["spi"]["1"]["baud"] = baud;
        }
        spi_bus(&document, 1).map(|bus| bus.clock_hz)
    };
    assert_eq!(baud(None), Some(5_000_000));
    assert_eq!(baud(Some(json!(10_000_000))), Some(10_000_000));
    assert_eq!(baud(Some(json!("fast"))), Some(5_000_000));
}

#[test]
fn the_spi_mode_is_the_one_validate_admits_or_3() {
    let mode = |mode: Value| {
        let document = json!({"buses": {"spi": {"1": {"sck": 48, "mosi": 47, "miso": 46, "mode": mode}}}});
        spi_bus(&document, 1).map(|bus| bus.mode)
    };
    for (written, read) in [
        (json!(0), 0),
        (json!(1), 1),
        (json!(2), 2),
        (json!(3), 3),
        (json!(2.0), 2),
        (json!(-0.0), 0),
        (json!(true), 1),
        (json!(false), 0),
    ] {
        assert_eq!(mode(written.clone()), Some(read), "mode {written}");
    }
    for refused in [json!(4), json!(-1), json!(1.5), json!("2"), json!(null), json!([2])] {
        assert_eq!(mode(refused.clone()), Some(3), "mode {refused}");
    }
}

#[test]
fn an_id_given_as_an_int_or_as_text_resolves_to_the_same_bus() {
    let document = json!({
        "buses": {"i2c": {"1": {"sda": 31, "scl": 30, "freq": 100_000}}},
        "sensors": [
            {"name": "as_int", "driver": "icp10111", "bus": "i2c", "id": 1, "addr": 99},
            {"name": "as_text", "driver": "icp10111", "bus": "i2c", "id": "1", "addr": 99},
            {"name": "as_float", "driver": "icp10111", "bus": "i2c", "id": 1.0, "addr": 99},
            {"name": "as_bool", "driver": "icp10111", "bus": "i2c", "id": true, "addr": 99},
            {"name": "as_null", "driver": "icp10111", "bus": "i2c", "id": null, "addr": 99},
            {"name": "absent", "driver": "icp10111", "bus": "i2c", "addr": 99}
        ]
    });
    let identifiers: Vec<Option<String>> = directory(&document).into_iter().map(|entry| entry.bus_id).collect();
    let text = |id: &str| Some(id.to_string());
    // Python's str(): the int and the text are one id, a float and a bool are other texts
    assert_eq!(
        identifiers,
        [text("1"), text("1"), text("1.0"), text("True"), None, None]
    );
    // the id's text is the bus key, as config.bus looks it up
    let resolved = |name: &str| {
        let id = entry(&document, name).bus_id?;
        i2c_bus(&document, id.parse().ok()?)
    };
    assert_eq!(resolved("as_int"), i2c(1, 31, 30, 100_000));
    assert_eq!(resolved("as_text"), resolved("as_int"));
    for unresolved in ["as_float", "as_bool", "as_null", "absent"] {
        assert_eq!(resolved(unresolved), None, "{unresolved}");
    }
}

#[test]
fn a_pin_wired_off_absent_or_unknown_is_none() {
    let document = json!({
        "pins": {"wired": 26, "zero": 0, "off_by_null": null, "off_by_negative": -1, "flag": true, "whole": 26.0,
                 "past": 300, "text": "26"},
        "components": [{"name": "servo", "driver": "sg90", "pin": "wired", "zero_pin": "zero",
                        "null_pin": "off_by_null", "negative_pin": "off_by_negative", "unknown_pin": "not_in_the_map",
                        "entry_null_pin": null, "number_pin": 26, "list_pin": ["wired"], "flag_pin": "flag",
                        "whole_pin": "whole", "past_pin": "past", "text_pin": "text"}]
    });
    let servo = entry(&document, "servo");
    let read = |key: &str| pin(&document, &servo, key);
    assert_eq!(read("pin"), Some(26));
    assert_eq!(read("zero_pin"), Some(0), "GPIO0 is a pin");
    assert_eq!(read("flag_pin"), Some(1), "a bool is Python's int");
    for (key, why) in [
        ("null_pin", "the map wires it off with null"),
        ("negative_pin", "the map wires it off with -1"),
        ("absent_pin", "the entry has no such key"),
        ("unknown_pin", "the map has no such name"),
        ("entry_null_pin", "the entry names no pin"),
        ("number_pin", "the entry names a pin by its name, never its number"),
        ("list_pin", "a list names no pin"),
        ("whole_pin", "a float is no int"),
        ("past_pin", "no GPIO past 255"),
        ("text_pin", "a text number is no int"),
    ] {
        assert_eq!(read(key), None, "{key}: {why}");
    }
    // no pins map, or one that is no object
    for pins in [None, Some(json!(null)), Some(json!([26]))] {
        let mut without = json!({"components": [{"name": "servo", "pin": "wired"}]});
        if let Some(pins) = pins {
            without["pins"] = pins;
        }
        assert_eq!(pin(&without, &entry(&without, "servo"), "pin"), None, "{without}");
    }
}

#[test]
fn no_provides_is_no_source_and_a_missing_number_is_0() {
    let document = json!({"sensors": [
        {"name": "none", "driver": "x"},
        {"name": "null", "driver": "x", "provides": null},
        {"name": "list", "driver": "x", "provides": ["agl"]},
        {"name": "empty", "driver": "x", "provides": {}},
        {"name": "bare", "driver": "x", "provides": {"agl": {}}},
        {"name": "odd", "driver": "x", "provides": {
            "z": {"priority": 2, "timeout_ms": 50, "reconcile": 1},
            "agl": 5,
            "a": {"priority": true, "timeout_ms": "100", "reconcile": 0}}},
        {"name": "wide", "driver": "x", "provides": {
            "far": {"priority": 1_u64 << 40, "timeout_ms": 1_u64 << 40},
            "near": {"priority": -(1_i64 << 40), "timeout_ms": -5, "reconcile": []},
            "whole": {"priority": 1.0, "timeout_ms": 200.0, "reconcile": "yes"}}}
    ]});
    for name in ["none", "null", "list", "empty"] {
        assert_eq!(sources(&document, name), [], "{name}");
    }
    assert_eq!(sources(&document, "bare"), [source("agl", 0, 0, false)]);
    // a spec that is no object is left out; a bool is Python's int, text is no number, reconcile is truthiness
    assert_eq!(
        sources(&document, "odd"),
        [source("a", 1, 0, false), source("z", 2, 50, true)]
    );
    // held within the databoard's types; a float is no int
    assert_eq!(
        sources(&document, "wide"),
        [
            source("far", i32::MAX, u32::MAX, false),
            source("near", i32::MIN, 0, false),
            source("whole", 0, 0, true)
        ]
    );
}

#[test]
fn setup_retries_reads_as_plan_b() {
    let with = |retries: Value| json!({"board": {"setup_retries": retries}});
    assert_eq!(setup_retries(&with(json!(3))), 3);
    assert_eq!(setup_retries(&with(json!(1))), 1);
    assert_eq!(setup_retries(&with(json!(u64::MAX))), u32::MAX, "saturates");
    for one in [
        json!(0),
        json!(-2),
        json!("3"),
        json!(3.0),
        json!(null),
        json!(true),
        json!(false),
        json!([3]),
    ] {
        assert_eq!(setup_retries(&with(one.clone())), 1, "setup_retries {one}");
    }
    for document in [
        json!({"board": {}}),
        json!({}),
        json!({"board": null}),
        json!({"board": [3]}),
        json!(null),
    ] {
        assert_eq!(setup_retries(&document), 1, "{document}");
    }
}

#[test]
fn the_directory_lists_enabled_named_objects_sensors_first() {
    let document = json!({
        "components": [{"name": "c_first", "activity": "cc"}, {"name": "c_off", "activity": "cc", "enabled": false}],
        "sensors": [
            {"name": "s_absent", "driver": "a"},
            {"name": "s_true", "driver": "a", "enabled": true},
            {"name": "s_false", "driver": "a", "enabled": false},
            {"name": "s_zero", "driver": "a", "enabled": 0},
            {"name": "s_null", "driver": "a", "enabled": null},
            {"name": "s_text", "driver": "a", "enabled": "yes"},
            {"name": "", "driver": "a"},
            {"driver": "unnamed"},
            {"name": 5, "driver": "a"},
            "not_an_object",
            7,
            null
        ]
    });
    // sensors first although the document's keys sort components first; `enabled` by truthiness
    assert_eq!(names(&document), ["s_absent", "s_true", "s_text", "c_first"]);
    let sections: Vec<Section> = directory(&document).iter().map(|entry| entry.section).collect();
    assert_eq!(
        sections,
        [
            Section::Sensors,
            Section::Sensors,
            Section::Sensors,
            Section::Components
        ]
    );
    // a list that is not a list holds nothing, and a document that is no object has no devices
    assert_eq!(
        names(&json!({"sensors": {"name": "x"}, "components": [{"name": "c"}]})),
        ["c"]
    );
    assert_eq!(names(&json!({"sensors": null})), Vec::<&str>::new());
    assert_eq!(names(&json!([{"name": "x"}])), Vec::<&str>::new());
}

#[test]
fn the_implementation_is_the_driver_or_else_the_activity() {
    let document = json!({"sensors": [
        {"name": "driver", "driver": "icp10111", "activity": "attitude"},
        {"name": "activity", "activity": "attitude"},
        {"name": "empty_driver", "driver": "", "activity": "attitude"},
        {"name": "null_driver", "driver": null, "activity": "attitude"},
        {"name": "number_driver", "driver": 5, "activity": "attitude"},
        {"name": "neither"}
    ]});
    let implementations: Vec<&str> = directory(&document)
        .into_iter()
        .map(|entry| entry.implementation)
        .collect();
    // a truthy driver that is no text wins, and names no implementation
    assert_eq!(
        implementations,
        ["icp10111", "attitude", "attitude", "attitude", "", ""]
    );
}

#[test]
fn an_address_is_a_seven_bit_int_and_a_bus_is_text() {
    let addresses = [
        json!(0x29),
        json!(0),
        json!(0x7f),
        json!(true),
        json!(0x80),
        json!(300),
        json!(-1),
        json!("0x29"),
        json!(41.0),
        json!(null),
    ];
    let sensors: Vec<Value> = addresses
        .iter()
        .enumerate()
        .map(|(index, address)| json!({"name": format!("s{index}"), "driver": "x", "bus": "i2c", "addr": address}))
        .collect();
    let document = json!({"sensors": sensors});
    let read: Vec<Option<u8>> = directory(&document).into_iter().map(|entry| entry.address).collect();
    assert_eq!(
        read,
        [
            Some(0x29),
            Some(0),
            Some(0x7f),
            Some(1),
            None,
            None,
            None,
            None,
            None,
            None
        ]
    );
    // the entry still holds what it wrote, for a caller that must tell an unusable address from an absent one
    assert_eq!(entry(&document, "s5").fields.get("addr"), Some(&json!(300)));
    assert_eq!(entry(&document, "s9").fields.get("addr"), Some(&Value::Null));
    let buses = json!({"sensors": [
        {"name": "text", "driver": "x", "bus": "spi"},
        {"name": "number", "driver": "x", "bus": 5},
        {"name": "null", "driver": "x", "bus": null},
        {"name": "absent", "driver": "x"}
    ]});
    let kinds: Vec<Option<&str>> = directory(&buses).into_iter().map(|entry| entry.bus).collect();
    assert_eq!(kinds, [Some("spi"), None, None, None]);
}

#[test]
fn entries_keep_the_disabled_ones_the_directory_drops() {
    let document = json!({
        "sensors": [{"name": "imu_bno055", "driver": "bno055", "enabled": false}, {"name": "baro", "driver": "icp10111"}],
        "components": [{"name": "cc", "activity": "cc"}, {"driver": "unnamed"}],
    });
    let entries: Vec<&str> = board_config::entries(&document)
        .iter()
        .map(|entry| entry.name)
        .collect();
    let directory: Vec<&str> = board_config::directory(&document)
        .iter()
        .map(|entry| entry.name)
        .collect();
    assert_eq!(entries, ["imu_bno055", "baro", "cc"]);
    assert_eq!(directory, ["baro", "cc"]);
}

#[test]
fn the_defaults_checkpoint_every_second_with_the_watchdog_off() {
    let document = defaults();
    assert_eq!(checkpoint(&document), Some(Ok(1.0)));
    assert!(warm_start(&document));
    // plan B ships the watchdog disabled: the task watchdog keeps IDF's warn-only default
    assert_eq!(watchdog(&document), None);
}

#[test]
fn the_boards_files_arm_the_watchdog_at_plan_bs_settings() {
    let expected = Watchdog {
        timeout_ms: 5000,
        period_ms: 200,
        stall_ms: 500.0,
    };
    for (board, text) in [("TMS-7F", TMS7F), ("TMS-7C", TMS7C)] {
        let document = board_file(text);
        assert_eq!(checkpoint(&document), Some(Ok(1.0)), "{board}");
        assert!(warm_start(&document), "{board}");
        assert_eq!(watchdog(&document), Some(Ok(expected)), "{board}");
    }
}
