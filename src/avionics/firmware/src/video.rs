//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight camera (components/coludo_video): OV5647 -> ISP -> H.264 hardware encoder -> raw Annex-B on the
//! microSD, every frame stamped with the boot id and its uptime in an SEI NAL. Files go to
//! `/sdcard/video/<session>/<start uptime ms>.h264`.

use esp_idf_svc::sys;

use crate::gcl::{self, Command, Gate, Kind, Parameter, Reach, Word};
use crate::{i2c, sd};

/// The OV5647's Bayer order as the ISP must be told it: BGGR. esp_cam_sensor's tables run the readout mirrored
/// but declare GBRG, which swapped green and magenta; BGGR (what Linux's ov5647 driver declares for the same
/// mirrored modes) gives the colour bars exactly: white, yellow, cyan, green, magenta, red, blue, black.
pub const OV5647_BAYER: i8 = 3;

/// The taster's OV5647 module runs on a 24 MHz oscillator: every frame rate came out 24/25 of the request in the
/// SEI frame stamps (the stock tables assume 25 MHz). Check each module (the Arducam B012V1HEP4 when it arrives).
pub const OV5647_CLOCK_HZ: u32 = 24_000_000;

/// What to record.
pub struct Settings {
    /// Sensor mode: 1920x1080 (crop) or 1280x960 (binned, full field of view).
    pub width: u16,
    pub height: u16,
    /// Frames per second (at most the mode's native 30 / 45).
    pub fps: u8,
    /// Target bit rate in kbit/s.
    pub kilobits: u32,
    /// The sensor's colour bars instead of the image: proves the pipeline without light.
    pub test_pattern: bool,
    /// Bayer order the ISP is told (0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR); -1 keeps the driver's.
    pub bayer: i8,
    /// The sensor AEC's target: mean luma of the linear image, 8-bit (the ISP's gamma curve brightens it after).
    pub exposure_target: u8,
}

/// Start recording; returns the file's path.
pub fn start(session: &str, boot_id: u32, settings: &Settings) -> anyhow::Result<String> {
    let uptime = unsafe { sys::esp_timer_get_time() } / 1000;
    let directory = format!("{}/video/{session}", sd::MOUNT);
    std::fs::create_dir_all(&directory)?;
    let path = format!("{directory}/{uptime}.h264");
    let c_path = std::ffi::CString::new(path.clone())?;
    let config = sys::coludo_video_config_t {
        i2c_bus: i2c::port0()?,
        width: settings.width,
        height: settings.height,
        fps: settings.fps,
        gop: settings.fps, // one IDR a second: a crash or a cut file decodes from the last second on
        qp_min: 20,
        qp_max: 42,
        bitrate: settings.kilobits * 1000,
        frame_bytes: 1 << 20, // an IDR at 1080p and a high bit rate fits with room
        ring_bytes: 2 << 20,  // ~2 s at 8 Mbit/s: rides out the card's 0.5 s write stalls
        boot_id,
        encode_core: 0, // the flight loop owns core 1
        encode_priority: 10,
        write_priority: 6,
        path: c_path.as_ptr(),
        test_pattern: settings.test_pattern as u8,
        bayer: settings.bayer,
        sensor_clock_hz: OV5647_CLOCK_HZ,
        exposure_target: settings.exposure_target,
    };
    // SAFETY: the config and the path outlive the call (the recorder copies what it keeps).
    sys::esp!(unsafe { sys::coludo_video_start(&config) })?;
    Ok(path)
}

/// Stop: flush, fsync, close.
pub fn stop() -> anyhow::Result<()> {
    // SAFETY: stops the running recorder (an error when none runs).
    sys::esp!(unsafe { sys::coludo_video_stop() })?;
    sd::measure_free();
    Ok(())
}

/// The counters of the running (or last) recording.
pub fn stats() -> sys::coludo_video_stats_t {
    let mut stats = sys::coludo_video_stats_t::default();
    // SAFETY: copies the counters into a live struct.
    unsafe { sys::coludo_video_stats(&mut stats) };
    stats
}

/// The recorder's parameters (`video start`): the flight setting by default, FHD@25 at 8 Mbit/s.
const START_PARAMETERS: &[Parameter] = &[
    Parameter {
        name: "width",
        kind: Kind::Integer { low: 1280, high: 1920 },
        default: Some("1920"),
        about: "sensor mode width: 1920 (1080p crop) or 1280 (960p binned, the full field of view)",
    },
    Parameter {
        name: "height",
        kind: Kind::Integer { low: 960, high: 1080 },
        default: Some("1080"),
        about: "sensor mode height: 1080 or 960",
    },
    Parameter {
        name: "fps",
        kind: Kind::Integer { low: 1, high: 45 },
        default: Some("25"),
        about: "frame rate; 25 is the flight setting (the encoder tops out near 26.6 at 1080p on rev v1.3)",
    },
    Parameter {
        name: "kbps",
        kind: Kind::Integer { low: 500, high: 20_000 },
        default: Some("8000"),
        about: "target bit rate, kbit/s",
    },
    Parameter {
        name: "bayer",
        kind: Kind::Integer { low: 0, high: 3 },
        default: Some("3"),
        about: "the Bayer order the ISP is told, 0 RGGB .. 3 BGGR (the OV5647's, see OV5647_BAYER)",
    },
    Parameter {
        name: "ae",
        kind: Kind::Integer { low: 16, high: 160 },
        default: Some("48"),
        about: "auto-exposure target, mean linear luma 0..255 (BT.709 gamma makes 48 a mid-grey ~107); stock 84 clips whites",
    },
];

/// The camera's console commands (the Glider Command Language, crate::gcl), registered when the card mounts:
/// recording needs it. `session` and `boot_id` name and stamp the files this boot writes.
pub fn register_commands(session: String, boot_id: u32) {
    gcl::register(vec![
        Command {
            owner: "camera",
            words: "video start",
            about: "record to /sdcard/video/<session>/<start uptime ms>.h264, every frame stamped (tools/video_sei.py)",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: START_PARAMETERS,
            bare_words: &[Word {
                name: "bars",
                about: "the sensor's colour-bar test pattern instead of the image: proves the pipeline in the dark",
            }],
            run: Box::new(move |arguments| {
                let settings = Settings {
                    width: arguments.integer("width") as u16,
                    height: arguments.integer("height") as u16,
                    fps: arguments.integer("fps") as u8,
                    kilobits: arguments.integer("kbps") as u32,
                    test_pattern: arguments.word("bars"),
                    bayer: arguments.integer("bayer") as i8,
                    exposure_target: arguments.integer("ae") as u8,
                };
                match start(&session, boot_id, &settings) {
                    Ok(path) => cc_protocol::build("ok", &["video", &path], &[]),
                    Err(error) => cc_protocol::build("err", &["video", &error.to_string()], &[]),
                }
            }),
        },
        Command {
            owner: "camera",
            words: "video stop",
            about: "stop recording: flush, fsync and close the file; the counters stay readable",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[],
            run: Box::new(|_| match stop() {
                Ok(()) => counters("stopped"),
                Err(error) => cc_protocol::build("err", &["video", &error.to_string()], &[]),
            }),
        },
        Command {
            owner: "camera",
            words: "video stats",
            about: "the recorder's counters: frames captured, encoded and dropped, encode and write times",
            gate: Gate::Open,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[],
            run: Box::new(|_| counters("running")),
        },
    ]);
}

/// The recorder's counters as one `ok video <state> name=value...` line. `sd_busy_pct` is the share of the
/// recording's wall time the writer spent inside write() and fsync() -- the card's utilisation by video.
fn counters(state: &str) -> String {
    let counters = stats();
    let average = counters.encode_microseconds_total / counters.frames_encoded.max(1) as u64;
    let end = match counters.stopped_microseconds {
        0 => unsafe { sys::esp_timer_get_time() }, // SAFETY: reads the monotonic timer
        stopped => stopped,
    };
    let elapsed = match counters.started_microseconds {
        0 => 0,
        started => (end - started).max(0) as u64,
    };
    // no span (a start that failed, or none yet): the writer's close of the empty file is no load figure
    let (busy_percent, bytes_per_second) = match elapsed {
        0 => (0.0, 0),
        span => (
            counters.write_microseconds_total as f32 * 100.0 / span as f32,
            counters.bytes_written * 1_000_000 / span,
        ),
    };
    let values = [
        ("captured", counters.frames_captured.to_string()),
        ("missed", counters.frames_missed.to_string()),
        ("encoded", counters.frames_encoded.to_string()),
        ("idr", counters.frames_idr.to_string()),
        ("dropped", counters.frames_dropped.to_string()),
        ("encode_errors", counters.encode_errors.to_string()),
        ("timeouts", counters.capture_timeouts.to_string()),
        ("write_errors", counters.write_errors.to_string()),
        ("written", counters.bytes_written.to_string()),
        ("encode_us_avg", average.to_string()),
        ("encode_us_max", counters.encode_microseconds_max.to_string()),
        ("write_us_max", counters.write_microseconds_max.to_string()),
        ("ring_max", counters.ring_used_max.to_string()),
        ("elapsed_ms", (elapsed / 1000).to_string()),
        ("sd_bytes_per_s", bytes_per_second.to_string()),
        ("sd_busy_pct", format!("{busy_percent:.1}")),
        // the grey-world white-balance trim over the sensor's own AWB (1.000 = none yet)
        (
            "red_gain",
            format!("{:.3}", counters.red_gain_thousandths as f32 / 1000.0),
        ),
        (
            "blue_gain",
            format!("{:.3}", counters.blue_gain_thousandths as f32 / 1000.0),
        ),
        ("balance_errors", counters.balance_errors.to_string()),
    ];
    let named: Vec<(&str, &str)> = values.iter().map(|(key, value)| (*key, value.as_str())).collect();
    cc_protocol::build("ok", &["video", state], &named)
}
