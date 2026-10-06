//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight camera (components/coludo_video): OV5647 -> ISP -> H.264 hardware encoder -> raw Annex-B on the
//! microSD, every frame stamped with the boot id and its uptime in an SEI NAL. Files go to
//! `/sdcard/video/<session>/<start uptime ms>.h264`.

use esp_idf_svc::sys;

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
