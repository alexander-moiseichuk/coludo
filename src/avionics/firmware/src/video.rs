//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight camera (components/coludo_video): OV5647 -> ISP -> H.264 hardware encoder -> raw Annex-B on the
//! microSD, every frame stamped with the boot id and its uptime in an SEI NAL. Files go to
//! `/sdcard/<session>/videos/<start uptime ms>.h264` (a two-partition card's to `/sdcard/videos/<session>/`, its
//! second partition), and on the pad a recording moves on to a new file every
//! [`Settings::segment_seconds`] (300 s for the flight's), bounding what one damaged file can cost; the file running
//! when the board leaves SETTING runs on to the end (M7b, coludo_video.c). The directories are crates/recorder's
//! layout's (`directory`, `videos_partition_directory`). A two-partition card whose videos partition is not mounted
//! (it would not mount, or the table is unsound) records no video at all, rather than fill the recordings partition
//! the ring_saver needs (sd::Layout::VideosFailed).
//!
//! **One owner, the `video` thread** (core 0, low priority, PSRAM stack), started only when the board config declares
//! a camera and the card mounts ([`start`]). It is the only caller of `coludo_video_start` and `coludo_video_stop`,
//! whose C side assumes one caller (its starts and stops never overlap). It looks at the flight word every
//! [`LOOK_PERIOD`] and follows the camera-by-stage rules (crates/sequencer `video`): armed in SETTING, the airframe
//! standing nose up on the rod for 2 s (the fused accel's +x, M9) starts the flight recording, replacing an
//! operator's -- at the arm itself on a board with no accelerometer; a disarm in SETTING stops it; DONE stops
//! whatever records; nothing starts outside SETTING. The GCL `video start` and `video stop` post to its mailbox and wait for the answer at most
//! [`ANSWER_DEADLINE`], so a recorder call that never returns cannot hold the CC executor. `video stats` only copies
//! the counters, on the executor. Video is a utility: none of this blocks `arm` or takes anything from the flight.

use std::sync::mpsc::{Receiver, RecvTimeoutError, SendError, SyncSender, sync_channel};
use std::time::{Duration, Instant};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::sys;
use recorder::layout;
use sequencer::video::{Duty, Owner, Refusal, Trigger};
use sequencer::{FlightWord, Stage};

use crate::buses::i2c;
use crate::gcl::{self, Command, Gate, Kind, Parameter, Reach, Word};
use crate::sd;

/// The OV5647's Bayer order as the ISP must be told it: BGGR. esp_cam_sensor's tables run the readout mirrored
/// but declare GBRG, which swapped green and magenta; BGGR (what Linux's ov5647 driver declares for the same
/// mirrored modes) gives the colour bars exactly: white, yellow, cyan, green, magenta, red, blue, black.
pub const OV5647_BAYER: i8 = 3;

/// The oscillator esp_cam_sensor's OV5647 tables assume (the Raspberry-Pi reference module's), used when the board
/// config's camera gives no usable `clock_hz`. Modules differ: the taster's runs at 24 MHz and 7F's Arducam at 27
/// MHz, and a wrong clock stretches every frame rate by its ratio (measured from the SEI frame stamps,
/// tools/video_sei.py), so each board's config declares its own (board_config::Camera).
pub const STOCK_CLOCK_HZ: u32 = 25_000_000;

/// The board's camera as the board config declares it (board_config::camera), resolved for the driver: its
/// oscillator, the stock one when the config gives none (logged).
#[derive(Clone, Copy, Debug)]
pub struct Camera {
    /// The module's oscillator in Hz.
    pub clock_hz: u32,
}

impl Camera {
    /// The camera the config declares, or None: no camera, no video (and no encoder reservation).
    pub fn from_config(document: &serde_json::Value) -> Option<Self> {
        let declared = board_config::camera(document)?;
        let stock = STOCK_CLOCK_HZ / 1_000_000;
        let clock_hz = match declared.clock {
            board_config::CameraClock::Hertz(hertz) => hertz,
            board_config::CameraClock::Missing => {
                log::warn!(
                    "video :: the camera entry gives no clock_hz: assuming the stock {stock} MHz, so frame rates may \
                     be off by the module's ratio"
                );
                STOCK_CLOCK_HZ
            }
            board_config::CameraClock::Unusable(written) => {
                log::warn!(
                    "video :: the camera's clock_hz {written} is not a whole number of Hz in 6-27 MHz: assuming the \
                     stock {stock} MHz, so frame rates may be off by the module's ratio"
                );
                STOCK_CLOCK_HZ
            }
        };
        Some(Self { clock_hz })
    }
}

/// What to record.
pub struct Settings {
    /// Sensor mode width: 1920 (1080p crop) or 1280 (960p binned, the full field of view).
    pub width: u16,
    /// Sensor mode height: 1080 or 960.
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
    /// On the pad, a new file at the first IDR this many seconds into a file; 0 keeps one file.
    pub segment_seconds: u16,
}

/// The flight setting, FHD@25 at 8 Mbit/s: what boot reserves the encoder for, and `video start`'s defaults
/// (START_PARAMETERS mirrors it).
pub const FLIGHT: Settings = Settings {
    width: 1920,
    height: 1080,
    fps: 25,
    kilobits: 8000,
    test_pattern: false,
    bayer: OV5647_BAYER,
    exposure_target: 48,
    segment_seconds: 300,
};

/// Reserves the encoder for the flight setting at boot, while the internal heap is still whole (coludo_video.h,
/// coludo_video_reserve): call it on the main task (core 0, where the encode task runs) before the ring threads start.
pub fn reserve() -> anyhow::Result<()> {
    // the oscillator is the camera half's (0, the stock one): the encoder half never reads it
    let config = configure(&FLIGHT, 0, 0, std::ptr::null_mut(), std::ptr::null(), std::ptr::null());
    // SAFETY: the config outlives the call; only its encoder fields are read.
    sys::esp!(unsafe { sys::coludo_video_reserve(&config) })?;
    Ok(())
}

/// The recorder's configuration for these settings (the C side copies what it keeps).
fn configure(
    settings: &Settings,
    clock_hz: u32,
    boot_id: u32,
    i2c_bus: sys::i2c_master_bus_handle_t,
    path: *const std::ffi::c_char,
    flight_word: *const u32,
) -> sys::coludo_video_config_t {
    sys::coludo_video_config_t {
        i2c_bus,
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
        path,
        test_pattern: settings.test_pattern as u8,
        bayer: settings.bayer,
        sensor_clock_hz: clock_hz,
        exposure_target: settings.exposure_target,
        segment_seconds: settings.segment_seconds,
        // the pad's camera aim (M7b): a JPEG a second while it records in SETTING, `GET /camera/preview.jpg`
        preview: 1,
        flight_word,
    }
}

/// The largest preview JPEG [`preview`] takes (720x384 at quality 60 is ~30-60 KB).
const PREVIEW_BYTES: usize = 160 << 10;

/// The latest preview JPEG and its frame's uptime in us (M7b), None before the first: one is made a second while the
/// camera records on the pad.
pub fn preview() -> Option<(Vec<u8>, i64)> {
    let mut jpeg = vec![0u8; PREVIEW_BYTES];
    let mut uptime = 0i64;
    // SAFETY: a live buffer of PREVIEW_BYTES and a live i64; the copy is made under the recorder's preview lock.
    let size = unsafe { sys::coludo_video_preview(jpeg.as_mut_ptr(), jpeg.len(), std::ptr::null_mut(), &mut uptime) };
    (size > 0).then(|| {
        jpeg.truncate(size);
        (jpeg, uptime)
    })
}

/// Start recording; returns the first file's path. Only the video thread calls it.
///
/// Args:
/// * `flight` - the flight word the recorder's segment cuts follow (none once the board leaves SETTING).
///
/// Errors: the card's videos partition is not mounted (no video then, see the module notes), the directory would not
/// be made, or the recorder would not start.
fn start_recording(
    session: &str,
    boot_id: u32,
    camera: Camera,
    settings: &Settings,
    flight: &'static FlightWord,
) -> anyhow::Result<String> {
    anyhow::ensure!(
        sd::layout() != Some(sd::Layout::VideosFailed),
        "the card's videos partition is not mounted: no video, so the recordings partition stays the ring_saver's"
    );
    // SAFETY: a plain read of the monotonic timer.
    let uptime = unsafe { sys::esp_timer_get_time() } / 1000;
    // session first (one partition, the layout); a two-partition card's videos go to its second partition
    let relative = if sd::videos_mounted() {
        layout::videos_partition_directory(session)
    } else {
        layout::directory(session, layout::VIDEOS)
    };
    let directory = format!("{}/{relative}", sd::MOUNT);
    std::fs::create_dir_all(&directory)?;
    let path = format!("{directory}/{uptime}.h264");
    let c_path = std::ffi::CString::new(path.clone())?;
    let config = configure(
        settings,
        camera.clock_hz,
        boot_id,
        i2c::port0()?,
        c_path.as_ptr(),
        flight.as_ptr(),
    );
    // SAFETY: the config and the path outlive the call (the recorder copies what it keeps), the flight word is a static;
    // the video thread is the only caller, so no start or stop overlaps it.
    sys::esp!(unsafe { sys::coludo_video_start(&config) })?;
    Ok(path)
}

/// Stop: flush, fsync, close; then, when `scan`, measure the card's free space and log it. Only the video thread
/// calls it.
///
/// The scan is slow (each volume's lock, and possibly megabytes of FAT), so it is skipped where something waits for
/// the card: at DONE, where the ring_saver catches up with the flight, and at arm, where the flight recording starts
/// next.
fn stop_recording(scan: bool) -> anyhow::Result<()> {
    // SAFETY: stops the running recorder (an error when none runs); the video thread is the only caller.
    sys::esp!(unsafe { sys::coludo_video_stop() })?;
    if scan {
        let volumes = sd::measure_free();
        log::info!("sd :: after the recording: {}", sd::describe(&volumes));
    }
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
    Parameter {
        name: "segment",
        kind: Kind::Integer { low: 0, high: 3600 },
        default: Some("300"),
        about: "on the pad, a new file at the first IDR this many seconds into a file; 0 keeps one file",
    },
];

/// How often the video thread looks at the flight word and the accel: the rod trigger's 8 looks are 2 s at this.
const LOOK_PERIOD: Duration = Duration::from_millis(250);

/// How long `video start` and `video stop` wait for the video thread's answer. A start (the first one opens the
/// camera) and a stop (one capture timeout, 1 s, the writer's flush of up to the 2 MB ring, and the free-space scan)
/// take a few seconds at most. The recorder's own waits are unbounded, so a call that never returns costs the
/// executor this long, well inside the hub's 30 s.
const ANSWER_DEADLINE: Duration = Duration::from_secs(10);

/// The orders that may wait for the video thread. The executor posts one at a time and waits for its answer, so a
/// second finds the mailbox full only behind an order whose asker gave up.
const MAILBOX_DEPTH: usize = 1;

/// The video thread's priority on core 0: low, beside the panic log's thread and under every ring task. The recorder's
/// own encode (10) and write (6) tasks keep their priorities: this thread only starts and stops them.
const PRIORITY: u8 = 2;

/// The video thread's stack (PSRAM): the recorder's start and stop calls, the FAT calls behind the directory and the
/// free-space scan, the log lines, and the counters' text after a stop.
const STACK_SIZE: usize = 8 << 10;

/// What the operator asks of the video thread.
enum Order {
    /// `video start`, with these settings.
    Start(Settings),
    /// `video stop`.
    Stop,
}

impl Order {
    /// The command's words, for the log.
    fn words(&self) -> &'static str {
        match self {
            Order::Start(_) => "video start",
            Order::Stop => "video stop",
        }
    }
}

/// One order posted to the video thread, and where its answer goes.
struct Posted {
    /// What is asked.
    order: Order,
    /// When its asker stops waiting: an order taken later is dropped, so nothing runs after the operator was told it
    /// did not.
    deadline: Instant,
    /// The answer, one CC line.
    answer: SyncSender<String>,
}

/// Starts the video thread (core 0, priority 2, PSRAM stack), the camera's one owner, and registers the camera's
/// console commands, which post to it. Call it when the board config declares a camera and the card mounts:
/// recording needs both. The thread's control block is internal RAM, so after the encoder's reservation
/// ([`reserve`]).
///
/// Args:
/// * `session` - this boot's session: the videos directory the files go to;
/// * `boot_id` - this boot's id, stamped into every frame;
/// * `camera` - the camera the board config declares;
/// * `flight` - the flight word the flight recording follows;
/// * `accel` - the fused `accel` quantity the rod trigger reads, None on a board with no accelerometer.
///
/// Errors: the thread would not spawn. Nothing is registered then, so `help` lists no `video`.
pub fn start(
    session: String,
    boot_id: u32,
    camera: Camera,
    flight: &'static FlightWord,
    accel: Option<&'static databoard::Parameter<[f32; 3]>>,
) -> anyhow::Result<()> {
    let (orders, posted) = sync_channel(MAILBOX_DEPTH);
    let owner = VideoThread {
        session,
        boot_id,
        camera,
        flight,
        accel,
        recording: None,
        trigger: Trigger::new(accel.is_some()),
    };
    ThreadSpawnConfiguration {
        name: Some(c"video"),
        stack_size: STACK_SIZE,
        priority: PRIORITY,
        pin_to_core: Some(Core::Core0),
        // stack in PSRAM (avionics-port.md "Internal RAM budget"): a slow core-0 thread that never calls
        // esp_mmu_map/unmap, the one runtime path that turns the cache off
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new()
        .stack_size(STACK_SIZE)
        .spawn(move || owner.run(posted));
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    register_commands(orders);
    Ok(())
}

/// The video thread: the camera's one owner, the recording it runs, and the flight word it follows.
struct VideoThread {
    /// This boot's session: the videos directory the files go to.
    session: String,
    /// This boot's id, stamped into every frame.
    boot_id: u32,
    /// The camera the board config declares.
    camera: Camera,
    /// The flight word it follows.
    flight: &'static FlightWord,
    /// The fused `accel` the rod trigger reads, None on a board with no accelerometer.
    accel: Option<&'static databoard::Parameter<[f32; 3]>>,
    /// The running recording: who started it, and its file.
    recording: Option<(Owner, String)>,
    /// The rod trigger: the arm episode's nose-up looks and whether it started.
    trigger: Trigger,
}

impl VideoThread {
    /// The thread, forever: a look at the flight word every [`LOOK_PERIOD`], and each order as it comes.
    fn run(mut self, orders: Receiver<Posted>) {
        let mut next_look = Instant::now();
        loop {
            if Instant::now() >= next_look {
                self.look();
                next_look = Instant::now() + LOOK_PERIOD;
            }
            let wait = next_look.saturating_duration_since(Instant::now());
            match orders.recv_timeout(wait) {
                Ok(posted) => self.serve(posted),
                Err(RecvTimeoutError::Timeout) => {}
                // the commands withdrawn: only the flight word is left to follow
                Err(RecvTimeoutError::Disconnected) => std::thread::sleep(wait),
            }
        }
    }

    /// Who started the running recording, None when nothing records.
    fn owner(&self) -> Option<Owner> {
        self.recording.as_ref().map(|(owner, _)| *owner)
    }

    /// One look at the flight word and the fused accel, and the duty they ask for (crates/sequencer `video::Trigger`).
    fn look(&mut self) {
        let now = self.flight.load();
        // SAFETY: a plain read of the monotonic timer.
        let now_us = unsafe { sys::esp_timer_get_time() } as u64;
        let accel = self
            .accel
            .and_then(|parameter| parameter.read(now_us))
            .map(|reading| reading.value);
        let duty = self.trigger.look(now, accel, self.owner());
        match duty {
            None => {}
            Some(Duty::StartFlight) => self.start_flight(),
            Some(Duty::StopFlight) => {
                if let Some((_, path)) = self.stop_running(true) {
                    log::info!("video :: disarmed: the flight recording {path} stopped");
                }
            }
            Some(Duty::StopLanded) => {
                if let Some((owner, path)) = self.stop_running(false) {
                    log::info!("video :: done: {} recording {path} stopped", whose(owner));
                }
            }
        }
    }

    /// Armed in SETTING and on the rod (or armed, with no accelerometer): the flight recording, after the operator's
    /// stops (logged). A start that fails is logged here, once: the next try is the next arm, and a dead camera never
    /// blocks one.
    fn start_flight(&mut self) {
        if let Some((_, path)) = self.stop_running(false) {
            log::info!("video :: on the rod: the operator's recording {path} stopped for the flight's");
        }
        match start_recording(&self.session, self.boot_id, self.camera, &FLIGHT, self.flight) {
            Ok(path) => {
                log::info!("video :: on the rod: the flight recording {path}");
                self.recording = Some((Owner::Flight, path));
            }
            Err(error) => log::error!(
                "video :: on the rod, but the flight recording did not start: {error} -- not retried before the next arm"
            ),
        }
    }

    /// Stop the running recording, with the free-space scan after it when `scan`.
    ///
    /// Returns: who ran it and its file; None when nothing records.
    fn stop_running(&mut self, scan: bool) -> Option<(Owner, String)> {
        let recording = self.recording.take()?;
        if let Err(error) = stop_recording(scan) {
            log::warn!("video :: stopping {}: {error}", recording.1);
        }
        Some(recording)
    }

    /// Serve one order, unless its asker has stopped waiting: then it is dropped, logged, and never run.
    fn serve(&mut self, posted: Posted) {
        let words = posted.order.words();
        if Instant::now() >= posted.deadline {
            log::warn!("video :: `{words}` waited past its deadline behind an earlier one: dropped, not run");
            return;
        }
        let answer = match posted.order {
            Order::Start(settings) => self.operator_start(&settings),
            Order::Stop => self.operator_stop(),
        };
        if let Err(SendError(answer)) = posted.answer.send(answer) {
            log::warn!("video :: `{words}` finished after its asker stopped waiting: {answer}");
        }
    }

    /// `video start`: the operator's recording, in SETTING (the gate is checked again here, since the order may have
    /// waited) while nothing records.
    fn operator_start(&mut self, settings: &Settings) -> String {
        if let Some(refusal) = sequencer::video::start_refused(self.flight.load(), self.owner()) {
            return refused(refusal);
        }
        match start_recording(&self.session, self.boot_id, self.camera, settings, self.flight) {
            Ok(path) => {
                let answer = cc_protocol::build("ok", &["video", &path], &[]);
                self.recording = Some((Owner::Operator, path));
                answer
            }
            Err(error) => failure(&error.to_string()),
        }
    }

    /// `video stop`: whatever records, on the ground; the free-space scan follows in SETTING only.
    fn operator_stop(&mut self) -> String {
        let flight = self.flight.load();
        if let Some(refusal) = sequencer::video::stop_refused(flight, self.owner()) {
            return refused(refusal);
        }
        match self.stop_running(flight.stage == Stage::Setting) {
            Some((Owner::Flight, path)) => {
                log::warn!(
                    "video :: `video stop` stopped the flight recording {path}: no flight video before the next arm"
                );
                counters("stopped")
            }
            Some((Owner::Operator, _)) => counters("stopped"),
            None => refused(Refusal::NotStarted),
        }
    }
}

/// Whose recording it is, for the log.
fn whose(owner: Owner) -> &'static str {
    match owner {
        Owner::Flight => "the flight",
        Owner::Operator => "the operator's",
    }
}

/// Post `order` to the video thread and wait for its answer, at most [`ANSWER_DEADLINE`]: the executor's side of
/// `video start` and `video stop`.
fn ask(orders: &SyncSender<Posted>, order: Order) -> String {
    let deadline = Instant::now() + ANSWER_DEADLINE;
    let (answer, answered) = sync_channel(1);
    let posted = Posted {
        order,
        deadline,
        answer,
    };
    if orders.try_send(posted).is_err() {
        // full: an order whose asker gave up still waits for a thread busy in the recorder
        return failure("the video thread is busy with an earlier request: try again");
    }
    answered
        .recv_timeout(deadline.saturating_duration_since(Instant::now()))
        .unwrap_or_else(|_| {
            failure(&format!(
                "the video thread did not answer within {} s (`video stats` shows whether it records)",
                ANSWER_DEADLINE.as_secs()
            ))
        })
}

/// A refusal's line: `err <code> <why>`.
fn refused(refusal: Refusal) -> String {
    cc_protocol::build("err", &[refusal.code(), &refusal.to_string()], &[])
}

/// A recorder failure's line: `err video <why>`.
fn failure(problem: &str) -> String {
    cc_protocol::build("err", &["video", problem], &[])
}

/// The camera's console commands (the Glider Command Language, crate::gcl): `video start` and `video stop` post to the
/// video thread through `orders` and wait for its answer; `video stats` reads the counters itself.
fn register_commands(orders: SyncSender<Posted>) {
    let stop_orders = orders.clone();
    gcl::register(vec![
        Command {
            owner: "camera",
            words: "video start",
            about: "record to /sdcard/<session>/videos/<start uptime ms>.h264, each frame stamped (tools/video_sei.py)",
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
                    segment_seconds: arguments.integer("segment") as u16,
                };
                ask(&orders, Order::Start(settings))
            }),
        },
        Command {
            owner: "camera",
            words: "video stop",
            about: "stop recording, the flight's included: flush, fsync and close the file; the counters stay \
                    readable. On the ground only: in setting, and at done",
            // open, so it runs at DONE too; the video thread refuses it in the air
            gate: Gate::Open,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[],
            run: Box::new(move |_| ask(&stop_orders, Order::Stop)),
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
        // the files it moved on to at the pad's cuts, and the pad's preview JPEGs (M7b)
        ("segments", counters.segments.to_string()),
        ("preview", counters.preview_frames.to_string()),
        ("preview_errors", counters.preview_errors.to_string()),
        ("preview_us_max", counters.preview_microseconds_max.to_string()),
    ];
    let named: Vec<(&str, &str)> = values.iter().map(|(key, value)| (*key, value.as_str())).collect();
    cc_protocol::build("ok", &["video", state], &named)
}
