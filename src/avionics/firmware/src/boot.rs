//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The bring-up's phases (doc/specs/avionics-port.md "Bring-up order"; doc/specs/avionics-refactor.md R4), in the
//! order `main` runs them, each documented with what it needs and what it gives:
//!
//! 1. [`storage`]: the boot count and the board config, which name this boot's records; the recorder opened before
//!    anything else logs, in flight when NVS holds a crumb in BOOSTING..LANDING; the last boot's panic filed;
//! 2. [`media`]: the card (one FAT volume, or two: the recordings and the videos), the camera encoder's internal block
//!    reserved before anything else splits it, the oops thread;
//! 3. [`ports`]: the I2C ports, each created by its owner thread on core 1 and lent to main;
//! 4. [`layout`]: the board's revision by the ports' scan, which rewrites the config and so names its id; the HITL
//!    mask; the ports at the rewritten config's clocks;
//! 5. [`recorder`]: the ring tasks (the writer, the eraser, the saver that copies the flash to the card) and the
//!    uart_saver;
//! 6. [`board`]: the board's identity and the provenance line;
//! 7. `devices::bring_up`: the devices in config order on the lent ports;
//! 8. [`recovery`]: the warm start's restore, which may commit a flight through the flight word's writer and rebase the
//!    barometers while main still holds both; the calibrations; each port back to its owner with its parts;
//! 9. [`flight`]: the fins, the control thread (which takes the writer), the supervisor, the sim-link, the GNSS;
//! 10. [`operator`]: the commands, the camera's thread, the CC executor, the UART console, the card's entry file;
//! 11. [`radio`]: the beacons, then Wi-Fi through the C6 and the CC hub, forever.

use std::sync::Arc;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::hal::temp_sensor::TempSensor;
use esp_idf_svc::hal::uart::{UART1, UART2};
use esp_idf_svc::nvs::{EspCustomNvsPartition, EspDefaultNvsPartition};
use firmware::buses::i2c::I2cPort;
use firmware::thermal::Thermometer;
use sequencer::FlightWriter;

use crate::{
    beacon, board, commands, config, control, devices, entry, fins, gnss, link, owner, panics_logger, recorder,
    sensors, sim_link, status, supervisor, warm,
};

/// Port 0 when the config defines no i2c:0: the WaveShare board's own wiring (the camera's SCCB and the codec are on
/// SDA 7 / SCL 8), at plan B's default clock.
const PORT0_STOCK: board_config::I2cSpecification = board_config::I2cSpecification {
    id: 0,
    sda: 7,
    scl: 8,
    clock_hz: 400_000,
};

/// The ports main lends the bring-up: each one's parked owner and the port.
pub type Lent = Vec<(owner::Parked, I2cPort)>;

/// Logs the internal RAM left after the boot phase `phase`: free, and the DMA-capable part of it. One line a phase,
/// so each feature's price in internal RAM shows in the boot log where it was paid (doc/specs/avionics-port.md §4.7).
pub fn checkpoint(phase: &str) {
    log::info!(
        "boot :: after {phase}: internal {} free, DMA-capable {} free",
        board::heap_free(esp_idf_svc::sys::MALLOC_CAP_INTERNAL),
        board::heap_free(esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_DMA)
    );
}

/// What [`storage`] gives the boot.
pub struct Storage {
    /// The NVS partition.
    pub partition: EspDefaultNvsPartition,
    /// This boot's number, None when NVS would not count it.
    pub boot_id: Option<i32>,
    /// The settings partition (the config store, the credentials, the ring's hint).
    pub settings: EspCustomNvsPartition,
    /// The config store.
    pub store: config::Store,
    /// The saved config's id: what `set-config board` answered for it.
    pub saved_config_id: String,
    /// This boot's session prefix.
    pub session: String,
    /// Whether a crumb in BOOSTING..LANDING says this is a reboot in flight.
    pub in_flight: bool,
}

/// The boot count, the board config and the recorder, needing only NVS (`partition`): every fin pin held low, the
/// recorder opened (every line and row kept from here, queued in PSRAM until the ring_writer runs), the config's
/// lines, and the last boot's panic filed before anything here can panic again.
///
/// Gives the boot's storage, and the board config as it loaded, which the layout rewrites before the run freezes it.
///
/// Errors: NVS would not count the boot, or the settings partition would not open.
pub fn storage(partition: EspDefaultNvsPartition) -> anyhow::Result<(Storage, board_config::Loaded)> {
    // the boot count and the board config first: they name this boot's records (plan B main: the boot id before
    // bring-up). Their own lines wait for the recorder below, so the capture has them too.
    let boot_id = board::count_boot(partition.clone())?;
    let settings = config::partition()?;
    let store = config::Store::open(settings.clone())?;
    let loaded = board_config::load(store.board().as_deref(), config::defaults());
    // every fin pin LOW from here: a floating one picks up its neighbours' pulses and the servos twitch (crate::fins)
    fins::hold_low(&loaded.document);
    // the saved config's id: what `set-config board` answered for it. The boot runs under the post-layout one below
    let saved_config_id = board_config::config_id(&loaded.document);
    let session = recorder::session_prefix(&loaded.document, boot_id);
    // a crumb in BOOSTING..LANDING: a reboot in flight, whose records go into flight blocks from the first one
    let in_flight = warm::peek(&partition);
    // storage first: from here on every log line and row is kept, queued in PSRAM until the ring_writer runs
    if let Err(error) = recorder::open(&loaded.document, boot_id, &session, in_flight) {
        log::error!("Recorder :: not recording this boot: {error}");
    }
    log::info!("config :: {saved_config_id} {}", loaded.source);
    for error in &loaded.errors {
        log::warn!("config :: {error}");
    }
    // the last boot's panic into the panic log, before anything here can panic again; then this boot's capture
    panics_logger::start(boot_id.unwrap_or(0) as u32, &session, &saved_config_id);
    let storage = Storage {
        partition,
        boot_id,
        settings,
        store,
        saved_config_id,
        session,
        in_flight,
    };
    checkpoint("storage");
    Ok((storage, loaded))
}

/// What [`media`] gives the boot.
pub struct Media {
    /// The die's thermometer, when it started.
    pub thermometer: Option<Thermometer>,
    /// Whether the card mounted.
    pub sd_mounted: bool,
    /// The card by partition 1's volume serial, when it mounted.
    pub card: Option<recorder::Card>,
    /// The camera the config declares.
    pub camera: Option<firmware::video::Camera>,
}

/// The die's thermometer (`temp_sensor`), the card, and the camera of `document`: the encoder's ~138 KB internal block
/// reserved before the ring threads' stacks and buffers split it (no camera or no card: no recording, no
/// reservation); then the oops thread, before the bring-up records a general call as an oops.
pub fn media(document: &serde_json::Value, temp_sensor: TempSensor<'static>) -> Media {
    let thermometer = Thermometer::new(temp_sensor)
        .inspect_err(|error| log::warn!("thermometer :: {error}"))
        .ok();
    let sd_mounted = firmware::sd::mount();
    // the card by partition 1's volume serial: the ring_saver's position hint is trusted only for the card it was
    // saved to
    let card = sd_mounted.then(|| recorder::Card {
        serial: firmware::sd::volume_serial(),
    });
    if let (Some(card), Some(layout)) = (card, firmware::sd::layout()) {
        log_card(card, layout);
    } else {
        log::warn!("sd :: no usable microSD -- running without it");
    }
    let camera = firmware::video::Camera::from_config(document);
    match camera {
        Some(camera) => log::info!("video :: camera ov5647, oscillator {} Hz", camera.clock_hz),
        // loud with a card: a config made from a template without the entry would fly without video
        None if sd_mounted => {
            log::warn!("video :: no camera in the board config (an 'ov5647' component): no video this boot")
        }
        None => log::info!("video :: no camera in the board config: no video"),
    }
    if sd_mounted
        && camera.is_some()
        && let Err(error) = firmware::video::reserve()
    {
        log::warn!("video :: encoder not reserved, `video start` will try again: {error}");
    }
    // the oops thread before the bring-up, which records a general call as an oops (its control block is internal,
    // so after the encoder's reservation)
    if let Err(error) = panics_logger::start_oops() {
        log::error!("panic :: the oops thread did not start, oopses are dropped: {error}");
    }
    checkpoint("media");
    Media {
        thermometer,
        sd_mounted,
        card,
        camera,
    }
}

/// The I2C ports of `document`, each created by its owner thread on core 1 and lent here: port 0 always (the camera's
/// SCCB is wired to it), port 1 when the config defines it. A port that will not come up is logged and left out.
pub fn ports(document: &serde_json::Value) -> Lent {
    let mut lent = Vec::new();
    for id in 0..2u8 {
        let Some(bus) = board_config::i2c_bus(document, id).or((id == 0).then_some(PORT0_STOCK)) else {
            continue;
        };
        match owner::spawn(id, bus.sda, bus.scl, bus.clock_hz) {
            Ok(loan) => lent.push(loan),
            Err(error) => log::error!("i2c:{id} :: no bus: {error}"),
        }
    }
    checkpoint("ports");
    lent
}

/// Which board this is, by the scan of the `lent` ports, before anything reads `document` for its devices (plan B
/// main.py:91-104): the config is rewritten in place, and from here on it is the one this boot runs. Then the HITL
/// mask, and the ports at the rewritten config's clocks. Gives the layout's verdict.
pub fn layout(document: &mut serde_json::Value, lent: &mut Lent) -> layout::Resolved {
    // a bus spec the view cannot read (a pin past 255) votes nothing, as plan B's Pin() raised on it, even though
    // port 0 runs on the stock pins then
    let readable = [0, 1].map(|bus| board_config::i2c_bus(document, bus).is_some());
    let resolved = layout::resolve(document, |bus, _| {
        let Some((_, port)) = lent
            .iter_mut()
            .find(|(_, port)| port.id() == bus && readable[usize::from(bus)])
        else {
            return Err(());
        };
        port.scan()
            .map_err(|error| log::warn!("layout :: i2c:{bus} scan failed ({error}): no votes from this bus"))
    });
    let undecided = matches!(resolved, layout::Resolved::Undecided { .. });
    for line in resolved.log_lines() {
        if undecided {
            log::warn!("{line}");
        } else {
            log::info!("{line}");
        }
    }
    // HITL: the sensors the simulator replaces, off again after the layout re-enabled what the revision fits (plan B
    // tools/hitl_run.py:47); before the config id, so the id is the config flown
    let masked = board_config::mask_simulated(document);
    if !masked.is_empty() {
        log::info!("hitl :: simulated, so masked: {}", masked.join(", "));
    }
    // the ports at the rewritten config's clocks
    for (_, port) in lent.iter_mut() {
        let id = port.id();
        let clock_hz = board_config::i2c_bus(document, id).map_or(PORT0_STOCK.clock_hz, |bus| bus.clock_hz);
        if let Err(error) = port.configure(clock_hz) {
            log::warn!("i2c:{id} :: not at the config's {clock_hz} Hz: {error}");
        }
    }
    checkpoint("layout");
    resolved
}

/// The recorder's threads for `document` under `config_id`: the uart_saver (M19: the wire lines to a receiver on
/// `uart1`, before the saver that feeds it starts), then the ring_writer, which scans the `log` partition in its own
/// thread and starts the ring_eraser and, with a `card`, the ring_saver -- all after the encoder's reservation. Boot
/// goes on, and the producers queue meanwhile.
pub fn recorder(
    document: &serde_json::Value,
    uart1: UART1<'static>,
    settings: EspCustomNvsPartition,
    card: Option<recorder::Card>,
    config_id: &str,
) {
    let recorder_uart = board_config::directory(document)
        .iter()
        .find(|entry| entry.implementation == "recorder")
        .filter(|entry| entry.bus == Some("uart"))
        .and_then(|entry| entry.bus_id.as_deref()?.parse::<u8>().ok())
        .and_then(|id| board_config::uart_transmitter(document, id).map(|link| (id, link)));
    match recorder_uart {
        Some((1, (tx, baud))) => {
            if let Err(error) = recorder::start_uart(uart1, tx, baud) {
                log::error!("ring :: the uart_saver did not start, no line leaves on its UART: {error}");
            }
        }
        Some((id, _)) => log::warn!("ring :: the recorder's uart:{id} is not UART1, the one free: no uart_saver"),
        None => {}
    }
    if let Err(error) = recorder::start(settings, card, config_id) {
        log::error!("Recorder :: the ring_writer did not start: {error}");
    }
    checkpoint("recorder");
}

/// The board of `storage` running `loaded` under `config_id`, with the media's thermometer and card, and the
/// provenance line (plan B main): the build and the config this capture comes from, stamped into the capture itself;
/// the saved config's id last, the one `set-config board` answered.
pub fn board(
    storage: &Storage,
    loaded: &board_config::Loaded,
    config_id: &str,
    thermometer: Option<Thermometer>,
    sd_mounted: bool,
) -> Arc<board::Board> {
    let board = Arc::new(board::Board::new(
        board_config::board_id(&loaded.document),
        storage.boot_id,
        storage.session.clone(),
        thermometer,
        sd_mounted,
    ));
    log::info!(
        "main :: boot: board {} | firmware {} | config {config_id} {} | reset_cause {} | boot {} | session {} | \
         saved_config {}",
        board.id,
        board::FIRMWARE_VERSION,
        loaded.source,
        board::reset_reason(),
        board
            .boot_id
            .map_or_else(|| "None".to_string(), |boot_id| boot_id.to_string()),
        board.session,
        storage.saved_config_id,
    );
    checkpoint("board");
    board
}

/// What [`recovery`] gives the boot.
pub struct Recovered {
    /// The flight word's one writer, for the control thread.
    pub writer: FlightWriter<'static>,
    /// The mission (the launch payload), for the executor.
    pub mission: mission::Mission,
    /// The warm start's identity, when this boot resumed a flight.
    pub seed: Option<warmstart::Identity>,
}

/// The warm start, while the parts and the flight word's one writer are still main's and nothing else runs: a reset
/// in flight resumes the crumb's stage, its barometers rebased to the pad and the mission's launch point and zone
/// restored (doc/specs/avionics-port.md "Bring-up order"; crate::warm). Then the parts' calibrations from NVS, as plan
/// B's setups restored them, and each `lent` port back to its owner with the parts of `devices` that came up on it.
pub fn recovery(
    storage: &Storage,
    document: &serde_json::Value,
    devices: &mut devices::Devices,
    lent: Lent,
) -> Recovered {
    checkpoint("devices");
    let (table, topology) = (devices.table, devices.topology);
    let members = &mut std::mem::take(&mut devices.members);
    // HITL simulates no separation switch (doc/specs/sim-link.md): its glide counts as separated, as plan B's
    // harness passed separated=True by construction (tools/hitl_run.py `_simulated_reboot`), or no warm start in a
    // HITL glide could pass the gate
    let separated = devices
        .separation
        .as_ref()
        .is_some_and(|member| member.part.separated())
        || devices.hitl.is_some();
    let mut writer = status::FLIGHT
        .writer()
        .expect("main takes the flight word's writer, once");
    let range = board_config::max_range_m(document).unwrap_or(mission::DEFAULT_MAX_RANGE_M);
    let mut mission = mission::Mission::load(storage.store.launch().as_deref(), range);
    let seed = warm::restore(
        &storage.partition,
        document,
        separated,
        &mut mission,
        members,
        &mut writer,
        storage.in_flight,
    );
    // the zone and the launch point the checkpoint freezes at BOOSTING entry; the executor republishes them from here
    warm::publish_mission(&mission);
    // the pitot's tare, the magnetometer's hard/soft-iron calibration and the BNO055's profile, as plan B's setups
    // restored them
    let mut lent = lent;
    sensors::restore_calibrations(&storage.partition, members, &mut lent);
    for (parked, port) in lent {
        let id = port.id();
        let parts = std::mem::take(&mut members[usize::from(id)]);
        if let Err(error) = owner::start(parked, port, parts, topology, table) {
            log::error!("i2c:{id} :: {error}");
        }
    }
    checkpoint("recovery");
    Recovered { writer, mission, seed }
}

/// The flight's threads: the fins (each on its LEDC channel, the control thread's; a timer or channel that will not
/// configure crashes every fin, so `arm` refuses; a cold boot centres them `fins.concurrency` at a time, a warm start
/// does not), the control thread on core 1 (which wakes the owners, steps the duties and takes the flight word's
/// writer), the supervisor with the checkpoint, the watchdog and the pad drift, the HITL sim-link and the GNSS on
/// their own core-0 threads. A thread that will not start crashes its entries.
#[expect(
    clippy::too_many_arguments,
    reason = "the bring-up's parts, each handed to its thread once"
)]
pub fn flight(
    document: &'static serde_json::Value,
    devices: devices::Devices,
    storage: &Storage,
    board: Arc<board::Board>,
    writer: FlightWriter<'static>,
    seed: Option<warmstart::Identity>,
    uart2: UART2<'static>,
    topology: &'static devices::Topology,
    table: &'static controller::Table<'static>,
) {
    // the checkpoint's pad latch stops at half the altitude launch's rise: a launch the accel trigger missed stays in
    // SETTING through the altitude launch's dwell, and must not latch its pad up the climb
    let pad_ceiling_m = devices
        .sequencer
        .as_ref()
        .map(|member| member.settings.launch_alt_m / 2.0);
    let schedule_hz = board_config::schedule_hz(document);
    let centre = seed
        .is_none()
        .then(|| document["fins"]["concurrency"].as_u64().unwrap_or(3) as usize);
    let fins = open_fins(document, devices.servos, table);
    if let Err(error) = control::start(
        schedule_hz,
        devices.spi_bus,
        devices.spi,
        devices.separation,
        devices.sequencer,
        devices.attitude,
        fins,
        centre,
        devices.flight,
        topology,
        table,
        writer,
    ) {
        log::error!("control :: the control thread did not start: {error}");
    }
    // the checkpoint keeps no crumb without its NVS namespace: its entry is crashed then, so `arm` refuses. A warm
    // start's identity seeds it, so the restored flight's crumbs carry the pad and the launch it launched with
    let keeping = devices.checkpoint.and_then(|member| {
        warm::Keeping::open(member, &storage.partition, topology, document, pad_ceiling_m, seed)
            .inspect_err(|error| {
                log::error!("{} :: {error}", member.name);
                devices::crash(table, [Some(member.name)], error);
            })
            .ok()
    });
    // the stall limit is at least two control ticks
    let tick_ms = 1000.0 / schedule_hz.max(1) as f32;
    let calibration = devices
        .gnss_calib
        .map(|member| gnss::Calibration::new(member, topology));
    if let Err(error) = supervisor::start(board, table, keeping, devices.watchdog, calibration, tick_ms) {
        log::error!("supervisor :: did not start: {error}");
    }
    // the HITL sim-link on its own core-0 thread (M12): a thread that will not spawn crashes its entry, so `arm` refuses
    if let Some(member) = devices.hitl {
        let name = member.name;
        if let Err(error) = sim_link::start(member, topology) {
            log::error!("{name} :: did not start: {error}");
            devices::crash(table, [Some(name)], &error.to_string());
        }
    }
    // the GNSS on UART2, on its own core-0 thread (M11): a UART that would not open crashes its entry, so `arm` refuses
    if let Some(member) = devices.gnss {
        let name = member.name;
        if let Err(error) = gnss::start(member, uart2, topology) {
            log::error!("{name} :: did not start: {error}");
            devices::crash(table, [Some(name)], &error.to_string());
        }
    }
    checkpoint("flight");
}

/// The fin servos of `document` on their LEDC channels, None without any; a timer or channel that will not configure
/// crashes every fin in `table` (so `arm` refuses). The probe sweeps the flown deflection range, the mixer's
/// `limit_deg`, not the servos' own (the operator, 2026-10-11).
fn open_fins(
    document: &serde_json::Value,
    servos: Vec<devices::ServoMember>,
    table: &'static controller::Table<'static>,
) -> Option<fins::Fins> {
    if servos.is_empty() {
        return None;
    }
    let names: Vec<&'static str> = servos.iter().map(|member| member.name).collect();
    let probe_limit = document["fins"]["mixer"]["limit_deg"]
        .as_i64()
        .and_then(|limit| i32::try_from(limit).ok());
    fins::open(servos, probe_limit)
        .inspect_err(|error| {
            log::error!("fins :: the PWM did not start: {error}");
            devices::crash(table, names.iter().map(|name| Some(*name)), &error.to_string());
        })
        .ok()
}

/// Logs the mounted card: partition 1's serial (the name the session-start record keeps), how its volumes are
/// mounted (one partition or two, doc/specs/avionics-port.md "SD and flash file layout"), the videos partition's
/// serial, and each volume's size and free space; and, loudly, a videos partition that would not mount.
fn log_card(card: recorder::Card, layout: firmware::sd::Layout) {
    let start = std::time::Instant::now();
    let volumes = firmware::sd::measure_free();
    let videos = match layout {
        firmware::sd::Layout::TwoPartitions => format!(
            " (videos {})",
            firmware::sd::videos_serial().map_or_else(|| "unknown".to_string(), firmware::sd::serial_name)
        ),
        firmware::sd::Layout::OnePartition => " (videos in each session's directory)".to_string(),
        firmware::sd::Layout::VideosFailed => String::new(),
    };
    log::info!(
        "sd :: mounted card {}, {}{videos}: {} (measured in {} ms)",
        card.name(),
        layout.name(),
        firmware::sd::describe(&volumes),
        start.elapsed().as_millis()
    );
    if layout == firmware::sd::Layout::VideosFailed {
        log::error!(
            "sd :: partition 2 is not mounted (it would not mount, or the table is unsound: sd_card's line above says \
             which): no video this boot"
        );
    }
}

/// The operator's side: each component's commands (the Glider Command Language: each registers its own as it
/// activates, so `help` lists exactly what this board runs), the card's commands and the camera's one owner thread
/// when the card mounted, the CC executor over `context`, the UART `console` (it drains the UART before taking it
/// over, so the lines above survive), the banner, and the card's entry file (M21) once bring-up is done. Gives the
/// executor.
///
/// Errors: the executor or the console would not start.
pub fn operator(
    context: commands::Context,
    credentials: &config::Credentials,
    console: Option<crate::console::Console>,
    camera: Option<firmware::video::Camera>,
    sd_mounted: bool,
) -> anyhow::Result<commands::Executor> {
    let board = context.board.clone();
    let accel = context.topology.vector("accel");
    firmware::gcl::set_cc_names(commands::names());
    crate::register_system_commands();
    owner::register_commands();
    control::register_commands();
    panics_logger::register_commands();
    crate::diagnostics::register_commands();
    credentials.register_commands();
    link::register_commands();
    crate::ota::register_commands();
    crate::console::register_commands();
    if sd_mounted {
        firmware::sd::register_commands(board.session.clone());
        entry::register_commands();
        // the camera's one owner: the flight recording by the flight word, and `video start`/`stop` through its
        // mailbox
        if let Some(camera) = camera
            && let Err(error) = firmware::video::start(
                board.session.clone(),
                board.boot_id.unwrap_or(0) as u32,
                camera,
                &status::FLIGHT,
                accel,
            )
        {
            log::error!("video :: the video thread did not start, no video this boot: {error}");
        }
    }
    let executor = commands::Executor::start(context)?;
    if let Some(console) = console {
        console.serve(executor.clone())?;
    }
    log::info!("{}", board.banner());
    if sd_mounted && let Err(error) = entry::start(executor.clone()) {
        log::error!("entry :: the runner did not start, the entry file is not run: {error}");
    }
    checkpoint("operator");
    Ok(executor)
}

/// The radio, forever: the recovery beacons (M11b, M20: the battery's capacity from the INA226 entry of `document`,
/// when the config gives one; the remote recorder's report to beacon.csv every few seconds, its radio off until
/// DONE), then Wi-Fi through the C6 and the CC hub (crate::link).
#[expect(clippy::too_many_arguments, reason = "the radio's parts, each taken once")]
pub fn radio(
    document: &serde_json::Value,
    board: &board::Board,
    topology: &'static devices::Topology,
    modem: Modem<'static>,
    event_loop: EspSystemEventLoop,
    partition: EspDefaultNvsPartition,
    credentials: config::Credentials,
    executor: &commands::Executor,
) -> ! {
    let battery_mah = board_config::directory(document)
        .iter()
        .find(|entry| entry.implementation == "ina226")
        .and_then(|entry| entry.fields.get("battery_mah"))
        .and_then(serde_json::Value::as_f64)
        .map(|capacity| capacity as f32);
    let beacon = beacon::Beacon::new(&board.id, topology, battery_mah);
    if let Err(error) = beacon::spawn_feed(beacon.clone()) {
        log::error!("beacon :: the feed did not start, the remote beacon gets no report: {error}");
    }
    link::run(
        modem,
        event_loop,
        partition,
        board_config::wifi(document),
        credentials,
        executor,
        beacon,
    )
}
