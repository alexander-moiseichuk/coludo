//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Avionics bring-up (doc/specs/avionics-port.md "Bring-up order"): the clock, the console's UART, then the boot's
//! phases (crate::boot: storage, media, ports, layout, the recorder's threads, the board, the devices, the warm
//! start's recovery, the flight's threads, the operator's side) in order, and the radio forever. A panic anywhere
//! prints its backtrace and reboots (sdkconfig: PANIC_PRINT_REBOOT).

mod beacon;
mod board;
mod boot;
mod clock;
mod commands;
mod config;
mod console;
mod control;
mod devices;
mod diagnostics;
mod entry;
mod files;
mod fins;
mod flight;
mod gnss;
mod link;
mod logging;
mod ota;
mod owner;
mod panics_logger;
mod recorder;
mod sensors;
mod sim_link;
mod status;
mod supervisor;
mod warm;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

/// The bring-up, phase by phase (crate::boot), then the radio forever.
fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    logging::install();
    clock::start();
    let peripherals = Peripherals::take()?;
    // the console's UART driver first: without it every console write polls the FIFO (crate::console)
    let console = console::install(peripherals.uart0, peripherals.pins.gpio37, peripherals.pins.gpio38)
        .inspect_err(|error| log::warn!("console :: no UART driver, the console polls and reads nothing: {error}"))
        .ok();
    let event_loop = EspSystemEventLoop::take()?;
    let (storage, mut loaded) = boot::storage(EspDefaultNvsPartition::take()?)?;
    let media = boot::media(&loaded.document, peripherals.temp_sensor);
    let mut lent = boot::ports(&loaded.document);
    let resolved = boot::layout(&mut loaded.document, &mut lent);
    // the id this boot runs under: the rewritten config's (plan B main.py:120-124), in its records from here on
    let config_id = board_config::config_id(&loaded.document);
    panics_logger::name_config(&config_id);
    boot::recorder(
        &loaded.document,
        peripherals.uart1,
        storage.settings.clone(),
        media.card,
        &config_id,
    );
    let board = boot::board(&storage, &loaded, &config_id, media.thermometer, media.sd_mounted);
    // the config is frozen for the run from here; the devices come up in its order on the lent ports, with nothing
    // else running (plan B's Controller.setup), and each port goes back to its owner with the parts that came up
    let loaded: &'static board_config::Loaded = Box::leak(Box::new(loaded));
    let mut devices = devices::bring_up(&loaded.document, &mut lent, recorder::get().is_some());
    let (table, topology) = (devices.table, devices.topology);
    let field = devices.field.take();
    let recovered = boot::recovery(&storage, &loaded.document, &mut devices, lent);
    boot::flight(
        &loaded.document,
        devices,
        &storage,
        board.clone(),
        recovered.writer,
        recovered.seed,
        peripherals.uart2,
        topology,
        table,
    );
    let credentials = config::Credentials::new(storage.settings.clone());
    let executor = boot::operator(
        commands::Context {
            board: board.clone(),
            store: storage.store,
            nvs: storage.partition.clone(),
            mission: recovered.mission,
            config: loaded,
            config_id,
            layout: resolved.label(),
            devices: table,
            topology,
            field: field.map(commands::FieldAgent::new),
        },
        &credentials,
        console,
        media.camera,
        media.sd_mounted,
    )?;
    boot::radio(
        &loaded.document,
        &board,
        topology,
        peripherals.modem,
        event_loop,
        storage.partition,
        credentials,
        &executor,
    )
}

/// The kinds of a deliberate panic (`panic kind=... confirm`).
const PANIC_KINDS: &[&str] = &["rust", "fault", "abort", "oops", "twdt", "stall", "hang"];

/// The system's own console command (the Glider Command Language): `panic kind=... confirm`, which proves a panic
/// reboots the board, warm-starting a flight in the air (crate::warm), and lands in the panic log
/// (crate::panics_logger); and that the supervisor's watchdog resets a board whose supervisor stops feeding the task
/// watchdog (`twdt`) or whose control thread stalls (`stall`). Open in every stage, for the console alone: it is the
/// one hands-free reset in GLIDING and at DONE (`reboot` runs only in SETTING), and never one the hub can send. `twdt`
/// and `stall` are refused when nothing would reset the board: a stalled control thread could not even be disarmed.
pub fn register_system_commands() {
    firmware::gcl::register(vec![firmware::gcl::Command {
        owner: "system",
        words: "panic",
        about: "panic on purpose and see it in `panic log`: rust (a Rust panic), fault (a store to address 0), abort \
                (C's abort()) reboot the board; oops records a non-fatal panic and goes on; twdt stops the \
                supervisor feeding the task watchdog, which panics the board wdt_timeout_ms later (refused unless the \
                task watchdog is armed); stall stops the control thread, and the supervisor's stall check restarts \
                the board (refused unless it controls a flight and the check would restart it); hang arms the \
                restart's RTC watchdog and stops this core, which the watchdog must reset within 1 s",
        gate: firmware::gcl::Gate::Open,
        reach: firmware::gcl::Reach::Console,
        parameters: &[firmware::gcl::Parameter {
            name: "kind",
            kind: firmware::gcl::Kind::Choice(PANIC_KINDS),
            default: Some("rust"),
            about: "rust, fault, abort, twdt, stall, hang (each resets) or oops (does not)",
        }],
        bare_words: &[firmware::gcl::CONFIRM],
        run: Box::new(|arguments| match arguments.text("kind") {
            "fault" => {
                // SAFETY: deliberately not: a store to address 0 is the access fault this kind exists to raise.
                unsafe { core::arch::asm!("sw zero, 0(zero)") };
                unreachable!("a store to address 0 faults")
            }
            // SAFETY: C's abort() never returns; it is the point of this kind.
            "abort" => unsafe { esp_idf_svc::sys::abort() },
            // SAFETY: never returns: the RTC watchdog the restart arms resets the chip (components/safe_restart).
            "hang" => unsafe { esp_idf_svc::sys::safe_restart_hang() },
            "oops" => {
                panics_logger::oops(
                    "test-oops",
                    "a deliberate oops from a console command: nothing is broken",
                );
                cc_protocol::build("ok", &["panic", "oops"], &[])
            }
            "twdt" => match supervisor::starve() {
                Ok(()) => cc_protocol::build("ok", &["panic", "twdt"], &[]),
                Err(refusal) => cc_protocol::build("err", &["refused", refusal], &[]),
            },
            "stall" => match supervisor::stall_refused() {
                None => {
                    control::stall();
                    cc_protocol::build("ok", &["panic", "stall"], &[])
                }
                Some(refusal) => cc_protocol::build("err", &["refused", refusal], &[]),
            },
            _ => panic!("deliberate panic from a console command"),
        }),
    }]);
}
