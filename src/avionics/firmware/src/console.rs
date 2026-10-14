//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device console on UART0 (the board's CH343P USB-serial port): a line reader, nothing more. Every line goes to
//! the CC executor, the one point of injection (commands/mod.rs), tagged as the console's, so the console runs
//! exactly the commands the hub runs -- the CC commands and every Glider Command Language command a component has
//! registered (crate::gcl; `help` lists them) -- under the same gates, plus the few that are console only (`wifi`:
//! its password must never cross the network).
//!
//! tools/avionics_console.py is the operator's end of it: history, completion from `help`, readable replies.
//!
//! The UART driver goes in first thing in main ([`install`]), the reader only once the executor runs
//! ([`Console::serve`]): without the driver IDF's console write polls the transmit FIFO, so the log printer
//! (crate::logging) spun on core 0 above main's priority and held the bring-up for every line it printed, ~8 ms
//! each at 115200 baud (the fast warm boot, operator 2026-10-11). With it a write waits on the transmit interrupt.

use std::io::{BufRead, Write};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use esp_idf_svc::io::vfs::BlockingStdIo;
use std::sync::atomic::{AtomicU32, Ordering};

use firmware::gcl::{self, Command, Gate, Kind, Origin, Parameter, Reach};

use crate::commands::Executor;
use firmware::threads::{self, Spec, Stack};

/// The console thread's stack: it reads a line and prints a reply, nothing deeper.
const STACK_SIZE: usize = 8192;

/// A rate `console baud` asked for, set once its answer has gone (0: none).
static PENDING_BAUD: AtomicU32 = AtomicU32::new(0);

/// The console's own command (crate::gcl): `console baud rate=<n>`, for big transfers over the console (the operator,
/// 2026-10-11: more reliable than Wi-Fi; tools/ota.py --console). The answer goes at the old rate, the rate changes
/// after it; the next boot is back at 115200 (the bootloader's and the ROM's rate).
pub fn register_commands() {
    gcl::register(vec![Command {
        owner: "console",
        words: "console baud",
        about: "the console's rate from after this answer until the next boot (115200 again), for big transfers",
        gate: Gate::Setting,
        reach: Reach::Console,
        parameters: &[Parameter {
            name: "rate",
            kind: Kind::Integer {
                low: 115_200,
                high: 3_000_000,
            },
            default: None,
            about: "baud, e.g. 2000000 (the CH343P takes up to 6 Mbaud)",
        }],
        bare_words: &[],
        run: Box::new(|arguments| {
            let rate = arguments.integer("rate") as u32;
            PENDING_BAUD.store(rate, Ordering::Release);
            cc_protocol::build("ok", &["console", "baud", &rate.to_string()], &[])
        }),
    }]);
}

/// UART0 with its driver in place, stdin and stdout through it, until [`Console::serve`] reads it.
pub struct Console(BlockingStdIo<'static, UartDriver<'static>>);

/// Take over UART0: its driver in place and the console's stdio through it, interrupt-driven from here.
///
/// Errors: the driver would not install (the console then stays IDF's polling one, and has no reader).
pub fn install(
    uart: impl Uart + 'static,
    transmit: impl OutputPin + 'static,
    receive: impl InputPin + 'static,
) -> anyhow::Result<Console> {
    // drain what the boot already printed: installing the driver mid-line cuts the line off
    // SAFETY: a polling wait on the console UART's transmit FIFO.
    unsafe { esp_idf_svc::sys::uart_wait_tx_idle_polling(0) };
    let driver = UartDriver::new(
        uart,
        transmit,
        receive,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &Default::default(),
    )?;
    Ok(Console(BlockingStdIo::uart(driver)?))
}

impl Console {
    /// Serve the console on its own thread (core 0, priority 5, beside the CC executor it feeds), forever.
    pub fn serve(self, executor: Executor) -> anyhow::Result<()> {
        let standard_io = self.0;
        // core 0 with the rest of the link: pthread's default leaves it unpinned, free to run on core 1, the flight
        // core
        // stack in PSRAM (avionics-port.md §4.7, internal RAM budget): a slow core-0 thread that never calls
        // esp_mmu_map/unmap, the one runtime path that turns the cache off
        threads::spawn(
            Spec {
                name: c"console",
                stack_size: STACK_SIZE,
                stack: Stack::Psram,
                priority: 5,
                core: Core::Core0,
            },
            move || {
                let _standard_io = standard_io; // stdin blocks only while this lives
                let mut line = String::new();
                loop {
                    line.clear();
                    if std::io::stdin().lock().read_line(&mut line).is_err() {
                        continue;
                    }
                    if let Some(reply) = executor.ask(line.trim(), Origin::Console) {
                        println!("{reply}");
                        let _ = std::io::stdout().flush();
                    }
                    // `console baud`: the new rate once its answer has left at the old one
                    let rate = PENDING_BAUD.swap(0, Ordering::AcqRel);
                    if rate != 0 {
                        // SAFETY: UART0's driver is installed (install); waits for the transmit FIFO, then sets the rate.
                        // the receive interrupt at 32 bytes in the 128-byte FIFO, not near its top: ~0.5 ms of slack at
                        // 2 Mbaud for an interrupt that comes late (a flash write's), where the default left microseconds
                        unsafe {
                            esp_idf_svc::sys::uart_wait_tx_done(0, 100);
                            esp_idf_svc::sys::uart_set_baudrate(0, rate);
                            esp_idf_svc::sys::uart_set_rx_full_threshold(0, 32);
                        }
                    }
                }
            },
        )?;
        Ok(())
    }
}
