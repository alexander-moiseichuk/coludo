//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device console on UART0 (the board's CH343P USB-serial port): a line reader, nothing more. Every line goes to
//! the CC executor, the one point of injection (commands/mod.rs), tagged as the console's, so the console runs
//! exactly the commands the hub runs -- the CC commands and every Glider Command Language command a component has
//! registered (crate::gcl; `help` lists them) -- under the same gates, plus the few that are console only (`wifi`:
//! its password must never cross the network).
//!
//! tools/avionics_console.py is the operator's end of it: history, completion from `help`, readable replies.

use std::io::{BufRead, Write};

use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use esp_idf_svc::io::vfs::BlockingStdIo;
use firmware::gcl::Origin;

use crate::commands::Executor;

/// The console thread's stack: it reads a line and prints a reply, nothing deeper.
const STACK_SIZE: usize = 8192;

/// Take over UART0 for blocking line reads and serve the console on its own thread, forever.
pub fn start(
    uart: impl Uart + 'static,
    transmit: impl OutputPin + 'static,
    receive: impl InputPin + 'static,
    executor: Executor,
) -> anyhow::Result<()> {
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
    let standard_io = BlockingStdIo::uart(driver)?;
    std::thread::Builder::new()
        .name("console".into())
        .stack_size(STACK_SIZE)
        .spawn(move || {
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
            }
        })?;
    Ok(())
}
